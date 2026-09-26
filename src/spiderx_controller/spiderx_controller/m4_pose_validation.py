"""M4 SIMULATION-ONLY runtime validation of four-leg static poses via IK in Gazebo Fortress.

    ros2 launch spiderx_bringup fortress_posture_hold.launch.py     # terminal 1 (M1 + ground truth)
    ros2 run spiderx_controller m4_pose_validation                  # terminal 2

For each pose of config/m4_pose_targets.yaml (neutral_stance, crouch_10mm, lift_lf_15mm):
  validated 12-joint IK command (m4_pose_targets, atomic) -> ONE trajectory goal -> settle ->
  FK vs TF and FK vs Gazebo (all four feet) -> 5 s hold sampling body height/roll/pitch ->
  FK checks again -> observed foot tips vs targets -> return to neutral_stance.
Negative-test poses are refused by the loader and never commanded.

Static poses only. Not walking, not a gait, not balance control, not hardware validation.

The decision/report logic at the top of this module is pure Python (unit-tested without ROS);
ROS imports happen only inside main()/the node factory.
Exit: 0 = all three outcome lines verified; 1 = "M4 all-leg kinematics validation not verified.";
      2 = refused (invalid pose configuration), nothing was sent.
"""

import argparse
import json
import os
import sys
import time

from spiderx_controller import kinematics_validation as kv
from spiderx_controller import leg_kinematics as lk
from spiderx_controller.posture_metrics import POSE_TOPIC, body_stats

FK_VERIFIED = 'All-leg forward kinematics verified for the current URDF/TF/Gazebo model.'
IK_VERIFIED = ('All-leg inverse kinematics verified for documented, joint-safe, simulation-only '
               'static poses.')
HOLD_VALIDATED = ('Static multi-leg pose hold via IK validated in Gazebo (no walking, no gait, '
                  'no hardware).')
NOT_VERIFIED = 'M4 all-leg kinematics validation not verified.'
POSE_ORDER = ('neutral_stance', 'crouch_10mm', 'lift_lf_15mm')
NEUTRAL = 'neutral_stance'
CONTROLLERS = ('joint_state_broadcaster', 'leg_trajectory_controller')
MODEL = 'spiderx'

# Runtime thresholds (SIMULATION-ONLY). Sources are pinned by unit tests:
TOLERANCES = {                         # = m3_kinematics_targets.yaml tolerances (M3, verified)
    'ik_position_tol_m': 1e-6,
    'fk_vs_tf_position_m': 1e-6,
    'fk_vs_tf_orientation_rad': 1e-6,
    'fk_vs_gazebo_position_m': 1e-3,
    'fk_vs_gazebo_orientation_rad': 5e-3,
    'target_reached_position_m': 2e-3,
}
MAX_ABS_ROLL_RAD = 0.10                # = m2_simulation_postures.yaml max_abs_roll_rad
MAX_ABS_PITCH_RAD = 0.10               # = m2_simulation_postures.yaml max_abs_pitch_rad
MIN_HOLD_SAMPLES = 20                  # = m2_simulation_postures.yaml min_pose_samples
HEIGHT_TOL_M = 0.003                   # M4 plan: body height vs geometric expectation, +/- 3 mm
HOLD_S = 5.0                           # M4 plan: hold, simulation seconds
SETTLE_S = 1.0                         # = m3 motion.settle_s
MIN_DURATION_S = 3.0                   # = m3 motion.min_duration_s
SPEED_FACTOR = 2.0                     # = m3 motion.speed_factor
START_POSE_TOL_RAD = 0.05              # all 12 joints near CAD neutral before any motion (M3)
RETURN_TOL_RAD = 0.01                  # returned to neutral (M3)
DISCLAIMER = [
    'Four-leg static pose hold via IK in Gazebo Fortress - simulation only.',
    'Not walking, not a gait, not balance control, not locomotion, not hardware validation.',
]
LIMITATIONS = [
    'Gazebo, TF and this code share one URDF: agreement shows model consistency, not physical truth.',
    'Placeholder actuators (100 N m, 100 rad/s): joints are near-rigid and track almost perfectly.',
    'Foot tips are derived mesh points, not measured contact points; contact is not measured.',
    'Body height/tilt come from Gazebo ground truth; contact, friction (mu 0.2), damping (none), '
    'masses (steel density) and physics (DART, 1 ms) are simulation assumptions.',
    'Three static poses with <= 15 mm foot offsets; no walking, gait, disturbance or hardware test.',
]


def default_output():
    return os.path.join(os.getcwd(), 'log', 'm4_all_leg_ik', 'latest_report.json')


def duration_for(q_from, q_to, v_max):
    dq = max(abs(a - b) for a, b in zip(q_from, q_to))
    return max(MIN_DURATION_S, SPEED_FACTOR * dq / v_max)


# ------------------------------------------------------------ pure decision logic
def fk_check_all(geoms, q12, tf_obs, gz_obs, label):
    """FK vs TF and Gazebo for all four legs. tf_obs/gz_obs: {leg: (pos, quat)} (missing = None)."""
    per_leg = {}
    for i, leg in enumerate(lk.ALL_LEGS):
        per_leg[leg] = kv.fk_check(geoms[leg], q12[3 * i:3 * i + 3], (tf_obs or {}).get(leg),
                                   (gz_obs or {}).get(leg), TOLERANCES, f'{label} {leg}')
    failures = [f'{leg} {f}' for leg, r in per_leg.items() for f in r['failures']]
    return {'label': label, 'legs': per_leg, 'passed': not failures,
            'failures': [f'{label}: {f}' for f in failures]}


def tip_errors(geoms, targets, gz_obs):
    """Observed foot tip (Gazebo link pose composed with the derived tip) vs target, per leg."""
    out = {}
    for leg in lk.ALL_LEGS:
        obs = (gz_obs or {}).get(leg)
        if obs is None:
            out[leg] = None
            continue
        R = lk.matrix_from_quaternion(*obs[1])
        tip = tuple(obs[0][i] + sum(R[i][k] * geoms[leg].tip_local[k] for k in range(3))
                    for i in range(3))
        out[leg] = dict(kv.position_error(tip, targets[leg]), observed_tip_m=list(tip))
    return out


def evaluate_pose(name, rec, expected_height_m):
    """Apply the M4 criteria to one pose record. Returns (fk_ok, ik_ok, hold_ok, failures)."""
    fk_fail = [f for c in rec.get('fk_checks', []) for f in c['failures']]
    if len(rec.get('fk_checks', [])) < 2:
        fk_fail.append(f'{name}: FK checks missing (expected at hold start and end)')
    ik_fail = []
    act = rec.get('action') or {}
    if act.get('error_code') != 0:
        ik_fail.append(f'{name}: trajectory result {act}')
    tips = rec.get('tip_errors') or {}
    for leg in lk.ALL_LEGS:
        e = tips.get(leg)
        if e is None:
            ik_fail.append(f'{name}: {leg} foot tip not observed')
        elif e['euclidean'] > TOLERANCES['target_reached_position_m']:
            ik_fail.append(f'{name}: {leg} tip {e["euclidean"] * 1000:.3f} mm from target '
                           f'> {TOLERANCES["target_reached_position_m"] * 1000:.1f} mm')
    if not rec.get('returned_to_neutral'):
        ik_fail.append(f'{name}: did not return to {NEUTRAL}')
    hold_fail = []
    body = rec.get('body')
    if not body or body['samples'] < MIN_HOLD_SAMPLES:
        hold_fail.append(f'{name}: only {body["samples"] if body else 0} body samples during the '
                         f'hold (< {MIN_HOLD_SAMPLES})')
    else:
        if body['roll_rad']['max_abs'] > MAX_ABS_ROLL_RAD:
            hold_fail.append(f'{name}: |roll| {body["roll_rad"]["max_abs"]:.4f} rad > '
                             f'{MAX_ABS_ROLL_RAD}')
        if body['pitch_rad']['max_abs'] > MAX_ABS_PITCH_RAD:
            hold_fail.append(f'{name}: |pitch| {body["pitch_rad"]["max_abs"]:.4f} rad > '
                             f'{MAX_ABS_PITCH_RAD}')
        dev = max(abs(body['height_m']['min'] - expected_height_m),
                  abs(body['height_m']['max'] - expected_height_m))
        rec['height_deviation_max_m'] = dev
        if dev > HEIGHT_TOL_M:
            hold_fail.append(f'{name}: body height {body["height_m"]["min"]:.4f}..'
                             f'{body["height_m"]["max"]:.4f} m deviates {dev * 1000:.2f} mm from '
                             f'the geometric expectation {expected_height_m:.4f} m '
                             f'(> {HEIGHT_TOL_M * 1000:.1f} mm)')
    hold_s = rec.get('hold_s')
    if hold_s is None or hold_s + 1e-6 < HOLD_S:
        hold_fail.append(f'{name}: hold lasted {hold_s} s of simulation time (< {HOLD_S})')
    return not fk_fail, not ik_fail, not hold_fail, fk_fail + ik_fail + hold_fail


def outcome(run):
    """(fk_ok, ik_ok, hold_ok, outcome_lines, failures) over the whole run."""
    failures = list(run.get('precondition_failures', []))
    poses = run.get('poses', {})
    fk_ok = ik_ok = hold_ok = not failures and set(poses) == set(POSE_ORDER)
    if not failures and set(poses) != set(POSE_ORDER):
        failures.append(f'poses tested {sorted(poses)} != {sorted(POSE_ORDER)}')
    for c in run.get('start_fk', []):
        fk_ok = fk_ok and c['passed']
        failures += c['failures']
    if not run.get('start_fk'):
        fk_ok = False
    for name, rec in poses.items():
        f, i, h, fails = evaluate_pose(name, rec, rec['expected_body_height_m'])
        rec.update(fk_passed=f, ik_passed=i, hold_passed=h, passed=f and i and h, failures=fails)
        fk_ok, ik_ok, hold_ok = fk_ok and f, ik_ok and i, hold_ok and h
        failures += fails
    neg = run.get('negative_poses', {})
    if not neg or any(n.get('commanded') or n.get('accepted') for n in neg.values()):
        ik_ok = False
        failures.append(f'negative poses not all refused without commanding: {neg}')
    if not run.get('final_return_ok'):
        ik_ok = hold_ok = False
        failures.append(f'robot did not end at {NEUTRAL}')
    ik_ok = ik_ok and fk_ok
    hold_ok = hold_ok and ik_ok
    lines = [line for ok, line in ((fk_ok, FK_VERIFIED), (ik_ok, IK_VERIFIED),
                                   (hold_ok, HOLD_VALIDATED)) if ok]
    if not (fk_ok and ik_ok and hold_ok):
        lines.append(NOT_VERIFIED)
    return fk_ok, ik_ok, hold_ok, lines, failures


def build_report(geoms, cfg, run):
    fk_ok, ik_ok, hold_ok, lines, failures = outcome(run)
    return {
        'simulation_only': True, 'disclaimer': DISCLAIMER, 'outcome': lines,
        'fk_verified': fk_ok, 'ik_verified': ik_ok, 'hold_validated': hold_ok,
        'passed': fk_ok and ik_ok and hold_ok, 'failures': failures, 'frame': lk.FRAME,
        'legs': list(lk.ALL_LEGS), 'joint_names': lk.all_joint_names(geoms),
        'config_version': cfg['config_version'], 'config_date': cfg['date'],
        'tolerances': TOLERANCES,
        'hold_criteria': {'hold_s': HOLD_S, 'max_abs_roll_rad': MAX_ABS_ROLL_RAD,
                          'max_abs_pitch_rad': MAX_ABS_PITCH_RAD, 'height_tol_m': HEIGHT_TOL_M,
                          'min_samples': MIN_HOLD_SAMPLES},
        'reference_tips_m': {leg: list(geoms[leg].tip0) for leg in lk.ALL_LEGS},
        'controller_checks': run.get('controller_checks', []),
        'joint_state_publishers': run.get('joint_state_publishers'),
        'start_positions_rad': run.get('start_positions'),
        'start_fk': run.get('start_fk', []), 'poses': run.get('poses', {}),
        'negative_poses': run.get('negative_poses', {}), 'goals_sent': run.get('goals_sent'),
        'final_return_ok': run.get('final_return_ok'), 'limitations': LIMITATIONS,
    }


# ------------------------------------------------------------ ROS runtime
def _make_node(geoms):
    import rclpy
    from rclpy.time import Time
    import tf2_ros
    from tf2_msgs.msg import TFMessage

    from spiderx_controller.trajectory_client import JointTestNode

    feet = {leg: geoms[leg].foot_link for leg in lk.ALL_LEGS}

    class M4Node(JointTestNode):
        def __init__(self):
            super().__init__('spiderx_m4_pose_validation')
            self.js_msg, self.gz_links, self.gz_model = None, {}, None
            self.recording, self.model_samples = False, []
            self.goals_sent, self.neutral_cmd = 0, None
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_subscription(TFMessage, POSE_TOPIC, self._on_pose, 10)

        def now_s(self):
            return self.get_clock().now().nanoseconds / 1e9

        def _on_state(self, msg):
            super()._on_state(msg)
            self.js_msg = msg

        def _on_pose(self, msg):
            for t in msg.transforms:
                tr, q = t.transform.translation, t.transform.rotation
                pose = ((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
                if t.child_frame_id == MODEL:
                    self.gz_model = pose
                    if self.recording:
                        self.model_samples.append((self.now_s(), pose[0] + pose[1]))
                for leg, link in feet.items():
                    if t.child_frame_id == link:
                        self.gz_links[leg] = pose

        def wait_for(self, predicate, timeout):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            return predicate()

        def send_positions(self, joint_names, targets, duration_s, timeout=None):
            self.goals_sent += 1
            return super().send_positions(joint_names, targets, duration_s, timeout)

        def controller_check(self, phase):
            states = self.controller_states()
            st = {n: s for n, _, s in states} if states is not None else {}
            return {'phase': phase, 'states': {c: st.get(c, 'missing') for c in CONTROLLERS}}

        def tf_ready(self):
            return all(self.tf_buffer.can_transform('base_link', link, Time())
                       for link in feet.values())

        def observe(self, names, settle_s=0.3):
            """(q12, TF {leg: pose}, Gazebo {leg: pose}) from fresh messages; TF at the JS stamp."""
            self.spin_for(settle_s)
            self.js_msg, self.gz_links = None, {}
            if not self.wait_for(lambda: self.js_msg is not None and len(self.gz_links) == 4,
                                 20.0):
                return None, {}, dict(self.gz_links)
            msg = self.js_msg
            pos = dict(zip(msg.name, msg.position))
            if any(j not in pos for j in names):
                return None, {}, dict(self.gz_links)
            gz = dict(self.gz_links)
            stamp, tf_obs = Time.from_msg(msg.header.stamp), {}
            for leg, link in feet.items():
                for _ in range(200):
                    try:
                        t = self.tf_buffer.lookup_transform('base_link', link, stamp)
                        tr, rq = t.transform.translation, t.transform.rotation
                        tf_obs[leg] = ((tr.x, tr.y, tr.z), (rq.x, rq.y, rq.z, rq.w))
                        break
                    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                            tf2_ros.ConnectivityException):
                        rclpy.spin_once(self, timeout_sec=0.05)
            return [pos[j] for j in names], tf_obs, gz

    return rclpy, M4Node


def _run(rclpy, node, geoms, plan, names, v_max, args):
    from spiderx_controller.m4_pose_targets import pose_command
    run = {'precondition_failures': [], 'controller_checks': [], 'start_fk': [], 'poses': {},
           'negative_poses': {}, 'final_return_ok': False}
    pre = run['precondition_failures']
    if not node.wait_for(lambda: node.get_clock().now().nanoseconds > 0, args.ready_timeout):
        pre.append('no /clock received: is the simulation running?')
        return run
    cc = node.controller_check('start')
    run['controller_checks'].append(cc)
    if any(s != 'active' for s in cc['states'].values()):
        pre.append(f'M1 controllers not all active: {cc["states"]}')
        return run
    if not node.wait_for(lambda: node.positions is not None and len(node.gz_links) == 4
                         and node.gz_model is not None, args.ready_timeout):
        pre.append(f'no /joint_states or no Gazebo poses of all four feet and the model on '
                   f'{POSE_TOPIC} (start fortress_posture_hold.launch.py)')
        return run
    if not node.wait_for(node.tf_ready, args.ready_timeout):
        pre.append('TF base_link -> all four foot links never became available')
        return run
    run['joint_state_publishers'] = [node.count_publishers('/joint_states')]
    if run['joint_state_publishers'][0] != 1:
        pre.append(f'/joint_states has {run["joint_state_publishers"][0]} publishers, expected 1')
        return run
    q0, tf_obs, gz_obs = node.observe(names, settle_s=0.5)
    if q0 is None:
        pre.append('/joint_states does not contain all 12 joints')
        return run
    run['start_positions'] = dict(zip(names, q0))
    far = {j: v for j, v in run['start_positions'].items() if abs(v) > START_POSE_TOL_RAD}
    if far:
        pre.append(f'start pose is not CAD neutral (|q| > {START_POSE_TOL_RAD} rad: {far}); run '
                   'ros2 run spiderx_controller test_neutral_pose.py first')
        return run
    run['start_fk'].append(fk_check_all(geoms, q0, tf_obs, gz_obs, 'start'))
    if not run['start_fk'][-1]['passed']:
        return run                                  # no motion unless FK agrees first
    neutral = pose_command(plan, NEUTRAL)
    node.neutral_cmd = neutral

    for name, rec in plan['negative'].items():      # refused by the loader; never commanded
        run['negative_poses'][name] = {'accepted': rec['ok'], 'failing_legs': rec['failing_legs'],
                                       'commanded': False}

    for name in POSE_ORDER:
        prec = plan['poses'][name]
        cmd = pose_command(plan, name)
        rec = {'targets_m': prec['targets_m'], 'command_rad': cmd,
               'support_legs': prec['support_legs'],
               'expected_body_height_m': prec['geometric_expected_body_height_m']['value'],
               'expected_body_height_note': prec['geometric_expected_body_height_m']['note'],
               'fk_checks': []}
        run['poses'][name] = rec
        now = node.fresh_positions()
        q_now = [now[j] for j in names] if now else neutral
        dur = duration_for(q_now, cmd, v_max)
        print(f'  {name}: one 12-joint trajectory, {dur:.1f} s ...')
        accepted, code = node.send_positions(names, cmd, dur)
        rec['action'] = {'accepted': accepted, 'error_code': code, 'duration_s': dur}
        if code != 0:                                # stop, but leave the robot at neutral
            node.send_positions(names, neutral, max(MIN_DURATION_S, dur))
            break
        node.spin_for(SETTLE_S)
        q, tf_obs, gz_obs = node.observe(names)
        if q is not None:
            rec['fk_checks'].append(fk_check_all(geoms, q, tf_obs, gz_obs, f'{name} hold start'))
        node.model_samples, node.recording = [], True
        t0 = node.now_s()
        wall_end = time.monotonic() + 20.0 * HOLD_S + 60.0
        while node.now_s() < t0 + HOLD_S and time.monotonic() < wall_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.recording = False
        rec['hold_s'] = node.now_s() - t0
        rec['body'] = body_stats(node.model_samples)
        q, tf_obs, gz_obs = node.observe(names)
        if q is not None:
            rec['fk_checks'].append(fk_check_all(geoms, q, tf_obs, gz_obs, f'{name} hold end'))
            rec['q_measured_rad'] = q
            rec['max_joint_tracking_error_rad'] = max(abs(a - b) for a, b in zip(q, cmd))
        rec['tip_errors'] = tip_errors(geoms, {leg: tuple(v) for leg, v in
                                               rec['targets_m'].items()}, gz_obs)
        back = node.send_positions(names, neutral, duration_for(cmd, neutral, v_max))
        node.spin_for(SETTLE_S)
        after = node.fresh_positions()
        rec['returned_to_neutral'] = (back[1] == 0 and after is not None and max(
            abs(after[j] - v) for j, v in zip(names, neutral)) <= RETURN_TOL_RAD)
    final = node.fresh_positions()
    run['final_return_ok'] = final is not None and max(
        abs(final[j] - v) for j, v in zip(names, neutral)) <= RETURN_TOL_RAD
    run['controller_checks'].append(node.controller_check('end'))
    if any(s != 'active' for s in run['controller_checks'][-1]['states'].values()):
        pre.append(f'controllers not active at the end: {run["controller_checks"][-1]["states"]}')
    run['joint_state_publishers'].append(node.count_publishers('/joint_states'))
    if run['joint_state_publishers'][-1] != 1:
        pre.append(f'/joint_states publishers at the end: {run["joint_state_publishers"][-1]}')
    run['goals_sent'] = node.goals_sent
    return run


def print_summary(report):
    print('-' * 78)
    print('SpiderX M4 all-leg static pose hold via IK - SIMULATION ONLY (Gazebo Fortress)')
    for c in report['controller_checks']:
        print(f'  controllers @ {c["phase"]}: {c["states"]}')
    print(f'/joint_states publishers (start, end): {report["joint_state_publishers"]}')
    for c in report['start_fk']:
        print(f'FK check [start]: {"PASS" if c["passed"] else "FAIL"}')
    for name, rec in report['poses'].items():
        tf_err = max((r['vs_tf']['link_origin_error_m']['euclidean']
                      for c in rec['fk_checks'] for r in c['legs'].values()
                      if 'link_origin_error_m' in r['vs_tf']), default=None)
        gz_err = max((r['vs_gazebo']['link_origin_error_m']['euclidean']
                      for c in rec['fk_checks'] for r in c['legs'].values()
                      if 'link_origin_error_m' in r['vs_gazebo']), default=None)
        tips = [e['euclidean'] for e in (rec.get('tip_errors') or {}).values() if e]
        b = rec.get('body') or {}
        print(f'Pose [{name}]: action {rec.get("action", {}).get("error_code")}, '
              f'FK vs TF max {tf_err if tf_err is None else f"{tf_err:.1e}"} m, '
              f'vs Gazebo max {gz_err if gz_err is None else f"{gz_err:.1e}"} m, '
              f'tip error max {max(tips) * 1000 if tips else float("nan"):.3f} mm')
        if b:
            print(f'    hold {rec.get("hold_s", 0):.2f} s, {b["samples"]} samples: height '
                  f'{b["height_m"]["min"]:.4f}..{b["height_m"]["max"]:.4f} m (expected '
                  f'{rec["expected_body_height_m"]:.4f} +/- {HEIGHT_TOL_M}), |roll| max '
                  f'{b["roll_rad"]["max_abs"]:.4f}, |pitch| max {b["pitch_rad"]["max_abs"]:.4f} '
                  f'rad; returned {rec.get("returned_to_neutral")} -> '
                  f'{"PASS" if rec.get("passed") else "FAIL"}')
    for name, n in report['negative_poses'].items():
        print(f'Negative [{name}]: refused by the loader ({n["failing_legs"]}), commanded '
              f'{n["commanded"]}')
    for f in report['failures']:
        print(f'FAILED: {f}')
    for line in report['outcome']:
        print(line)
    print('Static poses in simulation only. Not walking, gait, balance or hardware validation.')


def main(argv=None):
    p = argparse.ArgumentParser(description='SpiderX M4 simulation-only static pose validation')
    p.add_argument('--config', default=None,
                   help='pose YAML (default: installed config/m4_pose_targets.yaml)')
    p.add_argument('--output', default=None,
                   help='report JSON (default: ./log/m4_all_leg_ik/latest_report.json)')
    p.add_argument('--ready-timeout', type=float, default=60.0,
                   help='wall-clock wait for clock/controllers/topics/TF [s] (default 60)')
    p.add_argument('--check-only', action='store_true',
                   help='validate the pose config and exit (no ROS, no motion)')
    from rclpy.utilities import remove_ros_args      # importing rclpy does not start ROS
    args = p.parse_args(remove_ros_args(sys.argv if argv is None else argv)[1:])

    from spiderx_controller.m4_pose_targets import PoseTargetError, load_and_evaluate
    print('SIMULATION ONLY - four-leg static poses via IK. Not walking, balance or hardware.')
    try:
        cfg, geoms, errors, plan = load_and_evaluate(args.config)
    except PoseTargetError as e:
        print(f'REFUSED: invalid M4 pose configuration: {e}')
        print('Nothing was sent.')
        return 2
    if errors:
        for e in errors:
            print(f'REFUSED: {e}')
        print('Nothing was sent.')
        return 2
    print(f'Poses {list(plan["poses"])} validated (12 joints each, URDF limits -/+ '
          f'{cfg["margin"]} rad); negative poses {list(plan["negative"])} refused.')
    if args.check_only:
        return 0

    import yaml
    from spiderx_controller.m4_pose_targets import default_config_dir
    with open(os.path.join(default_config_dir(), 'spiderx_legs.yaml')) as f:
        v_max = float(yaml.safe_load(f)['motion_constraints']['max_joint_velocity_rad_s'])
    rclpy, M4Node = _make_node(geoms)
    names = lk.all_joint_names(geoms)
    rclpy.init()
    node = M4Node()
    try:
        data = _run(rclpy, node, geoms, plan, names, v_max, args)
    except KeyboardInterrupt:
        print('Interrupted: cancelling the active goal and returning to neutral_stance ...')
        node.cancel_active_goal()
        if node.neutral_cmd is not None:
            node.send_positions(names, node.neutral_cmd, 4.0)
        raise
    except Exception as e:           # report the failure instead of losing it
        data = {'precondition_failures': [f'runtime error: {type(e).__name__}: {e}']}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report = build_report(geoms, cfg, data)
    out = os.path.abspath(args.output or default_output())
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        json.dump(report, f, indent=2)
    print_summary(report)
    print(f'Report: {out}')
    return 0 if report['passed'] else 1
