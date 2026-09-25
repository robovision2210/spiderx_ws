"""Metrics and pass/fail evaluation for the M2 SIMULATION-ONLY posture-hold test.

Pure Python (no ROS graph): the runtime tool collects samples and this module turns them into
metrics, an outcome and a report. That keeps the correctness logic deterministic and unit-tested.

Body pose source and frame
--------------------------
Gazebo ground truth: /world/spiderx_fortress/pose/info (SceneBroadcaster, gz.msgs.Pose_V), bridged
to ROS as /spiderx/sim/world_poses (tf2_msgs/msg/TFMessage, NOT /tf) by
spiderx_bringup/launch/fortress_posture_hold.launch.py. The entry whose child_frame_id is the model
name ("spiderx") is the model pose in the Gazebo WORLD frame. The model frame is the canonical link
dummy_link, and dummy_joint has no offset, so it is the base_link pose.
  * height = z of the base_link origin above the ground plane (world z = 0);
  * roll / pitch / yaw = ZYX Euler angles of base_link in the world frame. base_link is not REP-103
    (+y front, +x right), so roll (about base x) is nose up/down and pitch (about base y) is side tilt.
The bridge leaves per-pose stamps at 0, so every sample is stamped with the node's /clock time.

This is simulation ground truth for a test, not odometry.
"""

import math

OUTCOME_VERIFIED = 'Simulation posture hold verified.'
OUTCOME_NOT_VERIFIED = 'Simulation posture hold not verified.'
MODEL_NAME = 'spiderx'
POSE_TOPIC = '/spiderx/sim/world_poses'
BODY_FRAME_DESCRIPTION = (
    'Gazebo model "spiderx" pose in the world frame of world "spiderx_fortress" '
    '(= dummy_link = base_link; dummy_joint has no offset). Height = base_link origin z above the '
    'ground plane (world z = 0). roll/pitch/yaw = ZYX Euler of base_link in world; roll is about '
    'base_link x (robot right: nose up/down), pitch about base_link y (robot front: side tilt).')
# URDF/CAD-derived (docs/SPIDERX_URDF_AUDIT.md section 3): at q = 0 every foot bottoms out
# 0.0545 m below base_link. Used only for an informational geometric indicator.
CAD_FOOT_PLANE_BELOW_BASE_M = 0.0545
FOOT_CONTACT = {
    'status': 'unavailable',
    'reason': ('The SpiderX Fortress model and world contain no contact sensor or Contact system, '
               'and no contact topic is published. M2 does not add one and does not fake contact '
               'values. Foot contact was NOT measured.'),
}
DISCLAIMER = [
    'Simulation-only posture hold.',
    'Not dynamic balance control.',
    'Not walking or gait control.',
    'Not inverse kinematics.',
    'Not hardware validation.',
    'Not real-servo torque validation.',
    'Not battery/current validation.',
    'Not proof of real-world stability.',
]
LIMITATIONS = [
    'Placeholder actuator effort/torque values (URDF effort=100 N m, velocity=100 rad/s; '
    'exporter placeholders, audit K4).',
    'Simulation contact model (Gazebo Fortress / DART default contact).',
    'Friction assumptions (mu1 = mu2 = 0.2, fusion2urdf template default, not measured).',
    'Damping assumptions (no joint damping or friction in the URDF).',
    'Link mass/inertia assumptions (CAD values at steel density, total 7.29 kg, not weighed).',
    'Gazebo physics timestep/solver assumptions (1 ms step, DART, gz_ros2_control position '
    'commands applied as velocity with proportional gain 0.1).',
    'No real actuator, battery, electronics, structural-flexibility, or IMU validation.',
]


class ObservationError(RuntimeError):
    """A required observation (topic, transform, controller state, clock) is missing or bad."""


def quat_to_rpy(x, y, z, w):
    """ZYX Euler angles (roll about x, pitch about y, yaw about z) from a unit quaternion."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ObservationError('zero-length orientation quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def tilt_from_quat(x, y, z, w):
    """Angle between the body z axis and world z (rad), independent of Euler conventions."""
    n2 = x * x + y * y + z * z + w * w
    zz = 1.0 - 2.0 * (x * x + y * y) / n2      # R[2][2]
    return math.acos(max(-1.0, min(1.0, zz)))


def extract_model_pose(transforms, model=MODEL_NAME):
    """(x, y, z, qx, qy, qz, qw) of `model` from a list of TransformStamped-like objects."""
    for t in transforms:
        if t.child_frame_id == model:
            tr, q = t.transform.translation, t.transform.rotation
            return (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
    raise ObservationError(f'model "{model}" not found in {POSE_TOPIC} (is the Gazebo pose bridge '
                           'running? ros2 launch spiderx_bringup fortress_posture_hold.launch.py)')


def joint_positions(names, positions, required):
    """{joint: position} from a JointState, validating that every required joint is present."""
    if len(names) != len(positions):
        raise ObservationError(f'malformed /joint_states: {len(names)} names but '
                               f'{len(positions)} positions')
    pos = dict(zip(names, positions))
    missing = [j for j in required if j not in pos]
    if missing:
        raise ObservationError(f'/joint_states is missing joints {missing}')
    bad = [j for j in required if not math.isfinite(pos[j])]
    if bad:
        raise ObservationError(f'/joint_states has non-finite positions for {bad}')
    return {j: float(pos[j]) for j in required}


def joint_error_stats(samples, targets):
    """Per-joint max |error|, overall max and RMS over a list of {joint: position} samples."""
    if not samples:
        return {'per_joint_max_abs_error_rad': {}, 'max_abs_error_rad': None,
                'rms_error_rad': None}
    per_joint = {j: max(abs(s[j] - t) for s in samples) for j, t in targets.items()}
    sq = [(s[j] - t) ** 2 for s in samples for j, t in targets.items()]
    return {
        'per_joint_max_abs_error_rad': per_joint,
        'max_abs_error_rad': max(per_joint.values()),
        'rms_error_rad': math.sqrt(sum(sq) / len(sq)),
    }


def body_stats(pose_samples):
    """Height / orientation statistics over [(t, (x, y, z, qx, qy, qz, qw)), ...]."""
    if not pose_samples:
        return None
    zs = [p[2] for _, p in pose_samples]
    rpys = [quat_to_rpy(*p[3:]) for _, p in pose_samples]
    tilts = [tilt_from_quat(*p[3:]) for _, p in pose_samples]
    first, last = pose_samples[0][1], pose_samples[-1][1]
    final_rpy = rpys[-1]
    return {
        'frame': BODY_FRAME_DESCRIPTION,
        'samples': len(pose_samples),
        'height_m': {'min': min(zs), 'max': max(zs), 'range': max(zs) - min(zs), 'final': zs[-1]},
        'roll_rad': {'max_abs': max(abs(r[0]) for r in rpys), 'final': final_rpy[0]},
        'pitch_rad': {'max_abs': max(abs(r[1]) for r in rpys), 'final': final_rpy[1]},
        'yaw_rad_final': final_rpy[2],
        'yaw_drift_rad': _wrap(rpys[-1][2] - rpys[0][2]),
        'tilt_rad_max': max(tilts),
        'xy_drift_m': math.hypot(last[0] - first[0], last[1] - first[1]),
        'final_position_m': list(last[:3]),
        'final_orientation_xyzw': list(last[3:]),
        'geometric_indicator_height_minus_cad_foot_plane_m': zs[-1] - CAD_FOOT_PLANE_BELOW_BASE_M,
    }


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def evaluate(posture, run):
    """Apply every M2 criterion to the collected run data.

    posture: output of posture_config.validate_posture().
    run: dict with keys
      controller_checks  [(sim_time_s, phase, {controller: state})]
      joint_state_publishers  [int] (counts at start and end)
      action  {'server_available': bool, 'accepted': bool, 'error_code': int|None}
      peak_joint_speed_rad_s  float
      hold  {'start_s': float, 'end_s': float}
      joint_samples  [{joint: position}]  (hold window)
      pose_samples  [(sim_time_s, pose tuple)]  (hold window)
      observation_errors  [str]
    Returns (passed, failures, metrics).
    """
    th = posture['thresholds']
    failures = list(run.get('observation_errors', []))

    for t, phase, states in run.get('controller_checks', []):
        for c in ('joint_state_broadcaster', 'leg_trajectory_controller'):
            if states.get(c) != 'active':
                failures.append(f'controller {c} is {states.get(c, "missing")!r} at {phase} '
                                f'(t = {t:.3f} s), expected active')
    if not run.get('controller_checks'):
        failures.append('controller states were never read')

    counts = run.get('joint_state_publishers')
    if not counts or any(n != 1 for n in counts):
        failures.append(f'/joint_states publisher counts {counts} (start, end), expected exactly 1')

    act = run.get('action', {})
    if not act.get('server_available'):
        failures.append('trajectory action server /leg_trajectory_controller/'
                        'follow_joint_trajectory unavailable')
    elif not act.get('accepted'):
        failures.append('trajectory goal was not accepted')
    elif act.get('error_code') is None:
        failures.append('trajectory result timed out (no result received)')
    elif act.get('error_code') != 0:
        failures.append(f'trajectory result error_code {act["error_code"]} (0 = SUCCESSFUL)')

    speed = run.get('peak_joint_speed_rad_s')
    if speed is not None and speed > posture['max_joint_velocity_rad_s'] + 1e-9:
        failures.append(f'peak joint speed {speed:.3f} rad/s exceeds '
                        f'{posture["max_joint_velocity_rad_s"]} rad/s')

    hold = run.get('hold') or {}
    hold_s = None
    if hold.get('start_s') is not None and hold.get('end_s') is not None:
        hold_s = hold['end_s'] - hold['start_s']
        if hold_s + 1e-6 < posture['hold_duration_s']:
            failures.append(f'hold lasted {hold_s:.3f} s of simulation time, '
                            f'required {posture["hold_duration_s"]} s')
    elif not failures:
        failures.append('hold window was not recorded')

    joints = joint_error_stats(run.get('joint_samples', []), posture['targets'])
    n_js = len(run.get('joint_samples', []))
    if n_js < th['min_joint_state_samples']:
        failures.append(f'only {n_js} joint-state samples during the hold, '
                        f'required {th["min_joint_state_samples"]}')
    if joints['max_abs_error_rad'] is not None:
        if joints['max_abs_error_rad'] > th['max_joint_error_rad']:
            worst = max(joints['per_joint_max_abs_error_rad'].items(), key=lambda kv: kv[1])
            failures.append(f'max joint error {joints["max_abs_error_rad"]:.4f} rad '
                            f'({worst[0]}) exceeds {th["max_joint_error_rad"]} rad')
        if joints['rms_error_rad'] > th['rms_joint_error_rad']:
            failures.append(f'RMS joint error {joints["rms_error_rad"]:.4f} rad exceeds '
                            f'{th["rms_joint_error_rad"]} rad')

    body = body_stats(run.get('pose_samples', []))
    n_pose = body['samples'] if body else 0
    if n_pose < th['min_pose_samples']:
        failures.append(f'only {n_pose} body-pose samples during the hold, '
                        f'required {th["min_pose_samples"]}')
    if body:
        if body['height_m']['min'] < th['min_body_height_m']:
            failures.append(f'body height fell to {body["height_m"]["min"]:.4f} m, below '
                            f'{th["min_body_height_m"]} m')
        if body['height_m']['range'] > th['max_body_height_range_m']:
            failures.append(f'body height varied by {body["height_m"]["range"]:.4f} m during the '
                            f'hold, above {th["max_body_height_range_m"]} m')
        if body['roll_rad']['max_abs'] > th['max_abs_roll_rad']:
            failures.append(f'|roll| reached {body["roll_rad"]["max_abs"]:.4f} rad, above '
                            f'{th["max_abs_roll_rad"]} rad')
        if body['pitch_rad']['max_abs'] > th['max_abs_pitch_rad']:
            failures.append(f'|pitch| reached {body["pitch_rad"]["max_abs"]:.4f} rad, above '
                            f'{th["max_abs_pitch_rad"]} rad')

    metrics = {'hold_duration_s': hold_s, 'joint_state_samples': n_js, 'joints': joints,
               'body': body}
    return (not failures), failures, metrics


def build_report(posture, run, passed, failures, metrics, extra=None):
    """Machine-readable report (JSON-serialisable dict)."""
    report = {
        'simulation_only': True,
        'disclaimer': DISCLAIMER,
        'outcome': OUTCOME_VERIFIED if passed else OUTCOME_NOT_VERIFIED,
        'passed': passed,
        'failures': failures,
        'posture': posture['name'],
        'config_version': posture['config_version'],
        'config_date': posture['date'],
        'commanded_positions_rad': posture['targets'],
        'command_duration_s': posture['command_duration_s'],
        'required_hold_duration_s': posture['hold_duration_s'],
        'thresholds': posture['thresholds'],
        'validation_margin_rad': posture['margin'],
        'start_positions_rad': run.get('start_positions'),
        'final_positions_rad': run.get('final_positions'),
        'final_abs_error_rad': (
            {j: abs(run['final_positions'][j] - t) for j, t in posture['targets'].items()}
            if run.get('final_positions') else None),
        'peak_joint_speed_rad_s': run.get('peak_joint_speed_rad_s'),
        'joints_moved_more_than_0_01_rad': run.get('joints_moved'),
        'action': run.get('action'),
        'controller_checks': [
            {'sim_time_s': t, 'phase': ph, 'states': st}
            for t, ph, st in run.get('controller_checks', [])],
        'joint_state_publishers': run.get('joint_state_publishers'),
        'sim_times_s': run.get('times'),
        'hold': run.get('hold'),
        'metrics': metrics,
        'foot_contact': FOOT_CONTACT,
        'limitations': LIMITATIONS,
    }
    if extra:
        report.update(extra)
    return report
