#!/usr/bin/env python3
"""M2 SIMULATION-ONLY posture-hold test for SpiderX in Gazebo Fortress.

    ros2 launch spiderx_bringup fortress_posture_hold.launch.py          # terminal 1
    ros2 run spiderx_controller run_posture_hold_test.py                 # terminal 2

Commands the existing CAD neutral pose (config/m2_simulation_postures.yaml, the single source of
truth) as ONE slow 12-joint trajectory to the M1 leg_trajectory_controller, then records joint
tracking, controller states and the Gazebo ground-truth body pose for the hold duration of
SIMULATION time, and applies the documented thresholds.

Simulation-only posture hold. Not dynamic balance control. Not walking or gait control. Not
inverse kinematics. Not hardware validation. Not real-servo torque validation. Not battery/current
validation. Not proof of real-world stability. It never talks to hardware.

Exit code: 0 = "Simulation posture hold verified."
           1 = "Simulation posture hold not verified." (report written with the reasons)
           2 = refused: the posture configuration is invalid; nothing was sent.
"""

import argparse
import json
import os
import sys
import time

import rclpy
from tf2_msgs.msg import TFMessage

from spiderx_controller import posture_metrics as pm
from spiderx_controller.posture_config import (
    DEFAULT_POSTURE, PostureConfigError, load_posture, peak_joint_speed)
from spiderx_controller.trajectory_client import JointTestNode

CONTROLLERS = ('joint_state_broadcaster', 'leg_trajectory_controller')


def default_output():
    ros_home = os.environ.get('ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros'))
    return os.path.join(ros_home, 'spiderx_m2', 'posture_hold_report.json')


class PostureHoldNode(JointTestNode):
    """JointTestNode + validated joint samples + Gazebo ground-truth body pose samples."""

    def __init__(self, joints):
        super().__init__('spiderx_posture_hold_test')
        self.joints = joints
        self.recording = False
        self.joint_samples, self.pose_samples, self.observation_errors = [], [], []
        self.latest_pose = None
        self.create_subscription(TFMessage, pm.POSE_TOPIC, self._on_pose, 10)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_state(self, msg):
        super()._on_state(msg)
        if not self.recording:
            return
        try:
            self.joint_samples.append(pm.joint_positions(msg.name, msg.position, self.joints))
        except pm.ObservationError as e:
            self._note(str(e))

    def _on_pose(self, msg):
        try:
            pose = pm.extract_model_pose(msg.transforms)
        except pm.ObservationError as e:
            if self.recording:
                self._note(str(e))
            return
        self.latest_pose = pose
        if self.recording:
            self.pose_samples.append((self.now_s(), pose))

    def _note(self, err):
        if err not in self.observation_errors:
            self.observation_errors.append(err)

    def wait_for(self, predicate, timeout):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return predicate()

    def controller_check(self, phase):
        states = self.controller_states()
        st = {n: s for n, _, s in states} if states is not None else {}
        if states is None:
            self._note(f'/controller_manager/list_controllers unavailable at {phase}')
        return (self.now_s(), phase, {c: st.get(c, 'missing') for c in CONTROLLERS})


def run(node, posture, order, args):
    """Collect everything evaluate() needs. Returns the run dict."""
    run = {'controller_checks': [], 'action': {'server_available': False, 'accepted': False,
                                               'error_code': None}, 'times': {}}
    obs = node.observation_errors
    if not node.wait_for(lambda: node.get_clock().now().nanoseconds > 0, args.ready_timeout):
        obs.append('no /clock received: is the simulation running?')
        return run
    run['controller_checks'].append(node.controller_check('start'))
    if not node.wait_for(lambda: node.positions is not None, args.ready_timeout):
        obs.append('no /joint_states received')
        return run
    if not node.wait_for(lambda: node.latest_pose is not None, args.ready_timeout):
        obs.append(f'no body pose for model "{pm.MODEL_NAME}" on {pm.POSE_TOPIC}. Start the '
                   'simulation with: ros2 launch spiderx_bringup fortress_posture_hold.launch.py')
        return run
    run['joint_state_publishers'] = [node.count_publishers('/joint_states')]

    try:
        start = pm.joint_positions(list(node.positions), list(node.positions.values()), order)
    except pm.ObservationError as e:
        obs.append(str(e))
        return run
    run['start_positions'] = start
    run['peak_joint_speed_rad_s'] = peak_joint_speed(
        start, posture['targets'], posture['command_duration_s'])
    if run['peak_joint_speed_rad_s'] > posture['max_joint_velocity_rad_s']:
        obs.append('trajectory refused: peak joint speed above the configured maximum')
        return run
    if any(s != 'active' for s in run['controller_checks'][-1][2].values()):
        obs.append('controllers not active before the command; nothing was sent')
        return run

    run['action']['server_available'] = node.action.wait_for_server(timeout_sec=args.ready_timeout)
    if not run['action']['server_available']:
        return run
    run['times']['command_sent_s'] = node.now_s()
    print(f'Sending ONE {posture["command_duration_s"]:.1f} s trajectory for all 12 joints '
          f'(peak joint speed {run["peak_joint_speed_rad_s"]:.3f} rad/s) ...')
    accepted, code = node.send_positions(
        order, [posture['targets'][j] for j in order], posture['command_duration_s'])
    run['action'].update(accepted=accepted, error_code=code)
    run['times']['trajectory_done_s'] = node.now_s()
    if not accepted or code is None:
        return run

    hold_s = posture['hold_duration_s']
    print(f'Trajectory finished (error_code {code}). Holding for {hold_s:.1f} s of simulation '
          'time and recording ...')
    node.recording = True
    t0 = node.now_s()
    wall_end = time.monotonic() + args.hold_wall_timeout_factor * hold_s + 60.0
    next_check = t0 + args.controller_poll_s
    while node.now_s() < t0 + hold_s and time.monotonic() < wall_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.now_s() >= next_check:
            run['controller_checks'].append(node.controller_check('hold'))
            next_check += args.controller_poll_s
    t1 = node.now_s()
    node.recording = False
    if t1 < t0 + hold_s:
        obs.append(f'hold did not complete within the wall-clock limit (simulation time advanced '
                   f'{t1 - t0:.2f} of {hold_s} s): simulation stalled?')
    run['hold'] = {'start_s': t0, 'end_s': t1}
    run['controller_checks'].append(node.controller_check('end'))
    run['joint_state_publishers'].append(node.count_publishers('/joint_states'))
    if node.joint_samples:
        run['final_positions'] = node.joint_samples[-1]
        run['joints_moved'] = sorted(
            j for j in order if abs(run['final_positions'][j] - start[j]) > 0.01)
    run['joint_samples'] = node.joint_samples
    run['pose_samples'] = node.pose_samples
    return run


def print_summary(report):
    m = report['metrics']
    print('-' * 72)
    print('SpiderX M2 posture hold - SIMULATION ONLY (Gazebo Fortress)')
    print(f'Posture: {report["posture"]} (config v{report["config_version"]}, '
          f'{report["config_date"]})')
    for t in report['controller_checks']:
        print(f'  controllers @ {t["phase"]:<5} t={t["sim_time_s"]:.2f}s: {t["states"]}')
    print(f'/joint_states publishers (start, end): {report["joint_state_publishers"]}')
    print(f'Action: {report["action"]}')
    if report.get('final_positions_rad'):
        print(f'{"joint":<16}{"start":>9}{"target":>9}{"final":>9}{"hold max|e|":>13}')
        per = m['joints']['per_joint_max_abs_error_rad']
        for j, tgt in report['commanded_positions_rad'].items():
            print(f'{j:<16}{report["start_positions_rad"][j]:>+9.4f}{tgt:>+9.4f}'
                  f'{report["final_positions_rad"][j]:>+9.4f}{per.get(j, float("nan")):>13.4f}')
        print(f'Joints that moved > 0.01 rad: {len(report["joints_moved_more_than_0_01_rad"])}'
              f'/12 {report["joints_moved_more_than_0_01_rad"]}')
        if not report['joints_moved_more_than_0_01_rad']:
            print('NOTE: every joint already started at the pose; the trajectory did not '
                  'exercise motion.')
    if m['joints']['max_abs_error_rad'] is not None:
        th = report['thresholds']
        print(f'Hold: {m["hold_duration_s"]:.2f} s simulation time, '
              f'{m["joint_state_samples"]} joint samples')
        print(f'Joint error: max {m["joints"]["max_abs_error_rad"]:.4f} rad '
              f'(<= {th["max_joint_error_rad"]}), RMS {m["joints"]["rms_error_rad"]:.4f} rad '
              f'(<= {th["rms_joint_error_rad"]})')
    b = m.get('body')
    if b:
        th = report['thresholds']
        print(f'Body ({b["samples"]} samples; base_link in Gazebo world frame):')
        print(f'  height min {b["height_m"]["min"]:.4f} m (>= {th["min_body_height_m"]}), '
              f'range {b["height_m"]["range"]:.4f} m (<= {th["max_body_height_range_m"]}), '
              f'final {b["height_m"]["final"]:.4f} m')
        print(f'  |roll| max {b["roll_rad"]["max_abs"]:.4f} rad (<= {th["max_abs_roll_rad"]}), '
              f'|pitch| max {b["pitch_rad"]["max_abs"]:.4f} rad (<= {th["max_abs_pitch_rad"]}), '
              f'tilt max {b["tilt_rad_max"]:.4f} rad')
        print(f'  xy drift {b["xy_drift_m"]:.4f} m, yaw drift {b["yaw_drift_rad"]:.4f} rad '
              '(informational)')
    print(f'Foot contact: {report["foot_contact"]["status"]} (not measured)')
    for f in report['failures']:
        print(f'FAILED: {f}')
    print(report['outcome'])
    print('Simulation-only posture hold. Not balance, walking, IK or hardware validation.')


def main(argv=None):
    p = argparse.ArgumentParser(description='SpiderX M2 simulation-only posture-hold test')
    p.add_argument('--posture', default=DEFAULT_POSTURE,
                   help=f'posture in m2_simulation_postures.yaml (default {DEFAULT_POSTURE})')
    p.add_argument('--config', default=None,
                   help='posture YAML (default: installed config/m2_simulation_postures.yaml)')
    p.add_argument('--output', default=default_output(), help='report JSON path')
    p.add_argument('--ready-timeout', type=float, default=60.0,
                   help='wall-clock wait for clock/topics/controllers/action [s] (default 60)')
    p.add_argument('--controller-poll-s', type=float, default=2.0,
                   help='controller-state poll period during the hold, simulation s (default 2)')
    p.add_argument('--hold-wall-timeout-factor', type=float, default=20.0,
                   help='wall-clock limit for the hold = factor x hold + 60 s (default 20)')
    args = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:])

    print('SIMULATION ONLY - posture hold of the existing CAD neutral pose. Not balance, walking, '
          'IK or hardware validation.')
    try:
        posture, order = load_posture(args.posture, args.config)
    except PostureConfigError as e:
        print(f'REFUSED: invalid M2 posture configuration: {e}')
        print('Nothing was sent.')
        return 2
    print(f'Posture "{posture["name"]}": 12 targets validated against the URDF limits minus '
          f'{posture["margin"]} rad and equal to cad_neutral.')

    rclpy.init()
    node = PostureHoldNode(order)
    try:
        data = run(node, posture, order, args)
    except KeyboardInterrupt:
        print('Interrupted: cancelling the active trajectory goal.')
        node.cancel_active_goal()
        raise
    finally:
        data_obs = list(node.observation_errors)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    data['observation_errors'] = data_obs
    passed, failures, metrics = pm.evaluate(posture, data)
    report = pm.build_report(posture, data, passed, failures, metrics)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(report, f, indent=2, sort_keys=False)
    print_summary(report)
    print(f'Report: {args.output}')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
