#!/usr/bin/env python3
"""M1 all-joint neutral-pose test for SpiderX - JOINT-POSITION TEST ONLY.

    ros2 run spiderx_controller test_neutral_pose.py

Commands all 12 joints, in ONE slow trajectory, to the documented pose `cad_neutral`
(spiderx_controller/config/spiderx_poses.yaml) and reports the per-joint final error.
It is NOT a standing controller: there is no balance, no body-height control and no feedback
beyond each joint's position tracking.

Needs: ros2 launch spiderx_bringup fortress_control.launch.py (controllers active).
Exit code 0 = PASS, 1 = FAIL, 2 = refused / not ready.
"""

import argparse
import sys

import rclpy

from spiderx_controller.joint_safety import (
    DEFAULT_MARGIN_RAD, UnsafeCommandError, check_pose, load_limits, load_pose)
from spiderx_controller.trajectory_client import JointTestNode

DISCLAIMER = 'Joint-position test only - this is NOT a standing controller.'


def main():
    p = argparse.ArgumentParser(description='SpiderX M1 neutral-pose joint-position test')
    p.add_argument('--pose', default='cad_neutral', help='named pose (default cad_neutral)')
    p.add_argument('--duration', type=float, default=6.0, help='motion duration [s] (default 6.0)')
    p.add_argument('--tolerance', type=float, default=0.05,
                   help='pass threshold on the max |final error| [rad] (default 0.05)')
    p.add_argument('--settle', type=float, default=2.0, help='wait after motion [s] (default 2.0)')
    args = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    print(DISCLAIMER)
    limits, order = load_limits()
    try:
        pose = load_pose(args.pose)
        check_pose(pose, limits, DEFAULT_MARGIN_RAD)
        if args.duration < 3.0:
            raise UnsafeCommandError('duration must be >= 3.0 s for a 12-joint move')
    except UnsafeCommandError as e:
        print(f'REFUSED: {e}')
        return 2
    print(f'Pose "{args.pose}": all 12 targets validated against URDF limits '
          f'(margin {DEFAULT_MARGIN_RAD} rad)')

    rclpy.init()
    node = JointTestNode('spiderx_test_neutral_pose')
    try:
        ctrls = node.controller_states()
        if ctrls is None or not any(n == 'leg_trajectory_controller' and s == 'active'
                                    for n, _, s in ctrls):
            print('NOT READY: leg_trajectory_controller is not active. '
                  'Start: ros2 launch spiderx_bringup fortress_control.launch.py')
            return 2
        before = node.fresh_positions()
        if before is None or any(j not in before for j in order):
            print('NOT READY: /joint_states does not contain all 12 joints')
            return 2
        accepted, code = node.send_positions(order, [pose[j] for j in order], args.duration)
        if not accepted:
            print('FAIL: trajectory goal was not accepted')
            return 1
        node.spin_for(args.settle)
        after = node.fresh_positions()

        print(f'{"joint":<16}{"before":>9}{"target":>9}{"after":>9}{"|error|":>9}')
        worst = 0.0
        for j in order:
            e = abs(after[j] - pose[j])
            worst = max(worst, e)
            print(f'{j:<16}{before[j]:>+9.4f}{pose[j]:>+9.4f}{after[j]:>+9.4f}{e:>9.4f}')
        moved = [j for j in order if abs(after[j] - before[j]) > 0.01]
        verdict = 'PASS' if worst <= args.tolerance else 'FAIL'
        print(f'Joints that moved > 0.01 rad: {len(moved)}/12 {moved}')
        if not moved:
            print('NOTE: every joint was already at the pose, so this run did not exercise motion.')
        print(f'Max |error| = {worst:.4f} rad (threshold {args.tolerance}) -> {verdict}; '
              f'action error_code={code} (0 = SUCCESSFUL)')
        print(DISCLAIMER)
        return 0 if verdict == 'PASS' else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
