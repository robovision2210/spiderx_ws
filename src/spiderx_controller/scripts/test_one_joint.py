#!/usr/bin/env python3
"""M1 single-joint position test for SpiderX (joint-position test only).

    ros2 run spiderx_controller test_one_joint.py --joint lf_hip --target 0.2

Needs: ros2 launch spiderx_bringup fortress_control.launch.py (controllers active).

Safety:
  * refuses any joint that is not one of the 12 validated SpiderX joints;
  * refuses targets outside [URDF lower + margin, URDF upper - margin];
  * sends exactly ONE slow trajectory; the other 11 joints are held where they are;
  * --return-to-initial sends one more trajectory back to the starting position;
  * Ctrl-C while waiting sends that return trajectory and exits.
Exit code 0 = PASS, 1 = FAIL, 2 = refused / not ready.
"""

import argparse
import sys

import rclpy

from spiderx_controller.joint_safety import (
    DEFAULT_MARGIN_RAD, UnsafeCommandError, check_target, load_limits)
from spiderx_controller.trajectory_client import JointTestNode


def main():
    p = argparse.ArgumentParser(description='SpiderX M1 single-joint position test')
    p.add_argument('--joint', required=True, help='one of the 12 SpiderX joint names')
    p.add_argument('--target', type=float, default=0.2, help='target position [rad] (default 0.2)')
    p.add_argument('--duration', type=float, default=3.0, help='motion duration [s] (default 3.0)')
    p.add_argument('--margin', type=float, default=DEFAULT_MARGIN_RAD,
                   help='keep-out margin inside the URDF limits [rad] (default 0.05)')
    p.add_argument('--tolerance', type=float, default=0.05,
                   help='pass threshold on |final error| [rad] (default 0.05)')
    p.add_argument('--settle', type=float, default=1.0, help='wait after motion [s] (default 1.0)')
    p.add_argument('--return-to-initial', action='store_true',
                   help='afterwards, move the joint back to its starting position')
    args = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    limits, order = load_limits()
    try:
        check_target(args.joint, args.target, limits, args.margin)
        if args.duration < 1.0:
            raise UnsafeCommandError('duration must be >= 1.0 s (slow test motion)')
    except UnsafeCommandError as e:
        print(f'REFUSED: {e}')
        return 2

    rclpy.init()
    node = JointTestNode('spiderx_test_one_joint')
    try:
        print('SpiderX M1 single-joint test (joint-position test only - not standing/walking)')
        ctrls = node.controller_states()
        if ctrls is None:
            print('NOT READY: /controller_manager/list_controllers unavailable. '
                  'Start: ros2 launch spiderx_bringup fortress_control.launch.py')
            return 2
        print('Controllers:')
        for name, ctype, state in ctrls:
            print(f'  {name:<28} {ctype:<55} {state}')
        if not any(n == 'leg_trajectory_controller' and s == 'active' for n, _, s in ctrls):
            print('NOT READY: leg_trajectory_controller is not active')
            return 2

        before = node.fresh_positions()
        if before is None or any(j not in before for j in order):
            print('NOT READY: /joint_states does not contain all 12 joints')
            return 2
        initial = [before[j] for j in order]
        targets = [args.target if j == args.joint else before[j] for j in order]
        print(f'Requested joint : {args.joint}')
        print(f'Requested target: {args.target:+.4f} rad over {args.duration:.1f} s')
        print(f'Observed before : {before[args.joint]:+.4f} rad')

        try:
            accepted, code = node.send_positions(order, targets, args.duration)
            node.spin_for(args.settle)
        except KeyboardInterrupt:
            print('Interrupted: returning joints to their initial positions')
            node.send_positions(order, initial, max(args.duration, 2.0))
            return 1
        if not accepted:
            print('FAIL: trajectory goal was not accepted')
            return 1

        after = node.fresh_positions()
        err = abs(after[args.joint] - args.target)
        others = max(abs(after[j] - before[j]) for j in order if j != args.joint)
        verdict = 'PASS' if err <= args.tolerance else 'FAIL'
        print(f'Observed after  : {after[args.joint]:+.4f} rad')
        print(f'Error magnitude : {err:.4f} rad (threshold {args.tolerance:.4f}) -> {verdict}')
        print(f'Action result   : error_code={code} (0 = SUCCESSFUL)')
        print(f'Largest change of the 11 held joints: {others:.4f} rad (information only)')

        if args.return_to_initial:
            node.send_positions(order, initial, args.duration)
            node.spin_for(args.settle)
            back = node.fresh_positions()
            print(f'Returned to initial: {args.joint} = {back[args.joint]:+.4f} rad '
                  f'(initial {before[args.joint]:+.4f})')
        return 0 if verdict == 'PASS' else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
