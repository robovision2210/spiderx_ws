"""M6.0-A OFFLINE conversion and preflight (CLI). No ROS graph, no Gazebo, no action goal.

    ros2 run spiderx_controller m6_offline_preflight                # convert + preflight + write
    ros2 run spiderx_controller m6_offline_preflight --check-only   # convert + preflight only
    ros2 run spiderx_controller m6_offline_preflight --trajectory F.json  # preflight a given file

Converts the M4-validated crouch_10mm pose into the ONE canonical M6.0-D trajectory
(neutral -> crouch_10mm -> neutral) and preflights it against the fixed M6.0 envelope.
Writes <out>/<trajectory_id>/trajectory.json and preflight.json (default out:
log/m6_playback/offline, git-ignored); an existing directory is never overwritten.

This module imports no ROS client library and cannot construct or send an action goal.

Exit codes: 0 = preflight PASS; 2 = REFUSED (preflight failure, unreadable input or an existing
output directory). Nothing is ever sent.
"""

import argparse
import json
import os
import sys

from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_trajectory as m6t

EXIT_OK, EXIT_REFUSED = 0, 2
DEFAULT_OUT = os.path.join('log', 'm6_playback', 'offline')
SCOPE = ('Scope: M6.0-A OFFLINE conversion and preflight. No ROS node, no controller, no Gazebo, '
         'no action goal. A PASS means the trajectory satisfies the M6.0 envelope; it is not '
         'evidence of motion, tracking, contact or walking.')


def parse_args(argv):
    p = argparse.ArgumentParser(description='SpiderX M6.0-A offline trajectory preflight')
    p.add_argument('--config-dir', default=None,
                   help='configuration directory (default: installed)')
    p.add_argument('--trajectory', default=None,
                   help='preflight this trajectory JSON instead of converting the crouch pose')
    p.add_argument('--out', default=DEFAULT_OUT,
                   help=f'output root (default: {DEFAULT_OUT}); files in <out>/<trajectory_id>/')
    p.add_argument('--check-only', action='store_true', help='write nothing')
    p.add_argument('--json', action='store_true', help='print the preflight report as JSON')
    return p.parse_args(argv)


def _print_report(report, traj):
    print(f'Preflight: {"PASS" if report.ok else "REFUSED"}  trajectory_id={report.trajectory_id}')
    s = report.summary
    if s:
        print(f'  points {s["points"]}, start delay {s["start_delay_s"]} s, duration '
              f'{s["duration_s"]} s, waypoints {" -> ".join(map(str, s["labels"]))}')
        print(f'  max commanded displacement from neutral {s["max_displacement_rad"]:.10f} rad '
              f'({s["max_displacement_joint"]}); cap {env.M6_0_D_MAX_DISPLACEMENT_RAD} rad '
              f'(+ epsilon {env.DISPLACEMENT_EPSILON_RAD})')
    print(f'  separate quantities: tracking tolerance {env.TRACKING_TOLERANCE_RAD} rad (not a '
          'command cap); joint-limit soft margin from spiderx_legs.yaml (limits only)')
    for name, state in report.checks.items():
        print(f'  [{state:7s}] {name}')
    for code, msg in report.failures:
        print(f'  REFUSED {code}: {msg}')
    if isinstance(traj, dict) and report.ok:
        print('  joint order: ' + ', '.join(traj['joint_names']))


def main(argv=None, sources=None):
    argv = list(argv if argv is not None else sys.argv)
    args = parse_args(argv[1:])
    print('SpiderX M6.0-A offline trajectory preflight')
    print(SCOPE)
    if sources is None:
        sources = m6t.load_sources(args.config_dir)
    if args.trajectory:
        try:
            traj = m6t.load_trajectory_file(args.trajectory)
        except (OSError, ValueError) as e:
            print(f'REFUSED: cannot read trajectory {args.trajectory}: {e}')
            return EXIT_REFUSED
    else:
        try:
            traj = m6t.build_trajectory(sources)
        except m6t.TrajectoryBuildError as e:
            report = m6t.preflight({}, sources)
            _print_report(report, None)
            print(f'REFUSED: {e}')
            return EXIT_REFUSED
    report = m6t.preflight(traj, sources)
    _print_report(report, traj)
    if args.json:
        print(json.dumps(report.to_dict(), indent=1, sort_keys=True))
    if not report.ok:
        print('REFUSED: the trajectory failed preflight; no goal can be constructed or sent.')
        return EXIT_REFUSED
    if not args.check_only:
        d = os.path.join(args.out, report.trajectory_id)
        if os.path.exists(d):
            print(f'REFUSED: {d} already exists (results are never overwritten); use another --out')
            return EXIT_REFUSED
        m6t.write_json(os.path.join(d, 'trajectory.json'), traj)
        m6t.write_json(os.path.join(d, 'preflight.json'), report.to_dict())
        print(f'Wrote {d}/trajectory.json and preflight.json')
    print('PASS: offline preflight only. Nothing was sent; no motion, tracking, contact or walking '
          'is shown.')
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
