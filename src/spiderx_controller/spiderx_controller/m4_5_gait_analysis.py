"""M4.5 OFFLINE gait analysis runner (CLI). Never starts Gazebo, ROS nodes or hardware.

    ros2 run spiderx_controller m4_5_gait_analysis                 # all gaits, tables + figures
    ros2 run spiderx_controller m4_5_gait_analysis --gait trot --gait wave
    ros2 run spiderx_controller m4_5_gait_analysis --no-plots --out /tmp/gaits
    ros2 run spiderx_controller m4_5_gait_analysis --check-only    # validate the YAML only
    ros2 run spiderx_controller m4_5_gait_analysis --strict        # exit 1 if any gait fails

Exit codes: 0 = analysis completed (a FAILING gait is a result, not an error - see the report);
1 = --strict and at least one gait failed; 2 = invalid configuration or geometry (nothing
evaluated).
"""

import argparse
import os
import sys

from spiderx_controller import gait_config as gc
from spiderx_controller import gait_metrics as gm
from spiderx_controller import gait_report as gr

DEFAULT_OUT = os.path.join('log', 'm4_5_gait_analysis')
SCOPE = ('Scope: OFFLINE kinematic analysis of gait configurations. Not walking, not a validated '
         'gait, not balance control, not hardware. Stability is a quasi-static approximation; '
         '"energy" values are heuristic proxies.')


def _summary_line(ev):
    m = ev.metrics
    margin = m['min_static_margin_m']
    return (f'  {ev.spec.name:<13} {"PASS" if ev.passed else "FAIL":<4}  '
            f'min margin {"   n/a" if margin is None else f"{margin * 1000:6.2f}"} mm  '
            f'stable {m["fraction_statically_stable"]:5.1%}  '
            f'max joint speed {m["max_joint_speed_rad_s"]:.3f} rad/s'
            + (f'  failed: {", ".join(ev.failed_checks)}' if ev.failed_checks else ''))


def main(argv=None):
    p = argparse.ArgumentParser(description='SpiderX M4.5 offline gait analysis')
    p.add_argument('--config', default=None, help='gait YAML (default: installed m4_5_gaits.yaml)')
    p.add_argument('--config-dir', default=None,
                   help='directory with spiderx_legs.yaml (default: installed config)')
    p.add_argument('--out', default=DEFAULT_OUT, help=f'output root (default: {DEFAULT_OUT})')
    p.add_argument('--gait', action='append', default=None, help='evaluate only this gait '
                   '(repeatable; default: all, in configuration order)')
    p.add_argument('--no-plots', action='store_true', help='write tables only')
    p.add_argument('--check-only', action='store_true', help='validate the configuration only')
    p.add_argument('--strict', action='store_true', help='exit 1 if any evaluated gait fails')
    args = p.parse_args((argv or sys.argv)[1:])

    print('SpiderX M4.5 offline gait analysis')
    print(SCOPE)
    try:
        if args.check_only:
            cfg = gc.load_gait_config(config_path=args.config, config_dir=args.config_dir)
        else:
            cfg, geoms, mass = gm.load_inputs(config_path=args.config, config_dir=args.config_dir)
        names = args.gait if args.gait else [g.name for g in cfg.gaits]
        specs = [cfg.gait(n) for n in names]
    except gc.GaitConfigError as e:
        print(f'REFUSED: {e}')
        print('Nothing was evaluated.')
        return 2
    print(f'Configuration v{cfg.config_version} ({cfg.date}): {len(cfg.gaits)} gaits, '
          f'{cfg.analysis.samples_per_cycle} samples per cycle')
    for s in specs:
        print('  ' + gc.describe(s))
    if args.check_only:
        print('Configuration valid (--check-only: no kinematics evaluated).')
        return 0

    plots = not args.no_plots and gr.plotting_available()
    if not args.no_plots and not plots:
        print('NOTE: matplotlib is not installed; tables are written, figures are skipped.')
    evaluations = []
    for spec in specs:
        ev = gm.evaluate_gait(spec, cfg, geoms, mass)
        gr.write_gait_report(ev, cfg, args.out, plots=plots)
        evaluations.append(ev)
    gr.write_run_info(cfg, evaluations, args.out, plots)
    print('Results (offline):')
    for ev in evaluations:
        print(_summary_line(ev))
    print(f'Reports: {os.path.abspath(args.out)}')
    failed = [ev.spec.name for ev in evaluations if not ev.passed]
    if failed:
        print(f'{len(failed)} gait(s) FAILED their checks: {failed} (see <gait>/report.json).')
    else:
        print('All evaluated gaits passed their offline checks.')
    return 1 if args.strict and failed else 0


if __name__ == '__main__':
    sys.exit(main())
