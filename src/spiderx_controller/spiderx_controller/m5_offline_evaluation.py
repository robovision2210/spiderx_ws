"""M5 OFFLINE evaluation study runner (CLI). Never starts Gazebo, ROS nodes or hardware.

    ros2 run spiderx_controller m5_offline_evaluation --check-only   # validate study + accounting
    ros2 run spiderx_controller m5_offline_evaluation --dry-run      # list every planned evaluation
    ros2 run spiderx_controller m5_offline_evaluation                # run the staged study

Exit codes: 0 = study completed (gate passed; failing, infeasible and N/A outcomes are RESULTS,
not errors); 2 = invalid study specification or configuration (nothing evaluated);
3 = the Stage 0/1 gate failed: Stages 2-4 were not evaluated (blocked), evidence is reported.
"""

import argparse
import json
import sys

from spiderx_controller import eval_runner as er
from spiderx_controller import eval_study as es
from spiderx_controller import gait_config as gc

EXIT_OK, EXIT_INVALID, EXIT_BLOCKED = 0, 2, 3
SCOPE = ('Scope: OFFLINE model analysis of gait configuration classes on the SpiderX URDF model '
         'with the unchanged M4.5 evaluator. Not walking, not dynamic stability, not energy, not '
         'hardware. Static support is an approximation; motion "energy" values are heuristic '
         'proxies; 0.5 rad/s is a provisional screening flag only.')


def parse_args(argv):
    p = argparse.ArgumentParser(description='SpiderX M5 offline evaluation study')
    p.add_argument('--study', default=None, help='study YAML (default: installed m5_study.yaml)')
    p.add_argument('--config-dir', default=None,
                   help='directory with m4_5_gaits.yaml and spiderx_legs.yaml (default: installed)')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--check-only', action='store_true',
                      help='validate the study and its accounting; evaluate nothing')
    mode.add_argument('--dry-run', action='store_true',
                      help='print every planned evaluation; evaluate and write nothing')
    return p, p.parse_args(argv)


def print_accounting(acc):
    print('Accounting (planned / duplicates of earlier stages / new unique evaluations):')
    for sid in es.STAGES:
        print(f'  {sid:4s} {acc["planned"][sid]:4d} / {acc["duplicates_of_earlier"][sid]:3d} / '
              f'{acc["new_unique"][sid]:4d}')
    s03, alls = acc['stages_0_3'], acc['all_stages']
    print(f'  Stages 0-3: {s03["planned"]} planned, {s03["unique"]} unique (configuration, n) '
          f'pairs, {s03["distinct_configurations"]} distinct configurations')
    print(f'  Stages 0-4: {alls["planned"]} planned, {alls["unique"]} unique (configuration, n) '
          f'pairs, {alls["distinct_configurations"]} distinct configurations')
    print(f'  Speed-scale assumption checks (counted separately): {acc["speed_checks"]}')


def print_plan(plan):
    for e in plan.entries + plan.speed_entries:
        dup = f'  = {e.first_label}' if e.duplicate else ''
        print(f'{e.stage:5s} {e.label:44s} eval {e.eval_id} config {e.config_id} n={e.n}{dup}')


def main(argv=None, evaluate=None, out_writer=None):
    _, args = parse_args((argv if argv is not None else sys.argv)[1:])
    print('SpiderX M5 offline evaluation study')
    print(SCOPE)
    try:
        study, context = es.load_study(args.study, config_dir=args.config_dir)
        plan = es.build_plan(study, context)
    except gc.GaitConfigError as e:
        print(f'REFUSED: {e}')
        print('Nothing was evaluated.')
        return EXIT_INVALID
    print(f'Study {study.study_id} (schema {es.SCHEMA})')
    print_accounting(plan.accounting)
    if args.check_only:
        print('Study valid (--check-only: nothing evaluated, nothing written).')
        return EXIT_OK
    if args.dry_run:
        print_plan(plan)
        print(f'Dry run: {len(plan.entries)} planned stage evaluations + '
              f'{len(plan.speed_entries)} speed checks listed; nothing evaluated, nothing written.')
        return EXIT_OK

    inputs = er.load_inputs(config_dir=args.config_dir) if evaluate is None else None
    result = er.run_study(plan, inputs, evaluate=evaluate or er.evaluate_variant,
                          progress=lambda i, n, label: print(f'  [{i:3d}/{n}] {label}',
                                                             flush=True))
    violations = er.verify_result(result)
    if violations:                       # an internal invariant broke: never report as a result
        print('INTERNAL ERROR: ' + '; '.join(violations))
        return EXIT_INVALID
    print('Stage 0/1 gate:')
    for g in result.gate:
        print(f'  {"PASS" if g.passed else "FAIL"}  {g.check_id}: {g.detail}')
    print(json.dumps({'study_id': study.study_id, 'status': result.status,
                      'counts': er.category_counts(result)}, indent=1, sort_keys=True))
    if out_writer is not None:
        out_writer(result)
    if result.status == 'blocked':
        print('BLOCKED: the Stage 0/1 gate failed; Stages 2-4 were not evaluated.')
        return EXIT_BLOCKED
    print('Study completed (offline model results; see the scope statement above).')
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
