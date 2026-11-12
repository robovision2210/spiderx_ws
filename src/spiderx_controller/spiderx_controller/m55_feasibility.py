"""M5.5 offline free-base gait feasibility report (`m55_gait_feasibility`).

Designs and validates the crawl template of every configured speed level (m55_crawl), builds the
phase-goal library the locomotion node would use, and records why the first free-base gait is a
crawl with body shift rather than a trot:
  * static margin of each three-foot support triangle in the neutral stance WITHOUT body shift;
  * the trot's two-foot (diagonal) support: zero area, so no static margin exists; the report
    gives the COM's distance from each diagonal for information only.
Offline only: no ROS graph, no simulator, nothing sent. Exit 0 = every level validated,
1 = at least one level failed validation, 2 = usage or configuration error.
"""

import argparse
import datetime
import json
import math
import os
import sys
import time

from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_crawl as cr
from spiderx_controller import m55_locomotion as loc

REPORT_SCHEMA = 'spiderx.m55.gait_feasibility/1'
TROT_DIAGONALS = (('front_left', 'rear_right'), ('front_right', 'rear_left'))


def _segment_distance(p, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / (ex * ex + ey * ey)))
    return math.hypot(a[0] + t * ex - p[0], a[1] + t * ey - p[1])


def support_analysis(designer):
    """Neutral-stance support facts (base_link, m): COM, 3-foot margins without body shift, and
    the trot diagonals (two feet: no support polygon, no static margin)."""
    from spiderx_controller import gait_metrics as gm
    q0 = {n: 0.0 for n in designer.joint_order}
    com = designer.mass.com(q0)
    tips = designer.tips
    three = {}
    for leg in lk.ALL_LEGS:
        tri = [tips[lg][:2] for lg in lk.ALL_LEGS if lg != leg]
        three[leg] = gm.stability_margin(com[:2], tri)
    trot = {f'{a}+{b}': {'support_area_m2': 0.0, 'static_margin_m': None,
                         'com_distance_to_diagonal_m': _segment_distance(com[:2], tips[a][:2],
                                                                         tips[b][:2])}
            for a, b in TROT_DIAGONALS}
    return {'com_neutral_base_link_m': list(com),
            'four_foot_margin_m': gm.stability_margin(com[:2], [t[:2] for t in tips.values()]),
            'three_foot_margin_no_body_shift_m': three, 'trot_two_foot_support': trot,
            'labels': {'margins': 'quasi-static approximation (CAD masses, point feet, flat '
                                  'ground, no slip, no dynamics)',
                       'trot': 'two-foot support has zero area: no static margin exists; '
                               'stability would be dynamic and is NOT shown by this repository'}}


def parse_args(argv):
    p = argparse.ArgumentParser(prog='m55_gait_feasibility', description=__doc__.split('\n')[0])
    p.add_argument('--strides', type=float, nargs='+', default=None,
                   help='stride levels (m); default: the configured gait.stride_levels_m')
    p.add_argument('--out', default=os.path.join('log', 'm55_gait_feasibility'))
    p.add_argument('--no-write', action='store_true')
    return p.parse_args(argv)


def main(argv=None, config_dir=None, designer=None, utc=None):
    try:
        args = parse_args(sys.argv[1:] if argv is None else argv)
        cfg = loc.load_config(config_dir)
        strides = cfg.gait.stride_levels_m if args.strides is None else tuple(args.strides)
        for s in strides:
            cfg.crawl_params(s).check()
    except (loc.LocomotionError, cr.CrawlError, OSError) as e:
        print(f'configuration error: {e}', file=sys.stderr)
        return 2
    except SystemExit as e:
        return 2 if e.code else 0
    designer = designer or cr.load_designer()
    levels, temps = [], []
    for s in strides:
        t0 = time.monotonic()
        try:
            t = cr.build_template(designer, cfg.crawl_params(s))
            temps.append(t)
            levels.append({'stride_m': s, 'ok': True, 'build_s': time.monotonic() - t0,
                           **{k: v for k, v in t.report.items() if k != 'labels'}})
        except cr.CrawlError as e:
            levels.append({'stride_m': s, 'ok': False, 'error': str(e),
                           'build_s': time.monotonic() - t0})
    library = None
    if temps and len(temps) == len(strides):
        try:
            library = loc.PhaseLibrary(temps, cfg.dispatch.phase_lead_s).describe()
        except loc.LocomotionError as e:
            levels.append({'library_error': str(e)})
    ok = all(lv.get('ok') for lv in levels if 'stride_m' in lv) and library is not None
    labels = temps[0].report.get('labels') if temps else None
    report = {'schema': REPORT_SCHEMA, 'milestone': c55.MILESTONE,
              'template_labels': labels,         # method, support, COM and units of the margins
              'config_sha256': cfg.sha256, 'config_status': cfg.status,
              'dispatch_gate': c55.M55_LOCOMOTION_DISPATCH_ENABLED, 'ok': ok,
              'levels': levels, 'library': library, 'support': support_analysis(designer),
              'non_claims': list(c55.NON_CLAIMS)}
    print(f'{"stride":>7} {"cycle":>6} {"speed*":>8} {"margin":>7} {"qdot":>6} {"spline":>8}  '
          f'phase durations (s)')
    speeds = {} if library is None else {round(lv['stride_m'], 9): lv['speed_m_s']
                                         for lv in library['levels']}
    for lv in levels:
        if 'stride_m' not in lv:
            continue
        if not lv['ok']:
            print(f'{lv["stride_m"] * 1000:5.0f}mm  FAILED: {lv["error"]}')
            continue
        sp = speeds.get(round(lv['stride_m'], 9))
        print(f'{lv["stride_m"] * 1000:5.0f}mm {lv["cycle_s"]:5.1f}s '
              f'{"?" if sp is None else f"{sp * 1000:5.2f}mm/s":>8} '
              f'{lv["min_static_margin_m"] * 1000:5.1f}mm {lv["max_joint_speed_rad_s"]:6.3f} '
              f'{lv["max_spline_error_rad"]:8.5f}  {lv["phase_durations_s"]}')
    print('* speed including the per-phase lead-in; margins are a quasi-static approximation')
    sup = report['support']
    print('three-foot margins without body shift (mm): ' + ', '.join(
        f'{k} {v * 1000:+.1f}' for k, v in sup['three_foot_margin_no_body_shift_m'].items()))
    print('trot: two-foot support, no static margin (dynamic stability not shown)')
    if not args.no_write:
        stamp = utc or datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        out = os.path.join(args.out, stamp)
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, 'report.json'), 'w') as f:
            json.dump(report, f, indent=1, sort_keys=True)
        print(f'report: {os.path.join(out, "report.json")}')
    return 0 if ok else 1


__all__ = ['main', 'support_analysis', 'REPORT_SCHEMA']
