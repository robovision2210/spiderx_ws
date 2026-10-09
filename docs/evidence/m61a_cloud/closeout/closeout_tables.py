#!/usr/bin/env python3
"""Closeout of the Cloud M6.1-A phase-1 observation (evidence helper; read-only on the evidence).

Reads ../observation_7f4c30f/ (unchanged; its SHA256SUMS still verifies) and writes, here:
  requirements_by_run.json / .md  - every M6.1-A section 10 criterion (plus the supplementary
                                    checks) per run: PASS / FAIL / UNMEASURED with evidence paths
  scan_masks.json                  - per run: the excluded beam indices with their justification,
                                    raw and seam-filtered results, per-beam error statistics
usage: python3 closeout_tables.py   (run from this directory; needs numpy and the observation tools)
"""
import glob
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OBS = os.path.normpath(os.path.join(HERE, '..', 'observation_7f4c30f'))
sys.path.insert(0, os.path.join(OBS, 'tools'))
import scan_check as sc  # noqa: E402

WORLD = '/home/user/spiderx_ws/src/spiderx_description/worlds/spiderx_fortress.sdf'
RUNS = ('run_01', 'run_02', 'run_03')
SEAM_CENTRES_DEG = (-135.0, -45.0, 45.0, 135.0)


def rel(run, *p):
    return '/'.join(('observation_7f4c30f', run) + p)


def obs_files(run):
    out = {}
    for p in sorted(glob.glob(os.path.join(OBS, run, 'observer', '*', 'observation.json'))):
        d = json.load(open(p))
        out[d['mode']] = (d, os.path.relpath(p, os.path.dirname(OBS)))
    return out


def verdict(ok):
    return 'UNMEASURED' if ok is None else ('PASS' if ok else 'FAIL')


def scan_masks(run):
    cap = json.load(open(os.path.join(OBS, run, 'captures', 'capture.json')))
    s0 = cap['scans'][0]
    n = len(s0['ranges'])
    ang = [s0['angle_min'] + i * s0['angle_increment'] for i in range(n)]
    lx, ly, lz, lyaw = sc.lidar_pose([0.0, 0.0, 0.125, 0.0])
    shapes = sc.world_shapes(WORLD, lz)
    R = np.array([[np.nan if r is None else r for r in s['ranges']] for s in cap['scans']])
    v = np.isfinite(R) & (R > s0['range_min']) & (R < s0['range_max'])
    mean = np.where(v.sum(0) > 0, np.nansum(np.where(v, R, 0), 0) / np.maximum(v.sum(0), 1),
                    np.nan)
    seam, grazing, used = [], [], []
    for i in range(n):
        d_seam = min(abs(math.degrees(math.atan2(math.sin(ang[i] - math.radians(c)),
                                                 math.cos(ang[i] - math.radians(c)))))
                     for c in SEAM_CENTRES_DEG)
        exp, who = sc.ray(shapes, lx, ly, lyaw + ang[i])
        dm = sc.ray(shapes, lx, ly, lyaw + ang[i] - math.radians(0.5))[0]
        dp = sc.ray(shapes, lx, ly, lyaw + ang[i] + math.radians(0.5))[0]
        res = float(mean[i] - exp) if np.isfinite(mean[i]) and np.isfinite(exp) else None
        rec = {'index': i, 'angle_lidar_deg': round(math.degrees(ang[i]), 3),
               'bearing_world_deg': round(math.degrees(lyaw + ang[i]), 3),
               'surface': who, 'expected_m': round(float(exp), 4) if np.isfinite(exp) else None,
               'measured_mean_m': round(float(mean[i]), 4) if np.isfinite(mean[i]) else None,
               'residual_m': None if res is None else round(res, 4)}
        if d_seam <= 1.0:
            seam.append(dict(rec, reason=f'within 1 deg of the gpu_lidar cube-map seam '
                                         f'({d_seam:.3f} deg from it)'))
            continue
        if not np.isfinite(exp) or abs(dm - exp) > 0.05 or abs(dp - exp) > 0.05:
            grazing.append(dict(rec, reason='edge-grazing at the weld-implied pose: the expected '
                                            'range changes > 5 cm within +-0.5 deg'))
            continue
        if res is None:
            continue
        b = lyaw + ang[i]
        inc = None
        if who.startswith('walls/'):
            nx, ny = (1, 0) if who in ('walls/wall_east', 'walls/wall_west') else (0, 1)
            inc = math.degrees(math.acos(min(1.0, abs(math.cos(b) * nx + math.sin(b) * ny))))
        used.append((i, who, res, inc))
    r = np.array([u[2] for u in used])
    wall_inc = [(u[2], u[3]) for u in used if u[3] is not None]

    def band(lo, hi):
        x = np.array([abs(a) for a, i in wall_inc if lo <= i < hi])
        return None if not x.size else {'beams': int(x.size), 'median_abs_mm': round(float(np.median(x)) * 1e3, 2),
                                        'p95_abs_mm': round(float(np.percentile(x, 95)) * 1e3, 2),
                                        'max_abs_mm': round(float(x.max()) * 1e3, 2)}
    raw = json.load(open(os.path.join(OBS, run, 'captures', 'scan_check.json')))
    filt = json.load(open(os.path.join(OBS, run, 'captures', 'scan_check_seams_excluded.json')))
    keep = ('beams_used', 'residual_median_m', 'residual_abs_p95_m', 'residual_abs_max_m',
            'fit_minus_expected_m_m_rad', 'fit_1sigma_m_m_rad', 'fit_residual_rms_m',
            'fit_beams_used')
    return {
        'scans_averaged': len(cap['scans']), 'beams': n,
        'excluded_seam_beams': seam, 'excluded_edge_grazing_beams': grazing,
        'beams_compared': len(used),
        'individual_ranges': {
            'within_8mm_fraction': round(float(np.mean(np.abs(r) <= 0.008)), 4),
            'median_abs_mm': round(float(np.median(np.abs(r))) * 1e3, 2),
            'p95_abs_mm': round(float(np.percentile(np.abs(r), 95)) * 1e3, 2),
            'max_abs_mm': round(float(np.max(np.abs(r))) * 1e3, 2),
            'walls_by_incidence_deg': {'0-30': band(0, 30), '30-45': band(30, 45.001)},
            'expected_noise_of_a_40_scan_mean_mm': round(10.0 / math.sqrt(len(cap['scans'])), 2)},
        'raw_all_beams_scan_check': {k: raw.get(k) for k in keep},
        'seam_filtered_scan_check': dict({k: filt.get(k) for k in keep},
                                         fit_1sigma_robust_m_m_rad=filt.get('fit_1sigma_robust_m_m_rad'),
                                         seam_beams_excluded=filt.get('seam_beams_excluded')),
        'evidence': [rel(run, 'captures', 'capture.json'), rel(run, 'captures', 'scan_check.json'),
                     rel(run, 'captures', 'scan_check_seams_excluded.json'),
                     rel(run, 'captures', 'scan_outliers.json')],
    }


def requirements(run, masks):
    of = obs_files(run)
    pf, pf_path = of['preflight']
    ob, ob_path = of['observe']
    st = ob['statistics']
    body, att = st['body'], st['attachment']
    cap = json.load(open(os.path.join(OBS, run, 'captures', 'capture.json')))
    an = json.load(open(os.path.join(OBS, run, 'analysis.json')))
    row = {r['id'] + '|' + r['criterion']: r for r in an['rows']}
    txt = lambda *p: open(os.path.join(OBS, run, *p)).read()
    ai = re.search(r'Action clients:\s*(\d+)', txt('captures', 'action_info.txt'))
    tp = re.search(r'Publisher count:\s*(\d+)', txt('captures', 'command_topic_info.txt'))
    sd = txt('shutdown.txt')
    lg = txt('launch.log')
    goal_lines = [ln for ln in lg.splitlines() if re.search(
        r'(Received new action goal|Accepted new action goal|new trajectory|Goal)', ln)]
    weld = cap.get('weld_joint') or {}
    link = re.sub(r'\x1b\[[0-9;]*m', '', txt('captures', 'ign_link_dummy_link.txt'))
    m = re.search(r'  - Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]*)\]\s*\[([^\]]*)\]',
                  link)
    gz_link = [float(x) for x in m.group(1).split()] if m else None
    z3 = [r for k, r in row.items() if k.startswith('Z3|')][0]
    cs = cap.get('controller_state') or {}
    sm = masks['seam_filtered_scan_check']
    fit = sm['fit_minus_expected_m_m_rad']
    sig = sm['fit_1sigma_robust_m_m_rad']
    raw = masks['raw_all_beams_scan_check']
    surf = json.load(open(os.path.join(OBS, run, 'captures', 'scan_check_seams_excluded.json')))['surfaces']
    worst = max(abs(v['median_residual_m']) for v in surf.values())
    js = an['facts'].get('joint_states_capture') or {}
    foot = os.path.exists(os.path.join(OBS, run, 'captures', 'screenshot_gui_foot_level.png'))
    R = []

    def add(cid, crit, scope, ok, value, threshold, evidence, note=''):
        R.append({'id': cid, 'criterion': crit, 'scope': scope, 'result': ok if isinstance(ok, str) else verdict(ok),
                  'value': value, 'threshold': threshold, 'evidence': evidence, 'note': note})
    add('1', 'preflight READY', 'per run', pf['ready'], pf['failure_codes'] or 'READY', 'READY, exit 0',
        [pf_path, rel(run, 'preflight.txt')])
    add('1b', '120 s observation READY at its end', 'per run', ob['ready'],
        ob['failure_codes'] or 'READY', 'READY, exit 0', [ob_path, rel(run, 'observe.txt')])
    add('2a', 'body z mean within 1 mm of 0.125 m', 'per run',
        abs(body['z_mean_m'] - 0.125) <= 0.001, body['z_mean_m'], '|z-0.125| <= 0.001 m', [ob_path])
    add('2a', 'body z range <= 1 mm', 'per run', body['z_range_m'] <= 0.001, body['z_range_m'],
        '<= 0.001 m', [ob_path])
    add('2b', 'attachment translation <= 1 mm', 'per run', att['max_translation_m'] <= 0.001,
        att['max_translation_m'], '<= 0.001 m', [ob_path])
    add('2b', 'attachment rotation <= 0.0033 rad', 'per run', att['max_rotation_rad'] <= 0.0033,
        att['max_rotation_rad'], '<= 0.0033 rad', [ob_path])
    add('2c', 'spawn deviation <= 1 mm', 'per run', att['max_spawn_deviation_m'] <= 0.001,
        att['max_spawn_deviation_m'], '<= 0.001 m', [ob_path])
    add('2d', 'link deviation <= 0.1 mm', 'per run', att['max_link_deviation_m'] <= 0.0001,
        att['max_link_deviation_m'], '<= 0.0001 m', [ob_path])
    add('2e', 'tilt <= 0.0033 rad', 'per run', body['tilt_max_rad'] <= 0.0033, body['tilt_max_rad'],
        '<= 0.0033 rad', [ob_path])
    pr = st['pose_receipt']
    add('3', 'pose receipt max gap < 1.0 s (p99.9 recorded)', 'per run', pr['max_s'] < 1.0,
        {'max_s': round(pr['max_s'], 4), 'p999_s': round(pr['p999_s'], 4), 'samples': pr['count']},
        '< 1.0 s (wall)', [ob_path])
    add('3', 'joint-state sim gap max <= 0.25 s', 'per run', st['joint_state_sim_gap_max_s'] <= 0.25,
        round(st['joint_state_sim_gap_max_s'], 4), '<= 0.25 s (sim)', [ob_path])
    add('4', 'no unexpected pose sample codes', 'per run', not st.get('sample_codes'),
        st.get('sample_codes') or {}, 'none', [ob_path])
    add('4', 'header.frame_id values recorded (not gated)', 'per run', True,
        st.get('header_frame_ids'), 'recorded', [ob_path],
        'every entry has an empty frame_id; see the frame-binding analysis')
    if foot:
        add('5', 'visual: legs clear of the ground (GUI screenshot)', 'per campaign (one screenshot)',
            'PASS', 'feet visibly above the floor (qualitative reviewer judgement)', 'visible gap',
            [rel(run, 'captures', 'screenshot_gui_foot_level.png')])
    else:
        add('5', 'visual: legs clear of the ground (GUI screenshot)', 'per campaign (one screenshot)',
            None, 'inconclusive: no foot-level view; the ~16 mm gap is not resolvable in the views taken',
            'visible gap', [rel(run, 'captures', 'screenshot_gui.png'),
                            rel(run, 'captures', 'screenshot_gui_side.png')],
            'preserved as inconclusive (run_01/NOTES.txt)')
    add('6a', '/scan pose agreement: fitted lidar pose within 8 mm, seam beams masked',
        'per run (frozen definition)', math.hypot(fit[0], fit[1]) <= 0.008 and math.hypot(sig[0], sig[1]) < 0.008,
        {'dx_mm': round(fit[0] * 1e3, 3), 'dy_mm': round(fit[1] * 1e3, 3), 'dyaw_mrad': round(fit[2] * 1e3, 3),
         'robust_1sigma_mm': [round(sig[0] * 1e3, 3), round(sig[1] * 1e3, 3)]},
        '|dxy| <= 8 mm and 1 sigma < 8 mm', [rel(run, 'captures', 'scan_check_seams_excluded.json')])
    add('6b', '/scan per-surface median residual within +-8 mm, seam beams masked',
        'per run (frozen definition)', worst <= 0.008, f'worst {worst * 1e3:.2f} mm', '|median| <= 8 mm',
        [rel(run, 'captures', 'scan_check_seams_excluded.json')])
    ir = masks['individual_ranges']
    add('6c', '/scan every individual range within +-8 mm (strict per-beam reading of M0)',
        'per run (reported)', ir['within_8mm_fraction'] == 1.0,
        f"{ir['within_8mm_fraction'] * 100:.1f} % within; p95 {ir['p95_abs_mm']} mm, max {ir['max_abs_mm']} mm",
        'every beam |r| <= 8 mm', [rel(run, 'captures', 'capture.json')],
        'systematic, grows with incidence angle (see scan_masks.json)')
    rf, rs = raw['fit_minus_expected_m_m_rad'], raw['fit_1sigma_m_m_rad']
    add('6-raw', '/scan pose agreement without the seam mask (raw, all beams)', 'per run (reported)',
        math.hypot(rf[0], rf[1]) <= 0.008 and math.hypot(rs[0], rs[1]) < 0.008,
        {'dx_mm': round(rf[0] * 1e3, 3), 'dy_mm': round(rf[1] * 1e3, 3),
         '1sigma_mm': [round(rs[0] * 1e3, 1), round(rs[1] * 1e3, 1)],
         'residual_abs_max_mm': round(raw['residual_abs_max_m'] * 1e3, 1)},
        '|dxy| <= 8 mm and 1 sigma < 8 mm', [rel(run, 'captures', 'scan_check.json')],
        'the fitted value agrees; the 1 sigma is inflated by the seam beams')
    gs, ge = ob['graph']['start'], ob['graph']['end']
    own = (int(ai.group(1)) if ai else None, int(tp.group(1)) if tp else None,
           gs['action_clients'], gs['command_publishers'], ge['action_clients'], ge['command_publishers'])
    add('7', 'command ownership: no other commander visible', 'per run',
        None if None in own else all(x == 0 for x in own),
        {'ros2_action_info_clients': own[0], 'ros2_topic_info_publishers': own[1],
         'observer_start': [own[2], own[3]], 'observer_end': [own[4], own[5]]}, 'all 0',
        [rel(run, 'captures', 'action_info.txt'), rel(run, 'captures', 'command_topic_info.txt'), ob_path])
    add('8', 'clean shutdown: launch group empty, nothing left', 'per run',
        'process group empty' in sd and 'LEFTOVER' not in sd and sd.strip().endswith('none'),
        re.search(r'process group empty after \d+ s', sd).group(0), 'no leftover process',
        [rel(run, 'shutdown.txt')])
    add('M', 'mount from the running model: weld world->dummy_link (0, 0, 0.125, rpy 0)', 'per run',
        weld.get('xyz') == '0.0 0.0 0.125' and weld.get('parent') == 'world' and gz_link is not None
        and all(abs(a - b) < 1e-9 for a, b in zip(gz_link, (0.0, 0.0, 0.125))),
        {'robot_description_weld': weld, 'gazebo_dummy_link_pose_xyz': gz_link}, 'equal',
        [rel(run, 'captures', 'capture.json'), rel(run, 'captures', 'ign_link_dummy_link.txt'),
         rel(run, 'captures', 'ign_model_pose.txt'), rel(run, 'captures', 'ign_joint_weld.txt')])
    add('Z', 'zero motion commands and goals', 'per run',
        not goal_lines and not cap['command_messages'] and
        not any(a['statuses'] for a in cap['action_status']) and ob.get('goals_sent') == 0
        and ob.get('publishers_created') == 0,
        {'launch_log_goal_lines': len(goal_lines), 'command_topic_messages': len(cap['command_messages']),
         'goal_statuses': sum(len(a['statuses']) for a in cap['action_status']),
         'observer_goals_sent': ob.get('goals_sent'), 'observer_publishers_created': ob.get('publishers_created')},
        'all 0', [rel(run, 'launch.log'), rel(run, 'captures', 'capture.json'), ob_path])
    add('H1', 'supplementary: controller reference constant over the capture', 'per run (supplementary)',
        z3['result'], z3['value'], 'change <= 1e-9 rad',
        [rel(run, 'captures', 'capture.json'), rel(run, 'analysis.json')],
        '' if z3['result'] is not None else 'not measured: helper defect (run_01/NOTES.txt)')
    add('H2', 'supplementary: joint states unchanged over the capture', 'per run (supplementary)',
        js.get('max_change_over_capture_rad') is not None and js['max_change_over_capture_rad'] <= 1e-9,
        js.get('max_change_over_capture_rad'), '<= 1e-9 rad', [rel(run, 'captures', 'capture.json')])
    return R


def main():
    masks = {r: scan_masks(r) for r in RUNS}
    req = {r: requirements(r, masks[r]) for r in RUNS}
    json.dump(masks, open(os.path.join(HERE, 'scan_masks.json'), 'w'), indent=1)
    json.dump(req, open(os.path.join(HERE, 'requirements_by_run.json'), 'w'), indent=1, default=str)
    lines = ['| ID | Criterion | Scope | run_01 | run_02 | run_03 |', '|---|---|---|---|---|---|']
    for k in range(len(req[RUNS[0]])):
        a = req[RUNS[0]][k]
        cells = []
        for r in RUNS:
            x = req[r][k]
            v = x['value']
            vs = json.dumps(v, default=str) if not isinstance(v, str) else v
            cells.append(f"**{x['result']}**: {vs[:90]}")
        lines.append(f"| {a['id']} | {a['criterion']} | {a['scope']} | " + ' | '.join(cells) + ' |')
    lines.append('')
    lines.append('Evidence paths per cell are in requirements_by_run.json (relative to '
                 'docs/evidence/m61a_cloud/).')
    open(os.path.join(HERE, 'requirements_by_run.md'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
