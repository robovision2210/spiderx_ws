#!/usr/bin/env python3
"""Per-run pass/fail table against the M6.1-A §10 phase-1 provisional criteria (evidence helper).
    python3 run_analysis.py <run_dir>     -> <run_dir>/analysis.json and a printed table
"""
import glob
import json
import math
import os
import re
import sys


def load(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def main(run):
    rows, facts = [], {}

    def row(cid, name, ok, value, crit):
        rows.append({'id': cid, 'criterion': name, 'result': ok, 'value': value,
                     'threshold': crit})
    pre = sorted(glob.glob(f'{run}/observer/*/observation.json'))
    obs = [json.load(open(p)) for p in pre]
    pf = [o for o in obs if o.get('mode') == 'preflight']
    ob = [o for o in obs if o.get('mode') == 'observe']
    row('1', 'preflight READY', bool(pf) and all(o['ready'] for o in pf),
        [o.get('failure_codes') for o in pf], 'READY, exit 0')
    o = ob[0] if ob else {}
    st = o.get('statistics', {})
    row('1b', '120 s observation READY at its end', bool(ob) and o.get('ready'),
        o.get('failure_codes'), 'READY, exit 0')
    body, att = st.get('body', {}), st.get('attachment', {})
    zm = body.get('z_mean_m')
    row('2a', 'body z mean within 1 mm of 0.125 m',
        zm is not None and abs(zm - 0.125) <= 0.001, zm, '|z - 0.125| <= 0.001 m')
    row('2a', 'body z range <= 1 mm', body.get('z_range_m') is not None and
        body['z_range_m'] <= 0.001, body.get('z_range_m'), '<= 0.001 m')
    row('2b', 'attachment max translation <= 1 mm', att.get('max_translation_m') is not None
        and att['max_translation_m'] <= 0.001, att.get('max_translation_m'), '<= 0.001 m')
    row('2b', 'attachment max rotation <= 0.0033 rad', att.get('max_rotation_rad') is not None
        and att['max_rotation_rad'] <= 0.0033, att.get('max_rotation_rad'), '<= 0.0033 rad')
    row('2c', 'spawn deviation <= 1 mm', att.get('max_spawn_deviation_m') is not None and
        att['max_spawn_deviation_m'] <= 0.001, att.get('max_spawn_deviation_m'), '<= 0.001 m')
    row('2d', 'link deviation <= 0.1 mm', att.get('max_link_deviation_m') is not None and
        att['max_link_deviation_m'] <= 0.0001, att.get('max_link_deviation_m'), '<= 0.0001 m')
    row('2e', 'tilt <= 0.0033 rad', body.get('tilt_max_rad') is not None and
        body['tilt_max_rad'] <= 0.0033, body.get('tilt_max_rad'), '<= 0.0033 rad')
    pg = st.get('pose_receipt', {})
    row('3', 'pose receipt max gap < 1.0 s (wall)', pg.get('max_s') is not None and
        pg['max_s'] < 1.0, {k: pg.get(k) for k in ('count', 'mean_s', 'p99_s', 'p999_s',
                                                    'max_s')}, '< 1.0 s')
    jg = st.get('joint_state_sim_gap_max_s')
    row('3', 'joint-state sim gap max <= 0.25 s', jg is not None and jg <= 0.25, jg,
        '<= 0.25 s (sim)')
    codes = st.get('sample_codes', {})
    row('4', 'no unexpected pose sample codes', not codes, codes, 'none')
    facts['header_frame_ids'] = st.get('header_frame_ids')
    facts['clock'] = st.get('clock')
    facts['joint_state_receipt'] = st.get('joint_state_receipt')
    facts['body'] = body
    facts['attachment'] = att
    facts['observer_checks_end'] = {k: v for k, v in (o.get('checks') or {}).items()}
    facts['observer_graph'] = o.get('graph')
    facts['observer_controllers'] = o.get('controllers')
    # ownership
    ai = load(f'{run}/captures/action_info.txt') or ''
    ti = load(f'{run}/captures/command_topic_info.txt') or ''
    m_ac = re.search(r'Action clients:\s*(\d+)', ai)
    m_pc = re.search(r'Publisher count:\s*(\d+)', ti)
    g_end = (o.get('graph') or {}).get('end') or {}
    g_start = (o.get('graph') or {}).get('start') or {}
    own = {'ros2_action_info_clients': int(m_ac.group(1)) if m_ac else None,
           'ros2_topic_info_publishers': int(m_pc.group(1)) if m_pc else None,
           'observer_start': {k: g_start.get(k) for k in ('command_publishers',
                                                          'action_clients')},
           'observer_end': {k: g_end.get(k) for k in ('command_publishers', 'action_clients')}}
    row('7', 'command ownership: no other commander visible',
        own['ros2_action_info_clients'] == 0 and own['ros2_topic_info_publishers'] == 0 and
        all(v == 0 for d in (own['observer_start'], own['observer_end']) for v in d.values()),
        own, 'all 0')
    # mount from the running model
    cap = {}
    if os.path.exists(f'{run}/captures/capture.json'):
        cap = json.load(open(f'{run}/captures/capture.json'))
    weld = cap.get('weld_joint')
    row('M', 'running /robot_description weld = world->dummy_link (0, 0, 0.125, rpy 0)',
        bool(weld) and weld['type'] == 'fixed' and weld['parent'] == 'world' and
        weld['child'] == 'dummy_link' and
        all(abs(float(a) - b) < 1e-9 for a, b in zip(weld['xyz'].split(), (0, 0, 0.125))) and
        all(abs(float(a)) < 1e-9 for a in weld['rpy'].split()), weld, 'fixed, 0 0 0.125, 0 0 0')
    js = cap.get('joint_states', [])
    if js:
        names = js[0]['name']
        first = dict(zip(names, js[0]['position']))
        mx = max(abs(v) for s in js for v in s['position'])
        drift = max(abs(dict(zip(s['name'], s['position']))[n] - first[n])
                    for s in js for n in names)
        facts['joint_states_capture'] = {'samples': len(js), 'joints': len(names),
                                         'max_abs_position_rad': mx,
                                         'max_change_over_capture_rad': drift,
                                         'max_abs_velocity': max(abs(v) for s in js
                                                                 for v in s['velocity'])
                                         if js[0]['velocity'] else None}
    poses = cap.get('poses', [])
    if poses:
        last = {t['child']: t for t in poses[-1]['transforms']}
        facts['pose_entries_last'] = last
    lg = load(f'{run}/launch.log') or ''
    goal_lines = [ln for ln in lg.splitlines() if re.search(
        r'(Received new action goal|Accepted new action goal|new trajectory|Goal)', ln)]
    facts['launch_log_goal_lines'] = goal_lines[:20]
    facts['launch_log_errors'] = [ln for ln in lg.splitlines()
                                  if re.search(r'\[ERROR\]|\[FATAL\]|Traceback', ln)][:30]
    facts['launch_log_warnings'] = [ln for ln in lg.splitlines() if '[WARN' in ln][:30]
    row('Z', 'no trajectory goal or topic command reached the controller (launch log)',
        not goal_lines, len(goal_lines), '0 goal lines')
    sc = load(f'{run}/captures/scan_check.json')
    if sc:
        s = json.loads(sc)
        med = {k: v['median_residual_m'] for k, v in s['surfaces'].items()}
        d = s['fit_minus_expected_m_m_rad']
        row('6', '/scan: per-surface median residual within +-8 mm (supplementary)',
            all(abs(v) <= 0.008 for v in med.values()), med, '|median| <= 0.008 m')
        row('6', '/scan: fitted lidar planar pose within 8 mm of the weld-implied pose',
            math.hypot(d[0], d[1]) <= 0.008, d, '|dxy| <= 0.008 m (yaw reported)')
        facts['scan_check'] = s
    sd = load(f'{run}/shutdown.txt') or ''
    row('8', 'clean shutdown: launch group empty, nothing left',
        'process group empty' in sd and 'LEFTOVER' not in sd and
        sd.strip().endswith('none'), sd.strip().splitlines()[-1] if sd else None,
        'no leftover process')
    out = {'run': os.path.basename(run), 'rows': rows, 'facts': facts}
    json.dump(out, open(f'{run}/analysis.json', 'w'), indent=1, default=str)
    for r in rows:
        v = r['value']
        vs = json.dumps(v, default=str) if not isinstance(v, float) else f'{v:.6g}'
        print(f"{'PASS' if r['result'] else 'FAIL':4s} {r['id']:3s} {r['criterion']:70s} "
              f"{vs[:110]}")


if __name__ == '__main__':
    main(sys.argv[1].rstrip('/'))
