#!/usr/bin/env python3
"""Score an S2 run (docs/S2_FIXED_BASE_CRAWL_DESIGN.md section 4, P1-P8) from its evidence.

Usage: analyze_s2.py <run dir> <bag dir> <disabled library.json> <restore report or ''> <out dir>
Writes <out>/summary.json and <out>/tracking.csv. Missing evidence is UNMEASURED, never PASS."""
import csv
import hashlib
import json
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from spiderx_controller import m55_locomotion as loc
from spiderx_controller import m55_rclpy as m55r

run, bag, lib_disabled, restore_report, out = sys.argv[1:6]
os.makedirs(out, exist_ok=True)
SCRIPT = [(0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)]
EXERCISES = [(0, 1, 1), (1, 0, -1), (0, 1, 1), (1, 0, -1), (0, 1, 1)]   # (b_from, b_to, dir)
TRACK_MAX, CONT_MAX = 0.02, 0.005
WELD_XY, WELD_Z, WELD_ROT, MOUNT_Z = 0.001, 0.001, 0.0033, 0.125


def jl(path):
    try:
        with open(path) as f:
            return [json.loads(ln) for ln in f if ln.strip()]
    except OSError:
        return None


def sha(path):
    try:
        with open(path, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return None


ev = jl(os.path.join(run, 'node', 'events.jsonl')) or []
dl = jl(os.path.join(run, 'driver', 'driver.jsonl')) or []
try:
    summary_driver = json.load(open(os.path.join(run, 'driver', 'driver_summary.json')))
except OSError:
    summary_driver = None
sent = [e for e in ev if e.get('event') == 'goal_sent']
results = {e['seq']: e for e in ev if e.get('event') == 'result'}
cancels = [e for e in ev if e.get('event') == 'cancel_sent']
faults = [e for e in ev if e.get('event') == 'fault']
statuses = [(d['mono'], d['status']) for d in dl if 'status' in d]

# ------------------------------------------------------------------ bag
cfg = loc.load_config()
counts, fb_by_goal, fb_order, weld, n_pose, n_js = {}, {}, [], None, 0, 0
q0, max_dq = None, 0.0
bag_ok = os.path.isdir(bag)
if bag_ok:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    counts = {t: 0 for t in types}
    weld = {'max_xy_m': 0.0, 'max_dz_m': 0.0, 'max_rot_rad': 0.0, 'bad_samples': 0}
    while r.has_next():
        topic, data, _ = r.read_next()
        counts[topic] += 1
        if topic.endswith('/_action/feedback'):
            m = deserialize_message(data, get_message(types[topic]))
            gid = bytes(m.goal_id.uuid).hex()
            if gid not in fb_by_goal:
                fb_by_goal[gid] = {'n': 0, 'max': 0.0, 'joint': None}
                fb_order.append(gid)
            g = fb_by_goal[gid]
            g['n'] += 1
            errs = list(m.feedback.error.positions)
            if errs:
                k = max(range(len(errs)), key=lambda i: abs(errs[i]))
                if abs(errs[k]) > g['max'] or not math.isfinite(errs[k]):
                    g['max'], g['joint'] = abs(errs[k]), m.feedback.joint_names[k]
        elif topic == '/spiderx/sim/world_poses':
            m = deserialize_message(data, get_message(types[topic]))
            codes, p = m55r.body_pose_sample(m.transforms, cfg)
            if p is None:
                weld['bad_samples'] += 1
                continue
            n_pose += 1
            weld['max_xy_m'] = max(weld['max_xy_m'], math.hypot(*p['xy']))
            weld['max_dz_m'] = max(weld['max_dz_m'], abs(p['z'] - MOUNT_Z))
            weld['max_rot_rad'] = max(weld['max_rot_rad'], p['tilt'], abs(p['yaw']))
        elif topic == '/joint_states':
            n_js += 1

# ------------------------------------------------------------------ per goal
rows = []
for i, g in enumerate(sent):
    gid = fb_order[i] if i < len(fb_order) else None
    fb = fb_by_goal.get(gid) if gid else None
    res = results.get(g['seq'], {})
    rows.append({'seq': g['seq'], 'level': g['level'], 'direction': g['direction'],
                 'b_from': g['b_from'], 'b_to': g['b_to'], 'kind': g.get('kind'),
                 'duration_s': g['duration_s'],
                 'continuity_error_rad': g.get('continuity_error_rad'),
                 'feedback_samples': None if fb is None else fb['n'],
                 'max_abs_tracking_error_rad': None if fb is None else fb['max'],
                 'joint_at_max': None if fb is None else fb['joint'],
                 'result_status': res.get('status'), 'error_code': res.get('error_code')})
with open(os.path.join(out, 'tracking.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ['seq'])
    w.writeheader()
    w.writerows(rows)


def verdict(ok, measured=True):
    return 'UNMEASURED' if not measured else ('PASS' if ok else 'FAIL')


P = {}
# P1: runs 1-6, 54 goals SUCCEEDED in script order, no fault, each run READY at boundary 0
cyc = rows[:54]
order_ok = len(cyc) == 54 and all(
    {(r['level'], r['direction']) for r in cyc[9 * k:9 * k + 9]} == {SCRIPT[k]} for k in range(6))
succ = all(r['result_status'] == loc.STATUS_SUCCEEDED for r in cyc)
pre_fault = [e for e in faults if e.get('reason') != 'estop']
ready0 = sum(1 for d in dl if d.get('reached', '').endswith('cycle: READY at boundary 0'))
P['P1'] = {'verdict': verdict(order_ok and succ and not pre_fault and ready0 == 6, bool(ev)),
           'goals_in_runs': len(cyc), 'script_order': order_ok, 'all_succeeded': succ,
           'unplanned_faults': [e.get('reason') for e in pre_fault], 'runs_ready_at_b0': ready0}
# P2: tracking <= 0.02 rad on every goal (feedback)
measured_t = bool(rows) and all(r['max_abs_tracking_error_rad'] is not None for r in rows)
mx = max((r['max_abs_tracking_error_rad'] or 0.0) for r in rows) if rows else None
P['P2'] = {'verdict': verdict(measured_t and mx <= TRACK_MAX, measured_t),
           'max_abs_error_rad': mx, 'goals_with_feedback': len(fb_order),
           'goals_sent': len(rows)}
# P3: continuity <= 0.005 rad (measured joints)
conts = [r['continuity_error_rad'] for r in rows if r['continuity_error_rad'] is not None]
cmax = max(conts) if conts else None
P['P3'] = {'verdict': verdict(bool(conts) and len(conts) == len(rows) and cmax <= CONT_MAX,
                              bool(conts)), 'max_continuity_rad': cmax}
# P4: ownership
graphs = [s['monitors'].get('graph') for _, s in statuses if s['monitors'].get('graph')]
foreign = max((g.get('foreign_action_clients', 0) for g in graphs), default=None)
cmdpub = max((g.get('command_topic_publishers', 0) for g in graphs), default=None)
jt = counts.get('/leg_trajectory_controller/joint_trajectory') if bag_ok else None
ex_ok = [(r['b_from'], r['b_to'], r['direction']) for r in rows[54:]] == EXERCISES
P['P4'] = {'verdict': verdict(len(rows) == 59 and len(cancels) == 1 and foreign == 0 and
                              cmdpub == 0 and jt == 0 and ex_ok,
                              bool(ev) and bool(graphs) and bag_ok),
           'goals_sent': len(rows), 'cancels_sent': len(cancels), 'max_foreign_clients': foreign,
           'max_command_topic_publishers': cmdpub, 'joint_trajectory_msgs': jt,
           'exercise_order_ok': ex_ok}


# P5: stop (1.7), lease (1.9), estop
def key_mono(part):
    return next((d['mono'] for d in dl if 'key' in d and part in d.get('why', '')), None)


def window(start, pred):
    before = [x for x in statuses if x[0] <= start]
    after = [x for x in statuses if x[0] >= start and pred(x[1])]
    if not before or not after:
        return None
    s0, s1 = before[-1][1], after[0][1]
    g = s0.get('goal') or {}
    if g.get('duration_s') is None:
        return None
    d_sim = s1['monitors']['sim_time_s'] - s0['monitors']['sim_time_s']
    return {'sim_s': round(d_sim, 3), 'wall_s': round(after[0][0] - start, 3),
            'goal_duration_s': g['duration_s'], 'bound_sim_s': round(g['duration_s'] + 1.0, 3),
            'pass': d_sim <= g['duration_s'] + 1.0}


sp = key_mono('stop exercise: space')
kill = next((d['mono'] for d in dl if d.get('event') == 'teleop_killed'), None)
c17 = None if sp is None else window(sp, lambda s: s['state'] == 'READY')
c19 = None if kill is None else window(kill, lambda s: s['state'] == 'DISARMED')
lease_ok = any(e.get('event') == 'transition' and e.get('why') == loc.LEASE_EXPIRED and
               e.get('state') == loc.DISARMED for e in ev)
last = rows[-1] if rows else {}
estop_ok = (len(cancels) == 1 and [e.get('reason') for e in faults] == ['estop'] and
            last.get('result_status') == loc.STATUS_CANCELED)
p5_meas = c17 is not None and c19 is not None and bool(ev)
P['P5'] = {'verdict': verdict(p5_meas and c17['pass'] and c17['wall_s'] <= 60 and c19['pass'] and
                              c19['wall_s'] <= 120 and lease_ok and estop_ok, p5_meas),
           'check_1_7': c17, 'check_1_9': c19, 'lease_disarmed': lease_ok,
           'estop_one_cancel_last_goal_canceled': estop_ok}
# P6: weld (E8 rule) from the physics-state body pose, every bag sample
P['P6'] = {'verdict': verdict(weld is not None and n_pose > 0 and weld['bad_samples'] == 0 and
                              weld['max_xy_m'] <= WELD_XY and weld['max_dz_m'] <= WELD_Z and
                              weld['max_rot_rad'] <= WELD_ROT, bool(weld) and n_pose > 0),
           'pose_samples': n_pose, **(weld or {})}
# P7: library identity
la, lb = sha(os.path.join(run, 'node', 'library.json')), sha(lib_disabled)
P['P7'] = {'verdict': verdict(la == lb, la is not None and lb is not None),
           'enabled_library_sha256': la, 'disabled_library_sha256': lb}
# P8: restore
rep = open(restore_report).read() if restore_report and os.path.exists(restore_report) else None
P['P8'] = {'verdict': verdict(rep is not None and 'VERDICT: DISABLED AND VERIFIED' in rep,
                              rep is not None), 'report': restore_report or None}

sims = [(m, s['monitors']['sim_time_s']) for m, s in statuses
        if s['monitors'].get('sim_time_s') is not None]
rtf = (sims[-1][1] - sims[0][1]) / (sims[-1][0] - sims[0][0]) if len(sims) > 2 else None
res = {'P': P, 'driver': summary_driver, 'bag_counts': counts, 'joint_state_samples': n_js,
       'rtf_from_status': None if rtf is None else round(rtf, 3),
       'goals_above_0_02_rad': [r['seq'] for r in rows
                                if (r['max_abs_tracking_error_rad'] or 0) > TRACK_MAX],
       'overall': 'PASS' if all(v['verdict'] == 'PASS' for v in P.values()) else
       ('FAIL' if any(v['verdict'] == 'FAIL' for v in P.values()) else 'INCONCLUSIVE')}
with open(os.path.join(out, 'summary.json'), 'w') as f:
    json.dump(res, f, indent=1, sort_keys=True, default=str)
print(json.dumps({k: v['verdict'] for k, v in P.items()}, sort_keys=True), res['overall'])
