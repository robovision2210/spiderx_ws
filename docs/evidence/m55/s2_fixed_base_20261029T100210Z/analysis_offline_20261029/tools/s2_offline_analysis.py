#!/usr/bin/env python3
"""Offline analysis of the 2026-10-29 S2 attempt (goal 1). Reads the original bag and logs; writes
nothing next to them. No ROS graph, no launch, no motion.

    s2_offline_analysis.py --bag <bag_0.db3> --run <run dir> --goal <goal1_rebuilt.json> --out <dir>

The bag is opened through SQLite in read-only, immutable mode (no journal or lock files) and its
SHA-256 is recorded before and after reading. Clocks are kept apart:
  sim      header stamps of /joint_states, the action feedback and the derived per-update series;
  mono     CLOCK_MONOTONIC of the node (events.jsonl "wall") and the driver (driver.jsonl "mono");
  realtime the bag recorder's receipt time and the controller process's launch.log stamps.
"""
import argparse
import csv
import hashlib
import json
import math
import os
import sqlite3

import numpy as np

NS = 1_000_000_000
UPDATE_NS = 10_000_000            # controller_manager update period (runtime log: 0.01 s)
LIMIT_RAD = 0.02                  # S2 stop condition and acceptance P2
FB_TOPIC = '/leg_trajectory_controller/follow_joint_trajectory/_action/feedback'
CS_TOPIC = '/leg_trajectory_controller/controller_state'
JS_TOPIC = '/joint_states'
ST_TOPIC = '/spiderx/locomotion/status'


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def stamp_ns(header):
    return header.stamp.sec * NS + header.stamp.nanosec


def load_bag(path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    con = sqlite3.connect(f'file:{path}?mode=ro&immutable=1', uri=True)
    topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
    types = {tid: get_message(t) for tid, (_, t) in topics.items()}
    out = {name: [] for name, _ in topics.values()}
    for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages '
                                     'ORDER BY timestamp, id'):
        out[topics[tid][0]].append((ts, deserialize_message(data, types[tid])))
    con.close()
    return out


# --------------------------------------------------------------------- JTC reference sampling
def _powers(n, x):
    p = [1.0]
    for _ in range(n):
        p.append(p[-1] * x)
    return p


def _interpolate(ta, pa, va, tb, pb, vb, ts):
    """joint_trajectory_controller Trajectory::interpolate_between_points for points with
    positions and velocities (cubic), with its clamping to linear outside [ta, tb]."""
    so_far, btwn, cubic = (ts - ta) / NS, (tb - ta) / NS, True
    if so_far < 0.0:
        so_far, cubic = 0.0, False
    if so_far > btwn:
        so_far, cubic = btwn, False
    t = _powers(5, so_far)
    n = len(pa)
    pos, vel = np.zeros(n), np.zeros(n)
    if not cubic:
        for i in range(n):
            c1 = (pb[i] - pa[i]) / btwn if btwn != 0.0 else 0.0
            pos[i] = t[0] * pa[i] + t[1] * c1
            vel[i] = t[0] * c1
        return pos, vel
    T = _powers(3, btwn)
    for i in range(n):
        sp, sv, ep, ev = pa[i], va[i], pb[i], vb[i]
        c2 = (-3.0 * sp + 3.0 * ep - 2.0 * sv * T[1] - ev * T[1]) / T[2]
        c3 = (2.0 * sp - 2.0 * ep + sv * T[1] + ev * T[1]) / T[3]
        pos[i] = t[0] * sp + t[1] * sv + t[2] * c2 + t[3] * c3
        vel[i] = t[0] * sv + t[1] * 2.0 * c2 + t[2] * 3.0 * c3
    return pos, vel


class Reference:
    """The 'splines' reference of a goal whose header stamp is 0: started at the update t0 from
    the measured state (x0, v0), a cubic lead-in to the first point, cubic segments between the
    points, then the last point held."""

    def __init__(self, goal, t0, x0, v0):
        self.t0, self.x0, self.v0 = t0, np.asarray(x0, float), np.asarray(v0, float)
        self.pts = [(int(round(p['time_from_start_s'] * NS)), np.array(p['positions'], float),
                     np.array(p['velocities'], float)) for p in goal['points']]

    def __call__(self, ts):
        if ts < self.t0:
            return self.x0.copy(), np.zeros(len(self.x0))
        first = self.t0 + self.pts[0][0]
        if ts < first:
            return _interpolate(self.t0, self.x0, self.v0, first, self.pts[0][1], self.pts[0][2], ts)
        for (ta, pa, va), (tb, pb, vb) in zip(self.pts, self.pts[1:]):
            if self.t0 + ta <= ts < self.t0 + tb:
                return _interpolate(self.t0 + ta, pa, va, self.t0 + tb, pb, vb, ts)
        return self.pts[-1][1].copy(), np.zeros(len(self.x0))


# --------------------------------------------------------------------------------- fitting
def golden(f, a, b, tol=1e-12, it=200):
    g = (math.sqrt(5.0) - 1.0) / 2.0
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(it):
        if abs(b - a) < tol:
            break
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a)
            fd = f(d)
    return (a + b) / 2.0


def stats(r):
    r = np.asarray(r, float).ravel()
    return {'n': int(r.size), 'rms_rad': float(np.sqrt(np.mean(r ** 2))),
            'max_abs_rad': float(np.max(np.abs(r)))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--run', required=True)
    ap.add_argument('--goal', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--worst-joint', default=None)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    R = {'inputs': {}}

    # ---------------------------------------------------------------- inputs, read-only
    bag_sha_before = sha256(a.bag)
    data = load_bag(a.bag)
    bag_sha_after = sha256(a.bag)
    run_files = ['launch.log', 'node/events.jsonl', 'node/library.json',
                 'driver/driver.jsonl', 'driver/driver_summary.json']
    R['inputs'] = {
        'bag': {'path': a.bag, 'sha256_before': bag_sha_before, 'sha256_after': bag_sha_after,
                'unchanged': bag_sha_before == bag_sha_after,
                'open_mode': 'sqlite3 URI mode=ro&immutable=1',
                'messages': {k: len(v) for k, v in data.items()}},
        'run_files': {f: sha256(os.path.join(a.run, f)) for f in run_files},
        'goal': {'path': os.path.basename(a.goal), 'sha256': sha256(a.goal)}}
    goal = json.load(open(a.goal))
    events = [json.loads(line) for line in open(os.path.join(a.run, 'node/events.jsonl'))]
    driver = [json.loads(line) for line in open(os.path.join(a.run, 'driver/driver.jsonl'))]
    library = json.load(open(os.path.join(a.run, 'node/library.json')))
    sent = [e for e in events if e['event'] == 'goal_sent']
    assert len(sent) == 1 and sent[0]['fingerprint'] == goal['fingerprint'], 'goal 1 mismatch'
    CN = list(goal['joint_names'])

    js, cs, fb = data[JS_TOPIC], data[CS_TOPIC], data[FB_TOPIC]
    JS = {}
    for rt, m in js:
        idx = [list(m.name).index(n) for n in CN]
        JS[stamp_ns(m.header)] = (rt, np.array(m.position)[idx], np.array(m.velocity)[idx])
    js_t = np.array(sorted(JS))
    js_rt = np.array([JS[t][0] for t in js_t])

    # ---------------------------------------------------------------- field and order checks
    checks = {}
    checks['joint_order'] = {
        'goal_and_library': list(library['library']['joint_names']) == CN,
        'action_feedback': all(list(m.feedback.joint_names) == CN for _, m in fb),
        'controller_state': all(list(m.joint_names) == CN for _, m in cs),
        'joint_states_order': list(js[0][1].name),
        'joint_states_distinct_orders': len({tuple(m.name) for _, m in js}),
        'note': '/joint_states uses the broadcaster order and is mapped by name everywhere here'}
    fb_rows = []
    for rt, m in fb:
        f = m.feedback
        s = stamp_ns(f.header)
        des, act, err = (np.array(f.desired.positions), np.array(f.actual.positions),
                         np.array(f.error.positions))
        fb_rows.append({'t': s, 'rt': rt, 'des': des, 'act': act, 'err': err,
                        'dvel': np.array(f.desired.velocities),
                        'avel': np.array(f.actual.velocities),
                        'err_identity': float(np.max(np.abs(err - (des - act)))),
                        'act_vs_js': float(np.max(np.abs(act - JS[s][1]))) if s in JS else None,
                        'avel_vs_js': float(np.max(np.abs(np.array(f.actual.velocities)
                                                          - JS[s][2]))) if s in JS else None})
    checks['feedback'] = {
        'samples': len(fb_rows), 'goal_ids': sorted({bytes(m.goal_id.uuid).hex() for _, m in fb}),
        'stamps_on_update_grid': all(r['t'] % UPDATE_NS == 0 for r in fb_rows),
        'stamps_found_in_joint_states': sum(r['t'] in JS for r in fb_rows),
        'max_abs_error_minus_desired_minus_actual': max(r['err_identity'] for r in fb_rows),
        'max_abs_actual_minus_joint_states_same_stamp': max(r['act_vs_js'] for r in fb_rows),
        'max_abs_actual_velocity_minus_joint_states': max(r['avel_vs_js'] for r in fb_rows)}
    cs_rows, unmatched, ambiguous = [], 0, 0
    for rt, m in cs:
        s = stamp_ns(m.header)
        fpos = np.array(m.feedback.positions)
        cand = [int(t) for t in js_t if -20 * UPDATE_NS <= t - s <= 20 * UPDATE_NS
                and np.array_equal(JS[int(t)][1], fpos)]
        row = {'stamp': s, 'rt': rt, 'ref': np.array(m.reference.positions),
               'rvel': np.array(m.reference.velocities), 'fbk': fpos,
               'err': np.array(m.error.positions), 'out': np.array(m.output.positions),
               'update': cand[0] if len(cand) == 1 else None, 'n_match': len(cand)}
        unmatched += not cand
        ambiguous += len(cand) > 1
        cs_rows.append(row)
    lag = [(r['stamp'] - r['update']) / 1e6 for r in cs_rows if r['update'] is not None]
    checks['controller_state'] = {
        'messages': len(cs_rows), 'feedback_equals_one_joint_states_sample': len(lag),
        'ambiguous_rest_matches': ambiguous, 'unmatched': unmatched,
        'header_stamp_minus_update_time_ms': {
            'min': min(lag), 'max': max(lag),
            'counts': {str(k): lag.count(k) for k in sorted(set(lag))}},
        'max_abs_error_minus_reference_minus_feedback': float(max(
            np.max(np.abs(r['err'] - (r['ref'] - r['fbk']))) for r in cs_rows)),
        'max_abs_output_minus_reference': float(max(
            np.max(np.abs(r['out'] - r['ref'])) for r in cs_rows)),
        'note': 'the header stamp is not the update time; the update time is identified by '
                'exact equality of the feedback block with one /joint_states sample'}
    checks['joint_states'] = {
        'messages': len(js), 'first_stamp_s': js_t[0] / NS, 'last_stamp_s': js_t[-1] / NS,
        'every_update_present': bool(np.all(np.diff(js_t) == UPDATE_NS))}
    R['checks'] = checks

    # ------------------------------------------------- reference reconstruction and start time
    obs = [(r['t'], 'feedback', r['des'], r['dvel']) for r in fb_rows]
    obs += [(r['update'], 'controller_state', r['ref'], r['rvel'])
            for r in cs_rows if r['update'] is not None]
    obs.sort(key=lambda o: (o[0], o[1]))
    fb_first, fb_last = fb_rows[0]['t'], fb_rows[-1]['t']
    scan = []
    for t0 in [int(t) for t in js_t if fb_first - 30 * UPDATE_NS <= t <= fb_first]:
        ref = Reference(goal, t0, JS[t0][1], JS[t0][2])
        win = [o for o in obs if t0 <= o[0] <= fb_last]
        scan.append((max(float(np.max(np.abs(ref(o[0])[0] - o[2]))) for o in win), t0, len(win)))
    scan.sort()
    t_start = scan[0][1]
    ref = Reference(goal, t_start, JS[t_start][1], JS[t_start][2])
    after = [o for o in obs if o[0] >= t_start]
    resid = [(o[0], o[1], float(np.max(np.abs(ref(o[0])[0] - o[2]))),
              float(np.max(np.abs(ref(o[0])[1] - o[3])))) for o in after]
    departs = [x for x in resid if x[2] > 1e-9]
    t_depart = departs[0][0]
    matching = [x for x in resid if x[0] < t_depart]
    t_last_match = matching[-1][0]
    hold_obs = [o for o in after if o[0] >= t_depart]
    hold = hold_obs[0][2]
    hold_from = [int(t) for t in js_t if np.array_equal(JS[int(t)][1], hold)]
    pre_goal_ref = [o for o in obs if o[0] < t_start and o[0] >= t_start - 10 * UPDATE_NS]
    R['reference'] = {
        'interpolation': "JTC 'splines': cubic Hermite on positions + velocities; lead-in cubic "
                         'from the measured state at the first update; header stamp 0 = start '
                         'at that update',
        'goal_fingerprint': goal['fingerprint'], 'points': len(goal['points']),
        'first_point_s': goal['points'][0]['time_from_start_s'],
        'last_point_s': goal['points'][-1]['time_from_start_s'],
        'start_scan_best': [{'max_abs_residual_rad': s[0], 't_start_s': s[1] / NS,
                             'observed_refs': s[2]} for s in scan[:3]],
        'trajectory_start_sim_s': t_start / NS,
        'observed_refs_matched': len(matching),
        'observed_refs_matched_by_source': {
            k: sum(1 for x in matching if x[1] == k) for k in ('feedback', 'controller_state')},
        'max_abs_position_residual_rad': max(x[2] for x in matching),
        'max_abs_velocity_residual_rad_s': max(x[3] for x in matching),
        'last_matching_observation_sim_s': t_last_match / NS,
        'first_departing_observation_sim_s': t_depart / NS,
        'hold_reference_equals_joint_states_at_sim_s': [t / NS for t in hold_from],
        'pre_goal_reference_minus_start_state_rad': [
            float(np.max(np.abs(o[2] - JS[t_start][1]))) for o in pre_goal_ref]}

    # ---------------------------------------------------------------- plant law (per update)
    t_hold = None
    t_end = js_t[-1] - UPDATE_NS

    def commanded(t, hold_onset):
        return ref(t)[0] if t < hold_onset else hold

    # hold onset: the first update whose command (inferred from the next step, if the law
    # holds) equals the hold rather than the trajectory; checked against the observations
    ks = [int(t) for t in js_t if t_start <= t <= t_end]
    law_rows = []
    for t in ks:
        x0, x1, v1 = JS[t][1], JS[t + UPDATE_NS][1], JS[t + UPDATE_NS][2]
        law_rows.append((t, x0, x1, v1))
    G_NOM = 0.1
    inferred = {t: x0 + (x1 - x0) / G_NOM for t, x0, x1, _ in law_rows}
    for t, *_ in law_rows:
        if t >= t_last_match and np.max(np.abs(inferred[t] - hold)) < 1e-9 and \
                np.max(np.abs(inferred[t] - ref(t)[0])) > 1e-6:
            t_hold = t
            break
    assert t_hold == t_depart, (t_hold, t_depart)
    W_end = 60 * UPDATE_NS                     # analysed hold window: 0.6 s after the onset

    def window(lo, hi):
        rows = [r for r in law_rows if lo <= r[0] < hi]
        E = np.array([commanded(t, t_hold) - x0 for t, x0, _, _ in rows])
        DX = np.array([x1 - x0 for _, x0, x1, _ in rows])
        V1 = np.array([v1 for *_, v1 in rows])
        EP = np.array([commanded(t - UPDATE_NS, t_hold) - JS[t - UPDATE_NS][1]
                       for t, *_ in rows])                       # one update older command
        EN = np.array([commanded(t + UPDATE_NS, t_hold) - x0 for t, x0, _, _ in rows])
        return rows, E, DX, V1, EP, EN

    law = {}
    for name, lo, hi in (('pre_stop_tracking', t_start, t_hold),
                         ('hold', t_hold, t_hold + W_end)):
        rows, E, DX, V1, EP, EN = window(lo, hi)
        e, dx, v1 = E.ravel(), DX.ravel(), V1.ravel()
        g = float(np.dot(e, dx) / np.dot(e, e))
        k = float(np.dot(e, v1) / np.dot(e, e))
        law[name] = {
            'updates': len(rows), 'joint_steps': int(e.size),
            'window_sim_s': [lo / NS, (rows[-1][0] + UPDATE_NS) / NS],
            'model': 'x[k+1] - x[k] = g (c[k] - x[k]) per 10 ms update',
            'g_pooled': g, 'g_per_joint': {
                n: float(np.dot(E[:, j], DX[:, j]) / np.dot(E[:, j], E[:, j]))
                for j, n in enumerate(CN)},
            'residual': stats(dx - g * e), 'residual_at_g_0.1': stats(dx - 0.1 * e),
            'velocity_law_k_per_s': k, 'velocity_law_residual_rad_s': stats(v1 - k * e),
            'dx_minus_dt_times_v_next': stats(dx - 0.01 * v1)}
        if name == 'pre_stop_tracking':
            # update-ordering variants (hypothesis B), on the updates whose neighbours are
            # also trajectory updates, so that no variant straddles the start or the hold
            inner = np.array([t_start + UPDATE_NS <= r[0] and r[0] + UPDATE_NS < t_hold
                              for r in rows])
            d_in = DX[inner].ravel()
            var = {}
            for vname, M in (('command_as_computed_this_update', E),
                             ('command_one_update_older', EP),
                             ('reference_one_update_ahead', EN)):
                m = M[inner].ravel()
                gv = float(np.dot(m, d_in) / np.dot(m, m))
                var[vname] = {'g': gv, 'residual': stats(d_in - gv * m)}
            law[name]['ordering_variants'] = {
                'updates': int(inner.sum()),
                'window_sim_s': [(t_start + UPDATE_NS) / NS, (t_hold - UPDATE_NS) / NS],
                'variants': var}
    R['plant_law'] = law

    # derived per-update error, desired velocity, segments
    upd = [int(t) for t in js_t if t_start - 5 * UPDATE_NS <= t <= t_hold + W_end]
    series = []
    for t in upd:
        c = commanded(t, t_hold) if t >= t_start else JS[t_start][1]
        cv = ref(t)[1] if t_start <= t < t_hold else np.zeros(len(CN))
        seg = ('before_goal' if t < t_start else 'pre_stop_tracking' if t < t_hold else 'hold')
        series.append((t, seg, c, JS[t][1], c - JS[t][1], cv, JS[t][2], ref(t)[0]))
    worst_pre = {}
    for j, n in enumerate(CN):
        e = [(abs(s[4][j]), s[0], s[4][j]) for s in series if s[1] == 'pre_stop_tracking']
        h = [(abs(s[4][j]), s[0], s[4][j]) for s in series if s[1] == 'hold']
        cross = [s[0] for s in series if s[1] == 'pre_stop_tracking' and abs(s[4][j]) > LIMIT_RAD]
        vmax = max(abs(s[5][j]) for s in series if s[1] == 'pre_stop_tracking')
        worst_pre[n] = {'max_abs_error_rad': max(e)[0], 'at_sim_s': max(e)[1] / NS,
                        'signed': max(e)[2], 'first_above_limit_sim_s':
                        cross[0] / NS if cross else None,
                        'max_abs_desired_velocity_rad_s': vmax,
                        'hold_max_abs_error_rad': max(h)[0], 'hold_max_at_sim_s': max(h)[1] / NS}
    worst = a.worst_joint or max(CN, key=lambda n: worst_pre[n]['max_abs_error_rad'])
    wj = CN.index(worst)
    first_derived = min(((v['first_above_limit_sim_s'], n) for n, v in worst_pre.items()
                         if v['first_above_limit_sim_s'] is not None))
    first_derived_err = float(next(s[4][CN.index(first_derived[1])] for s in series
                                   if s[0] == round(first_derived[0] * NS)))
    ties = {}
    for j, n in enumerate(CN):
        if n != worst:
            dmax = max(abs(s[4][j] - s[4][wj]) for s in series if s[1] == 'pre_stop_tracking')
            if dmax < 1e-9:
                ties[n] = dmax
    fb_cross = [(r['t'], CN[int(np.argmax(np.abs(r['err'])))], float(r['err'][np.argmax(np.abs(r['err']))]))
                for r in fb_rows if np.max(np.abs(r['err'])) > LIMIT_RAD]
    cs_cross = [(r['update'], CN[int(np.argmax(np.abs(r['err'])))],
                 float(r['err'][np.argmax(np.abs(r['err']))]))
                for r in cs_rows if r['update'] is not None and t_start <= r['update'] < t_hold
                and np.max(np.abs(r['err'])) > LIMIT_RAD]
    fb_max = max(fb_rows, key=lambda r: np.max(np.abs(r['err'])))
    R['tracking'] = {
        'worst_joint': worst, 'per_joint_pre_stop': worst_pre,
        'joints_tied_with_worst_max_abs_difference_rad': ties,
        'first_crossing_derived_per_update': {'sim_s': first_derived[0], 'joint': first_derived[1],
                                              'error_rad': first_derived_err},
        'first_crossing_recorded_feedback': {'sim_s': fb_cross[0][0] / NS, 'joint': fb_cross[0][1],
                                             'error_rad': fb_cross[0][2]} if fb_cross else None,
        'first_crossing_recorded_controller_state': {
            'update_sim_s': cs_cross[0][0] / NS, 'joint': cs_cross[0][1],
            'error_rad': cs_cross[0][2]} if cs_cross else None,
        'recorded_feedback_max': {'sim_s': fb_max['t'] / NS,
                                  'joint': CN[int(np.argmax(np.abs(fb_max['err'])))],
                                  'error_rad': float(fb_max['err'][np.argmax(np.abs(fb_max['err']))])},
        'pre_stop_max_abs_error_rad_all_joints': max(v['max_abs_error_rad'] for v in worst_pre.values()),
        'hold_max_abs_error_rad_all_joints': max(v['hold_max_abs_error_rad'] for v in worst_pre.values())}

    # apparent lag e/v of the worst joint (descriptive only; not a delay estimate)
    app = [(s[0] / NS, s[4][wj] / s[5][wj]) for s in series
           if s[1] == 'pre_stop_tracking' and abs(s[5][wj]) > 0.05]
    R['tracking']['apparent_lag_worst_joint_s'] = {
        'definition': 'error / desired velocity at the same update, |desired velocity| > 0.05 rad/s',
        'first': app[0], 'last': app[-1], 'min': min(x[1] for x in app), 'max': max(x[1] for x in app)}

    # ------------------------------------------------------------------- model comparison
    win = [s for s in series if s[1] == 'pre_stop_tracking']
    tw = np.array([s[0] for s in win])
    X = np.array([s[3] for s in win])
    x_start = JS[t_start][1]
    c_upd = np.array([s[2] for s in win])

    def discrete(g):
        x = x_start.copy()
        out = [x.copy()]
        for c in c_upd[:-1]:
            x = x + g * (c - x)
            out.append(x.copy())
        return np.array(out)

    fine = 250_000                                              # 0.25 ms
    tf = np.arange(t_start - 50 * UPDATE_NS, tw[-1] + 1, fine)
    cf = np.array([ref(int(t))[0] for t in tf])
    idx = np.searchsorted(tf, tw)

    def first_order(tau):
        x = x_start.copy()
        out = np.empty((len(tf), len(CN)))
        i0 = int(np.searchsorted(tf, t_start))
        out[:i0 + 1] = x
        h = fine / NS
        k = math.exp(-h / tau)
        for i in range(i0, len(tf) - 1):
            c0, c1 = cf[i], cf[i + 1]
            m = (c1 - c0) / h
            x = c1 - tau * m + (x - c0 + tau * m) * k        # exact for piecewise-linear c
            out[i + 1] = x
        return out[idx]

    def delayed(d):
        return np.array([ref(int(round(t - d * NS)))[0] for t in tw])

    def rms(P, cols=slice(None)):
        return float(np.sqrt(np.mean((P[:, cols] - X[:, cols]) ** 2)))

    fits = []
    for label, cols in (('all_joints', slice(None)), (worst, slice(wj, wj + 1))):
        g = golden(lambda g: rms(discrete(g), cols), 0.05, 0.2, tol=1e-13)
        tau = golden(lambda tau: rms(first_order(tau), cols), 0.03, 0.2, tol=1e-7)
        d = golden(lambda d: rms(delayed(d), cols), 0.0, 0.2, tol=1e-7)
        for model, par, P in (('discrete first-order, 10 ms updates (free run)', ('g', g), discrete(g)),
                              ('continuous first-order (free run)', ('tau_s', tau), first_order(tau)),
                              ('pure delay', ('delay_s', d), delayed(d))):
            r = (P[:, cols] - X[:, cols]).ravel()
            fits.append({'joints': label, 'model': model, 'parameter': par[0], 'value': par[1],
                         **stats(r)})
    R['model_fits'] = {
        'window_sim_s': [tw[0] / NS, tw[-1] / NS], 'samples_per_joint': len(tw),
        'data': '/joint_states positions at every update vs the reconstructed reference',
        'method': 'one parameter each, least squares (golden-section search on the RMS '
                  'position residual); free-run simulation from the measured state at the '
                  'trajectory start; the continuous model integrates the reconstructed spline '
                  'exactly on a 0.25 ms piecewise-linear grid',
        'fits': fits}

    # -------------------------------------------------------------------- clocks and timeline
    bst = [(rt, json.loads(m.data)) for rt, m in data[ST_TOPIC]]
    dst = [(e['mono'], e['status']) for e in driver if 'status' in e]
    pairs = [(mono, [rt for rt, b in bst if b == s]) for mono, s in dst]
    offs = np.array([h[0] / NS - mono for mono, h in pairs if len(h) == 1])
    off = float(np.median(offs))
    sent_wall = sent[0]['wall']
    pub = []
    for mono, s in dst:
        if s.get('goal'):
            p = sent_wall + s['goal']['elapsed_s']
            rx = [rt for rt, b in bst if b == s][0]
            pub.append({'node_publish_mono_s': p, 'driver_receipt_mono_s': mono,
                        'bag_receipt_realtime_s': rx / NS,
                        'node_sim_time_s': s['monitors']['sim_time_s'],
                        'node_joint_states_age_s': s['monitors']['joint_states_age_s'],
                        'max_tracking_error_rad': s['goal']['max_tracking_error_rad']})
    i0, i1 = int(np.searchsorted(js_t, t_start)), int(np.searchsorted(js_t, t_hold))
    R['clocks'] = {
        'mono_to_realtime_offset_s': off,
        'offset_method': 'median over the driver statuses of (bag receipt realtime - driver '
                         'receipt mono), matched by identical content',
        'offset_samples': len(offs),
        'offset_spread_ms': float((offs.max() - offs.min()) * 1e3),
        'bag_receipt_minus_node_publish_s': [x['bag_receipt_realtime_s'] - x['node_publish_mono_s']
                                             for x in pub],
        'goal_statuses': pub,
        'joint_states_receipts_in_stamp_order': bool(np.all(np.diff(js_rt) > 0)),
        'local_rtf_trajectory_start_to_hold': float((js_t[i1] - js_t[i0]) / (js_rt[i1] - js_rt[i0]))}
    assert R['clocks']['joint_states_receipts_in_stamp_order']

    def bracket(rt_s):
        i = int(np.searchsorted(js_rt, rt_s * NS, side='right'))
        return (js_t[i - 1] / NS, js_t[i] / NS, (rt_s * NS - js_rt[i - 1]) / 1e6,
                (js_rt[i] - rt_s * NS) / 1e6)

    def ev(name, wall=None, realtime=None, source='', exact=None, note=''):
        rt_s = realtime if realtime is not None else (wall + off if wall is not None else None)
        lo = hi = before = after_ = None
        if rt_s is not None:
            lo, hi, before, after_ = bracket(rt_s)
        return {'event': name, 'source': source,
                'mono_s': wall, 'realtime_s': rt_s,
                'realtime_is': 'recorded' if realtime is not None else
                ('mapped from mono' if wall is not None else None),
                'sim_exact_s': exact, 'sim_bracket_s': [lo, hi] if lo is not None else None,
                'js_receipt_before_ms': before, 'js_receipt_after_ms': after_, 'note': note}

    def node_wall(kind, **kw):
        return [e['wall'] for e in events if e['event'] == kind
                and all(e.get(k) == v for k, v in kw.items())][0]

    launch = open(os.path.join(a.run, 'launch.log')).read().splitlines()

    def log_rt(text):
        line = [ln for ln in launch if text in ln][0]
        return float(line.split('] [')[2])        # [proc] [LEVEL] [realtime] [logger]: text

    stop_status = [x for x in pub if x['max_tracking_error_rad'] > LIMIT_RAD][0]
    xkey = [e for e in driver if e.get('key') == "'x'"][0]['mono']
    fb_last_row = fb_rows[-1]
    stop_fb = [r for r in fb_rows if abs(float(np.max(np.abs(r['err']))) -
                                         stop_status['max_tracking_error_rad']) == 0.0]
    T = [
        ev('node: goal 1 sent', wall=sent_wall, source='node/events.jsonl goal_sent'),
        ev('controller: Received / Accepted new action goal',
           realtime=log_rt('Received new action goal'), source='run/launch.log'),
        ev('controller: trajectory start (first update sampling goal 1)', source='derived',
           exact=t_start / NS, note='reference reconstruction, exact equality'),
        ev('node: goal response (accepted)', wall=node_wall('goal_response'),
           source='node/events.jsonl (node tick that processed it)'),
        ev('derived: first update with |desired - actual| > 0.02 rad', source='derived',
           exact=first_derived[0], note=f'joint {first_derived[1]} (reference reconstruction '
           'and /joint_states)'),
        ev('recorded: first action-feedback sample with |error| > 0.02 rad', source='bag feedback',
           exact=fb_cross[0][0] / NS, note=f'joint {fb_cross[0][1]}, {fb_cross[0][2]:.5f} rad'),
        ev('recorded: feedback sample whose error the stop status reported', source='bag feedback',
           exact=stop_fb[0]['t'] / NS if stop_fb else None,
           note=f'{stop_status["max_tracking_error_rad"]:.5f} rad, exact float equality'),
        ev('node: status that triggered the stop (published)',
           wall=stop_status['node_publish_mono_s'], source='status goal.elapsed_s + goal_sent wall',
           note=f'its sim_time_s (newest /joint_states stamp at the node) = '
                f'{stop_status["node_sim_time_s"]}; max {stop_status["max_tracking_error_rad"]:.5f}'),
        ev('recorded: last action-feedback sample (maximum recorded error)',
           realtime=fb_last_row['rt'] / NS, source='bag receipt', exact=fb_last_row['t'] / NS,
           note='realtime is the recorder receipt'),
        ev('driver: stop request (x written to the teleop)', wall=xkey, source='driver/driver.jsonl'),
        ev('node: disarm request processed', wall=node_wall('request', request='disarm'),
           source='node/events.jsonl', note="teleop shutdown sequence after the driver's SIGTERM"),
        ev('node: stop request processed', wall=node_wall('request', request='stop'),
           source='node/events.jsonl', note="teleop shutdown sequence after the driver's SIGTERM"),
        ev('node: estop fault (FAULTED) and cancel request sent', wall=node_wall('cancel_sent'),
           source='node/events.jsonl fault + cancel_sent (same tick)'),
        ev('controller: Got request to cancel goal', realtime=log_rt('Got request to cancel goal'),
           source='run/launch.log'),
        ev('controller: cancel callback read the state of update', source='derived',
           exact=hold_from[-1] / NS if hold_from else None,
           note='hold reference equals /joint_states at this update exactly'),
        ev('controller: hold in effect (first update with the hold reference)', source='derived',
           exact=t_hold / NS, note='observed reference departs; inferred command equals the hold'),
        ev('node: cancel response processed', wall=node_wall('cancel_response'),
           source='node/events.jsonl (tick; callback time not logged)'),
        ev('node: result CANCELED processed', wall=node_wall('result'),
           source='node/events.jsonl (tick; callback time not logged)'),
    ]
    R['timeline'] = T

    # ------------------------------------------------------------------------ outputs
    with open(os.path.join(a.out, 'timeline.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['event', 'source', 'mono_s', 'realtime_s', 'realtime_is', 'sim_exact_s',
                    'sim_lower_s', 'sim_upper_s', 'js_receipt_before_ms', 'js_receipt_after_ms',
                    'note'])
        for e in T:
            b = e['sim_bracket_s'] or [None, None]
            w.writerow([e['event'], e['source'],
                        '' if e['mono_s'] is None else f"{e['mono_s']:.6f}",
                        '' if e['realtime_s'] is None else f"{e['realtime_s']:.6f}",
                        e['realtime_is'] or '',
                        '' if e['sim_exact_s'] is None else f"{e['sim_exact_s']:.3f}",
                        '' if b[0] is None else f'{b[0]:.3f}', '' if b[1] is None else f'{b[1]:.3f}',
                        '' if e['js_receipt_before_ms'] is None else f"{e['js_receipt_before_ms']:.1f}",
                        '' if e['js_receipt_after_ms'] is None else f"{e['js_receipt_after_ms']:.1f}",
                        e['note']])
    fb_by_t = {r['t']: r for r in fb_rows}
    cs_by_t = {r['update']: r for r in cs_rows if r['update'] is not None}
    with open(os.path.join(a.out, 'tracking_per_update.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['sim_s', 'segment', 'joint', 'desired_rad', 'actual_rad', 'error_rad',
                    'desired_velocity_rad_s', 'actual_velocity_rad_s',
                    'recorded_feedback_error_rad', 'recorded_controller_state_error_rad'])
        for t, seg, c, x, e, cv, av, _ in series:
            for j, n in enumerate(CN):
                w.writerow([f'{t / NS:.2f}', seg, n, repr(float(c[j])), repr(float(x[j])),
                            repr(float(e[j])), repr(float(cv[j])), repr(float(av[j])),
                            repr(float(fb_by_t[t]['err'][j])) if t in fb_by_t else '',
                            repr(float(cs_by_t[t]['err'][j])) if t in cs_by_t else ''])
    with open(os.path.join(a.out, 'model_fits.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['joints', 'model', 'parameter', 'value', 'n', 'rms_rad', 'max_abs_rad'])
        for x in fits:
            w.writerow([x['joints'], x['model'], x['parameter'], repr(x['value']), x['n'],
                        f"{x['rms_rad']:.3e}", f"{x['max_abs_rad']:.3e}"])
    R['outputs_note'] = 'timeline.csv, tracking_per_update.csv, model_fits.csv, figures'
    with open(os.path.join(a.out, 'results.json'), 'w') as f:
        json.dump(R, f, indent=1, sort_keys=True, default=float)

    plot(a.out, series, fb_rows, worst, wj, t_start, t_hold, first_derived, fb_cross,
         stop_fb[0]['t'], law_rows, commanded, law['pre_stop_tracking'], ties)
    print(json.dumps({'bag_unchanged': bag_sha_before == bag_sha_after,
                      'trajectory_start_s': t_start / NS, 'hold_onset_s': t_hold / NS,
                      'g': law['pre_stop_tracking']['g_pooled'], 'worst_joint': worst}, indent=1))


def plot(out, series, fb_rows, worst, wj, t_start, t_hold, first_derived, fb_cross, t_stop_fb,
         law_rows, commanded, law, ties):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    BLUE, VERM, GREEN, INK, MUTED = '#0072B2', '#D55E00', '#009E73', '#222222', '#6b6b6b'
    plt.rcParams.update({'font.size': 9, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK,
                         'xtick.color': INK, 'ytick.color': INK, 'axes.titlesize': 10,
                         'legend.frameon': False})
    keep = [s for s in series if s[0] <= t_hold + 30 * UPDATE_NS]       # 0.3 s of the hold
    t = np.array([s[0] for s in keep]) / NS
    des = np.array([s[2][wj] for s in keep])
    act = np.array([s[3][wj] for s in keep])
    err = np.array([s[4][wj] for s in keep])
    dvel = np.array([s[5][wj] for s in keep])
    avel = np.array([s[6][wj] for s in keep])
    fbt = np.array([r['t'] for r in fb_rows]) / NS
    fbe = np.array([r['err'][wj] for r in fb_rows])
    th, ts, tx = t_hold / NS, t_start / NS, first_derived[0]
    fig, ax = plt.subplots(3, 1, figsize=(9.0, 8.4), sharex=True,
                           gridspec_kw={'height_ratios': [1.0, 1.15, 0.9]})
    for k in ax:
        k.axvspan(th - 0.01, th, color='#f2c48d', alpha=0.7, lw=0)
        k.axvspan(th, t[-1], color='#ececec', alpha=0.7, lw=0)
        for x in (ts, tx):
            k.axvline(x, color=MUTED, lw=0.8, ls=':')
        k.grid(axis='y', color='#dddddd', lw=0.6)
        k.spines[['top', 'right']].set_visible(False)
    ax[0].plot(t, des, color=BLUE, lw=1.6, label='desired (controller reference = command)')
    ax[0].plot(t, act, color=VERM, lw=1.6, label='actual (/joint_states)')
    ax[0].set_ylabel('position (rad)')
    ax[0].legend(loc='lower left', fontsize=8)
    tie = ''.join(f'; {n} identical within {d:.0e} rad' for n, d in ties.items())
    ax[0].set_title(f'S2 goal 1, {worst} (worst joint{tie}): tracking against sim time',
                    loc='left', color=INK)
    ymax = ax[0].get_ylim()[1]
    ax[0].text(ts + 0.006, ymax, f'trajectory start {ts:.2f}', rotation=90, va='top', ha='left',
               fontsize=7.5, color=INK)
    ax[0].text(tx - 0.006, ymax, f'first |error| > 0.02: {tx:.2f}', rotation=90, va='top',
               ha='right', fontsize=7.5, color=INK)
    ax[0].text(th + 0.008, ymax, 'stop request → cancel\nin (74.07, 74.08);\nhold from 74.08',
               va='top', ha='left', fontsize=7.5, color=INK)
    ax[1].axhline(-0.02, color=INK, lw=0.9, ls='--')
    ax[1].axhline(0.02, color=INK, lw=0.9, ls='--')
    ax[1].text(ts + 0.01, 0.0203, 'S2 limit ±0.02 rad', color=INK, fontsize=8, va='bottom')
    ax[1].plot(t, err, color=INK, lw=1.2, label='desired − actual at every update (derived)')
    ax[1].plot(fbt, fbe, 'o', ms=4.5, mfc=GREEN, mec='white', mew=0.6,
               label=f'action-feedback error (recorded, {len(fb_rows)} samples)')
    for tt, ytext, lab in ((t_stop_fb / NS, -0.0135, 'the sample the stop status reported'),
                           (fbt[-1], -0.0235, 'last recorded sample = the maximum')):
        y = fbe[np.argmin(np.abs(fbt - tt))]
        ax[1].annotate(f'{tt:.2f} s: {y:.5f} rad, {lab}', xy=(tt, y), xytext=(73.22, ytext),
                       fontsize=7.5, color=INK, va='center',
                       arrowprops={'arrowstyle': '-', 'color': MUTED, 'lw': 0.7,
                                   'shrinkA': 2, 'shrinkB': 3})
    ax[1].set_ylim(-0.0285, 0.0245)
    ax[1].set_ylabel('error (rad)')
    ax[1].legend(loc='upper left', fontsize=8, bbox_to_anchor=(0.0, 0.93))
    ax[2].axhline(0, color=MUTED, lw=0.6)
    ax[2].plot(t, dvel, color=BLUE, lw=1.6, label='desired velocity (reference)')
    ax[2].plot(t, avel, color=VERM, lw=1.1, label='actual velocity (/joint_states)')
    ax[2].set_ylabel('velocity (rad/s)')
    ax[2].set_xlabel('sim time (s)')
    ax[2].legend(loc='lower left', fontsize=8)
    ax[2].set_xlim(t[0], t[-1])
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'worst_joint_tracking.png'), dpi=150)
    plt.close(fig)

    # the per-update law over every joint
    pre = [r for r in law_rows if t_start <= r[0] < t_hold]
    e = np.array([commanded(r[0], t_hold) - r[1] for r in pre]).ravel()
    dx = np.array([r[2] - r[1] for r in pre]).ravel()
    fig, ax = plt.subplots(figsize=(5.6, 4.4))
    ax.plot(e, dx, 'o', ms=3.2, mfc=BLUE, mec='white', mew=0.3,
            label=f'{len(e)} joint-updates, k = {pre[0][0] / NS:.2f} … {pre[-1][0] / NS:.2f} s')
    lim = np.array([e.min(), e.max()])
    ax.plot(lim, 0.1 * lim, color=VERM, lw=1.2,
            label=f"x[k+1] − x[k] = 0.1 · (c[k] − x[k]); max residual "
                  f"{law['residual_at_g_0.1']['max_abs_rad']:.1e} rad")
    ax.set_xlabel('c[k] − x[k]: command minus position at update k (rad)')
    ax.set_ylabel('x[k+1] − x[k] (rad)')
    ax.set_title('Every joint, every 10 ms update before the hold', loc='left', color=INK)
    ax.grid(color='#dddddd', lw=0.6)
    ax.spines[['top', 'right']].set_visible(False)
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'plant_law.png'), dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
