#!/usr/bin/env python3
"""Score an S3 run (docs/S3_FREE_BASE_CRAWL_DESIGN.md section 4, A1-A8), offline.

Specified BEFORE any S3 run, with criteria revision r1's evidence handling built in (not applied
after seeing a result):
- the final goal (the scripted estop) may be "tracking N/A - planned cancellation before
  feedback" only with proof (identity, scripted x, one early cancel processed, CANCELED, no prior
  refusal or unplanned fault, no feedback); missing feedback on any other goal is UNMEASURED;
- tracking N/A never decides A6: the estop hold on contact needs its own pose and joint
  observations over 10 s of sim after the cancel; a CANCELED result alone proves nothing;
- an absent command topic counts as zero messages only with independent recorder, continuity and
  command-owner evidence; otherwise UNMEASURED. A present topic with any message is FAIL;
- missing required evidence is UNMEASURED, never PASS. Per-foot slip is recorded, not scored.

Times: node events and the driver log are CLOCK_MONOTONIC (responses and results at the node's
20 Hz polling tick); bag receipt times are mapped onto it with the status messages that both the
driver and the recorder received (scripts/s2/adjudicate_s2.align). Body frame: +y forward, +x
right; cycle displacements use the boundary's yaw; yaw differences are wrapped.

Usage: score_s3.py --run <run dir> --bag <bag_0.db3> --disabled-library <library.json>
                   --restore-report <report> --harness <run script> --out <new dir>"""
import argparse
import collections
import csv
import gzip
import hashlib
import json
import math
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..', 's2'), HERE]

SCHEMA = 'spiderx.s3_score/1'
P = 9
EXPECTED_GOALS, EXPECTED_CANCELS = 55, 1
ESTOP_GOAL = (0, 1, 1)                          # (b_from, b_to, direction) at level 0
STATUS_SUCCEEDED, STATUS_CANCELED = 4, 5
BAND_FORWARD_M = (0.010, 0.030)
BAND_LATERAL_M = 0.010
BAND_YAW_RAD = 0.05
BAND_HEIGHT_M = 0.010
TILT_MAX_RAD = 0.10
TRACKING_ABORT_RAD, TRACKING_FINDING_RAD = 0.05, 0.02
HOLD_SIM_S, HOLD_Z_MIN_M, HOLD_JOINT_MOTION_RAD = 10.0, 0.045, 0.01
JS_STALE_S, POSE_STALE_S, GRAPH_STALE_S = 0.5, 1.0, 3.0      # the node's monitor limits
COMMAND_TOPIC = '/leg_trajectory_controller/joint_trajectory'
TOPIC_POSE = '/spiderx/sim/world_poses'
FEET = ('lf_foot_1', 'rf_foot_1', 'lr_foot_1', 'rr_foot_1')


def verdict(ok, measured):
    return 'PASS' if measured and ok else ('FAIL' if measured else 'UNMEASURED')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def body_delta(p0, p1):
    dx, dy = p1['xy'][0] - p0['xy'][0], p1['xy'][1] - p0['xy'][1]
    c, s = math.cos(p0['yaw']), math.sin(p0['yaw'])
    return -s * dx + c * dy, c * dx + s * dy, wrap(p1['yaw'] - p0['yaw'])


def nearest(samples, t, tol):
    """The (t, value) sample nearest to t within tol, from a time-sorted list; else None."""
    best = None
    for ts, v in samples:
        if abs(ts - t) <= tol and (best is None or abs(ts - t) < abs(best[0] - t)):
            best = (ts, v)
    return best


def max_gap(times, t0, t1):
    inside = sorted(t for t in times if t0 <= t <= t1)
    if not inside:
        return None
    edges = [t0] + inside + [t1]
    return max(b - a for a, b in zip(edges, edges[1:]))


# ------------------------------------------------------------------ A1
def assess_runs(goals, faults, statuses):
    """goals: [{seq, status, sent, result_t}]; statuses: [(t, status dict)]. Runs A (goals 1-9)
    and C (10-54) succeed, no unplanned fault before the estop, each run ends READY at b0."""
    if len(goals) < 54:
        return {'verdict': 'FAIL' if goals else 'UNMEASURED', 'goals_in_runs': len(goals)}
    runs = goals[:54]
    succeeded = all(g.get('status') == STATUS_SUCCEEDED for g in runs)
    estop_t = next((f['t'] for f in faults if f['reason'] == 'estop'), math.inf)
    unplanned = [f for f in faults if f['t'] < estop_t or f['reason'] != 'estop']

    def ready_between(t0, t1):
        return any(t0 < t <= t1 and s.get('state') == 'READY' and s.get('boundary') == 0
                   and s.get('goal') is None for t, s in statuses)
    ends = [(goals[8]['result_t'], goals[9]['sent']),
            (goals[53]['result_t'], goals[54]['sent'] if len(goals) > 54 else math.inf)]
    ready = [ready_between(a, b) for a, b in ends]
    return {'verdict': verdict(succeeded and not unplanned and all(ready), bool(statuses)),
            'all_succeeded': succeeded, 'unplanned_faults': unplanned,
            'runs_ready_at_b0': ready}


# ------------------------------------------------------------------ A2
def assess_cycles(boundary_times, poses, z_stand, tol):
    """boundary_times: [arm, end of cycle 1, ..., end of cycle 6]; poses: time-sorted
    [(t, {'xy', 'z', 'yaw', 'tilt'})]; z_stand: the settled height (m) or None."""
    rows, measured, ok = [], True, True
    if len(boundary_times) != 7 or z_stand is None:
        return {'verdict': 'UNMEASURED' if len(boundary_times) <= 7 else 'FAIL',
                'cycles': rows, 'reason': 'need the arm pose, 6 cycle boundaries and z_stand'}
    for k in range(6):
        t0, t1 = boundary_times[k], boundary_times[k + 1]
        a, b = nearest(poses, t0, tol), nearest(poses, t1, tol)
        inside = [p['tilt'] for t, p in poses if t0 <= t <= t1]
        if a is None or b is None or not inside:
            rows.append({'cycle': k + 1, 'verdict': 'UNMEASURED'})
            measured = False
            continue
        fwd, lat, dyaw = body_delta(a[1], b[1])
        r = {'cycle': k + 1, 'forward_m': fwd, 'lateral_m': lat, 'yaw_rad': dyaw,
             'z_end_m': b[1]['z'], 'height_vs_standing_m': b[1]['z'] - z_stand,
             'max_tilt_rad': max(inside)}
        r['verdict'] = 'PASS' if (BAND_FORWARD_M[0] <= fwd <= BAND_FORWARD_M[1]
                                  and abs(lat) <= BAND_LATERAL_M and abs(dyaw) <= BAND_YAW_RAD
                                  and abs(r['height_vs_standing_m']) <= BAND_HEIGHT_M
                                  and r['max_tilt_rad'] <= TILT_MAX_RAD) else 'FAIL'
        ok &= r['verdict'] == 'PASS'
        rows.append(r)
    v = 'FAIL' if not ok else ('PASS' if measured else 'UNMEASURED')
    return {'verdict': v, 'cycles': rows, 'z_stand_m': z_stand}


# ------------------------------------------------------------------ A3
def assess_tracking(goals, feedback, keys, cancels, faults, refusals, tol, period):
    """goals: [{seq, sent, accepted, response_t, result_t, status, b_from, b_to, direction,
    level, cancel_response_code, cancel_response_t}]; feedback: {goal_id: [(t, max |err|)]}."""
    mapped, unattributed = collections.defaultdict(list), []
    for gid, s in feedback.items():
        hits = [g['seq'] for g in goals if g.get('sent') is not None and g.get('result_t')
                is not None and g['sent'] - tol <= s[0][0] and s[-1][0] <= g['result_t'] + tol]
        (mapped[hits[0]].append(gid) if len(hits) == 1 else unattributed.append(gid))
    rows, fail, unmeasured, findings = [], False, bool(unattributed), {}
    last = max((g['seq'] for g in goals), default=None)
    for g in goals:
        ids = mapped.get(g['seq'], [])
        if len(ids) > 1:
            unmeasured = True
        samples = [e for gid in ids for _, e in feedback[gid]]
        row = {'seq': g['seq'], 'feedback_samples': len(samples)}
        if samples:
            m = max(samples)
            row.update(max_abs_error_rad=m, verdict='FAIL' if m > TRACKING_ABORT_RAD else 'PASS')
            fail |= m > TRACKING_ABORT_RAD
            if m > TRACKING_FINDING_RAD:
                findings[g['seq']] = m
        elif g['seq'] == last:
            proof = estop_na_proof(g, goals, feedback, keys, cancels, faults, refusals, tol,
                                   period)
            row['estop_proof'] = proof
            if proof['qualifies']:
                row.update(verdict='N/A', note='tracking N/A - planned cancellation before '
                           'feedback; A6 (the hold) is assessed separately')
            else:
                row['verdict'] = 'UNMEASURED'
                unmeasured = True
        else:
            row['verdict'] = 'UNMEASURED'
            unmeasured = True
        rows.append(row)
    v = 'FAIL' if fail else ('UNMEASURED' if unmeasured or not goals else 'PASS')
    return {'verdict': v, 'goals': rows, 'findings_above_0_02_rad': findings,
            'unattributed_feedback_ids': unattributed}


def estop_na_proof(g, goals, feedback, keys, cancels, faults, refusals, tol, period):
    t_sent, t_resp, t_res = g.get('sent'), g.get('response_t'), g.get('result_t')
    x = [k for k in keys if k['key'] == 'x' and 'estop exercise' in (k.get('why') or '')]
    x_in = [k for k in x if None not in (t_sent, t_res) and t_sent <= k['t'] <= t_res]
    t_x = x_in[0]['t'] if len(x_in) == 1 else None
    c = [k for k in cancels if k['seq'] == g['seq']]
    t_c = c[0]['t'] if len(c) == 1 else None
    estop = [f for f in faults if f['reason'] == 'estop']
    planned = [f for f in estop if t_x is not None and f['t'] >= t_x]
    unplanned = [f for f in faults if f not in planned and f['t'] <= (t_res or math.inf)]
    fb_window = [gid for gid, s in feedback.items() if None not in (t_sent, t_res)
                 and any(t_sent - tol <= t <= t_res + tol for t, _ in s)]
    checks = {
        'final_goal_of_the_script': g['seq'] == EXPECTED_GOALS == len(goals),
        'identity_is_the_estop_exercise': (g.get('b_from'), g.get('b_to'), g.get('direction'))
        == ESTOP_GOAL and g.get('level') == 0,
        'accepted': g.get('accepted') is True,
        'scripted_estop_key_during_goal': len(x_in) == 1 and len(x) == 1,
        'estop_fault_after_key': t_x is not None and len(estop) == 1 and len(planned) == 1,
        'exactly_one_cancel': len(cancels) == EXPECTED_CANCELS and len(c) == 1,
        'early_cancel_request': None not in (t_c, t_resp, t_x) and t_c >= t_x
        and (t_c <= t_resp or t_c - t_resp < period),
        'cancel_processed_by_server': g.get('cancel_response_code') == 0,
        'result_canceled': g.get('status') == STATUS_CANCELED,
        'no_unplanned_fault_before': not unplanned,
        'no_refusal_before_result': not [r for r in refusals if r['t'] <= (t_res or math.inf)],
        'no_feedback_for_goal': not fb_window,
    }
    return {'qualifies': all(checks.values()), 'checks': checks,
            'failed': [k for k, v in checks.items() if not v]}


# ------------------------------------------------------------------ A4, A8, A7
def assess_continuity(goals_sent, refusals):
    vals = [g.get('continuity_error_rad') for g in goals_sent]
    refused = [r for r in refusals if 'continuity' in (r.get('reason') or '')]
    return {'verdict': verdict(not refused, bool(goals_sent)),
            'max_continuity_error_rad': max((v for v in vals if v is not None), default=None),
            'continuity_refusals': refused}


def assess_library(enabled_sha, disabled_sha):
    return {'verdict': verdict(enabled_sha == disabled_sha, None not in (enabled_sha,
                                                                         disabled_sha)),
            'enabled_library_sha256': enabled_sha, 'disabled_library_sha256': disabled_sha}


def assess_restore(text):
    if not text:
        return {'verdict': 'UNMEASURED', 'reason': 'no restore report'}
    checks = {
        'verdict_disabled_and_verified': 'VERDICT: DISABLED AND VERIFIED' in text,
        'gates_false_imported': 'ok: a. all three gates False' in text,
        'm6_gait_replay_live_exit_3': re.search(r'ok: c\. m6_gait_replay\.py --live refused, '
                                                'exit 3', text) is not None,
        'm6_live_playback_live_exit_3': re.search(r'ok: c\. m6_live_playback --live refused, '
                                                  'exit 3', text) is not None,
        'no_simulation_or_node_leftover': 'ok: no simulation process' in text
        and 'FAIL:' not in text,
    }
    return {'verdict': verdict(all(checks.values()), True), 'checks': checks}


# ------------------------------------------------------------------ A5
def assess_ownership(counts, topic_present, topic_count, support):
    """counts: {goals_sent, cancels_sent, max_foreign, max_publishers}; support: the r1 checks
    for an absent topic ({name: bool})."""
    terms = {'goals_55': counts.get('goals_sent') == EXPECTED_GOALS,
             'cancels_1': counts.get('cancels_sent') == EXPECTED_CANCELS,
             'foreign_clients_0': counts.get('max_foreign') == 0,
             'command_publishers_0': counts.get('max_publishers') == 0}
    if topic_present:
        jt = 'PASS' if topic_count == 0 else 'FAIL'
    else:
        jt = 'PASS' if support and all(support.values()) else 'UNMEASURED'
    v = 'FAIL' if not all(terms.values()) or jt == 'FAIL' else jt
    return {'verdict': v, 'terms': terms, 'command_topic': {
        'present_in_bag': topic_present, 'messages': topic_count if topic_present else None,
        'term': jt, 'support': None if topic_present else support,
        'failed': None if topic_present else [k for k, x in (support or {}).items() if not x]}}


# ------------------------------------------------------------------ A6
def assess_hold(cancel_ok, result_canceled, start_t, sim_of, poses, joints):
    """The estop hold on contact: from the CANCELED result (start_t) over 10 s of sim.
    sim_of(t) -> sim time; poses: [(t, pose)]; joints: [(t, {joint: q})]."""
    out = {'faulted_with_one_cancel': cancel_ok, 'result_canceled': result_canceled}
    if not (cancel_ok and result_canceled) or start_t is None:
        out['verdict'] = 'FAIL' if start_t is not None else 'UNMEASURED'
        return out
    s0 = sim_of(start_t)
    end = next((t for t, _ in joints if t >= start_t and sim_of(t) >= s0 + HOLD_SIM_S), None)
    if s0 is None or end is None:
        out.update(verdict='UNMEASURED', reason='the recording does not cover 10 s of sim '
                   'after the cancel')
        return out
    ps = [(t, p) for t, p in poses if start_t <= t <= end]
    js = [(t, q) for t, q in joints if start_t <= t <= end]
    gap_p = max_gap([t for t, _ in ps], start_t, end)
    gap_j = max_gap([t for t, _ in js], start_t, end)
    if not ps or not js or gap_p > POSE_STALE_S or gap_j > JS_STALE_S:
        out.update(verdict='UNMEASURED', reason='pose or joint coverage gap in the hold',
                   max_pose_gap_s=gap_p, max_joint_gap_s=gap_j)
        return out
    ref = js[0][1]
    motion = max(abs(q[n] - ref[n]) for _, q in js for n in ref if n in q)
    out.update(window_sim_s=[s0, sim_of(end)], max_tilt_rad=max(p['tilt'] for _, p in ps),
               min_z_m=min(p['z'] for _, p in ps), max_joint_motion_rad=motion,
               pose_samples=len(ps), joint_samples=len(js))
    out['verdict'] = 'PASS' if (out['max_tilt_rad'] <= TILT_MAX_RAD and out['min_z_m'] >=
                                HOLD_Z_MIN_M and motion <= HOLD_JOINT_MOTION_RAD) else 'FAIL'
    return out


# ------------------------------------------------------------------ slip (recorded only)
def slip_rows(goals_sent, windows, feet):
    """Per goal and stance foot: the horizontal travel of the foot (max distance from its first
    position) over the goal. feet: {foot: [(t, (x, y))]}; windows: {seq: (t0, t1)}."""
    swing = {'lf': 'lf_foot_1', 'rf': 'rf_foot_1', 'lr': 'lr_foot_1', 'rr': 'rr_foot_1'}
    rows = []
    for g in goals_sent:
        w = windows.get(g['seq'])
        if w is None:
            continue
        for foot, s in feet.items():
            if swing.get(g.get('leg')) == foot:
                continue
            pts = [xy for t, xy in s if w[0] <= t <= w[1]]
            if len(pts) < 2:
                continue
            rows.append({'seq': g['seq'], 'kind': g.get('kind'), 'foot': foot,
                         'travel_m': max(math.hypot(x - pts[0][0], y - pts[0][1])
                                         for x, y in pts), 'samples': len(pts)})
    return rows


def overall(A):
    vs = [a['verdict'] for a in A.values()]
    return 'PASS' if all(v == 'PASS' for v in vs) else (
        'FAIL' if any(v == 'FAIL' for v in vs) else 'INCONCLUSIVE')


# ------------------------------------------------------------------ inputs from a run
def sha(path):
    try:
        with open(path, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return None


def load_and_score(run, bag, disabled_library, restore_report, harness):
    import sqlite3
    import adjudicate_s2 as adj
    import response_law as rl
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from spiderx_controller import m55_locomotion as loc
    from spiderx_controller import m61a_fixed_base as fb
    from spiderx_controller.m55_rclpy import body_pose_sample
    cfg = loc.load_config()
    js, cmds, statuses, meta = rl.read_bag(bag)
    con = sqlite3.connect(f'file:{bag}?mode=ro&immutable=1', uri=True)
    topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
    receipts, poses, feet = collections.defaultdict(list), [], {f: [] for f in FEET}
    pose_id = next((i for i, (n, _) in topics.items() if n == TOPIC_POSE), None)
    pose_type = get_message(topics[pose_id][1]) if pose_id is not None else None
    for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages '
                                     'ORDER BY timestamp, id'):
        receipts[topics[tid][0]].append(ts / 1e9)
        if tid == pose_id:
            m = deserialize_message(data, pose_type)
            codes, p = body_pose_sample(m.transforms, cfg)
            if not codes:
                poses.append((ts / 1e9, p))
            for f in FEET:
                sel = fb.select_entries(m.transforms, cfg.monitor.model_name, f)
                if not sel.codes:
                    w = sel.model * sel.body_link
                    feet[f].append((ts / 1e9, (w.p[0], w.p[1])))
    con.close()
    after = rl.sha256(bag)
    ev = [json.loads(ln) for ln in open(os.path.join(run, 'node', 'events.jsonl')) if ln.strip()]
    dpath = os.path.join(run, 'driver', 'driver.jsonl')
    drv = [json.loads(ln) for ln in (gzip.open(dpath + '.gz', 'rt') if not os.path.exists(dpath)
                                     else open(dpath))]
    al = adj.align([(d['mono'], d['status']) for d in drv if 'status' in d],
                   [(s.receipt_ns / 1e9, s.data) for s in statuses])
    if al is None:
        raise SystemExit('cannot align the bag to the node clock')
    off, tol = al['offset_s'], al['tolerance_s']
    mono = {k: [t - off for t in v] for k, v in receipts.items()}
    poses = [(t - off, p) for t, p in poses]
    feet = {f: [(t - off, xy) for t, xy in s] for f, s in feet.items()}
    jts = [(j.receipt_ns / 1e9 - off, dict(zip(j.names, j.positions)), j.stamp_ns / 1e9)
           for j in js]
    sim_pts = [(t, s) for t, _, s in jts]

    def sim_of(t):
        p = nearest(sim_pts, t, JS_STALE_S)
        return None if p is None else p[1]
    goals = collections.OrderedDict()
    for e in ev:
        s = e.get('seq')
        if e['event'] == 'goal_sent':
            goals[s] = {k: e.get(k) for k in ('seq', 'b_from', 'b_to', 'direction', 'level',
                                              'kind', 'leg', 'continuity_error_rad')}
            goals[s]['sent'] = e['wall']
        elif e['event'] == 'goal_response':
            goals[s].update(accepted=e['accepted'], response_t=e['wall'])
        elif e['event'] == 'cancel_response':
            goals[s].update(cancel_response_code=e['code'], cancel_response_t=e['wall'])
        elif e['event'] == 'result':
            goals[s].update(status=e['status'], result_t=e['wall'])
    gl = list(goals.values())
    faults = [{'reason': e['reason'], 't': e['wall']} for e in ev if e['event'] == 'fault']
    refusals = [{'request': e['request'], 'reason': e['reason'], 't': e['wall']}
                for e in ev if e['event'] == 'refused']
    cancels = [{'seq': e['seq'], 't': e['wall']} for e in ev if e['event'] == 'cancel_sent']
    keys = [{'key': d['key'].strip("'"), 't': d['mono'], 'why': d.get('why')}
            for d in drv if 'key' in d]
    st_mono = [(s.receipt_ns / 1e9 - off, s.data) for s in statuses]
    fb_ = collections.defaultdict(list)
    for c in cmds:
        if c.source == 'feedback':
            fb_[c.goal_id].append((c.receipt_ns / 1e9 - off, max(abs(x) for x in c.error)))
    gaps = sorted(b[0] - a[0] for v in fb_.values() for a, b in zip(v, v[1:]))
    period = statistics.median(gaps) if gaps else 0.05
    # boundaries: arm (the first 'arm' request), then each phase_done ending a forward cycle
    arm_t = next((e['wall'] for e in ev if e['event'] == 'request' and e.get('request') == 'arm'
                  and 'armed' in (e.get('message') or '')), None)
    ends = [e['wall'] for e in ev if e['event'] == 'phase_done' and e.get('direction') == 1
            and e.get('b') in (0, P)]
    settle = [p['z'] for t, p in poses if arm_t is not None and arm_t - 10.0 <= t <= arm_t]
    z_stand = statistics.mean(settle) if settle else None
    A = {}
    A['A1'] = assess_runs(gl, faults, st_mono)
    A['A2'] = assess_cycles(([arm_t] if arm_t is not None else []) + ends, poses, z_stand,
                            tol + 0.05)
    A['A3'] = assess_tracking(gl, dict(fb_), keys, cancels, faults, refusals, tol, period)
    A['A4'] = assess_continuity(gl, refusals)
    t0 = gl[0]['sent'] if gl else None
    t1 = gl[-1].get('result_t') if gl else None
    script = open(harness).read() if harness and os.path.exists(harness) else ''
    rec = re.search(r'ros2 bag record(.*?)>', script, re.S)
    requested = re.findall(r'(/[A-Za-z0-9_/]+)', rec.group(1)) if rec else []
    log_path = os.path.join(run, 'bag_record.log')
    log = [re.sub(r'\x1b\[[0-9;]*m', '', ln) for ln in open(log_path)] \
        if os.path.exists(log_path) else []
    stamp = lambda ln: float(re.search(r'\[(\d+\.\d+)\]', ln).group(1)) - off  # noqa: E731
    started = [stamp(ln) for ln in log if 'Recording...' in ln]
    stopped = [stamp(ln) for ln in log if 'Recording stopped' in ln]
    subs = [stamp(ln) for ln in log if 'Subscribed to topic' in ln]
    graphs = [(s.get('monitors') or {}).get('graph') for t, s in st_mono
              if t0 is not None and t1 is not None and t0 <= t <= t1]

    def pubs(name):
        p = os.path.join(run, 'captures', name)
        txt = open(p).read() if os.path.exists(p) else ''
        m = re.search(re.escape(COMMAND_TOPIC) + r'.*?Publisher count: (\d+)', txt, re.S)
        return int(m.group(1)) if m else None
    js_gap = max_gap(mono.get('/joint_states', []), t0, t1) if t0 and t1 else None
    pose_gap = max_gap(mono.get(TOPIC_POSE, []), t0, t1) if t0 and t1 else None
    support = {
        'requested_from_recorder': COMMAND_TOPIC in requested,
        'recording_covers_goal_interval': bool(started and stopped) and t0 is not None
        and t1 is not None and started[0] <= t0 and stopped[0] >= t1,
        'discovery_demonstrated_after_start': bool(started) and any(t > started[0] for t in subs),
        'recorder_log_without_failure': bool(log) and not any(
            re.search(r'\[(WARN|ERROR|FATAL)\]', ln) for ln in log),
        'node_monitored_streams_continuous': js_gap is not None and pose_gap is not None
        and js_gap <= JS_STALE_S and pose_gap <= POSE_STALE_S,
        'owner_monitor_zero_throughout': bool(graphs) and all(
            isinstance(g, dict) and g.get('command_topic_publishers') == 0
            and g.get('foreign_action_clients') == 0 for g in graphs),
        'captures_before_after_zero_publishers': pubs('before_topics.txt') == 0
        and pubs('after_topics.txt') == 0,
    }
    counts = {'goals_sent': len(gl), 'cancels_sent': len(cancels),
              'max_foreign': max((g.get('foreign_action_clients', -1) for g in graphs
                                  if isinstance(g, dict)), default=None),
              'max_publishers': max((g.get('command_topic_publishers', -1) for g in graphs
                                     if isinstance(g, dict)), default=None)}
    present = COMMAND_TOPIC in [n for n, _ in topics.values()]
    A['A5'] = assess_ownership(counts, present, len(receipts.get(COMMAND_TOPIC, [])), support)
    last = gl[-1] if gl else {}
    A['A6'] = assess_hold(len(cancels) == EXPECTED_CANCELS and any(f['reason'] == 'estop'
                                                                   for f in faults),
                          last.get('status') == STATUS_CANCELED, last.get('result_t'), sim_of,
                          poses, [(t, q) for t, q, _ in jts])
    A['A7'] = assess_restore(open(restore_report).read() if restore_report and
                             os.path.exists(restore_report) else None)
    A['A8'] = assess_library(sha(os.path.join(run, 'node', 'library.json')),
                             sha(disabled_library))
    windows = {g['seq']: (g['sent'], g.get('result_t')) for g in gl
               if g.get('result_t') is not None}
    slips = slip_rows(gl, windows, feet)
    return {'schema': SCHEMA, 'A': A, 'overall': overall(A),
            'alignment': al, 'feedback_period_s': period,
            'slip_recorded_only': {'rows': len(slips), 'max_travel_m': max(
                (r['travel_m'] for r in slips), default=None)},
            'bag': {**meta, 'sha256_after_pose_scan': after}}, slips


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for k in ('--run', '--bag', '--out'):
        a.add_argument(k, required=True)
    a.add_argument('--disabled-library')
    a.add_argument('--restore-report')
    a.add_argument('--harness')
    o = a.parse_args(argv)
    if os.path.isdir(o.out) and os.listdir(o.out):
        print(f'refusing to write into non-empty {o.out}', file=sys.stderr)
        return 3
    res, slips = load_and_score(o.run, o.bag, o.disabled_library, o.restore_report, o.harness)
    os.makedirs(o.out, exist_ok=True)
    with open(os.path.join(o.out, 'summary.json'), 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
    with open(os.path.join(o.out, 'slip.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['seq', 'kind', 'foot', 'travel_m', 'samples'])
        w.writeheader()
        w.writerows(slips)
    print(json.dumps({k: v['verdict'] for k, v in res['A'].items()}), res['overall'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
