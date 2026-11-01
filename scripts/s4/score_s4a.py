#!/usr/bin/env python3
"""Score an S4a run (docs/S4_KEYBOARD_TELEOP_PLAN.md, S4a), offline. Specified BEFORE any S4a run.

Reuses the frozen S3 scorer's generic parts unchanged (scripts/s3/score_s3.py: body frame and yaw
wrap, nearest sample, continuity (A4), the measured estop hold (A6), restore (A7), library (A8)).
S4a-specific:
- A1 runs: run R (goals 1-9, all reverse) and run F (goals 10-18, all forward) at level 0. Every
  goal SUCCEEDED, no unplanned fault before the estop, and each run ends READY at boundary 0.
- A2 cycles (2), at the node's own cycle ends: arm; phase_done direction -1 at b 0 (end of R);
  phase_done direction +1 at b 0/9 (end of F). Each cycle is measured in the frame of its start
  boundary's yaw, with the yaw difference wrapped.
  - Progress along body +y: reverse in [-30, -10] mm, forward return in [10, 30] mm.
  - |lateral| <= 10 mm, |yaw| <= 0.05 rad, height within +-10 mm of standing, max tilt
    <= 0.10 rad over the cycle.
  - The return is measured from the end of R, never assumed to land at the arm pose. The net
    displacement is recorded only.
- A3 tracking: per goal, max |feedback error| <= 0.05 rad, and > 0.02 is a finding. Every goal but
  the last needs attributed feedback. The last goal (the estop goal) may be N/A, never a value,
  only under this planned-cancel exception, which uses the driver's recorded key-write times
  (before and after os.write, full precision):
  - identity: seq 19, 0 -> 1 forward, level 0; accepted;
  - exactly one estop-capable key, the scripted x, with t_sent <= t_before <= t_after <= t_result;
  - exactly one estop fault, at or after t_before and at or before the result;
  - one node estop request at the fault; one teleop '[estop] ok' response with the node's message,
    after the fault; no driver estop-service fallback;
  - exactly one cancel, decided at the fault and early (at or before the polled goal response, or
    within one feedback period of it), answered with code 0; the result CANCELED;
  - the controller's log order (every goal received and accepted, then one cancel request after
    the last acceptance; launch log required);
  - event order consistent; no unplanned fault or refusal before the result; no feedback for the
    goal.
- A5 ownership: 19 goals, 1 cancel, no foreign client or publisher, and 0 joint_trajectory
  messages. An absent topic counts as zero only with recorder, continuity, owner-monitor and
  capture support.
- A6: the estop hold on contact, measured from poses and joint states over 10 s of sim after the
  CANCELED result. A CANCELED result alone proves nothing.
- A7 restore; A8 library.
Missing evidence is UNMEASURED, never PASS.

Usage: score_s4a.py --run <run dir> --bag <bag_0.db3> --disabled-library <library.json>
                    --restore-report <report> --harness <run script> --out <new dir>"""
import argparse
import collections
import gzip
import json
import math
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..', 's3'), os.path.join(HERE, '..', 's2'), HERE]
import score_s3 as sc  # noqa: E402  (frozen; imported unchanged)

SCHEMA = 'spiderx.s4a_score/1'
P = 9
EXPECTED_GOALS, EXPECTED_CANCELS = 2 * P + 1, 1
ESTOP_GOAL = (0, 1, 1)                     # (b_from, b_to, direction) at level 0
REVERSE, FORWARD = -1, 1
BAND_PROGRESS_M = {REVERSE: (-0.030, -0.010), FORWARD: (0.010, 0.030)}
ESTOP_KEYS = ("'x'", repr('\x1b'))          # the teleop's estop keys: x and Esc
verdict, body_delta, nearest = sc.verdict, sc.body_delta, sc.nearest


# ------------------------------------------------------------------ A1
def assess_runs(goals, faults, statuses):
    """goals: [{seq, status, direction, level, sent, result_t}]; statuses: [(t, status)]."""
    if len(goals) < 2 * P:
        return {'verdict': 'FAIL' if goals else 'UNMEASURED', 'goals_in_runs': len(goals)}
    runs = goals[:2 * P]
    succeeded = all(g.get('status') == sc.STATUS_SUCCEEDED for g in runs)
    directions = ([g.get('direction') for g in runs[:P]] == [REVERSE] * P
                  and [g.get('direction') for g in runs[P:]] == [FORWARD] * P)
    level0 = all(g.get('level') == 0 for g in goals)
    estop_t = next((f['t'] for f in faults if f['reason'] == 'estop'), math.inf)
    unplanned = [f for f in faults if f['t'] < estop_t or f['reason'] != 'estop']

    def ready_between(t0, t1):
        return any(t0 < t <= t1 and s.get('state') == 'READY' and s.get('boundary') == 0
                   and s.get('goal') is None for t, s in statuses)
    ends = [(goals[P - 1].get('result_t'), goals[P]['sent']),
            (goals[2 * P - 1].get('result_t'),
             goals[2 * P]['sent'] if len(goals) > 2 * P else math.inf)]
    ready = [a is not None and ready_between(a, b) for a, b in ends]
    return {'verdict': verdict(succeeded and directions and level0 and not unplanned
                               and all(ready), bool(statuses)),
            'all_succeeded': succeeded, 'directions_reverse_then_forward': directions,
            'all_level_0': level0, 'unplanned_faults': unplanned, 'runs_ready_at_b0': ready}


# ------------------------------------------------------------------ A2
def cycle_boundaries(events):
    """(arm, end of R, end of F) from node events; None where missing."""
    arm = next((e['wall'] for e in events if e['event'] == 'request'
                and e.get('request') == 'arm' and 'armed' in (e.get('message') or '')), None)
    r_end = next((e['wall'] for e in events if e['event'] == 'phase_done'
                  and e.get('direction') == REVERSE and e.get('b') == 0), None)
    f_end = next((e['wall'] for e in events if e['event'] == 'phase_done'
                  and e.get('direction') == FORWARD and e.get('b') in (0, P)
                  and r_end is not None and e['wall'] > r_end), None)
    return arm, r_end, f_end


def assess_cycles(boundaries, poses, z_stand, tol):
    """boundaries: (arm, end of R, end of F); poses: time-sorted [(t, {'xy','z','yaw','tilt'})]."""
    rows, measured, ok = [], True, True
    if None in boundaries or z_stand is None:
        return {'verdict': 'UNMEASURED', 'cycles': rows,
                'reason': 'need the arm time, the end of R, the end of F and z_stand'}
    for k, dirn in enumerate((REVERSE, FORWARD)):
        t0, t1 = boundaries[k], boundaries[k + 1]
        a, b = nearest(poses, t0, tol), nearest(poses, t1, tol)
        inside = [p['tilt'] for t, p in poses if t0 <= t <= t1]
        if a is None or b is None or not inside:
            rows.append({'cycle': k + 1, 'direction': dirn, 'verdict': 'UNMEASURED'})
            measured = False
            continue
        fwd, lat, dyaw = body_delta(a[1], b[1])
        lo, hi = BAND_PROGRESS_M[dirn]
        r = {'cycle': k + 1, 'direction': dirn, 'forward_m': fwd, 'lateral_m': lat,
             'yaw_rad': dyaw, 'z_end_m': b[1]['z'], 'height_vs_standing_m': b[1]['z'] - z_stand,
             'max_tilt_rad': max(inside), 'band_forward_m': [lo, hi]}
        r['verdict'] = 'PASS' if (lo <= fwd <= hi and abs(lat) <= sc.BAND_LATERAL_M
                                  and abs(dyaw) <= sc.BAND_YAW_RAD
                                  and abs(r['height_vs_standing_m']) <= sc.BAND_HEIGHT_M
                                  and r['max_tilt_rad'] <= sc.TILT_MAX_RAD) else 'FAIL'
        ok &= r['verdict'] == 'PASS'
        rows.append(r)
    out = {'verdict': 'FAIL' if not ok else ('PASS' if measured else 'UNMEASURED'),
           'cycles': rows, 'z_stand_m': z_stand}
    a, b = nearest(poses, boundaries[0], tol), nearest(poses, boundaries[2], tol)
    if a is not None and b is not None:
        f, lat, dyaw = body_delta(a[1], b[1])
        out['net_after_return_recorded_only'] = {'forward_m': f, 'lateral_m': lat,
                                                 'yaw_rad': dyaw}
    return out


# ------------------------------------------------------------------ A3
def key_write_bracket(driver_records, why='estop exercise'):
    """The scripted estop key's recorded write bracket: (t_before, t_after) around os.write, the
    write result, and the number of estop-capable key records."""
    estop = [d for d in driver_records if d.get('key') in ESTOP_KEYS]
    scripted = [d for d in estop if why in (d.get('why') or '')]
    out = {'n_estop_keys': len(estop), 'n_scripted_x': len(scripted), 't_before': None,
           't_after': None, 'write_result': None}
    if len(scripted) == 1:
        d = scripted[0]
        out.update(t_before=d.get('t_before_write'), t_after=d.get('t_after_write'),
                   write_result=d.get('write_result'))
    return out


def estop_proof(g, goals, feedback, kb, cancels, faults, refusals, requests, teleop, fallback,
                controller, tol, period):
    t_sent, t_resp, t_res = g.get('sent'), g.get('response_t'), g.get('result_t')
    tb, ta = kb.get('t_before'), kb.get('t_after')
    estop = [f for f in faults if f['reason'] == 'estop']
    planned = [f for f in estop if tb is not None and f['t'] >= tb
               and (t_res is None or f['t'] <= t_res)]
    unplanned = [f for f in faults if f not in planned and f['t'] <= (t_res or math.inf)]
    c = [k for k in cancels if k['seq'] == g['seq']]
    t_c = c[0]['t'] if len(c) == 1 else None
    req = [r for r in requests if r['request'] == 'estop']
    t_f = planned[0]['t'] if len(planned) == 1 else None
    fb_window = [gid for gid, s in feedback.items() if None not in (t_sent, t_res)
                 and any(t_sent - tol <= t <= t_res + tol for t, _ in s)]
    checks = {
        'final_goal_of_the_script': g['seq'] == EXPECTED_GOALS == len(goals),
        'identity_is_the_estop_exercise': (g.get('b_from'), g.get('b_to'), g.get('direction'))
        == ESTOP_GOAL and g.get('level') == 0,
        'accepted': g.get('accepted') is True,
        'one_scripted_estop_key_written_during_goal': kb.get('n_estop_keys') == 1
        and kb.get('n_scripted_x') == 1 and kb.get('write_result') == 1
        and None not in (tb, ta, t_sent, t_res) and t_sent <= tb <= ta <= t_res,
        'estop_fault_at_or_after_write_start': len(estop) == 1 and t_f is not None,
        'one_node_estop_request_at_the_fault': len(req) == 1 and t_f is not None
        and req[0]['t'] == t_f,
        'one_teleop_estop_response_after_the_fault': len(teleop) == 1 and t_f is not None
        and teleop[0]['ok'] and teleop[0]['t'] >= t_f and len(req) == 1
        and teleop[0]['message'] == req[0].get('message'),
        'no_driver_estop_service_fallback': not fallback,
        'exactly_one_cancel': len(cancels) == EXPECTED_CANCELS and len(c) == 1,
        'cancel_decision_at_the_fault_and_early': None not in (t_c, t_f, t_resp)
        and t_c == t_f and (t_c <= t_resp or t_c - t_resp < period),
        'cancel_processed_by_server': g.get('cancel_response_code') == 0,
        'result_canceled': g.get('status') == sc.STATUS_CANCELED,
        'controller_log_order': bool(controller and controller.get('ok')),
        'event_order_consistent': None not in (t_sent, t_res, t_resp, t_c) and t_sent <= t_c
        and t_c <= t_res and t_sent <= t_resp <= t_res,
        'no_unplanned_fault_before': not unplanned,
        'no_refusal_before_result': not [r for r in refusals if r['t'] <= (t_res or math.inf)],
        'no_feedback_for_goal': not fb_window,
    }
    return {'qualifies': all(checks.values()), 'checks': checks,
            'failed': [k for k, v in checks.items() if not v],
            'times': {'goal_sent': t_sent, 'key_before_write': tb, 'key_after_write': ta,
                      'estop_fault': t_f, 'cancel_decision': t_c,
                      'goal_response_polled': t_resp, 'result_polled': t_res}}


def assess_tracking(goals, feedback, tol, proof):
    """Feedback attributed to goals by window (the frozen rule); the last goal is N/A only under
    the planned-cancel exception, else UNMEASURED."""
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
            row.update(max_abs_error_rad=m, verdict='FAIL' if m > sc.TRACKING_ABORT_RAD else 'PASS')
            fail |= m > sc.TRACKING_ABORT_RAD
            if m > sc.TRACKING_FINDING_RAD:
                findings[g['seq']] = m
        elif g['seq'] == last and proof is not None and proof['qualifies']:
            row.update(verdict='N/A', note='tracking not measured: planned cancellation before '
                       'feedback (S4a exception); A6 is assessed separately')
        else:
            row['verdict'] = 'UNMEASURED'
            unmeasured = True
        rows.append(row)
    v = 'FAIL' if fail else ('UNMEASURED' if unmeasured or not goals else 'PASS')
    return {'verdict': v, 'goals': rows, 'findings_above_0_02_rad': findings,
            'unattributed_feedback_ids': unattributed, 'final_goal_exception': proof}


# ------------------------------------------------------------------ A5
def assess_ownership(counts, topic_present, topic_count, support):
    terms = {'goals_19': counts.get('goals_sent') == EXPECTED_GOALS,
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
        'term': jt, 'support': None if topic_present else support}}


def overall(A):
    vs = [a['verdict'] for a in A.values()]
    return 'FAIL' if 'FAIL' in vs else ('PASS' if all(v == 'PASS' for v in vs)
                                        else 'INCONCLUSIVE')


# ------------------------------------------------------------------ evidence parsing (pure)
def teleop_responses(driver_records, name='estop'):
    out = []
    for d in driver_records:
        for ln in re.split(r'[\r\n]+', d.get('teleop_out') or ''):
            m = re.match(r'\[%s\] (ok|REFUSED): (.*)$' % re.escape(name), ln.strip())
            if m:
                out.append({'t': d['mono'], 'ok': m.group(1) == 'ok', 'message': m.group(2)})
    return out


def controller_order(log_lines, n_goals):
    ev = []
    for ln in log_lines:
        m = re.search(r'\[(\d+\.\d+)\] \[leg_trajectory_controller\]: (Received new action goal|'
                      r'Accepted new action goal|Got request to cancel goal)', ln)
        if m:
            ev.append((float(m.group(1)), m.group(2)))
    rec = [t for t, s in ev if s.startswith('Received')]
    acc = [t for t, s in ev if s.startswith('Accepted')]
    can = [t for t, s in ev if s.startswith('Got request')]
    ok = (len(rec) == n_goals and len(acc) == n_goals and len(can) == 1
          and can[0] >= acc[-1] >= rec[-1])
    return {'ok': ok, 'received': len(rec), 'accepted': len(acc), 'cancel_requests': len(can)}


# ------------------------------------------------------------------ loading (read-only)
def load_and_score(run, bag, disabled_library, restore_report, harness):
    import sqlite3
    import adjudicate_s2 as adj
    import response_law as rl
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from spiderx_controller import m55_locomotion as loc
    from spiderx_controller.m55_rclpy import body_pose_sample
    cfg = loc.load_config()
    js, cmds, statuses, meta = rl.read_bag(bag)
    con = sqlite3.connect(f'file:{bag}?mode=ro&immutable=1', uri=True)
    topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
    receipts, poses = collections.defaultdict(list), []
    pose_id = next((i for i, (n, _) in topics.items() if n == sc.TOPIC_POSE), None)
    pose_type = get_message(topics[pose_id][1]) if pose_id is not None else None
    for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages '
                                     'ORDER BY timestamp, id'):
        receipts[topics[tid][0]].append(ts / 1e9)
        if tid == pose_id:
            codes, p = body_pose_sample(deserialize_message(data, pose_type).transforms, cfg)
            if not codes:
                poses.append((ts / 1e9, p))
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
    jts = [(j.receipt_ns / 1e9 - off, dict(zip(j.names, j.positions)), j.stamp_ns / 1e9)
           for j in js]
    sim_pts = [(t, s) for t, _, s in jts]

    def sim_of(t):
        p = nearest(sim_pts, t, sc.JS_STALE_S)
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
    requests = [{'request': e['request'], 't': e['wall'], 'message': e.get('message')}
                for e in ev if e['event'] == 'request']
    st_mono = [(s.receipt_ns / 1e9 - off, s.data) for s in statuses]
    fbk = collections.defaultdict(list)
    for c in cmds:
        if c.source == 'feedback':
            fbk[c.goal_id].append((c.receipt_ns / 1e9 - off, max(abs(x) for x in c.error)))
    gaps = sorted(b[0] - a[0] for v in fbk.values() for a, b in zip(v, v[1:]))
    period = statistics.median(gaps) if gaps else 0.05
    bounds = cycle_boundaries(ev)
    settle = [p['z'] for t, p in poses if bounds[0] is not None and bounds[0] - 10.0 <= t
              <= bounds[0]]
    z_stand = statistics.mean(settle) if settle else None
    log_lines = []
    for name in ('launch.log', 'launch.log.gz'):
        p = os.path.join(run, name)
        if os.path.exists(p):
            log_lines = (gzip.open(p, 'rt') if name.endswith('.gz') else open(p)).read() \
                .splitlines()
            break
    ctl = controller_order(log_lines, len(gl)) if log_lines else None
    A = {}
    A['A1'] = assess_runs(gl, faults, st_mono)
    A['A2'] = assess_cycles(bounds, poses, z_stand, tol + 0.05)
    proof = (estop_proof(gl[-1], gl, dict(fbk), key_write_bracket(drv), cancels, faults,
                         refusals, requests, teleop_responses(drv),
                         [d for d in drv if d.get('event') == 'estop_service'], ctl, tol, period)
             if gl else None)
    A['A3'] = assess_tracking(gl, dict(fbk), tol, proof)
    A['A4'] = sc.assess_continuity(gl, refusals)
    t0 = gl[0]['sent'] if gl else None
    t1 = gl[-1].get('result_t') if gl else None
    script = open(harness).read() if harness and os.path.exists(harness) else ''
    rec = re.search(r'ros2 bag record(.*?)>', script, re.S)
    requested = re.findall(r'(/[A-Za-z0-9_/]+)', rec.group(1)) if rec else []
    log_path = os.path.join(run, 'bag_record.log')
    blog = [re.sub(r'\x1b\[[0-9;]*m', '', ln) for ln in open(log_path)] \
        if os.path.exists(log_path) else []
    stamp = lambda ln: float(re.search(r'\[(\d+\.\d+)\]', ln).group(1)) - off  # noqa: E731
    started = [stamp(ln) for ln in blog if 'Recording...' in ln]
    stopped = [stamp(ln) for ln in blog if 'Recording stopped' in ln]
    subs = [stamp(ln) for ln in blog if 'Subscribed to topic' in ln]
    graphs = [(s.get('monitors') or {}).get('graph') for t, s in st_mono
              if t0 is not None and t1 is not None and t0 <= t <= t1]

    def pubs(name):
        p = os.path.join(run, 'captures', name)
        txt = open(p).read() if os.path.exists(p) else ''
        m = re.search(re.escape(sc.COMMAND_TOPIC) + r'.*?Publisher count: (\d+)', txt, re.S)
        return int(m.group(1)) if m else None
    js_gap = sc.max_gap(mono.get('/joint_states', []), t0, t1) if t0 and t1 else None
    pose_gap = sc.max_gap(mono.get(sc.TOPIC_POSE, []), t0, t1) if t0 and t1 else None
    support = {
        'requested_from_recorder': sc.COMMAND_TOPIC in requested,
        'recording_covers_goal_interval': bool(started and stopped) and t0 is not None
        and t1 is not None and started[0] <= t0 and stopped[0] >= t1,
        'discovery_demonstrated_after_start': bool(started) and any(t > started[0] for t in subs),
        'recorder_log_without_failure': bool(blog) and not any(
            re.search(r'\[(WARN|ERROR|FATAL)\]', ln) for ln in blog),
        'node_monitored_streams_continuous': js_gap is not None and pose_gap is not None
        and js_gap <= sc.JS_STALE_S and pose_gap <= sc.POSE_STALE_S,
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
    present = sc.COMMAND_TOPIC in [n for n, _ in topics.values()]
    A['A5'] = assess_ownership(counts, present, len(receipts.get(sc.COMMAND_TOPIC, [])), support)
    last = gl[-1] if gl else {}
    A['A6'] = sc.assess_hold(len(cancels) == EXPECTED_CANCELS and any(f['reason'] == 'estop'
                                                                      for f in faults),
                             last.get('status') == sc.STATUS_CANCELED, last.get('result_t'),
                             sim_of, poses, [(t, q) for t, q, _ in jts])
    A['A7'] = sc.assess_restore(open(restore_report).read() if restore_report and
                                os.path.exists(restore_report) else None)
    A['A8'] = sc.assess_library(sc.sha(os.path.join(run, 'node', 'library.json')),
                                sc.sha(disabled_library))
    return {'schema': SCHEMA, 'A': A, 'overall': overall(A), 'alignment': al,
            'feedback_period_s': period, 'bag': {**meta, 'sha256_after_pose_scan': after}}


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
    res = load_and_score(o.run, o.bag, o.disabled_library, o.restore_report, o.harness)
    os.makedirs(o.out, exist_ok=True)
    with open(os.path.join(o.out, 'summary.json'), 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
    print(json.dumps({k: v['verdict'] for k, v in res['A'].items()}), res['overall'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
