#!/usr/bin/env python3
"""Offline review of S3 acceptance A3, and of the recorded foot-travel metric, for one run.

Read-only and separate from the frozen scorer: scripts/s3/score_s3.py is imported unchanged, its
committed output is not touched, and the bag is opened read-only (SHA-256 recorded before and
after by the caller).

Timestamp semantics, from the code as run:
- Driver key record 'mono' (scripts/s2/s2_driver.py key() and log()): time.monotonic() at log()
  ENTRY, taken after os.write() has RETURNED, rounded to 0.1 ms. It is an upper bound on the write
  completion. Neither the write start nor its completion is recorded. The main thread's previous
  record (the 'reached' record of the wait that precedes the key) is taken before the write is
  issued, so it is a lower bound on the write start.
- Node 'fault', 'transition', 'cancel_sent' and 'request' (m55_rclpy._on_request, then
  session.request): time.monotonic() in the node's service callback, after its graph snapshot.
  'cancel_sent' is the cancel DECISION. With no goal handle yet, the transport queues it and sends
  it from the goal-response callback, at a time that is not recorded.
- Node 'goal_response', 'cancel_response', 'result' and feedback: appended by executor callbacks
  without a time, and stamped at the 20 Hz session tick that polled them.
- Controller (server) times: only its own log lines (ROS system time), mapped onto
  CLOCK_MONOTONIC with the bag alignment offset (uncertainty: the alignment spread).
- Driver and node read the same host-wide CLOCK_MONOTONIC.

Retrospective assessment under criteria revision S3-r1 (PROPOSED, not adopted; the original
verdict stands). The frozen final-estop exception changes in one place: the point time t_x becomes
the scripted key's write bracket (t_pre, t_post]. The estop fault and the cancel decision must come
after the bracket's earliest possible start, t_pre plus its 0.05 ms rounding. Provenance checks are
added:
- exactly one scripted x and no other estop key;
- exactly one node estop request, at the fault;
- exactly one teleop "[estop] ok" response carrying the node's message;
- no driver estop-service fallback;
- the controller's log order: every goal received and accepted, then exactly one cancel request
  after the last acceptance.
Every other A3 rule is unchanged. Goals 1..N-1 need attributed feedback. The final goal's tracking
stays unmeasured (N/A), never a value. A6 is assessed separately and never stands in for tracking.

Usage: review_a3.py --evidence <committed evidence dir> --bag <bag_0.db3> --out <new dir>"""
import argparse
import collections
import gzip
import json
import math
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..', 's2'), HERE]
import score_s3 as sc  # noqa: E402  (frozen; imported unchanged)

REVISION = 'S3-r1'
LABEL = f'Retrospective assessment under criteria revision {REVISION}'
ROUND_S = 0.00005                    # the driver log rounds time.monotonic() to 4 decimals
LEG_FOOT = {'front_left': 'lf_foot_1', 'front_right': 'rf_foot_1',
            'rear_left': 'lr_foot_1', 'rear_right': 'rr_foot_1'}
ESTOP_KEYS = ("'x'", "'\\x1b'")      # teleop KEYMAP: x and Esc request the estop service


# ------------------------------------------------------------------ evidence extraction (pure)
def key_bracket(driver_records, why='estop exercise'):
    """The scripted estop key's write bracket from the driver log (file order = log order).
    t_post: the key record's post-write time. t_pre: the main thread's last 'reached' record before
    it, with no other key in between (logged before the write was issued)."""
    keys = [(i, d) for i, d in enumerate(driver_records) if 'key' in d]
    estop = [(i, d) for i, d in keys if d['key'] in ESTOP_KEYS]
    scripted = [(i, d) for i, d in estop if why in (d.get('why') or '')]
    out = {'n_estop_keys': len(estop), 'n_scripted_x': len(scripted), 't_pre': None,
           't_post': None, 'pre_record': None}
    if len(scripted) != 1:
        return out
    i, d = scripted[0]
    out['t_post'] = d['mono']
    for j in range(i - 1, -1, -1):
        r = driver_records[j]
        if 'key' in r:
            break
        if 'reached' in r:
            out['t_pre'], out['pre_record'] = r['mono'], r['reached']
            break
    return out


def teleop_responses(driver_records, name='estop'):
    """'[name] ok: msg' / '[name] REFUSED: msg' lines the teleop printed (driver pump receipt)."""
    out = []
    for d in driver_records:
        for ln in re.split(r'[\r\n]+', d.get('teleop_out') or ''):
            m = re.match(r'\[%s\] (ok|REFUSED): (.*)$' % re.escape(name), ln.strip())
            if m:
                out.append({'t': d['mono'], 'ok': m.group(1) == 'ok', 'message': m.group(2)})
    return out


def controller_order(log_lines, n_goals, offset_s):
    """The controller's own log: every goal received and accepted, then exactly one cancel request,
    after the last acceptance (same process clock). Times mapped onto CLOCK_MONOTONIC."""
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
    mono = None if offset_s is None else {
        'last_received': rec[-1] - offset_s if rec else None,
        'last_accepted': acc[-1] - offset_s if acc else None,
        'cancel_request': can[0] - offset_s if can else None}
    return {'ok': ok, 'received': len(rec), 'accepted': len(acc), 'cancel_requests': len(can),
            'mapped_mono_s': mono}


# ------------------------------------------------------------------ the S3-r1 exception (pure)
def estop_bracket_proof(g, goals, feedback, xb, cancels, faults, refusals, requests, teleop,
                        fallback, controller, tol, period):
    t_sent, t_resp, t_res = g.get('sent'), g.get('response_t'), g.get('result_t')
    t_pre, t_post = xb.get('t_pre'), xb.get('t_post')
    lo = None if t_pre is None else t_pre + ROUND_S
    estop = [f for f in faults if f['reason'] == 'estop']
    planned = [f for f in estop if lo is not None and f['t'] > lo
               and (t_res is None or f['t'] <= t_res)]
    unplanned = [f for f in faults if f not in planned and f['t'] <= (t_res or math.inf)]
    c = [k for k in cancels if k['seq'] == g['seq']]
    t_c = c[0]['t'] if len(c) == 1 else None
    req = [r for r in requests if r['request'] == 'estop']
    fb_window = [gid for gid, s in feedback.items() if None not in (t_sent, t_res)
                 and any(t_sent - tol <= t <= t_res + tol for t, _ in s)]
    t_f = planned[0]['t'] if len(planned) == 1 else None
    checks = {
        'final_goal_of_the_script': g['seq'] == sc.EXPECTED_GOALS == len(goals),
        'identity_is_the_estop_exercise': (g.get('b_from'), g.get('b_to'), g.get('direction'))
        == sc.ESTOP_GOAL and g.get('level') == 0,
        'accepted': g.get('accepted') is True,
        'one_scripted_estop_key_bracketed_in_goal': xb.get('n_scripted_x') == 1
        and xb.get('n_estop_keys') == 1 and None not in (t_pre, t_post, t_sent, t_res)
        and t_sent <= t_pre <= t_post <= t_res,
        'estop_fault_after_bracket_start': len(estop) == 1 and t_f is not None,
        'one_node_estop_request_at_the_fault': len(req) == 1 and t_f is not None
        and req[0]['t'] == t_f,
        'one_teleop_estop_response_after_the_fault': len(teleop) == 1 and t_f is not None
        and teleop[0]['ok'] and teleop[0]['t'] >= t_f and len(req) == 1
        and teleop[0]['message'] == req[0].get('message'),
        'no_driver_estop_service_fallback': not fallback,
        'exactly_one_cancel': len(cancels) == sc.EXPECTED_CANCELS and len(c) == 1,
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
            'times': {'goal_sent': t_sent, 'bracket_pre_logged': t_pre, 'bracket_start': lo,
                      'key_post_write_logged': t_post, 'estop_fault': t_f, 'cancel_decision': t_c,
                      'goal_response_polled': t_resp, 'result_polled': t_res}}


def tracking_r1(goals, feedback, tol, proof):
    """A3 under S3-r1: the frozen per-goal rule (feedback attributed by window, abort 0.05, finding
    0.02); the final goal is N/A only if the S3-r1 exception qualifies, else UNMEASURED."""
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
                       'feedback (S3-r1 exception); A6 is assessed separately')
        else:
            row['verdict'] = 'UNMEASURED'
            unmeasured = True
        rows.append(row)
    v = 'FAIL' if fail else ('UNMEASURED' if unmeasured or not goals else 'PASS')
    return {'verdict': v, 'goals': rows, 'findings_above_0_02_rad': findings,
            'unattributed_feedback_ids': unattributed}


# ------------------------------------------------------------------ foot travel (pure parts)
def stl_vertices(path, xyz):
    """Unique vertices of a binary STL (mm, scale 0.001) in the link frame (collision origin xyz,
    rpy 0)."""
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack_from('<I', data, 80)[0]
    pts = set()
    for k in range(n):
        v = struct.unpack_from('<12f', data, 84 + 50 * k)
        for j in (3, 6, 9):
            pts.add((round(v[j] * 0.001 + xyz[0], 7), round(v[j + 1] * 0.001 + xyz[1], 7),
                     round(v[j + 2] * 0.001 + xyz[2], 7)))
    return sorted(pts)


def foot_rows(goals, windows, frames, verts):
    """Per goal and foot: link-origin travel (the frozen metric) and the travel of the CAD tip's
    material point (the mesh vertex lowest in the world at the window start, tracked rigidly with
    the link). Stance/swing from the goal's leg name. frames: {foot: [(t, Pose)]}."""
    rows = []
    for g in goals:
        w = windows.get(g['seq'])
        if w is None:
            continue
        swing = LEG_FOOT.get(g.get('leg'))
        for foot, s in frames.items():
            fs = [(t, p) for t, p in s if w[0] <= t <= w[1]]
            if len(fs) < 2:
                continue
            p0 = fs[0][1]
            o = [(p.p[0], p.p[1]) for _, p in fs]
            vi = min(range(len(verts[foot])), key=lambda k: p0.apply(verts[foot][k])[2])
            tip = [p.apply(verts[foot][vi]) for _, p in fs]
            rows.append({
                'seq': g['seq'], 'kind': g.get('kind'), 'leg': g.get('leg'), 'foot': foot,
                'role': 'swing' if foot == swing else 'stance', 'samples': len(fs),
                'origin_travel_m': max(math.hypot(x - o[0][0], y - o[0][1]) for x, y in o),
                'tip_travel_m': max(math.hypot(x - tip[0][0], y - tip[0][1]) for x, y, _ in tip),
                'tip_z_min_m': min(z for _, _, z in tip), 'tip_z_max_m': max(z for _, _, z in tip),
                'tip_vertex_link_frame': verts[foot][vi]})
    return rows


# ------------------------------------------------------------------ loading (read-only)
def load(evidence, bag):
    import sqlite3
    import adjudicate_s2 as adj
    import response_law as rl
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from spiderx_controller import m55_locomotion as loc
    from spiderx_controller import m61a_fixed_base as fb
    cfg = loc.load_config()
    js, cmds, statuses, meta = rl.read_bag(bag)
    ev = [json.loads(ln) for ln in open(os.path.join(evidence, 'node', 'events.jsonl'))
          if ln.strip()]
    drv = [json.loads(ln) for ln in gzip.open(os.path.join(evidence, 'driver',
                                                           'driver.jsonl.gz'), 'rt')]
    al = adj.align([(d['mono'], d['status']) for d in drv if 'status' in d],
                   [(s.receipt_ns / 1e9, s.data) for s in statuses])
    off, tol = al['offset_s'], al['tolerance_s']
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
    fbk = collections.defaultdict(list)
    for c in cmds:
        if c.source == 'feedback':
            fbk[c.goal_id].append((c.receipt_ns / 1e9 - off, max(abs(x) for x in c.error)))
    gaps = sorted(b[0] - a[0] for v in fbk.values() for a, b in zip(v, v[1:]))
    period = sorted(gaps)[len(gaps) // 2] if gaps else 0.05
    con = sqlite3.connect(f'file:{bag}?mode=ro&immutable=1', uri=True)
    topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
    pid = next(i for i, (n, _) in topics.items() if n == sc.TOPIC_POSE)
    ptype = get_message(topics[pid][1])
    frames = {f: [] for f in sc.FEET}
    for ts, data in con.execute('SELECT timestamp, data FROM messages WHERE topic_id = ? '
                                'ORDER BY timestamp, id', (pid,)):
        m = deserialize_message(data, ptype)
        for f in sc.FEET:
            sel = fb.select_entries(m.transforms, cfg.monitor.model_name, f)
            if not sel.codes:
                frames[f].append((ts / 1e9 - off, sel.model * sel.body_link))
    con.close()
    return {'goals': gl, 'feedback': dict(fbk), 'events': ev, 'driver': drv, 'alignment': al,
            'offset': off, 'tol': tol, 'period': period, 'frames': frames, 'bag_meta': meta}


def urdf_collision_xyz(urdf_text, link):
    m = re.search(r'<link name="%s".*?<collision>\s*<origin xyz="([^"]+)"' % link, urdf_text,
                  re.S)
    return tuple(float(v) for v in m.group(1).split())


def main(argv=None):
    a = argparse.ArgumentParser()
    a.add_argument('--evidence', required=True)
    a.add_argument('--bag', required=True)
    a.add_argument('--out', required=True)
    a.add_argument('--description', default=os.path.join(HERE, '..', '..', 'src',
                                                         'spiderx_description'))
    o = a.parse_args(argv)
    os.makedirs(o.out, exist_ok=True)
    d = load(o.evidence, o.bag)
    ev, drv, gl = d['events'], d['driver'], d['goals']
    faults = [{'reason': e['reason'], 't': e['wall']} for e in ev if e['event'] == 'fault']
    refusals = [{'request': e['request'], 'reason': e['reason'], 't': e['wall']}
                for e in ev if e['event'] == 'refused']
    cancels = [{'seq': e['seq'], 't': e['wall']} for e in ev if e['event'] == 'cancel_sent']
    requests = [{'request': e['request'], 't': e['wall'], 'message': e.get('message')}
                for e in ev if e['event'] == 'request']
    keys = [{'key': x['key'].strip("'"), 't': x['mono'], 'why': x.get('why')}
            for x in drv if 'key' in x]
    fallback = [x for x in drv if x.get('event') == 'estop_service']
    xb = key_bracket(drv)
    tel = teleop_responses(drv)
    log = gzip.open(os.path.join(o.evidence, 'run', 'launch.log.gz'), 'rt').read().splitlines()
    ctl = controller_order(log, len(gl), d['offset'])
    last = gl[-1]
    frozen = sc.estop_na_proof(last, gl, d['feedback'], keys, cancels, faults, refusals,
                               d['tol'], d['period'])
    frozen_inputs = {'t_x_used_by_frozen_proof': next((k['t'] for k in keys if k['key'] == 'x'),
                                                      None),
                     'estop_fault_t': [f['t'] for f in faults if f['reason'] == 'estop'],
                     'cancel_decision_t': [c['t'] for c in cancels]}
    proof = estop_bracket_proof(last, gl, d['feedback'], xb, cancels, faults, refusals, requests,
                                tel, fallback, ctl, d['tol'], d['period'])
    a3_frozen = sc.assess_tracking(gl, d['feedback'], keys, cancels, faults, refusals, d['tol'],
                                   d['period'])
    a3_r1 = tracking_r1(gl, d['feedback'], d['tol'], proof)
    # foot travel
    urdf = open(os.path.join(o.description, 'urdf', 'spiderx.urdf.xacro')).read()
    verts = {f: stl_vertices(os.path.join(o.description, 'meshes', f + '.stl'),
                             urdf_collision_xyz(urdf, f)) for f in sc.FEET}
    windows = {g['seq']: (g['sent'], g['result_t']) for g in gl if g.get('result_t') is not None}
    rows = foot_rows(gl, windows, d['frames'], verts)
    leg_names = sorted({str(g.get('leg')) for g in gl})
    frozen_swing_map = {'lf': 'lf_foot_1', 'rf': 'rf_foot_1', 'lr': 'lr_foot_1', 'rr': 'rr_foot_1'}

    def mx(rs, k):
        r = max(rs, key=lambda x: x[k]) if rs else None
        return None if r is None else {k: r[k], 'seq': r['seq'], 'kind': r['kind'],
                                       'leg': r['leg'], 'foot': r['foot'], 'role': r['role']}
    st = [r for r in rows if r['role'] == 'stance']
    sw = [r for r in rows if r['role'] == 'swing']
    foot = {
        'leg_names_in_events': leg_names,
        'frozen_swing_map_keys': sorted(frozen_swing_map),
        'frozen_swing_exclusion_matches_any_leg': any(n in frozen_swing_map for n in leg_names),
        'rows': len(rows), 'stance_rows': len(st), 'swing_rows': len(sw),
        'max_origin_travel_all_rows': mx(rows, 'origin_travel_m'),
        'max_origin_travel_stance': mx(st, 'origin_travel_m'),
        'max_tip_travel_stance': mx(st, 'tip_travel_m'),
        'max_tip_travel_swing': mx(sw, 'tip_travel_m'),
        'stance_tip_z_range_m': [min(r['tip_z_min_m'] for r in st),
                                 max(r['tip_z_max_m'] for r in st)],
        'swing_tip_z_max_m': max(r['tip_z_max_m'] for r in sw) if sw else None,
        'frame_composition': 'world = model pose (child spiderx) * link pose relative to the '
                             'model (m61a_fixed_base.select_entries), as the frozen scorer',
        'tracked_points': {'origin': 'the foot link origin = its joint axis (rf_foot_joint '
                                     'origin xyz 0.012 -0.0778 -0.0778 in the thigh frame)',
                           'tip': 'the CAD mesh vertex lowest in the world at the window start, '
                                  'carried rigidly with the link'}}
    with open(os.path.join(o.out, 'foot_travel.csv'), 'w') as f:
        cols = ['seq', 'kind', 'leg', 'foot', 'role', 'samples', 'origin_travel_m',
                'tip_travel_m', 'tip_z_min_m', 'tip_z_max_m']
        f.write(','.join(cols) + '\n')
        for r in rows:
            f.write(','.join(str(r[c]) for c in cols) + '\n')
    res = {'label': LABEL, 'adopted': False,
           'original_verdict': 'INCONCLUSIVE (frozen scorer; unchanged)',
           'alignment': d['alignment'], 'feedback_period_s': d['period'],
           'key_bracket': xb, 'teleop_estop_responses': tel, 'driver_estop_fallback': fallback,
           'node_estop_requests': [r for r in requests if r['request'] == 'estop'],
           'controller_log': ctl,
           'frozen_proof_recomputed': frozen, 'frozen_proof_inputs': frozen_inputs,
           'frozen_A3_recomputed_verdict': a3_frozen['verdict'],
           'S3_r1_proof': proof, 'S3_r1_A3': {k: v for k, v in a3_r1.items() if k != 'goals'},
           'S3_r1_A3_goal_rows': a3_r1['goals'],
           'foot_travel': foot, 'bag_meta': d['bag_meta']}
    with open(os.path.join(o.out, 'review.json'), 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
    print(json.dumps({'frozen_A3': a3_frozen['verdict'], 'frozen_failed': frozen['failed'],
                      LABEL: a3_r1['verdict'], 'S3_r1_failed': proof['failed']}))


if __name__ == '__main__':
    main()
