#!/usr/bin/env python3
"""Score ONE S4c level (docs/S4_KEYBOARD_TELEOP_PLAN.md, section 3.3), offline. Specified BEFORE any
S4c run. One invocation scores one level, named explicitly: --level 1 or --level 2, the node's
internal index (teleop labels 2/3 and 3/3).

Reuses the S4b scorer's extraction (score_s4b.py) and the S4a and frozen S3 scorers' generic parts
(score_s4a.py, score_s3.py), all unchanged. Each check is True, False or None (missing evidence).
Any False makes the check FAIL; otherwise any None makes it UNMEASURED; otherwise it is PASS. The
overall is FAIL, PASS or INCONCLUSIVE.

- C1 plan and budget.
  - Exactly the 18 planned goals in order (0->1 ... 8->9, then 9->8 ... 1->0), each SUCCEEDED.
  - 0 cancels, no fault, no refusal; one goal per seq; the event order is consistent.
  - The keys are exactly '+' x L, a, w, space, s, space, q, each written completely (result 1).
  - Before 'q', the requests are exactly arm, stop, stop: the arm before goal 1 and the two
    boundary stops. Never estop, reset or home.
  - Every status the driver received reports dispatch with the gate True; with --shadow, shadow
    with the gate False.
- C2 level.
  - The run's recorded level equals --level, and the node's stride for it is 40 or 60 mm.
  - Exactly L '+' keys, no '-' or '='. Each is written while the node is DISARMED with no goal,
    before the 'a' key and before the arm request.
  - Each '+' is confirmed by the teleop's own line "speed level <index + 1>/3: <speed> mm/s",
    after its write and before the next key: 2/3, then 3/3 at level 2, at the node's speeds.
  - Exactly one node level_change, 0 -> L, after the arm and at or before goal 1.
  - Every goal at level L.
- C3 forward cycle and C4 reverse return cycle. Each checks:
  - the space was written while the cycle's last goal (8->9, or 1->0) was in flight;
  - exactly one STOPPING ('stop') during that goal;
  - READY ('stopped') at boundary 0, within the goal's duration_s + 1.0 s of sim (check 1.7, from
    the last status before the space) and a 120 s wall backstop;
  - no goal between the space and READY;
  - the node's cycle count changed by +1 (forward) or -1 (reverse);
  - the driver's cycle check came before the next key ('s', or 'q').
  Displacement is measured along body +y in the frame of the cycle's start boundary, with yaw
  wrapped. The forward cycle runs from the arm to its READY. The reverse runs from the forward READY
  to its READY, never from the arm pose.
  - Bands: level 1 forward [30, 50] mm, reverse [-50, -30] mm; level 2 [50, 70] and [-70, -50].
  - |lateral| <= 10 mm, |yaw| <= 0.05 rad, height within +-10 mm of standing, tilt <= 0.10 rad.
  - The live stop band (forward [0, 60] or [0, 80] mm, reverse [-60, 0] or [-80, 0]) is recorded.
    The net return error (arm to the end of the reverse) is recorded only: no exact return is
    assumed.
  - With --shadow the signed bands are not applied. |progress| <= the outer stop bound and the
    drift limits still apply.
- C5 close.
  - 'q' was written at READY at boundary 0, with no goal, after the reverse cycle's READY.
  - After it come exactly the close requests (stop x 2 and disarm x 2, as m55_teleop sends them),
    with no refusal, and exactly one transition: READY -> DISARMED ('disarm').
  - The teleop's own exit is recorded with code 0.
  - No goal, key or other request after 'q'.
  - Every later status is DISARMED with no goal, over at least 10 s of sim, and the robot is still
    over those 10 s (the frozen S3 hold limits, measured from the bag).
- C6 tracking: every goal has attributed feedback (no N/A); max |error| <= 0.05 rad; > 0.02 is a
  finding. With --shadow: not applicable, since no trajectory runs (C8 requires zero feedback).
- C7 continuity (S3 A4).
- C8 ownership.
  - 18 goals, 0 cancels, no foreign client or command publisher.
  - The controller's log shows 18 received, 18 accepted and 0 cancel requests; with --shadow, 0, 0
    and 0, and no feedback in the bag.
  - 0 joint_trajectory messages. An absent topic counts only with the S4a support evidence.
- C9 margins and freshness.
  - The minimum measured joint margin to the URDF limits is at least 0.02 rad (bag).
  - The node-observed maximum ages from arm to close are within its monitor limits: joint states
    0.5 s, pose 1.0 s, graph 3.0 s.
  - The driver's result is 'script complete'.
- C10 restore (S3 A7); C11 library (S3 A8).
Missing evidence is UNMEASURED; a contradiction FAILs. No threshold changes after a run.

Usage: score_s4c.py --level {1,2} --run <run dir> --bag <bag_0.db3> --disabled-library <file>
                    --restore-report <report> --harness <run script> --out <new dir> [--shadow]"""
import argparse
import collections
import gzip
import json
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 's3'), os.path.join(HERE, '..', 's2')]
import score_s4b as sb  # noqa: E402  (unchanged; brings the S4a and S3 scorers)

s4a, sc = sb.s4a, sb.sc
nearest, body_delta, verdict_of = sb.nearest, sb.body_delta, sb.verdict_of
SCHEMA = 'spiderx.s4c_score/1'
P = 9
FORWARD, REVERSE = 1, -1
LEVELS = {
    1: {'stride_m': 0.040, 'teleop_label': '2/3',
        'band_m': {FORWARD: (0.030, 0.050), REVERSE: (-0.050, -0.030)},
        'stop_m': {FORWARD: (0.0, 0.060), REVERSE: (-0.060, 0.0)}},
    2: {'stride_m': 0.060, 'teleop_label': '3/3',
        'band_m': {FORWARD: (0.050, 0.070), REVERSE: (-0.070, -0.050)},
        'stop_m': {FORWARD: (0.0, 0.080), REVERSE: (-0.080, 0.0)}},
}
FORWARD_CYCLE = tuple((b, b + 1, FORWARD) for b in range(P))
REVERSE_CYCLE = tuple((b, b - 1, REVERSE) for b in range(P, 0, -1))
PLANNED = FORWARD_CYCLE + REVERSE_CYCLE
EXPECTED_GOALS, EXPECTED_CANCELS = len(PLANNED), 0
CLOSE_REQUESTS = collections.Counter({'stop': 2, 'disarm': 2})
LEVEL_KEYS = ("'+'", "'-'", "'='")
CONFIRM_RE = re.compile(r'speed level (\d+)/(\d+): ([0-9.]+) mm/s')
SIM_MARGIN_S, WALL_BACKSTOP_S = sb.SIM_MARGIN_S, sb.WALL_BACKSTOP_S        # 1.0 s, 120 s
JOINT_MARGIN_MIN_RAD = 0.02                       # the S3 joint-limit stop
AGE_LIMITS_S = {'joint_states_age_s': sc.JS_STALE_S, 'body_pose_age_s': sc.POSE_STALE_S,
                'graph_age_s': sc.GRAPH_STALE_S}
SCRIPT_COMPLETE = ('script complete',
                   'script complete (shadow rehearsal: dispatch disabled; not acceptance)')
SHADOW_NA = 'not applicable: dispatch disabled (shadow rehearsal)'


def scripted_keys(level):
    return ["'+'"] * level + ["'a'", "'w'", "' '", "'s'", "' '", "'q'"]


def outer_bound_m(level):
    return max(abs(LEVELS[level]['stop_m'][REVERSE][0]), abs(LEVELS[level]['stop_m'][FORWARD][1]))


def status_before(sts, t):
    """The last status the driver received at or before t."""
    s = [st for mono, _, st in sts if mono <= t]
    return s[-1] if s else None


# ------------------------------------------------------------------ C1
def assess_plan(ev, goals, cancels, faults, refusals, keys, requests, sts, level, shadow):
    got = [(g['b_from'], g['b_to'], g['direction']) for g in goals.values()]
    sent_seqs = [e.get('seq') for e in ev if e['event'] == 'goal_sent']
    arm = [r for r in requests if r['request'] == 'arm']
    first_sent = next(iter(goals.values()))['sent'] if goals else None
    q = next((k for k in keys if k['key'] == "'q'"), None)
    before_q = [r['request'] for r in requests if q is None or r['t'] < q['t_before_write']]
    modes = {(st.get('mode'), st.get('dispatch_gate')) for _, _, st in sts}
    mode_name = 'node_shadow_gate_false_throughout' if shadow \
        else 'node_dispatch_gate_true_throughout'
    checks = {
        'eighteen_planned_goals_in_order': got == list(PLANNED) if goals else None,
        'all_succeeded': all(g.get('status') == sb.STATUS_SUCCEEDED for g in goals.values())
        if goals else None,
        'no_cancel': len(cancels) == EXPECTED_CANCELS,
        'no_fault': not faults,
        'no_refusal': not refusals,
        'keys_exactly_as_scripted': [k['key'] for k in keys] == scripted_keys(level)
        if keys else None,
        'every_key_write_complete': all(k.get('write_result') == 1 for k in keys)
        if keys else None,
        'requests_before_q_exactly_arm_stop_stop': before_q == ['arm', 'stop', 'stop']
        if requests else None,
        'arm_once_before_goal_1': len(arm) == 1 and first_sent is not None
        and arm[0]['t'] <= first_sent,
        'one_goal_sent_per_seq': len(sent_seqs) == len(set(sent_seqs)) if sent_seqs else None,
        'event_order_consistent': sb.consistent_order(ev, goals) if goals else None,
        mode_name: None if not sts else modes == {('shadow', False) if shadow
                                                  else ('dispatch', True)},
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'goals': got,
            'cancels': len(cancels), 'faults': [f['reason'] for f in faults]}


# ------------------------------------------------------------------ C2
def confirmations(drv):
    """[(mono, label index, count, speed text)] from the teleop's own output, in order."""
    out = []
    for d in drv:
        for m in CONFIRM_RE.finditer(d.get('teleop_out') or ''):
            out.append((d['mono'], int(m.group(1)), int(m.group(2)), m.group(3)))
    return out


def assess_level(ev, goals, keys, requests, transitions, drv, sts, level, run_level):
    spec = LEVELS[level]
    plus = [k for k in keys if k['key'] == "'+'"]
    other = [k for k in keys if k['key'] in LEVEL_KEYS and k['key'] != "'+'"]
    a_key = next((k for k in keys if k['key'] == "'a'"), None)
    arm_req = next((r['t'] for r in requests if r['request'] == 'arm'), None)
    t_arm = next((tr['t'] for tr in transitions if tr['why'] == 'arm'), None)
    first_sent = next(iter(goals.values()))['sent'] if goals else None
    lc = [e for e in ev if e['event'] == 'level_change']
    conf = confirmations(drv)
    st0 = next((st for _, _, st in sts if st.get('levels_m_s')), None)
    speeds = (st0 or {}).get('levels_m_s') or []
    strides = (st0 or {}).get('strides_m') or []

    def confirmed():
        if not plus or not conf:
            return None
        if len(conf) != len(plus):
            return False
        nxt = [k['t_before_write'] for k in keys if k['t_before_write'] > plus[-1]['t_after_write']]
        for i, (k, c) in enumerate(zip(plus, conf), start=1):
            limit = plus[i]['t_before_write'] if i < len(plus) else (nxt[0] if nxt else None)
            if not (k['t_after_write'] <= c[0] and (limit is None or c[0] <= limit)):
                return False
            if (c[1], c[2]) != (i + 1, 3) or i >= len(speeds) \
                    or c[3] != f'{speeds[i] * 1000:.2f}':
                return False
        return True

    def while_disarmed(k):
        st = status_before(sts, k['t_before_write'])
        return None if st is None else (st.get('state') == 'DISARMED' and st.get('goal') is None)
    states = [while_disarmed(k) for k in plus]
    checks = {
        'recorded_level_equals_the_requested_level': None if run_level is None
        else run_level == level,
        'node_stride_for_the_level': None if len(strides) <= level
        else abs(strides[level] - spec['stride_m']) <= 1e-6,
        'exactly_L_plus_keys_and_no_other_level_key': len(plus) == level and not other,
        'level_keys_before_the_arm_key_and_request': None if a_key is None or arm_req is None
        else all(k['t_after_write'] <= a_key['t_before_write'] and k['t_after_write'] <= arm_req
                 for k in plus),
        'level_keys_while_disarmed_without_goal': None if not plus or None in states
        else all(states),
        'each_level_key_confirmed_by_the_teleop': confirmed(),
        'one_level_change_0_to_L_between_arm_and_goal_1': None if t_arm is None
        or first_sent is None else (len(lc) == 1 and lc[0].get('previous') == 0
                                    and lc[0].get('level') == level
                                    and t_arm <= lc[0]['wall'] <= first_sent),
        'every_goal_at_level_L': all(g.get('level') == level for g in goals.values())
        if goals else None,
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'level': level,
            'teleop_label': spec['teleop_label'], 'stride_m': spec['stride_m'],
            'confirmations': [{'label': f'{c[1]}/{c[2]}', 'speed_mm_s': c[3]} for c in conf]}


# ------------------------------------------------------------------ C3, C4
def assess_boundary_stop(direction, cycle_goals, space, transitions, sent_times, sts,
                         cycle_check, next_key):
    """The cycle's end: space while its last goal was in flight, READY ('stopped') at 0."""
    last = cycle_goals[-1] if len(cycle_goals) == P else None
    if last is None or space is None or last.get('result_t') is None:
        return {'checks': {'cycle_goals_and_space_recorded': None},
                'reason': 'the cycle goals, the last result or the space record is missing'}
    tb, ta = space['t_before_write'], space['t_after_write']
    stopping = [tr for tr in transitions if tr['state'] == 'STOPPING'
                and last['sent'] <= tr['t'] <= last['result_t']]
    ready = next((tr for tr in transitions if tr['t'] >= ta and tr['state'] == 'READY'), None)
    s0 = sb.sim_before(sts, tb)
    _, s1 = sb.first_status(sts, ta, lambda st: st.get('state') == 'READY'
                            and st.get('boundary') == 0 and st.get('goal') is None)
    bound = (last.get('duration_s') or 0.0) + SIM_MARGIN_S
    later = [t for t in sent_times if t > tb and (ready is None or t < ready['t'])]
    c_before = status_before(sts, cycle_goals[0]['sent'])
    c_after = None if ready is None else next(
        (st for mono, _, st in sts if mono >= ready['t'] and st.get('state') == 'READY'), None)
    checks = {
        'space_written_while_the_last_goal_was_in_flight': last['sent'] <= tb <= ta
        <= last['result_t'] and space.get('write_result') == 1,
        'one_stopping_stop_in_the_last_goal': len(stopping) == 1
        and stopping[0]['why'] == 'stop',
        'ready_stopped_at_boundary_0': None if ready is None else (
            ready['why'] == 'stopped' and last.get('done_b') == 0),
        'ready_within_sim_bound': None if None in (s0, s1) else s1 - s0 <= bound,
        'ready_within_wall_backstop': None if ready is None
        else ready['t'] - tb <= WALL_BACKSTOP_S,
        'no_goal_between_the_space_and_ready': not later if ready is not None else None,
        'cycle_count_changed_by_the_direction': None if c_before is None or c_after is None
        or None in (c_before.get('cycles'), c_after.get('cycles'))
        else c_after['cycles'] - c_before['cycles'] == direction,
        'cycle_checked_before_the_next_key': None if cycle_check is None or next_key is None
        else (cycle_check.get('direction') == direction
              and cycle_check['mono'] <= next_key['t_before_write']),
    }
    return {'checks': checks, 'ready_sim_s': None if None in (s0, s1) else s1 - s0,
            'sim_bound_s': bound, 'ready_t': None if ready is None else ready['t']}


def assess_displacement(direction, level, t0, t1, poses, z_stand, tol, shadow):
    """Progress along body +y in the frame of the start boundary, and drift, from the bag."""
    spec = LEVELS[level]
    if None in (t0, t1, z_stand):
        return {'verdict': 'UNMEASURED', 'reason': 'need the start and end times and z_stand'}
    a, b = nearest(poses, t0, tol), nearest(poses, t1, tol)
    inside = [p['tilt'] for t, p in poses if t0 <= t <= t1]
    if a is None or b is None or not inside:
        return {'verdict': 'UNMEASURED', 'reason': 'no pose at a boundary or in the cycle'}
    fwd, lat, dyaw = body_delta(a[1], b[1])
    lo, hi = spec['band_m'][direction]
    slo, shi = spec['stop_m'][direction]
    r = {'forward_m': fwd, 'lateral_m': lat, 'yaw_rad': dyaw, 'z_end_m': b[1]['z'],
         'height_vs_standing_m': b[1]['z'] - z_stand, 'max_tilt_rad': max(inside),
         'band_forward_m': [lo, hi], 'live_stop_band_m': [slo, shi],
         'within_live_stop_band_recorded': slo <= fwd <= shi}
    checks = {
        'progress_in_band': None if shadow else lo <= fwd <= hi,
        'progress_within_the_outer_bound': abs(fwd) <= outer_bound_m(level),
        'lateral_within_10_mm': abs(lat) <= sc.BAND_LATERAL_M,
        'yaw_within_0_05_rad': abs(dyaw) <= sc.BAND_YAW_RAD,
        'height_within_10_mm': abs(r['height_vs_standing_m']) <= sc.BAND_HEIGHT_M,
        'tilt_within_0_10_rad': r['max_tilt_rad'] <= sc.TILT_MAX_RAD,
    }
    if shadow:
        checks.pop('progress_in_band')
        r['progress_band'] = SHADOW_NA
    r.update(checks=checks, verdict=verdict_of(checks))
    return r


def assess_cycle(direction, boundary, displacement):
    checks = dict(boundary['checks'])
    checks['displacement'] = None if displacement is None or \
        displacement['verdict'] == 'UNMEASURED' else displacement['verdict'] == 'PASS'
    return {'verdict': verdict_of(checks), 'direction': direction, 'checks': checks,
            'ready_sim_s': boundary.get('ready_sim_s'), 'sim_bound_s': boundary.get('sim_bound_s'),
            'displacement': displacement}


# ------------------------------------------------------------------ C5
def assess_close(q, r_ready_t, goals, keys, requests, refusals, transitions, drv, sts, still):
    if q is None:
        return {'verdict': 'UNMEASURED', 'reason': "no 'q' key record"}
    tq = q['t_before_write']
    st_q = status_before(sts, tq)
    after_req = collections.Counter(r['request'] for r in requests if r['t'] >= tq)
    tr_after = [(tr['previous'], tr['state'], tr['why']) for tr in transitions if tr['t'] >= tq]
    dis = next((tr for tr in transitions if tr['t'] >= tq and tr['state'] == 'DISARMED'), None)
    closed = [d for d in drv if d.get('event') == 'teleop_closed']
    t1 = None if dis is None else sb.first_status(
        sts, dis['t'], lambda st: st.get('state') == 'DISARMED')[0]     # as S4b's lease window
    after = [st for mono, _, st in sts if t1 is not None and mono >= t1]
    sims = [sim for mono, sim, _ in sts if t1 is not None and mono >= t1 and sim is not None]
    checks = {
        'q_at_ready_boundary_0_without_goal': None if st_q is None else (
            st_q.get('state') == 'READY' and st_q.get('boundary') == 0
            and st_q.get('goal') is None),
        'q_after_the_reverse_ready': None if r_ready_t is None else r_ready_t <= tq,
        'q_written_completely': q.get('write_result') == 1,
        'exactly_the_close_requests_after_q': after_req == CLOSE_REQUESTS,
        'no_refusal_after_q': not any(r['t'] >= tq for r in refusals),
        'one_transition_ready_to_disarmed_disarm': tr_after == [('READY', 'DISARMED',
                                                                 'disarm')],
        'teleop_exit_code_0': None if not closed else (len(closed) == 1
                                                       and closed[0].get('exit_code') == 0),
        'no_goal_after_q': not any(g['sent'] > tq for g in goals.values()),
        'no_key_after_q': not any(k['t_before_write'] > tq for k in keys),
        'disarmed_without_goal_over_10_s_of_sim': None if not after or len(sims) < 2 else (
            all(st.get('state') == 'DISARMED' and st.get('goal') is None for st in after)
            and sims[-1] - sims[0] >= sc.HOLD_SIM_S - 0.5),
        'still_after_the_close': None if still is None or still['verdict'] == 'UNMEASURED'
        else still['verdict'] == 'PASS',
    }
    return {'verdict': verdict_of(checks), 'checks': checks,
            'requests_after_q': dict(after_req), 'transitions_after_q': tr_after,
            'teleop_closed': closed[0] if len(closed) == 1 else closed, 'still': still}


# ------------------------------------------------------------------ C8
def assess_ownership(counts, topic_present, topic_count, support, ctl, feedback_n, shadow):
    n = 0 if shadow else EXPECTED_GOALS
    terms = {'goals_18': counts.get('goals_sent') == EXPECTED_GOALS,
             'cancels_0': counts.get('cancels_sent') == EXPECTED_CANCELS,
             'foreign_clients_0': counts.get('max_foreign') == 0,
             'command_publishers_0': counts.get('max_publishers') == 0,
             f'controller_log_{n}_received_{n}_accepted_0_cancel': None if ctl is None else (
                 ctl['received'] == n and ctl['accepted'] == n and ctl['cancel_requests'] == 0)}
    if shadow:
        terms['no_feedback_in_the_bag'] = None if feedback_n is None else feedback_n == 0
    if topic_present:
        jt = 'PASS' if topic_count == 0 else 'FAIL'
    else:
        jt = 'PASS' if support and all(support.values()) else 'UNMEASURED'
    v = verdict_of({**terms, 'command_topic': {'PASS': True, 'FAIL': False}.get(jt)})
    return {'verdict': v, 'terms': terms, 'command_topic': {
        'present_in_bag': topic_present, 'messages': topic_count if topic_present else None,
        'term': jt, 'support': None if topic_present else support}}


# ------------------------------------------------------------------ C9
def assess_margins(min_margin, sts, t_arm, t_close, result):
    window = [st for mono, _, st in sts
              if None not in (t_arm, t_close) and t_arm <= mono <= t_close]
    ages = {k: max(((st.get('monitors') or {}).get(k) for st in window
                    if isinstance((st.get('monitors') or {}).get(k), (int, float))), default=None)
            for k in AGE_LIMITS_S}
    checks = {
        'min_joint_margin_at_least_0_02_rad': None if min_margin is None
        else min_margin[0] >= JOINT_MARGIN_MIN_RAD,
        'node_observed_ages_within_monitor_limits': None if not window
        or None in ages.values() else all(ages[k] <= v for k, v in AGE_LIMITS_S.items()),
        'driver_script_complete': None if result is None else result in SCRIPT_COMPLETE,
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'max_ages_s': ages,
            'min_joint_margin': min_margin, 'driver_result': result}


def overall(C):
    vs = [c['verdict'] for c in C.values() if c['verdict'] != 'N/A']
    return 'FAIL' if 'FAIL' in vs else ('PASS' if all(v == 'PASS' for v in vs)
                                        else 'INCONCLUSIVE')


# ------------------------------------------------------------------ the run (pure part)
def run_level(drv):
    sel = next((d['selection'] for d in drv if isinstance(d.get('selection'), dict)), None)
    if sel is not None:
        return sel.get('level')
    s = next((d['summary'] for d in drv if isinstance(d.get('summary'), dict)), None)
    return None if s is None else s.get('level')


def score_events(ev, drv, level, motion=None, shadow=False):
    """C1-C5 from the node events and the driver log. motion: the bag-based parts, or None (then
    they are UNMEASURED): {'forward': displacement, 'reverse': displacement, 'still': still}.
    Used by load_and_score and by the tests. Returns (C, context)."""
    if level not in LEVELS:
        raise ValueError(f'S4c scores level 1 or 2 (internal index), not {level!r}')
    motion = motion or {}
    goals = sb.goals_from_events(ev)
    gl = list(goals.values())
    transitions = sb.transitions_from_events(ev)
    requests = [{'request': e['request'], 't': e['wall']} for e in ev if e['event'] == 'request']
    refusals = [{'request': e['request'], 'reason': e['reason'], 't': e['wall']}
                for e in ev if e['event'] == 'refused']
    cancels = [{'seq': e.get('seq'), 't': e['wall']} for e in ev if e['event'] == 'cancel_sent']
    faults = [{'reason': e['reason'], 't': e['wall']} for e in ev if e['event'] == 'fault']
    keys = sb.key_writes(drv)
    sts = sb.statuses_from_driver(drv)
    sent_times = [e['wall'] for e in ev if e['event'] == 'goal_sent']
    spaces = [k for k in keys if k['key'] == "' '"]
    s_key = next((k for k in keys if k['key'] == "'s'"), None)
    q = next((k for k in keys if k['key'] == "'q'"), None)
    checks = [{**d['cycle_check'], 'mono': d.get('mono')} for d in drv
              if isinstance(d.get('cycle_check'), dict)]
    C = {'C1': assess_plan(ev, goals, cancels, faults, refusals, keys, requests, sts, level,
                           shadow),
         'C2': assess_level(ev, goals, keys, requests, transitions, drv, sts, level,
                            run_level(drv))}
    f_stop = assess_boundary_stop(FORWARD, gl[:P], spaces[0] if spaces else None, transitions,
                                  sent_times, sts, checks[0] if checks else None, s_key)
    r_stop = assess_boundary_stop(REVERSE, gl[P:2 * P], spaces[1] if len(spaces) > 1 else None,
                                  transitions, sent_times, sts,
                                  checks[1] if len(checks) > 1 else None, q)
    C['C3'] = assess_cycle(FORWARD, f_stop, motion.get('forward'))
    C['C4'] = assess_cycle(REVERSE, r_stop, motion.get('reverse'))
    if 'net_return_recorded_only' in motion:
        C['C4']['net_return_recorded_only'] = motion['net_return_recorded_only']
    C['C5'] = assess_close(q, r_stop.get('ready_t'), goals, keys, requests, refusals, transitions,
                           drv, sts, motion.get('still'))
    ctx = {'goals': goals, 'gl': gl, 'transitions': transitions, 'requests': requests,
           'cancels': cancels, 'keys': keys, 'sts': sts, 'refusals': refusals,
           'f_ready_t': f_stop.get('ready_t'), 'r_ready_t': r_stop.get('ready_t'), 'q': q}
    return C, ctx


# ------------------------------------------------------------------ loading (read-only)
def min_joint_margin(joints, limits):
    worst = None
    for _, q in joints:
        for n, v in q.items():
            lim = limits.get(n)
            if lim is None:
                continue
            m = min(v - lim[0], lim[1] - v)
            if worst is None or m < worst[0]:
                worst = (m, n, v)
    return worst


def load_and_score(run, bag, level, disabled_library, restore_report, harness, shadow=False):
    import sqlite3
    import adjudicate_s2 as adj
    import response_law as rl
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from spiderx_controller import joint_safety
    from spiderx_controller import m55_locomotion as loc
    from spiderx_controller.m55_rclpy import body_pose_sample
    cfg = loc.load_config()
    limits, _ = joint_safety.load_limits()
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
    C0, ctx = score_events(ev, drv, level, None, shadow)
    transitions, gl = ctx['transitions'], ctx['gl']
    t_arm = next((tr['t'] for tr in transitions if tr['why'] == 'arm'), None)
    t_dis = None if ctx['q'] is None else next(
        (tr['t'] for tr in transitions if tr['t'] >= ctx['q']['t_before_write']
         and tr['state'] == 'DISARMED'), None)
    settle = [p['z'] for t, p in poses if t_arm is not None and t_arm - 10.0 <= t <= t_arm]
    z_stand = statistics.mean(settle) if settle else None
    motion = {'forward': assess_displacement(FORWARD, level, t_arm, ctx['f_ready_t'], poses,
                                             z_stand, tol + 0.05, shadow),
              'reverse': assess_displacement(REVERSE, level, ctx['f_ready_t'], ctx['r_ready_t'],
                                             poses, z_stand, tol + 0.05, shadow),
              'still': sb.assess_still(t_dis, sim_of, poses, [(t, q) for t, q, _ in jts])
              if t_dis is not None else None}
    a = nearest(poses, t_arm, tol + 0.05) if t_arm is not None else None
    b = nearest(poses, ctx['r_ready_t'], tol + 0.05) if ctx['r_ready_t'] is not None else None
    if a is not None and b is not None:
        f, lat, dyaw = body_delta(a[1], b[1])
        motion['net_return_recorded_only'] = {'forward_m': f, 'lateral_m': lat, 'yaw_rad': dyaw,
                                              'note': 'arm pose to the end of the reverse; '
                                                      'recorded only, never a band'}
    C, ctx = score_events(ev, drv, level, motion, shadow)
    fbk = collections.defaultdict(list)
    for c in cmds:
        if c.source == 'feedback':
            fbk[c.goal_id].append((c.receipt_ns / 1e9 - off, max(abs(x) for x in c.error)))
    C['C6'] = {'verdict': 'N/A', 'note': SHADOW_NA} if shadow else \
        s4a.assess_tracking(gl, dict(fbk), tol, None)
    refusals = ctx['refusals']
    C['C7'] = sc.assess_continuity(gl, refusals)
    log_lines = []
    for name in ('launch.log', 'launch.log.gz'):
        p = os.path.join(run, name)
        if os.path.exists(p):
            log_lines = (gzip.open(p, 'rt') if name.endswith('.gz') else open(p)).read() \
                .splitlines()
            break
    ctl = s4a.controller_order(log_lines, len(gl)) if log_lines else None
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
    graphs = [(st.get('monitors') or {}).get('graph') for t, _, st in ctx['sts']
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
    counts = {'goals_sent': len(gl), 'cancels_sent': len(ctx['cancels']),
              'max_foreign': max((g.get('foreign_action_clients', -1) for g in graphs
                                  if isinstance(g, dict)), default=None),
              'max_publishers': max((g.get('command_topic_publishers', -1) for g in graphs
                                     if isinstance(g, dict)), default=None)}
    present = sc.COMMAND_TOPIC in [n for n, _ in topics.values()]
    C['C8'] = assess_ownership(counts, present, len(receipts.get(sc.COMMAND_TOPIC, [])), support,
                               ctl, sum(len(v) for v in fbk.values()), shadow)
    result = next((d['summary'].get('result') for d in drv if isinstance(d.get('summary'), dict)),
                  None)
    C['C9'] = assess_margins(min_joint_margin([(t, q) for t, q, _ in jts], limits), ctx['sts'],
                             t_arm, ctx['q']['t_before_write'] if ctx['q'] else None, result)
    C['C10'] = sc.assess_restore(open(restore_report).read() if restore_report and
                                 os.path.exists(restore_report) else None)
    C['C11'] = sc.assess_library(sc.sha(os.path.join(run, 'node', 'library.json')),
                                 sc.sha(disabled_library))
    v = overall(C)
    return {'schema': SCHEMA, 'level': level, 'shadow': shadow, 'C': C,
            'overall': f'{v} (SHADOW ONLY: not S4c acceptance)' if shadow else v,
            'alignment': al, 'bag': {**meta, 'sha256_after_pose_scan': after}}


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for k in ('--run', '--bag', '--out'):
        a.add_argument(k, required=True)
    a.add_argument('--level', type=int, required=True, choices=sorted(LEVELS),
                   help='internal level index: 1 (40 mm, teleop 2/3) or 2 (60 mm, 3/3)')
    a.add_argument('--disabled-library')
    a.add_argument('--restore-report')
    a.add_argument('--harness')
    a.add_argument('--shadow', action='store_true',
                   help='a shadow rehearsal: the signed bands and tracking are not applicable')
    o = a.parse_args(argv)
    if os.path.exists(o.out) and os.listdir(o.out):
        print(f'refusing to write into non-empty {o.out}', file=sys.stderr)
        return 3
    os.makedirs(o.out, exist_ok=True)
    res = load_and_score(o.run, o.bag, o.level, o.disabled_library, o.restore_report, o.harness,
                         o.shadow)
    with open(os.path.join(o.out, 'summary.json'), 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
    print(json.dumps({k: v['verdict'] for k, v in res['C'].items()}), res['overall'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
