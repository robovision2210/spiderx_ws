#!/usr/bin/env python3
"""Score an S4b run (docs/S4_KEYBOARD_TELEOP_PLAN.md, section 3.2), offline. Specified BEFORE any
S4b run, shadow or live.

Reused unchanged:
- from score_s4a.py: assess_tracking, controller_order, overall, nearest, body_delta, the S4a
  forward band;
- from the frozen score_s3.py: continuity, the measured 10 s hold, restore, library, and the S3
  limits.

The plan (s4b_driver.PLAN): 12 goals at level 0 and 0 cancels.
  goals 1-5   release   0->1 ... 4->5; released while 4->5 is in flight; READY at 5
  goals 6-7   reversal  5->6, then 6->5 (the same phase played back); released in 6->5; READY at 5
  goals 8-11  home      'h': 5->6 ... 8->9; READY at 0 ('homed')
  goal 12     lease     0->1; teleop SIGKILL while it is in flight; DISARMED at 1; then 10 s of sim

Assessments. Each check is True, False, or None (missing evidence). A False check gives FAIL;
otherwise a None check gives UNMEASURED; otherwise the assessment is PASS. Missing evidence is never
PASS, and a contradictory sequence is FAIL.
- B1 plan: exactly the 12 planned goals in order, at level 0, every result SUCCEEDED; 0 cancels;
  no fault.
  - The keys are exactly a, w, w, s, h, w, each written completely. No space, x, Esc, r, z or q.
  - No stop, estop, reset or disarm request; the arm request only once, before goal 1.
  - The event order is consistent.
- B2 release (goal 5) and B3 reversal (goals 6-7, released in goal 7). For each release:
  - released while the goal was in flight, from the driver's stop-the-repeat bracket. That is a
    key RELEASE, not an explicit stop: no space and no stop request (B1);
  - exactly one STOPPING while the goal was in flight, reason 'intent_stop', at least
    H - 1/30 s - T after the release (the hold window decided it; M5.5 section 16.1);
  - no goal sent between the release and READY;
  - READY ('stopped') at boundary 5;
  - READY within the goal's duration_s + 1.0 s of sim, from the last status before the release to
    the first READY (check 1.7, as the S4 criteria apply it to key release);
  - wall backstop 120 s.
  The detection latency is recorded; above the documented 0.95 s (+ one 30 Hz repeat) it is a
  finding.
  B3 also needs:
  - goal 6 = 5->6 and goal 7 = 6->5, the same phase;
  - 's' written while goal 6 was in flight, after a stop of the repeat (no release) and within
    the 0.8 s hold window of it;
  - goal 7 sent at goal 6's boundary (at or after its phase_done), with no STOPPING or READY in
    between.
- B4 home:
  - 'h' written at READY at boundary 5;
  - one homing event, forward, 4 phases (the fewest from boundary 5);
  - goals 8-11 = 5->6 ... 8->9;
  - no motion key during home;
  - READY ('homed') at boundary 0;
  - within the documented 12.0 s sim homing bound (from the last status before 'h' to the first
    READY at 0); wall backstop 120 s.
- B5 lease loss, measured from the driver's kill bracket (time.monotonic() before and after the
  SIGKILL):
  - the kill delivered while goal 12 was in flight;
  - exactly one STOPPING after the kill, reason 'lease_expired', at least L - B - T after it (the
    lease decided it);
  - DISARMED ('lease_expired') at boundary 1, within goal 12's duration_s + 1.0 s of sim (check
    1.9, from the last status before the kill); wall backstop 120 s;
  - after the kill: no goal, no request, no key, no teleop restart;
  - every later status DISARMED, with no goal in flight, over at least 10 s of sim;
  - the robot still over those 10 s, measured from the bag with the frozen S3 hold limits.
  A STOPPING latency above the documented 0.55 s is a finding.
- B6 cycle: releases, reversal and home make one complete forward cycle (arm to 'homed').
  - Progress along body +y within the S4a forward band [10, 30] mm.
  - |lateral| <= 10 mm, |yaw| <= 0.05 rad, height within +-10 mm, tilt <= 0.10 rad.
  The lease phase's displacement is recorded only.
- B7 tracking: every goal has attributed feedback (no N/A: no planned cancel); max |error|
  <= 0.05 rad; > 0.02 is a finding.
- B8 continuity (S3 A4).
- B9 ownership: 12 goals, 0 cancels, no foreign client or publisher, 0 joint_trajectory messages
  (an absent topic counts only with the S4a support evidence).
  - The controller's log: 12 received, 12 accepted, 0 cancel requests.
- B10 restore (S3 A7); B11 library (S3 A8).

Usage: score_s4b.py --run <run dir> --bag <bag_0.db3> --disabled-library <library.json>
                    --restore-report <report> --harness <run script> --out <new dir>"""
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
import score_s4a as s4a  # noqa: E402  (unchanged)

sc = s4a.sc                                      # the frozen S3 scorer
nearest, body_delta, overall = s4a.nearest, s4a.body_delta, s4a.overall
SCHEMA = 'spiderx.s4b_score/1'
P = 9
PLAN = (('release', ((0, 1, 1), (1, 2, 1), (2, 3, 1), (3, 4, 1), (4, 5, 1))),
        ('reversal', ((5, 6, 1), (6, 5, -1))),
        ('home', ((5, 6, 1), (6, 7, 1), (7, 8, 1), (8, 9, 1))),
        ('lease', ((0, 1, 1),)))
PLANNED = [g for _, gs in PLAN for g in gs]
EXPECTED_GOALS, EXPECTED_CANCELS = len(PLANNED), 0
SCRIPTED_KEYS = ["'a'", "'w'", "'w'", "'s'", "'h'", "'w'"]
FORBIDDEN_KEYS = ("' '", "'x'", repr('\x1b'), "'r'", "'z'", "'q'")
FORBIDDEN_REQUESTS = ('stop', 'estop', 'reset', 'disarm')
HOLD_S, HEARTBEAT_S, LEASE_S, TICK_S = 0.8, 0.1, 0.5, 0.05     # M5.5 section 16.1
REPEAT_S = 1 / 30                                 # the emulated key repeat period
DETECT_RELEASE_DOC_S, DETECT_LEASE_DOC_S = 0.95, 0.55
SIM_MARGIN_S, WALL_BACKSTOP_S, HOME_SIM_BOUND_S = 1.0, 120.0, 12.0
STATUS_SUCCEEDED = sc.STATUS_SUCCEEDED
LEASE_EXPIRED = 'lease_expired'                  # the node's reason (m55_locomotion)


def verdict_of(checks):
    vs = list(checks.values())
    return 'FAIL' if False in vs else ('UNMEASURED' if None in vs else 'PASS')


# ------------------------------------------------------------------ extraction (pure)
def goals_from_events(ev):
    """{seq: goal} with sent, response, result and its phase_done (time and boundary)."""
    goals, current = collections.OrderedDict(), None
    for e in ev:
        s = e.get('seq')
        if e['event'] == 'goal_sent':
            goals[s] = {k: e.get(k) for k in ('seq', 'b_from', 'b_to', 'direction', 'level',
                                              'phase', 'duration_s', 'continuity_error_rad')}
            goals[s]['sent'] = e['wall']
            current = s
        elif e['event'] == 'goal_response' and s in goals:
            goals[s].update(accepted=e['accepted'], response_t=e['wall'])
        elif e['event'] == 'result' and s in goals:
            goals[s].update(status=e['status'], result_t=e['wall'])
        elif e['event'] == 'phase_done' and current in goals:
            goals[current].update(done_t=e['wall'], done_b=e['b'])
    return goals


def transitions_from_events(ev):
    return [{'t': e['wall'], 'previous': e.get('previous'), 'state': e['state'],
             'why': e.get('why')} for e in ev if e['event'] == 'transition']


def statuses_from_driver(drv):
    """[(mono, sim, status)] as the driver received them."""
    out = []
    for d in drv:
        st = d.get('status')
        if isinstance(st, dict) and isinstance(d.get('mono'), (int, float)):
            out.append((d['mono'], (st.get('monitors') or {}).get('sim_time_s'), st))
    return out


def key_writes(drv):
    return [d for d in drv if 'key' in d and 't_before_write' in d]


def sim_before(sts, t):
    """Sim time of the last status received at or before t."""
    s = [sim for mono, sim, _ in sts if mono <= t and sim is not None]
    return s[-1] if s else None


def first_status(sts, t, pred):
    """(mono, sim) of the first status after t that satisfies pred."""
    return next(((mono, sim) for mono, sim, st in sts if mono >= t and pred(st)), (None, None))


def consistent_order(ev, goals):
    """Event numbers increase; seqs are 1..n consecutive; each result after its goal; at most one
    goal unresolved at a time."""
    ns = [e.get('n') for e in ev if isinstance(e.get('n'), int)]
    if ns != sorted(ns) or len(set(ns)) != len(ns):
        return False
    seqs = list(goals)
    if seqs != list(range(1, len(seqs) + 1)):
        return False
    prev_end = None
    for g in goals.values():
        if g.get('result_t') is None or g['result_t'] < g['sent']:
            return False
        if prev_end is not None and g['sent'] < prev_end:
            return False
        prev_end = g.get('done_t', g['result_t'])
    return True


# ------------------------------------------------------------------ B1
def assess_plan(ev, goals, cancels, faults, keys, requests):
    got = [(g['b_from'], g['b_to'], g['direction']) for g in goals.values()]
    sent_seqs = [e.get('seq') for e in ev if e['event'] == 'goal_sent']
    arm = [r for r in requests if r['request'] == 'arm']
    first_sent = next(iter(goals.values()))['sent'] if goals else None
    checks = {
        'twelve_planned_goals_in_order': got == PLANNED if goals else None,
        'all_level_0': all(g['level'] == 0 for g in goals.values()) if goals else None,
        'all_succeeded': all(g.get('status') == STATUS_SUCCEEDED for g in goals.values())
        if goals else None,
        'no_cancel': len(cancels) == EXPECTED_CANCELS,
        'no_fault': not faults,
        'keys_exactly_as_scripted': [k['key'] for k in keys] == SCRIPTED_KEYS if keys else None,
        'every_key_write_complete': all(k.get('write_result') == 1 for k in keys)
        if keys else None,
        'no_space_estop_reset_disarm_or_quit_key': not any(k['key'] in FORBIDDEN_KEYS
                                                           for k in keys),
        'no_stop_estop_reset_or_disarm_request': not any(r['request'] in FORBIDDEN_REQUESTS
                                                         for r in requests),
        'arm_once_before_goal_1': len(arm) == 1 and first_sent is not None
        and arm[0]['t'] <= first_sent,
        'one_goal_sent_per_seq': len(sent_seqs) == len(set(sent_seqs)) if sent_seqs else None,
        'event_order_consistent': consistent_order(ev, goals) if goals else None,
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'goals': got,
            'cancels': len(cancels), 'faults': [f['reason'] for f in faults]}


# ------------------------------------------------------------------ B2, B3
def assess_release(goal, release, transitions, sent_times, sts, expect_b):
    """One key release while `goal` is in flight (the driver stopped the repeat; no space).
    sent_times: every goal_sent time in the events (duplicates included)."""
    if goal is None or release is None or goal.get('result_t') is None:
        return {'verdict': 'UNMEASURED', 'reason': 'the goal, its result or the release record '
                'is missing'}
    t_rel = release['t_after_stop']
    ready = next((tr for tr in transitions if tr['t'] >= t_rel and tr['state'] == 'READY'), None)
    stopping = [tr for tr in transitions if tr['state'] == 'STOPPING'
                and goal['sent'] <= tr['t'] <= goal['result_t']]
    lat = stopping[0]['t'] - t_rel if stopping else None
    s0 = sim_before(sts, release['t_before_stop'])
    _, s1 = first_status(sts, t_rel, lambda st: st.get('state') == 'READY'
                         and st.get('boundary') == expect_b and st.get('goal') is None)
    bound = (goal.get('duration_s') or 0.0) + SIM_MARGIN_S
    later = [t for t in sent_times if t > t_rel and (ready is None or t < ready['t'])]
    checks = {
        'released_while_goal_in_flight': goal['sent'] <= release['t_before_stop']
        and t_rel <= goal['result_t'],
        'one_stopping_intent_stop': len(stopping) == 1 and stopping[0]['why'] == 'intent_stop',
        'stopping_after_the_hold_window': None if lat is None
        else lat >= HOLD_S - REPEAT_S - TICK_S,
        'no_goal_between_release_and_ready': not later if ready is not None else None,
        'ready_stopped_at_boundary': None if ready is None else (
            ready['why'] == 'stopped' and goal.get('done_b') == expect_b),
        'ready_within_sim_bound': None if s0 is None or s1 is None else s1 - s0 <= bound,
        'ready_within_wall_backstop': None if ready is None
        else ready['t'] - t_rel <= WALL_BACKSTOP_S,
    }
    doc = DETECT_RELEASE_DOC_S + REPEAT_S
    return {'verdict': verdict_of(checks), 'checks': checks, 'goal': goal['seq'],
            'detection_latency_s': lat, 'ready_sim_s': None if None in (s0, s1) else s1 - s0,
            'sim_bound_s': bound, 'finding_detection_above_documented': None if lat is None
            else lat > doc}


def assess_reversal(g6, g7, s_key, switch, transitions, release_b3):
    if None in (g6, g7, s_key, switch) or g6.get('result_t') is None:
        return {'verdict': 'UNMEASURED', 'reason': 'goal 6/7, the s key or the switch record is '
                'missing', 'release': release_b3}
    between = [tr for tr in transitions if g6['sent'] < tr['t'] < g7['sent']
               and tr['state'] in ('STOPPING', 'READY')]
    checks = {
        'forward_then_same_phase_back': (g6['b_from'], g6['b_to'], g6['direction']) == (5, 6, 1)
        and (g7['b_from'], g7['b_to'], g7['direction']) == (6, 5, -1)
        and g7['phase'] == g6['phase'] and g7['seq'] == g6['seq'] + 1,
        's_written_while_forward_goal_in_flight': g6['sent'] <= s_key['t_before_write']
        and s_key['t_after_write'] <= g6['result_t'],
        'switch_without_release': switch['t_after_stop'] <= s_key['t_before_write']
        <= switch['t_after_stop'] + HOLD_S,
        'reverse_sent_at_the_boundary': g6.get('done_t') is not None
        and g7['sent'] >= g6['done_t'] and not between,
        'then_released': release_b3['verdict'] == 'PASS' if release_b3['verdict'] != 'UNMEASURED'
        else None,
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'release': release_b3}


# ------------------------------------------------------------------ B4
def assess_home(h_key, homing, home_goals, transitions, sts, keys):
    if h_key is None or len(home_goals) != 4 or home_goals[-1].get('done_t') is None:
        return {'verdict': 'UNMEASURED', 'reason': "the 'h' key or the home goals are missing"}
    at = [st for mono, _, st in sts if mono <= h_key['t_before_write']]
    st0 = at[-1] if at else None
    homed = next((tr for tr in transitions if tr['t'] >= home_goals[-1]['done_t']
                  and tr['state'] == 'READY'), None)
    end = homed['t'] if homed else None
    motion = [k for k in keys if k['key'] in ("'w'", "'s'") and end is not None
              and h_key['t_after_write'] <= k['t_before_write'] <= end]
    s0 = sim_before(sts, h_key['t_before_write'])
    _, s1 = first_status(sts, h_key['t_after_write'], lambda st: st.get('state') == 'READY'
                         and st.get('boundary') == 0 and st.get('goal') is None)
    checks = {
        'h_at_ready_boundary_5': None if st0 is None else (
            st0.get('state') == 'READY' and st0.get('boundary') == 5 and st0.get('goal') is None),
        'one_homing_forward_4_phases': [(e['direction'], e['phases']) for e in homing]
        == [(1, 4)],
        'goals_forward_to_the_cycle_boundary': [(g['b_from'], g['b_to'], g['direction'])
                                                for g in home_goals] == list(PLAN[2][1])
        and all(g['sent'] >= h_key['t_before_write'] for g in home_goals),
        'no_motion_key_during_home': not motion if end is not None else None,
        'ready_homed_at_boundary_0': None if homed is None else (
            homed['why'] == 'homed' and home_goals[-1].get('done_b') == 0),
        'within_documented_homing_bound': None if None in (s0, s1)
        else s1 - s0 <= HOME_SIM_BOUND_S,
        'within_wall_backstop': None if end is None
        else end - h_key['t_after_write'] <= WALL_BACKSTOP_S,
    }
    return {'verdict': verdict_of(checks), 'checks': checks,
            'home_sim_s': None if None in (s0, s1) else s1 - s0}


# ------------------------------------------------------------------ B5
def assess_lease(kill, g12, goals, transitions, requests, keys, drv, sts, still):
    if kill is None or g12 is None or g12.get('result_t') is None:
        return {'verdict': 'UNMEASURED', 'reason': 'the kill record or goal 12 is missing'}
    tk = kill['t_before_kill']
    stopping = [tr for tr in transitions if tr['t'] >= tk and tr['state'] == 'STOPPING']
    disarmed = next((tr for tr in transitions if tr['t'] >= tk and tr['state'] == 'DISARMED'),
                    None)
    lat = stopping[0]['t'] - tk if stopping else None
    s0 = sim_before(sts, tk)
    t1, s1 = first_status(sts, kill['t_after_kill'], lambda st: st.get('state') == 'DISARMED')
    bound = (g12.get('duration_s') or 0.0) + SIM_MARGIN_S
    after = [st for mono, sim, st in sts if t1 is not None and mono >= t1]
    sims = [sim for mono, sim, st in sts if t1 is not None and mono >= t1 and sim is not None]
    i_kill = next((i for i, d in enumerate(drv) if d.get('event') == 'teleop_killed'), None)
    checks = {
        'kill_delivered_while_goal_in_flight': kill.get('result') == 'delivered'
        and g12['sent'] <= tk <= kill['t_after_kill'] <= g12['result_t'],
        'one_stopping_lease_expired': len(stopping) == 1
        and stopping[0]['why'] == LEASE_EXPIRED,
        'stopping_after_the_lease': None if lat is None
        else lat >= LEASE_S - HEARTBEAT_S - TICK_S,
        'disarmed_lease_expired_at_boundary_1': None if disarmed is None else (
            disarmed['why'] == LEASE_EXPIRED and g12.get('done_b') == 1),
        'disarmed_within_sim_bound': None if None in (s0, s1) else s1 - s0 <= bound,
        'disarmed_within_wall_backstop': None if disarmed is None
        else disarmed['t'] - tk <= WALL_BACKSTOP_S,
        'no_goal_after_the_kill': not any(g['sent'] > tk for g in goals.values()),
        'no_request_after_the_kill': not any(r['t'] > tk for r in requests),
        'no_key_after_the_kill': not any(k['t_before_write'] > tk for k in keys),
        'no_teleop_restart': None if i_kill is None else not any(
            d.get('event') == 'teleop_started' for d in drv[i_kill:]),
        'disarmed_throughout_10_s_of_sim': None if not after or len(sims) < 2 else (
            all(st.get('state') == 'DISARMED' and st.get('goal') is None for st in after)
            and sims[-1] - sims[0] >= sc.HOLD_SIM_S - 0.5),
        'still_after_lease_loss': None if still is None or still['verdict'] == 'UNMEASURED'
        else still['verdict'] == 'PASS',
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'stopping_latency_s': lat,
            'disarmed_sim_s': None if None in (s0, s1) else s1 - s0, 'sim_bound_s': bound,
            'finding_stopping_above_documented': None if lat is None
            else lat > DETECT_LEASE_DOC_S, 'still': still}


def assess_still(start_t, sim_of, poses, joints):
    """The frozen S3 hold measurement (10 s of sim, joint motion <= 0.01 rad, z >= 0.045 m,
    tilt <= 0.10 rad), here after lease loss: nothing was cancelled."""
    out = sc.assess_hold(True, True, start_t, sim_of, poses, joints)
    out.pop('faulted_with_one_cancel', None)
    out.pop('result_canceled', None)
    return out


# ------------------------------------------------------------------ B6
def assess_cycle(t_arm, t_homed, poses, z_stand, tol):
    """The composite forward cycle (goals 1-11): S4a's forward band, S3's drift limits."""
    if None in (t_arm, t_homed, z_stand):
        return {'verdict': 'UNMEASURED', 'reason': 'need the arm time, the homed time and '
                'z_stand'}
    a, b = nearest(poses, t_arm, tol), nearest(poses, t_homed, tol)
    inside = [p['tilt'] for t, p in poses if t_arm <= t <= t_homed]
    if a is None or b is None or not inside:
        return {'verdict': 'UNMEASURED', 'reason': 'no pose at a boundary or in the cycle'}
    fwd, lat, dyaw = body_delta(a[1], b[1])
    lo, hi = s4a.BAND_PROGRESS_M[s4a.FORWARD]
    r = {'forward_m': fwd, 'lateral_m': lat, 'yaw_rad': dyaw, 'z_end_m': b[1]['z'],
         'height_vs_standing_m': b[1]['z'] - z_stand, 'max_tilt_rad': max(inside),
         'band_forward_m': [lo, hi], 'z_stand_m': z_stand}
    r['verdict'] = 'PASS' if (lo <= fwd <= hi and abs(lat) <= sc.BAND_LATERAL_M
                              and abs(dyaw) <= sc.BAND_YAW_RAD
                              and abs(r['height_vs_standing_m']) <= sc.BAND_HEIGHT_M
                              and r['max_tilt_rad'] <= sc.TILT_MAX_RAD) else 'FAIL'
    return r


# ------------------------------------------------------------------ B9
def assess_ownership(counts, topic_present, topic_count, support, ctl):
    terms = {'goals_12': counts.get('goals_sent') == EXPECTED_GOALS,
             'cancels_0': counts.get('cancels_sent') == EXPECTED_CANCELS,
             'foreign_clients_0': counts.get('max_foreign') == 0,
             'command_publishers_0': counts.get('max_publishers') == 0,
             'controller_log_12_received_12_accepted_0_cancel': None if ctl is None else (
                 ctl['received'] == EXPECTED_GOALS and ctl['accepted'] == EXPECTED_GOALS
                 and ctl['cancel_requests'] == 0)}
    if topic_present:
        jt = 'PASS' if topic_count == 0 else 'FAIL'
    else:
        jt = 'PASS' if support and all(support.values()) else 'UNMEASURED'
    v = verdict_of({**terms, 'command_topic': {'PASS': True, 'FAIL': False}.get(jt)})
    return {'verdict': v, 'terms': terms, 'command_topic': {
        'present_in_bag': topic_present, 'messages': topic_count if topic_present else None,
        'term': jt, 'support': None if topic_present else support}}


# ------------------------------------------------------------------ the run
def lease_disarm_time(ev, drv):
    """Wall time of the first DISARMED after the recorded kill, or None."""
    kill = next((d for d in drv if d.get('event') == 'teleop_killed'), None)
    return None if kill is None else next(
        (tr['t'] for tr in transitions_from_events(ev) if tr['t'] >= kill['t_before_kill']
         and tr['state'] == 'DISARMED'), None)


def score_events(ev, drv, still=None):
    """B1-B5 from the node events and the driver log (still: the bag-measured stillness after
    lease loss, or None when no bag is given: then B5 is at best UNMEASURED). Used by
    load_and_score and by the bench tests. Returns (B, context for the bag-based checks)."""
    goals = goals_from_events(ev)
    gl = list(goals.values())
    transitions = transitions_from_events(ev)
    requests = [{'request': e['request'], 't': e['wall']} for e in ev if e['event'] == 'request']
    cancels = [{'seq': e.get('seq'), 't': e['wall']} for e in ev if e['event'] == 'cancel_sent']
    faults = [{'reason': e['reason'], 't': e['wall']} for e in ev if e['event'] == 'fault']
    homing = [e for e in ev if e['event'] == 'homing']
    keys = key_writes(drv)
    sts = statuses_from_driver(drv)
    rel = [d for d in drv if 'release' in d and 't_before_stop' in d]
    sw = [d for d in drv if 'switch' in d and 't_before_stop' in d]
    kills = [d for d in drv if d.get('event') == 'teleop_killed']

    def g(seq):
        return goals.get(seq)
    s_keys = [k for k in keys if k['key'] == "'s'"]
    h_keys = [k for k in keys if k['key'] == "'h'"]
    B = {'B1': assess_plan(ev, goals, cancels, faults, keys, requests)}
    sent_times = [e['wall'] for e in ev if e['event'] == 'goal_sent']
    B['B2'] = assess_release(g(5), rel[0] if rel else None, transitions, sent_times, sts, 5)
    b3_release = assess_release(g(7), rel[1] if len(rel) > 1 else None, transitions, sent_times,
                                sts, 5)
    B['B3'] = assess_reversal(g(6), g(7), s_keys[0] if s_keys else None,
                              sw[0] if sw else None, transitions, b3_release)
    B['B4'] = assess_home(h_keys[0] if h_keys else None, homing,
                          [g(k) for k in range(8, 12) if g(k)], transitions, sts, keys)
    B['B5'] = assess_lease(kills[0] if kills else None, g(12), goals, transitions, requests,
                           keys, drv, sts, still)
    ctx = {'goals': goals, 'gl': gl, 'transitions': transitions, 'requests': requests,
           'cancels': cancels, 'keys': keys, 'sts': sts}
    return B, ctx


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
    t_dis = lease_disarm_time(ev, drv)
    still = assess_still(t_dis, sim_of, poses, [(t, q) for t, q, _ in jts]) \
        if t_dis is not None else None
    B, ctx = score_events(ev, drv, still)
    gl, transitions = ctx['gl'], ctx['transitions']
    arm = next((tr['t'] for tr in transitions if tr['why'] == 'arm'), None)
    homed = next((tr['t'] for tr in transitions if tr['why'] == 'homed'), None)
    settle = [p['z'] for t, p in poses if arm is not None and arm - 10.0 <= t <= arm]
    z_stand = statistics.mean(settle) if settle else None
    B['B6'] = assess_cycle(arm, homed, poses, z_stand, tol + 0.05)
    if homed is not None and t_dis is not None:
        a, b = nearest(poses, homed, tol + 0.05), nearest(poses, t_dis, tol + 0.05)
        if a is not None and b is not None:
            f, lat, dyaw = body_delta(a[1], b[1])
            B['B6']['lease_phase_recorded_only'] = {'forward_m': f, 'lateral_m': lat,
                                                    'yaw_rad': dyaw}
    fbk = collections.defaultdict(list)
    for c in cmds:
        if c.source == 'feedback':
            fbk[c.goal_id].append((c.receipt_ns / 1e9 - off, max(abs(x) for x in c.error)))
    B['B7'] = s4a.assess_tracking(gl, dict(fbk), tol, None)
    refusals = [{'request': e['request'], 'reason': e['reason'], 't': e['wall']}
                for e in ev if e['event'] == 'refused']
    B['B8'] = sc.assess_continuity(gl, refusals)
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
    B['B9'] = assess_ownership(counts, present, len(receipts.get(sc.COMMAND_TOPIC, [])), support,
                               ctl)
    B['B10'] = sc.assess_restore(open(restore_report).read() if restore_report and
                                 os.path.exists(restore_report) else None)
    B['B11'] = sc.assess_library(sc.sha(os.path.join(run, 'node', 'library.json')),
                                 sc.sha(disabled_library))
    return {'schema': SCHEMA, 'B': B, 'overall': overall(B), 'alignment': al,
            'bag': {**meta, 'sha256_after_pose_scan': after}}


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for k in ('--run', '--bag', '--out'):
        a.add_argument(k, required=True)
    a.add_argument('--disabled-library')
    a.add_argument('--restore-report')
    a.add_argument('--harness')
    o = a.parse_args(argv)
    if os.path.exists(o.out) and os.listdir(o.out):
        print(f'refusing to write into non-empty {o.out}', file=sys.stderr)
        return 3
    os.makedirs(o.out, exist_ok=True)
    res = load_and_score(o.run, o.bag, o.disabled_library, o.restore_report, o.harness)
    with open(os.path.join(o.out, 'summary.json'), 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
    print(json.dumps({k: v['verdict'] for k, v in res['B'].items()}), res['overall'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
