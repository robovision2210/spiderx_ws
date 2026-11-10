#!/usr/bin/env python3
"""Score the S4d-M run (docs/S4D_ACCEPTANCE_PLAN.md, section 8), offline. Specified BEFORE any S4d
run; no threshold changes after one.

Reuses the S4c, S4b, S4a and frozen S3 scorers' generic parts (score_s4c.py, score_s4b.py,
score_s4a.py, score_s3.py), all unchanged. Each check is True, False or None (missing evidence).
Any False makes the M-check FAIL; otherwise any None makes it UNMEASURED; otherwise it is PASS. The
overall is FAIL if any M-check FAILs, PASS if every one PASSes, else INCONCLUSIVE. Bands are
inclusive. Owner decisions are OD1-OD16; these checks are M1-M16.

- M1 plan: exactly the 111 planned goals (level, b_from, b_to, direction), each SUCCEEDED; no
  cancel, fault or refusal anywhere; the main keys as scripted, every key written completely;
  requests before 'q' exactly arm, stop, stop, home; one arm, before goal 1; one goal per seq; a
  consistent event order; every status DISPATCH with the gate True (or, with --shadow, SHADOW with
  the gate False).
- M2 level and speed change (OD13): strides 40 and 60 mm; two '+' keys (no other level key), the
  first while DISARMED with no goal before the 'a' key and the arm, the second while goal 2 was in
  flight; each confirmed by the teleop's own line (S4c attribution); exactly two level_change
  events, 0 -> 1 between the arm and goal 1, 1 -> 2 in WALKING between goal 9's phase_done and
  goal 10's goal_sent; goals 1-9 at level 1, 10-111 at level 2.
- M3 SC cycle: B0 -> B9 in [30, 50] mm with per-cycle drift and height; cycle count 1.
- M4 forward leg (OD1): B9 -> B54 along B9's +y in [0.285, 0.315] m; F1-F5 in [50, 70] mm; walking
  continuously from goal 1 to goal 54; the space while goal 54 was in flight; one STOPPING 'stop';
  READY 'stopped' at 0 within 54's duration + 1.0 s sim (120 s wall); no goal between; count 6;
  the last cycle checked before 's'.
- M5 return leg (OD1): B54 -> B99 along B54's +y in [-0.315, -0.285] m; R1-R5 in [-70, -50] mm;
  as M4 for goals 55-99; count 1; checked before the next 'w'.
- M6 drift and attitude (OD2): at every boundary 0 from B9 to B99, in B9's frame, |lateral| <= 30 mm
  and |heading| <= 0.10 rad; tilt <= 0.10 rad from the arm to the close. Recorded only: the
  all-sample maxima from B9 to B99 (including the designed sway) and the net return.
- M7 releases (OD7): rows 9 and 12 with the S4b release rules; the two-goal allowance is not
  exercised.
- M8 reversal (OD14), M9 home (OD15, with the X composite cycle B99 -> B110 in [50, 70] mm and
  count 2): the S4b rules.
- M10 close while moving (OD7): the repeat stopped, then 'q' inside the hold window while goal 111
  was in flight; stop x 2 and disarm x 2 before the probe, no refusal, no goal, no key; exactly
  STOPPING ('stop', 'disarm' or 'intent_stop') after the 'q' write, then DISARMED ('lease_expired'
  or 'disarm') at boundary 1; the reason equals the node's own decision recomputed from node-clock
  evidence (close_reason_evidence), else UNMEASURED; DISARMED within 111's duration + 1.0 s sim and
  120 s wall; the teleop's exit 0 within 15 s; 10 s of sim DISARMED and still (bag).
- M11 tracking (OD6): attributed feedback for every one of the 111 goals, no N/A; <= 0.05 rad; above
  0.02 rad a finding. With --shadow: N/A (no trajectory runs; M13 requires zero feedback).
- M12 continuity (S3 A4). M13 ownership: 111 goals, 0 cancels, controller log 111/111/0, no foreign
  client or command publisher, no joint_trajectory message (with --shadow: 0/0/0 and no feedback).
- M14 margins and freshness (C9) and the driver result 'script complete'.
- M15 restore (C10) and library (C11), and the run installation's own gate check (OD12).
- M16 reconnect probe (OD11): section 5.5 of the plan.
Boundary poses (section 5.2): the bag pose nearest the node's phase_done instant within 0.05 s of
sim, at rest (|dxy| <= 0.5 mm over +-0.1 s of sim); the status alignment's uncertainty (the larger
of its |5th| and |95th| percentile deviations) at most 0.05 s. Otherwise UNMEASURED.
--shadow drops the signed bands and the leg bands (recorded), makes M11 N/A and labels the overall
'SHADOW ONLY: not S4d acceptance'. A shadow run scored as live, or the reverse, FAILs M1.

Usage: score_s4d.py --run <run dir> --bag <bag_0.db3> --disabled-library <file>
                    --restore-report <report> --run-install-gates <json> --harness <run script>
                    --out <new dir> [--shadow]"""
import argparse
import bisect
import collections
import gzip
import json
import math
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 's3'), os.path.join(HERE, '..', 's2')]
import score_s4c as s4c  # noqa: E402  (unchanged; brings the S4b, S4a and S3 scorers)

sb, s4a, sc = s4c.sb, s4c.s4a, s4c.sc
body_delta, verdict_of = sb.body_delta, sb.verdict_of
SCHEMA = 'spiderx.s4d_score/1'
P = 9
FORWARD, REVERSE = 1, -1


def _forward(level, b0, b1):
    return tuple((level, b, b + 1, FORWARD) for b in range(b0, b1))


PLANNED = (_forward(1, 0, P) + _forward(2, 0, P) * 5
           + tuple((2, b, b - 1, REVERSE) for b in range(P, 0, -1)) * 5
           + _forward(2, 0, 5) + ((2, 5, 6, FORWARD), (2, 6, 5, REVERSE)) + _forward(2, 5, P)
           + ((2, 0, 1, FORWARD),))
EXPECTED_GOALS, EXPECTED_CANCELS = len(PLANNED), 0                 # 111, 0 (OD5)
MAIN_KEYS = ["'+'", "'a'", "'w'", "'+'", "' '", "'s'", "' '", "'w'", "'w'", "'s'", "'h'", "'w'",
             "'q'"]
PROBE_KEYS = ["'+'", "'+'", "'w'", "'q'"]
REQUESTS_BEFORE_Q = ['arm', 'stop', 'stop', 'home']
CLOSE_REQUESTS = collections.Counter({'stop': 2, 'disarm': 2})
STOPPING_REASONS = ('stop', 'disarm', 'intent_stop')    # whichever the node handles first
DISARM_REASONS = ('lease_expired', 'disarm')            # m55_locomotion 990
PROBE_MESSAGES = {'stop': 'not moving (DISARMED); held until a zero command',
                  'disarm': 'already disarmed'}         # m55_locomotion 851-870, in DISARMED
STRIDE_M = {1: 0.040, 2: 0.060}
CYCLES = ((('SC', 0, 9, 'SC'),) + tuple((f'F{j}', 9 * j, 9 * j + 9, 'F') for j in range(1, 6))
          + tuple((f'R{j}', 45 + 9 * j, 54 + 9 * j, 'R') for j in range(1, 6))
          + (('X', 99, 110, 'X'),))
BAND_M = {'SC': (0.030, 0.050), 'F': (0.050, 0.070), 'R': (-0.070, -0.050), 'X': (0.050, 0.070)}
OUTER_M = {'SC': 0.060, 'F': 0.080, 'R': 0.080, 'X': 0.080}       # the live stop bounds
LEGS = {'F': (9, 54, (0.285, 0.315)), 'R': (54, 99, (-0.315, -0.285))}      # OD1
CUMULATIVE_GOALS = tuple(range(9, 100, 9))                          # B9 ... B99
CUM_LATERAL_M, CUM_HEADING_RAD = 0.030, 0.10                        # OD2
BOUNDARY_GOALS = (0,) + CUMULATIVE_GOALS + (110,)
BOUNDARY_SIM_TOL_S, REST_WINDOW_SIM_S, REST_MAX_M = 0.05, 0.1, 0.0005      # section 5.2
ALIGN_UNCERTAINTY_MAX_S = 0.05
SIM_MARGIN_S, WALL_BACKSTOP_S = sb.SIM_MARGIN_S, sb.WALL_BACKSTOP_S        # 1.0 s, 120 s
HOLD_S, REPEAT_S = sb.HOLD_S, sb.REPEAT_S                                  # 0.8 s, 1/30 s
TELEOP_EXIT_WALL_S = 15.0
LEASE_S = 0.5                       # command.lease_s; read from the run's library.json if there
REASON_EPS_S = 1e-6
PROBE_START_WALL_S, PROBE_CONFIRM_WALL_S, PROBE_RELEASE_WALL_S = 30.0, 5.0, 5.0
PROBE_HOLD_WALL_S = (5.0, 6.0)
TELEOP_STATUS_LINE = 'state DISARMED'
GATES = ('spiderx_controller.m61_live_contract.M61_LIVE_DISPATCH_ENABLED',
         'spiderx_controller.m6_live_contract.LIVE_DISPATCH_ENABLED',
         'spiderx_controller.m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED')
SCRIPT_COMPLETE = s4c.SCRIPT_COMPLETE
SHADOW_NA = 'not applicable: dispatch disabled (shadow rehearsal)'
OD1_CLAIM = 'approximately 0.3 m (0.300 m +- 0.015 m, OD1); never "at least 0.3 m"'


def number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def as_check(part):
    """An assessment's verdict as one check: PASS True, FAIL False, UNMEASURED None."""
    if part is None or part.get('verdict') in (None, 'UNMEASURED'):
        return None
    return part['verdict'] == 'PASS'


def all_of(values):
    """False if any value is False, else None if any is None, else True (empty: None)."""
    values = list(values)
    if False in values:
        return False
    return None if not values or None in values else True


# ------------------------------------------------------------------ extraction (pure)
def context(ev, drv):
    """Everything the event-based checks read, from the node's events and the driver's log."""
    goals = sb.goals_from_events(ev)
    keys = sb.key_writes(drv)
    starts = [d['mono'] for d in drv if d.get('event') == 'teleop_started'
              and number(d.get('mono'))]
    t_probe = starts[1] if len(starts) > 1 else None
    main = [k for k in keys if t_probe is None or k['t_before_write'] < t_probe]
    probe = [k for k in keys if t_probe is not None and k['t_before_write'] >= t_probe]
    transitions = sb.transitions_from_events(ev)
    q = next((k for k in main if k['key'] == "'q'"), None)
    t_dis = None if q is None else next(
        (tr['t'] for tr in transitions if tr['t'] >= q['t_before_write']
         and tr['state'] == 'DISARMED'), None)
    summary = next((d['summary'] for d in drv if isinstance(d.get('summary'), dict)), None)
    return {
        'ev': ev, 'drv': drv, 'goals': goals, 'gl': list(goals.values()),
        'transitions': transitions,
        'requests': [{'request': e['request'], 'message': e.get('message'), 't': e['wall']}
                     for e in ev if e['event'] == 'request'],
        'refusals': [{'request': e['request'], 'reason': e['reason'], 't': e['wall']}
                     for e in ev if e['event'] == 'refused'],
        'cancels': [{'seq': e.get('seq'), 't': e['wall']} for e in ev
                    if e['event'] == 'cancel_sent'],
        'faults': [{'reason': e['reason'], 't': e['wall']} for e in ev if e['event'] == 'fault'],
        'rejections': [e for e in ev if e['event'] == 'command_rejected'],
        'level_changes': [e for e in ev if e['event'] == 'level_change'],
        'homing': [e for e in ev if e['event'] == 'homing'],
        'sent_times': [e['wall'] for e in ev if e['event'] == 'goal_sent'],
        'keys': keys, 'main_keys': main, 'probe_keys': probe, 't_probe': t_probe,
        'q': q, 'probe_q': next((k for k in probe if k['key'] == "'q'"), None),
        't_arm': next((tr['t'] for tr in transitions if tr['why'] == 'arm'), None),
        't_dis': t_dis, 'sts': sb.statuses_from_driver(drv),
        'releases': {d.get('step'): d for d in drv if 'release' in d and 't_before_stop' in d},
        'switch': next((d for d in drv if 'switch' in d and 't_after_stop' in d), None),
        'repeat_stop': next((d['repeat_stop_before_q'] for d in drv
                             if isinstance(d.get('repeat_stop_before_q'), dict)), None),
        'closed': {d.get('step'): d for d in drv if d.get('event') == 'teleop_closed'},
        'checks': {d['cycle_check'].get('name'): {**d['cycle_check'], 'mono': d.get('mono')}
                   for d in drv if isinstance(d.get('cycle_check'), dict)},
        'observation_t': next((d['mono'] for d in drv if 'close_observation' in d), None),
        'result': None if summary is None else summary.get('result'),
    }


def statuses_in(c, t0, t1=math.inf):
    return [(mono, sim, st) for mono, sim, st in c['sts'] if t0 <= mono < t1]


# ------------------------------------------------------------------ M1
def assess_plan(c, shadow):
    gl = c['gl']
    got = [(g['level'], g['b_from'], g['b_to'], g['direction']) for g in gl]
    sent_seqs = [e.get('seq') for e in c['ev'] if e['event'] == 'goal_sent']
    arm = [r for r in c['requests'] if r['request'] == 'arm']
    q = c['q']
    before_q = None if q is None else [r['request'] for r in c['requests']
                                       if r['t'] < q['t_before_write']]
    modes = {(st.get('mode'), st.get('dispatch_gate')) for _, _, st in c['sts']}
    mode_name = 'node_shadow_gate_false_throughout' if shadow \
        else 'node_dispatch_gate_true_throughout'
    checks = {
        'exactly_111_planned_goals_in_order': got == list(PLANNED) if gl else None,
        'all_succeeded': all(g.get('status') == sb.STATUS_SUCCEEDED for g in gl)
        if gl else None,
        'no_cancel': not c['cancels'],
        'no_fault': not c['faults'],
        'no_refusal': not c['refusals'],
        'main_keys_exactly_as_scripted': [k['key'] for k in c['main_keys']] == MAIN_KEYS
        if c['main_keys'] else None,
        'every_key_write_complete': all(k.get('write_result') == 1 for k in c['keys'])
        if c['keys'] else None,
        'requests_before_q_exactly_arm_stop_stop_home': None if before_q is None
        else before_q == REQUESTS_BEFORE_Q,
        'one_arm_before_goal_1': len(arm) == 1 and bool(gl) and arm[0]['t'] <= gl[0]['sent'],
        'one_goal_sent_per_seq': len(sent_seqs) == len(set(sent_seqs)) if sent_seqs else None,
        'event_order_consistent': sb.consistent_order(c['ev'], c['goals']) if gl else None,
        mode_name: None if not c['sts'] else modes == {('shadow', False) if shadow
                                                       else ('dispatch', True)},
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'goals': len(gl),
            'cancels': len(c['cancels']), 'faults': [f['reason'] for f in c['faults']]}


# ------------------------------------------------------------------ M2
def level_presses(c, keys, conf):
    """The '+' presses among `keys`, their next-key bounds and their attribution (S4c rules)."""
    idx = [i for i, k in enumerate(keys) if k['key'] == "'+'"]
    plus = [keys[i] for i in idx]
    uppers = [keys[i + 1]['t_before_write'] if i + 1 < len(keys) else None for i in idx]
    st0 = next((st for _, _, st in c['sts'] if st.get('levels_m_s')), None)
    speeds = (st0 or {}).get('levels_m_s') or []
    ok, rows = s4c.attribute_confirmations(plus, conf, uppers, speeds)
    return plus, ok, rows


def split_confirmations(c):
    conf = s4c.confirmations(c['drv'])
    if conf is None:
        return None, None
    tp = c['t_probe']
    return ([x for x in conf if tp is None or x['t_receipt'] < tp],
            [x for x in conf if tp is not None and x['t_receipt'] >= tp])


def assess_level(c):
    main, goals = c['main_keys'], c['goals']
    conf_main, _ = split_confirmations(c)
    plus, confirmed, rows = level_presses(c, main, conf_main)
    other = [k for k in main if k['key'] in s4c.LEVEL_KEYS and k['key'] != "'+'"]
    a_key = next((k for k in main if k['key'] == "'a'"), None)
    arm_req = next((r['t'] for r in c['requests'] if r['request'] == 'arm'), None)
    st0 = next((st for _, _, st in c['sts'] if st.get('strides_m')), None)
    strides = (st0 or {}).get('strides_m') or []
    lc = c['level_changes']
    g1, g2, g9, g10 = (goals.get(k) for k in (1, 2, 9, 10))
    first = s4c.status_before(c['sts'], plus[0]['t_before_write']) if plus else None
    second = plus[1] if len(plus) > 1 else None
    checks = {
        'node_strides_40_and_60_mm': None if len(strides) < 3 else all(
            abs(strides[k] - v) <= 1e-6 for k, v in STRIDE_M.items()),
        'exactly_two_plus_keys_and_no_other_level_key': len(plus) == 2 and not other,
        'first_plus_while_disarmed_before_the_arm': None if not plus or first is None
        or a_key is None or arm_req is None else (
            first.get('state') == 'DISARMED' and first.get('goal') is None
            and plus[0]['t_after_write'] <= a_key['t_before_write']
            and plus[0]['t_after_write'] <= arm_req),
        'second_plus_while_goal_2_in_flight': None if second is None or g2 is None
        or g2.get('result_t') is None else (
            g2['sent'] <= second['t_before_write'] and second['t_after_write'] <= g2['result_t']),
        'each_plus_confirmed_by_the_teleop': confirmed,
        'exactly_two_level_changes': len(lc) == 2,
        'level_change_0_to_1_between_the_arm_and_goal_1': None if not lc or g1 is None
        or c['t_arm'] is None else (lc[0].get('previous'), lc[0].get('level')) == (0, 1)
        and c['t_arm'] <= lc[0]['wall'] <= g1['sent'],
        'level_change_1_to_2_walking_between_goal_9_and_goal_10': None if len(lc) < 2
        or g9 is None or g10 is None or g9.get('done_t') is None else (
            (lc[1].get('previous'), lc[1].get('level'), lc[1].get('state')) == (1, 2, 'WALKING')
            and g9['done_t'] <= lc[1]['wall'] <= g10['sent']),
        'goals_1_to_9_at_level_1_and_10_to_111_at_level_2': None if not c['gl'] else all(
            g.get('level') == (1 if g['seq'] <= 9 else 2) for g in c['gl']),
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'confirmations': rows}


# ------------------------------------------------------------------ bag measurements (pure)
def boundary_pose(motion, k):
    b = (motion.get('boundaries') or {}).get(k)
    return None if not b else b.get('pose')


def measure_cycle(name, motion, shadow):
    """One cycle, start to end boundary pose (bag), in the start pose's frame."""
    _, k0, k1, kind = next(x for x in CYCLES if x[0] == name)
    a, b, z = boundary_pose(motion, k0), boundary_pose(motion, k1), motion.get('z_stand')
    if a is None or b is None or z is None:
        return {'verdict': 'UNMEASURED', 'from_goal': k0, 'to_goal': k1,
                'reason': f'boundary pose B{k0} or B{k1}, or z_stand, missing'}
    fwd, lat, dyaw = body_delta(a, b)
    lo, hi = BAND_M[kind]
    r = {'from_goal': k0, 'to_goal': k1, 'forward_m': fwd, 'lateral_m': lat, 'yaw_rad': dyaw,
         'height_vs_standing_m': b['z'] - z, 'band_forward_m': [lo, hi],
         'outer_bound_m': OUTER_M[kind]}
    checks = {
        'progress_in_band': lo <= fwd <= hi,
        'progress_within_the_outer_bound': abs(fwd) <= OUTER_M[kind],
        'lateral_within_10_mm': abs(lat) <= sc.BAND_LATERAL_M,
        'yaw_within_0_05_rad': abs(dyaw) <= sc.BAND_YAW_RAD,
        'height_within_10_mm': abs(r['height_vs_standing_m']) <= sc.BAND_HEIGHT_M,
    }
    if shadow:
        checks.pop('progress_in_band')
        r['progress_band'] = SHADOW_NA
    r.update(checks=checks, verdict=verdict_of(checks))
    return r


def measure_leg(name, motion, shadow):
    """F or R leg (OD1), along its start boundary's +y."""
    k0, k1, (lo, hi) = LEGS[name]
    a, b = boundary_pose(motion, k0), boundary_pose(motion, k1)
    if a is None or b is None:
        return {'verdict': 'UNMEASURED', 'reason': f'boundary pose B{k0} or B{k1} missing'}
    fwd, lat, dyaw = body_delta(a, b)
    r = {'from_goal': k0, 'to_goal': k1, 'forward_m': fwd, 'lateral_m': lat, 'yaw_rad': dyaw,
         'band_forward_m': [lo, hi], 'claim': OD1_CLAIM}
    if shadow:
        r.update(verdict='N/A', note=SHADOW_NA)
        return r
    checks = {'leg_in_band': lo <= fwd <= hi}
    r.update(checks=checks, verdict=verdict_of(checks))
    return r


def measure_drift(motion):
    """OD2: lateral and heading of every boundary 0 from B9 to B99, in B9's frame."""
    b9, rows = boundary_pose(motion, 9), []
    for k in CUMULATIVE_GOALS:
        bk = boundary_pose(motion, k)
        if b9 is None or bk is None:
            rows.append({'goal': k, 'lateral_m': None, 'heading_rad': None})
            continue
        _, lat, dyaw = body_delta(b9, bk)
        rows.append({'goal': k, 'lateral_m': lat, 'heading_rad': dyaw})
    return rows


# ------------------------------------------------------------------ M3, M4, M5
def tilt_check(motion, part):
    t = (motion.get('tilt_max') or {}).get(part)
    return None if not number(t) else t <= sc.TILT_MAX_RAD


def status_after(c, t):
    return None if t is None else next((st for mono, _, st in c['sts'] if mono >= t), None)


def assess_sc(c, motion, shadow):
    g9 = c['goals'].get(9)
    after9 = status_after(c, None if g9 is None else g9.get('done_t'))
    cycle = measure_cycle('SC', motion, shadow)
    checks = {'cycle_count_1_after_goal_9': None if after9 is None else after9.get('cycles') == 1,
              'tilt_within_0_10_rad': tilt_check(motion, 'SC'),
              'displacement': as_check(cycle)}
    return {'verdict': verdict_of(checks), 'checks': checks, 'cycle': cycle}


def walking_continuously(c, first, last):
    """No STOPPING or READY between goal `first` and goal `last`: only the dispatch's STARTING and
    the acceptance's WALKING."""
    g0, g1 = c['goals'].get(first), c['goals'].get(last)
    if g0 is None or g1 is None:
        return None
    states = [tr['state'] for tr in c['transitions'] if g0['sent'] <= tr['t'] <= g1['sent']]
    return states == ['STARTING', 'WALKING']


def assess_stop_at_0(c, last_seq, space, check, next_key, count):
    """The space while the leg's last goal is in flight; READY ('stopped') at boundary 0."""
    g = c['goals'].get(last_seq)
    if g is None or space is None or g.get('result_t') is None:
        return {'goal_and_space_recorded': None}
    tb, ta = space['t_before_write'], space['t_after_write']
    tr = c['transitions']
    stopping = [x for x in tr if x['state'] == 'STOPPING' and g['sent'] <= x['t'] <= g['result_t']]
    ready = next((x for x in tr if x['t'] >= ta and x['state'] == 'READY'), None)
    s0 = sb.sim_before(c['sts'], tb)
    t1, s1 = sb.first_status(c['sts'], ta, lambda st: st.get('state') == 'READY'
                             and st.get('boundary') == 0 and st.get('goal') is None)
    later = [t for t in c['sent_times'] if t > tb and (ready is None or t < ready['t'])]
    st_ready = status_after(c, t1)
    return {
        'space_written_while_the_last_goal_was_in_flight': g['sent'] <= tb <= ta <= g['result_t']
        and space.get('write_result') == 1,
        'one_stopping_stop_in_the_last_goal': len(stopping) == 1
        and stopping[0]['why'] == 'stop',
        'ready_stopped_at_boundary_0': None if ready is None else (
            ready['why'] == 'stopped' and g.get('done_b') == 0),
        'ready_within_sim_bound': None if None in (s0, s1)
        else s1 - s0 <= (g.get('duration_s') or 0.0) + SIM_MARGIN_S,
        'ready_within_wall_backstop': None if ready is None
        else ready['t'] - tb <= WALL_BACKSTOP_S,
        'no_goal_between_the_space_and_ready': not later if ready is not None else None,
        f'cycle_count_{count}_at_ready': None if st_ready is None
        else st_ready.get('cycles') == count,
        'last_cycle_checked_before_the_next_key': None if check is None or next_key is None
        or not number(check.get('mono')) else check['mono'] <= next_key['t_before_write'],
    }


def assess_leg(name, c, motion, shadow):
    first, last, i_space, check, count = ((1, 54, 4, 'F5', 6) if name == 'F'
                                          else (55, 99, 6, 'R5', 1))
    main = c['main_keys']
    space = main[i_space] if len(main) > i_space and main[i_space]['key'] == "' '" else None
    nxt = main[i_space + 1] if len(main) > i_space + 1 else None
    checks = assess_stop_at_0(c, last, space, c['checks'].get(check), nxt, count)
    checks['walking_continuously'] = walking_continuously(c, first, last)
    checks['tilt_within_0_10_rad'] = tilt_check(motion, name)
    leg = measure_leg(name, motion, shadow)
    if leg['verdict'] != 'N/A':
        checks['leg_distance'] = as_check(leg)
    cycles = {x[0]: measure_cycle(x[0], motion, shadow) for x in CYCLES if x[3] == name}
    checks['every_cycle'] = all_of(as_check(v) for v in cycles.values())
    return {'verdict': verdict_of(checks), 'checks': checks, 'leg': leg, 'cycles': cycles}


# ------------------------------------------------------------------ M6
def assess_drift(motion):
    rows = measure_drift(motion)
    lat = [None if r['lateral_m'] is None else abs(r['lateral_m']) <= CUM_LATERAL_M
           for r in rows]
    hdg = [None if r['heading_rad'] is None else abs(r['heading_rad']) <= CUM_HEADING_RAD
           for r in rows]
    checks = {'cumulative_lateral_within_30_mm_at_every_boundary_0': all_of(lat),
              'cumulative_heading_within_0_10_rad_at_every_boundary_0': all_of(hdg),
              'tilt_within_0_10_rad_from_the_arm_to_the_close': tilt_check(motion, 'run')}
    b0, b9, b99 = (boundary_pose(motion, k) for k in (0, 9, 99))
    net = {}
    for name, a in (('from_b9', b9), ('from_arm', b0)):
        if a is not None and b99 is not None:
            f, la, y = body_delta(a, b99)
            net[name] = {'forward_m': f, 'lateral_m': la, 'yaw_rad': y}
    return {'verdict': verdict_of(checks), 'checks': checks, 'boundaries': rows,
            'all_sample_maxima_recorded_only': motion.get('all_sample_maxima'),
            'net_return_recorded_only': net or None}


# ------------------------------------------------------------------ M7, M8, M9
def assess_releases(c):
    r9 = sb.assess_release(c['goals'].get(104), c['releases'].get('X_release'),
                           c['transitions'], c['sent_times'], c['sts'], 5)
    r12 = sb.assess_release(c['goals'].get(106), c['releases'].get('X_reversal_release'),
                            c['transitions'], c['sent_times'], c['sts'], 5)
    checks = {'row_9_release_during_goal_104': as_check(r9),
              'row_12_release_during_goal_106': as_check(r12)}
    return {'verdict': verdict_of(checks), 'checks': checks, 'row_9': r9, 'row_12': r12,
            'two_goal_allowance': 'not exercised (OD7): READY within the in-flight goal + 1.0 s'}


def assess_reversal(c, m7):
    main = c['main_keys']
    s_key = main[9] if len(main) > 9 and main[9]['key'] == "'s'" else None
    r = sb.assess_reversal(c['goals'].get(105), c['goals'].get(106), s_key, c['switch'],
                           c['transitions'], m7['row_12'])
    return r


def assess_home(c, motion, shadow):
    main, goals = c['main_keys'], c['goals']
    h_key = next((k for k in main if k['key'] == "'h'"), None)
    home_goals = [goals[k] for k in range(107, 111) if k in goals]
    r = sb.assess_home(h_key, c['homing'], home_goals, c['transitions'], c['sts'], main)
    g110 = goals.get(110)
    after = status_after(c, None if g110 is None else g110.get('done_t'))
    cycle = measure_cycle('X', motion, shadow)
    checks = {'home_rules': as_check(r),
              'cycle_count_2_at_homed': None if after is None else after.get('cycles') == 2,
              'tilt_within_0_10_rad': tilt_check(motion, 'X'),
              'x_composite_cycle': as_check(cycle)}
    return {'verdict': verdict_of(checks), 'checks': checks, 'home': r, 'cycle': cycle}


# ------------------------------------------------------------------ M10
def close_reason_evidence(c, lease_s=LEASE_S):
    """The node's own disarm decision (m55_locomotion 990), recomputed from node-clock evidence.

    A status with goal 111 in flight gives t_s = goal_sent(111) + goal.elapsed_s and the last valid
    command as of then, lv_s = t_s - lease_age_s. The DISARMED event's node time is t_tr.
    The count at the transition is that of the first DISARMED status received after it, valid only
    while its lease_age_s is empty: the transition clears the lease (m55_locomotion 905), so an
    empty lease age proves no command was accepted since.
    - t_tr - lv_s <= lease_s: 'disarm' (later commands only renew the lease);
    - else, if the count at the transition equals the status's: 'lease_expired' (nothing renewed
      it);
    - else, if the count rose and t_tr - t_s <= lease_s: 'disarm' (a command after t_s);
    - else, or within 1 us of lease_s, or without the evidence: None (UNMEASURED).
    A falling commands_accepted is a contradiction. Key writes, the teleop's exit and sim time are
    never used."""
    g = c['goals'].get(EXPECTED_GOALS)
    dis = next((tr for tr in c['transitions'] if c['q'] is not None
                and tr['t'] >= c['q']['t_before_write'] and tr['state'] == 'DISARMED'), None)
    out = {'lease_s': lease_s, 'expected': None, 'contradiction': False}
    if g is None or dis is None:
        out['reason'] = 'goal 111 or the DISARMED transition is missing'
        return out
    t_tr = dis['t']
    out['t_disarmed_node'] = t_tr
    s = None
    for mono, _, st in c['sts']:
        gg = st.get('goal') or {}
        if mono < t_tr and gg.get('seq') == EXPECTED_GOALS and number(gg.get('elapsed_s')) \
                and number(st.get('lease_age_s')) and isinstance(st.get('commands_accepted'), int):
            s = st
    first_dis = next((st for mono, _, st in c['sts'] if mono >= t_tr
                      and st.get('state') == 'DISARMED'), None)
    if s is None:
        out['reason'] = 'no status with goal 111 in flight and a lease age before the transition'
        return out
    t_s = g['sent'] + s['goal']['elapsed_s']
    lv_s = t_s - s['lease_age_s']
    n_s = s['commands_accepted']
    n_tr = None if first_dis is None or first_dis.get('lease_age_s') is not None \
        else first_dis.get('commands_accepted')
    out.update(t_status_node=t_s, last_command_node=lv_s, commands_at_status=n_s,
               commands_at_transition=n_tr, since_last_command_s=t_tr - lv_s,
               since_status_s=t_tr - t_s)
    if isinstance(n_tr, int) and n_tr < n_s:
        out.update(contradiction=True, reason='commands_accepted fell')
        return out
    if t_tr - lv_s <= lease_s - REASON_EPS_S:
        out.update(expected='disarm', basis='the lease was fresh at the last status')
    elif t_tr - lv_s > lease_s + REASON_EPS_S and n_tr == n_s:
        out.update(expected='lease_expired', basis='no command after the last status')
    elif isinstance(n_tr, int) and n_tr > n_s and t_tr - t_s <= lease_s - REASON_EPS_S:
        out.update(expected='disarm', basis='a command after the last status, within the lease')
    else:
        out['reason'] = 'the evidence does not decide the lease at the transition'
    return out


def assess_close(c, still, lease_s=LEASE_S):
    q, g = c['q'], c['goals'].get(EXPECTED_GOALS)
    if q is None or g is None:
        return {'verdict': 'UNMEASURED', 'reason': "no 'q' key record or no goal 111"}
    tq, taq = q['t_before_write'], q['t_after_write']
    end = c['t_probe'] if c['t_probe'] is not None else math.inf
    req = [r for r in c['requests'] if tq <= r['t'] < end]
    after = collections.Counter(r['request'] for r in req)
    tr = [x for x in c['transitions'] if tq <= x['t'] < end]
    stopping = tr[0] if tr else None
    dis = tr[1] if len(tr) > 1 else None
    reason = close_reason_evidence(c, lease_s)
    rs = c['repeat_stop']
    s0 = sb.sim_before(c['sts'], tq)
    t1, s1 = sb.first_status(c['sts'], taq, lambda st: st.get('state') == 'DISARMED')
    closed = c['closed'].get('Q')
    held = [st for mono, _, st in statuses_in(c, t1 if t1 is not None else math.inf, end)]
    sims = [sim for mono, sim, _ in statuses_in(c, t1 if t1 is not None else math.inf, end)
            if sim is not None]
    checks = {
        'repeat_stopped_then_q_within_the_hold_window': None if rs is None
        else 0.0 <= tq - rs['t_after_stop'] <= HOLD_S - REPEAT_S,
        'q_written_while_goal_111_in_flight': None if g.get('result_t') is None
        else g['sent'] <= tq and taq <= g['result_t'] and q.get('write_result') == 1,
        'exactly_the_close_requests_after_q': after == CLOSE_REQUESTS,
        'no_refusal_after_q': not any(r['t'] >= tq for r in c['refusals']),
        'stopping_then_disarmed_at_boundary_1': [(x['previous'], x['state']) for x in tr]
        == [('WALKING', 'STOPPING'), ('STOPPING', 'DISARMED')]
        and stopping['why'] in STOPPING_REASONS and dis['why'] in DISARM_REASONS
        and g.get('done_b') == 1,
        'stopping_after_the_q_write_inside_goal_111': None if stopping is None
        or g.get('result_t') is None else tq <= stopping['t'] <= g['result_t'],
        'disarmed_reason_matches_the_node_clock_evidence': False if reason['contradiction']
        else None if reason['expected'] is None or dis is None
        else dis['why'] == reason['expected'],
        'disarmed_within_sim_bound': None if None in (s0, s1)
        else s1 - s0 <= (g.get('duration_s') or 0.0) + SIM_MARGIN_S,
        'disarmed_within_wall_backstop': None if dis is None
        else dis['t'] - tq <= WALL_BACKSTOP_S,
        'teleop_exit_code_0_within_15_s': None if closed is None or t1 is None else (
            closed.get('exit_code') == 0
            and closed.get('t_exit_seen', math.inf) - t1 <= TELEOP_EXIT_WALL_S),
        'no_goal_after_q': not any(x['sent'] > tq for x in c['gl']),
        'no_key_after_q_before_the_probe': not any(tq < k['t_before_write'] < end
                                                   for k in c['keys']),
        'disarmed_without_goal_over_10_s_of_sim': None if not held or len(sims) < 2 else (
            all(st.get('state') == 'DISARMED' and st.get('goal') is None for st in held)
            and sims[-1] - sims[0] >= sc.HOLD_SIM_S - 0.5),
        'still_after_the_close': as_check(still),
    }
    return {'verdict': verdict_of(checks), 'checks': checks,
            'requests_after_q': dict(after), 'request_order_recorded': [r['request'] for r in req],
            'transitions_after_q': [(x['previous'], x['state'], x['why']) for x in tr],
            'reason_evidence': reason,
            'timeline_recorded_only': {'q_write_driver': tq, 'teleop_exit_seen_driver':
                                       None if closed is None else closed.get('t_exit_seen'),
                                       'disarmed_sim_s': s1, 'sim_before_q_s': s0},
            'disarmed_sim_s': None if None in (s0, s1) else s1 - s0, 'still': still}


# ------------------------------------------------------------------ M16
def assess_probe(c):
    tp, keys = c['t_probe'], c['probe_keys']
    if tp is None:
        return {'verdict': 'UNMEASURED', 'reason': 'no second teleop (the probe) in the log'}
    pq = c['probe_q']
    w = next((k for k in keys if k['key'] == "'w'"), None)
    rel = c['releases'].get('probe_release')
    before = s4c.status_before(c['sts'], tp)
    n0 = None if before is None else before.get('commands_accepted')
    sts = statuses_in(c, tp)
    started = next((mono for mono, _, st in sts if isinstance(n0, int)
                    and isinstance(st.get('commands_accepted'), int)
                    and st['commands_accepted'] > n0), None)
    chunks = [(d['t_receipt'], d['teleop_out']) for d in c['drv'] if isinstance(
        d.get('teleop_out'), str) and number(d.get('t_receipt')) and d['t_receipt'] >= tp]
    text, line_t = '', None
    for t, x in chunks:
        text += x
        if TELEOP_STATUS_LINE in text:
            line_t = t
            break
    _, conf_probe = split_confirmations(c)
    plus, confirmed, rows = level_presses(c, keys, conf_probe)
    hold = [st for mono, _, st in sts if w is not None and rel is not None
            and w['t_after_write'] <= mono <= rel['t_before_stop']]
    stop_t = None if rel is None else next(
        (mono for mono, _, st in sts if mono >= rel['t_after_stop']
         and (st.get('intent') or {}).get('direction') == 0), None)
    ev_after = [e for e in c['ev'] if e['wall'] >= tp]
    req = [r for r in c['requests'] if r['t'] >= tp]
    closed = c['closed'].get('probe')
    last_n = next((st.get('commands_accepted') for _, _, st in reversed(sts)), None)
    checks = {
        'probe_after_the_close_observation': None if c['observation_t'] is None
        else tp >= c['observation_t'],
        'probe_keys_exactly_plus_plus_w_q': [k['key'] for k in keys] == PROBE_KEYS,
        'startup_within_30_s': None if started is None or line_t is None
        else max(started, line_t) - tp <= PROBE_START_WALL_S,
        'level_confirmations_2_3_and_3_3_attributed': confirmed,
        'confirmations_within_5_s': None if not confirmed else all(
            r['t_receipt'] - r['t_before_write'] <= PROBE_CONFIRM_WALL_S
            for r in rows if 'press' in r),
        'held_w_5_to_6_s': None if w is None or rel is None else (
            PROBE_HOLD_WALL_S[0] <= rel['t_before_stop'] - w['t_after_write']
            <= PROBE_HOLD_WALL_S[1]),
        'commands_delivered_forward_at_level_2': None if not hold else any(
            (st.get('intent') or {}).get('direction') == FORWARD
            and (st.get('intent') or {}).get('level') == 2
            and (st.get('intent') or {}).get('reason') is None for st in hold),
        'commands_accepted_rising': None if not isinstance(n0, int)
        or not isinstance(last_n, int) else last_n > n0,
        'intent_stopped_within_5_s_of_the_release': None if rel is None or not sts else (
            stop_t is not None and stop_t - rel['t_after_stop'] <= PROBE_RELEASE_WALL_S),
        'disarmed_without_goal_throughout': None if not sts else all(
            st.get('state') == 'DISARMED' and st.get('goal') is None for _, _, st in sts),
        'no_arm_goal_transition_or_refusal': not any(
            e['event'] in ('goal_sent', 'transition', 'refused', 'level_change', 'homing')
            or (e['event'] == 'request' and e.get('request') == 'arm') for e in ev_after),
        'no_command_rejection': not any(e['event'] == 'command_rejected' for e in ev_after)
        and not any(st.get('command_rejections') for _, _, st in sts),
        'no_request_before_the_probe_q': None if pq is None
        else not any(r['t'] < pq['t_before_write'] for r in req),
        'close_requests_accepted_as_no_ops': None if pq is None else (
            collections.Counter(r['request'] for r in req) == CLOSE_REQUESTS
            and all(r['message'] == PROBE_MESSAGES.get(r['request']) for r in req)),
        'teleop_exit_code_0_within_15_s': None if closed is None or pq is None else (
            closed.get('exit_code') == 0
            and closed.get('t_exit_seen', math.inf) - pq['t_after_write'] <= TELEOP_EXIT_WALL_S),
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'confirmations': rows,
            'commands_accepted': {'before': n0, 'last': last_n},
            'requests_after_the_probe_q': [(r['request'], r['message']) for r in req]}


# ------------------------------------------------------------------ the events (pure part)
def score_events(ev, drv, motion=None, shadow=False, lease_s=LEASE_S):
    """M1-M10 and M16 from the node events, the driver log and (optionally) the bag-based motion
    (motion_from_samples). Without motion, every bag-based check is UNMEASURED. Returns (M, c)."""
    motion = motion or {}
    c = context(ev, drv)
    M = {'M1': assess_plan(c, shadow), 'M2': assess_level(c),
         'M3': assess_sc(c, motion, shadow), 'M4': assess_leg('F', c, motion, shadow),
         'M5': assess_leg('R', c, motion, shadow), 'M6': assess_drift(motion)}
    M['M7'] = assess_releases(c)
    M['M8'] = assess_reversal(c, M['M7'])
    M['M9'] = assess_home(c, motion, shadow)
    M['M10'] = assess_close(c, motion.get('still'), lease_s)
    M['M16'] = assess_probe(c)
    return M, c


def overall(M):
    vs = [m['verdict'] for m in M.values() if m['verdict'] != 'N/A']
    return 'FAIL' if 'FAIL' in vs else ('PASS' if all(v == 'PASS' for v in vs)
                                        else 'INCONCLUSIVE')


# ------------------------------------------------------------------ bag samples (pure)
def alignment_status(al):
    """The bag-to-node clock alignment and its robust uncertainty (section 5.2)."""
    if al is None:
        return {'ok': False, 'reason': 'no alignment (fewer than 10 paired statuses)'}
    p = al.get('p05_p95_ms')
    if not isinstance(p, (list, tuple)) or len(p) != 2 or not all(number(x) for x in p):
        return {'ok': False, 'reason': 'no alignment percentiles'}
    u = max(abs(p[0]), abs(p[1])) / 1000.0
    ok = u <= ALIGN_UNCERTAINTY_MAX_S
    return {'ok': ok, 'uncertainty_s': u, 'offset_s': al.get('offset_s'),
            'reason': None if ok else f'uncertainty {u:.4f} s above {ALIGN_UNCERTAINTY_MAX_S} s'}


def make_sim_of(jts):
    """sim time at node time t: the stamp of the joint state received nearest t, within the node's
    joint-state staleness (0.5 s). jts: [(t, sim)] sorted by t."""
    ts, ss = [t for t, _ in jts], [s for _, s in jts]

    def sim_of(t):
        if t is None or not ts:
            return None
        i = bisect.bisect_left(ts, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(ts) and abs(ts[j] - t) <= sc.JS_STALE_S and \
                    (best is None or abs(ts[j] - t) < abs(ts[best] - t)):
                best = j
        return None if best is None else ss[best]
    return sim_of


def find_boundary(t, index, sim_of):
    """Section 5.2: the bag pose nearest node time t, within 0.05 s of sim, at rest (every pose
    within 0.1 s of sim of it lies within 0.5 mm in xy). index: [(sim, pose)] sorted by sim."""
    st = sim_of(t)
    if st is None:
        return {'pose': None, 'reason': 'no sim time at the instant'}
    sims = [s for s, _ in index]
    i = bisect.bisect_left(sims, st)
    near = [j for j in (i - 1, i) if 0 <= j < len(index)]
    j = min(near, key=lambda j: abs(sims[j] - st), default=None)
    if j is None or abs(sims[j] - st) > BOUNDARY_SIM_TOL_S:
        return {'pose': None, 'instant_sim': st,
                'reason': f'no bag pose within {BOUNDARY_SIM_TOL_S} s of sim'}
    s0, p0 = index[j]
    win = index[bisect.bisect_left(sims, s0 - REST_WINDOW_SIM_S):
                bisect.bisect_right(sims, s0 + REST_WINDOW_SIM_S)]
    if len(win) < 2:
        return {'pose': None, 'instant_sim': st,
                'reason': 'rest not measurable (fewer than 2 poses within 0.1 s of sim)'}
    moved = max(math.hypot(p['xy'][0] - p0['xy'][0], p['xy'][1] - p0['xy'][1]) for _, p in win)
    if moved > REST_MAX_M:
        return {'pose': None, 'instant_sim': st, 'moved_mm': moved * 1000,
                'reason': f'not at rest: {moved * 1000:.3f} mm within 0.1 s of sim'}
    return {'pose': p0, 'sim': s0, 'instant_sim': st, 'moved_mm': moved * 1000,
            'rest_window_poses': len(win), 'reason': None}


def motion_from_samples(poses, jts, alignment, goals, t_arm, t_dis, joints=None):
    """The bag-based measurements. poses: [(t, pose)] and jts: [(t, sim)] on the node clock (bag
    receipt minus the alignment offset), sorted; joints: [(t, {joint: q})] for the stillness."""
    out = {'alignment': alignment}
    keys = ('boundaries', 'z_stand', 'tilt_max', 'all_sample_maxima', 'still')
    if not alignment.get('ok'):
        out.update(dict.fromkeys(keys))
        out['boundaries'] = {k: {'pose': None, 'reason': f'alignment: {alignment.get("reason")}'}
                             for k in BOUNDARY_GOALS}
        return out
    sim_of = make_sim_of(jts)
    index = sorted(((sim_of(t), p) for t, p in poses if sim_of(t) is not None),
                   key=lambda x: x[0])

    def done(k):
        return None if k not in goals else goals[k].get('done_t')
    times = {0: t_arm, **{k: done(k) for k in BOUNDARY_GOALS[1:]}}
    out['boundaries'] = {k: find_boundary(t, index, sim_of) if t is not None else
                         {'pose': None, 'reason': f'no time for B{k}'} for k, t in times.items()}
    settle = [p['z'] for t, p in poses if t_arm is not None and t_arm - 10.0 <= t <= t_arm]
    out['z_stand'] = statistics.mean(settle) if settle else None
    spans = {'SC': (t_arm, done(9)), 'F': (done(9), done(54)), 'R': (done(54), done(99)),
             'X': (done(99), done(110)), 'run': (t_arm, t_dis)}
    out['tilt_max'] = {k: None if None in (a, b) else max(
        (p['tilt'] for t, p in poses if a <= t <= b and number(p.get('tilt'))), default=None)
        for k, (a, b) in spans.items()}
    b9 = (out['boundaries'].get(9) or {}).get('pose')
    a, b = done(9), done(99)
    inside = [p for t, p in poses if None not in (a, b) and a <= t <= b]
    out['all_sample_maxima'] = None if b9 is None or not inside else {
        'lateral_m': max(abs(body_delta(b9, p)[1]) for p in inside),
        'heading_rad': max(abs(body_delta(b9, p)[2]) for p in inside), 'samples': len(inside),
        'note': 'every bag sample from B9 to B99 in B9\'s frame, designed sway included; '
                'recorded only (OD2)'}
    out['still'] = sb.assess_still(t_dis, sim_of, poses, joints) \
        if t_dis is not None and joints else None
    return out


# ------------------------------------------------------------------ M13, M15
def assess_ownership(counts, topic_present, topic_count, support, ctl, feedback_n, shadow):
    n = 0 if shadow else EXPECTED_GOALS
    terms = {'goals_111': counts.get('goals_sent') == EXPECTED_GOALS,
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


def assess_restore_and_library(report_text, enabled_sha, disabled_sha, run_install):
    r = sc.assess_restore(report_text)
    lib = sc.assess_library(enabled_sha, disabled_sha)
    gates = None if not isinstance(run_install, dict) else run_install.get('gates')
    checks = {'restore_report_normal_installation': as_check(r),
              'library_identity': as_check(lib),
              'run_installation_imports_all_three_gates_false': None if not isinstance(
                  gates, dict) else (set(gates) == set(GATES)
                                     and all(v is False for v in gates.values()))}
    return {'verdict': verdict_of(checks), 'checks': checks, 'restore': r, 'library': lib,
            'run_installation': run_install}


# ------------------------------------------------------------------ loading (read-only)
def load_and_score(run, bag, disabled_library, restore_report, run_install_gates, harness,
                   shadow=False):
    import sqlite3
    import adjudicate_s2 as adj
    import response_law as rl
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from spiderx_controller import joint_safety
    from spiderx_controller import m55_locomotion as loc
    from spiderx_controller.m55_rclpy import body_pose_sample
    cfg = loc.load_config()
    lease_s = cfg.command.lease_s
    lib_path = os.path.join(run, 'node', 'library.json')
    if os.path.exists(lib_path):
        lease_s = json.load(open(lib_path)).get('config', {}).get('command', {}).get(
            'lease_s', lease_s)
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
    alignment = alignment_status(al)
    off = None if al is None else al['offset_s']
    tol = None if al is None else al['tolerance_s']
    conv = (lambda t: t - off) if off is not None else (lambda t: None)
    mono = {k: [conv(t) for t in v] for k, v in receipts.items()} if off is not None else {}
    poses_m = [(conv(t), p) for t, p in poses] if off is not None else []
    jts = [(conv(j.receipt_ns / 1e9), dict(zip(j.names, j.positions)), j.stamp_ns / 1e9)
           for j in js] if off is not None else []
    C0, c = score_events(ev, drv, None, shadow, lease_s)
    motion = motion_from_samples(poses_m, [(t, s) for t, _, s in jts], alignment, c['goals'],
                                 c['t_arm'], c['t_dis'], [(t, q) for t, q, _ in jts])
    M, c = score_events(ev, drv, motion, shadow, lease_s)
    gl = c['gl']
    fbk = collections.defaultdict(list)
    for x in cmds:
        if x.source == 'feedback' and off is not None:
            fbk[x.goal_id].append((x.receipt_ns / 1e9 - off, max(abs(e) for e in x.error)))
    if shadow:
        M['M11'] = {'verdict': 'N/A', 'note': SHADOW_NA}
    elif off is None:
        M['M11'] = {'verdict': 'UNMEASURED', 'reason': 'no bag alignment'}
    else:
        M['M11'] = s4a.assess_tracking(gl, dict(fbk), tol, None)
    M['M12'] = sc.assess_continuity(gl, c['refusals'])
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

    def stamp(ln):
        return float(re.search(r'\[(\d+\.\d+)\]', ln).group(1)) - (off or 0.0)
    started = [stamp(ln) for ln in blog if 'Recording...' in ln]
    stopped = [stamp(ln) for ln in blog if 'Recording stopped' in ln]
    subs = [stamp(ln) for ln in blog if 'Subscribed to topic' in ln]
    graphs = [(st.get('monitors') or {}).get('graph') for t, _, st in c['sts']
              if t0 is not None and t1 is not None and t0 <= t <= t1]

    def pubs(name):
        p = os.path.join(run, 'captures', name)
        txt = open(p).read() if os.path.exists(p) else ''
        m = re.search(re.escape(sc.COMMAND_TOPIC) + r'.*?Publisher count: (\d+)', txt, re.S)
        return int(m.group(1)) if m else None
    js_gap = sc.max_gap(mono.get('/joint_states', []), t0, t1) if t0 and t1 and mono else None
    pose_gap = sc.max_gap(mono.get(sc.TOPIC_POSE, []), t0, t1) if t0 and t1 and mono else None
    support = {
        'requested_from_recorder': sc.COMMAND_TOPIC in requested,
        'recording_covers_goal_interval': bool(started and stopped) and off is not None
        and t0 is not None and t1 is not None and started[0] <= t0 and stopped[0] >= t1,
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
    counts = {'goals_sent': len(gl), 'cancels_sent': len(c['cancels']),
              'max_foreign': max((g.get('foreign_action_clients', -1) for g in graphs
                                  if isinstance(g, dict)), default=None),
              'max_publishers': max((g.get('command_topic_publishers', -1) for g in graphs
                                     if isinstance(g, dict)), default=None)}
    present = sc.COMMAND_TOPIC in [n for n, _ in topics.values()]
    M['M13'] = assess_ownership(counts, present, len(receipts.get(sc.COMMAND_TOPIC, [])),
                                support, ctl, sum(len(v) for v in fbk.values()), shadow)
    M['M14'] = s4c.assess_margins(s4c.min_joint_margin([(t, q) for t, q, _ in jts], limits),
                                  c['sts'], c['t_arm'],
                                  c['q']['t_before_write'] if c['q'] else None, c['result'])
    run_install = json.load(open(run_install_gates)) if run_install_gates and \
        os.path.exists(run_install_gates) else None
    M['M15'] = assess_restore_and_library(
        open(restore_report).read() if restore_report and os.path.exists(restore_report)
        else None, sc.sha(lib_path), sc.sha(disabled_library), run_install)
    M = dict(sorted(M.items(), key=lambda kv: int(kv[0][1:])))
    v = overall(M)
    return {'schema': SCHEMA, 'shadow': shadow, 'M': M, 'lease_s': lease_s,
            'overall': f'{v} (SHADOW ONLY: not S4d acceptance)' if shadow else v,
            'alignment': {'adjudicate_s2': al, **alignment},
            'bag': {**meta, 'sha256_after_pose_scan': after}}


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for k in ('--run', '--bag', '--out'):
        a.add_argument(k, required=True)
    a.add_argument('--disabled-library')
    a.add_argument('--restore-report')
    a.add_argument('--run-install-gates',
                   help='JSON from the run installation after restoration: {"install_base": ..., '
                        '"gates": {<module>.<GATE>: false, ...}} (OD12)')
    a.add_argument('--harness')
    a.add_argument('--shadow', action='store_true',
                   help='a shadow rehearsal: the signed bands and tracking are not applicable')
    o = a.parse_args(argv)
    if os.path.exists(o.out) and os.listdir(o.out):
        print(f'refusing to write into non-empty {o.out}', file=sys.stderr)
        return 3
    os.makedirs(o.out, exist_ok=True)
    res = load_and_score(o.run, o.bag, o.disabled_library, o.restore_report,
                         o.run_install_gates, o.harness, o.shadow)
    with open(os.path.join(o.out, 'summary.json'), 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
    print(json.dumps({k: v['verdict'] for k, v in res['M'].items()}), res['overall'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
