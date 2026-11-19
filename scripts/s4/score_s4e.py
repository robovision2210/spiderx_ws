#!/usr/bin/env python3
"""Score an S4e session (docs/S4E_CONTRACT.md section 8), offline. Frozen with the contract:
written after it and before any S4e session; after its first test it changes only to fix a defect,
never to change a criterion. Generic helpers of the frozen S4b and S4d scorers are imported
unchanged.

H1 provenance     observer human-tty with its terminal checks; preflight human-tty; the operator's
                  attestation with its sentence; no S2-S4d driver at start or end; the teleop's
                  pseudo-terminal held only by the observer (master) and the teleop's session
                  (slave). Any scripted marker or driver record: FAIL. Missing: UNMEASURED.
H2 plan           keys exactly a; hold (K2, 10-20 s); hold with one space inside and w after
                  (K3); h; hold with one estop inside (K5); q; only listed bytes; requests arm,
                  stop, home, estop, then stop x2 and disarm x2; refusals only the two close
                  disarms; <= 30 goals, all level 0, no level_change; one cancel (the estop's);
                  one fault (estop); no command rejection; every status in the mode; the
                  observer's result 'session complete'
H3 key repeat     preflight PASS at the contract's 10 s; K2 >= 10 s, every gap < 0.8 s; no
                  'key repeat gap' line in K2 (+0.1 s); no stop from K2's first goal to its last
                  character
H4 release        t_stop - t_last in [0.80, 1.00] s (STOPPING intent_stop, the first transition
                  after K2's last character); READY within the goals in flight + 1.0 s of sim;
                  no goal until K3
H5 stop, latch    space inside K3; stop accepted at or after it; STOPPING (stop) within 0.050 s
                  if a goal was in flight (none: UNMEASURED); READY within the goal + 1.0 s of
                  sim; w held >= 1.0 s after READY (else UNMEASURED); no goal until h
H6 home           h at READY; b != 0: one homing, <= 4 goals, READY homed at 0 within 12.0 s of
                  sim; b = 0: 'already at a cycle boundary', no goal; no goal until K5
H7 estop          x inside K5 while a goal was active; one estop fault, FAULTED and one cancel
                  for it within 0.050 s; cancel response 0; result CANCELED (SUCCEEDED or none:
                  UNMEASURED; other: FAIL); 10 s of sim held FAULTED with no goal, joint motion
                  <= 0.01 rad, tilt <= 0.10 rad, z >= 0.045 m; no goal after; qualifier recorded
H8 close          q after the hold; then stop x2 (FAULTED message) and disarm x2
                  (faulted_reset_required); no transition; the teleop exits 0 within 15 s; no
                  byte relayed after q
H9 safety         the observer completed; travel <= 0.15 m, tilt <= 0.10, z >= 0.045, |lateral|
                  <= 30 mm, |heading| <= 0.10 rad from the arm pose; joint margin >= 0.02 rad;
                  live tracking <= 0.05 rad (> 0.02 a finding; shadow N/A); node ages in limits
H10 traffic       0 foreign action clients and 0 command publishers in every status; captures 0;
                  joint_trajectory absent with recorder support; live: controller received =
                  accepted = goals, 1 cancel request; shadow: 0, 0, 0 and no feedback
H11 restore       as S4d-E E12 (restore report, library identity, run installation gates)
H12 evidence      present: observer start/end, node events, final status, library, bag hash,
                  preflight, attestation, scans, captures; consistent byte counts and times

H3-H8 are human-input criteria: PASS only with H1 PASS; H1 FAIL makes them FAIL ("scripted-input
evidence can never satisfy a human-input criterion"), H1 UNMEASURED makes them UNMEASURED; the
underlying verdict is kept in each detail. Missing evidence is UNMEASURED, never PASS. Overall:
FAIL if any check fails, otherwise INCONCLUSIVE if any is UNMEASURED, otherwise PASS; a shadow
result reads "... (SHADOW ONLY: not S4e acceptance)".

Usage: score_s4e.py --session <dir> --bag <bag_0.db3> --disabled-library <file>
                    --restore-report <report> --run-install-gates <json> --out <new dir> [--shadow]
Session layout: preflight.json, attestation.txt, observer/observer.jsonl, run/node/events.jsonl,
run/node/final_status.json, run/node/library.json, run/captures/{before,after}_topics.txt,
run/launch.log, run/bag_record.log, run/process_scan_{start,end}.json."""
import argparse
import bisect
import collections
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 's3'), os.path.join(HERE, '..', 's2')]
import score_s4d as s4d  # noqa: E402  (frozen; generic helpers only)

sb, s4a, s4c, sc = s4d.sb, s4d.s4a, s4d.s4c, s4d.sc
verdict_of, number = sb.verdict_of, s4d.number
SCHEMA = 'spiderx.s4e_score/1'
HUMAN, SCRIPTED_SOURCES = 'human-tty', ('scripted-pty-test', 'test-fixture')
SCRIPTED_RULE = 'scripted-input evidence can never satisfy a human-input criterion'
HUMAN_CRITERIA = ('H3', 'H4', 'H5', 'H6', 'H7', 'H8')
ATTESTATION_PHRASES = ('by pressing keys on a physical keyboard myself',
                       'no script, macro, key remapper or other program generated any key')
ARM, FORWARD, STOP, HOME, ESTOP, QUIT = 'arm', 'forward', 'stop', 'home', 'estop', 'quit'
ALLOWED_BYTES = frozenset(b'awW hx\x1bq')
LEVEL, MAX_GOALS = 0, 30
HOLD_S, REPEAT_WINDOW_S = 0.8, 2.0
K2_MIN_S, K2_MAX_S = 10.0, 20.0
PREFLIGHT_MIN_HOLD_S = 10.0
RELEASE_MIN_S, RELEASE_MAX_S = 0.80, 1.00
READY_MARGIN_SIM_S, WALL_BACKSTOP_S = 1.0, 120.0
HANDLER_S = 0.050
REPEAT_GAP_LINE_AFTER_S = 0.1
LATCH_AFTER_READY_S = 1.0
HOME_MAX_GOALS, HOME_SIM_S = 4, 12.0
HOLD_SIM_S, HOLD_JOINT_MAX_RAD = 10.0, 0.01
TILT_MAX_RAD, Z_MIN_M, TRAVEL_MAX_M = 0.10, 0.045, 0.15
LATERAL_MAX_M, HEADING_MAX_RAD = 0.030, 0.10
JOINT_MARGIN_MIN_RAD = 0.02
TRACKING_ABORT_RAD, TRACKING_FINDING_RAD = 0.05, 0.02
MOVING_BODY_RAD = 0.005
TELEOP_EXIT_WALL_S = 15.0
COVERAGE_GAP_S = 2.0
AGE_LIMITS_S = {'joint_states_age_s': 0.5, 'body_pose_age_s': 1.0, 'graph_age_s': 3.0}
FAULTED_STOP_MESSAGE = 'not moving (FAULTED); held until a zero command'
RESET_REQUIRED = 'faulted_reset_required'
HOME_AT_ZERO = 'already at a cycle boundary'
CLOSE_COUNTS = {'stop': 2, 'disarm': 2}
SESSION_COMPLETE = 'session complete'
SHADOW_NA = 'not applicable: dispatch disabled (shadow rehearsal)'
MODE = {False: ('dispatch', True), True: ('shadow', False)}     # shadow -> (status mode, gate)


def res(checks, **detail):
    return {'verdict': verdict_of(checks), 'checks': checks, **detail}


def na(reason):
    return {'verdict': 'N/A', 'checks': {}, 'reason': reason}


def unmeasured(reason, **detail):
    return {'verdict': 'UNMEASURED', 'checks': {}, 'reason': reason, **detail}


# ------------------------------------------------------------------ extraction (pure)
def observer_context(obs):
    """The observer's records (observer.jsonl) as the scorer needs them."""
    by = collections.defaultdict(list)
    for r in obs:
        by[r.get('kind')].append(r)
    keys = by['key']
    end = by['end'][-1] if by['end'] else None
    return {
        'start': by['start'][0] if by['start'] else None,
        'end': end, 'summary': None if end is None else end.get('summary'),
        'keys': keys,
        'tokens': [(k['t_read'], a) for k in keys for a in k.get('actions', [])],
        'not_relayed': by['not_relayed'],
        'statuses': [(r['t_receipt'], r['status']) for r in by['status']
                     if isinstance(r.get('status'), dict)],
        'teleop_out': [(r['t'], r.get('text', '')) for r in by['teleop_out']],
        'holders': {r.get('at'): r for r in by['pty_holders']},
        'scans': {r.get('at'): r.get('drivers') for r in by['driver_scan']},
        'teleop_started': by['teleop_started'][0] if by['teleop_started'] else None,
        'teleop_exit': by['teleop_exit'][-1] if by['teleop_exit'] else None,
    }


def node_context(ev):
    goals = sb.goals_from_events(ev)
    for e in ev:
        s = e.get('seq')
        if e['event'] == 'cancel_response' and s in goals:
            goals[s].update(cancel_response_code=e.get('code'), cancel_response_t=e['wall'])
        elif e['event'] == 'result' and s in goals:
            goals[s]['error_string'] = e.get('error_string')
    return {
        'goals': goals, 'gl': list(goals.values()),
        'transitions': sb.transitions_from_events(ev),
        'requests': [{'request': e['request'], 'message': e.get('message'), 't': e['wall']}
                     for e in ev if e['event'] == 'request'],
        'refusals': [{'request': e['request'], 'reason': e.get('reason'), 't': e['wall']}
                     for e in ev if e['event'] == 'refused'],
        'cancels': [{'seq': e.get('seq'), 't': e['wall']} for e in ev
                    if e['event'] == 'cancel_sent'],
        'faults': [{'reason': e.get('reason'), 't': e['wall']} for e in ev
                   if e['event'] == 'fault'],
        'level_changes': [e for e in ev if e['event'] == 'level_change'],
        'homing': [{'t': e['wall'], 'direction': e.get('direction'), 'phases': e.get('phases')}
                   for e in ev if e['event'] == 'homing'],
        'command_rejected': [e for e in ev if e['event'] == 'command_rejected'],
    }


def macro(tokens):
    """Decoded keys -> steps. Forward characters within 2.0 s of the previous one form a hold; a
    stop or estop less than 0.8 s after the hold's last forward character is inside it."""
    out, cur = [], None
    for t, a in tokens:
        if a == FORWARD:
            if cur is not None and t - cur['times'][-1] <= REPEAT_WINDOW_S:
                cur['times'].append(t)
            else:
                cur = {'kind': 'hold', 'times': [t], 'inner': []}
                out.append(cur)
        elif cur is not None and a in (STOP, ESTOP) and t - cur['times'][-1] < HOLD_S:
            cur['inner'].append((t, a))
        else:
            cur = None
            out.append({'kind': a, 't': t})
    return out


def sim_mapper(statuses):
    """sim time at observer time t, interpolated between the statuses' receipts (None outside
    them or across a gap above 2 s)."""
    pts = sorted((t, (st.get('monitors') or {}).get('sim_time_s')) for t, st in statuses)
    pts = [(t, s) for t, s in pts if number(s)]
    ts = [t for t, _ in pts]

    def sim_of(t):
        if t is None or not pts or t < ts[0] or t > ts[-1]:
            return None
        i = bisect.bisect_left(ts, t)
        if ts[i] == t:
            return pts[i][1]
        (ta, sa), (tb, sb_) = pts[i - 1], pts[i]
        if tb - ta > COVERAGE_GAP_S:
            return None
        return sa + (sb_ - sa) * (t - ta) / (tb - ta)
    return sim_of


def context(ev, obs, preflight=None, attestation=None, harness_scans=None,
            driver_records=False, files=None):
    c = {**node_context(ev), 'obs': observer_context(obs), 'preflight': preflight,
         'attestation': attestation, 'harness_scans': harness_scans,
         'driver_records': driver_records, 'files': files or {}}
    c['steps'] = macro(c['obs']['tokens'])
    c['sim_of'] = sim_mapper(c['obs']['statuses'])
    c['t_arm'] = next((tr['t'] for tr in c['transitions'] if tr['why'] == 'arm'), None)
    return c


def plan_steps(c):
    """The K1..K6 steps if the keys parse as the plan, else None."""
    s = c['steps']
    kinds = [x['kind'] for x in s]
    if kinds != [ARM, 'hold', 'hold', HOME, 'hold', QUIT]:
        return None
    k2, k3, k5 = s[1], s[2], s[4]
    ok = (not k2['inner'] and [a for _, a in k3['inner']] == [STOP]
          and k3['times'][-1] > k3['inner'][0][0]
          and [a for _, a in k5['inner']] == [ESTOP])
    return None if not ok else {'K1': s[0]['t'], 'K2': k2, 'K3': k3, 'K4': s[3]['t'], 'K5': k5,
                                'K6': s[5]['t'], 'space': k3['inner'][0][0],
                                'x': k5['inner'][0][0]}


def first(items, pred):
    return next((x for x in items if pred(x)), None)


def goals_sent_in(c, t0, t1):
    return [g for g in c['gl'] if g.get('sent') is not None and t0 <= g['sent'] < t1]


def in_flight(c, t):
    """Goals sent at or before t with no result before t."""
    return [g for g in c['gl'] if g.get('sent') is not None and g['sent'] <= t
            and (g.get('result_t') is None or g['result_t'] >= t)]


def gaps(times):
    return [b - a for a, b in zip(times, times[1:])]


# ------------------------------------------------------------------ H1 provenance
def attestation_ok(text):
    norm = ' '.join(text.lower().split())
    return all(p in norm for p in ATTESTATION_PHRASES)


def assess_provenance(c):
    o, p = c['obs'], c['preflight']
    start = o['start']
    src = None if start is None else start.get('input_source')
    psrc = None if p is None else p.get('input_source')
    scripted = [f'observer input_source {src!r}'] if src in SCRIPTED_SOURCES else []
    scripted += [f'preflight input_source {psrc!r}'] if psrc in SCRIPTED_SOURCES else []
    if c['driver_records']:
        scripted.append('an S2-S4d driver record is in the session')
    tty = (start or {}).get('tty') or {}
    scans = o['scans']
    hs = c['harness_scans'] or {}
    scan_lists = [scans.get('start'), scans.get('end')] + list(hs.values())
    holders = o['holders']
    started = o['teleop_started']
    obs_pid = (start or {}).get('observer_pid')
    tl_pid = None if started is None else started.get('pid')

    def holders_ok():
        if set(holders) != {'start', 'end'} or obs_pid is None or tl_pid is None:
            return None
        return all([h['pid'] for h in holders[at].get('master_holders', [])] == [obs_pid]
                   and all(s.get('sid') == tl_pid for s in holders[at].get('slave_holders', []))
                   for at in ('start', 'end'))
    checks = {
        'observer_input_source_human_tty': None if start is None else src == HUMAN,
        'observer_terminal_checks': None if start is None else all(
            tty.get(k) is True for k in ('stdin_tty', 'stdout_tty', 'foreground')),
        'preflight_input_source_human_tty': None if p is None else (
            psrc == HUMAN and p.get('human_check') is True),
        'attestation_with_its_sentence': None if c['attestation'] is None else attestation_ok(
            c['attestation']),
        'no_driver_at_start_or_end': None if scans.get('start') is None or scans.get(
            'end') is None else all(not x for x in scan_lists),
        'pty_held_only_by_the_observer_and_the_teleop': holders_ok(),
    }
    if scripted:
        return {'verdict': 'FAIL', 'checks': checks, 'scripted_markers': scripted,
                'reason': SCRIPTED_RULE}
    return res(checks)


# ------------------------------------------------------------------ H2 plan
def assess_plan(c, shadow):
    o = c['obs']
    if not o['keys'] and o['start'] is None:
        return unmeasured('no observer record')
    if not c['transitions'] and not c['gl']:
        return unmeasured('no node events')
    k = plan_steps(c)
    relayed = b''.join(bytes.fromhex(r['hex']) for r in o['keys'])
    ignored = any('ignored' in r.get('actions', []) for r in o['keys'])
    t_q = None if k is None else k['K6']
    before_q = [r['request'] for r in c['requests'] if t_q is None or r['t'] < t_q]
    after_q = collections.Counter(r['request'] for r in c['requests'] if t_q is not None
                                  and r['t'] >= t_q)
    after_q.update(r['request'] for r in c['refusals'] if t_q is not None and r['t'] >= t_q)
    est = first(c['requests'], lambda r: r['request'] == ESTOP)
    active = [] if est is None else [g['seq'] for g in in_flight(c, est['t'])]
    smode, sgate = MODE[shadow]
    sts = o['statuses']
    summ = o['summary'] or {}
    k2 = None if k is None else k['K2']['times']
    checks = {
        'keys_parse_as_the_plan': k is not None,
        'only_listed_bytes': bool(relayed) and all(b in ALLOWED_BYTES for b in relayed)
        and not ignored,
        'k2_lasts_10_to_20_s': None if k2 is None else K2_MIN_S <= k2[-1] - k2[0] <= K2_MAX_S,
        'requests_before_q': None if t_q is None else before_q == [ARM, STOP, HOME, ESTOP],
        'close_requests_counts': None if t_q is None else dict(after_q) == CLOSE_COUNTS,
        'only_the_two_close_refusals': None if t_q is None else (
            len(c['refusals']) == 2 and all(r['request'] == 'disarm' and r['reason'] ==
                                            RESET_REQUIRED and r['t'] >= t_q
                                            for r in c['refusals'])),
        'goal_budget_and_level': len(c['gl']) <= MAX_GOALS and all(
            g.get('level') == LEVEL for g in c['gl']) and not c['level_changes'],
        'one_cancel_the_estops': len(c['cancels']) == 1 and c['cancels'][0]['seq'] in active,
        'one_fault_the_estop': [f['reason'] for f in c['faults']] == [ESTOP],
        'no_command_rejection': not c['command_rejected'],
        'every_status_in_the_mode': None if not sts else all(
            st.get('mode') == smode and st.get('dispatch_gate') is sgate for _, st in sts),
        'observer_result_session_complete': None if not summ else summ.get(
            'result') == SESSION_COMPLETE,
    }
    return res(checks, steps=None if k is None else {
        'K2_s': None if k2 is None else k2[-1] - k2[0], 'requests_before_q': before_q,
        'requests_after_q': dict(after_q)})


# ------------------------------------------------------------------ H3 key repeat (1.11)
def assess_repeat(c):
    k = plan_steps(c)
    p = c['preflight']
    if k is None:
        return unmeasured('the keys do not parse as the plan (H2)')
    times = k['K2']['times']
    first_t, last_t = times[0], times[-1]
    g1 = first(c['gl'], lambda g: g.get('sent') is not None and g['sent'] >= first_t)
    pv = None if p is None else (p.get('verdict') or {}).get('verdict')
    lines = [t for t, txt in c['obs']['teleop_out'] if 'key repeat gap' in txt
             and first_t <= t <= last_t + REPEAT_GAP_LINE_AFTER_S]
    stops = [] if g1 is None else [tr for tr in c['transitions']
                                   if g1['sent'] <= tr['t'] <= last_t
                                   and tr['state'] not in ('STARTING', 'WALKING')]
    dur, gl_ = last_t - first_t, gaps(times)
    checks = {
        'preflight_pass_at_10_s': None if p is None or pv in (None, 'UNMEASURED') or
        p.get('min_hold_s') != PREFLIGHT_MIN_HOLD_S else pv == 'PASS',
        'preflight_repeat_settings_recorded': None if p is None else isinstance(
            (p.get('environment') or {}).get('queries'), dict),
        'k2_at_least_10_s': None if dur < K2_MIN_S else True,
        'k2_every_gap_below_0_8_s': all(g < HOLD_S for g in gl_),
        'no_key_repeat_gap_line': not lines,
        'k2_started_walking': g1 is not None and g1['sent'] <= last_t,
        'no_stop_before_the_last_character': not stops,
    }
    return res(checks, k2={'duration_s': dur, 'characters': len(times),
                           'first_repeat_delay_s': gl_[0] if gl_ else None,
                           'max_gap_s': max(gl_) if gl_ else None,
                           'median_interval_s': sorted(gl_[1:])[len(gl_[1:]) // 2]
                           if len(gl_) > 1 else None},
               repeat_gap_lines=lines, stops_during_hold=stops)


# ------------------------------------------------------------------ H4 release (1.13, 1.7)
def assess_release(c):
    k = plan_steps(c)
    if k is None:
        return unmeasured('the keys do not parse as the plan (H2)')
    t_last = k['K2']['times'][-1]
    tr1 = first(c['transitions'], lambda tr: tr['t'] > t_last)
    if tr1 is None:
        return unmeasured('no node transition after the last K2 character')
    t_stop = tr1['t']
    ready = first(c['transitions'], lambda tr: tr['t'] > t_stop and tr['state'] == 'READY')
    flight = [g for g in c['gl'] if g.get('sent') is not None and g['sent'] <= t_stop
              and (g.get('result_t') is None or g['result_t'] >= t_last)]
    bound = sum(g['duration_s'] for g in flight) + READY_MARGIN_SIM_S if flight else None
    s0 = c['sim_of'](t_last)
    s1 = None if ready is None else c['sim_of'](ready['t'])
    k3_first = k['K3']['times'][0]
    delta = t_stop - t_last
    checks = {
        'first_transition_is_stopping_intent_stop': tr1['state'] == 'STOPPING'
        and tr1['why'] == 'intent_stop',
        'detection_0_80_to_1_00_s': RELEASE_MIN_S <= delta <= RELEASE_MAX_S,
        'ready_within_the_goals_in_flight_plus_1_s_sim': None if ready is None or bound is None
        or s0 is None or s1 is None else s1 - s0 <= bound,
        'ready_within_the_wall_backstop': None if ready is None else
        ready['t'] - t_last <= WALL_BACKSTOP_S,
        'no_goal_until_k3': not goals_sent_in(c, t_stop, k3_first),
    }
    if ready is None:
        checks['ready_within_the_goals_in_flight_plus_1_s_sim'] = False
    return res(checks, t_last=t_last, t_stop=t_stop, detection_s=delta,
               goals_in_window=[g['seq'] for g in flight],
               ready_sim_s=None if s0 is None or s1 is None else s1 - s0, bound_sim_s=bound)


# ------------------------------------------------------------------ H5 stop and latch
def assess_stop(c):
    k = plan_steps(c)
    if k is None:
        return unmeasured('the keys do not parse as the plan (H2)')
    t_sp, t_h = k['space'], k['K4']
    req = first(c['requests'], lambda r: r['request'] == STOP and t_sp <= r['t'] < t_h)
    if req is None:
        return unmeasured('no stop request between the space and h')
    flight = in_flight(c, req['t'])
    if not flight:
        return unmeasured('no goal in flight at the stop request: the stop was not exercised '
                          'while walking')
    stopping = first(c['transitions'], lambda tr: tr['state'] == 'STOPPING'
                     and tr['why'] == STOP and req['t'] <= tr['t'] <= req['t'] + HANDLER_S)
    ready = first(c['transitions'], lambda tr: tr['t'] > req['t'] and tr['state'] == 'READY')
    s0 = c['sim_of'](req['t'])
    s1 = None if ready is None else c['sim_of'](ready['t'])
    bound = flight[-1]['duration_s'] + READY_MARGIN_SIM_S
    k3_last = k['K3']['times'][-1]
    latch = None if ready is None else (True if k3_last >= ready['t'] + LATCH_AFTER_READY_S
                                        else None)
    checks = {
        'stopping_stop_within_0_050_s': stopping is not None,
        'ready_within_the_goal_plus_1_s_sim': False if ready is None else (
            None if s0 is None or s1 is None else s1 - s0 <= bound),
        'ready_within_the_wall_backstop': False if ready is None else
        ready['t'] - req['t'] <= WALL_BACKSTOP_S,
        'w_held_1_s_after_ready': latch,
        'no_goal_from_stopping_until_h': not goals_sent_in(
            c, req['t'] if stopping is None else stopping['t'], t_h),
    }
    return res(checks, t_request=req['t'], goal_in_flight=flight[-1]['seq'],
               ready_sim_s=None if s0 is None or s1 is None else s1 - s0, bound_sim_s=bound)


# ------------------------------------------------------------------ H6 home
def state_at(c, t):
    tr = [x for x in c['transitions'] if x['t'] <= t]
    return tr[-1]['state'] if tr else None


def assess_home(c):
    k = plan_steps(c)
    if k is None:
        return unmeasured('the keys do not parse as the plan (H2)')
    t_h, k5_first = k['K4'], k['K5']['times'][0]
    req = first(c['requests'], lambda r: r['request'] == HOME and t_h <= r['t'] < k5_first)
    if req is None:
        return unmeasured('no home request after h')
    homing = first(c['homing'], lambda h: req['t'] - HANDLER_S <= h['t'] <= req['t'])
    checks = {'h_at_ready': state_at(c, req['t'] - 1e-9) == 'READY'}
    if req['message'] == HOME_AT_ZERO:
        checks.update(at_boundary_0_no_homing=homing is None,
                      no_goal_until_k5=not goals_sent_in(c, req['t'], k5_first))
        return res(checks, case='boundary 0')
    ready = first(c['transitions'], lambda tr: tr['t'] > req['t'] and tr['state'] == 'READY')
    goals = goals_sent_in(c, req['t'], ready['t'] if ready else k5_first)
    s0 = c['sim_of'](req['t'])
    s1 = None if ready is None else c['sim_of'](ready['t'])
    after = [st for t, st in c['obs']['statuses'] if ready is not None and t > ready['t']]
    checks.update(
        one_homing_event=homing is not None,
        at_most_4_goals=0 < len(goals) <= HOME_MAX_GOALS,
        ready_homed=ready is not None and ready['why'] == 'homed',
        at_boundary_0=None if not after else after[0].get('boundary') == 0,
        within_12_s_sim=False if ready is None else (None if s0 is None or s1 is None
                                                     else s1 - s0 <= HOME_SIM_S),
        no_goal_after_ready_until_k5=ready is not None and not goals_sent_in(
            c, ready['t'], k5_first))
    return res(checks, case='homing', goals=[g['seq'] for g in goals],
               homing_sim_s=None if s0 is None or s1 is None else s1 - s0)


# ------------------------------------------------------------------ H7 estop
def window(samples, t0, t1):
    return [(t, v) for t, v in samples if t0 <= t <= t1]


def covered(samples, t0, t1, gap=1.0):
    ts = [t for t, _ in samples]
    if not ts or ts[0] > t0 + gap or ts[-1] < t1 - gap:
        return False
    return all(b - a <= gap for a, b in zip(ts, ts[1:]))


def joint_motion(samples):
    if not samples:
        return None
    ref = samples[0][1]
    return max((abs(q[n] - ref[n]) for _, q in samples for n in q if n in ref), default=0.0)


def hold_window(c, t_res):
    """(t_res, t_end): t_end the first status time whose sim is >= sim(t_res) + 10 s."""
    s0 = c['sim_of'](t_res)
    if s0 is None:
        return None
    for t, st in c['obs']['statuses']:
        s = (st.get('monitors') or {}).get('sim_time_s')
        if t >= t_res and number(s) and s >= s0 + HOLD_SIM_S:
            return t
    return None


def assess_estop(c, b, shadow):
    k = plan_steps(c)
    if k is None:
        return unmeasured('the keys do not parse as the plan (H2)')
    req = first(c['requests'], lambda r: r['request'] == ESTOP and r['t'] >= k['x'])
    if req is None:
        return unmeasured('no estop request after x')
    active = [g for g in in_flight(c, req['t']) if g.get('accepted') is True
              and g.get('response_t') is not None and g['response_t'] <= req['t']]
    g = active[-1] if active else None
    faults = [f for f in c['faults'] if f['reason'] == ESTOP]
    fault_tr = first(c['transitions'], lambda tr: tr['state'] == 'FAULTED'
                     and tr['why'] == ESTOP)
    cancels = [x for x in c['cancels'] if g is not None and x['seq'] == g['seq']]
    status = None if g is None else g.get('status')
    t_res = None if g is None else g.get('result_t')
    t_end = None if t_res is None else hold_window(c, t_res)
    sts = [] if t_end is None else [st for t, st in c['obs']['statuses'] if t_res < t <= t_end]
    jw = [] if t_end is None else window(b.get('joints') or [], t_res, t_end)
    pw = [] if t_end is None else window(b.get('poses') or [], t_res, t_end)
    cov = t_end is not None and covered(jw, t_res, t_end) and covered(pw, t_res, t_end)
    if status == sc.STATUS_CANCELED:              # action_msgs GoalStatus codes (S3: 4, 5)
        result_ok = True
    elif status in (None, sc.STATUS_SUCCEEDED):
        result_ok = None
    else:
        result_ok = False
    checks = {
        'x_while_a_goal_was_active': g is not None,
        'one_estop_fault_within_0_050_s': len(faults) == 1 and 0 <= faults[0]['t'] - req['t']
        <= HANDLER_S and fault_tr is not None and 0 <= fault_tr['t'] - req['t'] <= HANDLER_S,
        'one_cancel_for_it_within_0_050_s': g is not None and len(cancels) == 1
        and 0 <= cancels[0]['t'] - req['t'] <= HANDLER_S and len(c['cancels']) == 1,
        'cancel_response_0': None if g is None or g.get('cancel_response_code') is None
        else g['cancel_response_code'] == 0,
        'result_canceled': result_ok,
        'hold_10_s_sim_faulted_no_goal': None if t_end is None or not sts else all(
            st.get('state') == 'FAULTED' and st.get('goal') is None for st in sts),
        'hold_joint_motion_tilt_z': None if not cov else (
            joint_motion(jw) <= HOLD_JOINT_MAX_RAD
            and all(p['tilt'] <= TILT_MAX_RAD and p['z'] >= Z_MIN_M for _, p in pw)),
        'no_goal_after_the_fault': not goals_sent_in(c, req['t'], float('inf')),
    }
    qualifier = SHADOW_NA if shadow else None
    if not shadow and g is not None:
        before = window(b.get('joints') or [], g['response_t'], req['t'])
        m = joint_motion(before)
        qualifier = None if m is None else {
            'joint_motion_before_the_cancel_rad': m,
            'reads': 'moving-body stop' if m >= MOVING_BODY_RAD else
            'before measurable motion, as S4d-E'}
    return res(checks, goal=None if g is None else g['seq'], result_status=status,
               hold_window=None if t_end is None else [t_res, t_end],
               qualifier_not_scored=qualifier)


# ------------------------------------------------------------------ H8 close
def assess_close(c, hold_end):
    t_q = next((t for t, a in c['obs']['tokens'] if a == QUIT), None)   # K6, parsed or not
    if t_q is None:
        return unmeasured('no q was relayed')
    acc = [r for r in c['requests'] if r['t'] >= t_q]
    ref = [r for r in c['refusals'] if r['t'] >= t_q]
    ex = c['obs']['teleop_exit']
    after_keys = [r for r in c['obs']['keys'] if r['t_read'] > t_q]
    checks = {
        'q_after_the_hold': None if hold_end is None else t_q >= hold_end,
        'stop_x2_accepted_with_the_faulted_message': len(acc) == 2 and all(
            r['request'] == STOP and r['message'] == FAULTED_STOP_MESSAGE for r in acc),
        'disarm_x2_refused_reset_required': len(ref) == 2 and all(
            r['request'] == 'disarm' and r['reason'] == RESET_REQUIRED for r in ref),
        'no_transition_after_q': not [tr for tr in c['transitions'] if tr['t'] >= t_q],
        'teleop_exit_0_within_15_s': None if ex is None else (
            ex.get('code') == 0 and ex['t'] - t_q <= TELEOP_EXIT_WALL_S),
        'nothing_relayed_after_q': not after_keys,
    }
    return res(checks, t_q=t_q)


# ------------------------------------------------------------------ H9 safety
def assess_safety(c, b, shadow):
    summ = c['obs']['summary'] or {}
    t_arm = c['t_arm']
    poses = [(t, p) for t, p in (b.get('poses') or []) if t_arm is not None and t >= t_arm]
    sts = [st for t, st in c['obs']['statuses'] if t_arm is not None and t >= t_arm]
    t_end = c['obs']['statuses'][-1][0] if c['obs']['statuses'] else None
    cov = bool(poses) and t_end is not None and covered(poses, t_arm, t_end, COVERAGE_GAP_S)
    ref = poses[0][1] if poses else None
    if cov:
        d = [sc_body(ref, p) for _, p in poses]
        travel = max(math.hypot(p['xy'][0] - ref['xy'][0], p['xy'][1] - ref['xy'][1])
                     for _, p in poses)
    m = b.get('min_joint_margin')
    fb = b.get('feedback') or {}
    errs = [e for samples in fb.values() for _, e in samples]
    checks = {
        'observer_completed': None if not summ else summ.get('result') == SESSION_COMPLETE,
        'travel_le_0_15_m': None if not cov else travel <= TRAVEL_MAX_M,
        'tilt_le_0_10_and_z_ge_0_045': None if not cov else all(
            p['tilt'] <= TILT_MAX_RAD and p['z'] >= Z_MIN_M for _, p in poses),
        'lateral_le_30_mm_heading_le_0_10_rad': None if not cov else all(
            abs(lat) <= LATERAL_MAX_M and abs(h) <= HEADING_MAX_RAD for _, lat, h in d),
        'joint_margin_ge_0_02_rad': None if not number(m) else m >= JOINT_MARGIN_MIN_RAD,
        'node_ages_within_limits': None if not sts else all(
            number((st.get('monitors') or {}).get(k_)) and st['monitors'][k_] <= lim
            for st in sts for k_, lim in AGE_LIMITS_S.items()),
    }
    if not shadow:
        checks['tracking_le_0_05_rad'] = None if not errs else max(errs) <= TRACKING_ABORT_RAD
    out = res(checks, max_travel_m=travel if cov else None, min_joint_margin_rad=m,
              max_tracking_error_rad=max(errs) if errs else None,
              tracking=SHADOW_NA if shadow else None)
    if errs and max(errs) > TRACKING_FINDING_RAD:
        out['finding'] = f'tracking above {TRACKING_FINDING_RAD} rad (a finding, not a failure)'
    return out


def sc_body(p0, p1):
    """(forward, lateral, heading change) of p1 in p0's body frame (S3's body_delta)."""
    dx, dy = p1['xy'][0] - p0['xy'][0], p1['xy'][1] - p0['xy'][1]
    cy, sy = math.cos(p0['yaw']), math.sin(p0['yaw'])
    h = math.atan2(math.sin(p1['yaw'] - p0['yaw']), math.cos(p1['yaw'] - p0['yaw']))
    return -sy * dx + cy * dy, cy * dx + sy * dy, h


# ------------------------------------------------------------------ H10 traffic
def assess_traffic(c, b, shadow):
    graphs = [(st.get('monitors') or {}).get('graph') for _, st in c['obs']['statuses']]
    sup = b.get('support') or {}
    ctl = b.get('ctl')
    fb = b.get('feedback') or {}
    n = len(c['gl'])
    checks = {
        'zero_foreign_clients_and_publishers': None if not graphs or None in graphs else all(
            g.get('foreign_action_clients') == 0 and g.get('command_topic_publishers') == 0
            for g in graphs),
        'captures_zero_publishers': None if b.get('capture_before') is None or b.get(
            'capture_after') is None else b['capture_before'] == 0 and b['capture_after'] == 0,
        'joint_trajectory_absent_with_support': None if not sup or not all(sup.values())
        else b.get('command_topic_count') == 0,
    }
    if shadow:
        checks['controller_0_received_0_accepted_0_cancels'] = None if ctl is None else (
            ctl.get('received') == 0 and ctl.get('accepted') == 0
            and ctl.get('cancel_requests') == 0)
        checks['no_feedback'] = not fb
    else:
        checks['controller_received_accepted_equal_goals_one_cancel'] = None if ctl is None \
            else (ctl.get('received') == n and ctl.get('accepted') == n
                  and ctl.get('cancel_requests') == 1)
    return res(checks, goals=n, controller=ctl)


# ------------------------------------------------------------------ H12 evidence
def assess_evidence(c):
    o, f = c['obs'], c['files']
    summ = o['summary'] or {}
    relay = summ.get('relay') or {}
    keys = o['keys']
    nr = sum(r.get('n', 0) for r in o['not_relayed'])
    present = {name: f.get(name) for name in ('node_events', 'final_status', 'library_json',
                                              'bag_sha256', 'captures', 'harness_scans')}
    checks = {
        'observer_start_and_end': o['start'] is not None and o['end'] is not None,
        'preflight': c['preflight'] is not None,
        'attestation': c['attestation'] is not None,
        'observer_scans': 'start' in o['scans'] and 'end' in o['scans'],
        **{k: (True if v else None) for k, v in present.items()},
        'relayed_bytes_consistent': None if not relay else (
            sum(r['n'] for r in keys) == relay.get('bytes_relayed')
            and relay.get('bytes_read') == relay.get('bytes_relayed') + nr),
        'every_t_written_ge_t_read': all(r['t_written'] >= r['t_read'] for r in keys),
    }
    for k_ in ('observer_start_and_end', 'preflight', 'attestation', 'observer_scans'):
        checks[k_] = checks[k_] or None
    return res(checks)


# ------------------------------------------------------------------ overall
def human_rule(H):
    h1 = H['H1']['verdict']
    if h1 == 'PASS':
        return H
    for k in HUMAN_CRITERIA:
        under = H[k]['verdict']
        H[k] = {**H[k], 'underlying_verdict': under,
                'verdict': 'FAIL' if h1 == 'FAIL' else 'UNMEASURED',
                'human_rule': SCRIPTED_RULE if h1 == 'FAIL' else
                'H1 is UNMEASURED: no human-input criterion can pass'}
    return H


def overall(H):
    vs = [h['verdict'] for h in H.values() if h['verdict'] != 'N/A']
    return 'FAIL' if 'FAIL' in vs else ('PASS' if vs and all(v == 'PASS' for v in vs)
                                        else 'INCONCLUSIVE')


def score(c, b, shadow, restore_text=None, enabled_sha=None, disabled_sha=None,
          run_install=None):
    H = {'H1': assess_provenance(c), 'H2': assess_plan(c, shadow), 'H3': assess_repeat(c),
         'H4': assess_release(c), 'H5': assess_stop(c), 'H6': assess_home(c),
         'H7': assess_estop(c, b, shadow)}
    hw = H['H7'].get('hold_window')
    H['H8'] = assess_close(c, None if not hw else hw[1])
    H['H9'] = assess_safety(c, b, shadow)
    H['H10'] = assess_traffic(c, b, shadow)
    H['H11'] = s4d.assess_restore_and_library(restore_text, enabled_sha, disabled_sha,
                                              run_install)
    H['H12'] = assess_evidence(c)
    H = human_rule(H)
    v = overall(H)
    return {'schema': SCHEMA, 'shadow': shadow, 'H': H,
            'overall': f'{v} (SHADOW ONLY: not S4e acceptance)' if shadow else v,
            'recorded_only': recorded_only(c)}


def recorded_only(c):
    keys = c['obs']['keys']
    lat = sorted(r['t_written'] - r['t_read'] for r in keys)
    sts = c['obs']['statuses']
    rtf = None
    if len(sts) >= 2:
        s0 = (sts[0][1].get('monitors') or {}).get('sim_time_s')
        s1 = (sts[-1][1].get('monitors') or {}).get('sim_time_s')
        if number(s0) and number(s1) and sts[-1][0] > sts[0][0]:
            rtf = (s1 - s0) / (sts[-1][0] - sts[0][0])
    p = c['preflight'] or {}
    q = (p.get('verdict') or {}).get('qualifying_hold') or {}
    return {'rtf_mean': rtf,
            'relay_latency_s': None if not lat else {'median': lat[len(lat) // 2],
                                                     'max': lat[-1]},
            'operator_first_repeat_delay_s': q.get('first_repeat_delay_s'),
            'operator_repeat_rate_hz': q.get('rate_hz')}


# ------------------------------------------------------------------ loading (read-only)
def read_jsonl(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return [json.loads(ln) for ln in f if ln.strip()]


def read_json(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def read_text(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return f.read()


def load_session(session):
    """(events, observer records, preflight, attestation, harness scans, driver records?,
    files present) from a session directory."""
    run = os.path.join(session, 'run')
    ev = read_jsonl(os.path.join(run, 'node', 'events.jsonl')) or []
    obs = read_jsonl(os.path.join(session, 'observer', 'observer.jsonl')) or []
    scans = {at: read_json(os.path.join(run, f'process_scan_{at}.json'))
             for at in ('start', 'end')}
    scans = {k: v for k, v in scans.items() if v is not None}
    drv = any(name.startswith('driver') and name.endswith(('.jsonl', '.jsonl.gz'))
              for _, _, names in os.walk(session) for name in names)
    files = {'node_events': bool(ev),
             'final_status': os.path.exists(os.path.join(run, 'node', 'final_status.json')),
             'library_json': os.path.exists(os.path.join(run, 'node', 'library.json')),
             'captures': all(os.path.exists(os.path.join(run, 'captures', f'{x}_topics.txt'))
                             for x in ('before', 'after')),
             'harness_scans': set(scans) == {'start', 'end'}}
    return (ev, obs, read_json(os.path.join(session, 'preflight.json')),
            read_text(os.path.join(session, 'attestation.txt')), scans or None, drv, files)


def load_bag(bag, c, run):
    """Bag samples on the observer clock (aligned through the statuses, as S4d-E)."""
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
    ids = {n: i for i, (n, _) in topics.items()}
    ptype = get_message(topics[ids[sc.TOPIC_POSE]][1]) if sc.TOPIC_POSE in ids else None
    receipts, poses = collections.defaultdict(list), []
    for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages '
                                     'ORDER BY timestamp, id'):
        receipts[topics[tid][0]].append(ts / 1e9)
        if tid == ids.get(sc.TOPIC_POSE):
            codes, p = body_pose_sample(deserialize_message(data, ptype).transforms, cfg)
            if not codes:
                poses.append((ts / 1e9, p))
    con.close()
    al = adj.align(list(c['obs']['statuses']), [(s.receipt_ns / 1e9, s.data) for s in statuses])
    alignment = s4d.alignment_status(al)
    off = al['offset_s'] if al is not None and alignment.get('ok') else None
    jts = [] if off is None else [(j.receipt_ns / 1e9 - off, dict(zip(j.names, j.positions)))
                                  for j in js]
    fb = collections.defaultdict(list)
    for x in cmds:
        if x.source == 'feedback' and off is not None:
            fb[x.goal_id].append((x.receipt_ns / 1e9 - off, max(abs(e) for e in x.error)))
    log = read_text(os.path.join(run, 'launch.log'))
    ctl = None if log is None else s4a.controller_order(log.splitlines(), len(c['gl']))
    blog = read_text(os.path.join(run, 'bag_record.log'))
    blines = [] if blog is None else [re.sub(r'\x1b\[[0-9;]*m', '', ln)
                                      for ln in blog.splitlines()]

    def pubs(name):
        txt = read_text(os.path.join(run, 'captures', name)) or ''
        m = re.search(re.escape(sc.COMMAND_TOPIC) + r'.*?Publisher count: (\d+)', txt, re.S)
        return int(m.group(1)) if m else None
    sts = c['obs']['statuses']
    support = {'recorder_log_without_failure': bool(blines) and not any(
                   re.search(r'\[(WARN|ERROR|FATAL)\]', ln) for ln in blines),
               'recording_covers_the_session': bool(sts) and off is not None and any(
                   'Recording...' in ln for ln in blines) and bool(receipts.get(sc.TOPIC_POSE))
               and min(receipts[sc.TOPIC_POSE]) - off <= sts[0][0]
               and max(receipts[sc.TOPIC_POSE]) - off >= sts[-1][0]}
    return {'alignment': alignment, 'offset_s': off,
            'poses': [] if off is None else [(t - off, p) for t, p in poses],
            'joints': jts, 'feedback': dict(fb), 'ctl': ctl, 'support': support,
            'command_topic_count': len(receipts.get(sc.COMMAND_TOPIC, [])),
            'capture_before': pubs('before_topics.txt'), 'capture_after': pubs('after_topics.txt'),
            'min_joint_margin': s4c.min_joint_margin(jts, limits), 'meta': meta,
            'sha256': rl.sha256(bag)}


def load_and_score(session, bag, disabled_library, restore_report, run_install_gates,
                   shadow=False):
    ev, obs, pre, att, scans, drv, files = load_session(session)
    run = os.path.join(session, 'run')
    b = {}
    if bag and os.path.exists(bag):
        files['bag_sha256'] = True
        c0 = context(ev, obs)
        b = load_bag(bag, c0, run)
    c = context(ev, obs, pre, att, scans, drv, files)
    out = score(c, b, shadow, read_text(restore_report) if restore_report else None,
                sc.sha(os.path.join(run, 'node', 'library.json')), sc.sha(disabled_library),
                read_json(run_install_gates) if run_install_gates else None)
    out['bag'] = {k: v for k, v in b.items() if k in ('alignment', 'offset_s', 'meta', 'sha256')}
    return out


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for k in ('--session', '--out'):
        a.add_argument(k, required=True)
    for k in ('--bag', '--disabled-library', '--restore-report', '--run-install-gates'):
        a.add_argument(k)
    a.add_argument('--shadow', action='store_true', help='a shadow rehearsal (section 5)')
    o = a.parse_args(argv)
    if os.path.exists(o.out) and os.listdir(o.out):
        print(f'refusing to write into non-empty {o.out}', file=sys.stderr)
        return 3
    os.makedirs(o.out, exist_ok=True)
    r = load_and_score(o.session, o.bag, o.disabled_library, o.restore_report,
                       o.run_install_gates, o.shadow)
    with open(os.path.join(o.out, 'summary.json'), 'w') as f:
        json.dump(r, f, indent=1, sort_keys=True, default=str)
    print(json.dumps({k: v['verdict'] for k, v in r['H'].items()}), r['overall'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
