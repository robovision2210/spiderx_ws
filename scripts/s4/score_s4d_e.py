#!/usr/bin/env python3
"""Score an S4d-E run (docs/S4D_E_CONTRACT.md section 4), offline. Frozen with the contract, before
any S4d-E run. Reuses generic helpers of the frozen S4d-M scorer (scripts/s4/score_s4d.py) and the
S4a/S3 scorers, imported unchanged.

E1 plan and budget   one goal (seq 1, level 2, 0 -> 1, the shift), one cancel (seq 1); keys exactly
                     + + a w x q; requests before q exactly arm, estop; one fault (estop); no
                     refusal before q; every status DISPATCH/gate True (live) or SHADOW/gate False
                     (shadow); the driver's result 'script complete'
E2 level             two + while DISARMED before arm, confirmed 2/3 then 3/3, each attributed; one
                     level_change to 2 in READY after arm and before goal 1; goal 1 at level 2
E3 x                 one estop-capable key, the scripted x, written while goal 1 was active (after
                     its acceptance, before its result) and within 0.95 s of sim; LIVE: at least
                     one attributed feedback sample received by the driver before the write, with
                     goal 1's id (none: UNMEASURED); SHADOW: explicitly not applicable
E4 fault response    one estop fault; FAULTED (why estop) within 0.050 s of the request, on the
                     node clock, inclusive; one cancel (seq 1) in the same handling; one teleop
                     '[estop] ok' with the node's message; no estop-service fallback; the
                     key-write-to-request latency recorded only
E5 cancellation      cancel response 0; result CANCELED (SUCCEEDED despite the cancel: UNMEASURED;
                     other: FAIL; none: UNMEASURED); LIVE: no goal-1 feedback after the result, the
                     controller log's one cancel request, the controller reference within 0.01 rad
                     over the hold; SHADOW: the modeled 'shadow cancel'
E6 hold              10 s of sim from the result, from the bag: joint motion <= 0.01 rad, tilt <=
                     0.10 rad, z >= 0.045 m (S3's assess_hold); every status in it FAULTED, no goal
E7 no further motion one goal in total, none after the fault; no request after x but the close;
                     no reset, arm or home; the final state FAULTED
E8 tracking          LIVE: goal 1 needs attributed feedback before the cancel (none: UNMEASURED);
                     max |error| <= 0.05 rad (FAIL above); > 0.02 a finding. SHADOW: N/A, explicit
E9 traffic           0 foreign clients and 0 command publishers in every status; captures 0;
                     joint_trajectory absent with support; LIVE: controller log 1 received, 1
                     accepted, 1 cancel request, one feedback goal id; SHADOW: 0, 0, 0, no feedback
                     in the bag or the driver, controller reference unchanged
E10 close            after q exactly stop x2 (accepted, the FAULTED message) and disarm x2
                     (refused, faulted_reset_required), the run's only refusals; no transition
                     after q; the teleop exits 0 within 15 s of wall
E11 margins, safety  driver complete; joint margin >= 0.02 rad; node-observed ages within the
                     monitor limits; after arm, travel <= 0.15 m from the arm pose and tilt <= 0.10
E12 restore          as S4d-M's M15 (restore report, library identity, the run installation)
Missing evidence is UNMEASURED, never PASS. Overall: FAIL if any check fails, otherwise
INCONCLUSIVE if any is UNMEASURED, otherwise PASS; a shadow result is never acceptance.

Usage: score_s4d_e.py --run <run dir> --bag <bag_0.db3> --disabled-library <file>
                      --restore-report <report> --run-install-gates <json> --harness <run script>
                      --out <new dir> [--shadow]"""
import argparse
import collections
import gzip
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 's3'), os.path.join(HERE, '..', 's2')]
import score_s4d as s4d  # noqa: E402  (the frozen S4d-M scorer; generic helpers only)

s4c, sb, s4a, sc = s4d.s4c, s4d.sb, s4d.s4a, s4d.sc
verdict_of, number = sb.verdict_of, s4d.number
SCHEMA = 'spiderx.s4d_e_score/1'
LEVEL, E_GOAL, E_GOAL_KIND = 2, (0, 1, 1), 'shift'
EXPECTED_GOALS, EXPECTED_CANCELS = 1, 1
MAIN_KEYS = ["'+'", "'+'", "'a'", "'w'", "'x'", "'q'"]
REQUESTS_BEFORE_Q = ['arm', 'estop']
CLOSE_REQUESTS = collections.Counter({'stop': 2, 'disarm': 2})
FAULTED_STOP_MESSAGE = 'not moving (FAULTED); held until a zero command'
RESET_REQUIRED = 'faulted_reset_required'
FAULT_RESPONSE_MAX_S = 0.050                 # R4, node clock, inclusive
X_DEADLINE_SIM_S = 0.95
REFERENCE_CHANGE_MAX_RAD = 0.01
TRAVEL_MAX_M, TILT_MAX_RAD = 0.15, 0.10
JOINT_MARGIN_MIN_RAD = 0.02
TELEOP_EXIT_WALL_S = 15.0
AGE_LIMITS_S = {'joint_states_age_s': sc.JS_STALE_S, 'body_pose_age_s': sc.POSE_STALE_S,
                'graph_age_s': sc.GRAPH_STALE_S}
SHADOW_NA = 'not applicable: dispatch disabled (shadow rehearsal)'
SCRIPT_COMPLETE = {False: ('script complete',),
                   True: ('script complete (shadow rehearsal: dispatch disabled; not S4d-E '
                          'acceptance)',)}


def chk(v):
    """True/False/None as given; anything else (e.g. a missing comparison) is None."""
    return v if v in (True, False, None) else None


# ------------------------------------------------------------------ extraction (pure)
def context(ev, drv):
    goals = sb.goals_from_events(ev)
    for e in ev:
        s = e.get('seq')
        if e['event'] == 'goal_sent' and s in goals:
            goals[s]['kind'] = e.get('kind')
        elif e['event'] == 'cancel_response' and s in goals:
            goals[s].update(cancel_response_code=e.get('code'), cancel_response_t=e['wall'])
        elif e['event'] == 'result' and s in goals:
            goals[s]['error_string'] = e.get('error_string')
    keys = sb.key_writes(drv)
    summary = next((d['summary'] for d in drv if isinstance(d.get('summary'), dict)), None)
    q = next((k for k in keys if k['key'] == "'q'"), None)
    xs = [k for k in keys if k['key'] in s4a.ESTOP_KEYS]
    return {
        'ev': ev, 'drv': drv, 'goals': goals, 'gl': list(goals.values()), 'keys': keys,
        'transitions': sb.transitions_from_events(ev),
        'requests': [{'request': e['request'], 'message': e.get('message'), 't': e['wall']}
                     for e in ev if e['event'] == 'request'],
        'refusals': [{'request': e['request'], 'reason': e['reason'], 't': e['wall']}
                     for e in ev if e['event'] == 'refused'],
        'cancels': [{'seq': e.get('seq'), 't': e['wall']} for e in ev
                    if e['event'] == 'cancel_sent'],
        'faults': [{'reason': e['reason'], 't': e['wall']} for e in ev if e['event'] == 'fault'],
        'level_changes': [e for e in ev if e['event'] == 'level_change'],
        'q': q, 'x': xs[0] if len(xs) == 1 else None, 'n_estop_keys': len(xs),
        'w': next((k for k in keys if k['key'] == "'w'"), None),
        't_arm': next((tr['t'] for tr in sb.transitions_from_events(ev) if tr['why'] == 'arm'),
                      None),
        'sts': sb.statuses_from_driver(drv),
        'x_record': next((d['x_record'] for d in drv if isinstance(d.get('x_record'), dict)),
                         None),
        'selection': next((d['selection'] for d in drv if isinstance(d.get('selection'), dict)),
                          None),
        'closed': next((d for d in drv if d.get('event') == 'teleop_closed'), None),
        'driver_feedback': [d['feedback'] for d in drv if isinstance(d.get('feedback'), dict)],
        'estop_service': [d for d in drv if d.get('event') == 'estop_service'],
        'teleop_estop': s4a.teleop_responses(drv, 'estop'),
        'result': None if summary is None else summary.get('result'),
    }


def status_before(sts, t):
    s = [st for mono, _, st in sts if t is not None and mono <= t]
    return s[-1] if s else None


def feedback_attribution(fbk, goal, tol):
    """Bag feedback ({goal id: [(t, max |error|)]}, node clock) attributed to goal 1: the run's
    only goal id, whose first sample lies in goal 1's window [sent - tol, result + tol]. Its
    samples inside the window are goal 1's; samples after it are kept apart (execution
    continuing after the result), never dropped."""
    out = {'ids': sorted(fbk), 'goal_id': None, 'samples': [], 'after_result': [],
           'unattributed': []}
    t0, t1 = goal.get('sent'), goal.get('result_t')
    present = {gid: s for gid, s in fbk.items() if s}
    for gid, s in present.items():
        first_in = None not in (t0, t1, tol) and t0 - tol <= s[0][0] <= t1 + tol
        if first_in and len(present) == 1:
            out.update(goal_id=gid, samples=[(t, e) for t, e in s if t <= t1 + tol],
                       after_result=[(t, e) for t, e in s if t > t1 + tol])
        else:
            out['unattributed'].append(gid)
    return out


# ------------------------------------------------------------------ E1-E4, E7, E10 (events)
def assess_plan(c, shadow):
    gl, q = c['gl'], c['q']
    g = gl[0] if len(gl) == 1 else {}
    tq = None if q is None else q['t_before_write']
    mode = ('shadow', False) if shadow else ('dispatch', True)
    modes = {(st.get('mode'), st.get('dispatch_gate')) for _, _, st in c['sts']}
    checks = {
        'exactly_one_goal_level_2_0_to_1_the_shift': len(gl) == 1 and g.get('seq') == 1
        and (g.get('b_from'), g.get('b_to'), g.get('direction')) == E_GOAL
        and g.get('level') == LEVEL and g.get('kind') == E_GOAL_KIND,
        'exactly_one_cancel_for_goal_1': len(c['cancels']) == EXPECTED_CANCELS
        and c['cancels'][0]['seq'] == 1,
        'keys_exactly_plus_plus_a_w_x_q': [k['key'] for k in c['keys']] == MAIN_KEYS,
        'requests_before_q_exactly_arm_estop': None if tq is None else
        [r['request'] for r in c['requests'] if r['t'] < tq] == REQUESTS_BEFORE_Q,
        'one_fault_the_estop': [f['reason'] for f in c['faults']] == ['estop'],
        'no_refusal_before_q': None if tq is None else not [r for r in c['refusals']
                                                            if r['t'] < tq],
        'mode_and_gate_in_every_status': None if not modes else modes == {mode},
        'no_status_with_more_than_one_goal_recorded': all(
            not isinstance(st.get('goals_recorded'), int) or st['goals_recorded'] <= 1
            for _, _, st in c['sts']),
        'driver_script_complete': None if c['result'] is None
        else c['result'] in SCRIPT_COMPLETE[shadow],
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'goals': len(gl),
            'cancels': len(c['cancels']), 'keys': [k['key'] for k in c['keys']],
            'driver_result': c['result']}


def assess_level(c):
    keys, sel = c['keys'], c['selection'] or {}
    plus = [k for k in keys if k['key'] == "'+'"]
    arm = next((k for k in keys if k['key'] == "'a'"), None)
    confs = sel.get('confirmations') or []
    g = c['goals'].get(1) or {}
    lc = c['level_changes']
    before = [status_before(c['sts'], k['t_before_write']) for k in plus]
    checks = {
        'two_plus_before_arm': len(plus) == 2 and arm is not None
        and all(k['t_after_write'] <= arm['t_before_write'] for k in plus),
        'each_plus_while_disarmed_with_no_goal': len(before) == 2 and all(
            st is not None and st.get('state') == 'DISARMED' and st.get('goal') is None
            for st in before),
        'confirmed_2_3_then_3_3_each_attributed': [x.get('label') for x in confs]
        == ['2/3', '3/3'] and len(plus) == 2 and all(
            number(x.get('t_receipt')) and x.get('t_before_write') == k['t_before_write']
            and x['t_receipt'] >= k['t_before_write'] for x, k in zip(confs, plus)),
        'one_level_change_to_2_in_ready_after_arm_before_goal_1': len(lc) == 1
        and lc[0].get('level') == LEVEL and lc[0].get('state') == 'READY'
        and None not in (c['t_arm'], g.get('sent'))
        and c['t_arm'] <= lc[0]['wall'] <= g['sent'],
        'goal_1_at_level_2': g.get('level') == LEVEL,
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'confirmations': confs}


def assess_x(c, attr, shadow):
    x, w, g = c['x'], c['w'], c['goals'].get(1) or {}
    xr = c['x_record'] or {}
    req = [r for r in c['requests'] if r['request'] == 'estop']
    t_req = req[0]['t'] if len(req) == 1 else None
    checks = {
        'one_estop_key_the_scripted_x_written': c['n_estop_keys'] == 1 and x is not None
        and x.get('write_result') == 1 and x['key'] == "'x'",
        'x_written_after_goal_1_was_sent': None if x is None or g.get('sent') is None
        else g['sent'] <= x['t_before_write'] <= x['t_after_write'],
        'estop_request_while_goal_1_active': None if t_req is None
        or g.get('response_t') is None else g.get('accepted') is True
        and g['response_t'] <= t_req and (g.get('result_t') is None or t_req <= g['result_t']),
        'x_within_0_95_s_of_sim': None if not number(xr.get('sim_elapsed_since_goal1_seen_s'))
        else xr['sim_elapsed_since_goal1_seen_s'] <= X_DEADLINE_SIM_S,
    }
    out = {'checks': checks, 'x_record': xr, 'estop_request_t': t_req}
    if shadow:
        out['feedback_precondition'] = SHADOW_NA + ': no real feedback can exist in shadow'
    else:
        fb = [f for f in c['driver_feedback'] if number(f.get('t_receipt'))]
        ids = {f.get('goal_id') for f in fb}
        before = [f for f in fb if x is not None and f['t_receipt'] < x['t_before_write']]
        attributed = (w is not None and len(ids) == 1 and all(f['t_receipt'] >= w['t_after_write']
                                                              for f in fb))
        if not before:
            v = None                                   # no feedback before x: UNMEASURED
        elif not attributed:
            v = False
        else:
            v = None if attr.get('goal_id') is None else attr['goal_id'] in ids
        checks['attributed_feedback_received_before_x'] = v
        out.update(feedback_before_x=len(before), feedback_goal_ids=sorted(i for i in ids if i))
    out['verdict'] = verdict_of(checks)
    return out


def assess_fault(c):
    x = c['x']
    est = [f for f in c['faults'] if f['reason'] == 'estop']
    tr = [t for t in c['transitions'] if t['state'] == 'FAULTED']
    req = [r for r in c['requests'] if r['request'] == 'estop']
    can = c['cancels']
    t_f = est[0]['t'] if len(est) == 1 else None
    t_tr = tr[0]['t'] if len(tr) == 1 and tr[0]['why'] == 'estop' else None
    t_req = req[0]['t'] if len(req) == 1 else None
    t_c = can[0]['t'] if len(can) == 1 and can[0]['seq'] == 1 else None
    dt = None if None in (t_tr, t_req) else t_tr - t_req
    tele = c['teleop_estop']
    checks = {
        'one_estop_fault_after_the_write_began': None if x is None else len(est) == 1
        and t_f >= x['t_before_write'],
        'one_transition_to_faulted_why_estop': len(tr) == 1 and tr[0]['why'] == 'estop',
        'faulted_within_0_050_s_of_the_request_node_clock': None if dt is None
        else 0.0 <= dt <= FAULT_RESPONSE_MAX_S,
        'one_cancel_seq_1_in_the_same_handling': False if t_c is None else (
            None if t_req is None else 0.0 <= t_c - t_req <= FAULT_RESPONSE_MAX_S),
        'one_teleop_estop_ok_after_the_fault_with_the_node_message': len(tele) == 1
        and t_f is not None and tele[0]['ok'] and tele[0]['t'] >= t_f and len(req) == 1
        and tele[0]['message'] == req[0].get('message'),
        'no_estop_service_fallback': not c['estop_service'],
    }
    latency = None if x is None or t_req is None else t_req - x['t_after_write']
    return {'verdict': verdict_of(checks), 'checks': checks,
            'request_to_faulted_s_node_clock': dt,
            'key_write_to_request_latency_s_recorded_only': latency,
            'times': {'x_before_write': None if x is None else x['t_before_write'],
                      'x_after_write': None if x is None else x['t_after_write'],
                      'estop_request': t_req, 'faulted': t_tr, 'cancel_sent': t_c}}


def assess_no_further(c):
    est = [r['t'] for r in c['requests'] if r['request'] == 'estop']
    t_x = est[0] if len(est) == 1 else None
    q = c['q']
    tq = None if q is None else q['t_before_write']
    after_x = [r for r in c['requests'] + c['refusals']
               if t_x is not None and r['t'] > t_x and (tq is None or r['t'] < tq)]
    recovery = [r for r in c['requests'] + c['refusals']
                if t_x is not None and r['t'] > t_x and r['request'] in ('reset', 'arm', 'home')]
    last = c['sts'][-1][2] if c['sts'] else None
    checks = {
        'one_goal_in_total': len(c['gl']) == EXPECTED_GOALS,
        'no_goal_after_the_fault': None if t_x is None else not [
            g for g in c['gl'] if g.get('sent') is not None and g['sent'] > t_x],
        'no_request_between_x_and_q': None if t_x is None or tq is None else not after_x,
        'no_reset_arm_or_home_after_x': None if t_x is None else not recovery,
        'final_state_faulted': None if last is None else last.get('state') == 'FAULTED'
        and last.get('goal') is None,
        'last_transition_is_the_estop': bool(c['transitions'])
        and c['transitions'][-1]['state'] == 'FAULTED',
    }
    return {'verdict': verdict_of(checks), 'checks': checks}


def assess_close(c):
    q, closed = c['q'], c['closed'] or {}
    tq = None if q is None else q['t_before_write']
    req = [r for r in c['requests'] if tq is not None and r['t'] >= tq]
    ref = [r for r in c['refusals'] if tq is not None and r['t'] >= tq]
    exit_t = closed.get('t_exit_seen')
    checks = {
        'after_q_exactly_stop_x2_and_disarm_x2': None if tq is None else
        collections.Counter([r['request'] for r in req + ref]) == CLOSE_REQUESTS,
        'stops_accepted_with_the_faulted_message': None if tq is None else
        [r['request'] for r in req] == ['stop', 'stop']
        and all(r['message'] == FAULTED_STOP_MESSAGE for r in req),
        'disarms_refused_reset_required_and_no_other_refusal': None if tq is None else
        [(r['request'], r['reason']) for r in c['refusals']]
        == [('disarm', RESET_REQUIRED)] * 2 and len(ref) == 2,
        'no_transition_after_q': None if tq is None else not [
            t for t in c['transitions'] if t['t'] >= tq],
        'teleop_exit_0_within_15_s': None if q is None or not closed else
        closed.get('exit_code') == 0 and number(exit_t)
        and exit_t - q['t_after_write'] <= TELEOP_EXIT_WALL_S,
    }
    return {'verdict': verdict_of(checks), 'checks': checks,
            'close_requests': [(r['request'], 'ok', r['message']) for r in req]
            + [(r['request'], 'refused', r['reason']) for r in ref]}


# ------------------------------------------------------------------ E5, E6, E8, E9, E11 (bag)
def reference_change(ref, t0, t1):
    """Max |change| of the controller reference over [t0, t1] (from its first sample there)."""
    w = [p for t, p in ref if t0 <= t <= t1 and p]
    if not w:
        return None
    r0 = w[0]
    return max(max((abs(a - b) for a, b in zip(p, r0)), default=0.0) for p in w)


def assess_cancel(c, b, attr, hold_end, shadow):
    g = c['goals'].get(1) or {}
    st = g.get('status')
    if st == sc.STATUS_CANCELED:
        res = True
    elif st in (None, sc.STATUS_SUCCEEDED):
        res = None                     # no result, or it completed despite the cancel: UNMEASURED
    else:
        res = False
    checks = {'cancel_response_0': None if g.get('cancel_response_code') is None
              else g['cancel_response_code'] == 0,
              'result_canceled': res}
    out = {'result_status': st, 'error_string': g.get('error_string')}
    if shadow:
        checks['modeled_shadow_cancel'] = None if g.get('error_string') is None \
            else g['error_string'] == 'shadow cancel'
    else:
        t_res = g.get('result_t')
        ctl = b.get('ctl')
        checks.update({
            'no_goal_1_feedback_after_the_result': None if t_res is None
            or attr.get('goal_id') is None else not attr.get('after_result'),
            'controller_log_one_cancel_request_one_goal': None if ctl is None
            else ctl['cancel_requests'] == 1 and ctl['received'] == 1 and ctl['accepted'] == 1,
        })
        rc = None if t_res is None or hold_end is None else reference_change(
            b.get('reference') or [], t_res, hold_end)
        checks['controller_reference_within_0_01_rad_over_the_hold'] = None if rc is None \
            else rc <= REFERENCE_CHANGE_MAX_RAD
        out['reference_change_over_hold_rad'] = rc
    out.update(verdict=verdict_of(checks), checks=checks)
    if st == sc.STATUS_SUCCEEDED:
        out['note'] = 'completed despite the cancel: the cancellation is not demonstrated'
    return out


def assess_hold(c, b):
    g = c['goals'].get(1) or {}
    t_res = g.get('result_t')
    resolved = g.get('status') in (sc.STATUS_CANCELED, sc.STATUS_SUCCEEDED)
    h = sc.assess_hold(len(c['cancels']) == 1, resolved,
                       t_res, b.get('sim_of') or (lambda t: None), b.get('poses') or [],
                       b.get('joints') or [])
    end = None
    sim_of = b.get('sim_of')
    if t_res is not None and sim_of is not None and sim_of(t_res) is not None:
        s0 = sim_of(t_res)
        end = next((t for t, _ in b.get('joints') or []
                    if t >= t_res and sim_of(t) is not None and sim_of(t) >= s0 + sc.HOLD_SIM_S),
                   None)
    win = [st for mono, _, st in c['sts'] if end is not None and t_res <= mono <= end]
    checks = {'measured_hold': {'PASS': True, 'FAIL': False}.get(h.get('verdict')),
              'faulted_with_no_goal_throughout': None if not win else all(
                  st.get('state') == 'FAULTED' and st.get('goal') is None for st in win)}
    return {'verdict': verdict_of(checks), 'checks': checks, 'hold': h, 'statuses': len(win),
            'end_t': end}


def assess_tracking(c, attr, shadow):
    if shadow:
        return {'verdict': 'N/A', 'note': SHADOW_NA}
    g = c['goals'].get(1) or {}
    can = c['cancels'][0]['t'] if len(c['cancels']) == 1 else None
    s = attr.get('samples') or []
    before = [e for t, e in s if can is not None and t <= can]
    if attr.get('unattributed') or not before:
        v, m = 'UNMEASURED', max(before, default=None)
    else:
        m = max(e for _, e in s)
        v = 'FAIL' if m > sc.TRACKING_ABORT_RAD else 'PASS'
    return {'verdict': v, 'goal': g.get('seq'), 'samples': len(s), 'before_cancel': len(before),
            'max_abs_error_rad': m, 'finding_above_0_02_rad': m is not None
            and m > sc.TRACKING_FINDING_RAD, 'unattributed_feedback_ids': attr.get('unattributed')}


def assess_traffic(c, b, shadow):
    graphs = [(st.get('monitors') or {}).get('graph') for _, _, st in c['sts']]
    graphs = [x for x in graphs if isinstance(x, dict)]
    ctl, fb = b.get('ctl'), b.get('feedback') or {}
    n_fb = sum(len(v) for v in fb.values())
    terms = {
        'foreign_clients_0_and_publishers_0_in_every_status': None if not graphs else all(
            x.get('foreign_action_clients') == 0 and x.get('command_topic_publishers') == 0
            for x in graphs),
        'captures_before_after_zero_publishers': None if None in (
            b.get('capture_before'), b.get('capture_after'))
        else b['capture_before'] == 0 and b['capture_after'] == 0,
    }
    if b.get('command_topic_present'):
        terms['joint_trajectory_absent'] = b.get('command_topic_count') == 0
    else:
        sup = b.get('support') or {}
        terms['joint_trajectory_absent'] = True if sup and all(sup.values()) else None
    if shadow:
        terms.update({
            'controller_log_0_received_0_accepted_0_cancel': None if ctl is None
            else ctl['received'] == 0 and ctl['accepted'] == 0 and ctl['cancel_requests'] == 0,
            'no_feedback_in_the_bag': n_fb == 0,
            'no_feedback_at_the_driver': not c['driver_feedback'],
            'controller_reference_unchanged': None if b.get('reference_change_total') is None
            else b['reference_change_total'] <= 1e-9})
    else:
        terms.update({
            'controller_log_1_received_1_accepted_1_cancel': None if ctl is None
            else ctl['received'] == 1 and ctl['accepted'] == 1 and ctl['cancel_requests'] == 1,
            'one_feedback_goal_id': None if not fb else len(fb) == 1})
    return {'verdict': verdict_of(terms), 'terms': terms, 'feedback_messages': n_fb,
            'controller_log': ctl}


def assess_margins(c, b, shadow):
    t_arm = c['t_arm']
    q = c['q']
    t1 = None if q is None else q['t_before_write']
    window = [st for mono, _, st in c['sts'] if None not in (t_arm, t1) and t_arm <= mono <= t1]
    ages = {k: max(((st.get('monitors') or {}).get(k) for st in window
                    if number((st.get('monitors') or {}).get(k))), default=None)
            for k in AGE_LIMITS_S}
    ps = [(t, p) for t, p in b.get('poses') or [] if t_arm is not None and t >= t_arm]
    ref = ps[0][1]['xy'] if ps else None
    travel = None if not ps else max(math.hypot(p['xy'][0] - ref[0], p['xy'][1] - ref[1])
                                     for _, p in ps)
    tilt = None if not ps else max(p['tilt'] for _, p in ps)
    mm = b.get('min_joint_margin')
    checks = {
        'driver_script_complete': None if c['result'] is None
        else c['result'] in SCRIPT_COMPLETE[shadow],
        'min_joint_margin_at_least_0_02_rad': None if mm is None
        else mm[0] >= JOINT_MARGIN_MIN_RAD,
        'node_observed_ages_within_monitor_limits': None if not window or None in ages.values()
        else all(ages[k] <= v for k, v in AGE_LIMITS_S.items()),
        'travel_from_the_arm_pose_at_most_0_15_m': None if travel is None
        else travel <= TRAVEL_MAX_M,
        'tilt_at_most_0_10_rad_after_arm': None if tilt is None else tilt <= TILT_MAX_RAD,
    }
    return {'verdict': verdict_of(checks), 'checks': checks, 'max_ages_s': ages,
            'travel_m': travel, 'max_tilt_rad': tilt, 'min_joint_margin': mm}


def overall(E):
    vs = [e['verdict'] for e in E.values() if e['verdict'] != 'N/A']
    return 'FAIL' if 'FAIL' in vs else ('PASS' if vs and all(v == 'PASS' for v in vs)
                                        else 'INCONCLUSIVE')


def score(c, b, shadow, restore_text=None, enabled_sha=None, disabled_sha=None,
          run_install=None):
    g = c['goals'].get(1) or {}
    tol = (b.get('alignment') or {}).get('tol')
    attr = feedback_attribution(b.get('feedback') or {}, g, tol)
    E = {'E1': assess_plan(c, shadow), 'E2': assess_level(c), 'E3': assess_x(c, attr, shadow),
         'E4': assess_fault(c)}
    E6 = assess_hold(c, b)
    E['E5'] = assess_cancel(c, b, attr, E6.get('end_t'), shadow)
    E['E6'] = E6
    E['E7'] = assess_no_further(c)
    E['E8'] = assess_tracking(c, attr, shadow)
    E['E9'] = assess_traffic(c, b, shadow)
    E['E10'] = assess_close(c)
    E['E11'] = assess_margins(c, b, shadow)
    E['E12'] = s4d.assess_restore_and_library(restore_text, enabled_sha, disabled_sha,
                                              run_install)
    E = dict(sorted(E.items(), key=lambda kv: int(kv[0][1:])))
    v = overall(E)
    return {'schema': SCHEMA, 'shadow': shadow, 'E': E, 'feedback_attribution': attr,
            'overall': f'{v} (SHADOW ONLY: not S4d-E acceptance)' if shadow else v}


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
    limits, _ = joint_safety.load_limits()
    js, cmds, statuses, meta = rl.read_bag(bag)
    con = sqlite3.connect(f'file:{bag}?mode=ro&immutable=1', uri=True)
    topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
    ids = {n: i for i, (n, _) in topics.items()}
    types = {i: get_message(t) for i, (n, t) in topics.items()
             if n in (sc.TOPIC_POSE, '/leg_trajectory_controller/controller_state')}
    receipts, poses, ref = collections.defaultdict(list), [], []
    for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages '
                                     'ORDER BY timestamp, id'):
        receipts[topics[tid][0]].append(ts / 1e9)
        if tid == ids.get(sc.TOPIC_POSE):
            codes, p = body_pose_sample(deserialize_message(data, types[tid]).transforms, cfg)
            if not codes:
                poses.append((ts / 1e9, p))
        elif tid == ids.get('/leg_trajectory_controller/controller_state'):
            m = deserialize_message(data, types[tid])
            ref.append((ts / 1e9, list(m.reference.positions)))
    con.close()
    ev = [json.loads(ln) for ln in open(os.path.join(run, 'node', 'events.jsonl')) if ln.strip()]
    dpath = os.path.join(run, 'driver', 'driver.jsonl')
    drv = [json.loads(ln) for ln in (gzip.open(dpath + '.gz', 'rt') if not os.path.exists(dpath)
                                     else open(dpath))]
    c = context(ev, drv)
    al = adj.align([(d['mono'], d['status']) for d in drv if 'status' in d],
                   [(s.receipt_ns / 1e9, s.data) for s in statuses])
    alignment = s4d.alignment_status(al)
    off = al['offset_s'] if al is not None and alignment.get('ok') else None
    tol = None if off is None else max(al['tolerance_s'], alignment['uncertainty_s'])
    conv = (lambda t: t - off) if off is not None else (lambda t: None)
    jts = [(conv(j.receipt_ns / 1e9), dict(zip(j.names, j.positions)), j.stamp_ns / 1e9)
           for j in js] if off is not None else []
    fbk = collections.defaultdict(list)
    for x in cmds:
        if x.source == 'feedback' and off is not None:
            fbk[x.goal_id].append((x.receipt_ns / 1e9 - off, max(abs(e) for e in x.error)))
    log_lines = []
    for name in ('launch.log', 'launch.log.gz'):
        p = os.path.join(run, name)
        if os.path.exists(p):
            log_lines = (gzip.open(p, 'rt') if name.endswith('.gz') else open(p)).read() \
                .splitlines()
            break
    ctl = s4a.controller_order(log_lines, EXPECTED_GOALS) if log_lines else None
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
    g = c['goals'].get(1) or {}
    t0, t1 = g.get('sent'), g.get('result_t')
    graphs = [(st.get('monitors') or {}).get('graph') for _, _, st in c['sts']]

    def pubs(name):
        p = os.path.join(run, 'captures', name)
        txt = open(p).read() if os.path.exists(p) else ''
        m = re.search(re.escape(sc.COMMAND_TOPIC) + r'.*?Publisher count: (\d+)', txt, re.S)
        return int(m.group(1)) if m else None
    support = {
        'requested_from_recorder': sc.COMMAND_TOPIC in requested,
        'recording_covers_goal_1': bool(started and stopped) and off is not None
        and None not in (t0, t1) and started[0] <= t0 and stopped[0] >= t1,
        'recorder_log_without_failure': bool(blog) and not any(
            re.search(r'\[(WARN|ERROR|FATAL)\]', ln) for ln in blog),
        'owner_monitor_zero_throughout': bool(graphs) and all(
            isinstance(x, dict) and x.get('command_topic_publishers') == 0
            and x.get('foreign_action_clients') == 0 for x in graphs),
    }
    refm = [(conv(t), p) for t, p in ref] if off is not None else []
    b = {'alignment': {**alignment, 'offset': off, 'tol': tol, 'adjudicate_s2': al},
         'feedback': dict(fbk), 'poses': [(conv(t), p) for t, p in poses] if off is not None
         else [], 'joints': [(t, q) for t, q, _ in jts],
         'sim_of': s4d.make_sim_of([(t, s) for t, _, s in jts]), 'reference': refm,
         'reference_change_total': None if not ref else reference_change(
             ref, ref[0][0], ref[-1][0]),
         'ctl': ctl, 'command_topic_present': sc.COMMAND_TOPIC in [n for n, _ in topics.values()],
         'command_topic_count': len(receipts.get(sc.COMMAND_TOPIC, [])), 'support': support,
         'capture_before': pubs('before_topics.txt'), 'capture_after': pubs('after_topics.txt'),
         'min_joint_margin': s4c.min_joint_margin([(t, q) for t, q, _ in jts], limits)}
    lib_path = os.path.join(run, 'node', 'library.json')
    run_install = json.load(open(run_install_gates)) if run_install_gates and \
        os.path.exists(run_install_gates) else None
    res = score(c, b, shadow,
                open(restore_report).read() if restore_report and os.path.exists(restore_report)
                else None, sc.sha(lib_path), sc.sha(disabled_library), run_install)
    res['bag'] = {**meta, 'sha256_after_scan': rl.sha256(bag)}
    res['alignment'] = {k: v for k, v in b['alignment'].items() if k != 'adjudicate_s2'}
    return res


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for k in ('--run', '--bag', '--out'):
        a.add_argument(k, required=True)
    a.add_argument('--disabled-library')
    a.add_argument('--restore-report')
    a.add_argument('--run-install-gates')
    a.add_argument('--harness')
    a.add_argument('--shadow', action='store_true',
                   help='a shadow rehearsal: tracking and the feedback precondition are N/A')
    o = a.parse_args(argv)
    if os.path.exists(o.out) and os.listdir(o.out):
        print(f'refusing to write into non-empty {o.out}', file=sys.stderr)
        return 3
    os.makedirs(o.out, exist_ok=True)
    res = load_and_score(o.run, o.bag, o.disabled_library, o.restore_report,
                         o.run_install_gates, o.harness, o.shadow)
    with open(os.path.join(o.out, 'summary.json'), 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
    print(json.dumps({k: v['verdict'] for k, v in res['E'].items()}), res['overall'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
