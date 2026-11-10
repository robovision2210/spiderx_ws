#!/usr/bin/env python3
"""The S4d-M scorer, specified before any S4d run.
- M1, M2, M7-M10 and M16 run against real in-memory bench runs: the node's own events and the S4d
  driver's log, live (the gate True in memory) and shadow (the node's ShadowTransport, the gate
  False), in each close order. Copies of those runs are then mutated, to show that every
  contradiction FAILs and every gap is UNMEASURED.
- The bag-based parts use synthetic boundary poses and synthetic pose streams: band edges, legs,
  cumulative drift, boundary extraction at rest, and missing or uncertain alignment.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_score_s4d"""
import copy
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import score_s4d as s4d  # noqa: E402
import test_s4d_driver_live as s4dt  # noqa: E402

Z0 = s4dt.Z0
_RUNS = {}
UM = 1e-6
PASS = {'verdict': 'PASS'}
EVENT_PARTS = ('M1', 'M2', 'M7', 'M8', 'M16')
MOTION_PARTS = ('M3', 'M4', 'M5', 'M6', 'M9', 'M10')


def bench_run(shadow=False, rtf=0.6, **opts):
    """One S4d bench run per (shadow, RTF, bench options), cached: (rc, node events, driver
    records)."""
    key = (shadow, rtf, tuple(sorted((k, repr(v)) for k, v in opts.items())))
    if key not in _RUNS:
        saved = s4dt.c55.M55_LOCOMOTION_DISPATCH_ENABLED
        s4dt.c55.M55_LOCOMOTION_DISPATCH_ENABLED = not shadow       # tests only, in memory
        try:
            with tempfile.TemporaryDirectory() as tmp:
                mod = s4dt.load_driver()
                bench = s4dt.make_bench(rtf, shadow=shadow, **opts)
                d = s4dt.make_driver(mod, bench, tmp, shadow=shadow)
                rc, why = mod.run(d)
                d.log(summary={'result': why, **d.summary()})          # as main() does
                d.out.close()
                with open(os.path.join(tmp, 'driver.jsonl')) as f:
                    drv = [json.loads(ln) for ln in f]
                _RUNS[key] = (rc, list(bench.s.log), drv)
        finally:
            s4dt.c55.M55_LOCOMOTION_DISPATCH_ENABLED = saved
    rc, ev, drv = _RUNS[key]
    return rc, copy.deepcopy(ev), copy.deepcopy(drv)


def line(y=None, x=None, yaw=None, z=None):
    """Synthetic boundary poses on a line along +y (yaw 0), B9 at the origin: SC 40 mm, F and R
    60 mm per cycle, X 60 mm. y, x, yaw, z: {goal: value} overrides."""
    ys = {0: -0.040, 9: 0.0, **{9 + 9 * j: 0.060 * j for j in range(1, 6)},
          **{54 + 9 * j: 0.300 - 0.060 * j for j in range(1, 6)}, 110: 0.060}
    ys.update(y or {})
    return {k: {'xy': ((x or {}).get(k, 0.0), v), 'yaw': (yaw or {}).get(k, 0.0),
                'z': (z or {}).get(k, Z0), 'tilt': 0.01} for k, v in ys.items()}


def motion(poses=None, tilt=0.01, still=PASS, drop=()):
    poses = line() if poses is None else poses
    return {'alignment': {'ok': True}, 'z_stand': Z0,
            'boundaries': {k: {'pose': None if k in drop else p} for k, p in poses.items()},
            'tilt_max': dict.fromkeys(('SC', 'F', 'R', 'X', 'run'), tilt),
            'all_sample_maxima': {'lateral_m': 0.012, 'heading_rad': 0.01}, 'still': still}


def score(ev, drv, m=None, shadow=False, **kw):
    return s4d.score_events(ev, drv, motion() if m is None else m, shadow, **kw)[0]


def ev_of(ev, event, **match):
    return [e for e in ev if e['event'] == event and all(e.get(k) == v for k, v in match.items())]


def key_rec(drv, key, i=0):
    return [d for d in drv if d.get('key') == key and 't_before_write' in d][i]


def verdicts(M, parts):
    return {k: M[k]['verdict'] for k in parts}


class BenchRuns(unittest.TestCase):
    def test_live_and_shadow_bench_runs_pass_every_event_and_motion_check(self):
        for shadow in (False, True):
            with self.subTest(shadow=shadow):
                rc, ev, drv = bench_run(shadow)
                self.assertEqual(rc, 0)
                M = score(ev, drv, shadow=shadow)
                self.assertEqual(verdicts(M, EVENT_PARTS + MOTION_PARTS),
                                 dict.fromkeys(EVENT_PARTS + MOTION_PARTS, 'PASS'), M)
                self.assertEqual(M['M10']['requests_after_q'], {'stop': 2, 'disarm': 2})
                self.assertEqual([c['expected_label'] for c in M['M2']['confirmations']],
                                 ['2/3', '3/3'])
                self.assertEqual(M['M4']['leg']['verdict'], 'N/A' if shadow else 'PASS')

    def test_each_close_order_and_both_disarm_reasons_pass_with_the_node_clock_evidence(self):
        for opts, stopping, reason, basis in (
                ({}, 'stop', 'lease_expired', 'no command after the last status'),
                ({'close_order': s4dt.CLOSE_DISARM_FIRST}, 'disarm', 'lease_expired', None),
                ({'close_order': s4dt.CLOSE_ZERO_FIRST}, 'intent_stop', 'lease_expired', None)):
            with self.subTest(opts=opts):
                rc, ev, drv = bench_run(**opts)
                M = score(ev, drv)
                self.assertEqual(M['M10']['verdict'], 'PASS', M['M10'])
                tr = M['M10']['transitions_after_q']
                self.assertEqual((tr[0][2], tr[1][2]), (stopping, reason))
                self.assertEqual(M['M10']['reason_evidence']['expected'], reason)
                if basis:
                    self.assertEqual(M['M10']['reason_evidence']['basis'], basis)
        # the lease kept fresh to the phase end: the node disarms with 'disarm', decided by the
        # lease age at the last in-flight status (RTF 0.3) or by a command after it (RTF 0.4). At
        # RTF 0.6 the last such status is 0.5 s before the transition and the lease 0.6 s old:
        # undecidable, so UNMEASURED; never a FAIL
        for rtf, basis in ((0.3, 'the lease was fresh at the last status'),
                           (0.4, 'a command after the last status, within the lease'),
                           (0.6, None)):
            with self.subTest(rtf=rtf):
                rc, ev, drv = bench_run(rtf=rtf, zero_after_q_s=6.0)
                M = score(ev, drv)
                self.assertEqual(M['M10']['transitions_after_q'][-1][2], 'disarm')
                r = M['M10']['reason_evidence']
                self.assertEqual(r.get('basis'), basis, r)
                self.assertEqual(r['expected'], 'disarm' if basis else None)
                self.assertEqual(M['M10']['verdict'], 'PASS' if basis else 'UNMEASURED')

    def test_shadow_and_live_runs_are_never_scored_as_each_other(self):
        rc, ev, drv = bench_run(shadow=True)
        M = score(ev, drv, shadow=False)
        self.assertIs(M['M1']['checks']['node_dispatch_gate_true_throughout'], False)
        rc, ev, drv = bench_run()
        M = score(ev, drv, shadow=True)
        self.assertIs(M['M1']['checks']['node_shadow_gate_false_throughout'], False)
        self.assertEqual(M['M1']['verdict'], 'FAIL')

    def test_without_the_bag_every_motion_part_is_unmeasured_never_pass(self):
        rc, ev, drv = bench_run()
        M = s4d.score_events(ev, drv, None)[0]
        self.assertEqual(verdicts(M, EVENT_PARTS), dict.fromkeys(EVENT_PARTS, 'PASS'))
        self.assertEqual(verdicts(M, MOTION_PARTS), dict.fromkeys(MOTION_PARTS, 'UNMEASURED'))
        self.assertEqual(s4d.overall(M), 'INCONCLUSIVE')

    def test_the_plan_tables_match_the_driver(self):
        m = s4dt.load_driver()
        self.assertEqual(s4d.PLANNED, m.PLANNED)
        self.assertEqual((s4d.MAIN_KEYS, s4d.PROBE_KEYS), (m.MAIN_KEYS, m.PROBE_KEYS))
        self.assertEqual(s4d.REQUESTS_BEFORE_Q, list(m.REQUESTS_BEFORE_Q))


class Contradictions(unittest.TestCase):
    def mutate(self, f, shadow=False, m=None, **opts):
        rc, ev, drv = bench_run(shadow, **opts)
        f(ev, drv)
        return score(ev, drv, m, shadow)

    def assertFails(self, M, part, check):
        self.assertEqual(M[part]['verdict'], 'FAIL', M[part])
        self.assertIs(M[part]['checks'][check], False)

    def assertUnmeasured(self, M, part, check):
        self.assertEqual(M[part]['verdict'], 'UNMEASURED', M[part])
        self.assertIsNone(M[part]['checks'][check])

    # ------------------------------------------------------------------ M1
    def test_m1_plan_contradictions_fail(self):
        def wrong_direction(ev, drv):
            ev_of(ev, 'goal_sent', seq=55)[0].update(b_from=0, b_to=1, direction=1)

        def extra(ev, drv):
            last = ev_of(ev, 'goal_sent', seq=111)[0]
            i = ev.index(ev_of(ev, 'result', seq=111)[0])
            ev[i + 1:i + 1] = [{**last, 'seq': 112, 'n': last['n']},
                               {'event': 'result', 'seq': 112, 'status': 4, 'wall': last['wall']}]

        def duplicate(ev, drv):
            g = ev_of(ev, 'goal_sent', seq=30)[0]
            ev.insert(ev.index(g) + 1, dict(g))

        def cancel(ev, drv):
            ev.append({'event': 'cancel_sent', 'seq': 111, 'wall': ev[-1]['wall'], 'n': 10 ** 6})

        def fault(ev, drv):
            ev.append({'event': 'fault', 'reason': 'tracking', 'wall': ev[-1]['wall'],
                       'n': 10 ** 6})

        def refusal(ev, drv):
            ev.append({'event': 'refused', 'request': 'arm', 'reason': 'x',
                       'wall': ev[-1]['wall'], 'n': 10 ** 6})

        def extra_key(ev, drv):
            k = key_rec(drv, "'h'")
            drv.insert(drv.index(k), {**k, 'key': "'w'"})

        def home_before_stop(ev, drv):
            r = ev_of(ev, 'request', request='home')[0]
            s = ev_of(ev, 'request', request='stop')[1]
            r['request'], s['request'] = 'stop', 'home'

        def level_3(ev, drv):
            ev_of(ev, 'goal_sent', seq=20)[0]['level'] = 3
        for f, check in ((wrong_direction, 'exactly_111_planned_goals_in_order'),
                         (extra, 'exactly_111_planned_goals_in_order'),
                         (duplicate, 'one_goal_sent_per_seq'),
                         (cancel, 'no_cancel'), (fault, 'no_fault'), (refusal, 'no_refusal'),
                         (extra_key, 'main_keys_exactly_as_scripted'),
                         (home_before_stop, 'requests_before_q_exactly_arm_stop_stop_home'),
                         (level_3, 'exactly_111_planned_goals_in_order')):
            with self.subTest(case=f.__name__):
                self.assertFails(self.mutate(f), 'M1', check)

    # ------------------------------------------------------------------ M2
    def test_m2_speed_change_contradictions_fail_and_gaps_are_unmeasured(self):
        def plus_after_goal_2(ev, drv):
            g3 = ev_of(ev, 'goal_sent', seq=3)[0]['wall']
            k = key_rec(drv, "'+'", 1)
            k['t_before_write'], k['t_after_write'] = g3 + 0.1, g3 + 0.1001

        def third_change(ev, drv):
            lc = ev_of(ev, 'level_change')[1]
            ev.insert(ev.index(lc) + 1, dict(lc))

        def early_change(ev, drv):
            ev_of(ev, 'level_change')[1]['wall'] = ev_of(ev, 'goal_sent', seq=5)[0]['wall']

        def change_while_ready(ev, drv):
            ev_of(ev, 'level_change')[1]['state'] = 'READY'

        def goal_3_level_2(ev, drv):
            ev_of(ev, 'goal_sent', seq=3)[0]['level'] = 2

        def no_confirmation(ev, drv):
            tp = [d['mono'] for d in drv if d.get('event') == 'teleop_started'][1]
            for d in drv:
                if isinstance(d.get('teleop_out'), str) and d['t_receipt'] < tp:
                    d['teleop_out'] = d['teleop_out'].replace('speed level 3/3', 'speed lvl')
        for f, check in ((plus_after_goal_2, 'second_plus_while_goal_2_in_flight'),
                         (third_change, 'exactly_two_level_changes'),
                         (early_change, 'level_change_1_to_2_walking_between_goal_9_and_goal_10'),
                         (change_while_ready,
                          'level_change_1_to_2_walking_between_goal_9_and_goal_10'),
                         (goal_3_level_2, 'goals_1_to_9_at_level_1_and_10_to_111_at_level_2'),
                         (no_confirmation, 'each_plus_confirmed_by_the_teleop')):
            with self.subTest(case=f.__name__):
                self.assertFails(self.mutate(f), 'M2', check)

        def no_receipt_time(ev, drv):
            next(d for d in drv if 'teleop_out' in d).pop('t_receipt')
        M = self.mutate(no_receipt_time)
        self.assertUnmeasured(M, 'M2', 'each_plus_confirmed_by_the_teleop')
        self.assertIsNone(M['M16']['checks']['level_confirmations_2_3_and_3_3_attributed'])

    # ------------------------------------------------------------------ M4, M5
    def test_m4_m5_leg_event_contradictions_fail(self):
        def stop_mid_f(ev, drv):
            g = ev_of(ev, 'goal_sent', seq=30)[0]
            ev.insert(ev.index(g), {'event': 'transition', 'previous': 'WALKING',
                                    'state': 'STOPPING', 'why': 'intent_stop', 'wall': g['wall'],
                                    'n': g['n']})

        def space_too_late(ev, drv):
            k = key_rec(drv, "' '", 0)
            t = ev_of(ev, 'result', seq=54)[0]['wall'] + 0.01
            k['t_before_write'], k['t_after_write'] = t, t + 0.0001

        def wrong_ready(ev, drv):
            g = ev_of(ev, 'result', seq=99)[0]
            next(e for e in ev if e['event'] == 'transition' and e['wall'] >= g['wall']
                 and e['state'] == 'READY')['why'] = 'intent_stop'

        def wrong_count(ev, drv):
            t = ev_of(ev, 'result', seq=54)[0]['wall']
            for d in drv:
                if 'status' in d and d['mono'] >= t and d['status'].get('state') == 'READY':
                    d['status']['cycles'] = 5
        for f, part, check in ((stop_mid_f, 'M4', 'walking_continuously'),
                               (space_too_late, 'M4',
                                'space_written_while_the_last_goal_was_in_flight'),
                               (wrong_ready, 'M5', 'ready_stopped_at_boundary_0'),
                               (wrong_count, 'M4', 'cycle_count_6_at_ready')):
            with self.subTest(case=f.__name__):
                self.assertFails(self.mutate(f), part, check)

    # ------------------------------------------------------------------ M7, M8, M9
    def test_m7_m8_m9_release_reversal_and_home_contradictions_fail(self):
        def early_stopping(ev, drv):
            rel = next(d for d in drv if d.get('step') == 'X_release' and 'release' in d)
            g = ev_of(ev, 'goal_sent', seq=104)[0]
            st = next(e for e in ev if e['event'] == 'transition' and e['state'] == 'STOPPING'
                      and e['wall'] > g['wall'])
            st['wall'] = rel['t_after_stop'] + 0.5

        def switch_after_a_lapse(ev, drv):
            k = key_rec(drv, "'s'", 1)
            sw = next(d for d in drv if 'switch' in d)
            sw['t_after_stop'] = k['t_before_write'] - 1.0

        def five_homing_phases(ev, drv):
            ev_of(ev, 'homing')[0]['phases'] = 5

        def not_homed(ev, drv):
            g = ev_of(ev, 'result', seq=110)[0]
            next(e for e in ev if e['event'] == 'transition' and e['wall'] >= g['wall']
                 and e['state'] == 'READY')['why'] = 'stopped'
        M = self.mutate(early_stopping)
        self.assertFails(M, 'M7', 'row_9_release_during_goal_104')
        self.assertIs(M['M7']['row_9']['checks']['stopping_after_the_hold_window'], False)
        M = self.mutate(switch_after_a_lapse)
        self.assertFails(M, 'M8', 'switch_without_release')
        for f in (five_homing_phases, not_homed):
            with self.subTest(case=f.__name__):
                self.assertFails(self.mutate(f), 'M9', 'home_rules')

    # ------------------------------------------------------------------ M10
    def test_m10_close_contradictions_fail(self):
        def wrong_reason(ev, drv):
            ev_of(ev, 'transition', state='DISARMED')[-1]['why'] = 'disarm'

        def ready_between(ev, drv):
            d = ev_of(ev, 'transition', state='DISARMED')[-1]
            ev.insert(ev.index(d), {**d, 'state': 'READY', 'why': 'stopped',
                                    'previous': 'STOPPING'})
            d['previous'] = 'READY'

        def extra_request(ev, drv):
            d = ev_of(ev, 'transition', state='DISARMED')[-1]
            ev.insert(ev.index(d), {'event': 'request', 'request': 'home', 'message': 'x',
                                    'wall': d['wall'], 'n': d['n']})

        def q_after_the_goal(ev, drv):
            k = key_rec(drv, "'q'")
            t = ev_of(ev, 'result', seq=111)[0]['wall'] + 0.01
            k['t_before_write'], k['t_after_write'] = t, t + 0.0001

        def exit_1(ev, drv):
            next(d for d in drv if d.get('event') == 'teleop_closed'
                 and d.get('step') == 'Q')['exit_code'] = 1

        def key_after_q(ev, drv):
            k = key_rec(drv, "'q'")
            drv.insert(drv.index(k) + 1, {**k, 'key': "'w'", 't_before_write':
                                          k['t_before_write'] + 1.0})

        def counter_falls(ev, drv):
            t = ev_of(ev, 'transition', state='DISARMED')[-1]['wall']
            d = next(d for d in drv if 'status' in d and d['mono'] >= t
                     and d['status']['state'] == 'DISARMED')
            d['status']['commands_accepted'] -= 1

        def lapse_before_q(ev, drv):
            rs = next(d for d in drv if 'repeat_stop_before_q' in d)['repeat_stop_before_q']
            rs['t_after_stop'] = key_rec(drv, "'q'")['t_before_write'] - 1.0
        for f, check in ((wrong_reason, 'disarmed_reason_matches_the_node_clock_evidence'),
                         (ready_between, 'stopping_then_disarmed_at_boundary_1'),
                         (extra_request, 'exactly_the_close_requests_after_q'),
                         (q_after_the_goal, 'q_written_while_goal_111_in_flight'),
                         (exit_1, 'teleop_exit_code_0_within_15_s'),
                         (key_after_q, 'no_key_after_q_before_the_probe'),
                         (counter_falls, 'disarmed_reason_matches_the_node_clock_evidence'),
                         (lapse_before_q, 'repeat_stopped_then_q_within_the_hold_window')):
            with self.subTest(case=f.__name__):
                self.assertFails(self.mutate(f), 'M10', check)
        M = self.mutate(lambda ev, drv: None, m=motion(still={'verdict': 'FAIL'}))
        self.assertFails(M, 'M10', 'still_after_the_close')

    def test_m10_insufficient_timing_evidence_is_unmeasured(self):
        def no_lease_age(ev, drv):
            for d in drv:
                if (d.get('status', {}).get('goal') or {}).get('seq') == 111:
                    d['status']['lease_age_s'] = None

        def commands_after_the_transition(ev, drv):
            t = ev_of(ev, 'transition', state='DISARMED')[-1]['wall']
            for d in drv:
                if 'status' in d and d['mono'] >= t and d['status']['state'] == 'DISARMED':
                    d['status']['lease_age_s'] = 0.05           # renewed after the clear
                    d['status']['commands_accepted'] += 3
        for f in (no_lease_age, commands_after_the_transition):
            with self.subTest(case=f.__name__):
                M = self.mutate(f)
                self.assertUnmeasured(M, 'M10', 'disarmed_reason_matches_the_node_clock_evidence')

    # ------------------------------------------------------------------ M16
    def test_m16_probe_contradictions_fail(self):
        def probe_t(drv):
            return [d['mono'] for d in drv if d.get('event') == 'teleop_started'][1]

        def arm(ev, drv):
            t = probe_t(drv) + 2.0
            ev.append({'event': 'request', 'request': 'arm', 'message': 'armed', 'wall': t,
                       'n': 10 ** 6})

        def goal(ev, drv):
            t = probe_t(drv) + 2.0
            ev.append({'event': 'goal_sent', 'seq': 112, 'level': 2, 'b_from': 1, 'b_to': 2,
                       'direction': 1, 'wall': t, 'n': 10 ** 6})

        def wrong_message(ev, drv):
            r = [e for e in ev if e['event'] == 'request' and e['wall'] >= probe_t(drv)][0]
            r['message'] = 'stopping after the current phase'

        def no_delivery(ev, drv):
            for d in drv:
                if 'status' in d and d['mono'] >= probe_t(drv):
                    d['status']['intent']['direction'] = 0

        def short_hold(ev, drv):
            w = [k for k in drv if k.get('key') == "'w'" and 't_before_write' in k][-1]
            rel = next(d for d in drv if d.get('step') == 'probe_release' and 'release' in d)
            rel['t_before_stop'] = w['t_after_write'] + 4.9

        def exit_1(ev, drv):
            next(d for d in drv if d.get('event') == 'teleop_closed'
                 and d.get('step') == 'probe')['exit_code'] = 1

        def rejection(ev, drv):
            ev.append({'event': 'command_rejected', 'code': 'command_rate_exceeded',
                       'wall': probe_t(drv) + 3.0, 'n': 10 ** 6})

        def before_the_observation(ev, drv):
            next(d for d in drv if 'close_observation' in d)['mono'] = probe_t(drv) + 1.0

        def not_disarmed(ev, drv):
            d = next(d for d in drv if 'status' in d and d['mono'] >= probe_t(drv) + 3.0)
            d['status']['state'] = 'READY'
        for f, check in ((arm, 'no_arm_goal_transition_or_refusal'),
                         (goal, 'no_arm_goal_transition_or_refusal'),
                         (wrong_message, 'close_requests_accepted_as_no_ops'),
                         (no_delivery, 'commands_delivered_forward_at_level_2'),
                         (short_hold, 'held_w_5_to_6_s'),
                         (exit_1, 'teleop_exit_code_0_within_15_s'),
                         (rejection, 'no_command_rejection'),
                         (before_the_observation, 'probe_after_the_close_observation'),
                         (not_disarmed, 'disarmed_without_goal_throughout')):
            with self.subTest(case=f.__name__):
                self.assertFails(self.mutate(f), 'M16', check)

        def no_probe(ev, drv):
            drv.remove([d for d in drv if d.get('event') == 'teleop_started'][1])
        M = self.mutate(no_probe)
        self.assertEqual(M['M16']['verdict'], 'UNMEASURED')
        self.assertIs(M['M1']['checks']['main_keys_exactly_as_scripted'], False)


class CloseReason(unittest.TestCase):
    """close_reason_evidence on hand-built evidence (node clock throughout)."""

    @staticmethod
    def ctx(statuses, t_dis=105.0, why='lease_expired'):
        return {'goals': {111: {'sent': 100.0}}, 'q': {'t_before_write': 101.0},
                'transitions': [{'t': 101.1, 'state': 'STOPPING', 'previous': 'WALKING',
                                 'why': 'stop'},
                                {'t': t_dis, 'state': 'DISARMED', 'previous': 'STOPPING',
                                 'why': why}],
                'sts': statuses}

    @staticmethod
    def st(mono, elapsed=None, lease_age=None, n=10, state='STOPPING'):
        goal = None if elapsed is None else {'seq': 111, 'elapsed_s': elapsed}
        return (mono, None, {'state': state, 'goal': goal, 'lease_age_s': lease_age,
                             'commands_accepted': n})

    def test_each_rule(self):
        cases = (  # (statuses, expected, basis fragment)
            ([self.st(104.7, 4.7, 0.1), self.st(105.2, n=10, state='DISARMED')], 'disarm',
             'fresh'),                                                # 105 - 104.6 = 0.4
            ([self.st(104.5, 4.5, 0.4), self.st(105.2, n=10, state='DISARMED')],
             'lease_expired', 'no command'),                          # 105 - 104.1 = 0.9
            ([self.st(104.6, 4.6, 0.6), self.st(105.2, n=12, state='DISARMED')], 'disarm',
             'a command after'),                                      # renewed after 104.6
            ([self.st(104.4, 4.4, 0.6), self.st(105.2, n=12, state='DISARMED')], None, None),
            ([self.st(104.5, 4.5, 1e-9), self.st(105.2, n=10, state='DISARMED')], None,
             None),                                                   # within 1 us of lease_s
            ([self.st(104.5, 4.5, 0.4), self.st(105.2, 0.0, 0.05, n=10, state='DISARMED')],
             None, None),                                 # renewed after the clear: count unknown
            ([self.st(105.2, n=10, state='DISARMED')], None, None))   # no in-flight status
        for statuses, expected, basis in cases:
            with self.subTest(statuses=statuses):
                r = s4d.close_reason_evidence(self.ctx(statuses))
                self.assertEqual(r['expected'], expected, r)
                self.assertFalse(r['contradiction'])
                if basis:
                    self.assertIn(basis, r['basis'])

    def test_a_falling_counter_is_a_contradiction(self):
        r = s4d.close_reason_evidence(self.ctx([self.st(104.5, 4.5, 0.4),
                                                self.st(105.2, n=9, state='DISARMED')]))
        self.assertTrue(r['contradiction'])
        self.assertIsNone(r['expected'])

    def test_key_write_times_are_never_used(self):
        a = s4d.close_reason_evidence(self.ctx([self.st(104.5, 4.5, 0.4),
                                                self.st(105.2, state='DISARMED')]))
        c = self.ctx([self.st(104.5, 4.5, 0.4), self.st(105.2, state='DISARMED')])
        c['q'] = {'t_before_write': 104.9}                          # a key write near the end
        self.assertEqual(s4d.close_reason_evidence(c)['expected'], a['expected'])


class Displacement(unittest.TestCase):
    """The bag-based parts on synthetic boundary poses (yaw 0). Inclusive edges are tested on the
    measurement functions with exactly representable operands (start pose at the origin)."""

    def score_motion(self, poses, shadow=False, **kw):
        rc, ev, drv = bench_run(shadow)
        return score(ev, drv, motion(poses, **kw), shadow)

    @staticmethod
    def pair(k0, k1, x=0.0, y=0.0, yaw=0.0, z=0.0):
        """Two boundary poses: B<k0> at the origin, B<k1> at (x, y, yaw, z); z_stand 0."""
        return {'z_stand': 0.0, 'boundaries': {
            k0: {'pose': {'xy': (0.0, 0.0), 'yaw': 0.0, 'z': 0.0}},
            k1: {'pose': {'xy': (x, y), 'yaw': yaw, 'z': z}}}}

    def test_leg_bands_are_inclusive_od1(self):
        for name, k0, k1, y, ok in (('F', 9, 54, 0.285, True), ('F', 9, 54, 0.315, True),
                                    ('F', 9, 54, 0.285 - UM, False),
                                    ('F', 9, 54, 0.315 + UM, False),
                                    ('R', 54, 99, -0.315, True), ('R', 54, 99, -0.285, True),
                                    ('R', 54, 99, -0.315 - UM, False),
                                    ('R', 54, 99, -0.285 + UM, False)):
            with self.subTest(leg=name, y=y):
                r = s4d.measure_leg(name, self.pair(k0, k1, y=y), False)
                self.assertIs(r['checks']['leg_in_band'], ok, r)
                self.assertEqual(r['claim'], s4d.OD1_CLAIM)
        # on a whole run: a 0.27 m forward leg FAILs M4, a missing B54 leaves it UNMEASURED
        p = line(y={k: 0.054 * (k // 9 - 1) for k in range(18, 55, 9)})
        M = self.score_motion(p)
        self.assertIs(M['M4']['checks']['leg_distance'], False)
        M = self.score_motion(line(), drop=(54,))
        self.assertIsNone(M['M4']['checks']['leg_distance'])

    def test_cycle_bands_height_and_drift_are_inclusive(self):
        cases = (('SC', 0, 9, {'y': 0.050}, True), ('SC', 0, 9, {'y': 0.030}, True),
                 ('SC', 0, 9, {'y': 0.050 + UM}, False), ('SC', 0, 9, {'y': 0.030 - UM}, False),
                 ('F1', 9, 18, {'y': 0.070}, True), ('F1', 9, 18, {'y': 0.050 - UM}, False),
                 ('R1', 54, 63, {'y': -0.070}, True), ('R1', 54, 63, {'y': -0.050}, True),
                 ('R1', 54, 63, {'y': -0.050 + UM}, False),
                 ('X', 99, 110, {'y': 0.070}, True), ('X', 99, 110, {'y': 0.070 + UM}, False))
        for name, k0, k1, kw, ok in cases:
            with self.subTest(cycle=name, **kw):
                r = s4d.measure_cycle(name, self.pair(k0, k1, **kw), False)
                self.assertIs(r['checks']['progress_in_band'], ok, r)
        for kw, check, ok in (({'x': 0.010}, 'lateral_within_10_mm', True),
                              ({'x': 0.010 + UM}, 'lateral_within_10_mm', False),
                              ({'z': 0.010}, 'height_within_10_mm', True),
                              ({'z': -0.010 - UM}, 'height_within_10_mm', False),
                              ({'yaw': 0.05 - 1e-9}, 'yaw_within_0_05_rad', True),
                              ({'yaw': 0.05 + UM}, 'yaw_within_0_05_rad', False)):
            with self.subTest(check=check, **kw):
                r = s4d.measure_cycle('F1', self.pair(9, 18, y=0.060, **kw), False)
                self.assertIs(r['checks'][check], ok, r)
        # on a whole run: a 75 mm F2 FAILs M4; an 81 mm F1 in shadow exceeds the outer bound
        M = self.score_motion(line(y={27: 0.135, 36: 0.195, 45: 0.255, 54: 0.315,
                                      **{54 + 9 * j: 0.315 - 0.060 * j for j in range(1, 6)},
                                      110: 0.075}))
        self.assertIs(M['M4']['cycles']['F2']['checks']['progress_in_band'], False)
        self.assertEqual(M['M4']['verdict'], 'FAIL')

    def test_cumulative_drift_od2_at_every_boundary_0(self):
        lat = 'cumulative_lateral_within_30_mm_at_every_boundary_0'
        hdg = 'cumulative_heading_within_0_10_rad_at_every_boundary_0'

        def drift(x=0.0, yaw=0.0, at=54):
            b = {9: {'pose': {'xy': (0.0, 0.0), 'yaw': 0.0}}}
            for k in s4d.CUMULATIVE_GOALS[1:]:
                b[k] = {'pose': {'xy': (x if k == at else 0.0, 0.0),
                                 'yaw': yaw if k == at else 0.0}}
            return s4d.assess_drift({'boundaries': b, 'tilt_max': {'run': 0.01}})
        for kw, check, ok in (({'x': 0.030}, lat, True), ({'x': -0.030 - UM}, lat, False),
                              ({'x': 0.030, 'at': 99}, lat, True),
                              ({'x': 0.030 + UM, 'at': 99}, lat, False),
                              ({'yaw': 0.10 - 1e-9}, hdg, True), ({'yaw': -0.10 - UM}, hdg, False)):
            with self.subTest(**kw):
                self.assertIs(drift(**kw)['checks'][check], ok)
        # on a whole run: 7 mm per F cycle stays inside the per-cycle band but reaches 35 mm
        M = self.score_motion(line(x={**{9 + 9 * j: 0.007 * j for j in range(1, 6)},
                                      **{k: 0.035 for k in range(63, 111, 9)}, 110: 0.035}))
        self.assertTrue(all(c['checks']['lateral_within_10_mm']
                            for c in M['M4']['cycles'].values()))
        self.assertIs(M['M6']['checks'][lat], False)
        M = self.score_motion(line(), drop=(63,))
        self.assertIsNone(M['M6']['checks'][lat])
        self.assertEqual(M['M6']['verdict'], 'UNMEASURED')
        M = self.score_motion(line(x={k: 0.031 for k in range(63, 111, 9)}), drop=(54,))
        self.assertIs(M['M6']['checks'][lat], False)                # a contradiction wins
        self.assertEqual(M['M6']['net_return_recorded_only']['from_b9']['forward_m'], 0.0)

    def test_shadow_drops_the_signed_and_leg_bands_only(self):
        p = line(y={k: 0.0 for k in s4d.BOUNDARY_GOALS})
        M = self.score_motion(p, shadow=True)
        self.assertEqual(M['M4']['leg']['verdict'], 'N/A')
        self.assertNotIn('progress_in_band', M['M4']['cycles']['F1']['checks'])
        self.assertEqual((M['M3']['verdict'], M['M4']['verdict'], M['M5']['verdict']),
                         ('PASS', 'PASS', 'PASS'))
        M = self.score_motion(line(y={18: 0.081}), shadow=True)    # |progress| above 80 mm
        self.assertIs(M['M4']['cycles']['F1']['checks']['progress_within_the_outer_bound'],
                      False)
        M = self.score_motion(p, shadow=False)                      # the same run, live: FAIL
        self.assertIs(M['M4']['checks']['leg_distance'], False)


class BoundaryExtraction(unittest.TestCase):
    """motion_from_samples on synthetic pose streams: each phase moves the body smoothly (zero
    velocity at both ends, as the gait's boundaries are), then it rests in the next goal's lead-in;
    node time equals sim time (RTF 1)."""

    @staticmethod
    def stream(settle_m=0.0, gap_at=None):
        poses, goals, t, y = [], {}, 0.0, 0.0
        for i in range(203):                    # at rest, 20 Hz, up to the arm at t = 10.1 s
            poses.append((round(0.05 * i, 9), {'xy': (0.0, 0.0), 'yaw': 0.0, 'z': Z0,
                                               'tilt': 0.01}))
        t_arm = t = 10.1
        for k in range(1, 111):
            step = 0.060 / 9 * (1 if k <= 54 or k > 99 else -1)
            for i in range(1, 11):                              # the phase: 1.0 s, smoothstep
                s = i / 10
                if gap_at == k and i == 10:
                    continue                                    # no sample at its end
                h = 3 * s * s - 2 * s ** 3
                poses.append((round(t + 0.1 * i, 9), {'xy': (0.0, y + step * h), 'yaw': 0.0,
                                                      'z': Z0, 'tilt': 0.02}))
            t, y = round(t + 1.0, 9), y + step
            goals[k] = {'done_t': t}
            for i in range(1, 11):                              # rest: 0.2 s lead-in, 50 Hz
                if gap_at != k:
                    poses.append((round(t + 0.02 * i, 9), {'xy': (settle_m, y), 'yaw': 0.0,
                                                           'z': Z0, 'tilt': 0.01}))
            t = round(t + 0.2, 9)
        jts = [(round(0.01 * i, 9), round(0.01 * i, 9)) for i in range(int(t / 0.01) + 2)]
        return sorted(poses, key=lambda x: x[0]), jts, goals, t_arm

    def test_boundaries_are_found_at_rest_after_continuous_phases(self):
        poses, jts, goals, t_arm = self.stream()
        m = s4d.motion_from_samples(poses, jts, {'ok': True}, goals, t_arm, None)
        b = m['boundaries']
        self.assertTrue(all(b[k]['pose'] is not None for k in s4d.BOUNDARY_GOALS), b)
        self.assertAlmostEqual(b[54]['pose']['xy'][1] - b[9]['pose']['xy'][1], 0.300, places=9)
        self.assertLess(b[18]['moved_mm'], 0.5)               # the last 0.1 s of the phase
        self.assertAlmostEqual(m['z_stand'], Z0)
        self.assertEqual(m['tilt_max']['F'], 0.02)
        self.assertGreater(m['all_sample_maxima']['samples'], 0)

    def test_an_instant_inside_a_phase_is_not_at_rest(self):
        poses, jts, goals, t_arm = self.stream()
        early = {k: {'done_t': v['done_t'] - 0.3} for k, v in goals.items()}
        m = s4d.motion_from_samples(poses, jts, {'ok': True}, early, t_arm, None)
        self.assertIsNone(m['boundaries'][18]['pose'])
        self.assertIn('not at rest', m['boundaries'][18]['reason'])

    def test_the_0_5_mm_rest_criterion_is_inclusive(self):
        for settle, ok in ((0.0005 - 1e-9, True), (0.0005 + UM, False)):
            poses, jts, goals, t_arm = self.stream(settle_m=settle)
            m = s4d.motion_from_samples(poses, jts, {'ok': True}, goals, t_arm, None)
            self.assertIs(m['boundaries'][27]['pose'] is not None, ok, m['boundaries'][27])

    def test_no_pose_near_the_instant_is_unmeasured(self):
        poses, jts, goals, t_arm = self.stream(gap_at=36)
        m = s4d.motion_from_samples(poses, jts, {'ok': True}, goals, t_arm, None)
        self.assertIsNone(m['boundaries'][36]['pose'])
        self.assertIn('no bag pose', m['boundaries'][36]['reason'])
        self.assertIsNotNone(m['boundaries'][45]['pose'])
        rc, ev, drv = bench_run()
        M = score(ev, drv, {**m, 'still': PASS})
        self.assertEqual(M['M4']['cycles']['F3']['verdict'], 'UNMEASURED')
        self.assertIsNone(M['M6']['checks']['cumulative_lateral_within_30_mm_at_every_'
                                            'boundary_0'])

    def test_missing_or_uncertain_alignment_makes_every_bag_part_unmeasured(self):
        poses, jts, goals, t_arm = self.stream()
        rc, ev, drv = bench_run()
        parts = ('M3', 'M4', 'M5', 'M6', 'M9', 'M10')
        for al in (None, {'offset_s': 0.0, 'p05_p95_ms': [-60.0, 1.0], 'tolerance_s': 0.2},
                   {'offset_s': 0.0}):
            with self.subTest(alignment=al):
                status = s4d.alignment_status(al)
                self.assertFalse(status['ok'])
                m = s4d.motion_from_samples(poses, jts, status, goals, t_arm, None)
                self.assertTrue(all(b['pose'] is None for b in m['boundaries'].values()))
                M = score(ev, drv, m)
                self.assertEqual(verdicts(M, parts), dict.fromkeys(parts, 'UNMEASURED'))
                self.assertEqual(verdicts(M, EVENT_PARTS), dict.fromkeys(EVENT_PARTS, 'PASS'))
        ok = s4d.alignment_status({'offset_s': 1.0, 'p05_p95_ms': [-2.3, 1.2],
                                   'tolerance_s': 0.67})
        self.assertTrue(ok['ok'])                                 # outliers do not count
        self.assertAlmostEqual(ok['uncertainty_s'], 0.0023)


class Parts(unittest.TestCase):
    def test_ownership_restore_library_and_the_run_installation(self):
        good = {'goals_sent': 111, 'cancels_sent': 0, 'max_foreign': 0, 'max_publishers': 0}
        ctl = {'received': 111, 'accepted': 111, 'cancel_requests': 0}
        self.assertEqual(s4d.assess_ownership(good, True, 0, None, ctl, 5000, False)['verdict'],
                         'PASS')
        self.assertEqual(s4d.assess_ownership({**good, 'goals_sent': 110}, True, 0, None, ctl,
                                              5000, False)['verdict'], 'FAIL')
        self.assertEqual(s4d.assess_ownership(good, True, 0, None, None, 5000, False)['verdict'],
                         'UNMEASURED')
        shadow_ctl = {'received': 0, 'accepted': 0, 'cancel_requests': 0}
        self.assertEqual(s4d.assess_ownership(good, True, 0, None, shadow_ctl, 0,
                                              True)['verdict'], 'PASS')
        self.assertEqual(s4d.assess_ownership(good, True, 0, None, shadow_ctl, 3,
                                              True)['verdict'], 'FAIL')
        report = ('ok: a. all three gates False\nok: c. m6_gait_replay.py --live refused, exit 3\n'
                  'ok: c. m6_live_playback --live refused, exit 3\nok: no simulation process\n'
                  'VERDICT: DISABLED AND VERIFIED\n')
        gates = {'install_base': '/x', 'gates': dict.fromkeys(s4d.GATES, False)}
        r = s4d.assess_restore_and_library(report, 'a', 'a', gates)
        self.assertEqual(r['verdict'], 'PASS', r)
        bad = {'gates': {**gates['gates'], s4d.GATES[2]: True}}
        self.assertIs(s4d.assess_restore_and_library(report, 'a', 'a', bad)['checks'][
            'run_installation_imports_all_three_gates_false'], False)
        short = {'gates': {s4d.GATES[0]: False}}
        self.assertEqual(s4d.assess_restore_and_library(report, 'a', 'a', short)['verdict'],
                         'FAIL')
        self.assertEqual(s4d.assess_restore_and_library(report, 'a', 'a', None)['verdict'],
                         'UNMEASURED')
        self.assertEqual(s4d.assess_restore_and_library(report, 'a', 'b', gates)['verdict'],
                         'FAIL')

    def test_overall(self):
        P, F, U, N = ({'verdict': v} for v in ('PASS', 'FAIL', 'UNMEASURED', 'N/A'))
        self.assertEqual(s4d.overall({'M1': P, 'M11': N}), 'PASS')
        self.assertEqual(s4d.overall({'M1': P, 'M2': U}), 'INCONCLUSIVE')
        self.assertEqual(s4d.overall({'M1': U, 'M2': F}), 'FAIL')


if __name__ == '__main__':
    unittest.main()
