#!/usr/bin/env python3
"""The S4c scorer, specified before any S4c run, at level 1 and at level 2.
- C1-C5 run against real in-memory bench runs: the node's own events and the S4c driver's log,
  live (the gate True in memory) and shadow (the node's ShadowTransport, the gate False). Copies of
  those runs are then mutated, to show that every contradiction FAILs and every gap is UNMEASURED.
- The displacement bands, tracking, ownership, margins and the overall use synthetic samples.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_score_s4c"""
import copy
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import score_s4c as s4c  # noqa: E402
import test_s4c_driver_live as s4ct  # noqa: E402

sc = s4c.sc
LEVELS = (1, 2)
_RUNS = {}
PASS = {'verdict': 'PASS'}
MOTION_PASS = {'forward': PASS, 'reverse': PASS, 'still': PASS}


def bench_run(level, shadow=False):
    """One normal S4c bench run per (level, shadow), cached: (rc, node events, driver records)."""
    key = (level, shadow)
    if key not in _RUNS:
        saved = s4ct.c55.M55_LOCOMOTION_DISPATCH_ENABLED
        s4ct.c55.M55_LOCOMOTION_DISPATCH_ENABLED = not shadow       # tests only, in memory
        try:
            with tempfile.TemporaryDirectory() as tmp:
                mod = s4ct.load_driver()
                bench = s4ct.make_bench(0.4, shadow=shadow,
                                        **({'progress_m': -3.28e-10} if shadow else {}))
                d = s4ct.make_driver(mod, bench, tmp, level, shadow=shadow)
                rc, why = mod.run(d)
                d.log(summary={'result': why, **d.summary()})          # as main() does
                d.out.close()
                with open(os.path.join(tmp, 'driver.jsonl')) as f:
                    drv = [json.loads(ln) for ln in f]
                _RUNS[key] = (rc, list(bench.s.log), drv)
        finally:
            s4ct.c55.M55_LOCOMOTION_DISPATCH_ENABLED = saved
    rc, ev, drv = _RUNS[key]
    return rc, copy.deepcopy(ev), copy.deepcopy(drv)


def score(ev, drv, level, motion=MOTION_PASS, shadow=False):
    return s4c.score_events(ev, drv, level, motion, shadow)[0]


def ev_of(ev, event, **match):
    return [e for e in ev if e['event'] == event and all(e.get(k) == v for k, v in match.items())]


def key_rec(drv, key, i=0):
    return [d for d in drv if d.get('key') == key and 't_before_write' in d][i]


class BenchRuns(unittest.TestCase):
    def test_live_and_shadow_bench_runs_pass_c1_to_c5_at_both_levels(self):
        for level in LEVELS:
            for shadow in (False, True):
                with self.subTest(level=level, shadow=shadow):
                    rc, ev, drv = bench_run(level, shadow)
                    self.assertEqual(rc, 0)
                    C = score(ev, drv, level, shadow=shadow)
                    self.assertEqual({k: v['verdict'] for k, v in C.items()},
                                     {f'C{i}': 'PASS' for i in range(1, 6)}, C)
                    self.assertEqual([c['label'] for c in C['C2']['confirmations']],
                                     ['2/3', '3/3'][:level])
                    for k in ('C3', 'C4'):
                        self.assertLessEqual(C[k]['ready_sim_s'], C[k]['sim_bound_s'])
                    self.assertEqual(C['C5']['requests_after_q'], {'stop': 2, 'disarm': 2})

    def test_shadow_and_live_runs_are_never_scored_as_each_other(self):
        for level in LEVELS:
            with self.subTest(level=level):
                rc, ev, drv = bench_run(level, shadow=True)
                C = score(ev, drv, level, shadow=False)
                self.assertFalse(C['C1']['checks']['node_dispatch_gate_true_throughout'])
                rc, ev, drv = bench_run(level, shadow=False)
                C = score(ev, drv, level, shadow=True)
                self.assertFalse(C['C1']['checks']['node_shadow_gate_false_throughout'])

    def test_without_the_bag_the_motion_parts_are_unmeasured_never_pass(self):
        for level in LEVELS:
            rc, ev, drv = bench_run(level)
            C = s4c.score_events(ev, drv, level, None)[0]
            with self.subTest(level=level):
                for k in ('C3', 'C4', 'C5'):
                    self.assertEqual(C[k]['verdict'], 'UNMEASURED')
                self.assertEqual([k for k, v in C['C5']['checks'].items() if v is not True],
                                 ['still_after_the_close'])
                C = score(ev, drv, level, {'forward': PASS, 'reverse': {'verdict': 'FAIL'},
                                           'still': {'verdict': 'FAIL'}})
                self.assertEqual((C['C3']['verdict'], C['C4']['verdict'], C['C5']['verdict']),
                                 ('PASS', 'FAIL', 'FAIL'))

    def test_a_level_1_run_scored_as_level_2_fails(self):
        rc, ev, drv = bench_run(1)
        C = score(ev, drv, 2)
        self.assertEqual((C['C1']['verdict'], C['C2']['verdict']), ('FAIL', 'FAIL'))
        self.assertFalse(C['C2']['checks']['recorded_level_equals_the_requested_level'])
        with self.assertRaises(ValueError):
            s4c.score_events(ev, drv, 0)


class Contradictions(unittest.TestCase):
    def mutate(self, f, level=1, shadow=False):
        rc, ev, drv = bench_run(level, shadow)
        f(ev, drv)
        return score(ev, drv, level, shadow=shadow)

    def assertFails(self, C, part, check):
        self.assertEqual(C[part]['verdict'], 'FAIL', C[part])
        self.assertIs(C[part]['checks'][check], False)

    def test_plan_wrong_direction_extra_or_duplicate_goal_cancel_and_fault(self):
        def wrong_direction(ev, drv):
            ev_of(ev, 'goal_sent', seq=10)[0].update(b_from=0, b_to=1, direction=1)

        def extra(ev, drv):
            last = ev_of(ev, 'goal_sent', seq=18)[0]
            i = ev.index(ev_of(ev, 'result', seq=18)[0])
            ev[i + 1:i + 1] = [{**last, 'seq': 19, 'n': last['n']},
                               {'event': 'result', 'seq': 19, 'status': 4, 'wall': last['wall']}]

        def duplicate(ev, drv):
            g = ev_of(ev, 'goal_sent', seq=5)[0]
            ev.insert(ev.index(g) + 1, dict(g))

        def cancel(ev, drv):
            g = ev_of(ev, 'goal_sent', seq=18)[0]
            ev.insert(ev.index(g) + 1, {'event': 'cancel_sent', 'seq': 18, 'wall': g['wall']})

        def fault(ev, drv):
            g = ev_of(ev, 'goal_sent', seq=3)[0]
            ev.insert(ev.index(g) + 1, {'event': 'fault', 'reason': 'estop', 'wall': g['wall']})
        for level in LEVELS:
            for f, check in ((wrong_direction, 'eighteen_planned_goals_in_order'),
                             (extra, 'eighteen_planned_goals_in_order'),
                             (duplicate, 'one_goal_sent_per_seq'), (cancel, 'no_cancel'),
                             (fault, 'no_fault')):
                with self.subTest(level=level, case=f.__name__):
                    self.assertFails(self.mutate(f, level), 'C1', check)

    def test_keys_and_requests_beyond_the_script(self):
        def home_request(ev, drv):
            g = ev_of(ev, 'goal_sent', seq=9)[0]
            ev.insert(ev.index(g), {'event': 'request', 'request': 'home', 'wall': g['wall']})

        def extra_key(ev, drv):
            k = key_rec(drv, "'a'")
            drv.insert(drv.index(k) + 1, {**k, 'key': "'h'", 'why': 'injected'})

        def short_write(ev, drv):
            key_rec(drv, "'w'")['write_result'] = 0
        for level in LEVELS:
            for f, check in ((home_request, 'requests_before_q_exactly_arm_stop_stop'),
                             (extra_key, 'keys_exactly_as_scripted'),
                             (short_write, 'every_key_write_complete')):
                with self.subTest(level=level, case=f.__name__):
                    self.assertFails(self.mutate(f, level), 'C1', check)

    def test_level_selection_contradictions(self):
        def goal_level(ev, drv):
            ev_of(ev, 'goal_sent', seq=12)[0]['level'] = 0

        def plus_after_arm(ev, drv):
            k, a = key_rec(drv, "'+'"), key_rec(drv, "'a'")
            k['t_before_write'] = k['t_after_write'] = a['t_after_write'] + 0.01

        def plus_while_ready(ev, drv):
            k = key_rec(drv, "'+'")
            st = [d for d in drv if 'status' in d and d['mono'] <= k['t_before_write']][-1]
            st['status']['state'] = 'READY'

        def wrong_label(ev, drv):
            d = next(d for d in drv if 'speed level 2/3' in (d.get('teleop_out') or ''))
            d['teleop_out'] = d['teleop_out'].replace('2/3', '3/3')

        def confirmation_before_key(ev, drv):
            d = next(d for d in drv if 'speed level 2/3' in (d.get('teleop_out') or ''))
            d['mono'] = key_rec(drv, "'+'")['t_before_write'] - 1.0

        def two_level_changes(ev, drv):
            lc = ev_of(ev, 'level_change')[0]
            ev.insert(ev.index(lc) + 1, dict(lc))

        def level_change_after_goal_1(ev, drv):
            lc = ev_of(ev, 'level_change')[0]
            lc['wall'] = ev_of(ev, 'goal_sent', seq=1)[0]['wall'] + 0.5

        def minus_key(ev, drv):
            k = key_rec(drv, "'+'")
            drv.insert(drv.index(k) + 1, {**k, 'key': "'-'"})
        cases = ((goal_level, 'every_goal_at_level_L'),
                 (plus_after_arm, 'level_keys_before_the_arm_key_and_request'),
                 (plus_while_ready, 'level_keys_while_disarmed_without_goal'),
                 (wrong_label, 'each_level_key_confirmed_by_the_teleop'),
                 (confirmation_before_key, 'each_level_key_confirmed_by_the_teleop'),
                 (two_level_changes, 'one_level_change_0_to_L_between_arm_and_goal_1'),
                 (level_change_after_goal_1, 'one_level_change_0_to_L_between_arm_and_goal_1'),
                 (minus_key, 'exactly_L_plus_keys_and_no_other_level_key'))
        for level in LEVELS:
            for f, check in cases:
                with self.subTest(level=level, case=f.__name__):
                    self.assertFails(self.mutate(f, level), 'C2', check)

    def test_missing_confirmations_are_unmeasured(self):
        def no_output(ev, drv):
            for d in drv:
                d.pop('teleop_out', None)
        for level in LEVELS:
            with self.subTest(level=level):
                C = self.mutate(no_output, level)
                self.assertEqual(C['C2']['verdict'], 'UNMEASURED')
                self.assertIsNone(C['C2']['checks']['each_level_key_confirmed_by_the_teleop'])

    def test_boundary_stop_contradictions(self):
        def late_space(ev, drv):
            k = key_rec(drv, "' '")
            k['t_before_write'] = k['t_after_write'] = \
                ev_of(ev, 'result', seq=9)[0]['wall'] + 0.01

        def intent_stop(ev, drv):
            tr = [e for e in ev_of(ev, 'transition', why='stop')][1]
            tr['why'] = 'intent_stop'

        def not_stopped(ev, drv):
            ev_of(ev, 'transition', why='stopped')[0]['why'] = 'homed'

        def slow_ready(ev, drv):
            t = ev_of(ev, 'transition', why='stopped')[1]['wall']
            for d in drv:
                if 'status' in d and d['mono'] >= t:
                    d['status']['monitors']['sim_time_s'] += 30.0

        def check_after_key(ev, drv):
            c = [d for d in drv if 'cycle_check' in d][0]
            c['mono'] = key_rec(drv, "'s'")['t_before_write'] + 0.01
        for level in LEVELS:
            for f, part, check in (
                    (late_space, 'C3', 'space_written_while_the_last_goal_was_in_flight'),
                    (intent_stop, 'C4', 'one_stopping_stop_in_the_last_goal'),
                    (not_stopped, 'C3', 'ready_stopped_at_boundary_0'),
                    (slow_ready, 'C4', 'ready_within_sim_bound'),
                    (check_after_key, 'C3', 'cycle_checked_before_the_next_key')):
                with self.subTest(level=level, case=f.__name__):
                    self.assertFails(self.mutate(f, level), part, check)

    def test_a_missing_cycle_check_is_unmeasured(self):
        def no_checks(ev, drv):
            drv[:] = [d for d in drv if 'cycle_check' not in d]
        C = self.mutate(no_checks)
        self.assertEqual((C['C3']['verdict'], C['C4']['verdict']), ('UNMEASURED', 'UNMEASURED'))

    def test_close_contradictions(self):
        def q_while_walking(ev, drv):
            q = key_rec(drv, "'q'")
            st = [d for d in drv if 'status' in d and d['mono'] <= q['t_before_write']][-1]
            st['status']['state'] = 'WALKING'

        def goal_after_q(ev, drv):
            q = key_rec(drv, "'q'")
            g = dict(ev_of(ev, 'goal_sent', seq=18)[0])
            ev.append({**g, 'seq': 19, 'wall': q['t_after_write'] + 1.0})

        def key_after_q(ev, drv):
            q = key_rec(drv, "'q'")
            drv.append({**q, 'key': "'a'", 't_before_write': q['t_after_write'] + 1.0,
                        't_after_write': q['t_after_write'] + 1.0})

        def estop_after_q(ev, drv):
            r = ev_of(ev, 'request', request='disarm')[-1]
            ev.insert(ev.index(r) + 1, {**r, 'request': 'estop'})

        def lease_instead_of_disarm(ev, drv):
            ev_of(ev, 'transition', state='DISARMED')[-1]['why'] = 'lease_expired'

        def nonzero_exit(ev, drv):
            next(d for d in drv if d.get('event') == 'teleop_closed')['exit_code'] = 1

        def ready_after_close(ev, drv):
            t = ev_of(ev, 'transition', state='DISARMED')[-1]['wall']
            st = [d for d in drv if 'status' in d and d['mono'] > t][3]
            st['status']['state'] = 'READY'
        for level in LEVELS:
            for f, check in ((q_while_walking, 'q_at_ready_boundary_0_without_goal'),
                             (goal_after_q, 'no_goal_after_q'),
                             (key_after_q, 'no_key_after_q'),
                             (estop_after_q, 'exactly_the_close_requests_after_q'),
                             (lease_instead_of_disarm, 'one_transition_ready_to_disarmed_disarm'),
                             (nonzero_exit, 'teleop_exit_code_0'),
                             (ready_after_close, 'disarmed_without_goal_over_10_s_of_sim')):
                with self.subTest(level=level, case=f.__name__):
                    self.assertFails(self.mutate(f, level), 'C5', check)

    def test_a_missing_teleop_exit_record_is_unmeasured(self):
        def no_exit(ev, drv):
            drv[:] = [d for d in drv if d.get('event') != 'teleop_closed']
        C = self.mutate(no_exit)
        self.assertEqual(C['C5']['verdict'], 'UNMEASURED')
        self.assertIsNone(C['C5']['checks']['teleop_exit_code_0'])


def pose(fwd=0.0, lat=0.0, yaw=0.0, z=0.0545, tilt=0.01):
    return {'xy': (lat, fwd), 'z': z, 'yaw': yaw, 'tilt': tilt}


class Displacement(unittest.TestCase):
    def disp(self, level, direction, end, shadow=False, start=None, z_stand=0.0545):
        poses = [(0.0, start or pose()), (1.0, pose(tilt=0.02)), (2.0, end)]
        return s4c.assess_displacement(direction, level, 0.0, 2.0, poses, z_stand, 0.1, shadow)

    def test_band_edges_are_inclusive_and_one_nanometre_outside_fails(self):
        for level in LEVELS:
            for direction in (1, -1):
                lo, hi = s4c.LEVELS[level]['band_m'][direction]
                for fwd, ok in ((lo, True), (hi, True), (lo - 1e-9, False), (hi + 1e-9, False)):
                    with self.subTest(level=level, direction=direction, fwd=fwd):
                        r = self.disp(level, direction, pose(fwd))
                        self.assertEqual(r['verdict'], 'PASS' if ok else 'FAIL', r)
                        self.assertIs(r['checks']['progress_in_band'], ok)
                        self.assertTrue(r['within_live_stop_band_recorded'])

    def test_reverse_is_measured_from_its_own_start_never_the_arm_pose(self):
        for level in LEVELS:
            s = s4c.LEVELS[level]['stride_m']
            r = self.disp(level, -1, pose(s - s), start=pose(s))
            self.assertAlmostEqual(r['forward_m'], -s)
            self.assertEqual(r['verdict'], 'PASS')

    def test_drift_height_and_tilt_limits_are_unchanged(self):
        for level in LEVELS:
            s = s4c.LEVELS[level]['stride_m']
            for end, check in ((pose(s, lat=0.0101), 'lateral_within_10_mm'),
                               (pose(s, yaw=0.051), 'yaw_within_0_05_rad'),
                               (pose(s, z=0.0545 + 0.0101), 'height_within_10_mm'),
                               (pose(s, tilt=0.11), 'tilt_within_0_10_rad')):
                with self.subTest(level=level, check=check):
                    r = self.disp(level, 1, end)
                    self.assertEqual(r['verdict'], 'FAIL')
                    self.assertIs(r['checks'][check], False)

    def test_shadow_drops_only_the_signed_band(self):
        for level in LEVELS:
            outer = s4c.outer_bound_m(level)
            with self.subTest(level=level):
                r = self.disp(level, 1, pose(0.0), shadow=True)
                self.assertEqual(r['verdict'], 'PASS')
                self.assertNotIn('progress_in_band', r['checks'])
                self.assertEqual(r['progress_band'], s4c.SHADOW_NA)
                self.assertEqual(self.disp(level, 1, pose(0.0))['verdict'], 'FAIL')    # live
                r = self.disp(level, -1, pose(-outer - 1e-6), shadow=True)
                self.assertIs(r['checks']['progress_within_the_outer_bound'], False)
                r = self.disp(level, 1, pose(0.0, lat=0.011), shadow=True)
                self.assertEqual(r['verdict'], 'FAIL')

    def test_missing_poses_are_unmeasured(self):
        r = s4c.assess_displacement(1, 1, 0.0, 2.0, [(0.0, pose())], 0.0545, 0.1, False)
        self.assertEqual(r['verdict'], 'UNMEASURED')
        r = s4c.assess_displacement(1, 1, None, 2.0, [(0.0, pose())], 0.0545, 0.1, False)
        self.assertEqual(r['verdict'], 'UNMEASURED')


class TrackingOwnershipMarginsOverall(unittest.TestCase):
    def goals(self, n=18):
        return [{'seq': k, 'sent': 10.0 * k, 'result_t': 10.0 * k + 5.0} for k in range(1, n + 1)]

    def test_every_goal_needs_feedback_no_na_and_the_limits_are_unchanged(self):
        fb = {f'g{k}': [(10.0 * k + 1.0, 0.003), (10.0 * k + 2.0, 0.004)] for k in range(1, 19)}
        r = s4c.s4a.assess_tracking(self.goals(), fb, 0.1, None)
        self.assertEqual(r['verdict'], 'PASS')
        del fb['g18']                                 # the last goal is never N/A in S4c
        r = s4c.s4a.assess_tracking(self.goals(), fb, 0.1, None)
        self.assertEqual((r['verdict'], r['goals'][-1]['verdict']), ('UNMEASURED', 'UNMEASURED'))
        fb['g18'] = [(181.0, 0.051)]
        self.assertEqual(s4c.s4a.assess_tracking(self.goals(), fb, 0.1, None)['verdict'], 'FAIL')
        fb['g18'] = [(181.0, 0.021)]
        r = s4c.s4a.assess_tracking(self.goals(), fb, 0.1, None)
        self.assertEqual((r['verdict'], list(r['findings_above_0_02_rad'])), ('PASS', [18]))

    def test_ownership_live_and_shadow(self):
        counts = {'goals_sent': 18, 'cancels_sent': 0, 'max_foreign': 0, 'max_publishers': 0}
        ctl = {'received': 18, 'accepted': 18, 'cancel_requests': 0}
        support = {'a': True}
        r = s4c.assess_ownership(counts, False, 0, support, ctl, 900, False)
        self.assertEqual(r['verdict'], 'PASS')
        for k, v in (('goals_sent', 19), ('cancels_sent', 1), ('max_foreign', 1),
                     ('max_publishers', 1)):
            with self.subTest(term=k):
                self.assertEqual(s4c.assess_ownership({**counts, k: v}, False, 0, support, ctl,
                                                      900, False)['verdict'], 'FAIL')
        self.assertEqual(s4c.assess_ownership(counts, False, 0, support, {**ctl, 'accepted': 17},
                                              900, False)['verdict'], 'FAIL')
        self.assertEqual(s4c.assess_ownership(counts, True, 1, support, ctl, 900, False)
                         ['verdict'], 'FAIL')
        self.assertEqual(s4c.assess_ownership(counts, False, 0, {'a': False}, ctl, 900, False)
                         ['verdict'], 'UNMEASURED')
        self.assertEqual(s4c.assess_ownership(counts, False, 0, support, None, 900, False)
                         ['verdict'], 'UNMEASURED')
        zero = {'received': 0, 'accepted': 0, 'cancel_requests': 0}
        self.assertEqual(s4c.assess_ownership(counts, False, 0, support, zero, 0, True)
                         ['verdict'], 'PASS')
        self.assertEqual(s4c.assess_ownership(counts, False, 0, support, ctl, 0, True)
                         ['verdict'], 'FAIL')                 # a goal reached the controller
        self.assertEqual(s4c.assess_ownership(counts, False, 0, support, zero, 3, True)
                         ['verdict'], 'FAIL')                 # feedback in a shadow run

    def test_margins_and_freshness(self):
        st = {'monitors': {'joint_states_age_s': 0.08, 'body_pose_age_s': 0.03,
                           'graph_age_s': 1.0}}
        sts = [(t, None, copy.deepcopy(st)) for t in (1.0, 2.0, 3.0)]
        ok = s4c.assess_margins((0.3, 'j', 0.1), sts, 0.5, 3.5, 'script complete')
        self.assertEqual(ok['verdict'], 'PASS')
        self.assertEqual(s4c.assess_margins((0.019, 'j', 0.1), sts, 0.5, 3.5,
                                            'script complete')['verdict'], 'FAIL')
        for k, v in (('joint_states_age_s', 0.51), ('body_pose_age_s', 1.01),
                     ('graph_age_s', 3.01)):
            bad = copy.deepcopy(sts)
            bad[1][2]['monitors'][k] = v
            with self.subTest(age=k):
                self.assertEqual(s4c.assess_margins((0.3, 'j', 0.1), bad, 0.5, 3.5,
                                                    'script complete')['verdict'], 'FAIL')
        self.assertEqual(s4c.assess_margins((0.3, 'j', 0.1), sts, 0.5, 3.5,
                                            'STOP CONDITION: x')['verdict'], 'FAIL')
        self.assertEqual(s4c.assess_margins(None, sts, 0.5, 3.5, None)['verdict'], 'UNMEASURED')

    def test_overall_ignores_only_not_applicable(self):
        P_, U, F, NA = ({'verdict': v} for v in ('PASS', 'UNMEASURED', 'FAIL', 'N/A'))
        self.assertEqual(s4c.overall({'a': P_, 'b': NA}), 'PASS')
        self.assertEqual(s4c.overall({'a': P_, 'b': U, 'c': NA}), 'INCONCLUSIVE')
        self.assertEqual(s4c.overall({'a': F, 'b': U}), 'FAIL')


class Constants(unittest.TestCase):
    def test_the_scorer_and_the_driver_share_the_plan_levels_and_keys(self):
        d = s4ct.load_driver()
        self.assertEqual(s4c.PLANNED, d.PLANNED)
        self.assertEqual((s4c.EXPECTED_GOALS, s4c.EXPECTED_CANCELS), (18, 0))
        for level in LEVELS:
            self.assertEqual(s4c.scripted_keys(level), d.scripted_keys(level))
            self.assertEqual(s4c.LEVELS[level]['stop_m'], d.LEVELS[level]['stop_m'])
            self.assertEqual(s4c.LEVELS[level]['stride_m'], d.LEVELS[level]['stride_m'])
            for direction in (1, -1):
                lo, hi = s4c.LEVELS[level]['band_m'][direction]
                slo, shi = s4c.LEVELS[level]['stop_m'][direction]
                self.assertTrue(slo <= lo < hi <= shi)          # acceptance inside the stops
                s = s4c.LEVELS[level]['stride_m'] * direction
                self.assertAlmostEqual((lo + hi) / 2, s)         # centred on the stride
                self.assertAlmostEqual(hi - lo, 0.020)           # +-10 mm, as level 0
        self.assertEqual(s4c.CLOSE_REQUESTS, s4c.collections.Counter(d.CLOSE_REQUESTS))
        self.assertEqual((sc.BAND_LATERAL_M, sc.BAND_YAW_RAD, sc.BAND_HEIGHT_M,
                          sc.TILT_MAX_RAD), (0.010, 0.05, 0.010, 0.10))
        self.assertEqual((sc.TRACKING_ABORT_RAD, sc.TRACKING_FINDING_RAD), (0.05, 0.02))

    def test_cli_needs_level_1_or_2_and_refuses_a_non_empty_out(self):
        with tempfile.TemporaryDirectory() as tmp:
            for argv in ([], ['--level', '0'], ['--level', '3']):
                with self.subTest(argv=argv), self.assertRaises(SystemExit):
                    s4c.main(['--run', tmp, '--bag', tmp, '--out', tmp, *argv])
            open(os.path.join(tmp, 'x'), 'w').close()
            self.assertEqual(s4c.main(['--run', tmp, '--bag', tmp, '--out', tmp, '--level', '1']),
                             3)


if __name__ == '__main__':
    unittest.main()
