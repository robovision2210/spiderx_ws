#!/usr/bin/env python3
"""The S4b scorer, specified before any S4b run.
- B1-B5 run against a real in-memory bench run: the node's own events and the S4b driver's log.
  Copies of that run are then mutated, to show that every contradiction FAILs and every gap is
  UNMEASURED.
- B6, B7, B9 and the post-lease stillness use synthetic samples.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_score_s4b"""
import copy
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import score_s4b as sb  # noqa: E402
import test_s4b_driver_live as s4bt  # noqa: E402

sc = sb.sc
_RUN = {}


def bench_run():
    """One normal S4b bench run, cached: (rc, node events, driver records)."""
    if not _RUN:
        saved = s4bt.c55.M55_LOCOMOTION_DISPATCH_ENABLED
        s4bt.c55.M55_LOCOMOTION_DISPATCH_ENABLED = True             # tests only, in memory
        try:
            with tempfile.TemporaryDirectory() as tmp:
                mod = s4bt.load_driver()
                bench = s4bt.s4at.make_bench(0.2)
                d = s4bt.make_driver(mod, bench, tmp)
                rc, _ = mod.run(d)
                d.out.flush()
                with open(os.path.join(tmp, 'driver.jsonl')) as f:
                    drv = [json.loads(ln) for ln in f]
                _RUN.update(rc=rc, ev=list(bench.s.log), drv=drv)
        finally:
            s4bt.c55.M55_LOCOMOTION_DISPATCH_ENABLED = saved
    return _RUN['rc'], copy.deepcopy(_RUN['ev']), copy.deepcopy(_RUN['drv'])


STILL_PASS = {'verdict': 'PASS'}


def score(ev, drv, still=STILL_PASS):
    return sb.score_events(ev, drv, still)[0]


def ev_of(ev, event, **match):
    return [e for e in ev if e['event'] == event and all(e.get(k) == v for k, v in match.items())]


class BenchRun(unittest.TestCase):
    def test_the_bench_run_passes_b1_to_b5(self):
        rc, ev, drv = bench_run()
        self.assertEqual(rc, 0)
        B = score(ev, drv)
        self.assertEqual({k: v['verdict'] for k, v in B.items()},
                         {'B1': 'PASS', 'B2': 'PASS', 'B3': 'PASS', 'B4': 'PASS', 'B5': 'PASS'})
        self.assertAlmostEqual(B['B2']['detection_latency_s'], 0.8, delta=0.06)
        self.assertLessEqual(B['B2']['ready_sim_s'], B['B2']['sim_bound_s'])
        self.assertLessEqual(B['B4']['home_sim_s'], sb.HOME_SIM_BOUND_S)
        self.assertLessEqual(B['B5']['disarmed_sim_s'], B['B5']['sim_bound_s'])
        self.assertFalse(B['B2']['finding_detection_above_documented'])
        self.assertFalse(B['B5']['finding_stopping_above_documented'])

    def test_without_the_bag_b5_is_unmeasured_never_pass(self):
        rc, ev, drv = bench_run()
        B = sb.score_events(ev, drv, None)[0]
        self.assertEqual(B['B5']['verdict'], 'UNMEASURED')
        self.assertEqual([k for k, v in B['B5']['checks'].items() if v is not True],
                         ['still_after_lease_loss'])
        B = score(ev, drv, {'verdict': 'FAIL'})
        self.assertEqual(B['B5']['verdict'], 'FAIL')


class Contradictions(unittest.TestCase):
    def mutate(self, f):
        rc, ev, drv = bench_run()
        f(ev, drv)
        return score(ev, drv)

    def test_a_space_or_stop_request_is_not_a_release(self):
        def space(ev, drv):
            k = next(d for d in drv if d.get('key') == "'w'")
            drv.insert(drv.index(k) + 1, {**k, 'key': "' '", 'why': 'injected space'})
        B = self.mutate(space)
        self.assertEqual(B['B1']['verdict'], 'FAIL')
        self.assertFalse(B['B1']['checks']['no_space_estop_reset_disarm_or_quit_key'])

        def stop(ev, drv):
            tr = ev_of(ev, 'transition', why='intent_stop')[0]
            tr['why'] = 'stop'
            ev.insert(ev.index(tr), {**tr, 'event': 'request', 'request': 'stop'})
        B = self.mutate(stop)
        self.assertEqual((B['B1']['verdict'], B['B2']['verdict']), ('FAIL', 'FAIL'))
        self.assertFalse(B['B2']['checks']['one_stopping_intent_stop'])

    def test_a_stop_before_the_hold_window_is_not_a_release(self):
        def early(ev, drv):
            rel = next(d for d in drv if 'release' in d)
            tr = ev_of(ev, 'transition', why='intent_stop')[0]
            tr['wall'] = rel['t_after_stop'] + 0.3
        B = self.mutate(early)
        self.assertFalse(B['B2']['checks']['stopping_after_the_hold_window'])

    def test_a_cancel_fault_or_extra_goal_fails_the_plan(self):
        def cancel(ev, drv):
            g = ev_of(ev, 'goal_sent', seq=3)[0]
            ev.insert(ev.index(g) + 1, {'event': 'cancel_sent', 'seq': 3, 'wall': g['wall'],
                                        'n': g['n'], 'state': 'WALKING'})
        B = self.mutate(cancel)
        self.assertFalse(B['B1']['checks']['no_cancel'])

        def extra(ev, drv):
            ready = [e for e in ev if e['event'] == 'transition' and e['why'] == 'stopped'][0]
            g = {**ev_of(ev, 'goal_sent', seq=5)[0], 'seq': 6, 'b_from': 5, 'b_to': 6,
                 'wall': ready['wall'] - 0.5}
            ev.insert(ev.index(ready), g)
        B = self.mutate(extra)
        self.assertEqual((B['B1']['verdict'], B['B2']['verdict']), ('FAIL', 'FAIL'))
        self.assertFalse(B['B1']['checks']['one_goal_sent_per_seq'])
        self.assertFalse(B['B2']['checks']['no_goal_between_release_and_ready'])

    def test_contradictory_event_order_fails(self):
        def reorder(ev, drv):
            r = ev_of(ev, 'result', seq=4)[0]
            r['wall'] = ev_of(ev, 'goal_sent', seq=4)[0]['wall'] - 1.0
        B = self.mutate(reorder)
        self.assertFalse(B['B1']['checks']['event_order_consistent'])

    def test_reversal_contradictions_fail(self):
        def phase(ev, drv):
            ev_of(ev, 'goal_sent', seq=7)[0]['phase'] = 4
        self.assertFalse(self.mutate(phase)['B3']['checks']['forward_then_same_phase_back'])

        def late_s(ev, drv):
            k = next(d for d in drv if d.get('key') == "'s'")
            done = ev_of(ev, 'result', seq=6)[0]['wall']
            k['t_before_write'], k['t_after_write'] = done + 0.1, done + 0.1
        B = self.mutate(late_s)
        self.assertFalse(B['B3']['checks']['s_written_while_forward_goal_in_flight'])

        def stop_between(ev, drv):
            g7 = ev_of(ev, 'goal_sent', seq=7)[0]
            ev.insert(ev.index(g7), {'event': 'transition', 'previous': 'WALKING',
                                     'state': 'READY', 'why': 'intent_stop',
                                     'wall': g7['wall'] - 0.01, 'n': g7['n']})
        self.assertFalse(self.mutate(stop_between)['B3']['checks'][
            'reverse_sent_at_the_boundary'])

    def test_home_contradictions_fail(self):
        def backward(ev, drv):
            ev_of(ev, 'homing')[0]['direction'] = -1
        self.assertEqual(self.mutate(backward)['B4']['verdict'], 'FAIL')

        def motion(ev, drv):
            h = next(d for d in drv if d.get('key') == "'h'")
            drv.insert(drv.index(h) + 1, {**h, 'key': "'w'", 'why': 'injected',
                                          't_before_write': h['t_after_write'] + 1.0,
                                          't_after_write': h['t_after_write'] + 1.0})
        self.assertFalse(self.mutate(motion)['B4']['checks']['no_motion_key_during_home'])

    def test_lease_contradictions_fail(self):
        def key_after(ev, drv):
            k = next(d for d in drv if d.get('event') == 'teleop_killed')
            drv.append({'key': "'a'", 'why': 'injected', 't_before_write': k['t_after_kill'] + 1,
                        't_after_write': k['t_after_kill'] + 1, 'write_result': 1,
                        'mono': k['t_after_kill'] + 1})
        B = self.mutate(key_after)
        self.assertFalse(B['B5']['checks']['no_key_after_the_kill'])

        def goal_after(ev, drv):
            dis = ev_of(ev, 'transition', state='DISARMED')[-1]
            ev.append({**ev_of(ev, 'goal_sent', seq=12)[0], 'seq': 13, 'b_from': 1, 'b_to': 2,
                       'wall': dis['wall'] + 1.0, 'n': dis['n'] + 100})
        B = self.mutate(goal_after)
        self.assertFalse(B['B5']['checks']['no_goal_after_the_kill'])
        self.assertEqual(B['B1']['verdict'], 'FAIL')

        def clean_quit(ev, drv):
            for tr in ev_of(ev, 'transition'):
                if tr['why'] == 'lease_expired':
                    tr['why'] = 'disarm'
        B = self.mutate(clean_quit)
        self.assertFalse(B['B5']['checks']['one_stopping_lease_expired'])
        self.assertFalse(B['B5']['checks']['disarmed_lease_expired_at_boundary_1'])

        def too_early(ev, drv):
            k = next(d for d in drv if d.get('event') == 'teleop_killed')
            ev_of(ev, 'transition', state='STOPPING', why='lease_expired')[0]['wall'] = \
                k['t_before_kill'] + 0.1
        self.assertFalse(self.mutate(too_early)['B5']['checks']['stopping_after_the_lease'])

        def rearmed(ev, drv):
            k = next(d for d in drv if d.get('event') == 'teleop_killed')
            st = [d for d in drv if 'status' in d and d['mono'] > k['t_after_kill'] + 15]
            st[-1]['status']['state'] = 'READY'
        self.assertFalse(self.mutate(rearmed)['B5']['checks']['disarmed_throughout_10_s_of_sim'])

    def test_sim_bounds_and_wall_backstops(self):
        def slow_ready(ev, drv):
            rel = next(d for d in drv if 'release' in d)
            for d in drv:
                st = d.get('status')
                if st and d['mono'] > rel['t_after_stop'] and st.get('state') == 'READY' \
                        and st.get('boundary') == 5:
                    st['monitors']['sim_time_s'] += 5.0
                    break
        self.assertFalse(self.mutate(slow_ready)['B2']['checks']['ready_within_sim_bound'])

        def late_wall(ev, drv):
            ev_of(ev, 'transition', why='stopped')[0]['wall'] += 121.0
        self.assertFalse(self.mutate(late_wall)['B2']['checks']['ready_within_wall_backstop'])

        def slow_home(ev, drv):
            h = next(d for d in drv if d.get('key') == "'h'")
            for d in drv:
                st = d.get('status')
                if st and d['mono'] > h['t_after_write'] and st.get('state') == 'READY' \
                        and st.get('boundary') == 0:
                    st['monitors']['sim_time_s'] += 20.0
                    break
        self.assertFalse(self.mutate(slow_home)['B4']['checks'][
            'within_documented_homing_bound'])

        def slow_disarm(ev, drv):
            k = next(d for d in drv if d.get('event') == 'teleop_killed')
            for d in drv:
                st = d.get('status')
                if st and d['mono'] > k['t_after_kill'] and st.get('state') == 'DISARMED':
                    st['monitors']['sim_time_s'] += 5.0
                    break
        self.assertFalse(self.mutate(slow_disarm)['B5']['checks']['disarmed_within_sim_bound'])


class MissingEvidence(unittest.TestCase):
    def strip(self, f):
        rc, ev, drv = bench_run()
        ev, drv = f(ev, drv)
        return score(ev, drv)

    def test_missing_records_are_unmeasured(self):
        B = self.strip(lambda ev, drv: (ev, [d for d in drv if 'release' not in d]))
        self.assertEqual(B['B2']['verdict'], 'UNMEASURED')
        B = self.strip(lambda ev, drv: (ev, [d for d in drv if d.get('event') != 'teleop_killed']))
        self.assertEqual(B['B5']['verdict'], 'UNMEASURED')
        B = self.strip(lambda ev, drv: (ev, [d for d in drv if d.get('key') != "'h'"]))
        self.assertEqual(B['B4']['verdict'], 'UNMEASURED')
        B = self.strip(lambda ev, drv: (ev, [d for d in drv if 'switch' not in d]))
        self.assertEqual(B['B3']['verdict'], 'UNMEASURED')

    def test_missing_statuses_leave_the_sim_bounds_unmeasured(self):
        B = self.strip(lambda ev, drv: (ev, [d for d in drv if 'status' not in d]))
        for k in ('B2', 'B4', 'B5'):
            self.assertEqual(B[k]['verdict'], 'UNMEASURED', k)

    def test_missing_events_never_pass(self):
        B = self.strip(lambda ev, drv: ([e for e in ev if e['event'] != 'goal_sent'], drv))
        self.assertTrue(all(v['verdict'] != 'PASS' for v in B.values()))


def pose(x, y, yaw, z=0.0545, tilt=0.002):
    return {'xy': [x, y], 'yaw': yaw, 'z': z, 'tilt': tilt}


def moved(forward, lateral=0.0, yaw0=0.3, dyaw=0.0):
    """Poses: at rest at t 0-10 (arm at 10), moving to the homed time 40."""
    x1 = -math.sin(yaw0) * forward + math.cos(yaw0) * lateral
    y1 = math.cos(yaw0) * forward + math.sin(yaw0) * lateral
    ps = [(t, pose(0.0, 0.0, yaw0)) for t in range(0, 11)]
    ps += [(t, pose(x1 * (t - 10) / 30, y1 * (t - 10) / 30, yaw0 + dyaw * (t - 10) / 30))
           for t in range(11, 41)]
    return ps


class CycleTrackingOwnershipStill(unittest.TestCase):
    def test_the_composite_cycle_uses_the_s4a_forward_band(self):
        for f, v in ((0.020, 'PASS'), (0.010, 'PASS'), (0.0099, 'FAIL'), (0.030, 'PASS'),
                     (0.0301, 'FAIL'), (-0.020, 'FAIL')):
            r = sb.assess_cycle(10.0, 40.0, moved(f, yaw0=0.0), 0.0545, 0.6)  # exact edges
            self.assertEqual(r['verdict'], v, f)
        self.assertEqual(sb.assess_cycle(10.0, 40.0, moved(0.02), 0.0545, 0.6)['verdict'],
                         'PASS')                                    # in a rotated frame
        self.assertEqual(sb.assess_cycle(10.0, 40.0, moved(0.02, lateral=0.012), 0.0545,
                                         0.6)['verdict'], 'FAIL')
        self.assertEqual(sb.assess_cycle(10.0, 40.0, moved(0.02, dyaw=0.06), 0.0545,
                                         0.6)['verdict'], 'FAIL')
        self.assertEqual(sb.assess_cycle(10.0, 40.0, [], 0.0545, 0.6)['verdict'], 'UNMEASURED')
        self.assertEqual(sb.assess_cycle(None, 40.0, moved(0.02), 0.0545, 0.6)['verdict'],
                         'UNMEASURED')

    def test_tracking_needs_feedback_for_every_goal(self):
        goals = [{'seq': k, 'sent': 10.0 * k, 'result_t': 10.0 * k + 5} for k in range(1, 13)]
        fb = {f'g{k}': [(10.0 * k + 1, 0.002), (10.0 * k + 2, 0.003)] for k in range(1, 13)}
        self.assertEqual(sb.s4a.assess_tracking(goals, fb, 0.2, None)['verdict'], 'PASS')
        fb12 = {k: v for k, v in fb.items() if k != 'g12'}
        r = sb.s4a.assess_tracking(goals, fb12, 0.2, None)
        self.assertEqual(r['verdict'], 'UNMEASURED')               # no N/A without a proof
        fb['g5'] = [(51.0, 0.06)]
        self.assertEqual(sb.s4a.assess_tracking(goals, fb, 0.2, None)['verdict'], 'FAIL')
        fb['g5'] = [(51.0, 0.03)]
        r = sb.s4a.assess_tracking(goals, fb, 0.2, None)
        self.assertEqual((r['verdict'], list(r['findings_above_0_02_rad'])), ('PASS', [5]))

    def test_ownership_counts_twelve_goals_and_no_cancel(self):
        ok = {'goals_sent': 12, 'cancels_sent': 0, 'max_foreign': 0, 'max_publishers': 0}
        ctl = {'received': 12, 'accepted': 12, 'cancel_requests': 0}
        self.assertEqual(sb.assess_ownership(ok, True, 0, None, ctl)['verdict'], 'PASS')
        for bad in ({'goals_sent': 13}, {'cancels_sent': 1}, {'max_foreign': 1}):
            self.assertEqual(sb.assess_ownership({**ok, **bad}, True, 0, None, ctl)['verdict'],
                             'FAIL', bad)
        self.assertEqual(sb.assess_ownership(ok, True, 0, None, {**ctl, 'cancel_requests': 1})
                         ['verdict'], 'FAIL')
        self.assertEqual(sb.assess_ownership(ok, True, 0, None, None)['verdict'], 'UNMEASURED')
        self.assertEqual(sb.assess_ownership(ok, False, 0, {'a': True, 'b': False}, ctl)
                         ['verdict'], 'UNMEASURED')
        self.assertEqual(sb.assess_ownership(ok, True, 2, None, ctl)['verdict'], 'FAIL')

    def test_stillness_after_lease_loss_is_measured(self):
        def run(motion=0.0, until=12.0):
            n = int(until * 10) + 1
            poses = [(t / 10.0, pose(0, 0, 0)) for t in range(n)]
            joints = [(t / 10.0, {'j': motion * t / (until * 10)}) for t in range(n)]
            return sb.assess_still(0.0, lambda t: t, poses, joints)
        r = run()
        self.assertEqual(r['verdict'], 'PASS')
        self.assertNotIn('faulted_with_one_cancel', r)              # nothing was cancelled
        self.assertEqual(run(until=5.0)['verdict'], 'UNMEASURED')
        self.assertEqual(run(motion=0.02)['verdict'], 'FAIL')

    def test_overall(self):
        self.assertEqual(sb.overall({'B1': {'verdict': 'PASS'}}), 'PASS')
        self.assertEqual(sb.overall({'B1': {'verdict': 'PASS'}, 'B2': {'verdict': 'UNMEASURED'}}),
                         'INCONCLUSIVE')
        self.assertEqual(sb.overall({'B1': {'verdict': 'FAIL'}, 'B2': {'verdict': 'UNMEASURED'}}),
                         'FAIL')


class Constants(unittest.TestCase):
    def test_plan_matches_the_driver_and_limits_are_unchanged(self):
        drv = s4bt.load_driver()
        self.assertEqual(sb.PLANNED, drv.PLANNED)
        self.assertEqual((sb.EXPECTED_GOALS, sb.EXPECTED_CANCELS), (12, 0))
        self.assertEqual(sb.s4a.BAND_PROGRESS_M[1], (0.010, 0.030))
        self.assertEqual((sc.BAND_LATERAL_M, sc.BAND_YAW_RAD, sc.BAND_HEIGHT_M, sc.TILT_MAX_RAD),
                         (0.010, 0.05, 0.010, 0.10))
        self.assertEqual((sb.SIM_MARGIN_S, sb.WALL_BACKSTOP_S, sb.HOME_SIM_BOUND_S),
                         (1.0, 120.0, 12.0))
        self.assertEqual((sb.HOLD_S, sb.HEARTBEAT_S, sb.LEASE_S, sb.TICK_S), (0.8, 0.1, 0.5, 0.05))

    def test_refuses_a_non_empty_output_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            open(os.path.join(tmp, 'x'), 'w').close()
            self.assertEqual(sb.main(['--run', tmp, '--bag', tmp, '--out', tmp]), 3)


if __name__ == '__main__':
    unittest.main()
