#!/usr/bin/env python3
"""Synthetic tests of the S4a scorer's pure parts (score_s4a.py): A1 runs, A2 direction-aware
bands with wrapped yaw, the A3 planned-cancel exception on recorded key-write timestamps, A5,
the measured A6 hold (S3's, reused), and the evidence extraction.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_score_s4a"""
import copy
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import score_s4a as s4  # noqa: E402

sc = s4.sc
MSG = 'emergency stop: goal cancelled (if any), holding; reset and arm to continue'
TOL, PERIOD = 0.11, 0.05


def goals19():
    g = []
    for s in range(1, 19):
        rev = s <= 9
        b = 9 - (s - 1) if rev else s - 10
        g.append({'seq': s, 'sent': 10.0 * s, 'accepted': True, 'response_t': 10.0 * s + 0.05,
                  'result_t': 10.0 * s + 2.0, 'status': sc.STATUS_SUCCEEDED, 'level': 0,
                  'direction': -1 if rev else 1, 'b_from': b, 'b_to': b - 1 if rev else b + 1})
    g.append({'seq': 19, 'sent': 200.0, 'accepted': True, 'response_t': 200.05,
              'result_t': 200.10, 'status': sc.STATUS_CANCELED, 'cancel_response_code': 0,
              'level': 0, 'direction': 1, 'b_from': 0, 'b_to': 1})
    return g


def scenario():
    t_f = 200.0111                               # between the recorded before- and after-write
    return {
        'goals': goals19(),
        'feedback': {f'g{s}': [(10.0 * s + 0.1, 0.002), (10.0 * s + 1.5, 0.003)]
                     for s in range(1, 19)},
        'kb': {'n_estop_keys': 1, 'n_scripted_x': 1, 't_before': 200.0100, 't_after': 200.0116,
               'write_result': 1},
        'cancels': [{'seq': 19, 't': t_f}], 'faults': [{'reason': 'estop', 't': t_f}],
        'refusals': [], 'requests': [{'request': 'estop', 't': t_f, 'message': MSG}],
        'teleop': [{'t': 200.5, 'ok': True, 'message': MSG}], 'fallback': [],
        'controller': {'ok': True},
    }


def proof(s):
    return s4.estop_proof(s['goals'][-1], s['goals'], s['feedback'], s['kb'], s['cancels'],
                          s['faults'], s['refusals'], s['requests'], s['teleop'], s['fallback'],
                          s['controller'], TOL, PERIOD)


def set_fault(s, t):
    s['faults'][0]['t'] = s['cancels'][0]['t'] = s['requests'][0]['t'] = t


def pose(x, y, yaw, z=0.0545, tilt=0.003):
    return {'xy': (x, y), 'yaw': yaw, 'z': z, 'tilt': tilt}


def cycle_poses(r_fwd, f_fwd, yaw0=0.0, dyaw=(0.0, 0.0), lat=(0.0, 0.0), x0=1.0, y0=-2.0):
    """Poses at arm (t 0), end of R (t 20), end of F (t 40), with the body moving along its own
    +y by r_fwd then f_fwd (and lat along its x), in the frame of each start boundary's yaw."""
    def step(p, f, latv, dy):
        c, s = math.cos(p['yaw']), math.sin(p['yaw'])
        x = p['xy'][0] + c * latv - s * f
        y = p['xy'][1] + s * latv + c * f
        return pose(x, y, math.atan2(math.sin(p['yaw'] + dy), math.cos(p['yaw'] + dy)))
    p0 = pose(x0, y0, yaw0)
    p1 = step(p0, r_fwd, lat[0], dyaw[0])
    p2 = step(p1, f_fwd, lat[1], dyaw[1])
    mid = [(t, pose(x0, y0, yaw0)) for t in (5.0, 10.0, 15.0, 25.0, 30.0, 35.0)]
    return sorted([(0.0, p0), (20.0, p1), (40.0, p2)] + mid, key=lambda x: x[0])


class Runs(unittest.TestCase):
    def statuses(self):
        return [(19 * 10.0 + 2.5, {'state': 'READY', 'boundary': 0, 'goal': None}),
                (99.0, {'state': 'READY', 'boundary': 0, 'goal': None})]

    def test_runs_pass(self):
        r = s4.assess_runs(goals19(), [{'reason': 'estop', 't': 200.01}], self.statuses())
        self.assertEqual(r['verdict'], 'PASS', r)

    def test_wrong_direction_level_fault_or_no_ready_fail(self):
        g = goals19()
        g[3]['direction'] = 1
        self.assertEqual(s4.assess_runs(g, [], self.statuses())['verdict'], 'FAIL')
        g = goals19()
        g[12]['level'] = 1
        self.assertEqual(s4.assess_runs(g, [], self.statuses())['verdict'], 'FAIL')
        r = s4.assess_runs(goals19(), [{'reason': 'joint_states_stale', 't': 50.0}],
                           self.statuses())
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(s4.assess_runs(goals19(), [], self.statuses()[:1])['verdict'], 'FAIL')
        self.assertEqual(s4.assess_runs([], [], [])['verdict'], 'UNMEASURED')


class Cycles(unittest.TestCase):
    B = (0.0, 20.0, 40.0)

    def test_reverse_and_forward_within_bands_pass(self):
        r = s4.assess_cycles(self.B, cycle_poses(-0.020, 0.020), 0.0545, 0.2)
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual([c['direction'] for c in r['cycles']], [-1, 1])

    def test_reverse_band_edges(self):
        for f, v in ((-0.010, 'PASS'), (-0.030, 'PASS'), (-0.0095, 'FAIL'), (-0.0305, 'FAIL'),
                     (0.020, 'FAIL')):
            r = s4.assess_cycles(self.B, cycle_poses(f, 0.020, x0=0.0, y0=0.0), 0.0545, 0.2)
            self.assertEqual(r['cycles'][0]['verdict'], v, f)           # exact edges at origin

    def test_forward_band_edges(self):
        for f, v in ((0.010, 'PASS'), (0.030, 'PASS'), (0.0095, 'FAIL'), (-0.020, 'FAIL')):
            r = s4.assess_cycles(self.B, cycle_poses(-0.020, f, x0=0.0, y0=0.0), 0.0545, 0.2)
            self.assertEqual(r['cycles'][1]['verdict'], v, f)

    def test_return_is_measured_from_the_reverse_end_and_net_is_recorded_only(self):
        r = s4.assess_cycles(self.B, cycle_poses(-0.020, 0.024), 0.0545, 0.2)
        self.assertEqual(r['verdict'], 'PASS')
        self.assertAlmostEqual(r['cycles'][1]['forward_m'], 0.024, places=9)
        self.assertAlmostEqual(r['net_after_return_recorded_only']['forward_m'], 0.004, places=9)

    def test_yaw_is_wrapped_and_drift_bands_hold(self):
        r = s4.assess_cycles(self.B, cycle_poses(-0.020, 0.020, yaw0=math.pi - 0.01,
                                                 dyaw=(0.03, -0.03)), 0.0545, 0.2)
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertAlmostEqual(r['cycles'][0]['yaw_rad'], 0.03, places=9)
        r = s4.assess_cycles(self.B, cycle_poses(-0.020, 0.020, dyaw=(0.06, 0.0)), 0.0545, 0.2)
        self.assertEqual(r['cycles'][0]['verdict'], 'FAIL')
        r = s4.assess_cycles(self.B, cycle_poses(-0.020, 0.020, lat=(0.011, 0.0)), 0.0545, 0.2)
        self.assertEqual(r['cycles'][0]['verdict'], 'FAIL')

    def test_missing_boundary_or_standing_height_is_unmeasured(self):
        self.assertEqual(s4.assess_cycles((0.0, None, 40.0), cycle_poses(-0.02, 0.02), 0.0545,
                                          0.2)['verdict'], 'UNMEASURED')
        self.assertEqual(s4.assess_cycles(self.B, cycle_poses(-0.02, 0.02), None, 0.2)['verdict'],
                         'UNMEASURED')

    def test_cycle_boundaries_from_node_events(self):
        ev = [{'event': 'request', 'request': 'arm', 'message': 'armed at boundary 0', 'wall': 1.0},
              {'event': 'phase_done', 'direction': -1, 'b': 8, 'wall': 2.0},
              {'event': 'phase_done', 'direction': -1, 'b': 0, 'wall': 9.0},
              {'event': 'phase_done', 'direction': 1, 'b': 1, 'wall': 10.0},
              {'event': 'phase_done', 'direction': 1, 'b': 0, 'wall': 18.0}]
        self.assertEqual(s4.cycle_boundaries(ev), (1.0, 9.0, 18.0))
        self.assertEqual(s4.cycle_boundaries(ev[:3]), (1.0, 9.0, None))


class Exception_(unittest.TestCase):
    def test_reaction_inside_the_recorded_write_qualifies(self):
        self.assertTrue(proof(scenario())['qualifies'], proof(scenario())['failed'])

    def test_reaction_after_the_write_qualifies_and_at_write_start_too(self):
        for t in (200.0200, 200.0100):
            s = scenario()
            set_fault(s, t)
            self.assertTrue(proof(s)['qualifies'], (t, proof(s)['failed']))

    def test_reaction_before_the_write_started_is_rejected(self):
        s = scenario()
        set_fault(s, 200.0099)
        p = proof(s)
        self.assertIn('estop_fault_at_or_after_write_start', p['failed'])
        self.assertIn('no_unplanned_fault_before', p['failed'])

    def test_missing_or_failed_key_timestamps_are_rejected(self):
        for upd in ({'t_before': None}, {'t_after': None}, {'write_result': "OSError(5)"},
                    {'n_estop_keys': 2}, {'n_scripted_x': 0}):
            s = scenario()
            s['kb'].update(upd)
            self.assertIn('one_scripted_estop_key_written_during_goal', proof(s)['failed'], upd)

    def test_ambiguous_estop_sources_are_rejected(self):
        cases = {
            'second_request': (lambda s: s['requests'].append({'request': 'estop', 't': 200.03,
                                                               'message': MSG}),
                               'one_node_estop_request_at_the_fault'),
            'fallback': (lambda s: s['fallback'].append({'event': 'estop_service'}),
                         'no_driver_estop_service_fallback'),
            'no_teleop_response': (lambda s: s['teleop'].clear(),
                                   'one_teleop_estop_response_after_the_fault'),
            'message_differs': (lambda s: s['teleop'][0].update(message='x'),
                                'one_teleop_estop_response_after_the_fault'),
            'controller_log_missing': (lambda s: s.update(controller=None),
                                       'controller_log_order'),
        }
        for name, (mut, check) in cases.items():
            s = scenario()
            mut(s)
            self.assertIn(check, proof(s)['failed'], name)

    def test_missing_acceptance_cancel_response_or_canceled_result(self):
        for key, val, check in (('accepted', None, 'accepted'),
                                ('cancel_response_code', None, 'cancel_processed_by_server'),
                                ('status', sc.STATUS_SUCCEEDED, 'result_canceled')):
            s = scenario()
            s['goals'][-1][key] = val
            self.assertIn(check, proof(s)['failed'], key)

    def test_contradictory_sequences_are_rejected(self):
        s = scenario()
        s['goals'][-1]['result_t'] = 200.005                         # result before the cancel
        self.assertIn('event_order_consistent', proof(s)['failed'])
        s = scenario()
        s['kb'].update(t_before=199.9, t_after=199.91)              # written before the goal
        self.assertIn('one_scripted_estop_key_written_during_goal', proof(s)['failed'])
        s = scenario()
        s['cancels'][0]['t'] = 200.09                               # not at the fault
        self.assertIn('cancel_decision_at_the_fault_and_early', proof(s)['failed'])
        s = scenario()
        s['feedback']['g19'] = [(200.03, 0.001)]
        self.assertIn('no_feedback_for_goal', proof(s)['failed'])


class Tracking(unittest.TestCase):
    def test_pass_with_the_final_goal_na_and_no_value(self):
        s = scenario()
        r = s4.assess_tracking(s['goals'], s['feedback'], TOL, proof(s))
        self.assertEqual(r['verdict'], 'PASS')
        self.assertEqual(r['goals'][-1]['verdict'], 'N/A')
        self.assertNotIn('max_abs_error_rad', r['goals'][-1])

    def test_missing_feedback_on_a_normal_goal_is_unmeasured(self):
        s = scenario()
        del s['feedback']['g7']
        r = s4.assess_tracking(s['goals'], s['feedback'], TOL, proof(s))
        self.assertEqual((r['verdict'], r['goals'][6]['verdict']), ('UNMEASURED', 'UNMEASURED'))

    def test_final_goal_without_the_exception_is_unmeasured(self):
        s = scenario()
        set_fault(s, 200.0099)
        r = s4.assess_tracking(s['goals'], s['feedback'], TOL, proof(s))
        self.assertEqual(r['verdict'], 'UNMEASURED')

    def test_thresholds(self):
        s = scenario()
        s['feedback']['g4'] = [(40.2, 0.03)]
        r = s4.assess_tracking(s['goals'], s['feedback'], TOL, proof(s))
        self.assertEqual((r['verdict'], list(r['findings_above_0_02_rad'])), ('PASS', [4]))
        s['feedback']['g4'] = [(40.2, 0.051)]
        self.assertEqual(s4.assess_tracking(s['goals'], s['feedback'], TOL, proof(s))['verdict'],
                         'FAIL')


class OwnershipHoldOverall(unittest.TestCase):
    def test_ownership(self):
        ok = {'goals_sent': 19, 'cancels_sent': 1, 'max_foreign': 0, 'max_publishers': 0}
        self.assertEqual(s4.assess_ownership(ok, True, 0, None)['verdict'], 'PASS')
        self.assertEqual(s4.assess_ownership({**ok, 'goals_sent': 20}, True, 0,
                                             None)['verdict'], 'FAIL')
        self.assertEqual(s4.assess_ownership(ok, False, 0, {'a': True, 'b': False})['verdict'],
                         'UNMEASURED')
        self.assertEqual(s4.assess_ownership(ok, True, 3, None)['verdict'], 'FAIL')

    def hold(self, motion=0.0, until=12.0, canceled=True):
        sim_of = lambda t: t  # noqa: E731
        n = int(until * 10) + 1
        poses = [(t / 10.0, pose(0, 0, 0)) for t in range(n)]
        joints = [(t / 10.0, {'j': motion * t / (until * 10)}) for t in range(n)]
        return sc.assess_hold(True, canceled, 0.0, sim_of, poses, joints)

    def test_hold_is_measured_not_inferred_from_canceled(self):
        self.assertEqual(self.hold()['verdict'], 'PASS')
        self.assertEqual(self.hold(until=5.0)['verdict'], 'UNMEASURED')    # CANCELED, no 10 s
        self.assertEqual(self.hold(motion=0.02)['verdict'], 'FAIL')
        self.assertEqual(self.hold(canceled=False)['verdict'], 'FAIL')

    def test_overall(self):
        A = {'A1': {'verdict': 'PASS'}, 'A3': {'verdict': 'UNMEASURED'}}
        self.assertEqual(s4.overall(A), 'INCONCLUSIVE')
        self.assertEqual(s4.overall({**A, 'A2': {'verdict': 'FAIL'}}), 'FAIL')
        self.assertEqual(s4.overall({'A1': {'verdict': 'PASS'}}), 'PASS')


class Extraction(unittest.TestCase):
    def test_key_write_bracket(self):
        recs = [{'key': "'w'", 'why': 'estop exercise', 't_before_write': 1.0,
                 't_after_write': 1.1, 'write_result': 1, 'mono': 1.1},
                {'key': "'x'", 'why': 'estop exercise: x', 't_before_write': 2.0,
                 't_after_write': 2.0001, 'write_result': 1, 'mono': 2.0002}]
        kb = s4.key_write_bracket(recs)
        self.assertEqual((kb['t_before'], kb['t_after'], kb['write_result'], kb['n_estop_keys']),
                         (2.0, 2.0001, 1, 1))
        old = copy.deepcopy(recs)
        del old[1]['t_before_write']                                 # an S3-era record
        self.assertIsNone(s4.key_write_bracket(old)['t_before'])
        recs.append({'key': repr('\x1b'), 'why': 'other', 'mono': 3.0})
        self.assertEqual(s4.key_write_bracket(recs)['n_estop_keys'], 2)

    def test_teleop_and_controller_parsing(self):
        recs = [{'mono': 5.0, 'teleop_out': f'[disarm] REFUSED: x\r\n[estop] ok: {MSG}\r\n'}]
        self.assertEqual(s4.teleop_responses(recs), [{'t': 5.0, 'ok': True, 'message': MSG}])

        def line(t, s):
            return f'[INFO] [{t:.6f}] [leg_trajectory_controller]: {s}'
        log = [line(1.0, 'Received new action goal'), line(1.1, 'Accepted new action goal'),
               line(1.2, 'Got request to cancel goal')]
        self.assertTrue(s4.controller_order(log, 1)['ok'])
        self.assertFalse(s4.controller_order(log, 2)['ok'])


if __name__ == '__main__':
    unittest.main()
