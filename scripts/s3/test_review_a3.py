#!/usr/bin/env python3
"""Synthetic tests of the offline A3 review (review_a3.py): the S3-r1 bracketed final-estop
exception, its rejections, the per-goal tracking rule, and the evidence extraction helpers. The
frozen scorer (score_s3.py) is imported unchanged, to show where its point-time check differs.

Usage (stdlib only): cd scripts/s3 && python3 -m unittest -v test_review_a3"""
import copy
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..', 's2'), HERE]
import review_a3 as rv  # noqa: E402
import score_s3 as sc  # noqa: E402

MSG = 'emergency stop: goal cancelled (if any), holding; reset and arm to continue'
TOL, PERIOD = 0.11, 0.05


def scenario():
    goals = [{'seq': s, 'sent': 10.0 + s, 'accepted': True, 'response_t': 10.05 + s,
              'result_t': 10.9 + s, 'status': sc.STATUS_SUCCEEDED, 'b_from': (s - 1) % 9,
              'b_to': (s - 1) % 9 + 1, 'direction': 1, 'level': 0} for s in range(1, 55)]
    goals.append({'seq': 55, 'sent': 100.0, 'accepted': True, 'response_t': 100.05,
                  'result_t': 100.10, 'status': sc.STATUS_CANCELED, 'cancel_response_code': 0,
                  'b_from': 0, 'b_to': 1, 'direction': 1, 'level': 0})
    fault_t = 100.0111                         # after the bracket start, before the post-write log
    return {
        'goals': goals,
        'feedback': {f'g{s}': [(10.1 + s, 0.002), (10.5 + s, 0.003)] for s in range(1, 55)},
        'xb': {'n_estop_keys': 1, 'n_scripted_x': 1, 't_pre': 100.0100, 't_post': 100.0116,
               'pre_record': 'estop exercise: goal in flight'},
        'cancels': [{'seq': 55, 't': fault_t}],
        'faults': [{'reason': 'estop', 't': fault_t}],
        'refusals': [],
        'requests': [{'request': 'estop', 't': fault_t, 'message': MSG}],
        'teleop': [{'t': 100.5, 'ok': True, 'message': MSG}],
        'fallback': [],
        'controller': {'ok': True},
        'keys': [{'key': 'x', 't': 100.0116, 'why': 'estop exercise: x'}],
    }


def proof(s):
    return rv.estop_bracket_proof(s['goals'][-1], s['goals'], s['feedback'], s['xb'],
                                  s['cancels'], s['faults'], s['refusals'], s['requests'],
                                  s['teleop'], s['fallback'], s['controller'], TOL, PERIOD)


def frozen(s):
    return sc.estop_na_proof(s['goals'][-1], s['goals'], s['feedback'], s['keys'], s['cancels'],
                             s['faults'], s['refusals'], TOL, PERIOD)


def set_fault(s, t):
    s['faults'][0]['t'] = s['cancels'][0]['t'] = s['requests'][0]['t'] = t


class BracketedException(unittest.TestCase):
    def test_receiver_reacting_before_the_post_write_log_qualifies(self):
        s = scenario()
        self.assertTrue(proof(s)['qualifies'], proof(s)['failed'])
        self.assertEqual(sorted(frozen(s)['failed']), sorted(
            ['estop_fault_after_key', 'early_cancel_request', 'no_unplanned_fault_before']))

    def test_correctly_bracketed_reaction_after_the_post_write_log_qualifies(self):
        s = scenario()
        set_fault(s, 100.0200)
        self.assertTrue(proof(s)['qualifies'], proof(s)['failed'])
        self.assertTrue(frozen(s)['qualifies'], frozen(s)['failed'])

    def test_fault_before_the_write_could_start_is_rejected(self):
        for t in (100.0099, 100.0100 + 0.00004):     # before t_pre, or inside its rounding
            s = scenario()
            set_fault(s, t)
            p = proof(s)
            self.assertFalse(p['qualifies'])
            self.assertIn('estop_fault_after_bracket_start', p['failed'])
            self.assertIn('no_unplanned_fault_before', p['failed'])

    def test_ambiguous_or_unrelated_estop_sources_are_rejected(self):
        cases = {
            'second_node_estop_request': (
                lambda s: s['requests'].append({'request': 'estop', 't': 100.03, 'message': MSG}),
                'one_node_estop_request_at_the_fault'),
            'driver_estop_service_fallback': (
                lambda s: s['fallback'].append({'event': 'estop_service'}),
                'no_driver_estop_service_fallback'),
            'second_estop_key': (lambda s: s['xb'].update(n_estop_keys=2),
                                 'one_scripted_estop_key_bracketed_in_goal'),
            'no_teleop_response': (lambda s: s['teleop'].clear(),
                                   'one_teleop_estop_response_after_the_fault'),
            'teleop_refused': (lambda s: s['teleop'][0].update(ok=False),
                               'one_teleop_estop_response_after_the_fault'),
            'teleop_message_differs': (lambda s: s['teleop'][0].update(message='other'),
                                       'one_teleop_estop_response_after_the_fault'),
            'teleop_response_before_fault': (lambda s: s['teleop'][0].update(t=100.005),
                                             'one_teleop_estop_response_after_the_fault'),
            'second_estop_fault': (lambda s: s['faults'].append({'reason': 'estop', 't': 100.05}),
                                   'estop_fault_after_bracket_start'),
            'other_fault_first': (lambda s: s['faults'].insert(0, {'reason': 'joint_states_stale',
                                                                   't': 100.005}),
                                  'no_unplanned_fault_before'),
        }
        for name, (mutate, check) in cases.items():
            s = scenario()
            mutate(s)
            p = proof(s)
            self.assertFalse(p['qualifies'], name)
            self.assertIn(check, p['failed'], name)

    def test_missing_goal_acceptance_is_rejected(self):
        for acc in (None, False):
            s = scenario()
            s['goals'][-1]['accepted'] = acc
            self.assertIn('accepted', proof(s)['failed'])

    def test_missing_cancel_response_or_canceled_result_is_rejected(self):
        s = scenario()
        s['goals'][-1]['cancel_response_code'] = None
        self.assertIn('cancel_processed_by_server', proof(s)['failed'])
        s = scenario()
        s['goals'][-1]['status'] = sc.STATUS_SUCCEEDED
        self.assertIn('result_canceled', proof(s)['failed'])
        s = scenario()
        del s['goals'][-1]['result_t']
        p = proof(s)
        self.assertFalse(p['qualifies'])
        self.assertIn('event_order_consistent', p['failed'])

    def test_contradictory_sequences_are_rejected(self):
        s = scenario()                                 # result before the cancel decision
        s['goals'][-1]['result_t'] = 100.005
        p = proof(s)
        self.assertIn('event_order_consistent', p['failed'])
        self.assertIn('estop_fault_after_bracket_start', p['failed'])
        s = scenario()                                 # key bracket before the goal was sent
        s['xb'].update(t_pre=99.95, t_post=99.96)
        self.assertIn('one_scripted_estop_key_bracketed_in_goal', proof(s)['failed'])
        s = scenario()                                 # cancel decision not at the fault
        s['cancels'][0]['t'] = 100.08
        self.assertIn('cancel_decision_at_the_fault_and_early', proof(s)['failed'])
        s = scenario()                                 # controller saw the cancel out of order
        s['controller'] = {'ok': False}
        self.assertIn('controller_log_order', proof(s)['failed'])
        s = scenario()                                 # feedback for the cancelled goal
        s['feedback']['g55'] = [(100.03, 0.001)]
        self.assertIn('no_feedback_for_goal', proof(s)['failed'])


class TrackingRule(unittest.TestCase):
    def test_measured_goals_pass_and_the_final_goal_is_na_without_a_value(self):
        s = scenario()
        r = rv.tracking_r1(s['goals'], s['feedback'], TOL, proof(s))
        self.assertEqual(r['verdict'], 'PASS')
        last = r['goals'][-1]
        self.assertEqual(last['verdict'], 'N/A')
        self.assertNotIn('max_abs_error_rad', last)
        self.assertEqual(sum(1 for x in r['goals'] if x['verdict'] == 'PASS'), 54)

    def test_missing_feedback_on_a_normal_goal_stays_unmeasured(self):
        s = scenario()
        del s['feedback']['g20']
        r = rv.tracking_r1(s['goals'], s['feedback'], TOL, proof(s))
        self.assertEqual(r['verdict'], 'UNMEASURED')
        self.assertEqual(r['goals'][19]['verdict'], 'UNMEASURED')

    def test_final_goal_without_a_qualifying_exception_stays_unmeasured(self):
        s = scenario()
        set_fault(s, 100.0099)
        r = rv.tracking_r1(s['goals'], s['feedback'], TOL, proof(s))
        self.assertEqual((r['verdict'], r['goals'][-1]['verdict']), ('UNMEASURED', 'UNMEASURED'))

    def test_thresholds_unchanged(self):
        s = scenario()
        s['feedback']['g7'] = [(17.2, 0.03)]
        r = rv.tracking_r1(s['goals'], s['feedback'], TOL, proof(s))
        self.assertEqual((r['verdict'], list(r['findings_above_0_02_rad'])), ('PASS', [7]))
        s['feedback']['g7'] = [(17.2, 0.051)]
        self.assertEqual(rv.tracking_r1(s['goals'], s['feedback'], TOL, proof(s))['verdict'],
                         'FAIL')


class Extraction(unittest.TestCase):
    def test_key_bracket_uses_the_previous_main_thread_record(self):
        recs = [{'mono': 1.0001, 'reached': 'estop exercise: goal in flight'},
                {'mono': 1.0005, 'status': {}},
                {'mono': 1.0016, 'key': "'x'", 'why': 'estop exercise: x'},
                {'mono': 1.0016, 'release': "'w'"}]
        xb = rv.key_bracket(recs)
        self.assertEqual((xb['t_pre'], xb['t_post'], xb['n_estop_keys']), (1.0001, 1.0016, 1))

    def test_key_bracket_without_a_preceding_reached_record_has_no_start(self):
        recs = [{'mono': 1.0001, 'reached': 'x'}, {'mono': 1.0010, 'key': "'w'", 'why': 'hold'},
                {'mono': 1.0016, 'key': "'x'", 'why': 'estop exercise: x'}]
        self.assertIsNone(rv.key_bracket(recs)['t_pre'])

    def test_esc_counts_as_an_estop_key(self):
        recs = [{'mono': 1.0, 'reached': 'r'}, {'mono': 1.1, 'key': "'x'", 'why': 'estop exercise'},
                {'mono': 1.2, 'key': repr('\x1b'), 'why': 'other'}]
        self.assertEqual(rv.key_bracket(recs)['n_estop_keys'], 2)

    def test_teleop_responses_parse_multi_line_output(self):
        recs = [{'mono': 5.0, 'teleop_out': f'[disarm] REFUSED: x\r\n[estop] ok: {MSG}\r\n'}]
        self.assertEqual(rv.teleop_responses(recs), [{'t': 5.0, 'ok': True, 'message': MSG}])

    def test_controller_order(self):
        def line(t, s):
            return f'[ign gazebo-1] [INFO] [{t:.6f}] [leg_trajectory_controller]: {s}'
        log = [line(1.0, 'Received new action goal'), line(1.1, 'Accepted new action goal'),
               line(2.0, 'Received new action goal'), line(2.1, 'Accepted new action goal'),
               line(2.2, 'Got request to cancel goal')]
        self.assertTrue(rv.controller_order(log, 2, 0.0)['ok'])
        self.assertFalse(rv.controller_order(log, 3, 0.0)['ok'])
        bad = log[:3] + [line(2.05, 'Got request to cancel goal'),
                         line(2.1, 'Accepted new action goal')]
        self.assertFalse(rv.controller_order(bad, 2, 0.0)['ok'])

    def test_scenario_is_not_mutated_between_tests(self):
        a, b = scenario(), copy.deepcopy(scenario())
        self.assertEqual(a, b)


if __name__ == '__main__':
    unittest.main()
