#!/usr/bin/env python3
"""Focused tests of the criteria-revision r1 adjudication (adjudicate_s2.py), synthetic inputs only.

Run: cd scripts/s2 && python3 -m unittest -v test_adjudicate_s2"""
import copy
import unittest

import adjudicate_s2 as ad

TOL, PERIOD = 0.032, 0.05


def run(no_feedback=(), extra_cancels=(), estop_key=True, cancel_delay=0.011, fault='estop',
        refusal_before=False, error=0.002, cancel_code=0):
    """A synthetic 59-goal script: goals 1-58 complete with feedback; goal 59 is the scripted
    estop, its cancel requested before the goal response was polled. Times are node
    CLOCK_MONOTONIC seconds, as in the event log (responses and results at the polling tick)."""
    goals, feedback = [], {}
    for i in range(1, 60):
        sent = 10.0 * i
        b_from, b_to, d = ad.EXERCISES[i - 55] if i >= 55 else (i % 9, (i + 1) % 9, 1)
        g = {'seq': i, 'sent': sent, 'accepted': True, 'response_polled_t': sent + 0.05,
             'result_polled_t': sent + 2.0, 'status': 4, 'b_from': b_from, 'b_to': b_to,
             'direction': d, 'level': 0}
        if i < 59 and i not in no_feedback:
            feedback[f'id{i}'] = [(sent + 0.03 + PERIOD * k, error) for k in range(30)]
        goals.append(g)
    last = goals[-1]
    t0 = last['sent']
    last.update(response_polled_t=t0 + 0.053, result_polled_t=t0 + 0.098,
                status=ad.STATUS_CANCELED, cancel_response_code=cancel_code,
                cancel_response_polled_t=t0 + 0.053)
    keys = [{'key': 'x', 't': t0 + 0.010, 'why': 'estop exercise: x'}] if estop_key else []
    cancels = [{'seq': 59, 't': t0 + cancel_delay}]
    cancels += [{'seq': s, 't': 10.0 * s + 1.0} for s in extra_cancels]
    if cancel_delay > 0.098:
        last['cancel_response_polled_t'] = t0 + cancel_delay + 0.03
        last['result_polled_t'] = t0 + cancel_delay + 0.05
    faults = [{'reason': fault, 't': t0 + 0.0113}]
    refusals = [{'request': 'arm', 'reason': 'not_ready', 't': 100.0}] if refusal_before else []
    refusals.append({'request': 'disarm', 'reason': 'faulted_reset_required', 't': t0 + 3.0})
    return goals, feedback, keys, cancels, faults, refusals


def p2(*a, **k):
    return ad.assess_p2(*run(*a, **k), TOL, PERIOD)


class P2Amendment(unittest.TestCase):
    def test_planned_early_estop_without_feedback_is_na(self):
        r = p2()
        self.assertEqual(r['verdict'], 'PASS')
        self.assertEqual(r['goals_na'], [59])
        row = r['goals'][-1]
        self.assertEqual(row['note'], 'tracking N/A - planned cancellation before feedback')
        self.assertNotIn('max_abs_error_rad', row)            # nothing invented for goal 59
        self.assertTrue(row['estop_proof']['checks']['early_cancel_request'])
        self.assertTrue(row['estop_proof']['checks']['cancel_processed_by_server'])
        self.assertEqual(r['goals_with_feedback'], 58)

    def test_missing_feedback_on_a_normal_goal_stays_unmeasured(self):
        r = p2(no_feedback=(30,))
        self.assertEqual(r['verdict'], 'UNMEASURED')
        self.assertEqual(r['goals'][29]['verdict'], 'UNMEASURED')
        self.assertEqual(r['goals_na'], [59])                 # only the final estop goal
        self.assertTrue(any('goal 30' in p for p in r['problems']))

    def test_unexpected_cancellation_mid_script(self):
        r = p2(no_feedback=(30,), extra_cancels=(30,))
        self.assertEqual(r['verdict'], 'UNMEASURED')
        self.assertEqual(r['goals'][29]['verdict'], 'UNMEASURED')
        self.assertEqual(r['goals'][-1]['verdict'], 'UNMEASURED')    # two cancels: no N/A
        self.assertIn('exactly_one_cancel_overall', r['goals'][-1]['estop_proof']['failed'])

    def test_final_cancel_without_the_scripted_estop_is_not_na(self):
        r = p2(estop_key=False, fault='tracking_error')
        self.assertEqual(r['verdict'], 'UNMEASURED')
        failed = r['goals'][-1]['estop_proof']['failed']
        self.assertIn('scripted_estop_key_during_goal', failed)
        self.assertIn('no_unplanned_fault_before', failed)

    def test_cancel_long_after_the_goal_response_without_feedback_is_not_na(self):
        r = p2(cancel_delay=0.6)
        self.assertEqual(r['verdict'], 'UNMEASURED')
        self.assertIn('early_cancel_request', r['goals'][-1]['estop_proof']['failed'])

    def test_cancel_rejected_by_the_server_is_not_na(self):
        r = p2(cancel_code=1)
        self.assertEqual(r['verdict'], 'UNMEASURED')
        self.assertIn('cancel_processed_by_server', r['goals'][-1]['estop_proof']['failed'])

    def test_refusal_before_the_estop_is_not_na(self):
        r = p2(refusal_before=True)
        self.assertEqual(r['verdict'], 'UNMEASURED')
        self.assertIn('no_refusal_before_result', r['goals'][-1]['estop_proof']['failed'])

    def test_tracking_limit_unchanged(self):
        self.assertEqual(ad.TRACK_MAX_RAD, 0.02)
        self.assertEqual(p2(error=0.0201)['verdict'], 'FAIL')
        self.assertEqual(p2(error=0.02)['verdict'], 'PASS')

    def test_feedback_not_attributable_to_one_goal(self):
        goals, feedback, keys, cancels, faults, refusals = run()
        feedback['id5'] = feedback['id5'] + feedback['id6']   # spans two goals' windows
        del feedback['id6']
        r = ad.assess_p2(goals, feedback, keys, cancels, faults, refusals, TOL, PERIOD)
        self.assertEqual(r['verdict'], 'UNMEASURED')
        self.assertEqual(len(r['feedback_ids_unattributed']), 1)


FROZEN_P4 = {'goals_sent': 59, 'cancels_sent': 1, 'max_foreign_clients': 0,
             'max_command_topic_publishers': 0, 'exercise_order_ok': True,
             'joint_trajectory_msgs': None, 'verdict': 'FAIL'}
GOOD = dict(
    frozen_p4=FROZEN_P4, topic_present=False, topic_count=0,
    requested_topics=['/joint_states', ad.COMMAND_TOPIC],
    recorder={'started_t': 0.0, 'stopped_t': 1000.0, 'errors': [],
              'discovered_after_start': ['/cmd_vel']},
    interval=(10.0, 600.0),
    continuity={ad.TOPIC_JS: {'max_gap_s': 0.03, 'limit_s': 0.5},
                ad.TOPIC_POSE: {'max_gap_s': 0.05, 'limit_s': 1.0}},
    owner={'samples': 1000, 'samples_without_graph': 0, 'max_publishers': 0, 'max_foreign': 0,
           'max_graph_age_s': 1.0, 'graph_stale_s': 3.0, 'max_status_gap_s': 0.6},
    events_check={'owner_refusals': []}, captures={'before': 0, 'after': 0})


def p4(**changes):
    k = copy.deepcopy(GOOD)
    for name, value in changes.items():
        if isinstance(value, dict) and isinstance(k.get(name), dict):
            k[name].update(value)
        else:
            k[name] = value
    return ad.assess_p4(**k)


class P4Amendment(unittest.TestCase):
    def test_absent_topic_with_adequate_evidence_supports_zero(self):
        r = p4()
        self.assertEqual(r['verdict'], 'PASS')
        self.assertEqual(r['command_topic']['term'], 'PASS')
        self.assertFalse(r['command_topic']['present_in_bag'])
        self.assertTrue(r['sampling_limitations'])

    def test_absent_topic_with_insufficient_evidence_stays_unmeasured(self):
        cases = {
            'requested_from_recorder': dict(requested_topics=['/joint_states']),
            'recording_covers_goal_interval': dict(recorder={'started_t': 20.0}),
            'discovery_demonstrated_after_start': dict(recorder={'discovered_after_start': []}),
            'recorder_log_without_failure': dict(recorder={'errors': ['[ERROR] write']}),
            'node_monitored_streams_continuous': dict(continuity={
                ad.TOPIC_JS: {'max_gap_s': 0.8, 'limit_s': 0.5}}),
            'owner_monitor_zero_throughout': dict(owner={'samples_without_graph': 3}),
            'owner_monitor_fresh': dict(owner={'max_graph_age_s': 4.0}),
            'captures_before_after_zero_publishers': dict(captures={'before': None}),
            'no_owner_refusal_in_node_events': dict(events_check={'owner_refusals': [{}]}),
        }
        for check, change in cases.items():
            with self.subTest(check=check):
                r = p4(**change)
                self.assertEqual(r['verdict'], 'UNMEASURED')
                self.assertIn(check, r['command_topic']['failed'])

    def test_absent_topic_is_never_zero_by_default(self):
        r = ad.assess_p4(FROZEN_P4, False, 0, [], {}, (0.0, 1.0), {}, {}, {}, {})
        self.assertEqual(r['verdict'], 'UNMEASURED')

    def test_present_topic_with_an_unexpected_command_fails(self):
        r = p4(topic_present=True, topic_count=2)
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(r['command_topic']['term'], 'FAIL')

    def test_present_topic_without_messages_is_zero(self):
        self.assertEqual(p4(topic_present=True, topic_count=0)['verdict'], 'PASS')

    def test_a_frozen_term_failure_still_fails(self):
        r = p4(frozen_p4=dict(FROZEN_P4, goals_sent=58))
        self.assertEqual(r['verdict'], 'FAIL')


class Overall(unittest.TestCase):
    FROZEN = {k: {'verdict': 'PASS'} for k in ('P1', 'P3', 'P5', 'P6', 'P7', 'P8')}

    def test_pass_needs_every_criterion_and_a_supported_plant(self):
        p2_ = {'verdict': 'PASS'}
        p4_ = {'verdict': 'PASS'}
        frozen = dict(self.FROZEN, P2={'verdict': 'UNMEASURED'}, P4={'verdict': 'FAIL'})
        self.assertEqual(ad.overall(frozen, p2_, p4_, 'SUPPORTED')['result'], 'PASS')
        self.assertEqual(ad.overall(frozen, p2_, p4_, 'INSUFFICIENT_EVIDENCE')['result'],
                         'INCONCLUSIVE')
        self.assertEqual(ad.overall(frozen, p2_, {'verdict': 'UNMEASURED'}, 'SUPPORTED')['result'],
                         'INCONCLUSIVE')
        self.assertEqual(ad.overall(frozen, {'verdict': 'FAIL'}, p4_, 'SUPPORTED')['result'],
                         'FAIL')

    def test_does_not_modify_the_frozen_record(self):
        frozen = dict(self.FROZEN, P2={'verdict': 'UNMEASURED'}, P4={'verdict': 'FAIL'})
        before = copy.deepcopy(frozen)
        ad.overall(frozen, {'verdict': 'PASS'}, {'verdict': 'PASS'}, 'SUPPORTED')
        self.assertEqual(frozen, before)

    def test_label(self):
        self.assertEqual(ad.LABEL, 'Retrospective assessment under criteria revision r1')


class Alignment(unittest.TestCase):
    def test_offset_from_statuses_received_by_both(self):
        drv = [(100.0 + i, {'n': i}) for i in range(20)]
        bag = [(5000.0 + i + (0.001 if i % 2 else -0.001), {'n': i}) for i in range(20)]
        a = ad.align(drv, bag)
        self.assertAlmostEqual(a['offset_s'], 4900.0, places=2)
        self.assertLessEqual(a['tolerance_s'], 0.0021)
        self.assertEqual(a['pairs'], 20)

    def test_too_few_pairs(self):
        self.assertIsNone(ad.align([(1.0, {'n': 1})], [(2.0, {'n': 1})]))

    def test_max_gap_includes_the_interval_edges(self):
        self.assertAlmostEqual(ad.max_gap([1.0, 1.1, 1.2], 0.0, 1.5), 1.0)
        self.assertIsNone(ad.max_gap([], 0.0, 1.0))


if __name__ == '__main__':
    unittest.main()
