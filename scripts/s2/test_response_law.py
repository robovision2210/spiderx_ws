#!/usr/bin/env python3
"""Focused tests of response_law.py on synthetic runs (stdlib only; no ROS, no simulator).

    cd scripts/s2 && python3 -m unittest -v test_response_law
    S2_BAG=<bag_0.db3> (sourced toolchain) adds the 2026-10-29 real-bag regression.

The generator models the welded rig as the analyzer assumes, x[k+1] = x[k] + g (c[k] - x[k]) per
10 ms update, and records it as the stack does:
  * /joint_states at every update, with the joint order permuted per message;
  * controller_state every second update, its header stamp trailing the update by 0-8 ms;
  * action feedback every third update of a goal, stamped at the update;
  * the node status, giving each goal's seq and whether its cancel was sent.
Goals start from the measured state, as the controller's lead-in does. A cancellation holds the
state of the update before it, from the next update on.
"""
import math
import os
import random
import unittest

import response_law as rl

JOINTS = ('lf_hip', 'lf_thigh_joint', 'lf_foot_joint', 'rf_hip', 'rf_thigh_joint',
          'rf_foot_joint', 'lr_hip', 'lr_thigh_joint', 'lr_foot_joint', 'rr_hip',
          'rr_thigh_joint', 'rr_foot_joint')
T = rl.UPDATE_PERIOD_NS
T0 = 100 * rl.NS
GOALS = ((5, 65), (75, 135), (145, 205))


def make_run(g=1.0, goals=GOALS, n_updates=240, amplitude=0.1, unexcited=(), cancel_at=None,
             delay=False, permute=True, seed=7):
    """(joint_states, commands, statuses, truth) for one synthetic run."""
    rnd = random.Random(seed)
    amp = {n: 0.0 if n in unexcited else amplitude * (0.4 + 0.6 * (i + 1) / len(JOINTS)) *
           (-1) ** i for i, n in enumerate(JOINTS)}
    x = {n: 0.01 * i for i, n in enumerate(JOINTS)}
    c_prev = dict(x)
    hold = dict(x)
    xs, cs, active = [], [], []
    start = {}
    for k in range(n_updates):
        goal = next((i for i, (a, b) in enumerate(goals) if a <= k <= b), None)
        if cancel_at is not None and goal == len(goals) - 1 and k > cancel_at:
            goal = None
        if goal is not None:
            a, b = goals[goal]
            if k == a:
                start = dict(x)                      # lead-in from the measured state
            s = (1 - math.cos(math.pi * (k - a) / (b - a))) / 2
            c = {n: start[n] + amp[n] * s * (1 if goal % 2 == 0 else -1) for n in JOINTS}
            hold = dict(c)                           # a success holds the last point
        elif cancel_at is not None and k == cancel_at + 1:
            hold = dict(xs[-1])                      # the cancel holds x of the update before
            c = dict(hold)
        else:
            c = dict(hold)
        xs.append(dict(x))
        cs.append(c)
        active.append(goal)
        u = c_prev if delay else c
        x = {n: x[n] + g * (u[n] - x[n]) for n in JOINTS}
        c_prev = c
    joint_states, commands = [], []
    for k in range(n_updates):
        t = T0 + k * T
        names = list(JOINTS)
        if permute:
            rnd.shuffle(names)
        joint_states.append(rl.JointStateSample(t, 5 * rl.NS + k * 14_000_000, tuple(names),
                                                tuple(xs[k][n] for n in names)))
        err = {n: cs[k][n] - xs[k][n] for n in JOINTS}
        if k % 2 == 0:
            lag = (k * 3) % 9 * 1_000_000
            commands.append(rl.CommandSample(
                'controller_state', t - lag, 5 * rl.NS + k * 14_000_000 + 2_000_000, JOINTS,
                tuple(cs[k][n] for n in JOINTS), tuple(xs[k][n] for n in JOINTS),
                tuple(err[n] for n in JOINTS), tuple(cs[k][n] for n in JOINTS)))
        if active[k] is not None and k % 3 == 0:
            names = list(JOINTS)
            if permute:
                rnd.shuffle(names)
            commands.append(rl.CommandSample(
                'feedback', t, 5 * rl.NS + k * 14_000_000 + 3_000_000, tuple(names),
                tuple(cs[k][n] for n in names), tuple(xs[k][n] for n in names),
                tuple(err[n] for n in names), (), f'goal-{active[k] + 1}'))
    statuses = [rl.StatusSample(5 * rl.NS + a * 14_000_000, {'goal': {
        'seq': i + 1, 'fingerprint': f'fp{i + 1}', 'kind': 'shift',
        'cancel_sent': cancel_at is not None and i == len(goals) - 1}})
        for i, (a, _) in enumerate(goals)]
    return joint_states, commands, statuses, {'x': xs, 'c': cs}


def run(g=1.0, expected=1.0, required=None, **kw):
    js, cmds, st, _ = make_run(g=g, **kw)
    return rl.analyze(js, cmds, st, expected, required)


class GainRecovery(unittest.TestCase):
    def test_gain_0_1_is_recovered_and_supported(self):
        r = run(g=0.1, expected=0.1)
        p = r['plant']
        self.assertEqual('SUPPORTED', p['verdict'], p)
        self.assertLess(abs(p['pooled']['gain'] - 0.1), 1e-12)
        self.assertTrue(all(f['status'] == 'IDENTIFIED' for f in p['per_joint'].values()))
        self.assertEqual('FAIL', r['tracking']['verdict'])         # a 0.1 lag crosses 0.02 rad
        self.assertIsNotNone(r['tracking']['first_crossing'])

    def test_gain_1_0_is_recovered_and_supported(self):
        r = run(g=1.0, expected=1.0)
        self.assertEqual('SUPPORTED', r['plant']['verdict'], r['plant'])
        self.assertLess(abs(r['plant']['pooled']['gain'] - 1.0), 1e-12)

    def test_wrong_expected_gain_is_rejected(self):
        for g, expected, verdict in ((1.0, 0.1, 'CONTRADICTED'), (0.1, 1.0, 'CONTRADICTED'),
                                     (1.0 + 2e-6, 1.0, 'CONTRADICTED'),
                                     (1.0 + 5e-7, 1.0, 'SUPPORTED')):
            with self.subTest(g=g, expected=expected):
                p = run(g=g, expected=expected)['plant']
                self.assertEqual(verdict, p['verdict'], p['contradictions'])
                self.assertLess(abs(p['pooled']['gain'] - g), 1e-12)   # fitted, not assumed

    def test_fit_has_no_search_interval(self):
        for g in (0.02, 1.0, 1.9):
            self.assertLess(abs(run(g=g, expected=g)['plant']['pooled']['gain'] - g), 1e-12)


class Excitation(unittest.TestCase):
    def test_constant_joint_is_unmeasured_not_fabricated(self):
        r = run(unexcited=('rr_foot_joint',))
        f = r['plant']['per_joint']['rr_foot_joint']
        self.assertEqual('UNMEASURED', f['status'])
        self.assertIsNone(f['gain'])
        self.assertIn('max |c - x|', f['reason'])
        self.assertEqual('INSUFFICIENT_EVIDENCE', r['plant']['verdict'])
        others = [n for n in JOINTS if n != 'rr_foot_joint']
        self.assertEqual('SUPPORTED', run(unexcited=('rr_foot_joint',),
                                          required=others)['plant']['verdict'])

    def test_no_commands_is_insufficient(self):
        js, _, _, _ = make_run()
        r = rl.analyze(js, [], [], 1.0)
        self.assertEqual('INSUFFICIENT_EVIDENCE', r['plant']['verdict'])
        self.assertEqual('NO_DATA', r['tracking']['verdict'])


class Tracking(unittest.TestCase):
    def test_successful_run_without_a_crossing(self):
        r = run(g=1.0, amplitude=0.05)
        t = r['tracking']
        self.assertEqual('PASS', t['verdict'])
        self.assertIsNone(t['first_crossing'])
        self.assertEqual(0, t['samples_above_limit'])
        self.assertEqual(0, t['error_identity_mismatches'])
        self.assertEqual('SUPPORTED', r['plant']['verdict'])


class GoalsAndHolds(unittest.TestCase):
    def test_multiple_goals_and_a_cancelled_final_goal(self):
        # g = 0.1: the holds decay over many updates, so they carry identifiable pairs. With
        # g = 1 a hold is at rest after one update and its repeated states are ambiguous.
        self.assertEqual('SUPPORTED', run(g=1.0, cancel_at=170)['plant']['verdict'])
        r = run(g=0.1, expected=0.1, cancel_at=170)
        self.assertEqual('SUPPORTED', r['plant']['verdict'], r['plant']['contradictions'])
        self.assertEqual(3, len(r['goals']))
        self.assertTrue(r['goals'][2]['node']['cancel_sent'])
        self.assertFalse(r['goals'][0]['node']['cancel_sent'])
        self.assertIn('after_goal_3', r['evidence']['pairs_by_segment'])
        kinds = r['plant']['per_segment_kind']
        self.assertIn('tracking', kinds)
        self.assertIn('hold_after_cancel', kinds)
        self.assertIn('hold_after_goal', kinds)
        self.assertEqual(3, len(r['tracking']['per_goal']))

    def test_goal_ids_without_status_are_reported_not_guessed(self):
        js, cmds, _, _ = make_run()
        r = rl.analyze(js, cmds, [], 1.0)
        self.assertEqual('no node status recorded', r['goal_mapping_note'])
        self.assertIsNone(r['goals'][0]['node'])


class RecordingFaults(unittest.TestCase):
    def test_missing_updates_and_out_of_order_receipt(self):
        js, cmds, st, _ = make_run(g=1.0)
        dropped = {10, 11, 50, 120, 121, 122}
        js = [s for s in js if (s.stamp_ns - T0) // T not in dropped]
        js[3], js[4] = (rl.JointStateSample(js[3].stamp_ns, js[4].receipt_ns, js[3].names,
                                            js[3].positions),
                        rl.JointStateSample(js[4].stamp_ns, js[3].receipt_ns, js[4].names,
                                            js[4].positions))
        random.Random(3).shuffle(js)
        random.Random(4).shuffle(cmds)
        r = rl.analyze(js, cmds, st, 1.0)
        self.assertEqual('SUPPORTED', r['plant']['verdict'])
        self.assertEqual(3, r['timing']['simulation']['gaps'])        # 9-12, 49-51, 119-123
        self.assertGreater(r['evidence']['excluded'].get('next_update_missing', 0), 0)
        self.assertGreater(r['timing']['receipt']['joint_states_received_out_of_stamp_order'], 0)
        # Updates whose next update was dropped (9, 49, 119) are absent: no pair spans a gap.
        complete = make_run(g=1.0)
        full = rl.analyze(complete[0], complete[1], complete[2], 1.0)
        self.assertLess(r['evidence']['pairs'], full['evidence']['pairs'])

    def test_joint_name_permutations_change_nothing(self):
        a = run(g=1.0, permute=True)['plant']['per_joint']
        b = run(g=1.0, permute=False)['plant']['per_joint']
        self.assertEqual({n: f['gain'] for n, f in a.items()},
                         {n: f['gain'] for n, f in b.items()})

    def test_non_finite_and_conflicting_samples_are_excluded(self):
        js, cmds, st, _ = make_run(g=1.0)
        bad = js[30]
        js.append(rl.JointStateSample(bad.stamp_ns, bad.receipt_ns + 1, bad.names,
                                      (math.nan,) + bad.positions[1:]))
        js.append(rl.JointStateSample(js[40].stamp_ns, js[40].receipt_ns + 1, js[40].names,
                                      tuple(v + 1e-3 for v in js[40].positions)))
        r = rl.analyze(js, cmds, st, 1.0)
        ex = r['evidence']['excluded']
        self.assertEqual(1, ex['joint_states_non_finite'])
        self.assertEqual(1, ex['joint_states_conflicting_duplicate_stamp'])
        self.assertEqual('SUPPORTED', r['plant']['verdict'])


class WrongTimingMustNotPass(unittest.TestCase):
    def test_one_update_command_delay_is_contradicted(self):
        p = run(g=1.0, delay=True)['plant']
        self.assertEqual('CONTRADICTED', p['verdict'])
        self.assertTrue(any('residual' in c for c in p['contradictions']))

    def test_feedback_stamped_one_update_late_is_excluded(self):
        js, cmds, st, _ = make_run(g=1.0)
        shifted = [rl.CommandSample(c.source, c.stamp_ns + T, c.receipt_ns, c.names, c.command,
                                    c.measured, c.error, c.reference, c.goal_id)
                   for c in cmds if c.source == 'feedback']
        r = rl.analyze(js, shifted, st, 1.0)
        self.assertEqual('INSUFFICIENT_EVIDENCE', r['plant']['verdict'])
        self.assertEqual('UNMEASURED', r['plant']['pooled']['status'])
        # Only samples at rest survive (the shifted state is identical there); they carry no
        # excitation. Every moving sample is excluded rather than re-timed.
        excluded = r['evidence']['excluded']['feedback_without_identical_joint_state_at_its_stamp']
        self.assertGreaterEqual(excluded, len(shifted) - 2)

    def test_command_recorded_against_the_wrong_update_is_contradicted(self):
        js, cmds, st, truth = make_run(g=1.0)
        by_t = {c.stamp_ns: c for c in cmds if c.source == 'feedback'}
        wrong = []
        for t, c in by_t.items():
            k = (t - T0) // T
            if k + 1 < len(truth['c']):
                nxt = tuple(truth['c'][k + 1][n] for n in c.names)
                wrong.append(rl.CommandSample('feedback', t, c.receipt_ns, c.names, nxt,
                                              c.measured, c.error, (), c.goal_id))
        p = rl.analyze(js, wrong, st, 1.0)['plant']
        self.assertNotEqual('SUPPORTED', p['verdict'])

    def test_controller_state_stamps_outside_the_window_are_not_used(self):
        js, cmds, st, _ = make_run(g=1.0)
        far = [rl.CommandSample(c.source, c.stamp_ns - 10 * rl.WINDOW_NS, c.receipt_ns, c.names,
                                c.command, c.measured, c.error, c.reference)
               for c in cmds if c.source == 'controller_state']
        r = rl.analyze(js, far, st, 1.0)
        self.assertEqual('INSUFFICIENT_EVIDENCE', r['plant']['verdict'])
        self.assertEqual(0, r['evidence']['pairs'])


@unittest.skipUnless(os.environ.get('S2_BAG'), 'set S2_BAG to the 2026-10-29 bag_0.db3')
class OriginalBagRegression(unittest.TestCase):
    """The 2026-10-29 S2 bag: g = 0.1 and the original tracking findings."""

    @classmethod
    def setUpClass(cls):
        cls.js, cls.cmds, cls.st, cls.meta = rl.read_bag(os.environ['S2_BAG'])

    def test_bag_unchanged_by_reading(self):
        self.assertTrue(self.meta['unchanged'])

    def test_recorded_gain_0_1_is_recovered(self):
        p = rl.analyze(self.js, self.cmds, self.st, 0.1)['plant']
        self.assertEqual('SUPPORTED', p['verdict'], p['contradictions'] + p['insufficient'])
        self.assertLess(abs(p['pooled']['gain'] - 0.1), 1e-9)
        self.assertEqual('CONTRADICTED', rl.analyze(self.js, self.cmds, self.st,
                                                    1.0)['plant']['verdict'])

    def test_original_tracking_findings(self):
        t = rl.analyze(self.js, self.cmds, self.st, 0.1)['tracking']
        self.assertEqual('FAIL', t['verdict'])
        self.assertEqual(26, t['samples'])
        self.assertEqual(0.020972637967721877, t['max_abs_error_rad'])
        self.assertEqual('rr_thigh_joint', t['max_joint'])
        self.assertAlmostEqual(74.07, t['max_at_s'], places=9)
        self.assertEqual('rf_thigh_joint', t['first_crossing']['joint'])
        self.assertAlmostEqual(74.00, t['first_crossing']['s'], places=9)
        self.assertEqual(3, t['samples_above_limit'])


if __name__ == '__main__':
    unittest.main()
