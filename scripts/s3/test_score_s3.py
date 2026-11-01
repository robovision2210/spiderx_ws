#!/usr/bin/env python3
"""Focused tests of the S3 scorer's criteria (score_s3.py), synthetic inputs only; no ROS.

Run: cd scripts/s3 && python3 -m unittest -v test_score_s3"""
import math
import unittest

import score_s3 as sc


def poses(cycles=6, step=0.020, lateral=0.0, yaw0=0.3, dyaw=0.0, z=0.0545, tilt=0.01,
          dt=1.0, per_cycle=20, overrides=None):
    """Time-sorted body poses: 20 samples per cycle along body +y; boundaries at t = 20 k."""
    out, x, y = [], 0.0, 0.0
    n = cycles * per_cycle
    for i in range(n + 1):
        k = i / per_cycle
        yaw = yaw0 + dyaw * k
        if i:
            f, lat = step / per_cycle, lateral / per_cycle
            x += math.cos(yaw) * lat - math.sin(yaw) * f
            y += math.sin(yaw) * lat + math.cos(yaw) * f
        p = {'xy': (x, y), 'z': z, 'yaw': sc.wrap(yaw), 'tilt': tilt}
        if overrides:
            p.update(overrides(i) or {})
        out.append((i * dt, p))
    return out


BOUNDS = [20.0 * k for k in range(7)]


class Cycles(unittest.TestCase):
    def test_six_cycles_within_the_bands(self):
        r = sc.assess_cycles(BOUNDS, poses(), 0.0545, 0.1)
        self.assertEqual(r['verdict'], 'PASS')
        self.assertEqual(len(r['cycles']), 6)
        self.assertAlmostEqual(r['cycles'][0]['forward_m'], 0.020)

    def test_a_cycle_outside_the_forward_band_fails(self):
        self.assertEqual(sc.assess_cycles(BOUNDS, poses(step=0.035), 0.0545, 0.1)['verdict'],
                         'FAIL')

    def test_lateral_yaw_height_and_tilt_bands(self):
        self.assertEqual(sc.assess_cycles(BOUNDS, poses(lateral=0.012), 0.0545, 0.1)['verdict'],
                         'FAIL')
        self.assertEqual(sc.assess_cycles(BOUNDS, poses(dyaw=0.06), 0.0545, 0.1)['verdict'],
                         'FAIL')
        self.assertEqual(sc.assess_cycles(BOUNDS, poses(z=0.040), 0.0545, 0.1)['verdict'],
                         'FAIL')
        spike = poses(overrides=lambda i: {'tilt': 0.11} if i == 50 else None)
        self.assertEqual(sc.assess_cycles(BOUNDS, spike, 0.0545, 0.1)['verdict'], 'FAIL')

    def test_yaw_across_pi_is_wrapped(self):
        r = sc.assess_cycles(BOUNDS, poses(yaw0=math.pi - 0.02, dyaw=0.03), 0.0545, 0.1)
        self.assertEqual(r['verdict'], 'PASS')

    def test_missing_boundary_pose_or_standing_height_is_unmeasured(self):
        gap = [(t, p) for t, p in poses() if not 39.0 < t < 41.0]
        self.assertEqual(sc.assess_cycles(BOUNDS, gap, 0.0545, 0.1)['verdict'], 'UNMEASURED')
        self.assertEqual(sc.assess_cycles(BOUNDS, poses(), None, 0.1)['verdict'], 'UNMEASURED')
        self.assertEqual(sc.assess_cycles(BOUNDS[:5], poses(), 0.0545, 0.1)['verdict'],
                         'UNMEASURED')


def run(n=55, missing=(), err=0.002, x_key=True, cancel_code=0, cancels=None, status_last=5):
    goals, fb = [], {}
    for i in range(1, n + 1):
        t = 10.0 * i
        g = {'seq': i, 'sent': t, 'accepted': True, 'response_t': t + 0.05, 'result_t': t + 2.0,
             'status': 4, 'b_from': 0, 'b_to': 1, 'direction': 1, 'level': 0}
        if i < n and i not in missing:
            fb[f'g{i}'] = [(t + 0.06 + 0.05 * k, err) for k in range(30)]
        goals.append(g)
    last = goals[-1]
    t = last['sent']
    last.update(response_t=t + 0.053, result_t=t + 0.098, status=status_last,
                cancel_response_code=cancel_code, cancel_response_t=t + 0.053)
    keys = [{'key': 'x', 't': t + 0.010, 'why': 'estop exercise: x'}] if x_key else []
    cn = cancels if cancels is not None else [{'seq': n, 't': t + 0.0113}]
    faults = [{'reason': 'estop', 't': t + 0.0113}] if x_key else \
        [{'reason': 'tracking_error', 't': t + 0.0113}]
    return goals, fb, keys, cn, faults, []


class Tracking(unittest.TestCase):
    def a3(self, *a, **k):
        return sc.assess_tracking(*run(*a, **k), 0.032, 0.05)

    def test_planned_estop_goal_is_na_and_the_rest_pass(self):
        r = self.a3()
        self.assertEqual(r['verdict'], 'PASS')
        self.assertEqual(r['goals'][-1]['verdict'], 'N/A')
        self.assertIn('A6', r['goals'][-1]['note'])
        self.assertNotIn('max_abs_error_rad', r['goals'][-1])

    def test_findings_between_0_02_and_0_05_do_not_fail(self):
        r = self.a3(err=0.03)
        self.assertEqual(r['verdict'], 'PASS')
        self.assertEqual(len(r['findings_above_0_02_rad']), 54)

    def test_above_0_05_fails(self):
        self.assertEqual(self.a3(err=0.051)['verdict'], 'FAIL')

    def test_missing_feedback_on_a_normal_goal_is_unmeasured(self):
        r = self.a3(missing=(20,))
        self.assertEqual(r['verdict'], 'UNMEASURED')
        self.assertEqual(r['goals'][-1]['verdict'], 'N/A')

    def test_unplanned_or_rejected_cancellation_is_not_na(self):
        for kw, check in (({'x_key': False}, 'scripted_estop_key_during_goal'),
                          ({'cancel_code': 1}, 'cancel_processed_by_server'),
                          ({'status_last': 4}, 'result_canceled')):
            with self.subTest(**kw):
                r = self.a3(**kw)
                self.assertEqual(r['verdict'], 'UNMEASURED')
                self.assertIn(check, r['goals'][-1]['estop_proof']['failed'])

    def test_a_second_cancel_is_not_na(self):
        goals, fb, keys, cn, faults, ref = run()
        cn = cn + [{'seq': 30, 't': 301.0}]
        r = sc.assess_tracking(goals, fb, keys, cn, faults, ref, 0.032, 0.05)
        self.assertIn('exactly_one_cancel', r['goals'][-1]['estop_proof']['failed'])

    def test_wrong_goal_count_is_not_na(self):
        r = self.a3(n=54)
        self.assertIn('final_goal_of_the_script', r['goals'][-1]['estop_proof']['failed'])


def sim_of(t):
    return t


def hold_data(dur=12.0, tilt=0.01, z=0.054, motion=0.0, gap_at=None):
    ps = [(100.0 + 0.1 * i, {'xy': (0, 0), 'z': z, 'yaw': 0.0, 'tilt': tilt})
          for i in range(int(dur * 10) + 1)]
    js = [(100.0 + 0.02 * i, {'a': 0.1 + (motion if i > 200 else 0.0), 'b': -0.2})
          for i in range(int(dur * 50) + 1)]
    if gap_at is not None:
        js = [(t, q) for t, q in js if not gap_at < t < gap_at + 0.8]
    return ps, js


class Hold(unittest.TestCase):
    def test_canceled_alone_is_not_a_hold(self):
        r = sc.assess_hold(True, True, 100.0, sim_of, [], [])
        self.assertEqual(r['verdict'], 'UNMEASURED')

    def test_observed_hold_passes(self):
        ps, js = hold_data()
        r = sc.assess_hold(True, True, 100.0, sim_of, ps, js)
        self.assertEqual(r['verdict'], 'PASS')
        self.assertGreaterEqual(r['window_sim_s'][1] - r['window_sim_s'][0], 10.0)

    def test_tilt_height_or_joint_motion_in_the_hold_fail(self):
        for kw in ({'tilt': 0.12}, {'z': 0.040}, {'motion': 0.02}):
            with self.subTest(**kw):
                ps, js = hold_data(**kw)
                self.assertEqual(sc.assess_hold(True, True, 100.0, sim_of, ps, js)['verdict'],
                                 'FAIL')

    def test_short_or_gapped_recording_is_unmeasured(self):
        ps, js = hold_data(dur=8.0)
        self.assertEqual(sc.assess_hold(True, True, 100.0, sim_of, ps, js)['verdict'],
                         'UNMEASURED')
        ps, js = hold_data(gap_at=104.0)
        self.assertEqual(sc.assess_hold(True, True, 100.0, sim_of, ps, js)['verdict'],
                         'UNMEASURED')

    def test_cancel_or_result_wrong_fails(self):
        ps, js = hold_data()
        self.assertEqual(sc.assess_hold(False, True, 100.0, sim_of, ps, js)['verdict'], 'FAIL')
        self.assertEqual(sc.assess_hold(True, False, 100.0, sim_of, ps, js)['verdict'], 'FAIL')


SUPPORT = {k: True for k in ('requested_from_recorder', 'recording_covers_goal_interval',
                             'discovery_demonstrated_after_start', 'recorder_log_without_failure',
                             'node_monitored_streams_continuous',
                             'owner_monitor_zero_throughout',
                             'captures_before_after_zero_publishers')}
COUNTS = {'goals_sent': 55, 'cancels_sent': 1, 'max_foreign': 0, 'max_publishers': 0}


class Ownership(unittest.TestCase):
    def test_absent_topic_with_support_is_zero(self):
        self.assertEqual(sc.assess_ownership(COUNTS, False, 0, SUPPORT)['verdict'], 'PASS')

    def test_absent_topic_without_support_is_unmeasured(self):
        for k in SUPPORT:
            with self.subTest(k):
                r = sc.assess_ownership(COUNTS, False, 0, {**SUPPORT, k: False})
                self.assertEqual(r['verdict'], 'UNMEASURED')
                self.assertIn(k, r['command_topic']['failed'])
        self.assertEqual(sc.assess_ownership(COUNTS, False, 0, {})['verdict'], 'UNMEASURED')

    def test_present_topic_with_a_command_fails(self):
        self.assertEqual(sc.assess_ownership(COUNTS, True, 2, None)['verdict'], 'FAIL')
        self.assertEqual(sc.assess_ownership(COUNTS, True, 0, None)['verdict'], 'PASS')

    def test_counts(self):
        for k, v in (('goals_sent', 54), ('cancels_sent', 2), ('max_foreign', 1)):
            self.assertEqual(sc.assess_ownership({**COUNTS, k: v}, False, 0, SUPPORT)['verdict'],
                             'FAIL')


REPORT = """--- 1. stop
ok: no simulation process
ok: a. all three gates False in the imported modules, from this workspace
ok: c. m6_gait_replay.py --live refused, exit 3, HARD-DISABLED
ok: c. m6_live_playback --live refused, exit 3, HARD-DISABLED
VERDICT: DISABLED AND VERIFIED
"""


class RunsRestoreLibraryOverall(unittest.TestCase):
    def test_restore(self):
        self.assertEqual(sc.assess_restore(REPORT)['verdict'], 'PASS')
        self.assertEqual(sc.assess_restore(None)['verdict'], 'UNMEASURED')
        self.assertEqual(sc.assess_restore(REPORT + 'FAIL: leftover m55_teleop\n')['verdict'],
                         'FAIL')
        self.assertEqual(sc.assess_restore(REPORT.replace('VERDICT: DISABLED AND VERIFIED',
                                                          'VERDICT: NOT VERIFIED'))['verdict'],
                         'FAIL')

    def test_library(self):
        self.assertEqual(sc.assess_library('a', 'a')['verdict'], 'PASS')
        self.assertEqual(sc.assess_library('a', 'b')['verdict'], 'FAIL')
        self.assertEqual(sc.assess_library(None, 'b')['verdict'], 'UNMEASURED')

    def test_runs(self):
        goals = [{'seq': i, 'status': 4, 'sent': 10.0 * i, 'result_t': 10.0 * i + 2}
                 for i in range(1, 56)]
        st = [(93.0, {'state': 'READY', 'boundary': 0, 'goal': None}),
              (543.0, {'state': 'READY', 'boundary': 0, 'goal': None})]
        faults = [{'reason': 'estop', 't': 550.1}]
        self.assertEqual(sc.assess_runs(goals, faults, st)['verdict'], 'PASS')
        bad = [dict(g, status=6) if g['seq'] == 30 else g for g in goals]
        self.assertEqual(sc.assess_runs(bad, faults, st)['verdict'], 'FAIL')
        self.assertEqual(sc.assess_runs(goals, faults + [{'reason': 'pose_stale', 't': 300.0}],
                                        st)['verdict'], 'FAIL')
        self.assertEqual(sc.assess_runs(goals, faults, [])['verdict'], 'UNMEASURED')

    def test_continuity(self):
        g = [{'continuity_error_rad': 0.001}, {'continuity_error_rad': 0.004}]
        r = sc.assess_continuity(g, [])
        self.assertEqual((r['verdict'], r['max_continuity_error_rad']), ('PASS', 0.004))
        self.assertEqual(sc.assess_continuity(g, [{'reason': 'continuity'}])['verdict'], 'FAIL')
        self.assertEqual(sc.assess_continuity([], [])['verdict'], 'UNMEASURED')

    def test_overall(self):
        p = {k: {'verdict': 'PASS'} for k in ('A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'A8')}
        self.assertEqual(sc.overall(p), 'PASS')
        self.assertEqual(sc.overall({**p, 'A6': {'verdict': 'UNMEASURED'}}), 'INCONCLUSIVE')
        self.assertEqual(sc.overall({**p, 'A2': {'verdict': 'FAIL'}}), 'FAIL')

    def test_slip_excludes_the_swing_foot(self):
        feet = {f: [(1.0, (0.0, 0.0)), (2.0, (0.003, 0.004))] for f in sc.FEET}
        rows = sc.slip_rows([{'seq': 2, 'leg': 'lr', 'kind': 'swing'}], {2: (0.5, 2.5)}, feet)
        self.assertEqual({r['foot'] for r in rows}, set(sc.FEET) - {'lr_foot_1'})
        self.assertAlmostEqual(rows[0]['travel_m'], 0.005)


if __name__ == '__main__':
    unittest.main()
