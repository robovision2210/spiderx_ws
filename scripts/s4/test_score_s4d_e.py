#!/usr/bin/env python3
"""The S4d-E scorer (score_s4d_e.py) on bench runs of the S4d-E driver (the real node logic and
teleop key logic in memory; test_s4d_e_driver's bench), with synthetic bag samples, then mutated.
Each mutation must give the contracted verdict: missing evidence UNMEASURED, never PASS.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_score_s4d_e"""
import copy
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import test_s4d_e_driver as edt  # noqa: E402  (the S4d-E bench)
import score_s4d_e as se  # noqa: E402

sc = se.sc
RESTORE = os.path.join(HERE, '..', '..', 'docs', 'evidence', 'm55', 's4d_live_20261111T052817Z',
                       'restore', 'restore_verify_20261111T060321Z.txt')
RUN_INSTALL = {'install_base': '/x/install', 'gates': {g: False for g in se.s4d.GATES}}
_CACHE = {}


def bench_run(shadow):
    """(node events, driver records, bench statuses) of one complete bench run, cached."""
    if shadow in _CACHE:
        return copy.deepcopy(_CACHE[shadow])

    class T(edt.Base):
        gate = not shadow

        def runTest(self):
            pass
    t = T()
    t.setUp()
    try:
        rc, why, bench, d, events = t.run_case(rtf=0.6, shadow=shadow)
        assert rc == 0, why
        drv = t.records()
    finally:
        t.doCleanups()
        t.tearDown()
    # what the real teleop prints for each service response ("[name] ok|REFUSED: message",
    # m55_teleop.call), placed just after the node handled the request; and main()'s summary
    for e in events:
        if e['event'] in ('request', 'refused') and e['request'] in ('estop', 'stop', 'disarm'):
            ok = e['event'] == 'request'
            drv.append({'mono': e['wall'] + 0.002, 't_receipt': e['wall'] + 0.002,
                        'teleop_out': f"[{e['request']}] {'ok' if ok else 'REFUSED'}: "
                                      f"{e['message'] if ok else e['reason']}\n"})
    drv.sort(key=lambda d: d.get('mono', 0.0))
    drv.append({'mono': drv[-1]['mono'] + 1.0, 'summary': {'result': why}})
    _CACHE[shadow] = (json.loads(json.dumps(events, default=repr)), drv)
    return copy.deepcopy(_CACHE[shadow])


def synthetic_bag(ev, drv, shadow, motion=0.0, gap=None, travel=0.0, fb_err=0.002,
                  fb_after_result=False, ref_change=0.0):
    """Bag samples on the node clock (the bench clock: node and driver share it)."""
    sts = [(d['mono'], (d['status'].get('monitors') or {}).get('sim_time_s')) for d in drv
           if isinstance(d.get('status'), dict)]
    sts = [(t, s) for t, s in sts if s is not None]
    t0, t1 = sts[0][0], sts[-1][0]

    def sim_at(t):
        for (ta, sa), (tb, sb_) in zip(sts, sts[1:]):
            if ta <= t <= tb:
                return sa + (sb_ - sa) * (t - ta) / (tb - ta) if tb > ta else sa
        return sts[-1][1] if t >= t1 else sts[0][1]
    g = next(e for e in ev if e['event'] == 'goal_sent')
    res = next(e for e in ev if e['event'] == 'result')
    arm = next(e['wall'] for e in ev if e['event'] == 'transition' and e['why'] == 'arm')
    joints, poses, ref = [], [], []
    t = t0
    while t <= t1:
        if gap is None or not gap[0] <= t <= gap[1]:
            m = motion if t > res['wall'] + 1.0 else 0.0
            joints.append((t, {'j1': 0.1 + m, 'j2': -0.2}))
            dx = travel if t > arm + 0.5 else 0.0
            poses.append((t, {'xy': (dx, 0.0), 'z': 0.0545, 'tilt': 1e-6, 'yaw': 0.0}))
            r = ref_change if t > res['wall'] + 1.0 else 0.0
            ref.append((t, [0.1 + r, -0.2]))
        t = round(t + 0.02, 6)
    fbk = {}
    if not shadow:
        x = next(d for d in drv if 'key' in d and d['key'] == "'x'")
        s = [(d['feedback']['t_receipt'], fb_err) for d in drv
             if isinstance(d.get('feedback'), dict)]          # as the recorder received them
        assert s and s[0][0] < x['t_before_write'] and g['wall'] <= s[0][0]
        if fb_after_result:
            s.append((res['wall'] + 0.5, fb_err))
        fbk = {'goal-1': s}
    return {'alignment': {'ok': True, 'offset': 0.0, 'tol': 0.001}, 'feedback': fbk,
            'poses': poses, 'joints': joints,
            'sim_of': se.s4d.make_sim_of([(t, sim_at(t)) for t, _ in joints]),
            'reference': ref, 'reference_change_total': 0.0 if shadow else 0.35,
            'ctl': {'received': 0, 'accepted': 0, 'cancel_requests': 0, 'ok': False} if shadow
            else {'received': 1, 'accepted': 1, 'cancel_requests': 1, 'ok': True},
            'command_topic_present': False, 'command_topic_count': 0,
            'support': {'requested_from_recorder': True, 'recording_covers_goal_1': True,
                        'recorder_log_without_failure': True,
                        'owner_monitor_zero_throughout': True},
            'capture_before': 0, 'capture_after': 0, 'min_joint_margin': (0.3, 'j1', 0.1)}


class Base(unittest.TestCase):
    shadow = False

    def data(self):
        ev, drv = bench_run(self.shadow)
        return ev, drv, synthetic_bag(ev, drv, self.shadow)

    def score(self, ev, drv, b, restore=True):
        text = open(RESTORE).read() if restore else None
        return se.score(se.context(ev, drv), b, self.shadow, text, 'a' * 64, 'a' * 64,
                        RUN_INSTALL)

    def verdicts(self, r):
        return {k: v['verdict'] for k, v in r['E'].items()}

    def nominal(self):
        ev, drv, b = self.data()
        return ev, drv, b, self.score(ev, drv, b)


class Live(Base):
    def test_nominal_live_run_passes_every_check(self):
        ev, drv, b, r = self.nominal()
        self.assertEqual(r['overall'], 'PASS', json.dumps(self.verdicts(r)))
        self.assertTrue(all(v == 'PASS' for v in self.verdicts(r).values()), self.verdicts(r))
        e4 = r['E']['E4']
        self.assertLessEqual(e4['request_to_faulted_s_node_clock'], 0.050)
        self.assertIsNotNone(e4['key_write_to_request_latency_s_recorded_only'])

    def mutate_events(self, fn):
        ev, drv, b = self.data()
        fn(ev, drv, b)
        return self.score(ev, drv, b)

    def test_no_driver_feedback_before_x_is_unmeasured(self):
        r = self.mutate_events(lambda ev, drv, b: drv.__setitem__(slice(None), [
            d for d in drv if 'feedback' not in d]))
        self.assertEqual(r['E']['E3']['verdict'], 'UNMEASURED')
        self.assertEqual(r['overall'], 'INCONCLUSIVE')

    def test_no_bag_feedback_is_unmeasured_tracking(self):
        r = self.mutate_events(lambda ev, drv, b: b.__setitem__('feedback', {}))
        self.assertEqual(r['E']['E8']['verdict'], 'UNMEASURED')
        self.assertNotEqual(r['overall'], 'PASS')

    def test_tracking_limits_unchanged(self):
        ev, drv, _ = self.data()
        r = self.score(ev, drv, synthetic_bag(ev, drv, False, fb_err=0.06))
        self.assertEqual(r['E']['E8']['verdict'], 'FAIL')
        r = self.score(ev, drv, synthetic_bag(ev, drv, False, fb_err=0.03))
        self.assertEqual(r['E']['E8']['verdict'], 'PASS')
        self.assertTrue(r['E']['E8']['finding_above_0_02_rad'])

    def test_feedback_after_the_result_fails_the_cancellation(self):
        ev, drv, _ = self.data()
        r = self.score(ev, drv, synthetic_bag(ev, drv, False, fb_after_result=True))
        self.assertEqual(r['E']['E5']['verdict'], 'FAIL')

    def test_reference_moving_after_the_cancel_fails(self):
        ev, drv, _ = self.data()
        r = self.score(ev, drv, synthetic_bag(ev, drv, False, ref_change=0.02))
        self.assertEqual(r['E']['E5']['verdict'], 'FAIL')

    def test_a_second_goal_fails(self):
        def fn(ev, drv, b):
            g = copy.deepcopy(next(e for e in ev if e['event'] == 'goal_sent'))
            g.update(seq=2, b_from=1, b_to=2, wall=ev[-1]['wall'] + 1)
            ev.append(g)
        r = self.mutate_events(fn)
        self.assertEqual(r['E']['E1']['verdict'], 'FAIL')
        self.assertEqual(r['E']['E7']['verdict'], 'FAIL')

    def test_a_duplicate_cancel_fails(self):
        def fn(ev, drv, b):
            c = next(e for e in ev if e['event'] == 'cancel_sent')
            ev.append({**c, 'wall': c['wall'] + 0.01})
        r = self.mutate_events(fn)
        self.assertEqual(r['E']['E1']['verdict'], 'FAIL')
        self.assertEqual(r['E']['E4']['verdict'], 'FAIL')

    def test_a_wrong_fault_fails(self):
        def fn(ev, drv, b):
            next(e for e in ev if e['event'] == 'fault')['reason'] = 'tracking'
        r = self.mutate_events(fn)
        self.assertEqual(r['E']['E1']['verdict'], 'FAIL')
        self.assertEqual(r['E']['E4']['verdict'], 'FAIL')

    def shift_faulted(self, dt):
        def fn(ev, drv, b):
            req = next(e for e in ev if e['event'] == 'request' and e['request'] == 'estop')
            tr = next(e for e in ev if e['event'] == 'transition' and e['state'] == 'FAULTED')
            tr['wall'] = req['wall'] + dt
        return self.mutate_events(fn)

    def test_faulted_within_50_ms_inclusive(self):
        self.assertEqual(self.shift_faulted(0.050)['E']['E4']['verdict'], 'PASS')
        self.assertEqual(self.shift_faulted(0.0501)['E']['E4']['verdict'], 'FAIL')
        self.assertEqual(self.shift_faulted(-0.001)['E']['E4']['verdict'], 'FAIL')

    def test_missing_deadline_evidence_is_unmeasured(self):
        def fn(ev, drv, b):
            for d in drv:
                if isinstance(d.get('x_record'), dict):
                    d['x_record'].pop('sim_elapsed_since_goal1_seen_s')
        r = self.mutate_events(fn)
        self.assertEqual(r['E']['E3']['verdict'], 'UNMEASURED')

    def test_x_after_the_result_fails(self):
        def fn(ev, drv, b):
            req = next(e for e in ev if e['event'] == 'request' and e['request'] == 'estop')
            res = next(e for e in ev if e['event'] == 'result')
            res['wall'] = req['wall'] - 0.01
        r = self.mutate_events(fn)
        self.assertEqual(r['E']['E3']['verdict'], 'FAIL')

    def test_succeeded_despite_the_cancel_is_unmeasured_not_pass(self):
        def fn(ev, drv, b):
            next(e for e in ev if e['event'] == 'result')['status'] = sc.STATUS_SUCCEEDED
        r = self.mutate_events(fn)
        self.assertEqual(r['E']['E5']['verdict'], 'UNMEASURED')
        self.assertIn('despite the cancel', r['E']['E5']['note'])
        self.assertEqual(r['overall'], 'INCONCLUSIVE')

    def test_an_aborted_result_fails(self):
        def fn(ev, drv, b):
            next(e for e in ev if e['event'] == 'result')['status'] = 6     # ABORTED
        self.assertEqual(self.mutate_events(fn)['E']['E5']['verdict'], 'FAIL')

    def test_hold_motion_and_coverage(self):
        ev, drv, _ = self.data()
        r = self.score(ev, drv, synthetic_bag(ev, drv, False, motion=0.02))
        self.assertEqual(r['E']['E6']['verdict'], 'FAIL')
        res = next(e for e in ev if e['event'] == 'result')['wall']
        r = self.score(ev, drv, synthetic_bag(ev, drv, False, gap=(res + 2.0, res + 4.0)))
        self.assertEqual(r['E']['E6']['verdict'], 'UNMEASURED')

    def test_unexpected_refusals_fail_and_expected_ones_pass(self):
        def fn(ev, drv, b):
            q = next(d for d in drv if d.get('key') == "'q'")
            ev.append({'event': 'refused', 'request': 'arm', 'reason': se.RESET_REQUIRED,
                       'wall': q['t_before_write'] - 0.5, 'state': 'FAULTED', 'n': 0})
        r = self.mutate_events(fn)
        self.assertEqual(r['E']['E7']['verdict'], 'FAIL')
        self.assertEqual(r['E']['E10']['verdict'], 'FAIL')

    def test_a_disarm_refused_for_another_reason_fails(self):
        def fn(ev, drv, b):
            next(e for e in ev if e['event'] == 'refused')['reason'] = 'not_ready'
        self.assertEqual(self.mutate_events(fn)['E']['E10']['verdict'], 'FAIL')

    def test_a_reset_after_x_fails(self):
        def fn(ev, drv, b):
            q = next(d for d in drv if d.get('key') == "'q'")
            ev.append({'event': 'request', 'request': 'reset', 'message': 'reset',
                       'wall': q['t_before_write'] - 0.5, 'state': 'FAULTED', 'n': 0})
        self.assertEqual(self.mutate_events(fn)['E']['E7']['verdict'], 'FAIL')

    def test_teleop_exit_1_fails(self):
        def fn(ev, drv, b):
            next(d for d in drv if d.get('event') == 'teleop_closed')['exit_code'] = 1
        self.assertEqual(self.mutate_events(fn)['E']['E10']['verdict'], 'FAIL')

    def test_an_extra_key_fails(self):
        def fn(ev, drv, b):
            q = next(d for d in drv if d.get('key') == "'q'")
            drv.append({**q, 'key': "'r'", 'mono': q['mono'] + 0.1})
        self.assertEqual(self.mutate_events(fn)['E']['E1']['verdict'], 'FAIL')

    def test_a_wrong_mode_status_fails(self):
        def fn(ev, drv, b):
            next(d for d in drv if isinstance(d.get('status'), dict))['status']['mode'] = 'shadow'
        self.assertEqual(self.mutate_events(fn)['E']['E1']['verdict'], 'FAIL')

    def test_wrong_level_change_or_unattributed_confirmation_fails(self):
        def fn(ev, drv, b):
            next(e for e in ev if e['event'] == 'level_change')['level'] = 1
        self.assertEqual(self.mutate_events(fn)['E']['E2']['verdict'], 'FAIL')

        def fn2(ev, drv, b):
            sel = next(d for d in drv if isinstance(d.get('selection'), dict))['selection']
            c = sel['confirmations'][1]
            c['t_receipt'] = c['t_before_write'] - 0.1
        self.assertEqual(self.mutate_events(fn2)['E']['E2']['verdict'], 'FAIL')

    def test_travel_beyond_0_15_m_fails(self):
        ev, drv, _ = self.data()
        r = self.score(ev, drv, synthetic_bag(ev, drv, False, travel=0.2))
        self.assertEqual(r['E']['E11']['verdict'], 'FAIL')

    def test_restore_not_verified_fails_or_missing_is_unmeasured(self):
        ev, drv, b = self.data()
        c = se.context(ev, drv)
        r = se.score(c, b, False, 'VERDICT: NOT VERIFIED', 'a' * 64, 'a' * 64, RUN_INSTALL)
        self.assertEqual(r['E']['E12']['verdict'], 'FAIL')
        r = se.score(c, b, False, None, 'a' * 64, 'a' * 64, RUN_INSTALL)
        self.assertEqual(r['E']['E12']['verdict'], 'UNMEASURED')


class Shadow(Base):
    shadow = True

    def test_nominal_shadow_passes_as_shadow_only(self):
        ev, drv, b, r = self.nominal()
        self.assertEqual(r['overall'], 'PASS (SHADOW ONLY: not S4d-E acceptance)',
                         json.dumps(self.verdicts(r)))
        self.assertEqual(r['E']['E8']['verdict'], 'N/A')
        self.assertIn('not applicable', r['E']['E3']['feedback_precondition'])
        self.assertEqual(r['E']['E5']['checks']['modeled_shadow_cancel'], True)

    def test_controller_traffic_or_feedback_in_shadow_fails(self):
        ev, drv, b = self.data()
        b['ctl'] = {'received': 1, 'accepted': 1, 'cancel_requests': 1, 'ok': True}
        self.assertEqual(self.score(ev, drv, b)['E']['E9']['verdict'], 'FAIL')
        ev, drv, b = self.data()
        b['feedback'] = {'g': [(0.0, 0.001)]}
        self.assertEqual(self.score(ev, drv, b)['E']['E9']['verdict'], 'FAIL')
        ev, drv, b = self.data()
        b['reference_change_total'] = 0.01
        self.assertEqual(self.score(ev, drv, b)['E']['E9']['verdict'], 'FAIL')

    def test_a_dispatch_status_in_shadow_fails(self):
        ev, drv, b = self.data()
        next(d for d in drv if isinstance(d.get('status'), dict))['status']['mode'] = 'dispatch'
        self.assertEqual(self.score(ev, drv, b)['E']['E1']['verdict'], 'FAIL')

    def test_a_live_result_label_is_never_given_to_shadow(self):
        ev, drv, b, r = self.nominal()
        self.assertIn('SHADOW ONLY', r['overall'])


if __name__ == '__main__':
    unittest.main()
