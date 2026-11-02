#!/usr/bin/env python3
"""S4a driver against the IN-MEMORY M5.5 node on a simulated free base (no ROS, no simulator).

Reuses the S3 bench (scripts/s3/test_s3_driver_live.py), unchanged: the real LocomotionSession
(the dispatching fake with the gate True in memory, or the node's ShadowTransport with the gate
False), the real teleop key logic and 2 Hz status, and a free-base ground truth that follows the
node's planned displacement, so reverse moves -y. The S4a bench replaces only the raw pty write, so
the driver's own before/after write timestamps run as they would live.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_s4a_driver_live"""
import importlib.util
import json
import math
import os
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..', 's3')]
import test_s3_driver_live as s3t  # noqa: E402  (the S3 bench, unchanged)

c55, fk, loc, tp, LIMITS, Z0 = s3t.c55, s3t.fk, s3t.loc, s3t.tp, s3t.LIMITS, s3t.Z0
CREEP = -3.28e-10                    # m per planned cycle (the S3 rehearsals' standing creep)


def load_driver():
    spec = importlib.util.spec_from_file_location('s4a_driver_under_test',
                                                  os.path.join(HERE, 's4a_driver.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class S4aWorld(s3t.FreeBaseWorld):
    """The S3 free-base world with an optional forward_m(world, c) hook for the body's progress."""

    def tick(self):
        fk.FakeWorld.tick(self)
        c = self.cycles_done()
        f = (self.scn['forward_m'](self, c) if 'forward_m' in self.scn
             else c * self.scn.get('progress_m', self.stride))
        lat = self.scn.get('lateral_by_cycle', lambda k: 0.0)(c)
        yaw = self.scn.get('yaw0', 0.3) + self.scn.get('yaw_per_cycle', 0.0) * c
        df, dl = f - self.f, lat - self.lat
        self.f, self.lat, self.yaw = f, lat, yaw
        self.x += math.cos(yaw) * dl - math.sin(yaw) * df
        self.y += math.sin(yaw) * dl + math.cos(yaw) * df
        z = self.scn.get('z', lambda w, k: Z0)(self.wall, c)
        tilt = self.scn.get('tilt', lambda w, k: 0.01)(self.wall, c)
        self.s.on_body_pose(self.wall, (), {'xy': (self.x, self.y), 'z': z,
                                            'yaw': math.atan2(math.sin(yaw), math.cos(yaw)),
                                            'tilt': tilt})


def make_bench(rtf=0.2, shadow=False, **kw):
    b = s3t.Bench(rtf, shadow=shadow, **kw)
    b.world = S4aWorld(b.s, b.plant, b.scn, rtf=rtf, graph_period_s=1.0)
    b.next_status = b.world.wall
    return b


def make_driver(mod, bench, out_dir, shadow=False, write_hook=None):
    class InMemoryS4aDriver(mod.S4aDriver):
        def __init__(self):                                         # no ROS, no threads
            self._init_s3(LIMITS, shadow, {'bench': 'gates checked by main()'} if shadow else None)
            self._init_s4a()
            self.out = open(os.path.join(out_dir, 'driver.jsonl'), 'w')
            self.lock = threading.Lock()
            self.status, self.status_n = None, 0
            self.seen_seqs, self.max_seq = set(), 0
            self.expected_goals, self.planned_estop = 0, False
            self.events, self.hold_key, self.fd, self.pid = [], None, None, None

        def spawn(self):
            bench.teleop = tp.TeleopState(bench.cfg.teleop.hold_s)
            self.pid, self.teleop_alive = 1, True
            self.log(event='teleop_started')

        def _write(self, data):                                     # the pty write only
            if write_hook:
                write_hook(bench, data)
            bench.key(data.decode())
            return len(data)

        def hold(self, k, why):
            self.key(k, why)
            bench.held, bench.held_since, self.hold_key = k, bench.world.wall, k

        def release(self, why):
            bench.held, self.hold_key = None, None

        def call_estop_service(self, why):
            bench.requests.append(('estop', bench.s.request('estop', bench.world.wall)))
            self.log(event='estop_service', why=why)

        def emergency_stop(self, why):
            bench.held = None                                       # the key repeat stops
            bench.sent_at_stop = bench.plant.sent
            super().emergency_stop(why)

        def close(self):
            self.out.close()

    mod.time = bench
    mod.s3.time = bench                       # the inherited S3 and S2 methods use it too
    mod.s2.time = bench
    d = InMemoryS4aDriver()
    bench.driver = d
    return d


class Base(unittest.TestCase):
    gate = True

    def setUp(self):
        self.saved = c55.M55_LOCOMOTION_DISPATCH_ENABLED
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.gate             # tests only, in memory
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.saved
        self.tmp.cleanup()

    def run_case(self, rtf=0.2, patch=None, shadow=False, write_hook=None, **kw):
        self.mod = load_driver()
        bench = make_bench(rtf, shadow=shadow, **kw)
        d = make_driver(self.mod, bench, self.tmp.name, shadow=shadow, write_hook=write_hook)
        if patch:
            patch(d, bench)
        rc, why = self.mod.run(d)
        sent = (bench.plant.sent, bench.s.transport.sent)
        bench.sleep(30.0)                                           # nothing more is dispatched
        self.assertEqual((bench.plant.sent, bench.s.transport.sent), sent,
                         'a goal was dispatched or recorded after the run ended')
        d.out.flush()
        return rc, why, bench, d, list(bench.s.log)

    def records(self):
        with open(os.path.join(self.tmp.name, 'driver.jsonl')) as f:
            return [json.loads(ln) for ln in f]

    def keys_after(self, text):
        recs = self.records()
        i = next(i for i, r in enumerate(recs) if text in json.dumps(r))
        return [r['key'] for r in recs[i:] if 'key' in r]

    @staticmethod
    def count(events, name):
        return sum(1 for e in events if e.get('event') == name)

    def assert_stopped(self, rc, why, bench, events, text):
        self.assertEqual(rc, 2, why)
        self.assertIn(text, why)
        self.assertEqual(bench.s.state, loc.FAULTED)
        self.assertIn('estop', bench.s.faults)
        self.assertLessEqual(self.count(events, 'cancel_sent'), 1)


class Live(Base):
    def test_reverse_then_forward_return_19_goals_one_cancel(self):
        rc, why, bench, d, events = self.run_case()
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertEqual((bench.plant.sent, d.expected_goals), (19, 19))
        sent = [e for e in events if e.get('event') == 'goal_sent']
        self.assertEqual([(g['b_from'], g['b_to'], g['direction']) for g in sent[:9]],
                         [(9 - k, 8 - k, -1) for k in range(9)])           # 9->8 ... 1->0
        self.assertEqual([(g['b_from'], g['b_to'], g['direction']) for g in sent[9:18]],
                         [(k, k + 1, 1) for k in range(9)])                 # 0->1 ... 8->9
        self.assertEqual((sent[18]['b_from'], sent[18]['b_to'], sent[18]['direction']), (0, 1, 1))
        self.assertEqual({g['level'] for g in sent}, {0})
        self.assertEqual(self.count(events, 'cancel_sent'), 1)
        self.assertEqual(bench.s.faults, ['estop'])
        self.assertEqual([(c['direction'], c['cycles_count']) for c in d.cycle_log],
                         [(-1, -1), (1, 0)])
        self.assertAlmostEqual(d.cycle_log[0]['forward_m'], -0.020, delta=0.002)
        self.assertAlmostEqual(d.cycle_log[1]['forward_m'], 0.020, delta=0.002)
        h = d.hold_summary
        self.assertGreaterEqual(h['last_sim_s'] - h['from_sim_s'], 10.0 - 0.5)
        self.assertNotIn('progress_sign_bound', d.cycle_log[0])

    def test_key_writes_record_before_after_and_result(self):
        rc, why, bench, d, events = self.run_case()
        keys = [r for r in self.records() if 'key' in r]
        self.assertEqual([k['key'] for k in keys],
                         ["'a'", "'s'", "' '", "'w'", "' '", "'w'", "'x'", "'q'"])
        for k in keys:
            self.assertLessEqual(k['t_before_write'], k['t_after_write'])
            self.assertEqual(k['write_result'], 1)
            self.assertGreaterEqual(k['mono'], round(k['t_after_write'], 4))
        self.assertEqual(len(d.summary()['key_writes']), 8)

    def test_a_slow_write_is_bracketed_by_its_timestamps(self):
        def slow(bench, data):
            if data == b'x':
                bench.tick()                                        # 50 ms pass inside write()
        rc, why, bench, d, events = self.run_case(write_hook=slow)
        x = next(r for r in self.records() if r.get('key') == "'x'")
        self.assertAlmostEqual(x['t_after_write'] - x['t_before_write'], 0.05, places=6)
        fault = next(e for e in events if e.get('event') == 'fault')
        self.assertTrue(x['t_before_write'] <= fault['wall'] <= x['t_after_write'])

    def test_a_failed_estop_write_is_recorded_and_falls_back_to_the_service(self):
        def broken(bench, data):
            if data == b'x':
                raise OSError(5, 'Input/output error')
        rc, why, bench, d, events = self.run_case(write_hook=broken)
        self.assertEqual(rc, 3, why)
        xs = [r for r in self.records() if r.get('key') == "'x'"]
        self.assertTrue(xs and all('Input/output error' in r['write_result'] for r in xs))
        self.assertEqual(bench.requests[-1][0], 'estop')
        self.assertEqual(bench.s.state, loc.FAULTED)

    def test_reverse_cycle_in_the_wrong_direction_stops(self):
        rc, why, bench, d, events = self.run_case(progress_m=-0.020)   # reverse moves +y
        self.assert_stopped(rc, why, bench, events, 'reverse cycle moved 20.0 mm')
        self.assertEqual(bench.sent_at_stop, 9)

    def test_excessive_reverse_progress_stops(self):
        rc, why, bench, d, events = self.run_case(progress_m=0.045)
        self.assert_stopped(rc, why, bench, events, 'reverse cycle moved -45.0 mm')

    @staticmethod
    def return_moving(m):
        def f(w, c):
            if w.s.cycles <= -1:
                w.returned = True
            return -0.020 + m * (c + 1) if getattr(w, 'returned', False) else 0.020 * c
        return f

    def test_excessive_forward_return_stops(self):
        rc, why, bench, d, events = self.run_case(forward_m=self.return_moving(0.045))
        self.assert_stopped(rc, why, bench, events, 'forward cycle moved 45.0 mm')
        self.assertEqual(bench.sent_at_stop, 18)

    def test_forward_return_in_the_wrong_direction_stops(self):
        rc, why, bench, d, events = self.run_case(forward_m=self.return_moving(-0.005))
        self.assert_stopped(rc, why, bench, events, 'forward cycle moved -5.0 mm')

    def test_return_need_not_land_on_the_arm_pose(self):
        rc, why, bench, d, events = self.run_case(forward_m=self.return_moving(0.024))
        self.assertEqual(rc, 0, why)
        self.assertAlmostEqual(d.cycle_log[1]['forward_m'], 0.024, delta=0.002)

    @staticmethod
    def stale_from_reverse_end(k):
        """The status that first reports the reverse cycle's count, and the next k - 1, carry a
        stale pose: no fresh sample there."""
        first = {}

        def edit(st, n):
            if st.get('cycles') == -1:
                first.setdefault('n', n)
            if 'n' in first and n < first['n'] + k:
                st.setdefault('monitors', {})['body_pose_age_s'] = 5.0
            return st
        return edit

    def test_a_cycle_ending_without_a_fresh_sample_is_judged_before_the_next_key(self):
        rc, why, bench, d, events = self.run_case(status_edit=self.stale_from_reverse_end(6))
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertEqual([(c['direction'], c['cycles_count']) for c in d.cycle_log],
                         [(-1, -1), (1, 0)])
        self.assertAlmostEqual(d.cycle_log[0]['forward_m'], -0.020, delta=0.002)
        recs = self.records()
        checked = next(i for i, r in enumerate(recs)
                       if (r.get('cycle_check') or {}).get('direction') == -1)
        hold_w = next(i for i, r in enumerate(recs)
                      if r.get('why') == 'run F: one forward return cycle')
        self.assertLess(checked, hold_w)

    def test_no_fresh_sample_after_a_cycle_stops_before_the_next(self):
        rc, why, bench, d, events = self.run_case(
            status_edit=self.stale_from_reverse_end(10 ** 6))
        self.assert_stopped(rc, why, bench, events, 'cycle checked at a fresh sample')
        self.assertEqual(bench.plant.sent, 9)
        self.assertEqual(d.cycle_log, [])

    def test_yaw_across_pi_is_wrapped(self):
        rc, why, bench, d, events = self.run_case(yaw0=-math.pi + 0.01, yaw_per_cycle=0.03)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertAlmostEqual(d.cycle_log[0]['yaw_rad'], -0.03, places=6)
        self.assertAlmostEqual(d.cycle_log[1]['yaw_rad'], 0.03, places=6)

    def test_lateral_and_yaw_drift_stop(self):
        rc, why, bench, d, events = self.run_case(lateral_by_cycle=lambda c: 0.025 * abs(c))
        self.assert_stopped(rc, why, bench, events, 'drifted')
        rc, why, bench, d, events = self.run_case(yaw_per_cycle=0.12)
        self.assert_stopped(rc, why, bench, events, 'drifted')

    def test_tilt_fall_travel_joint_and_freshness_protections(self):
        cases = {
            'tilt': ({'tilt': lambda w, k: 0.15 if k < -0.3 else 0.01}, 'tilt'),
            'fall': ({'z': lambda w, k: 0.040 if k < -0.3 else Z0}, 'body z'),
        }
        for name, (kw, text) in cases.items():
            rc, why, bench, d, events = self.run_case(**kw)
            self.assert_stopped(rc, why, bench, events, text)

        def far(st, n):
            if st.get('ground_truth') and st['ground_truth'].get('delta_since_arm_xy') \
                    and st.get('state') == 'WALKING':
                st['ground_truth']['delta_since_arm_xy'] = [0.0, -0.16]
            return st
        rc, why, bench, d, events = self.run_case(status_edit=far)
        self.assert_stopped(rc, why, bench, events, 'travel')
        lo, start = LIMITS['rf_foot_joint'][0], {}

        def joint(w):
            start.setdefault('w', w + 60.0)
            return {'rf_foot_joint': lo + 0.015} if w > start['w'] else None
        rc, why, bench, d, events = self.run_case(joint=joint)
        self.assert_stopped(rc, why, bench, events, 'URDF limit')
        js = {}

        def js_off(w):
            js.setdefault('w', w + 90.0)
            return js['w'] < w < js['w'] + 2.0
        rc, why, bench, d, events = self.run_case(js_off=js_off)
        self.assertEqual(rc, 2, why)
        self.assertIn('joint_states_stale', why)

    def test_a_level_other_than_0_stops(self):
        def lvl(st, n):
            if st.get('state') == 'WALKING':
                st['level'] = 1
            return st
        rc, why, bench, d, events = self.run_case(status_edit=lvl)
        self.assert_stopped(rc, why, bench, events, 'level 1 reported')

    def test_interruption_timeout_fallback_and_driver_error(self):
        def interrupt(d, bench):
            orig = d.wait

            def w(what, pred, sim_s):
                if 'run F' in what and 'last phase' in what:
                    raise KeyboardInterrupt()
                return orig(what, pred, sim_s)
            d.wait = w
        rc, why, bench, d, events = self.run_case(patch=interrupt)
        self.assertEqual(rc, 4)
        self.assertEqual(bench.s.state, loc.FAULTED)
        rc, why, bench, d, events = self.run_case(deaf_teleop=True)
        self.assertEqual(rc, 2)
        self.assertIn('wall cap', why)

        def lose_teleop(d, bench):
            orig = d.run_cycle

            def rc_(direction, why_):
                if direction == 1:
                    bench.teleop, d.teleop_alive = None, False
                    bench.scn['z'] = lambda w, k: 0.040
                return orig(direction, why_)
            d.run_cycle = rc_
        rc, why, bench, d, events = self.run_case(patch=lose_teleop)
        self.assertEqual(rc, 2, why)
        self.assertEqual(bench.requests[-1][0], 'estop')

        def broken(d, bench):
            d.hold_observation = lambda: (_ for _ in ()).throw(RuntimeError('injected'))
        rc, why, bench, d, events = self.run_case(patch=broken)
        self.assertEqual(rc, 3)
        self.assertIn('DRIVER ERROR', why)


class Shadow(Base):
    gate = False

    def test_completes_with_wrong_signed_creep_and_records_it(self):
        rc, why, bench, d, events = self.run_case(shadow=True, progress_m=CREEP)
        self.assertEqual(rc, 0, why)
        self.assertIn('shadow rehearsal', why)
        self.assertEqual((bench.plant.sent, bench.s.transport.sent), (0, 19))
        self.assertEqual(self.count(events, 'cancel_sent'), 1)
        self.assertGreater(d.cycle_log[0]['forward_m'], 0.0)       # reverse cycle, + creep
        self.assertLess(d.cycle_log[1]['forward_m'], 0.0)          # forward cycle, - creep
        for c in d.cycle_log:
            self.assertEqual(c['progress_sign_bound'], self.mod.SHADOW_SIGN_NOTE)
        self.assertIn('not acceptance', d.summary()['shadow_rehearsal']['note'])

    def test_large_progress_still_stops_in_shadow(self):
        rc, why, bench, d, events = self.run_case(shadow=True, progress_m=0.045)
        self.assertEqual(rc, 2, why)
        self.assertIn('shadow rehearsal: |progress| at most 40', why)
        self.assertEqual(bench.plant.sent, 0)

    def test_missing_or_conflicting_mode_is_refused_before_arm(self):
        for edit, text in ((lambda st, n: {k: v for k, v in st.items() if k != 'mode'},
                            'mode None'),
                           (lambda st, n: {**st, 'dispatch_gate': True}, 'dispatch_gate True')):
            rc, why, bench, d, events = self.run_case(shadow=True, status_edit=edit)
            self.assertEqual(rc, 2, why)
            self.assertIn(text, why)
            self.assertIsNone(bench.teleop)
            self.assertFalse(any(e.get('request') == 'arm' for e in events))

    def test_mode_change_during_the_run_stops_the_script(self):
        def edit(st, n):
            if st.get('cycles') == -1:
                st['mode'] = 'dispatch'
            return st
        rc, why, bench, d, events = self.run_case(shadow=True, status_edit=edit)
        self.assertEqual(rc, 2, why)
        self.assertEqual(self.keys_after('"mode": "dispatch"'), ["'x'"])
        self.assertLess(bench.s.transport.sent, 19)
        self.assertEqual(bench.plant.sent, 0)


class LiveCreep(Base):
    def test_live_stops_on_the_same_wrong_signed_creep(self):
        rc, why, bench, d, events = self.run_case(progress_m=CREEP)
        self.assert_stopped(rc, why, bench, events, 'reverse cycle moved 0.0 mm')
        self.assertEqual(bench.sent_at_stop, 9)


class ShadowAgainstDispatch(Base):
    def test_shadow_refuses_a_dispatching_node_before_arm(self):
        self.mod = load_driver()
        bench = make_bench(0.2)                                     # dispatch mode (gate True)
        d = make_driver(self.mod, bench, self.tmp.name, shadow=True)
        rc, why = self.mod.run(d)
        self.assertEqual(rc, 2, why)
        self.assertIn("mode 'dispatch'", why)
        self.assertIsNone(bench.teleop)
        self.assertEqual(bench.plant.sent, 0)


class Constants(unittest.TestCase):
    def test_scope_constants(self):
        m = load_driver()
        self.assertEqual(m.EXPECTED_GOALS, 19)
        self.assertEqual(m.STOP_BAND_M, {-1: (-0.040, 0.0), 1: (0.0, 0.040)})
        self.assertEqual((m.LATERAL_MAX_M, m.YAW_MAX_RAD, m.SHADOW_MAX_ABS_M),
                         (0.020, 0.10, 0.040))
        s3 = m.s3
        self.assertEqual((s3.TILT_MAX_RAD, s3.Z_MIN_M, s3.TRAVEL_MAX_M), (0.10, 0.045, 0.15))
        self.assertEqual((s3.JOINT_LIMIT_STOP_RAD, s3.TRACKING_ABORT_RAD), (0.02, 0.05))
        src = open(os.path.join(HERE, 's4a_driver.py')).read().split('def script', 1)[1]
        self.assertNotIn("'+'", src)
        self.assertNotIn("'-'", src.split('def summary', 1)[0])


if __name__ == '__main__':
    unittest.main()
