#!/usr/bin/env python3
"""S3 driver against the IN-MEMORY M5.5 node on a simulated free base: no ROS, no simulator.

As the S2 bench (scripts/s2/test_s2_driver_live.py): the real LocomotionSession with the
dispatching fake (the gate set True in memory only), the real teleop key logic and the node's
2 Hz status; only the driver's I/O is replaced. Added: a free-base ground-truth pose (the body
follows the node's planned displacement, base_link +y forward, rotated by the current yaw) with
injected tilt, height, yaw, lateral drift and progress; joint states to the driver at 20 Hz with
injected values; duplicate status publications and a stalled simulation clock; the real URDF
joint limits (spiderx_legs.yaml). The synthetic library has small joint amplitudes (<= 0.16 rad),
inside every limit.

Usage (stdlib only): cd scripts/s3 && python3 -m unittest -v test_s3_driver_live"""
import importlib.util
import json
import math
import os
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.environ.get('SPIDERX_REPO', '/home/user/spiderx_ws')
PKG = os.path.join(REPO, 'src', 'spiderx_controller')
sys.path[:0] = [PKG, os.path.join(PKG, 'test'), os.path.join(HERE, '..', 's2')]

import m55_fixtures as fx  # noqa: E402
from spiderx_controller import joint_safety as js  # noqa: E402
from spiderx_controller import m55_contract as c55  # noqa: E402
from spiderx_controller import m55_fake as fk  # noqa: E402
from spiderx_controller import m55_locomotion as loc  # noqa: E402
from spiderx_controller import m55_teleop as tp  # noqa: E402

LIMITS, ORDER = js.load_limits(os.path.join(PKG, 'config'))
Z0 = 0.0545                                     # settled base height of the M2 bench (gain 0.1)


def load_driver():
    spec = importlib.util.spec_from_file_location('s3_driver_under_test',
                                                  os.path.join(HERE, 's3_driver.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Msg:
    def __init__(self, data):
        self.data = data


class TransientTrackingTransport(fk.FakeLocomotionTransport):
    """The dispatching fake with a TRANSIENT tracking error on goal `fail_phase`: it rises to
    peak_rad at mid-goal and returns to zero at the end. (The shared fake's tracking_drift keeps
    the offset, so the next goal's continuity check faults: a finding could never pass.)"""

    def __init__(self, *a, peak_rad=0.03, **k):
        super().__init__(*a, **k)
        self.peak_rad = peak_rad

    def _advance(self, wall):
        a = self._active
        if a is None or wall < a['t_accept'] or a['cancelled'] or a['seq'] != self.fail_phase:
            return super()._advance(wall)
        te, d = wall - a['t_accept'], a['goal'].duration_s
        err = self.peak_rad * max(0.0, 1.0 - abs(2.0 * min(te, d) / d - 1.0))
        self.q = [r - err for r in self._reference(a, wall)]
        self._events.append(('feedback', a['seq'], err))
        if te >= d:
            self._events.append(('result', a['seq'], loc.STATUS_SUCCEEDED, 0, ''))
            self._active = None


class FreeBaseWorld(fk.FakeWorld):
    """FakeWorld with a free-base ground-truth pose driven by the node's planned displacement."""

    def __init__(self, session, plant, scn, **kw):
        super().__init__(session, plant, pose=False, **kw)
        self.scn = scn
        self.x, self.y = 1.0, -2.0
        self.f = self.lat = 0.0
        self.yaw = scn.get('yaw0', 0.3)
        self.stride = 0.02

    def cycles_done(self):
        return self.s.planned_xy[1] / self.stride   # planned body +y progress, in cycles

    def tick(self):
        super().tick()
        c = self.cycles_done()
        f = c * self.scn.get('progress_m', self.stride)
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


class Bench:
    """The in-memory node, teleop, free base and clock; doubles as the driver module's `time`."""

    def __init__(self, rtf=0.2, scenario='ok', fail_phase=1, drift_rad_s=0.05, peak_rad=None,
                 **scn):
        self.cfg = fx.config()
        lib = loc.PhaseLibrary([fx.synthetic_template(s, phase_points=15, amp=0.05)
                                for s in (0.02, 0.04, 0.06)], 0.2)
        if peak_rad is not None:
            self.plant = TransientTrackingTransport(lib, lib.boundary_positions(0, 0),
                                                    fail_phase=fail_phase, peak_rad=peak_rad,
                                                    use_sim=True)
        else:
            self.plant = fk.FakeLocomotionTransport(lib, lib.boundary_positions(0, 0),
                                                    scenario=scenario, fail_phase=fail_phase,
                                                    drift_rad_s=drift_rad_s, use_sim=True)
        self.s = loc.LocomotionSession(self.cfg, lib, self.plant)
        self.world = FreeBaseWorld(self.s, self.plant, scn, rtf=rtf, graph_period_s=1.0)
        self.scn = scn
        self.teleop = None
        self.held, self.held_since = None, None
        self.next_status = self.world.wall
        self.driver = None
        self.requests = []
        self.deaf = scn.get('deaf_teleop', False)
        self.sent_at_stop = None
        self.n_status = 0

    def monotonic(self):
        return self.world.wall

    def sleep(self, dt):
        end = self.world.wall + dt
        while self.world.wall < end - 1e-9:
            self.tick()

    def tick(self):
        w = self.world.wall
        if self.teleop is not None:
            if self.held is not None and w - self.held_since >= 0.5 and not self.deaf:
                self.teleop.key(tp.KEYMAP[self.held], w)
            self.world.cmd = self.teleop.twist(w)
        else:
            self.world.cmd = None
        stall = self.scn.get('stall', lambda w: False)(w)
        self.world.sim_advancing = not stall
        self.world.tick()
        q = self.plant.joint_positions()
        names, pos = list(q), list(q.values())
        inject = self.scn.get('joint', lambda w: None)(self.world.wall)
        if inject:
            pos = [inject.get(n, p) for n, p in zip(names, pos)]
        self.driver._on_joint_state(names, pos, self.world.sim)
        if self.world.wall >= self.next_status - 1e-9:
            msg = Msg(json.dumps(self.s.status(self.world.wall), sort_keys=True, default=repr))
            if self.teleop is not None:
                self.teleop.on_status(json.loads(msg.data))
            self.n_status += 1
            edit = self.scn.get('status_edit')
            if edit:                                    # what the driver receives, this sample
                msg = Msg(json.dumps(edit(json.loads(msg.data), self.n_status), sort_keys=True))
            self.driver._on_status(msg)
            if self.scn.get('duplicate', lambda n: False)(self.n_status):
                self.driver._on_status(Msg(msg.data))           # the same publication again
            self.next_status = self.world.wall + 0.5

    def key(self, ch):
        if self.teleop is None or (self.deaf and ch == 'w'):
            return
        action = tp.KEYMAP[ch]
        service, _ = self.teleop.key(action, self.world.wall)
        if action == tp.QUIT:
            for name in (tp.STOP, tp.DISARM):
                self.requests.append((name, self.s.request(name, self.world.wall)))
            self.teleop = None
        elif service:
            self.requests.append((service, self.s.request(service, self.world.wall)))


def make_driver(mod, bench, out_dir):
    class InMemoryS3Driver(mod.S3Driver):
        def __init__(self):                                         # no ROS, no threads
            self._init_s3(LIMITS)
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

        def key(self, k, why):
            bench.key(k)
            self.log(key=repr(k), why=why)

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
    mod.s2.time = bench                       # the inherited S2 methods (wait, ...) use it too
    d = InMemoryS3Driver()
    bench.driver = d
    return d


class S3Live(unittest.TestCase):
    def setUp(self):
        self.gate = c55.M55_LOCOMOTION_DISPATCH_ENABLED
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = True                   # tests only, in memory
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.gate
        self.tmp.cleanup()

    def run_case(self, rtf=0.2, patch=None, **kw):
        self.mod = load_driver()
        bench = Bench(rtf, **kw)
        d = make_driver(self.mod, bench, self.tmp.name)
        if patch:
            patch(d, bench)
        rc, why = self.mod.run(d)
        sent = bench.plant.sent
        bench.sleep(30.0)                                           # nothing more is dispatched
        self.assertEqual(bench.plant.sent, sent, 'a goal was dispatched after the run ended')
        return rc, why, bench, d, list(bench.s.log)

    def tilted_statuses_seen(self):
        with open(os.path.join(self.tmp.name, 'driver.jsonl')) as f:
            recs = [json.loads(ln) for ln in f if '"status"' in ln]
        return sum(1 for r in recs if 'status' in r and
                   ((r['status'].get('ground_truth') or {}).get('tilt') or 0) > 0.10)

    @staticmethod
    def count(events, name):
        return sum(1 for e in events if e.get('event') == name)

    def assert_stopped(self, rc, why, bench, events, text):
        self.assertEqual(rc, 2, why)
        self.assertIn(text, why)
        self.assertEqual(bench.s.state, loc.FAULTED)
        self.assertIn('estop', bench.s.faults)
        self.assertLessEqual(self.count(events, 'cancel_sent'), 1)

    # ------------------------------------------------------------------ the full script
    def check_complete(self, rtf):
        rc, why, bench, d, events = self.run_case(rtf)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertEqual(bench.plant.sent, 55)
        self.assertEqual(d.expected_goals, 55)
        self.assertEqual(self.count(events, 'cancel_sent'), 1)
        self.assertEqual(bench.s.faults, ['estop'])
        sent = [e for e in events if e.get('event') == 'goal_sent']
        self.assertEqual({(g['level'], g['direction']) for g in sent}, {(0, 1)})   # no reverse
        self.assertEqual([g['b_to'] for g in sent[:54]], list(range(1, 10)) * 6)
        self.assertEqual((sent[54]['b_from'], sent[54]['b_to']), (0, 1))
        self.assertFalse(any(e.get('request') in ('faster', 'slower') for e in events))
        results = [e for e in events if e.get('event') == 'result']
        self.assertEqual([e['status'] for e in results[:-1]], [loc.STATUS_SUCCEEDED] * 54)
        self.assertEqual(results[-1]['status'], loc.STATUS_CANCELED)
        self.assertEqual(len(d.cycle_log), 6)
        for c in d.cycle_log:
            self.assertAlmostEqual(c['forward_m'], 0.020, delta=0.002)
            self.assertLess(abs(c['lateral_m']), 0.001)
        self.assertIsNotNone(d.settle_state.done)
        self.assertAlmostEqual(d.settle_state.done['z_mean_m'], Z0, places=6)
        h = d.hold_summary
        self.assertGreaterEqual(h['last_sim_s'] - h['from_sim_s'], 10.0 - 0.5)
        self.assertTrue(h['within_A6_limits'], h)
        self.assertEqual(d.findings, {})

    def test_complete_script_55_goals_one_cancel_at_rtf_0_2(self):
        self.check_complete(0.2)

    def test_complete_script_at_rtf_0_6(self):
        self.check_complete(0.6)

    # ------------------------------------------------------------------ pose stops
    @staticmethod
    def one_tilted_status(mark):
        def edit(st, n):
            if st.get('cycles') == 2 and mark.get('n') in (None, n):
                mark['n'] = n
                st['ground_truth']['tilt'] = 0.15
            return st
        return edit

    def test_single_tilt_spike_does_not_stop(self):
        mark = {}
        rc, why, bench, d, events = self.run_case(status_edit=self.one_tilted_status(mark))
        self.assertEqual(rc, 0, why)
        self.assertEqual(self.tilted_statuses_seen(), 1)

    def test_persistent_tilt_stops_after_two_fresh_samples(self):
        rc, why, bench, d, events = self.run_case(tilt=lambda w, k: 0.15 if k > 2.5 else 0.01)
        self.assert_stopped(rc, why, bench, events, 'tilt')
        self.assertLess(bench.sent_at_stop, 55)

    def test_duplicate_publication_does_not_complete_the_debounce(self):
        mark = {}
        rc, why, bench, d, events = self.run_case(status_edit=self.one_tilted_status(mark),
                                                  duplicate=lambda n: n == mark.get('n'))
        self.assertEqual(rc, 0, why)
        self.assertEqual(self.tilted_statuses_seen(), 2)            # the sample and its copy

    def test_two_tilted_fresh_samples_stop(self):
        mark = {}

        def edit(st, n):
            if st.get('cycles') == 2 and (mark.get('n') is None or n <= mark['n'] + 1):
                mark.setdefault('n', n)
                st['ground_truth']['tilt'] = 0.15
            return st
        rc, why, bench, d, events = self.run_case(status_edit=edit)
        self.assert_stopped(rc, why, bench, events, 'tilt')

    def test_stalled_simulation_samples_are_not_fresh(self):
        # 1.0 s of stalled sim time with the tilt exceeded: the repeated sim time is not counted
        win = {'t0': None}

        def stall(w):
            return win['t0'] is not None and w - win['t0'] < 1.0

        def tilt(w, k):
            if 2.5 < k and win['t0'] is None:
                win['t0'] = w + 0.3
            return 0.15 if stall(w) else 0.01
        rc, why, bench, d, events = self.run_case(tilt=tilt, stall=stall)
        self.assertEqual(rc, 0, why)
        self.assertGreaterEqual(self.tilted_statuses_seen(), 2)     # exceeded, but not fresh

    def test_fall_stops(self):
        rc, why, bench, d, events = self.run_case(z=lambda w, k: 0.040 if k > 3.2 else Z0)
        self.assert_stopped(rc, why, bench, events, 'body z')

    def test_travel_stop_can_end_a_run_whose_cycles_are_within_the_bands(self):
        # 28 mm per cycle: every cycle inside the 10-30 mm band, but 0.15 m is reached in cycle 6
        rc, why, bench, d, events = self.run_case(progress_m=0.028)
        self.assert_stopped(rc, why, bench, events, 'travel')
        self.assertTrue(all(0.010 <= c['forward_m'] <= 0.030 for c in d.cycle_log))
        self.assertEqual(len(d.cycle_log), 5)

    def test_cycle_progress_outside_0_40_mm_stops(self):
        rc, why, bench, d, events = self.run_case(progress_m=0.045)
        self.assert_stopped(rc, why, bench, events, 'along body +y')

    def test_cycle_lateral_drift_stops(self):
        rc, why, bench, d, events = self.run_case(
            lateral_by_cycle=lambda k: 0.0 if k < 2 else 0.025 * (k - 2) if k < 3 else 0.025)
        self.assert_stopped(rc, why, bench, events, 'drifted')

    def test_cycle_yaw_drift_stops(self):
        rc, why, bench, d, events = self.run_case(yaw_per_cycle=0.12)
        self.assert_stopped(rc, why, bench, events, 'drifted')

    def test_yaw_across_pi_is_wrapped(self):
        rc, why, bench, d, events = self.run_case(yaw0=math.pi - 0.02, yaw_per_cycle=0.03)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertTrue(all(abs(c['yaw_rad'] - 0.03) < 1e-6 for c in d.cycle_log))

    def test_joint_near_its_urdf_limit_stops(self):
        lo = LIMITS['rf_foot_joint'][0]
        start = {}

        def joint(w):
            start.setdefault('w', w + 120.0)
            return {'rf_foot_joint': lo + 0.015} if w > start['w'] else None
        rc, why, bench, d, events = self.run_case(joint=joint)
        self.assert_stopped(rc, why, bench, events, 'URDF limit')

    # ------------------------------------------------------------------ tracking
    def test_tracking_above_0_02_is_a_finding_not_a_stop(self):
        rc, why, bench, d, events = self.run_case(fail_phase=5, peak_rad=0.03)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertIn(5, d.findings)
        self.assertGreater(d.findings[5], 0.02)
        self.assertLessEqual(d.findings[5], 0.05)

    def test_tracking_above_0_05_aborts(self):
        rc, why, bench, d, events = self.run_case(scenario='tracking_drift', fail_phase=5,
                                                  drift_rad_s=0.08)
        self.assertEqual(rc, 2, why)
        self.assertTrue('tracking' in why or 'unplanned FAULTED' in why, why)
        self.assertEqual(bench.plant.sent, 5)

    # ------------------------------------------------------------------ driver failures
    def test_estop_service_fallback_without_a_live_teleop(self):
        def patch(d, bench):
            orig = d.run_cycles

            def lose_teleop_then_fall(n, why):
                if n == 5:
                    bench.teleop, d.teleop_alive = None, False      # the teleop died
                    bench.scn['z'] = lambda w, k: 0.040
                return orig(n, why)
            d.run_cycles = lose_teleop_then_fall
        rc, why, bench, d, events = self.run_case(patch=patch)
        self.assertEqual(rc, 2, why)
        self.assertEqual(bench.requests[-1][0], 'estop')
        self.assertTrue(bench.requests[-1][1][0], bench.requests[-1])
        self.assertEqual(bench.s.state, loc.FAULTED)

    def test_driver_error_presses_the_estop(self):
        def patch(d, bench):
            orig = d.run_cycles

            def broken(n, why):
                if n == 5:
                    raise RuntimeError('injected driver bug')
                return orig(n, why)
            d.run_cycles = broken
        rc, why, bench, d, events = self.run_case(patch=patch)
        self.assertEqual(rc, 3)
        self.assertIn('DRIVER ERROR', why)
        self.assertEqual(bench.s.state, loc.FAULTED)
        self.assertEqual(bench.plant.sent, 9)

    def test_interruption_presses_the_estop(self):
        def patch(d, bench):
            orig = d.wait

            def interrupted(what, pred, sim_s):
                if 'run C' in what and 'last phase' in what:
                    bench.sleep(20.0)
                    raise KeyboardInterrupt()
                return orig(what, pred, sim_s)
            d.wait = interrupted
        rc, why, bench, d, events = self.run_case(patch=patch)
        self.assertEqual(rc, 4)
        self.assertIn('INTERRUPTED', why)
        self.assertEqual(bench.s.state, loc.FAULTED)
        self.assertLessEqual(self.count(events, 'cancel_sent'), 1)

    def test_timeout_when_nothing_moves(self):
        rc, why, bench, d, events = self.run_case(deaf_teleop=True)
        self.assertEqual(rc, 2, why)
        self.assertIn('wall cap', why)
        self.assertEqual(bench.plant.sent, 0)

    def test_settle_failure_never_arms(self):
        rc, why, bench, d, events = self.run_case(z=lambda w, k: 0.040)
        self.assertEqual(rc, 2, why)
        self.assertIn('settled before arm', why)
        self.assertEqual(bench.plant.sent, 0)
        self.assertFalse(any(e.get('request') == 'arm' for e in events))


class Helpers(unittest.TestCase):
    def setUp(self):
        self.mod = load_driver()

    def test_body_frame_forward_is_plus_y(self):
        for yaw in (0.0, 0.7, -2.0, math.pi):
            p0 = {'xy': (0.0, 0.0), 'yaw': yaw}
            p1 = {'xy': (-math.sin(yaw) * 0.02, math.cos(yaw) * 0.02), 'yaw': yaw}
            f, lat, dyaw = self.mod.body_delta(p0, p1)
            self.assertAlmostEqual(f, 0.02)
            self.assertAlmostEqual(lat, 0.0)
            self.assertAlmostEqual(dyaw, 0.0)

    def test_yaw_wrap(self):
        self.assertAlmostEqual(self.mod.wrap(2 * math.pi + 0.1), 0.1)
        self.assertAlmostEqual(self.mod.body_delta({'xy': (0, 0), 'yaw': math.pi - 0.01},
                                                   {'xy': (0, 0), 'yaw': -math.pi + 0.01})[2],
                               0.02)

    def test_fresh_pose_rejects_stale_duplicate_and_missing(self):
        gt = {'xy': [0, 0], 'z': 0.05, 'tilt': 0.0, 'yaw': 0.0}
        st = {'ground_truth': gt, 'monitors': {'sim_time_s': 10.0, 'body_pose_age_s': 0.1}}
        self.assertIsNotNone(self.mod.fresh_pose(st, 9.9))
        self.assertIsNone(self.mod.fresh_pose(st, 10.0))                     # duplicate
        self.assertIsNone(self.mod.fresh_pose({**st, 'monitors': {'sim_time_s': 11.0,
                                                                  'body_pose_age_s': 1.5}}, 10.0))
        self.assertIsNone(self.mod.fresh_pose({**st, 'ground_truth': None}, None))
        self.assertIsNone(self.mod.fresh_pose({**st, 'ground_truth': {**gt, 'tilt': None}}, None))

    def test_scope_constants(self):
        m = self.mod
        self.assertEqual(m.EXPECTED_GOALS, 55)
        self.assertEqual((m.TILT_MAX_RAD, m.Z_MIN_M, m.TRAVEL_MAX_M), (0.10, 0.045, 0.15))
        self.assertEqual((m.PROGRESS_MIN_M, m.PROGRESS_MAX_M, m.LATERAL_MAX_M, m.YAW_MAX_RAD),
                         (0.0, 0.040, 0.020, 0.10))
        self.assertEqual((m.JOINT_LIMIT_STOP_RAD, m.TRACKING_ABORT_RAD, m.TRACKING_FINDING_RAD),
                         (0.02, 0.05, 0.02))
        self.assertEqual((m.DEBOUNCE_SAMPLES, m.HOLD_SIM_S, m.LEVEL), (2, 10.0, 0))
        src = open(os.path.join(HERE, 's3_driver.py')).read()
        self.assertNotIn("'s'", src.split('def script', 1)[1].split('def summary', 1)[0])
        self.assertNotIn("'+'", src)


if __name__ == '__main__':
    unittest.main()
