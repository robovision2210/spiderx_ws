#!/usr/bin/env python3
"""S2 driver against the IN-MEMORY M5.5 node: no ROS, no simulator, no motion.

The driver's script logic runs unchanged against the real LocomotionSession with the
dispatching fake (FakeLocomotionTransport, the gate set True in memory only) or the shadow
transport, the real teleop key logic (m55_teleop.TeleopState, its heartbeat sent by FakeWorld at
10 Hz) and the node's 2 Hz status. Only the driver's I/O is replaced: keys go to the in-memory
teleop, the status comes from session.status(), and its clock is the fake world's clock.

Usage (stdlib only; no ROS needed): cd scripts/s2 && python3 -m unittest -v test_s2_driver_live
Env: S2_DRIVER=<driver to test> (default: s2_driver.py beside this file), SPIDERX_REPO.
"""
import importlib.util
import json
import os
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.environ.get('SPIDERX_REPO', '/home/user/spiderx_ws')
PKG = os.path.join(REPO, 'src', 'spiderx_controller')
sys.path[:0] = [PKG, os.path.join(PKG, 'test')]

import m55_fixtures as fx  # noqa: E402
from spiderx_controller import m55_contract as c55  # noqa: E402
from spiderx_controller import m55_fake as fk  # noqa: E402
from spiderx_controller import m55_locomotion as loc  # noqa: E402
from spiderx_controller import m55_teleop as tp  # noqa: E402

DRIVER = os.environ.get('S2_DRIVER', os.path.join(HERE, 's2_driver.py'))


def load_driver():
    spec = importlib.util.spec_from_file_location('s2_driver_under_test', DRIVER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Msg:
    def __init__(self, data):
        self.data = data


class Bench:
    """The in-memory node, teleop and clock. Doubles as the driver module's `time`."""

    def __init__(self, mode, rtf, scenario='ok', fail_phase=1, drift_rad_s=0.05):
        # synthetic phases of 1.5 s + 0.2 s lead-in = the real level-0 goal (1.7 s sim), so the
        # status/lease timing margins are the real ones (the default synthetic phase is 0.4 s)
        self.cfg = fx.config()
        lib = loc.PhaseLibrary([fx.synthetic_template(s, phase_points=15)
                                for s in (0.02, 0.04, 0.06)], 0.2)
        start = lib.boundary_positions(0, 0)
        if mode == 'dispatch':
            self.plant = fk.FakeLocomotionTransport(lib, start, scenario=scenario,
                                                    fail_phase=fail_phase,
                                                    drift_rad_s=drift_rad_s, use_sim=True)
            self.s = loc.LocomotionSession(self.cfg, lib, self.plant)
            self.world = fk.FakeWorld(self.s, self.plant, rtf=rtf, graph_period_s=1.0)
        else:
            self.plant = None
            self.s = loc.LocomotionSession(self.cfg, lib, loc.ShadowTransport(lib.fingerprints))
            self.world = fk.FakeWorld(self.s, fk.StaticPlant(start), rtf=rtf, graph_period_s=1.0)
        self.teleop = None
        self.held, self.held_since = None, None
        self.next_status = self.world.wall
        self.driver = None
        self.requests = []

    # ---- the driver's time module
    def monotonic(self):
        return self.world.wall

    def sleep(self, dt):
        end = self.world.wall + dt
        while self.world.wall < end - 1e-9:
            self.tick()

    def tick(self):
        w = self.world.wall
        if self.teleop is not None:
            if self.held is not None and w - self.held_since >= 0.5:
                self.teleop.key(tp.KEYMAP[self.held], w)          # terminal key repeat
            self.world.cmd = self.teleop.twist(w)
        else:
            self.world.cmd = None                                   # no teleop: nothing sent
        self.world.tick()
        if self.world.wall >= self.next_status - 1e-9:              # the node's 2 Hz status
            msg = Msg(json.dumps(self.s.status(self.world.wall), sort_keys=True, default=repr))
            if self.teleop is not None:
                self.teleop.on_status(json.loads(msg.data))
            self.driver._on_status(msg)
            self.next_status = self.world.wall + 0.5

    # ---- the teleop (m55_teleop.TeleopNode.on_key, in memory)
    def key(self, ch):
        if self.teleop is None:
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
    class InMemoryDriver(mod.Driver):
        def __init__(self):                                         # no ROS, no threads
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

        def kill_teleop(self):
            bench.teleop, bench.held, self.hold_key = None, None, None
            self.teleop_alive = False
            self.log(event='teleop_killed', signal='SIGKILL')

        def call_estop_service(self, why):                          # the node's service
            bench.requests.append(('estop', bench.s.request('estop', bench.world.wall)))
            self.log(event='estop_service', why=why)

        def close(self):
            self.out.close()

    mod.time = bench
    d = InMemoryDriver()
    bench.driver = d
    return d


def run_script(mod, d, levels):
    """The driver's own run() when it has one; else main()'s flow (the frozen driver)."""
    if hasattr(mod, 'run'):
        return mod.run(d, levels)
    try:
        d.script(levels)
        return 0, 'script complete'
    except mod.Abort as e:
        d.key('x', 'stop condition: emergency stop')
        return 2, f'STOP CONDITION: {e}'


class LiveBranch(unittest.TestCase):
    def setUp(self):
        self.gate = c55.M55_LOCOMOTION_DISPATCH_ENABLED
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.gate             # tests only, in memory
        self.tmp.cleanup()

    def run_case(self, mode, rtf, **kw):
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = (mode == 'dispatch')   # tests only, in memory
        mod = load_driver()
        bench = Bench(mode, rtf, **kw)
        d = make_driver(mod, bench, self.tmp.name)
        rc, why = run_script(mod, d, [0, 1, 2])
        events = list(bench.s.log)
        return rc, why, bench, d, events

    @staticmethod
    def cancels(events):
        return sum(1 for e in events if e.get('event') == 'cancel_sent')

    def check_live_complete(self, rtf):
        rc, why, bench, d, events = self.run_case('dispatch', rtf)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertEqual(bench.plant.sent, 59)
        self.assertEqual(d.expected_goals, 59)
        self.assertEqual(self.cancels(events), 1)
        self.assertEqual(bench.s.state, loc.FAULTED)
        self.assertEqual(bench.s.faults, ['estop'])
        self.assertEqual(dict(bench.s.lease.rejections), {})
        sent = [e for e in events if e.get('event') == 'goal_sent']
        # six full cycles, forward then reverse at levels 0, 1, 2
        for k, (lvl, dirn) in enumerate([(0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)]):
            run = sent[9 * k:9 * k + 9]
            self.assertEqual({(g['level'], g['direction']) for g in run}, {(lvl, dirn)}, k)
            self.assertEqual(run[-1]['b_to'], 9 if dirn > 0 else 0)
        # stop + home, lease + re-arm at boundary 1 + home, estop
        ex = sent[54:]
        self.assertEqual([(g['b_from'], g['b_to'], g['direction']) for g in ex],
                         [(0, 1, 1), (1, 0, -1), (0, 1, 1), (1, 0, -1), (0, 1, 1)])
        lease = [e for e in events if e.get('event') == 'transition' and
                 e.get('why') == loc.LEASE_EXPIRED and e['state'] == loc.DISARMED]
        self.assertEqual(len(lease), 1)
        arms = [e for e in events if e.get('request') == 'arm' and 'armed' in e.get('message', '')]
        self.assertEqual(len(arms), 2)
        self.assertIn('boundary 1', arms[1]['message'])               # re-armed at its rest
        results = [e for e in events if e.get('event') == 'result']
        self.assertEqual([e['status'] for e in results[:-1]], [loc.STATUS_SUCCEEDED] * 58)
        self.assertEqual(results[-1]['status'], loc.STATUS_CANCELED)

    def test_live_branch_completes_the_59_goal_script_at_rtf_0_59(self):
        self.check_live_complete(0.59)

    def test_live_branch_completes_at_rtf_0_20(self):
        self.check_live_complete(0.20)

    def test_live_branch_completes_at_rtf_1_0(self):
        self.check_live_complete(1.0)

    def test_above_rtf_1_6_the_lease_may_add_a_phase_and_the_driver_stops_at_59(self):
        """The kill comes <= 0.5 s (2 Hz status) after dispatch and the lease expires 0.55 s
        later: past a 1.7 s goal only when RTF > ~1.6. Then the node correctly sends one more
        phase (M5.5 section 16.1) and the driver stops on the goal count, never exceeding 59.
        S2 runs at RTF ~0.6 on the fixed base."""
        rc, why, bench, d, events = self.run_case('dispatch', 3.0)
        if rc == 0:
            self.assertEqual(bench.plant.sent, 59)
        else:
            self.assertIn('above the script', why)
            self.assertLessEqual(bench.plant.sent, 59)
            # no live teleop at that moment: the estop went to the node's service
            self.assertEqual(bench.s.state, loc.FAULTED)
            self.assertEqual(bench.s.faults, ['estop'])
            self.assertEqual(self.cancels(events), 1)
            self.assertIn(('estop', (True, bench.requests[-1][1][1])), bench.requests)

    def test_shadow_branch_plans_58_goals(self):
        rc, why, bench, d, events = self.run_case('shadow', 0.59)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertEqual(d.expected_goals, 58)
        self.assertEqual(sum(1 for e in events if e.get('event') == 'goal_sent'), 58)

    def test_tracking_error_above_0_02_stops_at_once(self):
        # goal 5 drifts at 0.015 rad/s: above 0.02 after 1.3 s, never reaching the 0.05 abort
        rc, why, bench, d, events = self.run_case('dispatch', 0.59, scenario='tracking_drift',
                                                  fail_phase=5, drift_rad_s=0.015)
        self.assertEqual(rc, 2, why)
        self.assertIn('tracking', why)
        self.assertEqual(bench.plant.sent, 5)                         # nothing after goal 5
        self.assertEqual(self.cancels(events), 1)
        self.assertEqual(bench.s.faults, ['estop'])

    def test_an_aborted_goal_stops_the_script(self):
        rc, why, bench, d, events = self.run_case('dispatch', 0.59, scenario='abort',
                                                  fail_phase=12)
        self.assertEqual(rc, 2, why)
        self.assertIn('unplanned FAULTED', why)
        self.assertEqual(bench.plant.sent, 12)

    def test_a_driver_error_presses_the_emergency_stop(self):
        mod = load_driver()
        if not hasattr(mod, 'run'):
            self.fail('the driver has no run(): an unexpected exception skips the estop')
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = True
        bench = Bench('dispatch', 0.59)
        d = make_driver(mod, bench, self.tmp.name)
        calls = {'n': 0}
        orig = d.run_cycle

        def broken(*a, **k):
            calls['n'] += 1
            if calls['n'] == 2:
                raise RuntimeError('injected driver bug')
            return orig(*a, **k)
        d.run_cycle = broken
        rc, why = mod.run(d, [0, 1, 2])
        self.assertEqual(rc, 3)
        self.assertIn('DRIVER ERROR', why)
        self.assertEqual(bench.s.state, loc.FAULTED)
        self.assertEqual(self.cancels(list(bench.s.log)), 0)          # between goals: none


if __name__ == '__main__':
    unittest.main()
