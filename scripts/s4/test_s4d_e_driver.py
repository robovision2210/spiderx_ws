#!/usr/bin/env python3
"""S4d-E driver against the IN-MEMORY M5.5 node on a simulated free base (no ROS, no simulator).

Reuses the S4d bench (test_s4d_driver_live.py, unchanged), with the frozen library's own phase
durations (the level-2 0 -> 1 shift lasts 1.9 s of sim): the real LocomotionSession (the
dispatching fake with the gate True in memory, or the node's ShadowTransport with the gate False),
the real teleop key logic and status cadence, the close as m55_teleop does it. Added here only:
the driver's own action-feedback subscription, fed from the fake plant's feedback events (LIVE
benches), optionally delayed, dropped, given a second goal id, or injected in SHADOW.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_s4d_e_driver"""
import collections
import importlib.util
import json
import os
import sys
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import test_s4d_driver_live as s4dt  # noqa: E402  (the S4d bench, unchanged)

c55, loc, LIMITS = s4dt.c55, s4dt.loc, s4dt.LIMITS
MAIN_KEYS = ["'+'", "'+'", "'a'", "'w'", "'x'", "'q'"]


def load_driver():
    spec = importlib.util.spec_from_file_location('s4d_e_driver_under_test',
                                                  os.path.join(HERE, 's4d_e_driver.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def make_bench(rtf=0.6, shadow=False, scenario='ok', fail_phase=1, drift_rad_s=0.05, **kw):
    """The S4d bench with the frozen durations (s4dt.make_bench with library=), keeping the fake
    plant's scenario (s4dt.make_bench fixes it to 'ok' when it is given a library)."""
    library = s4dt.actual_library()
    b = s4dt.S4dBench(rtf, shadow=shadow, **kw)
    b.plant = s4dt.fk.FakeLocomotionTransport(library, library.boundary_positions(0, 0),
                                              scenario=scenario, fail_phase=fail_phase,
                                              drift_rad_s=drift_rad_s, use_sim=True)
    b.s = loc.LocomotionSession(b.cfg, library, loc.ShadowTransport(library.fingerprints)
                                if shadow else b.plant)
    b.world = s4dt.S4dWorld(b.s, b.plant, b.scn, rtf=rtf, graph_period_s=1.0)
    b.next_status = b.world.wall
    b.s.log = collections.deque(maxlen=None)
    b.last_state = b.s.state
    return b


def feed_feedback(bench, delay_s=0.0, drop=False, goal_id=None, second_id=False):
    """Forward the fake plant's feedback events to the driver's subscription handler (second_id:
    the first sample comes with a second, foreign goal id at the same instant). Each sample is
    stamped 1 us inside the tick: on the real clock a receipt always precedes the key write it
    enables; the bench's discrete clock would otherwise give both the same stamp."""
    orig, pending, first = bench.plant.poll, [], [True]

    def poll(wall, sim=None):
        evs = orig(wall, sim)
        due = [p for p in pending if p[0] <= wall + 1e-9]
        for p in due:
            pending.remove(p)
            bench.driver._on_feedback_sample(p[1], p[2], wall - 1e-6)
        for e in evs:
            if e[0] == 'feedback' and not drop and bench.driver is not None:
                gid = goal_id(e[1]) if goal_id else f'goal-{e[1]}'
                if second_id and first[0]:
                    first[0] = False
                    bench.driver._on_feedback_sample('foreign-goal', e[2], wall - 1e-6)
                if delay_s > 0:
                    pending.append((wall + delay_s, gid, e[2]))
                else:
                    bench.driver._on_feedback_sample(gid, e[2], wall - 1e-6)
        return evs
    bench.plant.poll = poll


def make_driver(mod, bench, out_dir, shadow=False):
    class InMemoryS4dEDriver(mod.S4dEDriver):
        def __init__(self):                                         # no ROS, no threads
            self._init_s4de()
            self._init_s3(LIMITS, shadow, {'bench': 'gates checked by main()'} if shadow else None)
            self._init_s4a()
            self._init_s4c(mod.LEVEL)
            self.out = open(os.path.join(out_dir, 'driver.jsonl'), 'w')
            self.lock = threading.Lock()
            self.status, self.status_n = None, 0
            self.seen_seqs, self.max_seq = set(), 0
            self.expected_goals, self.planned_estop = 0, False
            self.events, self.hold_key, self.fd, self.pid = [], None, None, None

        def spawn(self):
            bench.spawn_teleop()
            self.pid, self.teleop_alive = bench.n_spawn, True
            self.log(event='teleop_started', pid=self.pid)

        def _write(self, data):                                     # the pty write only
            bench.deliver(data.decode())
            return len(data)

        def _teleop_exit_status(self):
            return bench.teleop_exit

        def hold(self, k, why):
            self.key(k, why)
            bench.held, bench.held_since, self.hold_key = k, bench.world.wall, k

        def release(self, why):
            bench.held = None
            super().release(why)

        def call_estop_service(self, why):
            bench.requests.append(('estop', bench.s.request('estop', bench.world.wall)))
            self.log(event='estop_service', why=why)

        def emergency_stop(self, why):
            bench.held = None
            super().emergency_stop(why)

        def close(self):
            self.out.close()

    for m in (mod, mod.s4c, mod.s4a, mod.s3, mod.s2):
        m.time = bench                       # the inherited methods use the bench clock too
    d = InMemoryS4dEDriver()
    bench.driver = d
    return d


class Base(unittest.TestCase):
    gate = True

    def setUp(self):
        self.saved = c55.M55_LOCOMOTION_DISPATCH_ENABLED
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.gate             # tests only, in memory
        self.tmp = s4dt.tempfile.TemporaryDirectory()

    def tearDown(self):
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.saved
        self.tmp.cleanup()

    def run_case(self, rtf=0.6, shadow=False, feedback=None, patch=None, **kw):
        self.mod = load_driver()
        bench = make_bench(rtf, shadow=shadow, **kw)
        self.bench = bench
        d = make_driver(self.mod, bench, self.tmp.name, shadow=shadow)
        self.addCleanup(d.out.close)
        if not shadow and feedback is not False:
            feed_feedback(bench, **(feedback or {}))
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

    def keys(self):
        return [r['key'] for r in self.records() if 'key' in r and 't_before_write' in r]

    @staticmethod
    def of(events, name, **match):
        return [e for e in events if e.get('event') == name
                and all(e.get(k) == v for k, v in match.items())]

    def goals_sent(self, bench):
        return bench.s.transport.sent

    def assert_stopped(self, rc, why, bench, events, text):
        self.assertEqual(rc, 2, why)
        self.assertIn(text, why)
        self.assertLessEqual(self.goals_sent(bench), 1, 'a second goal was sent')
        self.assertIn('estop', [r[0] for r in bench.requests] + list(bench.s.faults))
        self.assertLessEqual(len(self.of(events, 'cancel_sent')), 1)


class Live(Base):
    def check_complete(self, rc, why, bench, d, events):
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertEqual(self.keys(), MAIN_KEYS)
        self.assertEqual(bench.plant.sent, 1)                       # exactly one goal
        g = self.of(events, 'goal_sent')
        self.assertEqual([(e['seq'], e['level'], e['b_from'], e['b_to'], e['direction'], e['kind'],
                           e['duration_s']) for e in g], [(1, 2, 0, 1, 1, 'shift', 1.9)])
        self.assertEqual([e['seq'] for e in self.of(events, 'cancel_sent')], [1])
        self.assertEqual([(e['reason']) for e in self.of(events, 'fault')], ['estop'])
        tr = self.of(events, 'transition', state='FAULTED')
        req = self.of(events, 'request', request='estop')
        self.assertEqual(len(tr), 1)
        self.assertEqual(len(req), 1)
        self.assertLessEqual(tr[0]['wall'] - req[0]['wall'], 0.050)
        res = self.of(events, 'result', seq=1)
        self.assertEqual([e['status'] for e in res], [loc.STATUS_CANCELED])
        x = d.x_record
        self.assertGreaterEqual(x['attributed_feedback_before_x'], 1)
        self.assertLessEqual(x['sim_elapsed_since_goal1_seen_s'], 0.95)
        refused = self.of(events, 'refused')
        self.assertEqual([(e['request'], e['reason']) for e in refused],
                         [('disarm', 'faulted_reset_required')] * 2)
        stops = [e for e in self.of(events, 'request', request='stop')]
        self.assertEqual([e['message'] for e in stops],
                         ['not moving (FAULTED); held until a zero command'] * 2)
        self.assertEqual(bench.s.state, 'FAULTED')
        self.assertTrue(d.hold_summary['within_A6_limits'], d.hold_summary)
        self.assertEqual(d.close_record['exit_code'], 0)
        self.assertEqual(d.after_close['state'], 'FAULTED')

    def test_complete_one_goal_one_cancel_across_rtfs(self):
        for rtf in (0.3, 0.6, 1.0):
            with self.subTest(rtf=rtf):
                self.check_complete(*self.run_case(rtf))

    def test_level_2_selected_while_disarmed_before_arm(self):
        rc, why, bench, d, events = self.run_case()
        self.assertEqual(rc, 0, why)
        self.assertEqual([c['label'] for c in d.selection['confirmations']], ['2/3', '3/3'])
        lc = self.of(events, 'level_change')
        self.assertEqual([(e['level'], e['state']) for e in lc], [(2, 'READY')])
        arm = self.of(events, 'transition', why='arm')[0]['wall']
        self.assertLess(arm, lc[0]['wall'])
        self.assertLessEqual(lc[0]['wall'], self.of(events, 'goal_sent')[0]['wall'])

    def test_no_feedback_stops_with_x_and_never_sends_a_second_goal(self):
        rc, why, bench, d, events = self.run_case(feedback=False)
        self.assert_stopped(rc, why, bench, events, 'no attributed feedback')
        self.assertEqual(bench.plant.sent, 1)

    def test_feedback_after_the_deadline_stops_before_goal_1_ends(self):
        rc, why, bench, d, events = self.run_case(rtf=1.0, feedback={'delay_s': 3.0})
        self.assert_stopped(rc, why, bench, events, 'no attributed feedback')
        self.assertEqual([e['status'] for e in self.of(events, 'result', seq=1)],
                         [loc.STATUS_CANCELED])

    def test_a_second_feedback_goal_id_stops(self):
        rc, why, bench, d, events = self.run_case(feedback={'second_id': True})
        self.assert_stopped(rc, why, bench, events, 'goal ids')

    def test_an_unplanned_fault_before_x_stops(self):
        def edit(st, n):
            if (st.get('goal') or {}).get('seq') == 1 and st['state'] != 'FAULTED':
                st['state'], st['faults'] = 'FAULTED', ['tracking']
            return st
        rc, why, bench, d, events = self.run_case(status_edit=edit)
        self.assertEqual(rc, 2, why)
        self.assertIn('unplanned FAULTED', why)
        self.assertEqual(bench.plant.sent, 1)

    def test_a_goal_other_than_level_2_0_to_1_stops(self):
        def edit(st, n):
            if st.get('goal'):
                st['goal']['level'] = 1
            return st
        rc, why, bench, d, events = self.run_case(status_edit=edit)
        self.assert_stopped(rc, why, bench, events, 'the plan is one goal')

    def test_a_mode_change_stops(self):
        def edit(st, n):
            if st.get('goal'):
                st['mode'] = 'shadow'
            return st
        rc, why, bench, d, events = self.run_case(status_edit=edit)
        self.assert_stopped(rc, why, bench, events, 'DISPATCH with the gate True')

    def test_leaving_faulted_after_the_estop_stops(self):
        def edit(st, n):
            if st['state'] == 'FAULTED' and st.get('goal') is None and n % 7 == 0:
                st['state'] = 'READY'
            return st
        rc, why, bench, d, events = self.run_case(status_edit=edit)
        self.assertEqual(rc, 2, why)
        self.assertIn('FAULTED with no goal must hold', why)

    def test_travel_beyond_0_15_m_stops(self):
        def patch(d, bench):
            bench.world.scn['fwd'] = lambda world, py: 0.2 if world.s.state != 'DISARMED' else 0.0
        rc, why, bench, d, events = self.run_case(patch=patch)
        self.assertEqual(rc, 2, why)
        self.assertIn('travel', why)

    def test_teleop_exit_1_after_q_stops(self):
        rc, why, bench, d, events = self.run_case(exit_codes={1: 256})
        self.assertEqual(rc, 2, why)
        self.assertIn('exited with 1', why)
        self.assertEqual(bench.plant.sent, 1)

    def test_a_cancel_acknowledged_late_is_not_hidden(self):
        rc, why, bench, d, events = self.run_case(scenario='cancel_late', fail_phase=1)
        res = self.of(events, 'result', seq=1)
        self.assertEqual([e['status'] for e in res], [loc.STATUS_SUCCEEDED])   # despite the cancel
        self.assertEqual(bench.plant.sent, 1)
        self.assertEqual(self.of(events, 'fault')[0]['reason'], 'estop')

    def test_interruption_sends_x_and_exits_4(self):
        def patch(d, bench):
            orig = d.hold_observation

            def interrupted():
                raise self.mod.s3.Interrupted('SIGTERM')
            d.hold_observation = interrupted
            del orig
        rc, why, bench, d, events = self.run_case(patch=patch)
        self.assertEqual(rc, 4, why)
        self.assertEqual(bench.plant.sent, 1)

    def test_the_script_wall_cap_stops(self):
        def patch(d, bench):
            d.t_start -= self.mod.SCRIPT_CAP_WALL_S
        rc, why, bench, d, events = self.run_case(patch=patch)
        self.assert_stopped(rc, why, bench, events, 'script wall cap')


class Shadow(Base):
    gate = False

    def test_shadow_complete_one_recorded_goal_its_modeled_cancel_nothing_sent(self):
        rc, why, bench, d, events = self.run_case(shadow=True)
        self.assertEqual(rc, 0, why)
        self.assertIn('shadow rehearsal', why)
        self.assertEqual(self.keys(), MAIN_KEYS)
        self.assertEqual(bench.plant.sent, 0)                       # nothing to the controller
        self.assertEqual(bench.s.transport.sent, 1)                 # one recorded shadow goal
        res = self.of(events, 'result', seq=1)
        self.assertEqual([(e['status'], e['error_string']) for e in res],
                         [(loc.STATUS_CANCELED, 'shadow cancel')])
        self.assertEqual(d.feedback, [])
        self.assertEqual(d.x_record['attributed_feedback_before_x'], 0)
        self.assertIn('not applicable', d.x_record['feedback_precondition'])
        self.assertEqual([(e['request'], e['reason']) for e in self.of(events, 'refused')],
                         [('disarm', 'faulted_reset_required')] * 2)
        s = d.summary()
        self.assertIn('not live cancel-and-hold validation', s['shadow_rehearsal']['note'])

    def test_feedback_in_shadow_stops(self):
        def patch(d, bench):
            orig = bench.tick

            def tick():
                orig()
                if bench.s.goal is not None and not d.feedback:
                    d._on_feedback_sample('stray', 0.001, bench.world.wall)
            bench.tick = tick
        rc, why, bench, d, events = self.run_case(shadow=True, patch=patch)
        self.assertEqual(rc, 2, why)
        self.assertIn('shadow rehearsal', why)
        self.assertEqual(bench.plant.sent, 0)

    def test_a_dispatch_status_in_shadow_stops(self):
        def edit(st, n):
            if n > 30:
                st['mode'] = 'dispatch'
            return st
        rc, why, bench, d, events = self.run_case(shadow=True, status_edit=edit)
        self.assertEqual(rc, 2, why)
        self.assertIn('SHADOW with the gate False', why)


class Gates(unittest.TestCase):
    def test_gate_refusal_is_exact_per_mode(self):
        mod = load_driver()
        off = {k: [False, 'f'] for k in mod.LIVE_GATES}
        live = {k: [v, 'f'] for k, v in mod.LIVE_GATES.items()}
        self.assertIsNone(mod.gate_refusal(off, shadow=True))
        self.assertIsNotNone(mod.gate_refusal(live, shadow=True))
        self.assertIsNone(mod.gate_refusal(live, shadow=False))
        self.assertIsNotNone(mod.gate_refusal(off, shadow=False))
        bad = dict(live, M61_LIVE_DISPATCH_ENABLED=[True, 'f'])
        self.assertIsNotNone(mod.gate_refusal(bad, shadow=False))
        self.assertIsNotNone(mod.gate_refusal({}, shadow=True))


if __name__ == '__main__':
    unittest.main()
