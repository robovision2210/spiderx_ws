#!/usr/bin/env python3
"""S4c driver against the IN-MEMORY M5.5 node on a simulated free base (no ROS, no simulator).

Reuses the S4a bench (test_s4a_driver_live.py, unchanged), which reuses the S3 bench: the real
LocomotionSession (the dispatching fake with the gate True in memory, or the node's ShadowTransport
with the gate False), the real teleop key logic and 2 Hz status, and a free-base ground truth that
follows the node's planned displacement. The S4c bench adds what S4c reads and does:
- the teleop's own messages (its level confirmations), delivered as pty output;
- the teleop's close as m55_teleop does it: 'q' calls stop and disarm, its shutdown sequence calls
  them again, and the process exits 0;
- keys the teleop never receives, or receives as another key (deaf or swapped keys).
Every case runs at level 1 and at level 2.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_s4c_driver_live"""
import importlib.util
import json
import os
import re
import subprocess
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import test_s4a_driver_live as s4at  # noqa: E402  (the S4a bench, unchanged)

s3t = s4at.s3t
c55, loc, tp, LIMITS, Z0 = s3t.c55, s3t.loc, s3t.tp, s3t.LIMITS, s3t.Z0
LEVELS = (1, 2)
STRIDE = {0: 0.02, 1: 0.04, 2: 0.06}


def load_driver():
    spec = importlib.util.spec_from_file_location('s4c_driver_under_test',
                                                  os.path.join(HERE, 's4c_driver.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class S4cBench(s3t.Bench):
    """The S3 bench; the teleop's messages reach the driver as pty output chunks with their
    receipt times, and 'q' closes as m55_teleop does. Output options: confirm_delay_s (received
    after the write returned), split_at/split_gap_s (one line in two chunks), duplicate_output,
    mangle (altered text); write_s: the pty write takes that long, so output can arrive inside
    the write bracket."""

    def __init__(self, *a, ignore=(), swap=None, write_s=0.0, confirm_delay_s=0.0, split_at=None,
                 split_gap_s=0.0, duplicate_output=False, mangle=None, **kw):
        super().__init__(*a, **kw)
        self.ignore, self.swap = set(ignore), dict(swap or {})
        self.teleop_exit = None
        self.write_s, self.confirm_delay_s = write_s, confirm_delay_s
        self.split_at, self.split_gap_s = split_at, split_gap_s
        self.duplicate_output, self.mangle = duplicate_output, mangle
        self.pending_out = []                    # [(due wall, text)]

    def say(self, text):
        text = self.mangle(text) if self.mangle else text
        parts = [(self.confirm_delay_s, text)]
        if self.split_at is not None:
            n = self.split_at
            parts = [(self.confirm_delay_s, text[:n]),
                     (self.confirm_delay_s + self.split_gap_s, text[n:])]
        for _ in range(2 if self.duplicate_output else 1):
            for delay, part in parts:
                if delay <= 0:
                    self.driver._on_teleop_out(part, self.world.wall)
                else:
                    self.pending_out.append((self.world.wall + delay, part))

    def tick(self):
        due = [p for p in self.pending_out if p[0] <= self.world.wall + 1e-9]
        self.pending_out = [p for p in self.pending_out if p not in due]
        for _, part in due:                      # received now, in order
            self.driver._on_teleop_out(part, self.world.wall)
        super().tick()

    def key(self, ch):
        ch = self.swap.get(ch, ch)
        if self.teleop is None or ch in self.ignore or (self.deaf and ch == 'w'):
            return
        action = tp.KEYMAP[ch]
        service, message = self.teleop.key(action, self.world.wall)
        if message and self.driver is not None:
            self.say(message + '\n')
        if action == tp.QUIT:              # handle(q): stop, disarm; then _shutdown_sequence
            for name in (tp.STOP, tp.DISARM, tp.STOP, tp.DISARM):
                self.requests.append((name, self.s.request(name, self.world.wall)))
            self.teleop, self.teleop_exit = None, 0
        elif service:
            self.requests.append((service, self.s.request(service, self.world.wall)))


def make_bench(rtf=0.2, shadow=False, **kw):
    b = S4cBench(rtf, shadow=shadow, **kw)
    b.world = s4at.S4aWorld(b.s, b.plant, b.scn, rtf=rtf, graph_period_s=1.0)
    b.next_status = b.world.wall
    return b


def make_driver(mod, bench, out_dir, level, shadow=False, write_hook=None):
    class InMemoryS4cDriver(mod.S4cDriver):
        def __init__(self):                                         # no ROS, no threads
            self._init_s3(LIMITS, shadow, {'bench': 'gates checked by main()'} if shadow else None)
            self._init_s4a()
            self._init_s4c(level)
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
            if bench.write_s:
                bench.sleep(bench.write_s / 2)
            bench.key(data.decode())
            if bench.write_s:
                bench.sleep(bench.write_s / 2)
            return len(data)

        def _teleop_exit_status(self):                              # the wait status only
            return bench.teleop_exit

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
    mod.s4a.time = bench
    d = InMemoryS4cDriver()
    bench.driver = d
    return d


def progress(level, fwd_m, rev_m):
    """forward_m hook: the forward cycle moves fwd_m (times the synthetic library's planned
    fraction, 1 - 1e-7), then the reverse return moves rev_m from where the forward cycle ended."""
    def f(world, c):
        k = c * STRIDE[0] / STRIDE[level]                # planned progress, in level-L cycles
        peak = getattr(world, 'k_peak', 0.0)
        if k >= peak:                                    # forward (or at rest after it)
            world.k_peak = k
            return k * fwd_m
        return peak * fwd_m + (peak - k) / peak * rev_m
    return f


class Base(unittest.TestCase):
    gate = True

    def setUp(self):
        self.saved = c55.M55_LOCOMOTION_DISPATCH_ENABLED
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.gate             # tests only, in memory
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.saved
        self.tmp.cleanup()

    def run_case(self, level, rtf=0.6, patch=None, shadow=False, write_hook=None, **kw):
        self.mod = load_driver()
        bench = make_bench(rtf, shadow=shadow, **kw)
        d = make_driver(self.mod, bench, self.tmp.name, level, shadow=shadow,
                        write_hook=write_hook)
        self.addCleanup(d.out.close)
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

    def assert_stopped(self, rc, why, bench, events, text, cancels=(0, 1)):
        self.assertEqual(rc, 2, why)
        self.assertIn(text, why)
        self.assertIn('estop', [r[0] for r in bench.requests] + bench.s.faults)
        self.assertIn(len(self.of(events, 'cancel_sent')), cancels)


class Live(Base):
    def check_complete(self, level, rc, why, bench, d, events):
        m = self.mod
        self.assertEqual((rc, why), (0, 'script complete'))
        sent = self.of(events, 'goal_sent')
        self.assertEqual([(g['b_from'], g['b_to'], g['direction']) for g in sent],
                         list(m.PLANNED))
        self.assertEqual({g['level'] for g in sent}, {level})
        self.assertEqual((bench.plant.sent, d.expected_goals, len(sent)), (18, 18, 18))
        self.assertEqual(self.of(events, 'cancel_sent'), [])
        self.assertEqual(self.of(events, 'fault'), [])
        self.assertEqual(self.keys(), m.scripted_keys(level))
        lc = self.of(events, 'level_change')
        self.assertEqual([(e['previous'], e['level']) for e in lc], [(0, level)])
        self.assertLess(lc[0]['n'], sent[0]['n'])
        self.assertEqual([r[0] for r in bench.requests],
                         ['arm', 'stop', 'stop', 'stop', 'disarm', 'stop', 'disarm'])
        tr = [(e['previous'], e['state'], e['why']) for e in self.of(events, 'transition')]
        self.assertEqual(tr[-1], ('READY', 'DISARMED', 'disarm'))
        self.assertEqual([t[2] for t in tr].count('stopped'), 2)
        self.assertEqual(bench.s.state, loc.DISARMED)
        s = d.summary()
        self.assertEqual(s['close']['exit_code'], 0)
        self.assertEqual(s['close_observation']['state_after'], 'DISARMED')
        self.assertTrue(s['close_observation']['within_A6_limits'])
        self.assertEqual([c['label'] for c in s['selection']['confirmations']],
                         ['2/3', '3/3'][:level])
        self.assertEqual(s['selection']['teleop_label'], f'{level + 1}/3')
        self.assertAlmostEqual(s['selection']['stride_m'], STRIDE[level])
        cyc = [(c['direction'], c['cycles_count']) for c in d.cycle_log]
        self.assertEqual(cyc, [(1, 1), (-1, 0)])
        self.assertAlmostEqual(d.cycle_log[0]['forward_m'], STRIDE[level], delta=1e-6)
        self.assertAlmostEqual(d.cycle_log[1]['forward_m'], -STRIDE[level], delta=1e-6)
        self.assertAlmostEqual(s['net_return_recorded_only']['forward_m'], 0.0, delta=1e-6)
        recs = self.records()
        plus = [i for i, r in enumerate(recs) if r.get('key') == "'+'"]
        arm = next(i for i, r in enumerate(recs) if r.get('key') == "'a'")
        self.assertEqual(len(plus), level)
        self.assertTrue(all(i < arm and recs[i]['t_after_write'] <= recs[arm]['t_before_write']
                            for i in plus))
        sel = next(r['selection'] for r in recs if 'selection' in r)
        self.assertEqual(sel['level'], level)
        for c in sel['confirmations']:
            self.assertGreaterEqual(c['t_receipt'], c['t_before_write'])
        self.assertTrue(all('t_receipt' in r for r in recs if 'teleop_out' in r))
        q = next(r for r in recs if r.get('key') == "'q'")
        self.assertEqual([r for r in recs if r.get('mono', 0) > q['mono'] and 'key' in r], [])
        checks = [r for r in recs if 'cycle_check' in r]
        s_key = next(r for r in recs if r.get('key') == "'s'")
        self.assertLess(checks[0]['mono'], s_key['t_before_write'])
        self.assertLess(checks[1]['mono'], q['t_before_write'])
        for r in recs:
            if 'key' in r and 't_before_write' in r:
                self.assertLessEqual(r['t_before_write'], r['t_after_write'])
                self.assertEqual(r['write_result'], 1)

    def test_complete_script_18_goals_zero_cancels_both_levels(self):
        for level in LEVELS:
            for rtf in (0.2, 0.6):
                with self.subTest(level=level, rtf=rtf):
                    rc, why, bench, d, events = self.run_case(level, rtf=rtf)
                    self.check_complete(level, rc, why, bench, d, events)

    def test_duplicate_publications_do_not_change_the_script(self):
        for level in LEVELS:
            with self.subTest(level=level):
                rc, why, bench, d, events = self.run_case(level, duplicate=lambda n: True)
                self.check_complete(level, rc, why, bench, d, events)

    def test_a_delayed_cycle_check_is_waited_for_before_the_next_key(self):
        for level in LEVELS:
            stale_sims = []

            def stale(st, n):                    # the first 6 statuses after the count changes
                if st.get('cycles') == 1 and len(stale_sims) < 6:
                    stale_sims.append(st['monitors']['sim_time_s'])
                    st['monitors']['body_pose_age_s'] = 5.0                 # not fresh
                return st
            with self.subTest(level=level):
                rc, why, bench, d, events = self.run_case(level, status_edit=stale)
                self.check_complete(level, rc, why, bench, d, events)
                self.assertEqual(len(stale_sims), 6)
                self.assertGreater(d.cycle_log[0]['sim'], max(stale_sims))

    def test_no_fresh_sample_after_a_cycle_stops_before_the_next_key(self):
        def stale(st, n):
            if st.get('cycles') == 1:
                st['monitors']['body_pose_age_s'] = 5.0
            return st
        for level in LEVELS:
            with self.subTest(level=level):
                rc, why, bench, d, events = self.run_case(level, status_edit=stale)
                self.assert_stopped(rc, why, bench, events, 'cycle checked at a fresh sample')
                self.assertNotIn("'s'", self.keys())
                self.assertEqual(bench.plant.sent, 9)

    def test_band_edges_of_the_live_stops(self):
        um = 1e-6
        for level in LEVELS:
            outer = {1: 0.060, 2: 0.080}[level]
            s = STRIDE[level]
            cases = [  # (forward, reverse, completes, text)
                (outer - um, -s, True, None),
                (outer + um, -s, False, 'forward cycle moved'),
                (-um, -s, False, 'forward cycle moved'),
                (s, -outer + um, True, None),
                (s, -outer - um, False, 'reverse cycle moved'),
                (s, um, False, 'reverse cycle moved'),
            ]
            for fwd, rev, ok, text in cases:
                with self.subTest(level=level, forward=fwd, reverse=rev):
                    rc, why, bench, d, events = self.run_case(
                        level, yaw0=0.0, forward_m=progress(level, fwd, rev))
                    if ok:
                        self.assertEqual(rc, 0, why)
                        self.assertEqual(bench.plant.sent, 18)
                    else:
                        self.assert_stopped(rc, why, bench, events, text)
                        self.assertEqual(bench.plant.sent, 9 if text.startswith('forward')
                                         else 18)

    def test_lateral_and_yaw_drift_stop_unchanged(self):
        for level in LEVELS:
            with self.subTest(level=level, drift='lateral'):
                rc, why, bench, d, events = self.run_case(
                    level, lateral_by_cycle=lambda k: 0.021 * k / (STRIDE[level] / 0.02))
                self.assert_stopped(rc, why, bench, events, 'drifted')
            with self.subTest(level=level, drift='yaw'):
                rc, why, bench, d, events = self.run_case(
                    level, yaw_per_cycle=0.11 / (STRIDE[level] / 0.02))
                self.assert_stopped(rc, why, bench, events, 'drifted')

    def test_tilt_fall_travel_and_joint_protections_unchanged(self):
        for level in LEVELS:
            cases = {'tilt': ({'tilt': lambda w, k: 0.15 if k > 0.3 else 0.01}, 'tilt'),
                     'fall': ({'z': lambda w, k: 0.040 if k > 0.3 else Z0}, 'body z')}
            for name, (kw, text) in cases.items():
                with self.subTest(level=level, case=name):
                    rc, why, bench, d, events = self.run_case(level, **kw)
                    self.assert_stopped(rc, why, bench, events, text)

            def far(st, n):
                if st.get('ground_truth') and st['ground_truth'].get('delta_since_arm_xy') \
                        and st.get('state') == 'WALKING':
                    st['ground_truth']['delta_since_arm_xy'] = [0.0, 0.151]
                return st
            with self.subTest(level=level, case='travel'):
                rc, why, bench, d, events = self.run_case(level, status_edit=far)
                self.assert_stopped(rc, why, bench, events, 'travel')
            lo, start = LIMITS['rf_foot_joint'][0], {}

            def joint(w):
                start.setdefault('w', w + 60.0)
                return {'rf_foot_joint': lo + 0.015} if w > start['w'] else None
            with self.subTest(level=level, case='joint'):
                rc, why, bench, d, events = self.run_case(level, joint=joint)
                self.assert_stopped(rc, why, bench, events, 'URDF limit')

    def test_goal_beyond_or_off_the_plan_stops(self):
        # space never reaches the teleop and the key is never released: the node keeps walking
        # forward, so goal 10 is 0->1 instead of 9->8 (an extra forward dispatch)
        def no_release(d, bench):
            d.release = lambda why: d.log(note='test: key not released')
        for level in LEVELS:
            with self.subTest(level=level, case='extra forward dispatch'):
                rc, why, bench, d, events = self.run_case(level, ignore={' '}, patch=no_release)
                self.assert_stopped(rc, why, bench, events, 'goal 10 is (0, 1, 1), planned '
                                    '(9, 8, -1)')
                self.assertEqual(bench.plant.sent, 10)
            with self.subTest(level=level, case="'s' read as 'w': wrong direction"):
                rc, why, bench, d, events = self.run_case(level, swap={'s': 'w'})
                self.assert_stopped(rc, why, bench, events, 'goal 10 is (0, 1, 1)')

    def test_a_wrong_level_is_stopped_at_the_first_status(self):
        def back_to_level_0(bench, data):                   # the teleop forgets the selection
            if data == b'w':
                bench.teleop.level = 0
        for level in LEVELS:
            with self.subTest(level=level):
                rc, why, bench, d, events = self.run_case(level, write_hook=back_to_level_0)
                self.assert_stopped(rc, why, bench, events, 'level')
                self.assertLessEqual(bench.plant.sent, 1)
                self.assertEqual({g['level'] for g in self.of(events, 'goal_sent')}, {0})
        for level in LEVELS:                                # a goal at another level, reported
            def other(st, n, level=level):
                if st.get('goal'):
                    st['goal']['level'] = 3 - level
                return st
            with self.subTest(level=level, case='reported'):
                rc, why, bench, d, events = self.run_case(level, status_edit=other)
                self.assert_stopped(rc, why, bench, events, f'goal 1 at level {3 - level}')

    def test_selection_is_confirmed_by_the_teleop_before_arm(self):
        def unknown(bench, data):                           # the teleop has no levels yet
            if data == b'+':
                bench.teleop.levels = None

        def extra(bench, data):                             # one press moves two levels
            if data == b'+':
                bench.teleop.level += 1
        for level in LEVELS:
            for hook, text in ((unknown, 'no speed levels yet'),
                               (extra, 'teleop confirmed speed level')):
                with self.subTest(level=level, case=text):
                    rc, why, bench, d, events = self.run_case(level, write_hook=hook)
                    self.assertEqual(rc, 2, why)
                    self.assertIn(text, why)
                    self.assertNotIn("'a'", self.keys())
                    self.assertEqual((bench.plant.sent, self.of(events, 'goal_sent')), (0, []))
            with self.subTest(level=level, case='no confirmation'):
                rc, why, bench, d, events = self.run_case(level, ignore={'+'})
                self.assertEqual(rc, 2, why)
                self.assertIn('teleop confirms speed level 2/3', why)
                self.assertNotIn("'a'", self.keys())

    def selection(self):
        return next(r['selection'] for r in self.records() if 'selection' in r)

    def test_confirmation_received_before_or_after_the_write_returned(self):
        for level in LEVELS:
            for case, kw in (('during the write bracket', {'write_s': 0.1}),
                             ('after the write returned', {'confirm_delay_s': 0.3})):
                with self.subTest(level=level, case=case):
                    rc, why, bench, d, events = self.run_case(level, **kw)
                    self.check_complete(level, rc, why, bench, d, events)
                    for c in self.selection()['confirmations']:
                        inside = c['t_before_write'] < c['t_receipt'] < c['t_after_write']
                        after = c['t_receipt'] > c['t_after_write']
                        self.assertTrue(inside if 'during' in case else after, c)
                        self.assertEqual(c['received_before_the_write_returned'], inside)

    def test_a_confirmation_split_across_output_chunks(self):
        for level in LEVELS:
            for gap in (0.0, 0.2):
                with self.subTest(level=level, gap=gap):
                    rc, why, bench, d, events = self.run_case(level, split_at=19,
                                                              split_gap_s=gap,
                                                              confirm_delay_s=gap)
                    self.check_complete(level, rc, why, bench, d, events)
                    chunks = [r['teleop_out'] for r in self.records() if 'teleop_out' in r]
                    heads = [c for c in chunks if 'speed level' in c and 'mm/s' not in c]
                    tails = [c for c in chunks if 'mm/s' in c and 'speed level' not in c]
                    self.assertEqual((len(heads), len(tails)), (level, level))   # split lines
                    for c in self.selection()['confirmations']:       # the completing chunk
                        self.assertGreaterEqual(c['t_receipt'], c['t_before_write'] + 2 * gap)

    def test_two_presses_have_distinct_ordered_confirmations(self):
        rc, why, bench, d, events = self.run_case(2, write_s=0.1)
        self.check_complete(2, rc, why, bench, d, events)
        c1, c2 = self.selection()['confirmations']
        self.assertEqual((c1['label'], c2['label']), ('2/3', '3/3'))
        self.assertNotEqual(c1['speed_mm_s'], c2['speed_mm_s'])
        self.assertLess(c1['t_receipt'], c2['t_before_write'])
        self.assertLess(c2['t_before_write'], c2['t_receipt'])

    def test_stale_duplicate_or_wrong_confirmations_stop_before_arm(self):
        def stale(bench, data):                  # a confirmation received before the write
            if data == b'+':
                bench.driver._on_teleop_out('speed level 2/3: 1.90 mm/s\n',
                                            bench.world.wall - 0.001)

        def wrong_speed(text):
            return re.sub(r': [0-9.]+ mm/s', ': 9.99 mm/s', text)
        for level in LEVELS:
            for case, kw, text in (
                    ('stale', {'ignore': {'+'}, 'write_hook': stale}, 'stale level confirmation'),
                    ('duplicate', {'duplicate_output': True}, 'level confirmations for one key'),
                    ('wrong speed', {'mangle': wrong_speed}, 'expected 2/3')):
                with self.subTest(level=level, case=case):
                    rc, why, bench, d, events = self.run_case(level, **kw)
                    self.assertEqual(rc, 2, why)
                    self.assertIn(text, why)
                    self.assertNotIn("'a'", self.keys())
                    self.assertEqual((bench.plant.sent, self.of(events, 'goal_sent')), (0, []))

    def test_level_keys_are_refused_unless_disarmed_before_arm(self):
        for level in LEVELS:
            for state, goal, armed in (('READY', None, True), ('WALKING', {'seq': 1}, True),
                                       ('DISARMED', {'seq': 1}, False)):
                with self.subTest(level=level, state=state, goal=goal):
                    mod = load_driver()
                    bench = make_bench(0.6)
                    d = make_driver(mod, bench, self.tmp.name, level)
                    d.status = {'state': state, 'goal': goal, 'levels_m_s': [1, 2, 3],
                                'strides_m': [0.02, 0.04, 0.06]}
                    d.armed = armed
                    with self.assertRaises(mod.s2.Abort):
                        d.select_level()
                    d.out.close()
                    self.assertEqual(self.keys(), [])

    def test_wrong_node_levels_or_strides_are_refused_before_any_key(self):
        for level in LEVELS:
            for levels, strides in (([1, 2], [0.02, 0.04]), ([1, 2, 3], [0.02, 0.05, 0.07])):
                with self.subTest(level=level, strides=strides):
                    mod = load_driver()
                    d = make_driver(mod, make_bench(0.6), self.tmp.name, level)
                    d.status = {'state': 'DISARMED', 'goal': None, 'levels_m_s': levels,
                                'strides_m': strides}
                    with self.assertRaises(mod.s2.Abort):
                        d.select_level()
                    d.out.close()
                    self.assertEqual(self.keys(), [])

    def test_close_needs_ready_after_both_cycle_checks(self):
        for level in LEVELS:
            mod = load_driver()
            d = make_driver(mod, make_bench(0.6), self.tmp.name, level)
            d.status = {'state': 'READY', 'boundary': 0, 'goal': None}
            d.cycle_log = [{}]                                  # one cycle checked, not two
            with self.subTest(level=level):
                with self.assertRaises(mod.s2.Abort):
                    d.close_teleop()
            d.out.close()
            self.assertEqual(self.keys(), [])

    def test_no_motion_after_the_close(self):
        for level in LEVELS:
            seen = {'n': 0}

            def goal_after_close(st, n, level=level):        # the 3rd DISARMED status on
                if st.get('state') == 'DISARMED' and st.get('phases_completed') == 18:
                    seen['n'] += 1
                    if seen['n'] >= 3:
                        st['goal'] = {'seq': 18, 'b_from': 1, 'b_to': 0, 'direction': -1,
                                      'level': level}
                return st
            with self.subTest(level=level, case='a goal after the close'):
                rc, why, bench, d, events = self.run_case(level, status_edit=goal_after_close)
                self.assertEqual(rc, 2, why)
                self.assertIn('after the close', why)
                self.assertEqual(bench.requests[-1][0], 'estop')    # teleop gone: the service
                self.assertEqual(bench.plant.sent, 18)
            with self.subTest(level=level, case='the teleop never exits'):
                def hang(d, bench):
                    d._teleop_exit_status = lambda: None
                rc, why, bench, d, events = self.run_case(level, patch=hang)
                self.assertEqual(rc, 2, why)
                self.assertIn('still running', why)
                self.assertEqual(bench.plant.sent, 18)
            with self.subTest(level=level, case='the teleop exits non-zero'):
                def bad_exit(d, bench):
                    d._teleop_exit_status = lambda: 256 if bench.teleop is None else None
                rc, why, bench, d, events = self.run_case(level, patch=bad_exit)
                self.assertEqual(rc, 2, why)
                self.assertIn('exited with 1', why)

    def test_faulted_interruption_timeout_fallback_and_driver_error(self):
        for level in LEVELS:
            def interrupt(d, bench):
                orig = d.wait

                def w(what, pred, sim_s):            # goal 18 is in flight: x cancels it
                    if 'run R' in what and 'READY at boundary 0' in what:
                        raise KeyboardInterrupt()
                    return orig(what, pred, sim_s)
                d.wait = w
            with self.subTest(level=level, case='interrupted'):
                rc, why, bench, d, events = self.run_case(level, patch=interrupt)
                self.assertEqual(rc, 4)
                self.assertEqual(bench.s.state, loc.FAULTED)
                self.assertEqual(len(self.of(events, 'cancel_sent')), 1)    # a safety cancel
            with self.subTest(level=level, case='nothing moves'):
                rc, why, bench, d, events = self.run_case(level, deaf_teleop=True)
                self.assertEqual(rc, 2)
                self.assertIn('wall cap', why)

            def lose_teleop(d, bench):
                orig = d.run_cycle

                def rc_(direction, why_):
                    if direction == -1:
                        bench.teleop, d.teleop_alive = None, False
                        bench.scn['z'] = lambda w, k: 0.040
                    return orig(direction, why_)
                d.run_cycle = rc_
            with self.subTest(level=level, case='estop service fallback'):
                rc, why, bench, d, events = self.run_case(level, patch=lose_teleop)
                self.assertEqual(rc, 2, why)
                self.assertEqual(bench.requests[-1][0], 'estop')

            def broken(d, bench):
                d.hold_observation = lambda: (_ for _ in ()).throw(RuntimeError('injected'))
            with self.subTest(level=level, case='driver error'):
                rc, why, bench, d, events = self.run_case(level, patch=broken)
                self.assertEqual(rc, 3)
                self.assertIn('DRIVER ERROR', why)

            def fault(st, n):
                if st.get('state') == 'WALKING':
                    st['state'], st['faults'] = 'FAULTED', ['joint_states_stale']
                return st
            with self.subTest(level=level, case='unplanned FAULTED'):
                rc, why, bench, d, events = self.run_case(level, status_edit=fault)
                self.assertEqual(rc, 2, why)
                self.assertIn('no planned cancel', why)


class Shadow(Base):
    gate = False

    def test_complete_stationary_shadow_script_both_levels(self):
        for level in LEVELS:
            with self.subTest(level=level):
                rc, why, bench, d, events = self.run_case(level, shadow=True,
                                                          progress_m=s4at.CREEP)
                self.assertEqual((rc, why), (0, 'script complete (shadow rehearsal: dispatch '
                                             'disabled; not acceptance)'))
                self.assertEqual((bench.plant.sent, bench.s.transport.sent), (0, 18))
                sent = self.of(events, 'goal_sent')
                self.assertEqual([(g['b_from'], g['b_to'], g['direction']) for g in sent],
                                 list(self.mod.PLANNED))
                self.assertEqual({g['level'] for g in sent}, {level})
                self.assertEqual(self.of(events, 'cancel_sent'), [])
                self.assertEqual(self.keys(), self.mod.scripted_keys(level))
                self.assertTrue(all(c['progress_sign_bound'] == self.mod.SHADOW_SIGN_NOTE
                                    for c in d.cycle_log))
                self.assertEqual(len(d.cycle_log), 2)
                self.assertEqual(d.summary()['close']['exit_code'], 0)

    def test_progress_beyond_the_outer_bound_still_stops_in_shadow(self):
        for level in LEVELS:
            outer = {1: 0.060, 2: 0.080}[level]
            with self.subTest(level=level):
                rc, why, bench, d, events = self.run_case(
                    level, shadow=True, yaw0=0.0, forward_m=progress(level, outer + 1e-6, 0.0))
                self.assertEqual(rc, 2, why)
                self.assertIn('shadow rehearsal: |progress| at most', why)
                self.assertEqual(bench.plant.sent, 0)

    def test_mode_mismatch_or_change_stops_before_the_next_key(self):
        for level in LEVELS:
            def dispatching(st, n):
                st['mode'], st['dispatch_gate'] = 'dispatch', True
                return st
            with self.subTest(level=level, case='dispatch from the start'):
                rc, why, bench, d, events = self.run_case(level, shadow=True,
                                                          status_edit=dispatching)
                self.assertEqual(rc, 2, why)
                self.assertIn('shadow rehearsal', why)
                self.assertFalse({"'+'", "'a'", "'w'"} & set(self.keys()))
                self.assertEqual(bench.s.transport.sent, 0)

            def flips(st, n):
                if st.get('state') == 'READY' and st.get('cycles') == 1:
                    st['dispatch_gate'] = True
                return st
            with self.subTest(level=level, case='gate flips after the forward cycle'):
                rc, why, bench, d, events = self.run_case(level, shadow=True, status_edit=flips)
                self.assertEqual(rc, 2, why)
                self.assertIn('shadow rehearsal', why)
                self.assertNotIn("'s'", self.keys())


class ShadowAgainstDispatch(Base):
    gate = True

    def test_shadow_refuses_a_dispatching_node_before_any_key(self):
        for level in LEVELS:
            with self.subTest(level=level):
                rc, why, bench, d, events = self.run_case(level, shadow=False,
                                                          patch=self.as_shadow)
                self.assertEqual(rc, 2, why)
                self.assertIn('shadow rehearsal', why)
                self.assertEqual(bench.plant.sent, 0)
                self.assertNotIn("'a'", self.keys())

    @staticmethod
    def as_shadow(d, bench):
        d.shadow_rehearsal = True


class Constants(unittest.TestCase):
    def test_levels_plan_and_budget(self):
        m = load_driver()
        self.assertEqual({k: (v['stride_m'], v['teleop_label']) for k, v in m.LEVELS.items()},
                         {1: (0.040, '2/3'), 2: (0.060, '3/3')})
        self.assertEqual(m.LEVELS[1]['stop_m'], {1: (0.0, 0.060), -1: (-0.060, 0.0)})
        self.assertEqual(m.LEVELS[2]['stop_m'], {1: (0.0, 0.080), -1: (-0.080, 0.0)})
        self.assertEqual((m.EXPECTED_GOALS, m.EXPECTED_CANCELS), (18, 0))
        self.assertEqual(m.PLANNED[:9], tuple((k, k + 1, 1) for k in range(9)))
        self.assertEqual(m.PLANNED[9:], tuple((9 - k, 8 - k, -1) for k in range(9)))
        self.assertEqual(m.scripted_keys(1), ["'+'", "'a'", "'w'", "' '", "'s'", "' '", "'q'"])
        self.assertEqual(m.scripted_keys(2)[:2], ["'+'", "'+'"])
        self.assertEqual(m.CLOSE_REQUESTS, ('stop', 'disarm', 'stop', 'disarm'))
        with self.assertRaises(ValueError):
            m.S4cDriver.__new__(m.S4cDriver)._init_s4c(0)
        a, s3 = m.s4a, m.s3
        self.assertEqual((a.LATERAL_MAX_M, a.YAW_MAX_RAD), (0.020, 0.10))
        self.assertEqual((s3.TILT_MAX_RAD, s3.Z_MIN_M, s3.TRAVEL_MAX_M), (0.10, 0.045, 0.15))
        self.assertEqual((s3.JOINT_LIMIT_STOP_RAD, s3.TRACKING_ABORT_RAD), (0.02, 0.05))
        self.assertEqual(m.s2.CYCLE_S[1:], (21.1, 25.6))

    def test_the_selection_and_close_match_the_teleop_and_the_config(self):
        m = load_driver()
        cfg = loc.load_config()
        self.assertEqual(list(cfg.gait.stride_levels_m), [0.02, 0.04, 0.06])
        for level in (1, 2):
            t = tp.TeleopState(0.8)
            t.on_status({'levels_m_s': [0.00106, 0.0019, 0.00234]})
            for i in range(1, level + 1):
                svc, msg = t.key(tp.FASTER, 0.0)
                self.assertIsNone(svc)                              # no service, no motion
                self.assertEqual(m.CONFIRM_RE.findall(msg)[0][:2], (str(i + 1), '3'))
            self.assertEqual(t.level, level)
            self.assertEqual(f'{t.level + 1}/3', m.LEVELS[level]['teleop_label'])
            self.assertEqual(t.twist(0.0), (0.0, 0.0, 0.0))         # nothing held: zero
        t = tp.TeleopState(0.8)
        self.assertIn(m.UNKNOWN_LEVELS, t.key(tp.FASTER, 0.0)[1])

        calls = []

        class Node:
            state, out = tp.TeleopState(0.8), staticmethod(lambda *a: None)
            call = staticmethod(lambda name: calls.append(name))
            handle = tp.TeleopNode.handle
            publish = staticmethod(lambda wall: calls.append('zero'))
        n = Node()
        self.assertFalse(n.handle(tp.QUIT, 0.0))                     # q: the teleop quits
        tp._shutdown_sequence(n, type('E', (), {'spin_once': lambda *a, **k: None})(), print)
        self.assertEqual([c for c in calls if c != 'zero'], list(m.CLOSE_REQUESTS))

    def test_the_cli_refuses_level_0_3_or_none_before_any_ros_call(self):
        for args in (['--level', '0'], ['--level', '3'], []):
            with self.subTest(args=args), tempfile.TemporaryDirectory() as tmp:
                r = subprocess.run([sys.executable, os.path.join(HERE, 's4c_driver.py'),
                                    '--out', tmp, *args], capture_output=True, text=True,
                                   timeout=60, env={**os.environ, 'PYTHONPATH': ''})
                self.assertEqual(r.returncode, 5, r.stderr)
                with open(os.path.join(tmp, 'driver_summary.json')) as f:
                    self.assertIn('REFUSED: --level must be 1 or 2', json.load(f)['result'])


if __name__ == '__main__':
    unittest.main()
