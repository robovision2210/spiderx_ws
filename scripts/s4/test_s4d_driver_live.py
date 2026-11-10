#!/usr/bin/env python3
"""S4d-M driver against the IN-MEMORY M5.5 node on a simulated free base (no ROS, no simulator).

Reuses the S4c bench (test_s4c_driver_live.py, unchanged), which reuses the S4a and S3 benches: the
real LocomotionSession (the dispatching fake with the gate True in memory, or the node's
ShadowTransport with the gate False), the real teleop key logic and 2 Hz status, and a free-base
ground truth that follows the node's planned displacement (it moves at each phase's end, so a live
check sees the boundary pose exactly). The S4d bench adds:
- a world whose forward progress, lateral drift and yaw follow the planned displacement or the
  completed phases (per-cycle, cumulative, travel and band-edge cases);
- the teleop's status lines (its main loop prints one whenever the line changes);
- a second teleop (the reconnect probe), optionally blind to the node's status;
- the close as m55_teleop does it, in each asynchronous order: handle(q) calls stop and disarm, the
  shutdown sequence publishes one zero heartbeat and calls them again (default); disarm first (as
  S4c L2 live recorded); or the zero heartbeat handled, with a tick, before either request;
- late key deliveries (a late space, release, switch or q);
- the node's status cadence: every 0.5 s wall AND on every state change, as m55_rclpy publishes
  (_after: a status in the same callback as each transition);
- optionally (ActualDurations) a bench library whose phase durations are the FROZEN r2 library's
  own (library.json, sha256 917fc4d7...; read only, never rebuilt): goals 0.8-4.7 s at level 2,
  in both directions. Joint waypoints stay synthetic; only the timing is the real gait's.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_s4d_driver_live"""
import collections
import hashlib
import importlib.util
import json
import os
import re
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import test_s4c_driver_live as s4ct  # noqa: E402  (the S4c bench, unchanged)

s4at, s3t = s4ct.s4at, s4ct.s3t
c55, loc, tp, fk, LIMITS, Z0 = s3t.c55, s3t.loc, s3t.tp, s3t.fk, s3t.LIMITS, s3t.Z0
fx = s3t.fx
RTFS = (0.1, 0.3, 0.6, 1.0)
UM = 1e-6
CLOSE_DEFAULT = ('stop', 'disarm', 'zero', 'stop', 'disarm')    # handle(q), then the shutdown
CLOSE_DISARM_FIRST = ('disarm', 'stop', 'zero', 'disarm', 'stop')
CLOSE_ZERO_FIRST = ('zero', 'tick', 'stop', 'disarm', 'stop', 'disarm')


def load_driver():
    spec = importlib.util.spec_from_file_location('s4d_driver_under_test',
                                                  os.path.join(HERE, 's4d_driver.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class S4dWorld(s4at.S4aWorld):
    """The free-base world, driven by the node's planned displacement py (planned_xy[1]). Hooks:
    fwd(world, py) -> forward (m, default py); lat(world) -> lateral (m); yaw(world) -> yaw offset
    (rad); z(wall, py), tilt(wall, py). world.s is the session (phases_completed, cycles)."""

    def tick(self):
        fk.FakeWorld.tick(self)
        py = self.s.planned_xy[1]
        f = self.scn['fwd'](self, py) if 'fwd' in self.scn else py
        lat = self.scn['lat'](self) if 'lat' in self.scn else 0.0
        yaw = self.scn.get('yaw0', 0.3) + (self.scn['yaw'](self) if 'yaw' in self.scn else 0.0)
        df, dl = f - self.f, lat - self.lat
        self.f, self.lat, self.yaw = f, lat, yaw
        self.x += s4at.math.cos(yaw) * dl - s4at.math.sin(yaw) * df
        self.y += s4at.math.sin(yaw) * dl + s4at.math.cos(yaw) * df
        z = self.scn.get('z', lambda w, k: Z0)(self.wall, py)
        tilt = self.scn.get('tilt', lambda w, k: 0.01)(self.wall, py)
        self.s.on_body_pose(self.wall, (), {'xy': (self.x, self.y), 'z': z,
                                            'yaw': s4at.math.atan2(s4at.math.sin(yaw),
                                                                   s4at.math.cos(yaw)),
                                            'tilt': tilt})


class S4dBench(s4ct.S4cBench):
    """The S4c bench with the teleop's status lines, a second teleop, the close orders and late
    deliveries. delay: {(key, occurrence): seconds} delivers that write late; close_order: the
    node's handling order of the close (requests, 'zero' heartbeat, 'tick'); zero_after_q_s: zero
    heartbeats continue after 'q', up to that long, while the phase is in flight (as when 'q'
    lands within the lease of the phase's end); exit_codes: {spawn number: wait status};
    blind_spawn: that teleop never receives a node status."""

    def __init__(self, *a, close_order=CLOSE_DEFAULT, zero_after_q_s=0.0, delay=None,
                 exit_codes=None, blind_spawn=None, **kw):
        super().__init__(*a, **kw)
        self.close_order, self.zero_after_q_s = close_order, zero_after_q_s
        self.delay, self.exit_codes = dict(delay or {}), dict(exit_codes or {})
        self.blind_spawn, self.blind = blind_spawn, False
        self.pending_keys, self.writes = [], collections.Counter()
        self.last_line, self.n_spawn = None, 0
        self.zero_until, self.next_zero = None, None
        self.q_walls = []

    def spawn_teleop(self):
        self.n_spawn += 1
        self.teleop, self.teleop_exit, self.last_line = tp.TeleopState(self.cfg.teleop.hold_s), \
            None, None
        self.blind = self.blind_spawn == self.n_spawn
        if self.blind:
            self.teleop.on_status = lambda st: None
        self.driver._on_teleop_out(tp.HELP.format(gate=c55.M55_LOCOMOTION_DISPATCH_ENABLED)
                                   + '\n' + tp.status_line(None) + '\n', self.world.wall)

    def deliver(self, ch):
        self.writes[ch] += 1
        d = self.delay.get((ch, self.writes[ch]))
        if d:
            self.pending_keys.append((self.world.wall + d, ch))
        else:
            self.key(ch)

    def tick(self):
        due = [p for p in self.pending_keys if p[0] <= self.world.wall + 1e-9]
        self.pending_keys = [p for p in self.pending_keys if p not in due]
        for _, ch in due:
            self.key(ch)
        if self.zero_until is not None and self.world.wall < self.zero_until and \
                self.s.goal is not None and self.world.wall >= self.next_zero - 1e-9:
            self.s.on_command(self.world.wall, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self.next_zero = self.world.wall + 0.1
        if self.held in self.swap:                     # a key read as another, repeats too
            self.held = self.swap[self.held]
        n = self.n_status
        super().tick()
        if self.s.state != self.last_state:            # m55_rclpy _after: publish on a change
            self.last_state = self.s.state
            if self.n_status == n and self.driver is not None:
                self._publish()
        if self.n_status != n and self.teleop is not None and not self.blind and \
                self.driver is not None:
            line = tp.status_line(self.s.status(self.world.wall))
            if line != self.last_line:
                self.last_line = line
                self.driver._on_teleop_out(line + '\n', self.world.wall)

    def _publish(self):
        """One status publication, as S3's bench does its periodic one."""
        msg = s3t.Msg(json.dumps(self.s.status(self.world.wall), sort_keys=True, default=repr))
        if self.teleop is not None:
            self.teleop.on_status(json.loads(msg.data))
        self.n_status += 1
        edit = self.scn.get('status_edit')
        if edit:
            msg = s3t.Msg(json.dumps(edit(json.loads(msg.data), self.n_status), sort_keys=True))
        self.driver._on_status(msg)
        if self.scn.get('duplicate', lambda n: False)(self.n_status):
            self.driver._on_status(s3t.Msg(msg.data))

    def key(self, ch):
        ch = self.swap.get(ch, ch)
        if self.teleop is None or ch in self.ignore or (self.deaf and ch == 'w'):
            return
        if tp.KEYMAP[ch] != tp.QUIT:
            return super().key(ch)
        w = self.world.wall
        self.teleop.key(tp.QUIT, w)                     # motion 0
        for item in self.close_order:
            if item == 'zero':
                self.s.on_command(w, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            elif item == 'tick':
                self.s.step(w)
            else:
                self.requests.append((item, self.s.request(item, w)))
        self.q_walls.append(w)
        self.teleop, self.teleop_exit = None, self.exit_codes.get(self.n_spawn, 0)
        if self.zero_after_q_s:
            self.zero_until, self.next_zero = w + self.zero_after_q_s, w + 0.1


def make_bench(rtf=0.6, shadow=False, library=None, **kw):
    b = S4dBench(rtf, shadow=shadow, **kw)
    if library is not None:                  # as S3's bench builds them, with this library
        b.plant = fk.FakeLocomotionTransport(library, library.boundary_positions(0, 0),
                                             scenario='ok', fail_phase=1, drift_rad_s=0.05,
                                             use_sim=True)
        b.s = loc.LocomotionSession(b.cfg, library, loc.ShadowTransport(library.fingerprints)
                                    if shadow else b.plant)
    b.world = S4dWorld(b.s, b.plant, b.scn, rtf=rtf, graph_period_s=1.0)
    b.next_status = b.world.wall
    b.s.log = collections.deque(maxlen=None)  # every event, as the node's events.jsonl keeps them
    b.last_state = b.s.state
    return b


FROZEN_LIBRARY = os.path.join(HERE, '..', '..', 'docs', 'evidence', 'm55',
                              's4c_l2_shadow_20261103T094244Z', 'node', 'library.json')
FROZEN_LIBRARY_SHA256 = '917fc4d796efe2344d0edcf038a1f7186d053622df250d9b389690a87d485d8e'
_ACTUAL = {}


def frozen_durations():
    """(lead_s, {level: [phase durations, s]}) from the frozen r2 library (read only)."""
    with open(FROZEN_LIBRARY, 'rb') as f:
        raw = f.read()
    if hashlib.sha256(raw).hexdigest() != FROZEN_LIBRARY_SHA256:
        raise AssertionError('the frozen library.json changed')
    lib = json.loads(raw)['library']
    return lib['lead_s'], {L['level']: L['phase_durations_s'] for L in lib['levels']}


def actual_duration_template(stride, durations_s, dt=0.1, amp=0.05):
    """m55_fixtures.synthetic_template with phase k lasting durations_s[k] (the frozen gait's own
    phase durations). Synthetic joint waypoints; rest at every boundary."""
    P, scale = len(fx.PHASES), stride / 0.02

    def boundary(k):
        return [amp * scale * s4at.math.sin(s4at.math.pi * k / P) * (1 + 0.5 * k / P)
                * (1 + 0.1 * j) for j in range(len(fx.JOINTS))]
    pts, bounds, t = [], [0], 0.0
    for k in range(P):
        n = int(round(durations_s[k] / dt))
        a, b = boundary(k), boundary(k + 1)
        for i in range(n + (1 if k == 0 else 0)):
            s = (i if k == 0 else i + 1) / n
            h, dh = 3 * s * s - 2 * s ** 3, (6 * s - 6 * s * s) / (n * dt)
            pts.append({'time_from_start_s': round(t + s * n * dt, 9),
                        'positions': [x + (y - x) * h for x, y in zip(a, b)],
                        'velocities': [(y - x) * dh for x, y in zip(a, b)]})
        t = round(t + n * dt, 9)
        bounds.append(len(pts) - 1)
    for i in bounds:
        pts[i]['velocities'] = [0.0] * len(fx.JOINTS)
    pts[-1]['positions'] = [0.0] * len(fx.JOINTS)
    params = fx.cr.CrawlParams(stride, 0.012, 0.5, 0.6, 0.02, 0.015, 0.25, 0.9, dt, 0.02, 0.002)
    body = [(0.0, stride * k / P) for k in range(P + 1)]
    report = {'schema': fx.cr.SCHEMA, 'failures': [], 'speed_m_s': stride / t,
              'phase_durations_s': list(durations_s)}
    return fx.cr.CrawlTemplate(params, 1, list(fx.JOINTS), pts, report, bounds, list(fx.PHASES),
                               body)


def actual_library():
    """A bench PhaseLibrary with the frozen library's strides, lead-in and phase durations."""
    if 'lib' not in _ACTUAL:
        lead, durations = frozen_durations()
        _ACTUAL['lib'] = loc.PhaseLibrary([actual_duration_template(s, durations[L])
                                           for L, s in enumerate((0.02, 0.04, 0.06))], lead)
    return _ACTUAL['lib']


def make_driver(mod, bench, out_dir, shadow=False, write_hook=None):
    class InMemoryS4dDriver(mod.S4dDriver):
        def __init__(self):                                         # no ROS, no threads
            self._init_s3(LIMITS, shadow, {'bench': 'gates checked by main()'} if shadow else None)
            self._init_s4a()
            self._init_s4d()
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
            if write_hook:
                write_hook(bench, data)
            if bench.write_s:
                bench.sleep(bench.write_s / 2)
            bench.deliver(data.decode())
            if bench.write_s:
                bench.sleep(bench.write_s / 2)
            return len(data)

        def _teleop_exit_status(self):                              # the wait status only
            return bench.teleop_exit

        def hold(self, k, why):
            self.key(k, why)
            bench.held, bench.held_since, self.hold_key = k, bench.world.wall, k

        def _stop_repeat(self):                                     # the repeat stop only
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

    for m in (mod, mod.s4b, mod.s4c, mod.s4a, mod.s3, mod.s2):
        m.time = bench                       # the inherited methods use the bench clock too
    d = InMemoryS4dDriver()
    bench.driver = d
    return d


def nth_write(key, n, action):
    """A write hook calling action(bench) at the n-th write of key (1-based)."""
    seen = collections.Counter()

    def hook(bench, data):
        ch = data.decode()
        seen[ch] += 1
        if ch == key and seen[ch] == n:
            action(bench)
    return hook


def after_phases(n, value, before=0.0):
    """A world hook: `value` once the node has completed n phases, else `before`."""
    return lambda world: value if world.s.phases_completed >= n else before


class Base(unittest.TestCase):
    gate = True

    def setUp(self):
        self.saved = c55.M55_LOCOMOTION_DISPATCH_ENABLED
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.gate             # tests only, in memory
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = self.saved
        self.tmp.cleanup()

    def run_case(self, rtf=0.6, patch=None, shadow=False, write_hook=None, **kw):
        self.mod = load_driver()
        bench = make_bench(rtf, shadow=shadow, **kw)
        self.bench = bench
        d = make_driver(self.mod, bench, self.tmp.name, shadow=shadow, write_hook=write_hook)
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


class Complete(Base):
    """check_complete only: the full-script assertions shared by Live and ActualDurations."""

    def check_complete(self, rc, why, bench, d, events, close_order=CLOSE_DEFAULT):
        m = self.mod
        self.assertEqual((rc, why), (0, 'script complete'))
        sent = self.of(events, 'goal_sent')
        self.assertEqual([(g['level'], g['b_from'], g['b_to'], g['direction']) for g in sent],
                         list(m.PLANNED))
        self.assertEqual((bench.plant.sent, d.expected_goals, len(sent)), (111, 111, 111))
        for name in ('cancel_sent', 'fault', 'refused', 'command_rejected'):
            self.assertEqual(self.of(events, name), [], name)
        self.assertEqual(self.keys(), m.SCRIPTED_KEYS)
        lc = self.of(events, 'level_change')
        self.assertEqual([(e['previous'], e['level'], e['state']) for e in lc],
                         [(0, 1, 'READY'), (1, 2, 'WALKING')])
        done = self.of(events, 'phase_done')
        self.assertLess(done[8]['n'], lc[1]['n'])                    # after goal 9's phase_done
        self.assertLess(lc[1]['n'], sent[9]['n'])                    # before goal 10's goal_sent
        tr = self.of(events, 'transition')
        for first, last in ((0, 53), (54, 98)):                     # continuous: 1-54, 55-99
            self.assertEqual([t['state'] for t in tr
                              if sent[first]['n'] < t['n'] < sent[last]['n']],
                             ['STARTING', 'WALKING'])
        close = [x for x in close_order if x not in ('zero', 'tick')]
        self.assertEqual([r[0] for r in bench.requests],
                         ['arm', 'stop', 'stop', 'home'] + close + close)
        last = [(t['previous'], t['state']) for t in tr[-2:]]
        self.assertEqual(last, [('WALKING', 'STOPPING'), ('STOPPING', 'DISARMED')])
        self.assertEqual(self.of(events, 'phase_done')[-1]['b'], 1)
        self.assertEqual(bench.s.state, loc.DISARMED)
        s = d.summary()
        self.assertEqual([(c['name'], c['cycles_count']) for c in s['cycles']],
                         [('SC', 1)] + [(f'F{j}', 1 + j) for j in range(1, 6)]
                         + [(f'R{j}', 6 - j) for j in range(1, 6)] + [('X', 2)])
        for c in s['cycles']:
            want = {1: 0.040, 2: 0.060}[c['level']] * c['direction']
            self.assertAlmostEqual(c['forward_m'], want, delta=1e-6)
            if c['name'] in m.CUMULATIVE_CHECKS:
                self.assertLess(abs(c['cumulative_from_b9']['lateral_m']), 1e-6)
        self.assertEqual(s['selection']['confirmations'][0]['label'], '2/3')
        self.assertEqual(s['speed_change']['label'], '3/3')
        self.assertEqual(s['speed_change']['goal_in_flight_at_write'], 2)
        self.assertEqual([r['step'] for r in s['releases']],
                         ['F_space', 'R_space', 'X_release', 'X_reversal_release',
                          'probe_release'])
        self.assertEqual(len(s['switches']), 1)
        self.assertEqual(s['close']['exit_code'], 0)
        self.assertTrue(s['close_observation']['within_A6_limits'])
        p = s['probe']
        self.assertEqual([c['label'] for c in p['confirmations']], ['2/3', '3/3'])
        self.assertEqual((p['delivery']['intent']['direction'], p['delivery']['intent']['level']),
                         (1, 2))
        self.assertEqual(p['stopped_after_release']['intent']['direction'], 0)
        self.assertGreater(p['commands_accepted_after'], p['commands_accepted_before'])
        self.assertEqual(p['close']['exit_code'], 0)
        recs = self.records()
        q = [r for r in recs if r.get('key') == "'q'"]
        rs = recs.index(next(r for r in recs if 'repeat_stop_before_q' in r))
        gap = q[0]['t_before_write'] - recs[rs]['repeat_stop_before_q']['t_after_stop']
        self.assertTrue(0.0 <= gap < 0.8 - 1 / 30, gap)             # inside the hold window
        # the probe: no arm, no goal, no transition; its close requests are accepted no-ops
        t_probe = next(r['mono'] for r in recs if r.get('event') == 'teleop_started'
                       and r.get('pid') == 2)
        after = [e for e in events if e['wall'] >= t_probe - 1e-3]
        self.assertEqual([e['event'] for e in after if e['event'] != 'request'], [])
        self.assertEqual(sorted((e['request'], e['message']) for e in after),
                         sorted([('stop', 'not moving (DISARMED); held until a zero command'),
                                 ('disarm', 'already disarmed')] * 2))
        return s


class Live(Complete):
    def test_complete_script_111_goals_zero_cancels_across_the_rtf_range(self):
        for rtf in RTFS:
            with self.subTest(rtf=rtf):
                rc, why, bench, d, events = self.run_case(rtf)
                self.check_complete(rc, why, bench, d, events)
                dis = self.of(events, 'transition', state='DISARMED')[-1]
                self.assertEqual(dis['why'], 'lease_expired')       # q early in a 1.7 s phase

    def test_duplicate_publications_do_not_change_the_script(self):
        rc, why, bench, d, events = self.run_case(duplicate=lambda n: True)
        self.check_complete(rc, why, bench, d, events)

    def test_both_close_request_orders_and_the_intent_stop_order(self):
        for order, stopping in ((CLOSE_DEFAULT, 'stop'), (CLOSE_DISARM_FIRST, 'disarm'),
                                (CLOSE_ZERO_FIRST, 'intent_stop')):
            with self.subTest(order=order):
                rc, why, bench, d, events = self.run_case(close_order=order)
                self.check_complete(rc, why, bench, d, events, close_order=order)
                tr = self.of(events, 'transition')
                self.assertEqual((tr[-2]['why'], tr[-1]['why']), (stopping, 'lease_expired'))

    def test_a_lease_kept_fresh_to_the_phase_end_disarms_with_reason_disarm(self):
        # zero heartbeats continue past the phase end (as when q lands within 0.5 s of it)
        rc, why, bench, d, events = self.run_case(zero_after_q_s=6.0)
        self.check_complete(rc, why, bench, d, events)
        self.assertEqual(self.of(events, 'transition')[-1]['why'], 'disarm')

    # ------------------------------------------------------------------ the speed change
    def test_level_adopted_early_or_not_at_all_stops(self):
        def early(st, n):                                   # goal 3 reported at level 2
            g = st.get('goal')
            if g and g.get('seq') == 3:
                g['level'] = 2
            return st

        def late(st, n):                                    # goal 10 reported at level 1
            g = st.get('goal')
            if g and g.get('seq') == 10:
                g['level'] = 1
            return st

        def node_mid_cycle(st, n):                          # the node's level 2 inside goal 5
            if (st.get('goal') or {}).get('seq') == 5:
                st['level'] = 2
            return st
        for name, edit, text in (('early', early, 'goal 3 is (2, 2, 3, 1), planned (1, 2, 3, 1)'),
                                 ('late', late, 'goal 10 is (1, 0, 1, 1), planned (2, 0, 1, 1)'),
                                 ('mid-cycle', node_mid_cycle, 'node level 2 while goal 5')):
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(status_edit=edit)
                self.assert_stopped(rc, why, bench, events, text)
                self.assertLessEqual(bench.plant.sent, 10)

    def test_a_real_mid_cycle_adoption_stops(self):
        def adopt_anywhere(d, bench):                       # the node switches at any boundary
            s, original = bench.s, bench.s._after_rest

            def after_rest(wall):
                intent, _ = s._lease(wall)
                if intent.moving and intent.level != s.level and s.b != 0:
                    s.level = intent.level
                return original(wall)
            s._after_rest = after_rest
        rc, why, bench, d, events = self.run_case(patch=adopt_anywhere)
        self.assertEqual(rc, 2, why)
        self.assertTrue(any(x in why for x in ('FAULTED', 'planned (1, ', 'node level 2')), why)
        self.assertLess(bench.plant.sent, 10)

    def test_an_unconfirmed_or_wrong_speed_change_stops_before_the_boundary(self):
        def wrong_speed(text):
            return re.sub(r': [0-9.]+ mm/s', ': 9.99 mm/s', text)
        # never confirmed: the teleop stays at 2/3 and the node walks on at level 1, which the
        # plan check stops at boundary 0 (goal 10); a wrong confirmation stops at once
        for name, hook, text, n in (
                ('ignored', nth_write('+', 2, lambda b: b.ignore.add('+')),
                 'goal 10 is (1, 0, 1, 1), planned (2, 0, 1, 1)', 10),
                ('wrong speed', nth_write('+', 2, lambda b: setattr(b, 'mangle', wrong_speed)),
                 'expected 3/3 at', 2)):
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(write_hook=hook)
                self.assert_stopped(rc, why, bench, events, text)
                self.assertEqual(bench.plant.sent, n)
                self.assertEqual({g['level'] for g in self.of(events, 'goal_sent')}, {1})

    # ------------------------------------------------------------------ the plan
    def test_extra_wrong_level_and_wrong_direction_goals_stop(self):
        def no_release(d, bench):
            d._stop_repeat = lambda: d.log(note='test: the repeat was not stopped')

        def level_1(b):
            b.teleop.level = 1
        cases = (
            ('extra forward goal', {'ignore': {' '}, 'patch': no_release},
             'goal 55 is (2, 0, 1, 1), planned (2, 9, 8, -1)'),
            ('wrong level', {'write_hook': nth_write('s', 1, level_1)},
             'goal 55 is (1, 9, 8, -1), planned (2, 9, 8, -1)'),
            ('wrong direction', {'write_hook': nth_write('s', 1, lambda b: b.swap.update(s='w'))},
             'goal 55 is (2, 0, 1, 1), planned (2, 9, 8, -1)'))
        for name, kw, text in cases:
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(**kw)
                self.assert_stopped(rc, why, bench, events, text)
                self.assertEqual(bench.plant.sent, 55)

    def test_a_moving_intent_at_another_level_stops(self):
        def level_0(st, n):
            if (st.get('intent') or {}).get('level') == 1:
                st['intent']['level'] = 0
            return st
        rc, why, bench, d, events = self.run_case(status_edit=level_0)
        self.assert_stopped(rc, why, bench, events, 'intent level 0')

    def test_late_space_release_reversal_and_close_stop(self):
        def keep_repeating(d, bench):                       # the repeat outlives the action
            d._stop_repeat = lambda: d.log(note='test: the repeat continued')

        def late_switch(b):               # the switch not in effect at boundary 6: w goes on
            b.swap['s'] = 'w'
        cases = (
            ('late space', {'patch': keep_repeating, 'delay': {(' ', 1): 30.0}},
             'goal 55 is (2, 0, 1, 1), planned (2, 9, 8, -1)', 55),
            ('late reversal', {'write_hook': nth_write('s', 2, late_switch)},
             'goal 106 is (2, 6, 7, 1), planned (2, 6, 5, -1)', 106))
        for name, kw, text, n in cases:
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(**kw)
                self.assert_stopped(rc, why, bench, events, text)
                self.assertEqual(bench.plant.sent, n)

    def test_a_release_that_does_not_stop_the_node_exceeds_the_step_budget(self):
        def late_release(d, bench):
            original = d.release

            def release(why, step=None):
                if step == 'X_release':
                    d.log(note='test: the release never reached the teleop')
                    d.releases.append({'step': step})
                    return
                original(why, step)
            d.release = release
        rc, why, bench, d, events = self.run_case(patch=late_release)
        self.assert_stopped(rc, why, bench, events, 'goal seq 105 above the step budget (104)')
        self.assertEqual(bench.plant.sent, 105)

    def test_a_close_never_read_or_read_too_late_stops_through_the_estop_service(self):
        def keep_repeating(d, bench):
            orig = d._stop_repeat_timed

            def timed(why):
                if why.startswith('Q:'):
                    return {'key': repr(d.hold_key), 'why': why, 't_before_stop': 0.0,
                            't_after_stop': 0.0}
                return orig(why)
            d._stop_repeat_timed = timed
        for name, kw, text in (
                ('q never read, w still held: goal 112',
                 {'ignore': {'q'}, 'patch': keep_repeating}, 'goal 112'),
                ('q never read, w released: READY instead of DISARMED',
                 {'ignore': {'q'}}, "state READY with goal None not allowed in step 'closing'")):
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(**kw)
                self.assert_stopped(rc, why, bench, events, text)
                self.assertIn('estop_service', [r.get('event') for r in self.records()])
                self.assertNotIn("'x'", self.keys())

    # ------------------------------------------------------------------ confirmations
    def test_split_delayed_and_bracketed_confirmations_complete(self):
        for name, kw in (('split, gap 0.2 s', {'split_at': 19, 'split_gap_s': 0.2,
                                               'confirm_delay_s': 0.2}),
                         ('after the write returned', {'confirm_delay_s': 0.3}),
                         ('inside the write bracket', {'write_s': 0.1})):
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(**kw)
                self.check_complete(rc, why, bench, d, events)
                recs = self.records()
                confs = ([recs_s['selection']['confirmations'][0] for recs_s in recs
                          if 'selection' in recs_s]
                         + [r['speed_change'] for r in recs if 'speed_change' in r]
                         + next(r['probe']['confirmations'] for r in recs if 'probe' in r))
                self.assertEqual(len(confs), 4)
                for c in confs:
                    self.assertGreaterEqual(c['t_receipt'], c['t_before_write'])
                    inside = c['t_before_write'] < c['t_receipt'] < c['t_after_write']
                    self.assertEqual(c['received_before_the_write_returned'], inside)
                    if name == 'inside the write bracket':
                        self.assertTrue(inside, c)

    def test_stale_duplicate_or_missing_confirmations_stop(self):
        def stale(bench, data):
            if data == b'+':
                bench.driver._on_teleop_out('speed level 2/3: 1.90 mm/s\n',
                                            bench.world.wall - 0.001)
        for name, kw, text, armed in (
                ('stale', {'ignore': {'+'}, 'write_hook': stale}, 'stale level confirmation',
                 False),
                ('duplicate', {'duplicate_output': True}, 'level confirmations for one key',
                 False),
                ('missing', {'ignore': {'+'}}, 'teleop confirms speed level 2/3', False),
                ('duplicate in the probe', {'write_hook': nth_write(
                    '+', 3, lambda b: setattr(b, 'duplicate_output', True))},
                 'level confirmations for one key', True)):
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(**kw)
                self.assertEqual(rc, 2, why)
                self.assertIn(text, why)
                if armed:
                    self.assertEqual(bench.plant.sent, 111)
                    self.assertIn('estop', bench.s.faults)          # x through the probe teleop
                else:
                    self.assertNotIn("'a'", self.keys())
                    self.assertEqual(self.of(events, 'goal_sent'), [])

    def test_the_first_level_key_is_refused_unless_disarmed_before_arm(self):
        for state, goal, armed in (('READY', None, True), ('WALKING', {'seq': 1}, True),
                                   ('DISARMED', {'seq': 1}, False)):
            with self.subTest(state=state, goal=goal):
                mod = load_driver()
                d = make_driver(mod, make_bench(0.6), self.tmp.name)
                d.status = {'state': state, 'goal': goal, 'levels_m_s': [1, 2, 3],
                            'strides_m': [0.02, 0.04, 0.06]}
                d.armed = armed
                with self.assertRaises(mod.s2.Abort):
                    d.select_level_1()
                d.out.close()
                self.assertEqual(self.keys(), [])
        for strides in ([0.02, 0.04], [0.02, 0.05, 0.06], [0.02, 0.04, 0.07]):
            with self.subTest(strides=strides):
                mod = load_driver()
                d = make_driver(mod, make_bench(0.6), self.tmp.name)
                d.status = {'state': 'DISARMED', 'goal': None, 'levels_m_s': [1, 2, 3][
                    :len(strides)], 'strides_m': strides}
                with self.assertRaises(mod.s2.Abort):
                    d.select_level_1()
                d.out.close()
                self.assertEqual(self.keys(), [])

    # ------------------------------------------------------------------ live stops
    def test_per_cycle_band_edges_are_inclusive(self):
        def fwd_cycle(k, length):
            """Body forward = planned, except that cycle check k measures `length`."""
            def f(world, py):
                done = world.s.phases_completed
                return py + (length - nominal[k]) * (sign[k] if done >= ends[k] else 0)
            return f
        nominal = {'SC': 0.040, 'F3': 0.060, 'R2': 0.060, 'X': 0.060}
        sign = {'SC': 1, 'F3': 1, 'R2': -1, 'X': 1}
        ends = {'SC': 9, 'F3': 36, 'R2': 72, 'X': 110}
        cases = (('SC', 0.060 - UM, True), ('SC', 0.060 + UM, False), ('SC', -UM, False),
                 ('F3', 0.080 - UM, True), ('F3', 0.080 + UM, False),
                 ('R2', 0.080 - UM, True), ('R2', 0.080 + UM, False), ('R2', -UM, False),
                 ('X', 0.080 - UM, True), ('X', 0.080 + UM, False))
        for k, length, ok in cases:
            with self.subTest(cycle=k, length=length):
                rc, why, bench, d, events = self.run_case(yaw0=0.0, fwd=fwd_cycle(k, length))
                if ok:
                    self.assertEqual(rc, 0, why)
                else:
                    self.assert_stopped(rc, why, bench, events, f'{k} cycle moved')

    def test_per_cycle_and_cumulative_drift_stops_are_inclusive(self):
        def steps(per, last, start, n=5):
            """per cycle `per` from goal `start` on; the n-th step lands exactly on `last`."""
            def f(world):
                k = max(0, min(n, (world.s.phases_completed - start) // 9 + 1))
                return 0.0 if k == 0 else (last if k == n else per * k)
            return f
        cases = (
            ('cycle lateral', {'lat': after_phases(27, 0.020 - UM)}, True, None),
            ('cycle lateral', {'lat': after_phases(27, 0.020 + UM)}, False, 'F2 cycle drifted'),
            ('cycle yaw', {'yaw': after_phases(72, 0.10 - UM)}, True, None),
            ('cycle yaw', {'yaw': after_phases(72, 0.10 + UM)}, False, 'R2 cycle drifted'),
            ('cumulative lateral', {'lat': steps(0.010, 0.050 - UM, 18)}, True, None),
            ('cumulative lateral', {'lat': steps(0.010, 0.050 + UM, 18)}, False,
             'F5: cumulative drift'),
            ('cumulative heading', {'yaw': steps(0.03, 0.15 - UM, 18)}, True, None),
            ('cumulative heading', {'yaw': steps(0.03, 0.15 + UM, 18)}, False,
             'F5: cumulative drift'))
        for name, kw, ok, text in cases:
            with self.subTest(case=name, ok=ok):
                rc, why, bench, d, events = self.run_case(yaw0=0.0, **kw)
                if ok:
                    self.assertEqual(rc, 0, why)
                else:
                    self.assert_stopped(rc, why, bench, events, text)

    def test_cumulative_drift_spread_over_the_out_and_back_stops_in_r(self):
        # 9 mm per cycle: every cycle within the 20 mm per-cycle stop; from B9 the drift is 45 mm
        # at F5 and 54 mm at R1, which the cumulative stop catches
        def lat(world):
            return 0.009 * (max(0, world.s.phases_completed - 9) // 9)
        rc, why, bench, d, events = self.run_case(lat=lat)
        self.assert_stopped(rc, why, bench, events, 'R1: cumulative drift')
        s = d.summary()
        self.assertTrue(all(abs(c['lateral_m']) <= 0.020 for c in s['cycles']))
        self.assertAlmostEqual(s['cycles'][-1]['cumulative_from_b9']['lateral_m'], 0.054,
                               delta=1e-6)

    def test_the_s4d_travel_cap_is_0_40_m_from_the_arm_pose(self):
        for over, ok in ((-UM, True), (UM, False)):
            scale = (0.36 + over) / 0.30

            def fwd(world, py, scale=scale):
                return py if py <= 0.04 else 0.04 + (py - 0.04) * scale
            with self.subTest(over=over):
                rc, why, bench, d, events = self.run_case(yaw0=0.0, fwd=fwd)
                if ok:
                    self.assertEqual(rc, 0, why)
                else:
                    self.assert_stopped(rc, why, bench, events, 'travel')
        self.assertEqual(self.mod.s3.TRAVEL_MAX_M, 0.15)            # the shared S3 cap unchanged

    def test_stale_samples_delay_a_check_and_a_cycle_without_one_stops(self):
        stale_sims = []

        def stale(st, n):                    # the first 6 statuses after the count reaches 1
            if st.get('cycles') == 1 and len(stale_sims) < 6 and st.get('ground_truth'):
                stale_sims.append(st['monitors']['sim_time_s'])
                st['monitors']['body_pose_age_s'] = 5.0
            return st
        rc, why, bench, d, events = self.run_case(status_edit=stale)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertEqual(len(stale_sims), 6)
        self.assertGreater(d.cycle_log[0]['sim'], max(stale_sims))

        def blind_f1(st, n):                 # no fresh sample in the whole first F cycle
            if st.get('cycles') == 2:
                st['monitors']['body_pose_age_s'] = 5.0
            return st
        rc, why, bench, d, events = self.run_case(status_edit=blind_f1)
        self.assert_stopped(rc, why, bench, events, 'cycle count 1 -> 3: not one forward cycle')

    def test_a_stalled_clock_is_not_fresh(self):
        win = {'t0': None}

        def stall(w):
            return win['t0'] is not None and w - win['t0'] < 1.0

        def tilt(w, py):
            if py > 0.1 and win['t0'] is None:
                win['t0'] = w + 0.3
            return 0.15 if stall(w) else 0.01
        rc, why, bench, d, events = self.run_case(tilt=tilt, stall=stall)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertIsNotNone(win['t0'])

    def test_tilt_fall_and_joint_protections_unchanged(self):
        for name, kw, text in (('tilt', {'tilt': lambda w, py: 0.15 if py > 0.2 else 0.01},
                                'tilt'),
                               ('fall', {'z': lambda w, py: 0.040 if py > 0.2 else Z0}, 'body z')):
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(**kw)
                self.assert_stopped(rc, why, bench, events, text)
        lo, start = LIMITS['rf_foot_joint'][0], {}

        def joint(w):
            start.setdefault('w', w + 60.0)
            return {'rf_foot_joint': lo + 0.015} if w > start['w'] else None
        rc, why, bench, d, events = self.run_case(joint=joint)
        self.assert_stopped(rc, why, bench, events, 'URDF limit')

    def test_the_90_minute_script_cap(self):
        def old(d, bench):                  # the cap is reached 200 s (bench wall) into the run
            d.t_start -= self.mod.SCRIPT_CAP_WALL_S - 200.0
        rc, why, bench, d, events = self.run_case(patch=old)
        self.assert_stopped(rc, why, bench, events, 'script wall cap 90 min exceeded')

    # ------------------------------------------------------------------ the probe
    def test_probe_failures_stop(self):
        def arm_in_probe(b):
            b.requests.append(('arm', b.s.request('arm', b.world.wall)))

        def reject(st, n):
            if st.get('state') == 'DISARMED' and (st.get('commands_accepted') or 0) > 0 and \
                    rejected['on']:
                st['command_rejections'] = {'command_rate_exceeded': 1}
            return st
        rejected = {'on': False}
        cases = (
            ('blind teleop', {'blind_spawn': 2}, 'probe: the teleop shows a node status'),
            ('w never delivered', {'write_hook': nth_write(
                'w', 5, lambda b: setattr(b, 'deaf', True))}, 'probe: no status showed'),
            ('an arm during the probe', {'write_hook': nth_write('+', 3, arm_in_probe)},
             ("state READY with goal None not allowed in step 'probe'", 'goal 112')),
            ('non-zero exit', {'exit_codes': {2: 256}}, 'probe: the teleop exited with 1'),
            ('a command rejection', {'status_edit': reject, 'write_hook': nth_write(
                '+', 3, lambda b: rejected.update(on=True))}, 'command rejections'))
        for name, kw, text in cases:
            rejected['on'] = False
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_case(**kw)
                texts = text if isinstance(text, tuple) else (text,)
                self.assertTrue(any(x in why for x in texts), why)
                self.assert_stopped(rc, why, bench, events, '')
                # an arm lets the held w dispatch one goal before the stop (statuses are
                # periodic on the bench; the real node also publishes on every state change)
                self.assertIn(bench.plant.sent, (111, 112) if isinstance(text, tuple) else (111,))

    def test_the_main_close_must_exit_cleanly_before_the_probe(self):
        rc, why, bench, d, events = self.run_case(exit_codes={1: 256})
        self.assert_stopped(rc, why, bench, events, 'Q: the teleop exited with 1')
        self.assertEqual(bench.n_spawn, 1)


class ActualDurations(Complete):
    """The complete plan on a bench library with the FROZEN r2 gait's phase durations
    (library.json, read only): level-2 goals of 0.8-4.7 s in both directions, the 1.2 s final
    phase under the space, the 3.2 s and 4.2 s release goals, the 1.9 s goal under 'q'; at RTF
    0.1-1.0, with the node's status cadence (2 Hz and on every state change)."""

    def run_actual(self, rtf=0.6, **kw):
        return self.run_case(rtf, library=actual_library(), **kw)

    def test_the_plan_goals_carry_the_frozen_durations(self):
        lead, durations = frozen_durations()
        lib, m = actual_library(), load_driver()
        used = set()
        for level, b0, b1, d in m.PLANNED:
            k = b0 if d > 0 else b1
            g = lib.goal(level, b0, d)
            self.assertEqual((g.level, g.b_from, g.b_to), (level, b0, b1))
            self.assertAlmostEqual(g.duration_s, lead + durations[level][k], places=9)
            used.add((level, k, d, round(g.duration_s, 6)))
        self.assertTrue({(2, 2, 1, 0.8), (2, 6, 1, 0.8), (2, 2, -1, 0.8), (2, 6, -1, 0.8),
                         (2, 8, 1, 1.2), (2, 0, -1, 1.9), (2, 4, 1, 3.2),
                         (2, 5, -1, 4.2), (2, 3, 1, 4.7), (1, 1, 1, 3.2)} <= used, used)
        self.assertEqual([round(sum(lib.goal(L, b, 1).duration_s for b in range(9)), 6)
                          for L in (0, 1, 2)], [18.8, 21.1, 25.6])
        self.assertEqual([f'{v * 1000:.2f}' for v in lib.speeds_m_s], ['1.06', '1.90', '2.34'])

    def test_complete_script_at_the_frozen_durations_across_the_rtf_range(self):
        lead, durations = frozen_durations()
        for rtf in (0.1, 0.17, 0.3, 0.6, 1.0):
            with self.subTest(rtf=rtf):
                rc, why, bench, d, events = self.run_actual(rtf)
                s = self.check_complete(rc, why, bench, d, events)
                for g in self.of(events, 'goal_sent'):
                    k = g['b_from'] if g['direction'] > 0 else g['b_to']
                    self.assertAlmostEqual(g['duration_s'], lead + durations[g['level']][k],
                                           places=9)
                self.assertEqual((s['selection']['confirmations'][0]['speed_mm_s'],
                                  s['speed_change']['speed_mm_s']), ('1.90', '2.34'))
                tr = self.of(events, 'transition')
                self.assertEqual(tr[-1]['why'], 'lease_expired')     # q early in the 1.9 s goal

    def test_late_actions_at_the_frozen_durations_produce_an_off_plan_goal_and_stop(self):
        def keep_repeating(d, bench):
            d._stop_repeat = lambda: d.log(note='test: the repeat continued')

        def late_release(d, bench):
            original = d.release

            def release(why, step=None):
                if step == 'X_release':
                    d.log(note='test: the release never reached the teleop')
                    d.releases.append({'step': step})
                    return
                original(why, step)
            d.release = release

        def q_unread(d, bench):
            orig = d._stop_repeat_timed

            def timed(why):
                if why.startswith('Q:'):
                    return {'key': repr(d.hold_key), 'why': why, 't_before_stop': 0.0,
                            't_after_stop': 0.0}
                return orig(why)
            d._stop_repeat_timed = timed
        cases = (
            ('space after the final 1.2 s phase', {'patch': keep_repeating,
                                                   'delay': {(' ', 1): 30.0}},
             'goal 55 is (2, 0, 1, 1), planned (2, 9, 8, -1)', 55),
            ('release never in effect', {'patch': late_release},
             'goal seq 105 above the step budget (104)', 105),
            ('switch not in effect at boundary 6',
             {'write_hook': nth_write('s', 2, lambda b: b.swap.update(s='w'))},
             'goal 106 is (2, 6, 7, 1), planned (2, 6, 5, -1)', 106),
            ('q never read, w still held', {'ignore': {'q'}, 'patch': q_unread}, 'goal 112',
             112))
        for name, kw, text, n in cases:
            with self.subTest(case=name):
                rc, why, bench, d, events = self.run_actual(1.0, **kw)
                self.assert_stopped(rc, why, bench, events, text)
                self.assertEqual(bench.plant.sent, n)

    def test_missing_and_duplicate_evidence_at_the_frozen_durations(self):
        rc, why, bench, d, events = self.run_actual(0.3, duplicate=lambda n: True)
        self.check_complete(rc, why, bench, d, events)
        stale_sims = []

        def stale(st, n):                    # the first 6 statuses after the count reaches 1
            if st.get('cycles') == 1 and len(stale_sims) < 6 and st.get('ground_truth'):
                stale_sims.append(st['monitors']['sim_time_s'])
                st['monitors']['body_pose_age_s'] = 5.0
            return st
        rc, why, bench, d, events = self.run_actual(0.3, status_edit=stale)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertGreater(d.cycle_log[0]['sim'], max(stale_sims))

        def blind_f1(st, n):
            if st.get('cycles') == 2:
                st['monitors']['body_pose_age_s'] = 5.0
            return st
        rc, why, bench, d, events = self.run_actual(0.3, status_edit=blind_f1)
        self.assert_stopped(rc, why, bench, events, 'cycle count 1 -> 3: not one forward cycle')


class Shadow(Base):
    gate = False

    def test_shadow_rehearsal_runs_the_whole_script_with_nothing_dispatched(self):
        rc, why, bench, d, events = self.run_case(
            shadow=True, fwd=lambda world, py: -3.28e-10 * world.s.phases_completed / 9)
        self.assertEqual((rc, why), (0, 'script complete (shadow rehearsal: dispatch disabled; '
                                        'not acceptance)'))
        self.assertEqual((bench.plant.sent, bench.s.transport.sent), (0, 111))
        s = d.summary()
        self.assertEqual(len(s['cycles']), 12)
        self.assertTrue(all(c.get('progress_sign_bound') == self.mod.SHADOW_SIGN_NOTE
                            for c in s['cycles']))

    def test_a_status_that_is_not_shadow_stops_the_rehearsal(self):
        def gate_true(st, n):
            if st.get('cycles') == 3:
                st['dispatch_gate'] = True
            return st
        rc, why, bench, d, events = self.run_case(shadow=True, status_edit=gate_true)
        self.assert_stopped(rc, why, bench, events, 'shadow rehearsal: node status')


class Constants(unittest.TestCase):
    def test_s4d_limits_and_plan_as_decided(self):
        m = load_driver()
        self.assertEqual((m.EXPECTED_GOALS, m.EXPECTED_CANCELS), (111, 0))
        x = m.X_RELEASE + m.X_REVERSAL + m.X_HOME
        self.assertEqual((len(m.SC), len(m.F), len(m.R), len(x), len(m.Q)), (9, 45, 45, 11, 1))
        self.assertEqual((m.PLANNED[1], m.PLANNED[9], m.PLANNED[54], m.PLANNED[98],
                          m.PLANNED[105], m.PLANNED[110]),
                         ((1, 1, 2, 1), (2, 0, 1, 1), (2, 9, 8, -1), (2, 1, 0, -1),
                          (2, 6, 5, -1), (2, 0, 1, 1)))
        self.assertEqual((m.TRAVEL_MAX_M, m.CUM_LATERAL_STOP_M, m.CUM_YAW_STOP_RAD,
                          m.SCRIPT_CAP_WALL_S), (0.40, 0.050, 0.15, 5400.0))
        self.assertEqual((m.LATERAL_MAX_M, m.YAW_MAX_RAD), (0.020, 0.10))
        self.assertEqual((m.s3.TRAVEL_MAX_M, m.s3.LATERAL_MAX_M, m.s3.YAW_MAX_RAD),
                         (0.15, 0.020, 0.10))                       # shared constants unchanged
        self.assertEqual(m.MAIN_KEYS.count("'q'") + m.PROBE_KEYS.count("'q'"), 2)


if __name__ == '__main__':
    unittest.main()
