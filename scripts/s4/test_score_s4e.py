#!/usr/bin/env python3
"""The S4e scorer (score_s4e.py; docs/S4E_CONTRACT.md section 8) on BENCH SESSIONS, then mutated.

The bench is the real M5.5 node logic (LocomotionSession, with its ShadowTransport, or the
dispatching fake with the gate True in memory for these tests only), the real teleop key logic
(KeyDecoder, TeleopState, the quit's two stop/disarm pairs), the free-base fake world and the
frozen library's level-0 phase durations, on one simulated clock. Key streams are generated with
a terminal-like auto-repeat (a press, a delay, then a steady rate).

SYNTHETIC SCORER FIXTURES, NOT EVIDENCE: the observer records built here (including those
labelled input_source 'human-tty', which the PASS path needs) exist only in memory to test the
scorer's rules. Nothing here is human input or an S4e session. Each mutation must give the
contracted verdict: a contradiction FAILs, missing evidence is UNMEASURED, never PASS.

Usage: cd scripts/s4 && python3 -m unittest -v test_score_s4e
(SPIDERX_REPO must point at the checkout; the direct runs set it)"""
import collections
import copy
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 's3')]
import test_s4d_driver_live as s4dt  # noqa: E402  (the frozen-duration library)
import test_s3_driver_live as s3t  # noqa: E402  (the free-base fake world)
import score_s4e as se  # noqa: E402

fx, fk, loc, tp, c55 = s3t.fx, s3t.fk, s3t.loc, s3t.tp, s3t.c55
RESTORE = os.path.join(HERE, '..', '..', 'docs', 'evidence', 'm55', 's4d_live_20261111T052817Z',
                       'restore', 'restore_verify_20261111T060321Z.txt')
RUN_INSTALL = {'install_base': '/x/install', 'gates': {g: False for g in se.s4d.GATES}}
ATTESTATION = ('I, AB, operated the S4e session on 2026-12-01 by pressing keys on a physical '
               'keyboard myself. No script, macro, key remapper or other program generated any '
               'key.\n')
OBS_PID, TELEOP_PID = 4242, 4343
_CACHE = {}


class Bench:
    """One in-memory session: node, teleop key logic and free base on a simulated clock."""

    def __init__(self, rtf=0.6, shadow=True):
        lib = s4dt.actual_library()
        self.cfg = fx.config()
        self.plant = fk.FakeLocomotionTransport(lib, lib.boundary_positions(0, 0), scenario='ok',
                                                fail_phase=1, drift_rad_s=0.05, use_sim=True)
        self.s = loc.LocomotionSession(self.cfg, lib, loc.ShadowTransport(lib.fingerprints)
                                       if shadow else self.plant)
        self.s.log = collections.deque(maxlen=None)
        self.world = s3t.FreeBaseWorld(self.s, self.plant, {}, rtf=rtf, graph_period_s=1.0)
        self.teleop, self.dec = tp.TeleopState(self.cfg.teleop.hold_s), tp.KeyDecoder()
        self.alive, self.quit_t = True, None
        self.obs, self.joints, self.poses = [], [], []
        self.feedback = collections.defaultdict(list)
        self.next_status = self.world.wall
        self.shadow = shadow

    @property
    def wall(self):
        return self.world.wall

    def tick(self):
        w = self.world.wall
        self.world.cmd = self.teleop.twist(w) if self.alive else None
        self.world.tick()
        w = self.world.wall
        self.joints.append((w, dict(self.plant.joint_positions())))
        self.poses.append((w, {'xy': (self.world.x, self.world.y), 'yaw': self.world.yaw,
                               'z': s3t.Z0, 'tilt': 0.01}))
        g = self.s.goal
        if not self.shadow and g is not None and g.accepted and g.result is None:
            self.feedback[f'id{g.seq}'].append((w, 0.002))
        if w >= self.next_status - 1e-9:
            st = json.loads(json.dumps(self.s.status(w), sort_keys=True, default=repr))
            if self.alive:
                self.teleop.on_status(st)
            self.obs.append({'kind': 'status', 'mono': w, 't_receipt': w, 'status': st})
            self.next_status = w + 0.5

    def run_until(self, pred, cap_s=200.0):
        end = self.wall + cap_s
        while not pred():
            if self.wall > end:
                raise AssertionError(f'bench wait cap: state {self.s.state}')
            self.tick()

    def idle(self, s):
        end = self.wall + s
        self.run_until(lambda: self.wall >= end - 1e-9)

    def type(self, data):
        """Bytes delivered to the observer now, relayed to the teleop at once."""
        t = self.wall
        actions = [a for a, _ in self.dec.decode(data.decode())]
        self.obs.append({'kind': 'key', 'mono': t, 't_read': t, 't_written': t + 0.0002,
                         'hex': data.hex(), 'n': len(data), 'actions': actions})
        for a in actions:
            self.handle(a, t + 0.0002)

    def handle(self, action, wall):
        if not self.alive:
            return
        service, msg = self.teleop.key(action, wall)
        if msg and 'key repeat gap' in msg:
            self.obs.append({'kind': 'teleop_out', 'mono': wall, 't': wall, 'text': msg + '\r\n'})
        if action == tp.QUIT:                    # handle(QUIT), then the shutdown sequence
            self.quit_t = wall
            for name in (tp.STOP, tp.DISARM, tp.STOP, tp.DISARM):
                self.s.request(name, wall)
            self.alive = False
        elif service:
            self.s.request(service, wall)

    def hold(self, key, until, delay=0.4, interval=1 / 30, inner=(), stall=None):
        """Hold `key` (terminal auto-repeat) until until(seconds held) is True. inner:
        [(offset s, bytes)] typed during the hold; stall: (from s, to s) with no repeat."""
        start, next_rep, first = self.wall, self.wall + delay, True
        pending = sorted(inner)
        while True:
            now, chunk = self.wall, b''
            if first:
                chunk, first = key, False
            while next_rep <= now + 1e-9:
                if stall is None or not stall[0] <= next_rep - start < stall[1]:
                    chunk += key
                next_rep += interval
            while pending and start + pending[0][0] <= now + 1e-9:
                chunk += pending.pop(0)[1]
            if chunk:
                self.type(chunk)
            if until(now - start):
                return
            self.tick()

    def state(self):
        return self.s.state


def nominal(bench, k2_s=11.0, delay=0.4, space_at=3.0, after_ready_s=2.0, x_at=2.0,
            stall=None, space_while_ready=False):
    """K1-K6 as the contract's section 4.2, on the bench."""
    b = bench
    b.idle(4.0)
    b.type(b'a')
    b.run_until(lambda: b.state() == 'READY')
    b.idle(1.0)
    b.hold(b'w', lambda t: t >= k2_s, delay=delay, stall=stall)                    # K2
    b.run_until(lambda: b.state() == 'READY')
    b.idle(3.0)
    seen = {}

    def k3_done(t):
        if t >= space_at + 0.1 and b.state() == 'READY':
            seen.setdefault('ready', t)
        return 'ready' in seen and t >= seen['ready'] + after_ready_s
    if space_while_ready:
        b.type(b' ')
        b.idle(0.5)
        b.hold(b'w', lambda t: t >= 2.0)
    else:
        b.hold(b'w', k3_done, delay=delay, inner=[(space_at, b' ')])               # K3
    b.run_until(lambda: b.state() == 'READY')
    b.idle(3.0)
    b.type(b'h')                                                                     # K4
    b.idle(0.2)
    b.run_until(lambda: b.state() == 'READY')
    b.idle(3.0)
    b.hold(b'w', lambda t: t >= x_at + 0.1, delay=delay, inner=[(x_at, b'x')])      # K5
    b.run_until(lambda: b.state() == 'FAULTED' and b.s.goal is None)
    sim0 = b.world.sim
    b.run_until(lambda: b.world.sim >= sim0 + 11.0)
    b.idle(1.0)
    b.type(b'q')                                                                     # K6
    b.idle(2.0)


def observer_frame(b, input_source='human-tty'):
    """The observer records the bench session implies (synthetic scorer fixtures)."""
    t0, t1 = b.obs[0]['mono'], b.wall
    relayed = sum(r['n'] for r in b.obs if r['kind'] == 'key')
    holders = {'slave': '/dev/pts/9',
               'master_holders': [{'pid': OBS_PID, 'cmd': ['python3', 's4e_observer.py']}],
               'slave_holders': [{'pid': TELEOP_PID, 'sid': TELEOP_PID, 'cmd': ['ros2']}]}
    head = [{'kind': 'start', 'mono': t0 - 1.0, 'schema': 'spiderx.s4e_observer/1',
             'mode': 'shadow' if b.shadow else 'live', 'input_source': input_source,
             'tty': {'stdin_tty': True, 'stdout_tty': True, 'foreground': True},
             'observer_pid': OBS_PID},
            {'kind': 'teleop_started', 'mono': t0 - 0.9, 'pid': TELEOP_PID, 't': t0 - 0.9},
            {'kind': 'pty_holders', 'mono': t0 - 0.8, 'at': 'start', **holders},
            {'kind': 'driver_scan', 'mono': t0 - 0.8, 'at': 'start', 'drivers': []}]
    tail = [{'kind': 'teleop_exit', 'mono': b.quit_t + 0.6, 't': b.quit_t + 0.6, 'code': 0},
            {'kind': 'pty_holders', 'mono': t1, 'at': 'end', **{**holders, 'slave_holders': []}},
            {'kind': 'driver_scan', 'mono': t1, 'at': 'end', 'drivers': []},
            {'kind': 'end', 'mono': t1, 'summary': {
                'result': 'session complete', 'input_source': input_source,
                'relay': {'bytes_read': relayed, 'bytes_relayed': relayed}}}]
    return head + sorted(b.obs, key=lambda r: r['mono']) + tail


def preflight(verdict='PASS', min_hold_s=10.0, source='human-tty'):
    return {'schema': 'spiderx.s4e_preflight/1', 'input_source': source,
            'human_check': source == 'human-tty', 'min_hold_s': min_hold_s,
            'environment': {'queries': {'xset_q': 'not available (no DISPLAY)'}},
            'verdict': {'verdict': verdict, 'qualifying_hold': {'first_repeat_delay_s': 0.4,
                                                                'rate_hz': 30.0}}}


def bench_session(shadow, **kw):
    key = (shadow, tuple(sorted(kw.items())))
    if key not in _CACHE:
        saved = c55.M55_LOCOMOTION_DISPATCH_ENABLED
        c55.M55_LOCOMOTION_DISPATCH_ENABLED = not shadow          # tests only, in memory
        try:
            b = Bench(shadow=shadow)
            nominal(b, **kw)
        finally:
            c55.M55_LOCOMOTION_DISPATCH_ENABLED = saved
        ev = json.loads(json.dumps(list(b.s.log), default=repr))
        n = len([e for e in ev if e['event'] == 'goal_sent'])
        bag = {'poses': b.poses, 'joints': b.joints, 'feedback': dict(b.feedback),
               'ctl': {'received': 0 if shadow else n, 'accepted': 0 if shadow else n,
                       'cancel_requests': 0 if shadow else 1},
               'support': {'recorder_log_without_failure': True,
                           'recording_covers_the_session': True},
               'command_topic_count': 0, 'capture_before': 0, 'capture_after': 0,
               'min_joint_margin': 0.3}
        _CACHE[key] = (ev, observer_frame(b), bag)
    return copy.deepcopy(_CACHE[key])


FILES = {'node_events': True, 'final_status': True, 'library_json': True, 'bag_sha256': True,
         'captures': True, 'harness_scans': True}


def scored(ev, obs, bag, shadow, pre=None, att=ATTESTATION, scans=None, drv=False, files=None,
           restore=True):
    c = se.context(ev, obs, preflight() if pre is None else pre, att,
                   {'start': [], 'end': []} if scans is None else scans, drv,
                   FILES if files is None else files)
    with open(RESTORE) as f:
        rt = f.read() if restore else None
    return se.score(c, bag, shadow, rt, 'same', 'same', RUN_INSTALL)


def V(r):
    return {k: v['verdict'] for k, v in r['H'].items()}


class Base(unittest.TestCase):
    shadow = True

    def session(self, **kw):
        return bench_session(self.shadow, **kw)

    def score(self, *a, **kw):
        return scored(*a, shadow=self.shadow, **kw)

    def assert_only(self, r, want):
        """want: {criterion: verdict}; every other criterion PASS (or N/A)."""
        got = V(r)
        for k, v in got.items():
            self.assertEqual(v, want.get(k, 'PASS'), (k, r['H'][k]))


class Nominal(Base):
    def test_shadow_session_passes_as_shadow_only(self):
        ev, obs, bag = self.session()
        r = self.score(ev, obs, bag)
        self.assert_only(r, {})
        self.assertEqual(r['overall'], 'PASS (SHADOW ONLY: not S4e acceptance)')
        self.assertEqual(r['H']['H9']['tracking'], se.SHADOW_NA)
        self.assertEqual(r['H']['H7']['qualifier_not_scored'], se.SHADOW_NA)

    def test_the_bench_session_follows_the_plan(self):
        ev, obs, bag = self.session()
        c = se.context(ev, obs)
        k = se.plan_steps(c)
        self.assertIsNotNone(k)
        self.assertGreaterEqual(k['K2']['times'][-1] - k['K2']['times'][0], 11.0)
        d = scored(ev, obs, bag, True)['H']['H4']['detection_s']
        self.assertTrue(0.80 < d <= 0.95 + 1e-9, d)


class Live(Base):
    shadow = False

    def test_live_session_passes(self):
        ev, obs, bag = self.session()
        r = self.score(ev, obs, bag)
        self.assert_only(r, {})
        self.assertEqual(r['overall'], 'PASS')
        q = r['H']['H7']['qualifier_not_scored']
        self.assertIn(q['reads'], ('moving-body stop', 'before measurable motion, as S4d-E'))

    def test_tracking_limits_and_the_finding(self):
        ev, obs, bag = self.session()
        bag['feedback'] = {'id1': [(obs[5]['mono'], 0.051)]}
        self.assert_only(self.score(ev, obs, bag), {'H9': 'FAIL'})
        bag['feedback'] = {'id1': [(obs[5]['mono'], 0.03)]}
        r = self.score(ev, obs, bag)
        self.assert_only(r, {})
        self.assertIn('finding', r['H']['H9'])
        bag['feedback'] = {}
        self.assert_only(self.score(ev, obs, bag), {'H9': 'UNMEASURED'})

    def test_controller_traffic_must_match_the_node(self):
        ev, obs, bag = self.session()
        bag['ctl']['cancel_requests'] = 2
        self.assert_only(self.score(ev, obs, bag), {'H10': 'FAIL'})
        bag['ctl'] = None
        self.assert_only(self.score(ev, obs, bag), {'H10': 'UNMEASURED'})

    def test_succeeded_despite_the_cancel_is_unmeasured_and_aborted_fails(self):
        ev, obs, bag = self.session()
        res = [e for e in ev if e['event'] == 'result']
        last = res[-1]
        last['status'] = 4                          # action_msgs GoalStatus SUCCEEDED
        self.assert_only(self.score(ev, obs, bag), {'H7': 'UNMEASURED'})
        last['status'] = 6                          # ABORTED
        self.assert_only(self.score(ev, obs, bag), {'H7': 'FAIL'})


class Provenance(Base):
    def test_scripted_input_fails_h1_and_every_human_criterion(self):
        ev, obs, bag = self.session()
        obs[0]['input_source'] = 'scripted-pty-test'
        r = self.score(ev, obs, bag)
        want = {'H1': 'FAIL', **{k: 'FAIL' for k in se.HUMAN_CRITERIA}}
        self.assert_only(r, want)
        for k in se.HUMAN_CRITERIA:
            self.assertEqual(r['H'][k]['underlying_verdict'], 'PASS')
            self.assertEqual(r['H'][k]['human_rule'], se.SCRIPTED_RULE)
        self.assertTrue(r['overall'].startswith('FAIL'))

    def test_other_scripted_markers_fail(self):
        ev, obs, bag = self.session()
        for kw in ({'pre': preflight(source='scripted-pty-test')}, {'drv': True}):
            self.assertEqual(V(self.score(ev, obs, bag, **kw))['H1'], 'FAIL', kw)
        obs2 = copy.deepcopy(obs)
        obs2[0]['input_source'] = 'test-fixture'
        self.assertEqual(V(self.score(ev, obs2, bag))['H3'], 'FAIL')

    def test_missing_provenance_is_unmeasured_and_blocks_every_human_criterion(self):
        ev, obs, bag = self.session()
        r = self.score(ev, obs, bag, att=None)
        self.assert_only(r, {'H1': 'UNMEASURED', 'H12': 'UNMEASURED',
                             **{k: 'UNMEASURED' for k in se.HUMAN_CRITERIA}})
        self.assertTrue(r['overall'].startswith('INCONCLUSIVE'))

    def test_attestation_driver_scan_terminal_and_holders(self):
        ev, obs, bag = self.session()
        self.assertEqual(V(self.score(ev, obs, bag, att='I typed it.'))['H1'], 'FAIL')
        self.assertEqual(V(self.score(ev, obs, bag, scans={'start': [{'pid': 1}],
                                                           'end': []}))['H1'], 'FAIL')
        for mutate in (lambda o: o[0]['tty'].update(foreground=False),
                       lambda o: o[2]['master_holders'].append({'pid': 7}),
                       lambda o: o[2]['slave_holders'].append({'pid': 8, 'sid': 8}),
                       lambda o: o[3].update(drivers=[{'pid': 9, 'cmd': ['s4d_driver.py']}])):
            o = copy.deepcopy(obs)
            mutate(o)
            self.assertEqual(V(self.score(ev, o, bag))['H1'], 'FAIL')


class Plan(Base):
    def test_a_forbidden_key_fails_the_plan(self):
        ev, obs, bag = self.session()
        k = next(i for i, r in enumerate(obs) if r['kind'] == 'key' and r['actions'] == ['home'])
        obs[k]['hex'] = (b's').hex()
        obs[k]['actions'] = ['reverse']
        r = self.score(ev, obs, bag)
        self.assertEqual(V(r)['H2'], 'FAIL')

    def test_an_escape_sequence_fails_the_plan(self):
        ev, obs, bag = self.session()
        k = next(r for r in obs if r['kind'] == 'key' and r['actions'] == ['arm'])
        k['hex'] = (b'a\x1b[A').hex()
        k['n'] = 4
        k['actions'] = ['arm', 'ignored']
        self.assertEqual(V(self.score(ev, obs, bag))['H2'], 'FAIL')

    def test_a_short_k2_fails_the_plan_and_leaves_h3_unmeasured(self):
        ev, obs, bag = self.session(k2_s=8.0)
        r = self.score(ev, obs, bag)
        self.assertEqual(V(r)['H2'], 'FAIL')
        self.assertEqual(V(r)['H3'], 'UNMEASURED')

    def test_refusal_level_budget_fault_rejection_and_mode(self):
        ev, obs, bag = self.session()
        t_a = next(e['wall'] for e in ev if e['event'] == 'request' and e['request'] == 'arm')
        cases = [
            lambda e, o: e.append({'event': 'refused', 'request': 'home', 'reason': 'x',
                                   'wall': t_a + 0.1, 'state': 'READY'}),
            lambda e, o: e.append({'event': 'level_change', 'wall': t_a + 0.1, 'level': 1}),
            lambda e, o: [g.update(level=1) for g in e if g['event'] == 'goal_sent'][:1],
            lambda e, o: e.append({'event': 'fault', 'reason': 'stale_pose', 'wall': t_a + 0.2}),
            lambda e, o: e.append({'event': 'command_rejected', 'wall': t_a + 0.2, 'code': 1}),
            lambda e, o: next(r for r in o if r['kind'] == 'status')['status'].update(
                mode='dispatch'),
            lambda e, o: o[-1]['summary'].update(result='STOP CONDITION: tilt'),
        ]
        for i, mut in enumerate(cases):
            e2, o2 = copy.deepcopy(ev), copy.deepcopy(obs)
            mut(e2, o2)
            e2.sort(key=lambda x: x['wall'])
            self.assertEqual(V(self.score(e2, o2, bag))['H2'], 'FAIL', i)

    def test_more_than_30_goals_fails(self):
        ev, obs, bag = self.session()
        g = next(e for e in ev if e['event'] == 'goal_sent')
        extra = [{**g, 'seq': 100 + i} for i in range(31)]
        self.assertEqual(V(self.score(ev + extra, obs, bag))['H2'], 'FAIL')


class Repeat(Base):
    def test_a_stall_in_the_hold_fails_repeat_and_shows_in_the_node(self):
        ev, obs, bag = self.session(stall=(5.0, 6.0))
        r = self.score(ev, obs, bag)
        self.assertEqual(V(r)['H3'], 'FAIL', r['H']['H3'])
        self.assertFalse(r['H']['H3']['checks']['k2_every_gap_below_0_8_s'])

    def test_a_slow_first_repeat_fails(self):
        ev, obs, bag = self.session(delay=0.9)
        r = self.score(ev, obs, bag)
        self.assertEqual(V(r)['H3'], 'FAIL', r['H']['H3'])

    def test_a_key_repeat_gap_line_fails(self):
        ev, obs, bag = self.session()
        c = se.context(ev, obs)
        t = se.plan_steps(c)['K2']['times'][3]
        obs.append({'kind': 'teleop_out', 'mono': t, 't': t, 'text': 'key repeat gap 900 ms'})
        obs.sort(key=lambda r: r['mono'])
        self.assertEqual(V(self.score(ev, obs, bag))['H3'], 'FAIL')

    def test_the_preflight_decides_or_is_unmeasured(self):
        ev, obs, bag = self.session()
        self.assertEqual(V(self.score(ev, obs, bag, pre=preflight('FAIL')))['H3'], 'FAIL')
        for pre in (preflight('UNMEASURED'), preflight(min_hold_s=1.0)):
            self.assertEqual(V(self.score(ev, obs, bag, pre=pre))['H3'], 'UNMEASURED')


class Release(Base):
    def stop_tr(self, ev, obs):
        c = se.context(ev, obs)
        t_last = se.plan_steps(c)['K2']['times'][-1]
        return next(e for e in ev if e['event'] == 'transition' and e['wall'] > t_last), t_last

    def test_band_edges(self):
        # just inside and just outside each inclusive edge (exact float edges are not exact)
        for delta, want in ((0.7999, 'FAIL'), (0.8001, 'PASS'), (0.9999, 'PASS'),
                            (1.0001, 'FAIL')):
            ev, obs, bag = self.session()
            tr, t_last = self.stop_tr(ev, obs)
            tr['wall'] = t_last + delta
            self.assertEqual(V(self.score(ev, obs, bag))['H4'], want, delta)

    def test_a_wrong_first_transition_fails_and_none_is_unmeasured(self):
        ev, obs, bag = self.session()
        tr, _ = self.stop_tr(ev, obs)
        tr['why'] = 'stop'
        self.assertEqual(V(self.score(ev, obs, bag))['H4'], 'FAIL')
        ev, obs, bag = self.session()
        tr, t_last = self.stop_tr(ev, obs)
        ev2 = [e for e in ev if not (e['event'] == 'transition' and e['wall'] > t_last)]
        self.assertIn(V(self.score(ev2, obs, bag))['H4'], ('UNMEASURED',))

    def test_a_goal_before_k3_fails(self):
        ev, obs, bag = self.session()
        tr, _ = self.stop_tr(ev, obs)
        g = next(e for e in ev if e['event'] == 'goal_sent')
        ev.append({**g, 'seq': 99, 'wall': tr['wall'] + 0.5})
        ev.sort(key=lambda e: e['wall'])
        self.assertEqual(V(self.score(ev, obs, bag))['H4'], 'FAIL')


class StopLatch(Base):
    def test_space_at_ready_is_unmeasured(self):
        ev, obs, bag = self.session(space_while_ready=True)
        r = self.score(ev, obs, bag)
        self.assertEqual(V(r)['H5'], 'UNMEASURED', r['H']['H5'])

    def test_latch_not_exercised_is_unmeasured(self):
        ev, obs, bag = self.session(after_ready_s=0.5)
        r = self.score(ev, obs, bag)
        self.assertEqual(V(r)['H5'], 'UNMEASURED', r['H']['H5'])

    def test_a_goal_after_the_stop_before_h_fails(self):
        ev, obs, bag = self.session()
        c = se.context(ev, obs)
        k = se.plan_steps(c)
        g = next(e for e in ev if e['event'] == 'goal_sent')
        ev.append({**g, 'seq': 98, 'wall': k['K4'] - 0.5})
        ev.sort(key=lambda e: e['wall'])
        self.assertEqual(V(self.score(ev, obs, bag))['H5'], 'FAIL')

    def test_no_stopping_on_the_request_fails(self):
        ev, obs, bag = self.session()
        c = se.context(ev, obs)
        sp = se.plan_steps(c)['space']
        tr = next(e for e in ev if e['event'] == 'transition' and e['wall'] >= sp
                  and e['why'] == 'stop')
        tr['wall'] += 0.06
        self.assertEqual(V(self.score(ev, obs, bag))['H5'], 'FAIL')


class Home(Base):
    def test_home_at_boundary_0_branch(self):
        ev, obs, bag = self.session()
        c = se.context(ev, obs)
        r = se.assess_home(c)
        self.assertEqual(r['verdict'], 'PASS', r)
        req = next(e for e in ev if e['event'] == 'request' and e['request'] == 'home')
        if r['case'] == 'homing':                 # turn it into the boundary-0 case
            t = req['wall']
            ready = next(e['wall'] for e in ev if e['event'] == 'transition' and e['wall'] > t
                         and e['state'] == 'READY')
            ev = [e for e in ev if not (e['event'] in ('homing', 'goal_sent') and
                                        t - 0.01 <= e['wall'] <= ready)]
            for e in ev:
                if e is not req and e['event'] == 'transition' and t <= e['wall'] <= ready:
                    e['event'] = 'dropped'
            req = next(e for e in ev if e['event'] == 'request' and e['request'] == 'home')
            req['message'] = se.HOME_AT_ZERO
            r = se.assess_home(se.context(ev, obs))
            self.assertEqual(r['case'], 'boundary 0')
            self.assertEqual(r['verdict'], 'PASS', r)

    def test_home_not_at_ready_or_too_many_goals_fails(self):
        ev, obs, bag = self.session()
        r0 = se.assess_home(se.context(ev, obs))
        if r0['case'] != 'homing':
            self.skipTest('this bench session homed from boundary 0')
        req = next(e for e in ev if e['event'] == 'request' and e['request'] == 'home')
        g = next(e for e in ev if e['event'] == 'goal_sent' and e['wall'] >= req['wall'])
        extra = [{**g, 'seq': 90 + i, 'wall': g['wall'] + 0.01 * (i + 1)} for i in range(4)]
        ev2 = sorted(ev + extra, key=lambda e: e['wall'])
        self.assertEqual(se.assess_home(se.context(ev2, obs))['verdict'], 'FAIL')


class Estop(Base):
    def test_two_cancels_or_a_late_fault_fail(self):
        ev, obs, bag = self.session()
        cs = next(e for e in ev if e['event'] == 'cancel_sent')
        ev2 = sorted(ev + [{**cs, 'wall': cs['wall'] + 0.01}], key=lambda e: e['wall'])
        self.assertEqual(V(self.score(ev2, obs, bag))['H7'], 'FAIL')
        ev, obs, bag = self.session()
        tr = next(e for e in ev if e['event'] == 'transition' and e['state'] == 'FAULTED')
        tr['wall'] += 0.051
        self.assertEqual(V(self.score(ev, obs, bag))['H7'], 'FAIL')

    def test_hold_motion_fails_and_a_coverage_gap_is_unmeasured(self):
        ev, obs, bag = self.session()
        c = se.context(ev, obs)
        t0, t1 = se.assess_estop(c, bag, self.shadow)['hold_window']
        name = next(iter(bag['joints'][0][1]))
        bag2 = copy.deepcopy(bag)
        for t, q in bag2['joints']:
            if t0 + 2.0 < t <= t1:
                q[name] += 0.02
        self.assertEqual(V(self.score(ev, obs, bag2))['H7'], 'FAIL')
        bag3 = copy.deepcopy(bag)
        bag3['joints'] = [(t, q) for t, q in bag3['joints'] if not t0 + 2.0 < t < t0 + 4.0]
        self.assertEqual(V(self.score(ev, obs, bag3))['H7'], 'UNMEASURED')

    def test_a_goal_after_the_fault_fails(self):
        ev, obs, bag = self.session()
        f = next(e for e in ev if e['event'] == 'fault')
        g = next(e for e in ev if e['event'] == 'goal_sent')
        ev2 = sorted(ev + [{**g, 'seq': 97, 'wall': f['wall'] + 1.0}], key=lambda e: e['wall'])
        self.assertEqual(V(self.score(ev2, obs, bag))['H7'], 'FAIL')


class Close(Base):
    def test_close_rules(self):
        ev, obs, bag = self.session()
        for change in ({'code': 1}, {'t_add': 16.0}):
            o2 = copy.deepcopy(obs)
            ex = next(r for r in o2 if r['kind'] == 'teleop_exit')
            if 'code' in change:
                ex['code'] = change['code']
            else:
                ex['t'] += change['t_add']
            self.assertEqual(V(self.score(ev, o2, bag))['H8'], 'FAIL', change)
        q = next(r for r in obs if r['kind'] == 'key' and r['actions'] == ['quit'])
        obs2 = copy.deepcopy(obs)
        obs2.append({**q, 'mono': q['mono'] + 0.3, 't_read': q['t_read'] + 0.3,
                     't_written': q['t_written'] + 0.3, 'hex': (b'w').hex(),
                     'actions': ['forward']})
        obs2.sort(key=lambda r: r['mono'])
        self.assertEqual(V(self.score(ev, obs2, bag))['H8'], 'FAIL')
        ev2 = copy.deepcopy(ev)
        stop = next(e for e in ev2 if e['event'] == 'request' and e['request'] == 'stop'
                    and e['wall'] >= q['t_read'])
        ev2.append({**stop, 'wall': stop['wall'] + 0.001})
        ev2.sort(key=lambda e: e['wall'])
        self.assertEqual(V(self.score(ev2, obs, bag))['H8'], 'FAIL')


class Safety(Base):
    def test_pose_limits(self):
        for mut in (lambda p: p.update(xy=(p['xy'][0], p['xy'][1] + 0.16)),
                    lambda p: p.update(tilt=0.11), lambda p: p.update(z=0.044)):
            ev, obs, bag = self.session()
            mut(bag['poses'][-5][1])
            self.assertEqual(V(self.score(ev, obs, bag))['H9'], 'FAIL')

    def test_joint_margin_ages_and_missing_poses(self):
        ev, obs, bag = self.session()
        bag['min_joint_margin'] = 0.019
        self.assertEqual(V(self.score(ev, obs, bag))['H9'], 'FAIL')
        ev, obs, bag = self.session()
        st = [r for r in obs if r['kind'] == 'status'][-3]['status']
        st['monitors']['body_pose_age_s'] = 1.5
        self.assertEqual(V(self.score(ev, obs, bag))['H9'], 'FAIL')
        ev, obs, bag = self.session()
        bag['poses'] = []
        self.assertEqual(V(self.score(ev, obs, bag))['H9'], 'UNMEASURED')


class Traffic(Base):
    def test_shadow_traffic_rules(self):
        for mut, want in ((lambda b, o: b['ctl'].update(received=1), 'FAIL'),
                          (lambda b, o: b.update(feedback={'id1': [(1.0, 0.0)]}), 'FAIL'),
                          (lambda b, o: b.update(capture_after=1), 'FAIL'),
                          (lambda b, o: b.update(command_topic_count=1), 'FAIL'),
                          (lambda b, o: b.update(support={}), 'UNMEASURED'),
                          (lambda b, o: next(r for r in o if r['kind'] == 'status')['status'][
                              'monitors'].update(graph={'foreign_action_clients': 1,
                                                        'command_topic_publishers': 0}),
                           'FAIL')):
            ev, obs, bag = self.session()
            mut(bag, obs)
            self.assertEqual(V(self.score(ev, obs, bag))['H10'], want)


class RestoreAndEvidence(Base):
    def test_restore_missing_is_unmeasured(self):
        ev, obs, bag = self.session()
        self.assertEqual(V(self.score(ev, obs, bag, restore=False))['H11'], 'UNMEASURED')

    def test_evidence_presence_and_consistency(self):
        ev, obs, bag = self.session()
        self.assertEqual(V(self.score(ev, obs, bag, files={**FILES, 'final_status': False}))[
            'H12'], 'UNMEASURED')
        o2 = copy.deepcopy(obs)
        o2[-1]['summary']['relay']['bytes_relayed'] += 1
        self.assertEqual(V(self.score(ev, o2, bag))['H12'], 'FAIL')
        o3 = copy.deepcopy(obs)
        k = next(r for r in o3 if r['kind'] == 'key')
        k['t_written'] = k['t_read'] - 0.001
        self.assertEqual(V(self.score(ev, o3, bag))['H12'], 'FAIL')


class Overall(unittest.TestCase):
    def test_overall_rule(self):
        H = {'H1': {'verdict': 'PASS'}, 'H2': {'verdict': 'N/A'}}
        self.assertEqual(se.overall(H), 'PASS')
        H['H3'] = {'verdict': 'UNMEASURED'}
        self.assertEqual(se.overall(H), 'INCONCLUSIVE')
        H['H4'] = {'verdict': 'FAIL'}
        self.assertEqual(se.overall(H), 'FAIL')
        self.assertEqual(se.overall({'H1': {'verdict': 'N/A'}}), 'INCONCLUSIVE')

    def test_macro_parsing(self):
        F, S, E = se.FORWARD, se.STOP, se.ESTOP
        toks = [(0, 'arm'), (1, F), (1.5, F), (5, F), (5.5, S), (6.0, F), (9, 'home'), (10, F),
                (10.5, E), (12, 'quit')]
        kinds = [x['kind'] for x in se.macro(toks)]
        self.assertEqual(kinds, ['arm', 'hold', 'hold', 'home', 'hold', 'quit'])
        self.assertEqual(se.macro([(0, F), (1.0, S)])[1]['kind'], S)     # not inside: >= 0.8 s

    def test_the_contract_numbers(self):
        self.assertEqual((se.RELEASE_MIN_S, se.RELEASE_MAX_S), (0.80, 1.00))
        self.assertEqual((se.K2_MIN_S, se.K2_MAX_S, se.MAX_GOALS), (10.0, 20.0, 30))
        self.assertEqual((se.HOLD_SIM_S, se.HOLD_JOINT_MAX_RAD, se.HOME_SIM_S),
                         (10.0, 0.01, 12.0))
        self.assertEqual((se.LATERAL_MAX_M, se.HEADING_MAX_RAD, se.TRAVEL_MAX_M),
                         (0.030, 0.10, 0.15))
        self.assertEqual(se.HOLD_S, fx.config().teleop.hold_s)
        self.assertEqual(se.REPEAT_WINDOW_S, tp.REPEAT_GAP_WINDOW_S)
        self.assertEqual((se.ARM, se.FORWARD, se.STOP, se.HOME, se.ESTOP, se.QUIT),
                         (tp.ARM, tp.FORWARD, tp.STOP, tp.HOME, tp.ESTOP, tp.QUIT))


if __name__ == '__main__':
    unittest.main()
