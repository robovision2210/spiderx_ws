#!/usr/bin/env python3
"""S4b driver against the IN-MEMORY M5.5 node on a simulated free base (no ROS, no simulator).

Reuses, unchanged:
- the S3 bench (scripts/s3/test_s3_driver_live.py): the real LocomotionSession (the dispatching
  fake with the gate True in memory, or the node's ShadowTransport with the gate False); the real
  teleop key logic (hold window, heartbeats, home service); a free base that follows the node's
  planned displacement phase by phase;
- the S4a world hook (scripts/s4/test_s4a_driver_live.py).

The S4b bench replaces only:
- the raw pty write;
- the repeat stop;
- the raw teleop kill: the heartbeats stop, so the node's own lease logic expires.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_s4b_driver_live"""
import importlib.util
import json
import os
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 's3')]
import test_s4a_driver_live as s4at  # noqa: E402  (the S4a bench world, unchanged)

s3t = s4at.s3t
c55, loc, tp, LIMITS, Z0 = s3t.c55, s3t.loc, s3t.tp, s3t.LIMITS, s3t.Z0
CREEP = s4at.CREEP
EXPECTED_TRANSITIONS = [
    ('DISARMED', 'READY', 'arm'),
    ('READY', 'STARTING', 'dispatch'), ('STARTING', 'WALKING', 'accepted'),     # 1 release
    ('WALKING', 'STOPPING', 'intent_stop'), ('STOPPING', 'READY', 'stopped'),
    ('READY', 'STARTING', 'dispatch'), ('STARTING', 'WALKING', 'accepted'),     # 2 reversal
    ('WALKING', 'STOPPING', 'intent_stop'), ('STOPPING', 'READY', 'stopped'),
    ('READY', 'STARTING', 'dispatch'), ('STARTING', 'WALKING', 'accepted'),     # 3 home
    ('WALKING', 'READY', 'homed'),
    ('READY', 'STARTING', 'dispatch'), ('STARTING', 'WALKING', 'accepted'),     # 4 lease
    ('WALKING', 'STOPPING', 'lease_expired'), ('STOPPING', 'DISARMED', 'lease_expired')]


def load_driver():
    spec = importlib.util.spec_from_file_location('s4b_driver_under_test',
                                                  os.path.join(HERE, 's4b_driver.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def make_driver(mod, bench, out_dir, shadow=False, write_hook=None, kill_hook=None):
    class InMemoryS4bDriver(mod.S4bDriver):
        def __init__(self):                                         # no ROS, no threads
            self._init_s3(LIMITS, shadow, {'bench': 'gates checked by main()'} if shadow else None)
            self._init_s4a()
            self._init_s4b()
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
            if bench.teleop is None:
                raise OSError(5, 'Input/output error')              # the pty of a dead teleop
            bench.key(data.decode())
            return len(data)

        def hold(self, k, why):
            self.key(k, why)
            bench.held, bench.held_since, self.hold_key = k, bench.world.wall, k

        def _stop_repeat(self):                                     # the repeat stop only
            bench.held, self.hold_key = None, None

        def _signal_teleop(self):                                   # the raw kill only
            if kill_hook:
                kill_hook(bench)
            bench.teleop, bench.held = None, None                   # heartbeats stop
            bench.killed_at = bench.world.wall

        def _reap_teleop(self):
            pass

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
    mod.s4a.time = bench                      # the inherited S4a, S3 and S2 methods use it too
    mod.s3.time = bench
    mod.s2.time = bench
    d = InMemoryS4bDriver()
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

    def run_case(self, rtf=0.2, patch=None, shadow=False, write_hook=None, kill_hook=None,
                 **kw):
        self.mod = load_driver()
        bench = s4at.make_bench(rtf, shadow=shadow, **kw)
        bench.killed_at = None
        d = make_driver(self.mod, bench, self.tmp.name, shadow=shadow, write_hook=write_hook,
                        kill_hook=kill_hook)
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

    @staticmethod
    def count(events, name):
        return sum(1 for e in events if e.get('event') == name)

    @staticmethod
    def sent(events):
        return [e for e in events if e['event'] == 'goal_sent']

    @staticmethod
    def transitions(events):
        return [(e['previous'], e['state'], e.get('why')) for e in events
                if e['event'] == 'transition']

    def assert_stopped(self, rc, why, bench, events, text):
        self.assertEqual(rc, 2, why)
        self.assertIn(text, why)
        self.assertEqual(bench.s.state, loc.FAULTED)
        self.assertIn('estop', bench.s.faults)
        self.assertLessEqual(self.count(events, 'cancel_sent'), 1)


class Live(Base):
    def test_normal_script_exact_sequence_budget_and_states(self):
        rc, why, bench, d, events = self.run_case()
        self.assertEqual((rc, why), (0, 'script complete'))
        sent = self.sent(events)
        self.assertEqual([(g['b_from'], g['b_to'], g['direction']) for g in sent],
                         self.mod.PLANNED)
        self.assertEqual(sent[6]['phase'], sent[5]['phase'])        # the same phase played back
        self.assertEqual((bench.plant.sent, d.expected_goals, self.mod.EXPECTED_GOALS),
                         (12, 12, 12))
        self.assertEqual({g['level'] for g in sent}, {0})
        self.assertEqual((self.count(events, 'cancel_sent'), self.count(events, 'fault')), (0, 0))
        self.assertEqual(self.transitions(events), EXPECTED_TRANSITIONS)
        self.assertEqual([(e['direction'], e['phases']) for e in events
                          if e['event'] == 'homing'], [(1, 4)])
        self.assertEqual([e['request'] for e in events if e['event'] == 'request'],
                         ['arm', 'home'])
        self.assertEqual((bench.s.state, bench.s.b, bench.s.cycles), (loc.DISARMED, 1, 1))
        self.assertEqual([(c['direction'], c['cycles_count']) for c in d.cycle_log], [(1, 1)])
        self.assertAlmostEqual(d.cycle_log[0]['forward_m'], 0.020, delta=0.002)
        o = d.lease_observation
        self.assertTrue(o['within_A6_limits'])
        self.assertGreaterEqual(o['last_sim_s'] - o['from_sim_s'], 10.0 - 0.5)
        self.assertEqual((o['state_after'], o['goals_recorded_after'], o['max_seq']),
                         ('DISARMED', 12, 12))

    def test_key_writes_are_bracketed_and_none_follows_the_kill(self):
        rc, why, bench, d, events = self.run_case()
        recs = self.records()
        keys = [r for r in recs if 't_before_write' in r]
        self.assertEqual([k['key'] for k in keys], ["'a'", "'w'", "'w'", "'s'", "'h'", "'w'"])
        for k in keys:
            self.assertLessEqual(k['t_before_write'], k['t_after_write'])
            self.assertEqual(k['write_result'], 1)
        kill = d.kills[0]
        self.assertEqual((kill['signal'], kill['result']), ('SIGKILL', 'delivered'))
        self.assertLessEqual(kill['t_before_kill'], kill['t_after_kill'])
        i = next(i for i, r in enumerate(recs) if r.get('event') == 'teleop_killed')
        self.assertFalse([r for r in recs[i:] if 't_before_write' in r or r.get('event') in (
            'teleop_started', 'estop_service')])
        self.assertTrue(all(k['t_after_write'] < kill['t_before_kill'] for k in keys))

    def test_release_is_a_key_release_not_an_explicit_stop(self):
        rc, why, bench, d, events = self.run_case()
        stops = [e for e in events if e['event'] == 'transition' and e['state'] == 'STOPPING']
        self.assertEqual([e['why'] for e in stops], ['intent_stop', 'intent_stop',
                                                     'lease_expired'])
        self.assertFalse([e for e in events if e['event'] == 'request'
                          and e['request'] in ('stop', 'estop', 'disarm')])
        self.assertEqual(len(d.releases), 2)
        for rel, stop in zip(d.releases, stops[:2]):
            self.assertTrue(0.75 <= stop['wall'] - rel['t_after_stop'] <= 1.0)
        for rel in d.releases:
            self.assertIn('(no space)', rel['why'])

    def test_reversal_takes_effect_at_the_next_phase_boundary(self):
        rc, why, bench, d, events = self.run_case()
        g6, g7 = self.sent(events)[5:7]
        done6 = next(e for e in events if e['event'] == 'phase_done'
                     and e['wall'] >= g6['wall'] and e['b'] == 6)
        self.assertTrue(done6['wall'] <= g7['wall'] <= done6['wall'] + 2.0)
        s_key = next(r for r in self.records() if r.get('key') == "'s'")
        self.assertTrue(g6['wall'] <= s_key['t_before_write'] <= done6['wall'])
        self.assertFalse([e for e in events if e['event'] == 'transition'
                          and g6['wall'] < e['wall'] < g7['wall']
                          and e['state'] in ('STOPPING', 'READY')])
        sw = d.switches[0]
        self.assertTrue(sw['t_after_stop'] <= s_key['t_before_write']
                        <= sw['t_after_stop'] + 0.8)

    def test_home_is_forward_from_boundary_5_and_checked_before_the_lease_key(self):
        rc, why, bench, d, events = self.run_case()
        sent = self.sent(events)
        h = next(r for r in self.records() if r.get('key') == "'h'")
        self.assertTrue(all(g['wall'] >= h['t_before_write'] for g in sent[7:11]))
        recs = self.records()
        check = next(i for i, r in enumerate(recs) if 'cycle_check' in r)
        lease_w = next(i for i, r in enumerate(recs) if r.get('why') == 'lease: hold w')
        self.assertLess(check, lease_w)

    def test_lease_loss_disarms_after_the_phase_and_nothing_follows(self):
        rc, why, bench, d, events = self.run_case()
        kill = d.kills[0]
        stop = next(e for e in events if e['event'] == 'transition' and e['state'] == 'STOPPING'
                    and e['why'] == 'lease_expired')
        dis = next(e for e in events if e['event'] == 'transition' and e['state'] == 'DISARMED'
                   and e['wall'] > kill['t_before_kill'])
        self.assertTrue(0.35 <= stop['wall'] - kill['t_before_kill'] <= 0.55 + 1e-6)
        self.assertEqual(dis['why'], 'lease_expired')
        self.assertFalse([e for e in events if e['event'] in ('goal_sent', 'request')
                          and e['wall'] > kill['t_before_kill']])
        self.assertEqual((bench.s.state, bench.s.b, bench.plant.sent), (loc.DISARMED, 1, 12))

    def test_an_extra_goal_from_a_late_release_is_never_hidden(self):
        # a fast plant (RTF 8) puts phase boundaries inside the 0.8 s release window: the node
        # sends a goal beyond the exercise, and the script stops on it (fail closed)
        rc, why, bench, d, events = self.run_case(rtf=8.0)
        self.assertEqual(rc, 2, why)
        self.assertRegex(why, r'above the script|beyond the S4b plan|, planned \(')
        self.assertEqual(bench.s.state, loc.FAULTED)
        self.assertGreater(bench.sent_at_stop, 5)                   # recorded, then stopped

    def test_wrong_direction_and_excessive_progress_stop(self):
        rc, why, bench, d, events = self.run_case(progress_m=-0.020)
        self.assert_stopped(rc, why, bench, events, 'forward cycle moved -20.0 mm')
        self.assertEqual(bench.sent_at_stop, 11)
        rc, why, bench, d, events = self.run_case(progress_m=0.045)
        self.assert_stopped(rc, why, bench, events, 'forward cycle moved 45.0 mm')

    def test_a_goal_different_from_the_plan_stops(self):
        def wrong_home(st, n):
            g = st.get('goal')
            if g and g.get('seq') == 8:
                st['goal'] = {**g, 'b_from': 5, 'b_to': 4, 'direction': -1}
            return st
        rc, why, bench, d, events = self.run_case(status_edit=wrong_home)
        self.assert_stopped(rc, why, bench, events, 'goal 8 is (5, 4, -1), planned (5, 6, 1)')

    def test_dispatch_or_rearm_after_lease_loss_stops_without_a_key(self):
        box = {}

        def keep(d, bench):
            box['d'] = d

        def rearm(st, n):
            if box['d'].lease_phase == 'disarmed':
                st['state'] = 'READY'
            return st

        def redispatch(st, n):
            if box['d'].lease_phase == 'disarmed':
                st['goal'] = {'seq': 13, 'b_from': 1, 'b_to': 2, 'direction': 1, 'level': 0}
            return st

        def ready_after_kill(st, n):
            if box['d'].lease_phase == 'killed' and st['state'] == 'STOPPING':
                st['state'] = 'READY'
            return st
        for edit, text in ((rearm, 'after lease loss: state READY'),
                           (redispatch, 'beyond the S4b plan'),
                           (ready_after_kill, 'after the teleop kill: state READY')):
            rc, why, bench, d, events = self.run_case(patch=keep, status_edit=edit)
            self.assertEqual(rc, 2, why)
            self.assertIn(text, why)
            self.assertEqual(bench.requests[-1][0], 'estop')        # the service: no teleop
            i = next(i for i, r in enumerate(self.records()) if r.get('event') == 'teleop_killed')
            self.assertFalse([r for r in self.records()[i:] if 't_before_write' in r])

    def test_duplicate_statuses_change_nothing(self):
        rc, why, bench, d, events = self.run_case(duplicate=lambda n: True)
        self.assertEqual((rc, why), (0, 'script complete'))
        self.assertEqual((bench.plant.sent, len(d.cycle_log)), (12, 1))

    @staticmethod
    def stale_from_cycle_end(k):
        first = {}

        def edit(st, n):
            if st.get('cycles') == 1:
                first.setdefault('n', n)
            if 'n' in first and n < first['n'] + k:
                st.setdefault('monitors', {})['body_pose_age_s'] = 5.0
            return st
        return edit

    def test_a_delayed_cycle_check_still_precedes_the_lease_key(self):
        rc, why, bench, d, events = self.run_case(status_edit=self.stale_from_cycle_end(6))
        self.assertEqual((rc, why), (0, 'script complete'))
        recs = self.records()
        check = next(i for i, r in enumerate(recs) if 'cycle_check' in r)
        lease_w = next(i for i, r in enumerate(recs) if r.get('why') == 'lease: hold w')
        self.assertLess(check, lease_w)

    def test_no_fresh_sample_after_the_cycle_stops_before_the_lease_exercise(self):
        rc, why, bench, d, events = self.run_case(status_edit=self.stale_from_cycle_end(10 ** 6))
        self.assert_stopped(rc, why, bench, events, 'cycle checked at a fresh sample')
        self.assertEqual(bench.sent_at_stop, 11)
        self.assertEqual(d.kills, [])

    def test_tilt_fall_travel_joint_and_joint_state_protections(self):
        for kw, text in (({'tilt': lambda w, k: 0.15 if k > 0.3 else 0.01}, 'tilt'),
                         ({'z': lambda w, k: 0.040 if k > 0.3 else Z0}, 'body z')):
            rc, why, bench, d, events = self.run_case(**kw)
            self.assert_stopped(rc, why, bench, events, text)

        def far(st, n):
            if st.get('ground_truth') and st['ground_truth'].get('delta_since_arm_xy') \
                    and st.get('state') == 'WALKING':
                st['ground_truth']['delta_since_arm_xy'] = [0.0, 0.16]
            return st
        rc, why, bench, d, events = self.run_case(status_edit=far)
        self.assert_stopped(rc, why, bench, events, 'travel')
        lo, start = LIMITS['rf_foot_joint'][0], {}

        def joint(w):
            start.setdefault('w', w + 60.0)
            return {'rf_foot_joint': lo + 0.015} if w > start['w'] else None
        rc, why, bench, d, events = self.run_case(joint=joint)
        self.assert_stopped(rc, why, bench, events, 'URDF limit')
        box = {}

        def js_off(w):                                              # 2 s, from goal 2 in flight
            b = box.get('b')
            if b is not None and 'w' not in box and b.s.goal is not None and b.s.goal.seq == 2:
                box['w'] = w
            return 'w' in box and box['w'] < w < box['w'] + 2.0
        rc, why, bench, d, events = self.run_case(patch=lambda d, b: box.update(b=b),
                                                  js_off=js_off)
        self.assertEqual(rc, 2, why)
        self.assertIn('joint_states_stale', why)                    # an unplanned fault ends it

    def test_a_level_other_than_0_stops(self):
        def lvl(st, n):
            if st.get('state') == 'WALKING':
                st['level'] = 1
            return st
        rc, why, bench, d, events = self.run_case(status_edit=lvl)
        self.assert_stopped(rc, why, bench, events, 'level 1 reported')

    def test_interruption_timeout_kill_failure_and_driver_error(self):
        def interrupt_at(text):
            def patch(d, bench):
                orig = d.wait

                def w(what, pred, sim_s):
                    if text in what:
                        raise KeyboardInterrupt()
                    return orig(what, pred, sim_s)
                d.wait = w
            return patch
        rc, why, bench, d, events = self.run_case(patch=interrupt_at('reversal: goal 6->5'))
        self.assertEqual(rc, 4, why)
        self.assertEqual(bench.s.state, loc.FAULTED)
        rc, why, bench, d, events = self.run_case(patch=interrupt_at('estop hold'))
        self.assertEqual(rc, 4, why)                                # during the observation
        self.assertEqual(bench.requests[-1][0], 'estop')            # no teleop: the service
        rc, why, bench, d, events = self.run_case(deaf_teleop=True)
        self.assertEqual(rc, 2)
        self.assertIn('wall cap', why)

        def refuse(bench):
            raise OSError(1, 'Operation not permitted')
        rc, why, bench, d, events = self.run_case(kill_hook=refuse)
        self.assertEqual(rc, 3, why)                                # a driver error: x (alive)
        self.assertEqual(d.kills[0]['event'], 'teleop_kill_failed')
        self.assertEqual(bench.s.state, loc.FAULTED)

        def broken(d, bench):
            d.hold_observation = lambda: (_ for _ in ()).throw(RuntimeError('injected'))
        rc, why, bench, d, events = self.run_case(patch=broken)
        self.assertEqual(rc, 3)
        self.assertIn('DRIVER ERROR', why)


class Shadow(Base):
    gate = False

    def test_completes_with_stationary_creep_and_records_it(self):
        rc, why, bench, d, events = self.run_case(shadow=True, progress_m=CREEP)
        self.assertEqual(rc, 0, why)
        self.assertIn('shadow rehearsal', why)
        self.assertEqual((bench.plant.sent, bench.s.transport.sent), (0, 12))
        self.assertEqual(self.transitions(events), EXPECTED_TRANSITIONS)
        self.assertEqual(self.count(events, 'cancel_sent'), 0)
        self.assertEqual(d.cycle_log[0]['progress_sign_bound'], self.mod.s4a.SHADOW_SIGN_NOTE)
        self.assertLess(d.cycle_log[0]['forward_m'], 0.0)           # creep: not required

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
            if st.get('state') == 'READY' and st.get('boundary') == 5:
                st['mode'] = 'dispatch'
            return st
        rc, why, bench, d, events = self.run_case(shadow=True, status_edit=edit)
        self.assertEqual(rc, 2, why)
        self.assertIn("mode 'dispatch'", why)
        self.assertLess(bench.s.transport.sent, 12)
        self.assertEqual(bench.plant.sent, 0)


class ShadowAgainstDispatch(Base):
    def test_shadow_refuses_a_dispatching_node_before_arm(self):
        self.mod = load_driver()
        bench = s4at.make_bench(0.2)                                # dispatch mode (gate True)
        d = make_driver(self.mod, bench, self.tmp.name, shadow=True)
        rc, why = self.mod.run(d)
        self.assertEqual(rc, 2, why)
        self.assertIn("mode 'dispatch'", why)
        self.assertIsNone(bench.teleop)
        self.assertEqual(bench.plant.sent, 0)


class Constants(unittest.TestCase):
    def test_scope_constants_and_unchanged_limits(self):
        m = load_driver()
        self.assertEqual((m.EXPECTED_GOALS, m.EXPECTED_CANCELS), (12, 0))
        self.assertLessEqual(m.EXPECTED_GOALS, 30)
        self.assertEqual(m.FIRST, {'release': 1, 'reversal': 6, 'home': 8, 'lease': 12})
        self.assertEqual(m.PLANNED[6], (6, 5, -1))
        self.assertLess(m.REPEAT_QUIESCE_S, 0.8)                    # inside the hold window
        a = m.s4a
        self.assertEqual(a.STOP_BAND_M[1], (0.0, 0.040))
        self.assertEqual((a.LATERAL_MAX_M, a.YAW_MAX_RAD), (0.020, 0.10))
        s3 = m.s3
        self.assertEqual((s3.TILT_MAX_RAD, s3.Z_MIN_M, s3.TRAVEL_MAX_M), (0.10, 0.045, 0.15))
        self.assertEqual((s3.JOINT_LIMIT_STOP_RAD, s3.TRACKING_ABORT_RAD), (0.02, 0.05))
        src = open(os.path.join(HERE, 's4b_driver.py')).read().split('class S4bDriver', 1)[1]
        script = src.split('def release_exercise', 1)[1].split('def summary', 1)[0]
        for k in ("' '", "'x'", "'r'", "'z'", "'q'", "'+'", "'-'"):
            self.assertNotIn(f'key({k}', script.replace(' ', '').replace('self.', ''))
        self.assertEqual(script.count("self.key('a'"), 1)          # armed once, never re-armed


if __name__ == '__main__':
    unittest.main()
