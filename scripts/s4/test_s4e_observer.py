#!/usr/bin/env python3
"""Tests of the S4e observer (s4e_observer.py; docs/S4E_CONTRACT.md 4.2, 7 and 12).

  - static: its only write to the teleop is the relay of bytes just read from the operator; no
    publisher, action client, /cmd_vel or key option; its one service client is the estop;
  - the relay (no ROS) on socket pairs: exact bytes in order, times, the quit cut, input closing;
  - the monitor (no ROS): every stop condition of contract section 3, and the hold message;
  - SCRIPTED PSEUDO-TERMINAL SESSIONS: this file plays the operator's terminal (a pseudo-terminal
    it owns) and a FAKE teleop (a small script, not ROS); statuses come from a fake ROS link.
    They test the relay, the stop handling and the terminal restore. Every session records
    input_source 'scripted-pty-test': none of it is, or could stand in for, human input.

Usage: cd scripts/s4 && python3 -m unittest -v test_s4e_observer
(needs spiderx_controller importable: the direct runs source the installation)"""
import ast
import json
import os
import select
import socket
import subprocess
import sys
import tempfile
import termios
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 's4e_observer.py')
sys.path[:0] = [HERE]
import s4e_observer as so  # noqa: E402
from spiderx_controller import m55_teleop as tp  # noqa: E402

SCRIPTED = 'scripted-pty-test'
FAKE_TELEOP = r'''
import os, signal, sys, termios, time, tty
log = open(os.environ['FAKE_TELEOP_LOG'], 'a', buffering=1)
mode = os.environ.get('FAKE_TELEOP_MODE', '')
def term(sig, frame):
    log.write(f'{time.monotonic():.6f} SIGTERM\n')
    print('fake teleop: SIGTERM, quitting', flush=True)
    sys.exit(0)
signal.signal(signal.SIGTERM, signal.SIG_IGN if mode == 'ignore_term' else term)
saved = termios.tcgetattr(0)
tty.setcbreak(0)
print('fake teleop ready', flush=True)
try:
    while True:
        data = os.read(0, 64)
        if not data:
            break
        log.write(f'{time.monotonic():.6f} {data.hex()}\n')
        print(f'got {data!r}', flush=True)
        if mode == 'crash_on_k' and b'K' in data:
            sys.exit(1)
        if b'q' in data and mode != 'ignore_q':
            print('fake teleop quit', flush=True)
            break
except KeyboardInterrupt:
    log.write(f'{time.monotonic():.6f} SIGINT\n')
finally:
    termios.tcsetattr(0, termios.TCSADRAIN, saved)
'''


def source():
    with open(SRC) as f:
        return f.read()


def status(n=1, **kw):
    """A nominal shadow status (sim advances with n); keyword overrides, dotted for nesting."""
    st = {'mode': 'shadow', 'dispatch_gate': False, 'state': 'READY', 'level': 0, 'goal': None,
          'command_rejections': {}, 'faults': [],
          'monitors': {'sim_time_s': 0.25 * n, 'body_pose_age_s': 0.05},
          'ground_truth': {'xy': [1.0, -2.0], 'z': 0.07, 'yaw': 0.3, 'tilt': 0.01,
                           'delta_since_arm_xy': [0.0, 0.0]}}
    for k, v in kw.items():
        d, *rest = k.split('__')
        if rest:
            st[d] = dict(st[d] or {}, **{rest[0]: v})
        else:
            st[d] = v
    return st


# ------------------------------------------------------------------ static
class NeverSendsAKey(unittest.TestCase):
    def setUp(self):
        self.tree = ast.parse(source())

    def calls(self, name):
        return [n for n in ast.walk(self.tree) if isinstance(n, ast.Call)
                and ast.unparse(n.func) == name]

    def test_its_only_write_to_the_teleop_relays_the_operator_bytes(self):
        writes = self.calls('os.write')
        targets = sorted(ast.unparse(c.args[0]) for c in writes)
        # the teleop's input, the operator's screen, and spawn()'s exec-failure note to itself
        self.assertEqual(targets, ['self.fd_out', 'self.master', 'sync_w'])
        spawn = next(n for n in ast.walk(self.tree)
                     if isinstance(n, ast.FunctionDef) and n.name == 'spawn')
        self.assertIn("os.write(sync_w, b'exec failed')", ast.unparse(spawn))
        relay = next(n for n in ast.walk(self.tree)
                     if isinstance(n, ast.FunctionDef) and n.name == '_relay_operator')
        w = [c for c in ast.walk(relay) if isinstance(c, ast.Call)
             and ast.unparse(c.func) == 'os.write']
        self.assertEqual([ast.unparse(c.args[0]) for c in w], ['self.master'])
        self.assertEqual(ast.unparse(w[0].args[1]), 'view')
        body = ast.unparse(relay)
        self.assertIn('data = os.read(self.fd_in, 1024)', body)
        self.assertIn('view = memoryview(data)', body)
        self.assertIn("data, rest = (data, b'') if cut is None else (data[:cut], data[cut:])",
                      body)

    def test_no_publisher_action_client_cmd_vel_or_injection(self):
        src = source()
        for word in ('create_publisher', 'ActionClient', 'Twist', 'cmd_vel', 'TIOCSTI',
                     'pty.fork', 'hold_key', 'emergency_stop('):
            self.assertNotIn(word, src, word)
        self.assertEqual(len(self.calls('self.node.create_client')), 1)
        self.assertEqual(so.ESTOP_SERVICE, '/spiderx/locomotion/estop')

    def test_the_cli_has_no_key_or_teleop_option(self):
        opts = sorted(s.value for c in self.calls('a.add_argument') for s in c.args
                      if isinstance(s, ast.Constant))
        self.assertEqual(opts, ['--domain-id', '--mode', '--out'])
        self.assertEqual(so.TELEOP_ARGV, ('ros2', 'run', 'spiderx_controller',
                                          'm55_teleop_keyboard'))

    def test_constants_match_the_teleop_and_the_contract(self):
        self.assertEqual((so.ESTOP, so.QUIT), (tp.ESTOP, tp.QUIT))
        self.assertEqual((so.TRAVEL_MAX_M, so.TILT_MAX_RAD, so.Z_MIN_M), (0.15, 0.10, 0.045))
        self.assertEqual((so.LATERAL_STOP_M, so.HEADING_STOP_RAD), (0.050, 0.15))
        self.assertEqual((so.JOINT_LIMIT_STOP_RAD, so.TRACKING_ABORT_RAD), (0.02, 0.05))
        self.assertEqual((so.MAX_GOALS, so.LEVEL, so.SESSION_CAP_S), (30, 0, 1200.0))
        self.assertEqual((so.FIRST_STATUS_WITHIN_S, so.STATUS_GAP_MAX_S), (60.0, 5.0))
        self.assertEqual((so.TELEOP_EXIT_WAIT_S, so.HOLD_SIM_S), (15.0, 10.0))


class Gates(unittest.TestCase):
    def g(self, m55, m61, m6):
        return {'M55_LOCOMOTION_DISPATCH_ENABLED': [m55, 'f'],
                'M61_LIVE_DISPATCH_ENABLED': [m61, 'f'], 'LIVE_DISPATCH_ENABLED': [m6, 'f']}

    def test_gate_refusal_is_exact_per_mode(self):
        self.assertIsNone(so.gate_refusal(self.g(False, False, False), 'shadow'))
        self.assertIsNone(so.gate_refusal(self.g(True, False, False), 'live'))
        for gates, mode in ((self.g(True, False, False), 'shadow'),
                            (self.g(False, False, False), 'live'),
                            (self.g(True, True, False), 'live'),
                            (self.g(None, False, False), 'shadow'),
                            (self.g(0, False, False), 'shadow'), ({}, 'shadow')):
            self.assertIsNotNone(so.gate_refusal(gates, mode), (gates, mode))


# ------------------------------------------------------------------ relay
class RelayTests(unittest.TestCase):
    def setUp(self):
        self.op_test, self.op_obs = socket.socketpair()       # the operator's terminal
        self.tl_test, self.tl_obs = socket.socketpair()       # the teleop's pseudo-terminal
        self.screen_test, self.screen_obs = socket.socketpair()
        for s in (self.op_test, self.op_obs, self.tl_test, self.tl_obs, self.screen_test,
                  self.screen_obs):
            self.addCleanup(s.close)
        self.records, self.actions = [], []
        self.r = so.Relay(self.op_obs.fileno(), self.screen_obs.fileno(), self.tl_obs.fileno(),
                          lambda kind, **k: self.records.append({'kind': kind, **k}),
                          tp.KeyDecoder().decode, time.monotonic,
                          lambda a, t: self.actions.append(a))

    def teleop_got(self):
        self.tl_test.setblocking(False)
        try:
            return self.tl_test.recv(65536)
        except BlockingIOError:
            return b''

    def test_bytes_are_relayed_unchanged_in_order_with_times(self):
        sent = [b'a', b'w', b'www', b' ', b'\x1b[A', b'h', b'x']
        for b in sent:
            self.op_test.send(b)
            self.r.pump(1.0)
        self.assertEqual(self.teleop_got(), b''.join(sent))
        keys = [r for r in self.records if r['kind'] == 'key']
        self.assertEqual([bytes.fromhex(k['hex']) for k in keys], sent)
        self.assertTrue(all(k['t_written'] >= k['t_read'] for k in keys))
        self.assertEqual(keys[4]['actions'], ['ignored'])
        self.assertEqual(self.actions, ['arm', 'forward', 'forward', 'forward', 'forward',
                                        'stop', 'ignored', 'home', 'estop'])
        self.assertEqual((self.r.bytes_read, self.r.bytes_relayed), (11, 11))

    def test_nothing_after_a_quit_is_relayed(self):
        for b in (b'w', b'qww', b'w'):
            self.op_test.send(b)
            self.r.pump(1.0)
        self.assertEqual(self.teleop_got(), b'wq')
        nr = [r for r in self.records if r['kind'] == 'not_relayed']
        self.assertEqual([bytes.fromhex(r['hex']) for r in nr], [b'ww', b'w'])
        self.assertFalse(self.r.input_open)
        self.assertEqual((self.r.bytes_read, self.r.bytes_relayed), (5, 2))

    def test_ctrl_c_and_ctrl_d_are_quits_too(self):
        for q in (b'\x03', b'\x04'):
            self.assertEqual(so.quit_cut(b'w' + q + b'w', tp.KeyDecoder().decode), 2)
        self.assertIsNone(so.quit_cut(b'w x', tp.KeyDecoder().decode))
        self.assertIsNone(so.quit_cut(b'\x1b[q', tp.KeyDecoder().decode))   # a sequence's 'q'

    def test_closed_input_relays_nothing(self):
        self.r.close_input('stop: test')
        self.op_test.send(b'w')
        self.r.pump(1.0)
        self.assertEqual(self.teleop_got(), b'')
        self.assertEqual(self.records[-1]['kind'], 'not_relayed')
        self.assertEqual(self.records[-1]['why'], 'stop: test')

    def test_teleop_output_reaches_the_screen_and_the_record(self):
        self.tl_test.send(b'state READY\r\n')
        self.r.pump(1.0)
        self.assertEqual(self.screen_test.recv(100), b'state READY\r\n')
        self.assertEqual(self.records[-1]['text'], 'state READY\r\n')

    def test_the_operator_terminal_closing_is_noticed(self):
        self.op_test.close()
        self.r.pump(1.0)
        self.assertTrue(self.r.operator_gone)
        self.assertEqual(self.records[-1]['kind'], 'operator_terminal_gone')


# ------------------------------------------------------------------ monitor
class MonitorTests(unittest.TestCase):
    LIMITS = {'j1': (-1.0, 1.0)}

    def mon(self, mode='shadow'):
        self.t = 100.0
        return so.Monitor(mode, self.LIMITS, self.t, clock=lambda: self.t)

    def feed(self, m, *sts):
        for st in sts:
            self.t += 0.5
            m.on_status(st, self.t)
        return m.check(self.t)

    def armed(self, mode='shadow'):
        m = self.mon(mode)
        mode_kw = {} if mode == 'shadow' else {'mode': 'dispatch', 'dispatch_gate': True}
        self.feed(m, status(1, state='DISARMED', **mode_kw), status(2, **mode_kw))
        return m, mode_kw

    def test_nominal_statuses_pass_and_set_the_arm_reference(self):
        m, _ = self.armed()
        self.assertEqual(m.arm_ref, {'xy': (1.0, -2.0), 'yaw': 0.3})

    def test_wrong_mode_stops_both_ways(self):
        with self.assertRaisesRegex(so.Stop, 'shadow: status mode'):
            self.feed(self.mon('shadow'), status(1, mode='dispatch', dispatch_gate=True))
        with self.assertRaisesRegex(so.Stop, 'live: status mode'):
            self.feed(self.mon('live'), status(1))
        with self.assertRaisesRegex(so.Stop, 'status mode'):
            self.feed(self.mon('shadow'), status(1, dispatch_gate=None))

    def test_rejections_unplanned_fault_level_goal_budget_and_tracking_stop(self):
        cases = [(status(3, command_rejections={'rate': 1}), 'command rejections'),
                 (status(3, state='FAULTED', faults=['stale_pose']), 'FAULTED before'),
                 (status(3, level=1), 'node level 1'),
                 (status(3, goal={'seq': 31, 'level': 0}), 'above the budget'),
                 (status(3, goal={'seq': 2, 'level': 1}), 'at level 1'),
                 (status(3, goal={'seq': 2, 'level': 0, 'max_tracking_error_rad': 0.051}),
                  'tracking error')]
        for st, text in cases:
            m, _ = self.armed()
            with self.assertRaisesRegex(so.Stop, text):
                self.feed(m, st)

    def test_faulted_after_the_persons_estop_is_planned(self):
        m, _ = self.armed()
        m.person_estop(self.t)
        self.feed(m, status(3, state='FAULTED', faults=['estop']))

    def test_a_tracking_finding_is_recorded_not_a_stop(self):
        m, _ = self.armed()
        self.feed(m, status(3, goal={'seq': 2, 'level': 0, 'max_tracking_error_rad': 0.03}))
        self.assertEqual(m.findings, {2: 0.03})

    def test_tilt_and_fall_need_two_fresh_samples(self):
        for key, bad, text in (('ground_truth__tilt', 0.11, 'tilt'),
                               ('ground_truth__z', 0.044, 'body z')):
            m, _ = self.armed()
            self.feed(m, status(3, **{key: bad}), status(4))
            self.feed(m, status(5, **{key: bad}), status(5, **{key: bad}))  # not fresh: same sim
            with self.assertRaisesRegex(so.Stop, text):
                self.feed(m, status(6, **{key: bad}))

    def test_travel_and_cumulative_drift_stop(self):
        m, _ = self.armed()
        with self.assertRaisesRegex(so.Stop, 'travel 0.1510'):
            self.feed(m, status(3, ground_truth__delta_since_arm_xy=[0.0, 0.151]))
        m, _ = self.armed()            # yaw 0.3: body +x (lateral) is (cos 0.3, sin 0.3)
        lat = 0.051
        with self.assertRaisesRegex(so.Stop, 'cumulative drift'):
            self.feed(m, status(3, ground_truth__xy=[1.0 + 0.9553365 * lat,
                                                     -2.0 + 0.2955202 * lat]))
        m, _ = self.armed()
        with self.assertRaisesRegex(so.Stop, 'heading'):
            self.feed(m, status(3, ground_truth__yaw=0.3 + 0.151))
        m, _ = self.armed()
        self.feed(m, status(3, ground_truth__yaw=0.3 + 0.149,
                            ground_truth__delta_since_arm_xy=[0.0, 0.149]))

    def test_the_joint_limit_stop(self):
        m, _ = self.armed()
        m.on_joint_state(['j1', 'other'], [0.975, 5.0])
        self.feed(m, status(3))
        m.on_joint_state(['j1'], [0.981])
        with self.assertRaisesRegex(so.Stop, 'URDF limit'):
            self.feed(m, status(4))
        self.assertAlmostEqual(m.min_joint_margin[0], 0.019)

    def test_status_cadence_and_session_cap(self):
        m = self.mon()
        self.t += so.FIRST_STATUS_WITHIN_S + 0.1
        with self.assertRaisesRegex(so.Stop, 'no node status within'):
            m.check(self.t)
        m, _ = self.armed()
        self.t += so.STATUS_GAP_MAX_S + 0.1
        with self.assertRaisesRegex(so.Stop, 'no node status for'):
            m.check(self.t)
        m, _ = self.armed()
        self.t = m.t_start + so.SESSION_CAP_S + 0.1
        m.on_status(status(9), self.t)
        with self.assertRaisesRegex(so.Stop, 'session cap'):
            m.check(self.t)

    def test_the_hold_message_after_10_s_of_sim_in_faulted(self):
        m, _ = self.armed()
        m.person_estop(self.t)
        msgs = self.feed(m, status(3, state='FAULTED', goal={'seq': 5, 'level': 0}))
        self.assertEqual(msgs, [])
        for n in range(4, 46):       # 0.25 s of sim per status: sim 1.0 .. 11.25
            msgs += self.feed(m, status(n, state='FAULTED'))
        self.assertEqual(msgs, [so.HOLD_COMPLETE])
        self.assertEqual(m.hold_from_sim, 1.0)
        self.assertTrue(m.hold_done)


# ------------------------------------------------------------------ scripted pty sessions
class FakeRos:
    def __init__(self, monitor, rec, template, period=0.1):
        self.monitor, self.template, self.period = monitor, template, period
        self.estops, self.n, self.closed = [], 0, False
        threading.Thread(target=self._publish, daemon=True).start()

    def _publish(self):
        while not self.closed:
            self.n += 1
            self.monitor.on_status(self.template(self.n), time.monotonic())
            time.sleep(self.period)

    def call_estop(self):
        self.estops.append(time.monotonic())
        return {'available': True, 'success': True, 'message': 'fake estop'}

    def close(self):
        self.closed = True


class Sessions(unittest.TestCase):
    """SCRIPTED PSEUDO-TERMINAL SESSIONS with a fake teleop (see the module docstring)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.fake = os.path.join(self.tmp.name, 'fake_teleop.py')
        with open(self.fake, 'w') as f:
            f.write(FAKE_TELEOP)
        self.log = os.path.join(self.tmp.name, 'teleop.log')
        os.environ['FAKE_TELEOP_LOG'] = self.log
        self.addCleanup(os.environ.pop, 'FAKE_TELEOP_LOG', None)
        self.addCleanup(os.environ.pop, 'FAKE_TELEOP_MODE', None)

    def session(self, script, template=status, teleop_mode='', wait_s=1.0, timeout=30.0):
        os.environ['FAKE_TELEOP_MODE'] = teleop_mode
        master, slave = os.openpty()
        before = termios.tcgetattr(slave)
        out = os.path.join(self.tmp.name, f'obs{len(os.listdir(self.tmp.name))}')
        os.makedirs(out)
        holder, result = {}, {}

        def factory(mon, rec):
            holder['ros'] = FakeRos(mon, rec, template)
            return holder['ros']
        obs = so.Observer(out, 'shadow', {}, fd_in=slave, fd_out=slave,
                          teleop_argv=[sys.executable, self.fake], ros_factory=factory,
                          gates={'test': 'not checked'}, tty={'test': 'scripted pty'},
                          teleop_exit_wait_s=wait_s, input_source=SCRIPTED)
        th = threading.Thread(target=lambda: result.update(rs=obs.run()), daemon=True)
        th.start()
        screen, t_start, t0 = bytearray(), time.monotonic(), None
        pending, closed = list(script), {'master': False}
        self.close_master = lambda obs, m: (os.close(m), closed.update(master=True))
        while th.is_alive() and time.monotonic() - t_start < timeout:
            now = time.monotonic()
            if t0 is None and b'fake teleop ready' in screen:
                t0 = now
            while t0 is not None and pending and pending[0][0] <= now - t0:
                _, act = pending.pop(0)
                if callable(act):
                    act(obs, master)
                else:
                    os.write(master, act)
            if closed['master']:
                time.sleep(0.01)
                continue
            r, _, _ = select.select([master], [], [], 0.01)
            if r:
                try:
                    screen += os.read(master, 65536)
                except OSError:
                    pass
        th.join(5.0)
        self.assertFalse(th.is_alive(), 'the observer did not finish')
        try:
            after = termios.tcgetattr(slave)
        except termios.error:
            after = None
        for fd in (master, slave):
            try:
                os.close(fd)
            except OSError:
                pass
        rc, summary = result['rs']
        with open(os.path.join(out, 'observer.jsonl')) as f:
            recs = [json.loads(ln) for ln in f]
        with open(self.log) as f:
            tlog = f.read().split('\n')
        return {'rc': rc, 'summary': summary, 'screen': screen.decode(errors='replace'),
                'recs': recs, 'tlog': [ln for ln in tlog if ln], 'before': before,
                'after': after, 'ros': holder.get('ros'), 'out': out}

    @staticmethod
    def relayed(tlog):
        return b''.join(bytes.fromhex(ln.split()[1]) for ln in tlog
                        if len(ln.split()) == 2 and ln.split()[1] not in ('SIGTERM', 'SIGINT'))

    def check_common(self, s):
        self.assertEqual(s['after'], s['before'], 'operator terminal not restored')
        self.assertIs(s['summary']['terminal_restored'], True)
        self.assertEqual(s['recs'][0]['input_source'], SCRIPTED)
        self.assertEqual(s['summary']['input_source'], SCRIPTED)
        keys = [r for r in s['recs'] if r['kind'] == 'key']
        self.assertEqual(self.relayed(s['tlog']),
                         b''.join(bytes.fromhex(k['hex']) for k in keys))
        self.assertTrue(all(k['t_written'] >= k['t_read'] for k in keys))
        holders = [r for r in s['recs'] if r['kind'] == 'pty_holders']
        self.assertEqual([h['at'] for h in holders], ['start', 'end'])
        start = holders[0]
        self.assertEqual([h['pid'] for h in start['master_holders']], [os.getpid()])
        self.assertTrue(start['slave_holders'])
        self.assertTrue(all(h['sid'] == s['summary']['teleop']['pid']
                            for h in start['slave_holders']))
        return keys

    def test_a_complete_session_relays_exactly_and_ends_on_q(self):
        script = ([(0.2, b'a')] + [(0.4 + i / 30, b'w') for i in range(15)]
                  + [(1.2, b' '), (1.5, b'h'), (1.8, b'w'), (1.9, b'x'), (2.3, b'q')])
        s = self.session(script)
        self.assertEqual(s['rc'], 0, s['summary'])
        self.assertEqual(s['summary']['result'], 'session complete')
        keys = self.check_common(s)
        self.assertEqual(self.relayed(s['tlog']), b'a' + b'w' * 15 + b' hwxq')
        self.assertEqual(sum(k['n'] for k in keys), s['summary']['relay']['bytes_relayed'])
        self.assertEqual(s['summary']['teleop']['exit_code'], 0)
        self.assertEqual(s['ros'].estops, [])
        self.assertIsNotNone(s['summary']['monitor']['person_estop_t'])
        self.assertIn('the observer never sends a key', s['screen'])

    def test_a_stop_condition_closes_input_calls_estop_and_ends_the_teleop(self):
        tilted = {'v': False}

        def template(n):
            return status(n, **({'ground_truth__tilt': 0.2} if tilted['v'] else {}))
        script = [(0.2, b'a'), (0.5, lambda obs, m: tilted.update(v=True)), (1.5, b'w')]
        s = self.session(script, template=template)
        self.assertEqual(s['rc'], 2, s['summary'])
        self.assertIn('STOP CONDITION: tilt', s['summary']['result'])
        self.check_common(s)
        self.assertEqual(len(s['ros'].estops), 1)
        self.assertTrue(any(ln.endswith('SIGTERM') for ln in s['tlog']))
        self.assertEqual(self.relayed(s['tlog']), b'a')
        self.assertIn('STOP:', s['screen'])
        kinds = [r['kind'] for r in s['recs']]
        self.assertLess(kinds.index('input_closed'), kinds.index('estop_service'))

    def test_a_teleop_ignoring_sigterm_is_killed(self):
        script = [(0.2, b'a'), (0.4, lambda obs, m: obs.interrupt('SIGTERM'))]
        s = self.session(script, teleop_mode='ignore_term', wait_s=1.0)
        self.assertEqual(s['rc'], 4, s['summary'])
        self.check_common(s)
        self.assertEqual(s['summary']['teleop']['exit_code'], -9)
        sigs = [r['signal'] for r in s['recs'] if r['kind'] == 'teleop_signal']
        self.assertEqual(sigs, ['SIGTERM', 'SIGKILL'])

    def test_a_teleop_crash_is_a_stop_condition(self):
        s = self.session([(0.2, b'a'), (0.4, b'K')], teleop_mode='crash_on_k')
        self.assertEqual(s['rc'], 2, s['summary'])
        self.assertIn('exited with 1 without a quit', s['summary']['result'])
        self.check_common(s)
        self.assertEqual(len(s['ros'].estops), 1)

    def test_the_operator_terminal_closing_is_a_stop_condition(self):
        s = self.session([(0.2, b'a'), (0.4, lambda obs, m: self.close_master(obs, m))])
        self.assertEqual(s['rc'], 2, s['summary'])
        self.assertIn("operator's terminal closed", s['summary']['result'])
        self.assertEqual(len(s['ros'].estops), 1)
        self.assertTrue(any(ln.endswith('SIGTERM') for ln in s['tlog']))

    def test_a_wrong_mode_status_is_a_stop_condition(self):
        def dispatching(n):
            return status(n, mode='dispatch', dispatch_gate=True)
        s = self.session([(5.0, b'q')], template=dispatching)
        self.assertEqual(s['rc'], 2, s['summary'])
        self.assertIn("status mode 'dispatch'", s['summary']['result'])
        self.check_common(s)

    def test_bytes_after_q_in_the_same_chunk_are_not_relayed(self):
        s = self.session([(0.2, b'w'), (0.4, b'qww')])
        self.assertEqual(s['rc'], 0, s['summary'])
        self.check_common(s)
        self.assertEqual(self.relayed(s['tlog']), b'wq')
        nr = [r for r in s['recs'] if r['kind'] == 'not_relayed']
        self.assertEqual(bytes.fromhex(nr[0]['hex']), b'ww')

    def test_the_teleop_not_exiting_after_q_is_a_stop_condition(self):
        s = self.session([(0.2, b'q')], teleop_mode='ignore_q', wait_s=0.5)
        self.assertEqual(s['rc'], 2, s['summary'])
        self.assertIn('did not exit within', s['summary']['result'])
        self.check_common(s)
        self.assertEqual(len(s['ros'].estops), 1)
        self.assertTrue(any(ln.endswith('SIGTERM') for ln in s['tlog']))


class Cli(unittest.TestCase):
    def test_refused_without_a_terminal_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'obs')
            r = subprocess.run([sys.executable, SRC, '--out', out, '--mode', 'shadow'],
                               stdin=subprocess.DEVNULL, capture_output=True, text=True,
                               timeout=120)
            self.assertEqual(r.returncode, 5, r.stderr)
            self.assertIn('REFUSED (nothing started)', r.stderr)
            with open(os.path.join(out, 'observer_summary.json')) as f:
                rec = json.load(f)
            self.assertTrue(rec['result'].startswith('REFUSED: the human check needs'))
            self.assertFalse(os.path.exists(os.path.join(out, 'observer.jsonl')))
            r = subprocess.run([sys.executable, SRC, '--out', out, '--mode', 'shadow'],
                               stdin=subprocess.DEVNULL, capture_output=True, text=True,
                               timeout=120)
            self.assertEqual(r.returncode, 5)
            self.assertIn('never overwritten', r.stderr)

    def test_mode_is_required_and_limited(self):
        r = subprocess.run([sys.executable, SRC, '--out', '/nonexistent/x', '--mode', 'enabled'],
                           stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=120)
        self.assertEqual(r.returncode, 2)
        self.assertIn('invalid choice', r.stderr)


if __name__ == '__main__':
    unittest.main()
