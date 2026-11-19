#!/usr/bin/env python3
"""Tests of the S4e terminal preflight (s4e_terminal_preflight.py; docs/S4E_CONTRACT.md 4.1).

Two kinds, kept apart:
  - pure analysis tests on synthetic timings (no terminal at all);
  - SCRIPTED PSEUDO-TERMINAL TESTS: this file writes the characters into a pseudo-terminal it
    owns. They test the tool's terminal handling only. Every one passes --scripted-pty-test, so
    the tool labels its output and record "SCRIPTED PTY TEST - NOT HUMAN INPUT"; none of them is,
    or could stand in for, human input.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_s4e_terminal_preflight"""
import ast
import fcntl
import json
import os
import select
import signal
import subprocess
import sys
import tempfile
import termios
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, 's4e_terminal_preflight.py')
sys.path[:0] = [HERE]
import s4e_terminal_preflight as pf  # noqa: E402

W = ord('w')


def source():
    with open(TOOL) as f:
        return f.read()


def hold(start, delay, interval, duration, b=W):
    """Characters of one held key: the press, then repeats after `delay` every `interval`."""
    t, out = start, [(start, b)]
    t = start + delay
    while t <= start + duration + 1e-9:
        out.append((round(t, 6), b))
        t += interval
    return out


# ------------------------------------------------------------------ pure analysis
class Analysis(unittest.TestCase):
    def test_a_good_hold_passes_with_its_delay_rate_and_largest_gap(self):
        rl = pf.runs(hold(0.0, 0.40, 1 / 30, 12.0))
        self.assertEqual(len(rl), 1)
        r = rl[0]
        self.assertAlmostEqual(r['first_repeat_delay_s'], 0.40, places=6)
        self.assertAlmostEqual(r['median_interval_s'], 1 / 30, places=4)
        self.assertAlmostEqual(r['rate_hz'], 30.0, places=1)
        self.assertAlmostEqual(r['max_gap_s'], 0.40, places=6)
        self.assertGreaterEqual(r['duration_s'], 11.9)
        v = pf.verdict(rl)
        self.assertEqual(v['verdict'], 'PASS', v)
        self.assertIs(v['qualifying_hold'], r)

    def test_a_first_repeat_at_or_above_the_hold_window_fails(self):
        for delay in (0.80, 0.95, 1.5):
            v = pf.verdict(pf.runs(hold(0.0, delay, 1 / 30, 12.0)))
            self.assertEqual(v['verdict'], 'FAIL', (delay, v))
            self.assertIn('hold window', v['reasons'][0])
        self.assertEqual(pf.verdict(pf.runs(hold(0.0, 0.79, 1 / 30, 12.0)))['verdict'], 'PASS')

    def test_a_stall_inside_a_hold_fails(self):
        ev = hold(0.0, 0.3, 1 / 30, 5.0) + hold(6.0, 0.3, 1 / 30, 8.0)[1:]
        v = pf.verdict(pf.runs(ev))
        self.assertEqual(v['verdict'], 'FAIL', v)

    def test_a_first_repeat_slower_than_the_repeat_window_is_caught(self):
        # 2.5 s delay: the teleop (and the 2.0 s run window) see a single character, then a run
        ev = hold(0.0, 2.5, 1 / 30, 14.0)
        rl = pf.runs(ev)
        self.assertEqual([r['n'] for r in rl][0], 1)
        v = pf.verdict(rl)
        self.assertEqual(v['verdict'], 'FAIL', v)
        self.assertIn('first repeat came after the hold window', v['reasons'][0])

    def test_release_then_a_tap_is_normal(self):
        ev = hold(0.0, 0.5, 1 / 30, 11.0) + [(14.0, W)]
        rl = pf.runs(ev)
        self.assertEqual([r['n'] for r in rl][1], 1)
        self.assertEqual(pf.verdict(rl)['verdict'], 'PASS')

    def test_a_short_hold_is_unmeasured_never_pass(self):
        v = pf.verdict(pf.runs(hold(0.0, 0.4, 1 / 30, 6.0)))
        self.assertEqual(v['verdict'], 'UNMEASURED')
        self.assertEqual(pf.verdict([])['verdict'], 'UNMEASURED')
        self.assertEqual(pf.verdict(pf.runs(hold(0.0, 0.4, 1 / 30, 6.0)), min_hold_s=5.0)[
            'verdict'], 'PASS')

    def test_a_reverse_key_counts_for_gaps_but_not_as_the_forward_hold(self):
        v = pf.verdict(pf.runs(hold(0.0, 0.4, 1 / 30, 12.0, b=ord('s'))))
        self.assertEqual(v['verdict'], 'UNMEASURED')
        v = pf.verdict(pf.runs(hold(0.0, 0.9, 1 / 30, 12.0, b=ord('s'))))
        self.assertEqual(v['verdict'], 'FAIL')

    def test_slow_non_motion_keys_do_not_fail(self):
        ev = [(0.0, ord('a')), (1.0, ord('a'))] + hold(2.0, 0.4, 1 / 30, 11.0)
        self.assertEqual(pf.verdict(pf.runs(ev))['verdict'], 'PASS')

    def test_a_different_key_or_a_gap_beyond_the_window_starts_a_new_run(self):
        ev = [(0.0, W), (0.5, W), (0.6, ord('a')), (0.7, W), (3.0, W)]
        self.assertEqual([r['n'] for r in pf.runs(ev)], [2, 1, 1, 1])

    def test_key_names(self):
        self.assertEqual([pf.key_name(b) for b in (0x77, 0x20, 0x1b, 0x04, 0x03, 0x7f, 0x80)],
                         ['w', 'space', 'Esc', 'Ctrl+D', 'Ctrl+C', 'Backspace', '0x80'])

    def test_no_more_characters_is_stated_as_not_a_key_up(self):
        self.assertIn('NOT a key-up event', pf.NOT_KEY_UP)
        self.assertIn('never reports a key release', pf.NOT_KEY_UP)

    def test_xset_parsing(self):
        txt = ('Keyboard Control:\n  auto repeat:  on    key click percent:  0    LED mask:  0\n'
               '  auto repeat delay:  250    repeat rate:  30\n')
        self.assertEqual(pf.parse_xset(txt), {'auto_repeat_delay_ms': 250,
                                              'repeat_rate_per_s': 30, 'auto_repeat_on': True})
        self.assertEqual(pf.parse_xset(''), {'auto_repeat_delay_ms': None,
                                             'repeat_rate_per_s': None, 'auto_repeat_on': None})

    def test_refusal_reasons(self):
        ok = {'stdin_tty': True, 'stdout_tty': True, 'foreground': True}
        self.assertIsNone(pf.refusal(ok))
        self.assertIn('interactive terminal', pf.refusal({**ok, 'stdin_tty': False}))
        self.assertIn('standard output', pf.refusal({**ok, 'stdout_tty': False}))
        self.assertIn('foreground', pf.refusal({**ok, 'foreground': False}))


class Standalone(unittest.TestCase):
    STDLIB = {'argparse', 'json', 'os', 'select', 'shutil', 'signal', 'statistics', 'subprocess',
              'sys', 'termios', 'time'}

    def test_imports_are_the_standard_library_only(self):
        tree = ast.parse(source())
        names = set()
        for n in ast.walk(tree):
            if isinstance(n, ast.Import):
                names |= {a.name.split('.')[0] for a in n.names}
            elif isinstance(n, ast.ImportFrom):
                names.add(n.module.split('.')[0])
        self.assertEqual(names, self.STDLIB)

    def test_importing_it_loads_no_ros_and_no_spiderx_module(self):
        code = ('import sys; sys.path.insert(0, sys.argv[1]); import s4e_terminal_preflight; '
                'print(sorted(m for m in sys.modules if m.startswith(("rclpy", "spiderx", '
                '"rosidl", "ament"))))')
        r = subprocess.run([sys.executable, '-c', code, HERE], capture_output=True, text=True,
                           timeout=60, cwd='/')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.strip(), '[]')


class TeleopConsistency(unittest.TestCase):
    """The standalone copies of the teleop's numbers and keys match the teleop and its config."""

    @classmethod
    def setUpClass(cls):
        try:
            from spiderx_controller import m55_locomotion as loc
            from spiderx_controller import m55_teleop as tp
        except ImportError as e:            # the direct runs source the installation
            raise unittest.SkipTest(f'spiderx_controller not importable: {e}')
        cls.loc, cls.tp = loc, tp

    def test_hold_window_and_repeat_window(self):
        self.assertEqual(pf.HOLD_S, self.loc.load_config().teleop.hold_s)
        self.assertEqual(pf.REPEAT_WINDOW_S, self.tp.REPEAT_GAP_WINDOW_S)

    def test_motion_keys(self):
        dec = self.tp.KeyDecoder()
        for ch in pf.MOTION_KEYS:
            want = self.tp.FORWARD if ch.lower() == 'w' else self.tp.REVERSE
            self.assertEqual(dec.decode(ch), [(want, None)])
        self.assertEqual({k for k, v in self.tp.KEYMAP.items()
                          if v in (self.tp.FORWARD, self.tp.REVERSE)}, {'w', 's'})


# ------------------------------------------------------------------ scripted pseudo-terminal
def run_in_pty(args, script, env_extra=None, timeout=30.0):
    """SCRIPTED PTY TEST: run the tool on a pseudo-terminal this test owns (the tool is its
    session leader and foreground). After its "Hold w" line, perform `script`: [(seconds, bytes
    to write, or a callable(pid))]. Returns (exit code, output, termios before, termios after),
    the termios read from the slave this test keeps open."""
    master, slave = os.openpty()
    before = termios.tcgetattr(slave)
    pid = os.fork()
    if pid == 0:                                      # the child: the tool, on the slave
        try:
            os.close(master)
            os.setsid()
            fcntl.ioctl(slave, termios.TIOCSCTTY, 0)
            for fd in (0, 1, 2):
                os.dup2(slave, fd)
            env = dict(os.environ, **(env_extra or {}))
            os.execve(sys.executable, [sys.executable, TOOL, '--no-env-query'] + args, env)
        finally:
            os._exit(127)
    out, status, t_start, t_exit = bytearray(), None, time.monotonic(), None
    t0, pending = None, list(script)
    while True:
        now = time.monotonic()
        if t0 is None and b'Hold w' in out:
            t0 = now + 0.3                          # let cbreak take effect before the first key
        while t0 is not None and pending and pending[0][0] <= now - t0:
            _, action = pending.pop(0)
            if callable(action):
                action(pid)
            else:
                os.write(master, action)
        r, _, _ = select.select([master], [], [], 0.005)
        if r:
            try:
                out += os.read(master, 65536)
            except OSError:
                pass
        if status is None:
            p, st = os.waitpid(pid, os.WNOHANG)
            if p:
                status, t_exit = st, now
        if status is not None and not r and now - t_exit > 0.3:
            break
        if now - t_start > timeout:
            os.kill(pid, signal.SIGKILL)
            os.waitpid(pid, 0)
            raise AssertionError(f'timeout; output so far:\n{out.decode(errors="replace")}')
    after = termios.tcgetattr(slave)
    os.close(master)
    os.close(slave)
    return os.waitstatus_to_exitcode(status), out.decode(errors='replace'), before, after


def repeat_script(start, delay, interval, duration, ch=b'w'):
    return [(t, ch) for t, _ in hold(start, delay, interval, duration)]


class ScriptedPty(unittest.TestCase):
    """SCRIPTED PSEUDO-TERMINAL TESTS (not human input; see the module docstring)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.out = os.path.join(self.tmp.name, 'preflight.json')

    def record(self):
        with open(self.out) as f:
            return json.load(f)

    def assert_restored_and_labelled(self, before, after, text):
        self.assertEqual(after, before, 'the terminal settings were not restored')
        lines = [ln for ln in text.replace('\r', '').splitlines() if ln.strip()]
        self.assertTrue(lines)
        self.assertTrue(all(ln.startswith(f'[{pf.SCRIPTED_LABEL}]') for ln in lines),
                        [ln for ln in lines if not ln.startswith(f'[{pf.SCRIPTED_LABEL}]')][:3])

    def test_normal_hold_release_tap_and_ctrl_d(self):
        script = repeat_script(0.0, 0.30, 1 / 30, 1.5) + [(4.0, b'w'), (4.4, b'\x04')]
        rc, text, before, after = run_in_pty(['--scripted-pty-test', '--min-hold-s', '1.0',
                                              '--out', self.out], script)
        self.assertEqual(rc, 0, text)
        self.assert_restored_and_labelled(before, after, text)
        rec = self.record()
        self.assertEqual(rec['input_source'], pf.SCRIPTED)
        self.assertIs(rec['human_check'], False)
        self.assertIs(rec['terminal_restored'], True)
        self.assertEqual(rec['exit'], {'code': 0, 'reason': 'Ctrl+D'})
        self.assertEqual(rec['verdict']['verdict'], 'PASS', rec['verdict'])
        self.assertEqual([r['n'] for r in rec['runs']][1], 1)
        first = rec['runs'][0]
        self.assertGreater(first['n'], 30)
        self.assertLess(first['max_gap_s'], pf.HOLD_S)
        self.assertIn('repeat input of', text)
        self.assertIn('NOT a key-up event', text)
        self.assertIn('is not the contract value', text)
        self.assertEqual(rec['min_hold_s'], 1.0)
        self.assertEqual(rec['contract_min_hold_s'], 10.0)

    def test_repeated_keys_and_a_slow_first_repeat_fail(self):
        script = ([(0.0, b'a')] + repeat_script(0.3, 0.9, 1 / 30, 1.5)
                  + [(2.3, b' '), (2.5, b'\x04')])
        rc, text, before, after = run_in_pty(['--scripted-pty-test', '--min-hold-s', '1.0',
                                              '--out', self.out], script)
        self.assertEqual(rc, 0, text)
        self.assert_restored_and_labelled(before, after, text)
        rec = self.record()
        self.assertEqual([r['key'] for r in rec['runs']], ['a', 'w', 'space'])
        self.assertEqual(rec['verdict']['verdict'], 'FAIL', rec['verdict'])
        self.assertGreaterEqual(rec['runs'][1]['first_repeat_delay_s'], pf.HOLD_S)
        self.assertIn('VERDICT', text)

    def test_idle_is_reported_and_is_not_a_key_up(self):
        script = [(0.0, b'w'), (2.6, b'\x04')]
        rc, text, before, after = run_in_pty(['--scripted-pty-test', '--out', self.out], script)
        self.assertEqual(rc, 0, text)
        self.assert_restored_and_labelled(before, after, text)
        self.assertIn('idle: nothing for 2 s', text)
        self.assertIn('idle is not a key-up either', text)
        self.assertIn('single', text)
        rec = self.record()
        self.assertEqual(len(rec['idle']), 1)
        self.assertEqual(rec['verdict']['verdict'], 'UNMEASURED')

    def test_the_duration_cap_ends_it(self):
        rc, text, before, after = run_in_pty(['--scripted-pty-test', '--duration', '0.8',
                                              '--out', self.out], [])
        self.assertEqual(rc, 0, text)
        self.assert_restored_and_labelled(before, after, text)
        self.assertEqual(self.record()['exit']['reason'], 'duration cap')

    def test_ctrl_c_interrupts_and_restores(self):
        script = repeat_script(0.0, 0.3, 1 / 30, 0.6) + [(0.8, b'\x03')]
        rc, text, before, after = run_in_pty(['--scripted-pty-test', '--out', self.out], script)
        self.assertEqual(rc, 4, text)
        self.assert_restored_and_labelled(before, after, text)
        rec = self.record()
        self.assertEqual(rec['exit'], {'code': 4, 'reason': 'Ctrl+C (SIGINT)'})
        self.assertIs(rec['terminal_restored'], True)

    def test_sigterm_and_sighup_interrupt_and_restore(self):
        for sig in (signal.SIGTERM, signal.SIGHUP):
            out = os.path.join(self.tmp.name, f'p_{sig.name}.json')
            script = [(0.0, b'w'), (0.5, lambda pid, s=sig: os.kill(pid, s))]
            rc, text, before, after = run_in_pty(['--scripted-pty-test', '--out', out], script)
            self.assertEqual(rc, 4, text)
            self.assertEqual(after, before)
            with open(out) as f:
                self.assertEqual(json.load(f)['exit']['reason'], f'{sig.name} received')

    def test_an_internal_error_restores_the_terminal_and_keeps_the_record(self):
        script = [(0.0, b'w'), (0.2, b'w')]        # nothing after the error: the restored
        #                                               terminal would echo it (not the tool)
        rc, text, before, after = run_in_pty(
            ['--scripted-pty-test', '--out', self.out], script,
            env_extra={'S4E_PREFLIGHT_TEST_RAISE_AFTER_CHUNKS': '2'})
        self.assertEqual(rc, 3, text)
        self.assert_restored_and_labelled(before, after, text)
        rec = self.record()
        self.assertTrue(rec['exit']['reason'].startswith('error:'), rec['exit'])
        self.assertIs(rec['terminal_restored'], True)

    def test_the_error_hook_is_inert_without_the_scripted_flag(self):
        src = source()
        self.assertIn("p.label == SCRIPTED_LABEL and test_raise", src)


class Refusals(unittest.TestCase):
    def test_the_human_check_refuses_without_a_terminal_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'p.json')
            r = subprocess.run([sys.executable, TOOL, '--no-env-query', '--out', out],
                               stdin=subprocess.DEVNULL, capture_output=True, text=True,
                               timeout=60)
            self.assertEqual(r.returncode, 5, r.stderr)
            self.assertIn('REFUSED', r.stderr)
            self.assertIn('interactive terminal', r.stderr)
            self.assertFalse(os.path.exists(out))

    def test_an_existing_record_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'p.json')
            with open(out, 'w') as f:
                f.write('{"keep": true}')
            r = subprocess.run([sys.executable, TOOL, '--no-env-query', '--out', out],
                               stdin=subprocess.DEVNULL, capture_output=True, text=True,
                               timeout=60)
            self.assertEqual(r.returncode, 5)
            self.assertIn('never overwritten', r.stderr)
            with open(out) as f:
                self.assertEqual(json.load(f), {'keep': True})


if __name__ == '__main__':
    unittest.main()
