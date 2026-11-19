#!/usr/bin/env python3
"""Tests of the S4e run-local harness template (s4e_harness_template/; docs/S4E_CONTRACT.md 3-6).

STUBS ONLY: the launch, the recorder, the captures, the gates import, the free-disk query and the
restore are replaced by small stub commands, in a throwaway git workspace. Nothing is launched,
recorded, enabled or restored for real, and no observer or teleop is started.

Covered: the refusals before the launch (mode, installation, gates, ref, time, disk), the
nominal shadow and live flows, the watchdog, an interruption, owned-only cleanup with unowned
look-alike decoys left running, and the restore when no pid file exists yet.

Usage (stdlib only): cd scripts/s4 && python3 -m unittest -v test_s4e_harness_template"""
import json
import os
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, 's4e_harness_template')
BRANCH = 'local/m55-s4e-h-20261119T000000Z'
CONTRACT = 'src/spiderx_controller/spiderx_controller/m55_contract.py'
GATE = 'src/spiderx_controller/test/m55_gate.py'


def sh(*args, cwd=None):
    return subprocess.run(args, cwd=cwd, capture_output=True, text=True, check=True).stdout


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    with open(f'/proc/{pid}/stat') as f:
        return f.read().split()[2] != 'Z'


def session_members(sid):
    """Live (non-zombie) members of a session."""
    r = subprocess.run(['pgrep', '-s', str(sid)], capture_output=True, text=True)
    return [int(p) for p in r.stdout.split() if alive(int(p))]


class Harness(unittest.TestCase):
    """A run-local copy of the template, a git workspace and stubs, per test."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='s4e_harness_test_')
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.h = os.path.join(self.tmp, 'h')
        shutil.copytree(TEMPLATE, self.h)
        self.ws = os.path.join(self.tmp, 'ws')
        os.makedirs(os.path.join(self.ws, os.path.dirname(CONTRACT)))
        os.makedirs(os.path.join(self.ws, os.path.dirname(GATE)))
        for p, txt in ((CONTRACT, 'M55_LOCOMOTION_DISPATCH_ENABLED = False\n'),
                       (GATE, 'ENABLED = False\n')):
            with open(os.path.join(self.ws, p), 'w') as f:
                f.write(txt)
        sh('git', 'init', '-q', '-b', 'base', cwd=self.ws)
        sh('git', '-c', 'user.name=t', '-c', 'user.email=t@t', 'add', '.', cwd=self.ws)
        sh('git', '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-q', '-m', 'base',
           cwd=self.ws)
        self.baseline = sh('git', 'rev-parse', 'HEAD', cwd=self.ws).strip()
        self.runinst = os.path.join(self.tmp, 'runinst')
        self.session = os.path.join(self.tmp, 'session')
        self.run_dir = os.path.join(self.session, 'run')
        self.bag = os.path.join(self.tmp, 'bags', 'bag')
        os.makedirs(os.path.dirname(self.bag))
        self.calls = os.path.join(self.tmp, 'restore_calls.txt')
        stubs = os.path.join(self.tmp, 'stubs')
        os.makedirs(stubs)
        self.launch_stub = os.path.join(stubs, 'launch.sh')
        with open(self.launch_stub, 'w') as f:
            f.write('#!/usr/bin/env bash\n'
                    'bash -c \'exec -a "ign gazebo server" sleep 300\' &\n'
                    'bash -c \'exec -a m55_locomotion_node sleep 300\' &\n'
                    'echo "M5.5 locomotion node up in SHADOW mode"\n'
                    'trap "kill 0; exit 0" INT TERM\nwait\n')
        self.decoys = []
        for name in ('ign gazebo server', 'ros2 bag record', 'm55_locomotion_node'):
            p = subprocess.Popen(['bash', '-c', f'exec -a "{name}" sleep 300'],
                                 start_new_session=True)
            self.decoys.append(p)
            self.addCleanup(self.kill, p)
        self.env = {
            'PATH': os.environ['PATH'], 'HOME': os.environ.get('HOME', '/root'),
            'S4E_MODE': 'shadow', 'WS': self.ws, 'RUNINST': self.runinst,
            'OLD_INSTALL': os.path.join(self.tmp, 'old_install'), 'BASELINE': self.baseline,
            'AMENT_PREFIX_PATH': f'{self.runinst}/spiderx_controller',
            'PYTHONPATH': f'{self.runinst}/spiderx_controller/lib',
            'WATCHDOG_S': '60', 'SESSION_CAP_S': '1', 'AFTER_S': '1',
            'NODE_UP_WAIT_S': '10', 'PREMOTION_WAIT_S': '8', 'SESSION_WAIT_S': '8',
            'GATES_CMD': f'echo False False False {self.runinst}/spiderx_controller/c.py',
            'DISK_FREE_CMD': 'echo 999999999999999 #',
            'LAUNCH_CMD': f'bash {self.launch_stub}',
            'BAG_CMD': 'bash -c \'exec -a "ros2 bag record" sleep 300\'',
            'CAPTURE_CMD': 'echo "Publisher count: 0"',
            'RESTORE_CMD': f'echo restore >> {self.calls}; echo',
        }

    @staticmethod
    def kill(p):
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        p.wait()

    def enable(self, branch=BRANCH, two_line=True):
        sh('git', 'switch', '-q', '-c', branch, cwd=self.ws)
        with open(os.path.join(self.ws, CONTRACT), 'w') as f:
            f.write('M55_LOCOMOTION_DISPATCH_ENABLED = True\n')
        with open(os.path.join(self.ws, GATE), 'w') as f:
            f.write('ENABLED = True\n' + ('' if two_line else 'X = 1\n'))
        sh('git', '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-q', '-am', 'enable',
           cwd=self.ws)
        with open(os.path.join(self.session, 'enabling_branch.txt'), 'w') as f:
            f.write(branch + '\n')
        gates = f'echo True False False {self.runinst}/spiderx_controller/c.py'
        self.env.update(S4E_MODE='live', GATES_CMD=gates)

    def start_watchdog(self, deadline_s=None):
        os.makedirs(self.run_dir, exist_ok=True)
        args = ['bash', os.path.join(self.h, 'watchdog.sh'), self.run_dir]
        if deadline_s is not None:
            args.append(str(deadline_s))
        wd = subprocess.Popen(args, env=self.env, start_new_session=True)
        self.addCleanup(self.kill, wd)
        for _ in range(100):
            if os.path.exists(os.path.join(self.run_dir, 'watchdog_deadline_epoch.txt')):
                break
            time.sleep(0.05)
        return wd

    def guarded(self):
        p = subprocess.Popen(['bash', os.path.join(self.h, 'guarded.sh'), self.session,
                              self.bag], env=self.env, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, start_new_session=True)
        self.addCleanup(self.kill, p)
        return p

    def steps(self):
        p = os.path.join(self.run_dir, 'steps.log')
        if not os.path.exists(p):
            return ''
        with open(p) as f:
            return f.read()

    def read(self, name):
        p = os.path.join(self.run_dir, name)
        if not os.path.exists(p):
            return ''
        with open(p) as f:
            return f.read()

    def wait_for(self, text, cap=20.0):
        end = time.monotonic() + cap
        while text not in self.steps():
            if time.monotonic() > end:
                raise AssertionError(f'{text!r} never appeared:\n{self.steps()}')
            time.sleep(0.05)

    def finish(self, p, cap=60.0):
        out, _ = p.communicate(timeout=cap)
        return p.returncode, out.decode(errors='replace')

    def restore_calls(self):
        if not os.path.exists(self.calls):
            return 0
        with open(self.calls) as f:
            return f.read().count('restore')

    def assert_decoys_alive(self):
        for d in self.decoys:
            self.assertTrue(alive(d.pid), 'an unowned look-alike was stopped')


class Flows(Harness):
    def nominal(self):
        self.start_watchdog()
        p = self.guarded()
        self.wait_for('PRE-KEY CHECKPOINT')
        with open(os.path.join(self.run_dir, 'go.flag'), 'w') as f:
            f.write('stub pre-key checks passed\n')
        self.wait_for('SESSION:')
        launch_sid = int(self.read('launch.pid'))
        bag_pid = int(self.read('bag.pid'))
        self.assertTrue(session_members(launch_sid))
        os.makedirs(os.path.join(self.session, 'observer'))
        with open(os.path.join(self.session, 'observer', 'observer_summary.json'), 'w') as f:
            json.dump({'result': 'session complete'}, f)
        rc, out = self.finish(p)
        self.assertEqual(rc, 0, self.steps() + out)
        self.assertTrue(self.steps().rstrip().endswith('end'))
        self.assertEqual(session_members(launch_sid), [])
        self.assertFalse(alive(bag_pid))
        self.assert_decoys_alive()
        cl = self.read('cleanup.log')
        self.assertIn('listed, not killed', cl)
        self.assertIn(f'launch session {launch_sid}: stopped by SIGINT', cl)
        self.assertIn(f'recorder group {bag_pid}: stopped by SIGINT', cl)
        for at in ('start', 'end'):
            self.assertEqual(json.loads(self.read(f'process_scan_{at}.json')), [])
        self.assertIn('observer: session complete', self.steps())
        running = subprocess.run(['pgrep', '-af', 's4e_observer'], capture_output=True,
                                 text=True).stdout
        self.assertNotIn('s4e_observer', running)
        self.assertEqual(self.restore_calls(), 1)
        return self.read('restore.log')

    def test_shadow_nominal_flow_and_verify_only_restore(self):
        log = self.nominal()
        self.assertIn('--verify-only', log)
        self.assertIn('--launch-pid-file', log)
        self.assertNotIn('--enabling-ref', log)

    def test_live_flow_restores_with_the_recorded_branch(self):
        os.makedirs(self.session, exist_ok=True)
        self.enable()
        log = self.nominal()
        self.assertIn(f'--enabling-ref {BRANCH} --delete-enabling-branch', log)
        self.assertNotIn('--verify-only', log)


class Refusals(Harness):
    def refused(self, why, setup=None):
        os.makedirs(self.session, exist_ok=True)
        if setup:
            setup()
        self.start_watchdog()
        rc, out = self.finish(self.guarded())
        self.assertEqual(rc, 1, self.steps() + out)
        self.assertIn('REFUSED before the launch (nothing launched)', self.steps())
        self.assertIn(why, self.steps())
        self.assertFalse(os.path.exists(os.path.join(self.run_dir, 'launch.pid')))
        self.assertEqual(self.restore_calls(), 1)          # restored even with no pid file
        self.assertIn('no pid recorded', self.read('cleanup.log'))
        return self.read('restore.log')

    def test_wrong_mode(self):
        self.env['S4E_MODE'] = 'enabled'
        self.refused("is not shadow or live")

    def test_wrong_gates_for_the_mode(self):
        self.env['GATES_CMD'] = f'echo True False False {self.runinst}/spiderx_controller/c.py'
        self.refused("shadow needs the gates 'False False False'")

    def test_live_with_gates_false(self):
        def setup():
            self.enable()
            self.env['GATES_CMD'] = f'echo False False False {self.runinst}/x/c.py'
        self.refused("live needs the gates 'True False False'", setup)

    def test_gates_from_another_installation(self):
        self.env['GATES_CMD'] = 'echo False False False /elsewhere/c.py'
        self.refused('were not imported from the run installation')

    def test_the_old_installation_in_the_environment(self):
        self.env['AMENT_PREFIX_PATH'] += ':' + self.env['OLD_INSTALL']
        self.refused('is in the environment')

    def test_the_run_installation_missing(self):
        self.env['AMENT_PREFIX_PATH'] = '/opt/other'
        self.env['PYTHONPATH'] = ''
        self.refused('is not in the environment')

    def test_shadow_not_at_the_baseline(self):
        def setup():
            with open(os.path.join(self.ws, 'extra.txt'), 'w') as f:
                f.write('x\n')
            sh('git', '-c', 'user.name=t', '-c', 'user.email=t@t', 'add', '.', cwd=self.ws)
            sh('git', '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-q', '-m', 'x',
               cwd=self.ws)
        self.refused('HEAD is not the disabled baseline', setup)

    def test_a_dirty_worktree(self):
        def setup():
            with open(os.path.join(self.ws, CONTRACT), 'a') as f:
                f.write('# edit\n')
        self.refused('is not clean', setup)

    def test_live_ref_refusals(self):
        cases = [
            ('local/m55-s4e-20261119T000000Z', True, None, 'no recorded local/m55-s4e-h-*'),
            (BRANCH, False, None, 'not exactly the two-line M55 patch'),
            (BRANCH, True, 'pushed', 'remote-tracking ref'),
        ]
        for branch, two_line, extra, why in cases:
            with self.subTest(why=why):
                self.setUp()

                def setup(branch=branch, two_line=two_line, extra=extra):
                    self.enable(branch, two_line)
                    if extra == 'pushed':
                        sh('git', 'update-ref', f'refs/remotes/origin/{branch}', branch,
                           cwd=self.ws)
                self.refused(why, setup)

    def test_watchdog_not_running(self):
        os.makedirs(self.session, exist_ok=True)
        rc, out = self.finish(self.guarded())
        self.assertEqual(rc, 1)
        self.assertIn('the watchdog is not running', self.steps())
        self.assertEqual(self.restore_calls(), 1)

    def test_too_little_watchdog_time(self):
        self.env.update(SESSION_CAP_S='1200', AFTER_S='900')
        self.refused('s left < 1200 + 900 s')

    def test_too_little_disk(self):
        self.env['DISK_FREE_CMD'] = 'echo 1000 #'
        self.refused('B free <')


class Safety(Harness):
    def test_the_watchdog_cleans_up_and_restores(self):
        os.makedirs(self.session, exist_ok=True)
        self.env['PREMOTION_WAIT_S'] = '30'
        self.start_watchdog(deadline_s=4)
        p = self.guarded()
        self.wait_for('PRE-KEY CHECKPOINT')
        launch_sid = int(self.read('launch.pid'))
        end = time.monotonic() + 20
        while 'watchdog: done' not in self.read('watchdog.txt') and time.monotonic() < end:
            time.sleep(0.1)
        self.assertIn('deadline reached', self.read('watchdog.txt'))
        self.assertTrue(os.path.exists(os.path.join(self.run_dir, 'stopped.flag')))
        self.assertEqual(session_members(launch_sid), [])
        self.assertEqual(self.restore_calls(), 1)
        self.assert_decoys_alive()
        os.kill(p.pid, signal.SIGTERM)
        self.finish(p)
        self.assertEqual(self.restore_calls(), 1)                    # idempotent

    def test_an_interruption_stops_owned_processes_only_and_restores(self):
        os.makedirs(self.session, exist_ok=True)
        self.start_watchdog()
        p = self.guarded()
        self.wait_for('PRE-KEY CHECKPOINT')
        launch_sid = int(self.read('launch.pid'))
        self.assertTrue(session_members(launch_sid))
        os.kill(p.pid, signal.SIGTERM)
        rc, out = self.finish(p)
        self.assertNotEqual(rc, 0)
        self.assertEqual(session_members(launch_sid), [])
        self.assert_decoys_alive()
        self.assertEqual(self.restore_calls(), 1)
        self.assertTrue(os.path.exists(os.path.join(self.run_dir, 'stopped.flag')))

    def test_an_interruption_before_any_pid_file_still_restores(self):
        os.makedirs(self.session, exist_ok=True)
        self.env['GATES_CMD'] = f'sleep 5; echo False False False {self.runinst}/s/c.py'
        self.start_watchdog()
        p = self.guarded()
        end = time.monotonic() + 10
        while 'gates' not in self.steps() and time.monotonic() < end:
            time.sleep(0.05)
            if os.path.exists(os.path.join(self.run_dir, 'env_run.txt')):
                break
        time.sleep(0.3)
        os.kill(p.pid, signal.SIGTERM)
        self.finish(p)
        self.assertFalse(os.path.exists(os.path.join(self.run_dir, 'launch.pid')))
        cl = self.read('cleanup.log')
        self.assertIn('recorder: no pid recorded: not running', cl)
        self.assertIn('launch: no pid recorded: not running', cl)
        self.assertEqual(self.restore_calls(), 1)
        self.assertIn('--verify-only', self.read('restore.log'))
        self.assertNotIn('--launch-pid-file', self.read('restore.log'))


if __name__ == '__main__':
    unittest.main()
