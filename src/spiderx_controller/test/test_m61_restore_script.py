"""colcon test: scripts/m61_restore_and_verify_disabled.sh (M6.1 one-cycle plan, E11; M5.5 S2,
P8).

Hermetic: each test builds a throwaway git workspace whose install/ points at its src/ (as a
symlink install does) and whose `ros2` is a stand-in: `pkg prefix`, `daemon stop`, and
`run ... m6_gait_replay.py|m6_live_playback --live`, each of which refuses with exit 3 when its
imported gate is False and otherwise only leaves a marker file. The script must also check the
M5.5 gate (m55_contract, imported) and accept local/m55-s2-* enabling branches. No ROS graph, no
simulator, and the real workspace is never touched.
"""
import os
import re
import subprocess
import time

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'scripts',
                      'm61_restore_and_verify_disabled.sh')
PATH = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
GIT = ['-c', 'user.name=test', '-c', 'user.email=test@example.invalid']
ENABLING = 'local/m61-one-cycle-20261010T000000Z'
ENABLING_S2 = 'local/m55-s2-20261013T000000Z'
FAKE_ROS2 = """#!/usr/bin/env bash
case "$1 $2 $3" in
  "pkg prefix spiderx_controller") echo "{ws}/install/spiderx_controller" ;;
  "daemon stop ") exit 0 ;;
  "run spiderx_controller m6_gait_replay.py")
    if python3 -c "import sys, spiderx_controller.m61_live_contract as c
sys.exit(0 if c.M61_LIVE_DISPATCH_ENABLED is False else 1)"; then
      echo "REFUSED: Live M6.1 trot-cycle dispatch is HARD-DISABLED in this build."; exit 3
    fi
    touch "{ws}/LIVE_PATH_REACHED"; echo "live path reached"; exit 0 ;;
  "run spiderx_controller m6_live_playback")
    [ -e "{ws}/PLAYBACK_BROKEN" ] && {{ touch "{ws}/LIVE_PATH_REACHED"; echo "started"; exit 0; }}
    if python3 -c "import sys, spiderx_controller.m6_live_contract as c
sys.exit(0 if c.LIVE_DISPATCH_ENABLED is False else 1)"; then
      echo "REFUSED: Live M6.0-D dispatch is HARD-DISABLED in this build."; exit 3
    fi
    touch "{ws}/LIVE_PATH_REACHED"; echo "live path reached"; exit 0 ;;
  *) echo "fake ros2: $*" >&2; exit 64 ;;
esac
"""
# Same patterns as the script's leftovers() (arguments), for the precondition below.
SIM_ARGS = re.compile(r'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|ros2 launch|'
                      r'_ros2_daemon|Xvfb|spawner|create -name|ros_gz|m55_locomotion_node|'
                      r'm55_teleop_keyboard')


def git(ws, *args):
    return subprocess.run(['git', *GIT, '-C', str(ws), *args], check=True, capture_output=True,
                          text=True).stdout.strip()


def write_gates(ws, m61=False, m6d=False, m55=False):
    pkg = ws / 'src' / 'spiderx_controller' / 'spiderx_controller'
    pkg.mkdir(parents=True, exist_ok=True)
    (pkg / '__init__.py').write_text('')
    (pkg / 'm61_live_contract.py').write_text(f'M61_LIVE_DISPATCH_ENABLED = {m61}\n')
    (pkg / 'm6_live_contract.py').write_text(f'LIVE_DISPATCH_ENABLED = {m6d}\n')
    (pkg / 'm55_contract.py').write_text(f'M55_LOCOMOTION_DISPATCH_ENABLED = {m55}\n')


@pytest.fixture
def ws(tmp_path):
    """A disabled workspace on branch `base`, with a fake symlink install and ROS setup."""
    w = tmp_path / 'ws'
    w.mkdir()
    subprocess.run(['git', 'init', '-q', str(w)], check=True)
    git(w, 'symbolic-ref', 'HEAD', 'refs/heads/base')
    write_gates(w)
    (w / '.gitignore').write_text('build/\ninstall/\nlog/\n__pycache__/\nLIVE_PATH_REACHED\n'
                                  'PLAYBACK_BROKEN\n')
    git(w, 'add', '-A')
    git(w, 'commit', '-q', '-m', 'disabled base')
    site = w / 'install' / 'lib' / 'site-packages'
    site.mkdir(parents=True)
    (site / 'spiderx_controller').symlink_to(w / 'src' / 'spiderx_controller' /
                                             'spiderx_controller')
    (w / 'install' / 'spiderx_controller').mkdir()
    bin_dir = w / 'install' / 'bin'
    bin_dir.mkdir()
    (bin_dir / 'ros2').write_text(FAKE_ROS2.format(ws=w))
    (bin_dir / 'ros2').chmod(0o755)
    (w / 'install' / 'setup.bash').write_text(
        f'export PYTHONPATH="{site}${{PYTHONPATH:+:$PYTHONPATH}}"\nexport PATH="{bin_dir}:$PATH"\n')
    (tmp_path / 'ros_setup.sh').write_text('true\n')
    return w


def enable_on_branch(w, branch=ENABLING, **gates):
    git(w, 'switch', '-q', '-c', branch)
    write_gates(w, **(gates or {'m61': True}))
    git(w, 'commit', '-q', '-am', 'LOCAL ONLY: enable')
    return git(w, 'rev-parse', 'HEAD')


@pytest.fixture
def quiet_host():
    """The script must find no simulation process. Stop a ros2 CLI daemon left by other work
    (harmless: it restarts on demand); anything else matching is a real problem."""
    def matching():
        out = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout
        return [ln for ln in out.splitlines() if SIM_ARGS.search(ln)
                and 'm61_restore_and_verify_disabled' not in ln]
    if any('_ros2_daemon' in ln for ln in matching()):
        subprocess.run(['bash', '-c', 'ros2 daemon stop'], capture_output=True, timeout=60)
        time.sleep(1.0)
    left = matching()
    assert not left, f'simulation-like processes already running: {left}'


def run(w, *extra, env_extra=None, prefix=''):
    env = {'PATH': PATH, 'HOME': str(w.parent), 'USER': 'test'}
    env.update(env_extra or {})
    cmd = (f'{prefix}bash {SCRIPT} --ws {w} --ros-setup {w.parent / "ros_setup.sh"} '
           f'--evidence {w.parent / "evidence"} ' + ' '.join(extra))
    r = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, env=env, timeout=300)
    return r.returncode, r.stdout + r.stderr


def report(w):
    (f,) = sorted((w.parent / 'evidence').glob('restore_verify_*.txt'))[-1:]
    return f.read_text()


def test_a_disabled_workspace_verifies(ws, quiet_host):
    rc, out = run(ws, '--base base', '--verify-only')
    assert rc == 0, out
    rep = report(ws)
    for line in ('ok: no simulation process', 'ok: src/ equals the base',
                 'ok: a. all three gates False', 'ok: b. ros2 pkg prefix',
                 'ok: c. m6_gait_replay.py --live refused, exit 3, HARD-DISABLED',
                 'ok: c. m6_live_playback --live refused, exit 3, HARD-DISABLED',
                 'spiderx_controller.m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED = False',
                 'ok: d. the calling environment cannot import',
                 'VERDICT: DISABLED AND VERIFIED'):
        assert line in rep, rep
    assert not (ws / 'LIVE_PATH_REACHED').exists()


def test_an_enabled_checkout_fails_and_never_reaches_the_live_path(ws, quiet_host):
    enable_on_branch(ws)
    rc, out = run(ws, '--base base', '--verify-only')
    assert rc == 1, out
    rep = report(ws)
    assert 'FAIL: src/ differs from the base' in rep
    assert 'M61_LIVE_DISPATCH_ENABLED = True' in rep
    assert 'FAIL: a. imported gate module enabled' in rep
    assert 'FAIL: c. --live NOT run' in rep
    assert 'VERDICT: NOT VERIFIED' in rep
    assert not (ws / 'LIVE_PATH_REACHED').exists()        # the enabled path was never invoked


def test_full_restore_preserves_before_restoring_and_deletes_only_after_verifying(
        ws, quiet_host):
    commit = enable_on_branch(ws)
    rc, out = run(ws, '--base base', f'--enabling-ref {ENABLING}', "--build-cmd 'true'",
                  '--delete-enabling-branch')
    assert rc == 0, out
    ev = ws.parent / 'evidence'
    assert (ev / 'enabling_commit.txt').read_text().split() == [ENABLING, commit]
    assert '+M61_LIVE_DISPATCH_ENABLED = True' in (ev / 'enabling_commit.patch').read_text()
    assert '+M61_LIVE_DISPATCH_ENABLED = True' in (ev / 'enabling_src_vs_base.diff').read_text()
    assert git(ws, 'rev-parse', '--abbrev-ref', 'HEAD') == 'base'
    assert git(ws, 'branch', '--list', 'local/m61-one-cycle-*') == ''
    rep = report(ws)
    order = [rep.index(s) for s in ('--- 2. preserve', '--- 3. restore', 'ok: rebuilt',
                                    'ok: c. m6_live_playback --live refused',
                                    '--- 6. enabling branch',
                                    f'Deleted branch {ENABLING}')]
    assert order == sorted(order), rep
    assert not (ws / 'LIVE_PATH_REACHED').exists()


def test_a_failed_check_keeps_the_enabling_branch(ws, quiet_host):
    enable_on_branch(ws)
    (ws / 'install' / 'setup.bash').write_text('true\n')          # the install is broken
    rc, _ = run(ws, '--base base', f'--enabling-ref {ENABLING}', "--build-cmd 'true'",
                '--delete-enabling-branch')
    assert rc == 1
    assert git(ws, 'branch', '--list', ENABLING).strip() == ENABLING
    assert 'not deleted: a check failed' in report(ws)


def test_the_calling_environment_must_not_import_an_enabled_module(ws, tmp_path, quiet_host):
    shadow = tmp_path / 'shadow' / 'spiderx_controller'
    shadow.mkdir(parents=True)
    (shadow / '__init__.py').write_text('')
    (shadow / 'm61_live_contract.py').write_text('M61_LIVE_DISPATCH_ENABLED = True\n')
    (shadow / 'm6_live_contract.py').write_text('LIVE_DISPATCH_ENABLED = False\n')
    rc, out = run(ws, '--base base', '--verify-only',
                  env_extra={'PYTHONPATH': str(tmp_path / 'shadow')})
    assert rc == 1, out
    rep = report(ws)
    assert 'ok: a. all three gates False' in rep                 # the fresh shell is fine
    assert 'FAIL: d. the calling environment imports an enabled module' in rep


def test_leftover_processes_fail_but_the_invoking_shell_is_not_one(ws, quiet_host):
    dummy = subprocess.Popen(['bash', '-c', 'exec -a parameter_bridge sleep 120'])
    try:
        time.sleep(0.5)
        # the invoking shell's own command line mentions the patterns: it must not be reported
        rc, out = run(ws, '--base base', '--verify-only', prefix=': ign gazebo ros2 launch; ')
        assert rc == 1, out
        rep = report(ws)
        assert 'FAIL: simulation processes remain:' in rep
        listed = [ln for ln in rep.splitlines() if re.match(r'^\d+ ', ln)]
        assert [int(ln.split()[0]) for ln in listed] == [dummy.pid], listed
    finally:
        dummy.kill()
        dummy.wait()


def test_uncommitted_src_changes_are_saved_and_discarded_only_on_request(ws, quiet_host):
    write_gates(ws, m61=True)                                      # an interrupted apply
    rc, _ = run(ws, '--base base', "--build-cmd 'true'")
    assert rc == 1
    saved = sorted((ws.parent / 'evidence').glob('uncommitted_src_*.diff'))
    assert saved and '+M61_LIVE_DISPATCH_ENABLED = True' in saved[-1].read_text()
    assert 'True' in (ws / 'src/spiderx_controller/spiderx_controller/m61_live_contract.py'
                      ).read_text()                                # nothing discarded
    time.sleep(1.1)                                                # a new report name (UTC s)
    rc, out = run(ws, '--base base', "--build-cmd 'true'", '--discard-uncommitted-src')
    assert rc == 0, out
    assert 'False' in (ws / 'src/spiderx_controller/spiderx_controller/m61_live_contract.py'
                       ).read_text()


def test_usage_errors_exit_2(ws):
    assert run(ws)[0] == 2                                         # no --base
    assert run(ws, '--base base', '--bogus')[0] == 2


# ------------------------------------------------------------------ M5.5 S2 (P8)
def test_an_enabled_m55_gate_fails_and_never_reaches_a_live_path(ws, quiet_host):
    enable_on_branch(ws, ENABLING_S2, m55=True)
    rc, out = run(ws, '--base base', '--verify-only')
    assert rc == 1, out
    rep = report(ws)
    assert 'M55_LOCOMOTION_DISPATCH_ENABLED = True' in rep
    assert 'FAIL: a. imported gate module enabled' in rep
    assert 'FAIL: c. --live NOT run' in rep
    assert 'VERDICT: NOT VERIFIED' in rep
    assert not (ws / 'LIVE_PATH_REACHED').exists()


def test_an_m55_module_missing_from_the_workspace_fails(ws, quiet_host):
    (ws / 'src/spiderx_controller/spiderx_controller/m55_contract.py').unlink()
    git(ws, 'commit', '-q', '-am', 'no m55 contract')
    rc, out = run(ws, '--base base', '--verify-only')
    assert rc == 1, out
    assert 'FAIL: a. imported gate module enabled, unreadable' in report(ws)


def test_full_s2_restore_preserves_restores_and_deletes_the_m55_branch(ws, quiet_host):
    commit = enable_on_branch(ws, ENABLING_S2, m55=True)
    rc, out = run(ws, '--base base', f'--enabling-ref {ENABLING_S2}', "--build-cmd 'true'",
                  '--delete-enabling-branch')
    assert rc == 0, out
    ev = ws.parent / 'evidence'
    assert (ev / 'enabling_commit.txt').read_text().split() == [ENABLING_S2, commit]
    assert '+M55_LOCOMOTION_DISPATCH_ENABLED = True' in (ev / 'enabling_commit.patch').read_text()
    assert git(ws, 'rev-parse', '--abbrev-ref', 'HEAD') == 'base'
    assert git(ws, 'branch', '--list', 'local/m55-s2-*') == ''
    rep = report(ws)
    assert 'ok: a. all three gates False' in rep and f'Deleted branch {ENABLING_S2}' in rep
    assert rep.index('ok: c. m6_live_playback --live refused') < rep.index('Deleted branch')


def test_a_branch_outside_both_patterns_is_never_deleted(ws, quiet_host):
    other = 'local/m55-other-20261013T000000Z'
    enable_on_branch(ws, other, m55=True)
    rc, _ = run(ws, '--base base', f'--enabling-ref {other}', "--build-cmd 'true'",
                '--delete-enabling-branch')
    assert rc == 1
    assert git(ws, 'branch', '--list', other).strip() == other
    assert 'refusing to delete' in report(ws)


def test_a_live_playback_tool_that_does_not_refuse_fails(ws, quiet_host):
    (ws / 'PLAYBACK_BROKEN').write_text('')
    rc, out = run(ws, '--base base', '--verify-only')
    assert rc == 1, out
    rep = report(ws)
    assert 'ok: c. m6_gait_replay.py --live refused' in rep
    assert 'FAIL: c. m6_live_playback --live exit 0 (expected 3 with HARD-DISABLED)' in rep


def test_a_leftover_m55_node_fails(ws, quiet_host):
    dummy = subprocess.Popen(['bash', '-c', 'exec -a m55_locomotion_node sleep 120'])
    try:
        time.sleep(0.5)
        rc, out = run(ws, '--base base', '--verify-only')
        assert rc == 1, out
        rep = report(ws)
        assert 'FAIL: simulation processes remain:' in rep
        listed = [ln for ln in rep.splitlines() if re.match(r'^\d+ ', ln)]
        assert [int(ln.split()[0]) for ln in listed] == [dummy.pid], listed
    finally:
        dummy.kill()
        dummy.wait()
