"""colcon test: scripts/m61_restore_and_verify_disabled.sh (M6.1 one-cycle plan, E11).

Hermetic: each test builds a throwaway git workspace whose install/ points at its src/ (as a
symlink install does) and whose `ros2` is a stand-in: `pkg prefix`, `daemon stop`, and
`run ... --live`, which refuses with exit 3 when the imported gate is False and otherwise only
leaves a marker file. No ROS graph, no simulator, and the real workspace is never touched.
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
FAKE_ROS2 = """#!/usr/bin/env bash
case "$1 $2" in
  "pkg prefix") echo "{ws}/install/spiderx_controller" ;;
  "daemon stop") exit 0 ;;
  "run spiderx_controller")
    if python3 -c "import sys, spiderx_controller.m61_live_contract as c
sys.exit(0 if c.M61_LIVE_DISPATCH_ENABLED is False else 1)"; then
      echo "REFUSED: Live M6.1 trot-cycle dispatch is HARD-DISABLED in this build."; exit 3
    fi
    touch "{ws}/LIVE_PATH_REACHED"; echo "live path reached"; exit 0 ;;
  *) echo "fake ros2: $*" >&2; exit 64 ;;
esac
"""
# Same patterns as the script's leftovers() (arguments), for the precondition below.
SIM_ARGS = re.compile(r'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|ros2 launch|'
                      r'_ros2_daemon|Xvfb|spawner|create -name|ros_gz')


def git(ws, *args):
    return subprocess.run(['git', *GIT, '-C', str(ws), *args], check=True, capture_output=True,
                          text=True).stdout.strip()


def write_gates(ws, m61=False, m6d=False):
    pkg = ws / 'src' / 'spiderx_controller' / 'spiderx_controller'
    pkg.mkdir(parents=True, exist_ok=True)
    (pkg / '__init__.py').write_text('')
    (pkg / 'm61_live_contract.py').write_text(f'M61_LIVE_DISPATCH_ENABLED = {m61}\n')
    (pkg / 'm6_live_contract.py').write_text(f'LIVE_DISPATCH_ENABLED = {m6d}\n')


@pytest.fixture
def ws(tmp_path):
    """A disabled workspace on branch `base`, with a fake symlink install and ROS setup."""
    w = tmp_path / 'ws'
    w.mkdir()
    subprocess.run(['git', 'init', '-q', str(w)], check=True)
    git(w, 'symbolic-ref', 'HEAD', 'refs/heads/base')
    write_gates(w)
    (w / '.gitignore').write_text('build/\ninstall/\nlog/\n__pycache__/\nLIVE_PATH_REACHED\n')
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


def enable_on_branch(w):
    git(w, 'switch', '-q', '-c', ENABLING)
    write_gates(w, m61=True)
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
                 'ok: a. both gates False', 'ok: b. ros2 pkg prefix',
                 'ok: c. --live refused, exit 3, HARD-DISABLED',
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
                                    'ok: c. --live refused', '--- 6. enabling branch',
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
    assert 'ok: a. both gates False' in rep                      # the fresh shell is fine
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
