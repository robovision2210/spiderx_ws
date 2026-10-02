"""colcon test: M6.0-D live-enabling design - the single gate and the gated --live wiring.

No ROS graph: the gate is set explicitly per test (monkeypatch; never in package code), and every
enabled-path test injects the in-memory mock transport and fake graph observations. Proves:
  - while the gate is False, --live refuses (exit 3) before configuration, ROS import or input;
  - the gate is the ONLY thing between the existing gates and one goal; with it True, every other
    M6.0-D gate (domain id, readiness, confirmation, fingerprint, single use) still refuses;
  - the M6.0-D safety logic is byte-identical to the merged M6.0-D (main @ e65c213).
"""
import ast
import hashlib
import itertools
import json
import os
import signal
import subprocess
import sys

import m6d_gate
import pytest

from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_mock as mock
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_live_preflight as lp
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC_CONFIG = os.path.join(PKG, 'config')
NAMES, NEUTRAL = lp.contract_from_config(SRC_CONFIG)
STAMP = '20261002T000000Z'


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


class Graph:
    """Read-only observations of a healthy stack (no ROS)."""

    def __init__(self, **kw):
        self.action_servers = [('/leg_trajectory_controller', [lp.env.ACTION_TYPE])]
        self.controllers = [('joint_state_broadcaster', 'x', 'active'),
                            ('leg_trajectory_controller', lp.JTC_TYPE, 'active')]
        self.joint_state_publishers = [('/joint_state_broadcaster', lp.JOINT_STATE_TYPE)]
        self.joint_state_messages = [(1.0 + 0.01 * i, list(NAMES), list(NEUTRAL))
                                     for i in range(4)]
        self.probe_errors = []
        self.__dict__.update(kw)


class Factories:
    """Injected live dependencies: the in-memory mock transport and fake graph observations."""

    def __init__(self, sources, graph=None, **transport_kw):
        self.traj = m6t.build_trajectory(sources)
        self.graph = graph or Graph()
        self.transport_kw = transport_kw
        self.made = []

    def transport(self, fp, domain_id):
        t = mock.FakeTransport(self.traj, fp, **self.transport_kw)
        t.domain_id = domain_id
        self.made.append(t)
        return t

    def collect(self, transport):
        return lambda: self.graph


def word():
    return lambda: lc.CONFIRMATION_WORD + '\n'


def never(*a, **k):
    raise AssertionError('must not be reached')


def live_args(tmp_path, *extra):
    return lpb.parse_args(['--live', '--out', str(tmp_path), *extra])


def run_enabled(sources, tmp_path, f, *extra, reader=None):
    return lpb._live_main(live_args(tmp_path, *extra), sources, reader or word(),
                          transport_factory=f.transport, collect_factory=f.collect,
                          utc_stamp=STAMP)


@pytest.fixture
def enabled(monkeypatch):
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', True)        # tests only


@pytest.fixture
def disabled(monkeypatch):
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', False)


# ---------------------------------------------------------------- the gate itself
def test_gate_matches_the_single_test_expectation():
    # m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED is False in this build (the only place it is pinned)
    assert lc.LIVE_DISPATCH_ENABLED is m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED
    assert ('HARD-DISABLED' in lpb.LIVE_STATE) is (not lc.LIVE_DISPATCH_ENABLED)


def test_gate_false_refuses_live_before_anything(disabled, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(lpb, '_live_main', never)
    monkeypatch.setattr(m6t, 'load_sources', never)
    rc = lpb.main(['m6_live_playback', '--live', '--domain-id', '0', '--out', str(tmp_path)],
                  reader=never)
    assert rc == lpb.EXIT_DISABLED == 3
    assert 'REFUSED' in capsys.readouterr().out and os.listdir(tmp_path) == []


def test_gate_false_blocks_the_live_wiring_too(disabled, sources):
    with pytest.raises(PermissionError):
        lpb._run_live(sources, never, never, never, lpb.InterruptLatch(), 0)


def test_gate_false_live_cli_imports_no_ros_client_and_reads_no_input():
    code = ('import sys; from spiderx_controller import m6_live_playback as m;'
            'm.lc.LIVE_DISPATCH_ENABLED = False;'
            'rc = m.main(["m6_live_playback", "--live", "--domain-id", "0"]);'
            'bad=[x for x in sys.modules if x.split(".")[0] == "rclpy"'
            ' or x.endswith("m6_live_adapter")];'
            'print(rc, bad); sys.exit(0 if rc == 3 and not bad else 1)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                         stdin=subprocess.DEVNULL)
    assert out.returncode == 0, out.stdout + out.stderr


def test_help_states_the_gate_and_has_no_bypass(capsys):
    with pytest.raises(SystemExit):
        lpb.main(['m6_live_playback', '--help'])
    out = capsys.readouterr().out
    assert lpb.LIVE_STATE.split()[0] in out and '--domain-id' in out
    for bad in ('--yes', '--force', '--enable', '--retry', '--repeat', '--trajectory'):
        assert bad not in out


# ---------------------------------------------------------------- gate True: other gates hold
def test_enabled_without_domain_id_refuses_before_ros(enabled, tmp_path, monkeypatch):
    monkeypatch.setattr(m6t, 'load_sources', never)
    for extra in ([], ['--domain-id', '-1'], ['--domain-id', '233']):
        assert lpb._live_main(live_args(tmp_path, *extra), None, never, never, never,
                              utc_stamp=STAMP) == lpb.EXIT_REFUSED
    assert os.listdir(tmp_path) == []


def test_enabled_without_domain_id_imports_no_ros_client():
    code = ('import sys; from spiderx_controller import m6_live_playback as m;'
            'm.lc.LIVE_DISPATCH_ENABLED = True;'          # test process only
            'rc = m.main(["m6_live_playback", "--live", "--no-write"]);'
            'bad=[x for x in sys.modules if x.split(".")[0] == "rclpy"];'
            'print(rc, bad); sys.exit(0 if rc == 2 and not bad else 1)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                         stdin=subprocess.DEVNULL)
    assert out.returncode == 0, out.stdout + out.stderr


def test_domain_id_is_only_valid_with_live(sources, tmp_path):
    assert lpb.main(['x', '--dry-run', '--domain-id', '0', '--out', str(tmp_path)],
                    sources=sources) == lpb.EXIT_REFUSED
    assert os.listdir(tmp_path) == []


def test_enabled_wrong_confirmation_sends_nothing(enabled, sources, tmp_path):
    f = Factories(sources)
    for answer in ('', 'yes\n', 'send-one-crouch-goal\n', ' SEND-ONE-CROUCH-GOAL\n'):
        f.made.clear()
        rc = lpb._live_main(live_args(tmp_path, '--domain-id', '0', '--no-write'), sources,
                            lambda a=answer: a, f.transport, f.collect, utc_stamp=STAMP)
        assert rc == lpb.EXIT_REFUSED
        assert [len(t.sent) for t in f.made] == [0] and f.made[0].closed


@pytest.mark.parametrize('graph, code', [
    (Graph(controllers=None), 'controller_manager_unavailable'),
    (Graph(action_servers=[]), 'action_server_missing'),
    (Graph(joint_state_messages=[]), 'joint_states_no_messages'),
    (Graph(joint_state_publishers=[('/a', lp.JOINT_STATE_TYPE), ('/b', lp.JOINT_STATE_TYPE)]),
     'joint_states_multiple_publishers'),
])
def test_enabled_not_ready_stack_sends_nothing(enabled, sources, tmp_path, graph, code):
    f = Factories(sources, graph=graph)
    rc = run_enabled(sources, tmp_path, f, '--domain-id', '0')
    data = json.loads((tmp_path / 'live' / STAMP / 'live_outcome.json').read_text())
    assert rc == lpb.EXIT_REFUSED and data['state'] == lpb.REFUSED
    assert data['goals_sent'] == 0 and len(f.made[0].sent) == 0 and f.made[0].closed
    if code:
        assert code in json.dumps(data['readiness'])


def test_enabled_stale_readiness_sends_nothing(enabled, sources, tmp_path):
    f = Factories(sources)
    clock = itertools.count(0.0, 20.0)         # every reading 20 s later: always > 10 s old

    def transport(fp, domain_id):
        t = Factories.transport(f, fp, domain_id)
        t.wall_now = lambda: next(clock)
        return t
    rc = lpb._live_main(live_args(tmp_path, '--domain-id', '0', '--no-write'), sources, word(),
                        transport, f.collect, utc_stamp=STAMP)
    assert rc == lpb.EXIT_REFUSED and len(f.made[0].sent) == 0


def test_enabled_healthy_mock_sends_exactly_one_goal(enabled, sources, tmp_path):
    f = Factories(sources)
    rc = run_enabled(sources, tmp_path, f, '--domain-id', '7')
    data = json.loads((tmp_path / 'live' / STAMP / 'live_outcome.json').read_text())
    assert rc == lpb.EXIT_OK and data['state'] == lpb.SUCCEEDED
    assert (data['mode'], data['domain_id'], data['goals_sent']) == ('live', 7, 1)
    assert data['cancels_sent'] == 0 and data['retries'] == 0
    assert data['automatic_return_goals'] == 0
    assert len(f.made) == 1 and len(f.made[0].sent) == 1 and f.made[0].closed
    assert f.made[0].domain_id == 7


@pytest.mark.parametrize('scenario, state, rc', [
    ('interrupt', lpb.CANCEL_CONFIRMED, lpb.EXIT_FAILED),
    ('tracking_error', lpb.TRACKING_FAILED, lpb.EXIT_FAILED),
    ('stale_joint_states', lpb.READINESS_LOST, lpb.EXIT_FAILED),
    ('controller_lost', lpb.HELD_ERROR, lpb.EXIT_FAILED),
    ('rejected', lpb.REJECTED, lpb.EXIT_FAILED),
])
def test_enabled_failures_never_retry_or_return(enabled, sources, tmp_path, scenario, state, rc):
    latch = lpb.InterruptLatch()
    f = Factories(sources, latch=latch, **mock.SCENARIOS[scenario])
    out = lpb._live_main(live_args(tmp_path, '--domain-id', '0', '--no-write'), sources, word(),
                         f.transport, f.collect, latch=latch, utc_stamp=STAMP)
    assert out == rc
    t = f.made[0]
    assert len(f.made) == 1 and len(t.sent) == 1 and t.cancels <= 1 and t.closed


def test_enabled_report_is_never_overwritten(enabled, sources, tmp_path):
    f = Factories(sources)
    assert run_enabled(sources, tmp_path, f, '--domain-id', '0') == lpb.EXIT_OK
    f2 = Factories(sources)
    assert run_enabled(sources, tmp_path, f2, '--domain-id', '0') == lpb.EXIT_REFUSED
    assert f2.made == []                       # refused before any transport was created


def test_enabled_run_restores_signal_handlers(enabled, sources, tmp_path):
    before = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    f = Factories(sources)
    run_enabled(sources, tmp_path, f, '--domain-id', '0', '--no-write')
    assert {s: signal.getsignal(s) for s in before} == before


def test_live_main_has_no_loop_retry_or_direct_send(enabled, sources, tmp_path):
    """_live_main builds one transport and one session; it never loops or sends by itself."""
    f = Factories(sources)
    run_enabled(sources, tmp_path, f, '--domain-id', '0', '--no-write')
    assert len(f.made) == 1 and len(f.made[0].sent) == 1
    fn = [n for n in ast.parse(open(lpb.__file__).read()).body
          if getattr(n, 'name', None) == '_live_main'][0]
    nodes = list(ast.walk(fn))
    assert not [n for n in nodes if isinstance(n, (ast.For, ast.While, ast.comprehension))]
    names = {n.id for n in nodes if isinstance(n, ast.Name)} | \
        {n.attr for n in nodes if isinstance(n, ast.Attribute)}
    assert not names & {'send_goal', 'cancel_goal', 'LiveSession', 'retry'}
    assert [n.func.id for n in nodes if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Name)].count('_run_live') == 1


# ---------------------------------------------------------------- unchanged safety logic
# SHA-256 of each definition's source as merged in main @ e65c213 (PR #15). The enabling change
# must not alter any of them.
UNCHANGED = {
    'InterruptLatch': '66627f50d7fefeaeed439661e581eb85fb6bcfe162beb3b0ff17621209e46570',
    'StreamMonitor': '486e029d82009987f9c65c506896973c3876005bd83a3ace072d14fc59c61925',
    'graph_violation': '0ad1611cad17b2c008ed12e44704c4d89e40e2343b6e3c6d0349d6bbcf6fa332',
    'LiveSession': '109ab5c9a439033cf8a3c0f101becc14a2636c500b881a69a14edf27a8b148c0',
    '_run_live': '4bdd27965f3e1375f24ac2d75288da3b090e15b53e292244e073c6b60be847eb',
}
UNCHANGED_FILES = {
    'm6_goal_fingerprint.py': 'edf942110ca3987a400a44e456554800bc55f30ca695c0684f6e9f4a37eb3a35',
    'm6_live_readiness.py': '6707c999bab9ee0c515b359564ebfd6ce0415321cd56278d18cb929d09db21a0',
    'm6_live_mock.py': '1c7c8dd897601936b91db85886f5207d6bd458b9387903013429c01531ae60e3',
}


def test_session_and_gates_are_byte_identical_to_merged_m6d():
    src = open(lpb.__file__).read()
    got = {n.name: hashlib.sha256((ast.get_source_segment(src, n, padded=True) + '\n')
                                  .encode()).hexdigest()
           for n in ast.parse(src).body if getattr(n, 'name', None) in UNCHANGED}
    assert got == UNCHANGED


@pytest.mark.parametrize('name', sorted(UNCHANGED_FILES))
def test_fingerprint_readiness_and_mock_modules_are_unchanged(name):
    with open(os.path.join(PKG, 'spiderx_controller', name), 'rb') as f:
        assert hashlib.sha256(f.read()).hexdigest() == UNCHANGED_FILES[name]


def test_limits_are_unchanged():
    d = lc.as_dict()
    assert lc.CONFIRMATION_WORD == 'SEND-ONE-CROUCH-GOAL'
    assert lc.APPROVED_POINT_TIMES_S == (3.0, 6.0, 9.0) and lc.APPROVED_GOAL_COUNT == 1
    assert (lc.GOAL_POSITION_TOLERANCE_RAD, lc.PATH_POSITION_TOLERANCE_RAD,
            lc.GOAL_VELOCITY_TOLERANCE_RAD_S, lc.GOAL_TIME_TOLERANCE_S) == (0.05, 0.05, 0.05, 1.0)
    assert (lc.GOAL_RESPONSE_TIMEOUT_S, lc.CANCEL_RESPONSE_TIMEOUT_S, lc.RESULT_WATCHDOG_S,
            lc.READINESS_MAX_AGE_S) == (10.0, 5.0, 120.0, 10.0)
    assert d['live_dispatch_enabled'] is lc.LIVE_DISPATCH_ENABLED
