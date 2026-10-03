"""colcon test: M6.0-D Batch D - same-process readiness provider and the m6_live_playback CLI.

No ROS graph: the readiness provider is fed fake graph observations; --dry-run builds messages
only; --mock uses the in-memory mock transport; --live is hard-disabled. Proves zero dispatch in
dry-run/mock modes, report determinism, the hard gate and the absence of bypass options.
"""
import filecmp
import json
import os
import subprocess
import sys

import m6d_gate
import pytest

from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_mock as mock
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_live_preflight as lp
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC_CONFIG = os.path.join(PKG, 'config')
NAMES, NEUTRAL = lp.contract_from_config(SRC_CONFIG)


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


def word():
    return lambda: lc.CONFIRMATION_WORD + '\n'


def cli(sources, tmp_path, *args, reader=None):
    return lpb.main(['m6_live_playback', *args, '--out', str(tmp_path)], sources=sources,
                    reader=reader or word())


# ---------------------------------------------------------------- readiness provider
class Graph:
    def __init__(self, **kw):
        self.action_servers = [('/leg_trajectory_controller', [lp.env.ACTION_TYPE])]
        self.controllers = [('joint_state_broadcaster', 'x', 'active'),
                            ('leg_trajectory_controller', lp.JTC_TYPE, 'active')]
        self.joint_state_publishers = [('/joint_state_broadcaster', lp.JOINT_STATE_TYPE)]
        self.joint_state_messages = [(1.0 + 0.01 * i, list(NAMES), [0.0] * 12) for i in range(4)]
        self.probe_errors = []
        self.__dict__.update(kw)


class Clock:
    def __init__(self, t=50.0):
        self.t = t

    def __call__(self):
        return self.t


def provider(clock, graph=None, raise_exc=None, versions=None):
    def collect():
        clock.t += 2.5                                   # observation takes 2.5 s
        if raise_exc:
            raise raise_exc
        return graph or Graph()
    iface = mock.mock_readiness_report(NAMES, NEUTRAL)['interface']
    return rd.make_readiness_provider(collect, NAMES, NEUTRAL, clock,
                                      versions=versions or dict(mock.MOCK_VERSIONS),
                                      interface=iface)


def test_provider_ready_warning_with_owner_versions():
    clock = Clock()
    r = provider(clock)()
    assert r.ready and r.label == rd.WARNING and len(r.reasons) == 8
    # PR #16 finding 3: stamped when collection STARTS (oldest evidence); the 2.5 s the
    # collection took is recorded and already counted in the age
    assert r.observed_monotonic == 50.0 and r.collection_s == 2.5
    assert r.to_dict(52.5)['age_s'] == 2.5 and r.to_dict()['collection_s'] == 2.5
    assert rd.dispatch_permitted(r, 60.0) == (True, None)
    assert rd.dispatch_permitted(r, 60.1) == (False, 'readiness_stale')


@pytest.mark.parametrize('graph, code', [
    (Graph(action_servers=[]), 'stack_incompatible'),
    (Graph(controllers=[('joint_state_broadcaster', 'x', 'active'),
                        ('leg_trajectory_controller', lp.JTC_TYPE, 'inactive')]),
     'readiness_not_ready'),
    (Graph(joint_state_publishers=[('/a', lp.JOINT_STATE_TYPE), ('/b', lp.JOINT_STATE_TYPE)]),
     'readiness_not_ready'),
    (Graph(joint_state_messages=[(5.0, list(NAMES), [0.0] * 12)] * 3), 'readiness_not_ready'),
    (Graph(joint_state_messages=[(1.0 + i, list(NAMES), [0.0] * 11 + [0.06]) for i in range(3)]),
     'readiness_not_ready'),
])
def test_provider_refusals(graph, code):
    clock = Clock()
    r = provider(clock, graph=graph)()
    assert rd.dispatch_permitted(r, clock.t)[1] == code


def test_provider_graph_exception_is_not_ready():
    clock = Clock()
    r = provider(clock, raise_exc=RuntimeError('graph gone'))()
    assert not r.ready
    assert rd.dispatch_permitted(r, clock.t)[0] is False


def test_provider_compatible_with_reference_versions():
    r = provider(Clock(), versions=dict(lp.REFERENCE_VERSIONS))()
    assert r.label == rd.COMPATIBLE and r.ready


# ---------------------------------------------------------------- CLI options
def test_help_lists_no_bypass_options(capsys):
    with pytest.raises(SystemExit) as e:
        lpb.main(['m6_live_playback', '--help'])
    assert e.value.code == 0
    text = capsys.readouterr().out
    for opt in ('--yes', '--force', '--trajectory', '--repeat', '--cycles', '--retry',
                '--config-dir', '--skip', '--no-confirm', '--bypass'):
        assert opt not in text
    assert lpb.LIVE_STATE.split()[0] in text


def test_a_mode_is_required():
    with pytest.raises(SystemExit) as e:
        lpb.main(['m6_live_playback'])
    assert e.value.code == 2


def test_live_mode_is_hard_disabled(sources, tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', False)
    calls = []
    assert lpb.main(['m6_live_playback', '--live', '--out', str(tmp_path)], sources=sources,
                    reader=lambda: calls.append(1) or lc.CONFIRMATION_WORD) == lpb.EXIT_DISABLED
    out = capsys.readouterr().out
    assert 'HARD-DISABLED' in out and 'separate owner authorization' in out
    assert calls == [] and os.listdir(tmp_path) == []      # no input read, nothing written


def test_live_mode_imports_no_ros_client():
    code = ('import sys; from spiderx_controller import m6_live_playback as m;'
            'm.lc.LIVE_DISPATCH_ENABLED = False;'
            'rc = m.main(["m6_live_playback", "--live"]);'
            'bad=[x for x in sys.modules if x.split(".")[0] == "rclpy"];'
            'print(rc, bad); sys.exit(0 if rc == 3 and not bad else 1)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                         stdin=subprocess.DEVNULL)
    assert out.returncode == 0, out.stdout + out.stderr


def test_run_live_refuses_before_touching_anything(sources, monkeypatch):
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', False)

    def boom(*a):
        raise AssertionError('factory must not be called')
    with pytest.raises(PermissionError):
        lpb._run_live(sources, boom, boom, word(), lpb.InterruptLatch(), 151)


def test_live_wiring_only_needs_the_gate(sources, monkeypatch):
    """MUTATION-style: with the gate flipped (tests only) and injected fakes, the live wiring
    sends exactly one approved goal - so the flag is what keeps live dispatch off."""
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', True)
    traj = m6t.build_trajectory(sources)
    made = []

    def transport_factory(fp, domain_id):
        t = mock.FakeTransport(traj, fp)
        made.append(t)
        return t
    out = lpb._run_live(sources, transport_factory, lambda t: (lambda: Graph()), word(),
                        lpb.InterruptLatch(), 151)
    assert out['state'] == lpb.SUCCEEDED and len(made[0].sent) == 1 and made[0].closed


# ---------------------------------------------------------------- dry run
def test_dry_run_passes_and_records_the_goal_contract(sources, tmp_path):
    assert cli(sources, tmp_path, '--dry-run') == lpb.EXIT_OK
    (tid,) = os.listdir(tmp_path / 'dry_run')
    data = json.loads((tmp_path / 'dry_run' / tid / 'dry_run_report.json').read_text())
    assert data['verdict'] == 'PASS' and data['goals_sent'] == 0
    assert data['live_dispatch_enabled'] is m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED
    spec = data['goal_spec']
    assert all(t['velocity'] == 0.05 and t['position'] == 0.05 for t in spec['goal_tolerance'])
    assert spec['header_stamp_ns'] == 0 and spec['goal_time_tolerance_ns'] == 1_000_000_000
    assert [p['time_ns'] for p in spec['points']] == [3_000_000_000, 6_000_000_000,
                                                      9_000_000_000]
    assert len(data['goal_fingerprint']) == 64


def test_dry_run_is_deterministic_and_never_overwrites(sources, tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    assert cli(sources, a, '--dry-run') == lpb.EXIT_OK
    assert cli(sources, b, '--dry-run') == lpb.EXIT_OK
    assert not filecmp.dircmp(a, b).diff_files
    (tid,) = os.listdir(a / 'dry_run')
    assert filecmp.cmp(a / 'dry_run' / tid / 'dry_run_report.json',
                       b / 'dry_run' / tid / 'dry_run_report.json', shallow=False)
    assert cli(sources, a, '--dry-run') == lpb.EXIT_REFUSED


def test_dry_run_and_mock_import_no_ros_client(sources):
    code = ('import sys, io; from spiderx_controller import m6_live_playback as m;'
            'sys.stdin = io.StringIO("SEND-ONE-CROUCH-GOAL\\n");'
            'a = m.main(["x", "--dry-run", "--no-write"]);'
            'b = m.main(["x", "--mock", "--no-write"]);'
            'bad=[x for x in sys.modules if x.split(".")[0] == "rclpy"];'
            'print(a, b, bad); sys.exit(0 if (a, b) == (0, 0) and not bad else 1)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr


# ---------------------------------------------------------------- mock
@pytest.mark.parametrize('scenario, state, code', [
    ('success', lpb.SUCCEEDED, lpb.EXIT_OK),
    ('interrupt', lpb.CANCEL_CONFIRMED, lpb.EXIT_FAILED),
    ('tracking_error', lpb.TRACKING_FAILED, lpb.EXIT_FAILED),
    ('stale_joint_states', lpb.READINESS_LOST, lpb.EXIT_FAILED),
    ('controller_lost', lpb.HELD_ERROR, lpb.EXIT_FAILED),
    ('rejected', lpb.REJECTED, lpb.EXIT_FAILED),
])
def test_mock_scenarios(scenario, state, code, sources, tmp_path):
    assert cli(sources, tmp_path, '--mock', '--scenario', scenario) == code
    data = json.loads((tmp_path / 'mock' / scenario / 'mock_report.json').read_text())
    assert data['state'] == state and data['mode'] == 'mock'
    assert data['mock_server_goals_received'] <= 1 and data['mock_server_cancels_received'] <= 1
    assert data['limits']['live_dispatch_enabled'] is m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED


def test_mock_requires_the_typed_word(sources, tmp_path):
    assert cli(sources, tmp_path, '--mock', reader=lambda: 'yes\n') == lpb.EXIT_REFUSED
    data = json.loads((tmp_path / 'mock' / 'success' / 'mock_report.json').read_text())
    assert data['reason'] == 'confirmation_refused' and data['mock_server_goals_received'] == 0


def test_mock_is_deterministic(sources, tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    assert cli(sources, a, '--mock') == lpb.EXIT_OK
    assert cli(sources, b, '--mock') == lpb.EXIT_OK
    assert filecmp.cmp(a / 'mock' / 'success' / 'mock_report.json',
                       b / 'mock' / 'success' / 'mock_report.json', shallow=False)


def test_unknown_scenario_is_refused(sources, tmp_path):
    assert cli(sources, tmp_path, '--mock', '--scenario', 'walk') == lpb.EXIT_REFUSED


# ---------------------------------------------------------------- static scan
def _package_sources():
    d = os.path.join(PKG, 'spiderx_controller')
    for name in sorted(os.listdir(d)):
        if name.endswith('.py'):
            with open(os.path.join(d, name)) as f:
                yield name, f.read()


def test_live_gate_is_a_single_false_literal():
    import re
    hits = [(n, line) for n, src in _package_sources() for line in src.splitlines()
            if re.match(r'\s*(\w+\.)?LIVE_DISPATCH_ENABLED\s*=', line)]
    assert hits == [('m6_live_contract.py',
                     f'LIVE_DISPATCH_ENABLED = {m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED}')]


def test_no_environment_or_flag_can_enable_live_dispatch():
    srcs = dict(_package_sources())
    for name in ('m6_live_contract.py', 'm6_live_playback.py', 'm6_goal_fingerprint.py',
                 'm6_live_readiness.py', 'm6_live_adapter.py'):
        for word in ('os.environ', 'getenv', 'LIVE_DISPATCH_ENABLED or', "add_argument('--enable",
                     "add_argument('--yes'", "add_argument('--force'"):
            assert word not in srcs[name], (name, word)


def test_only_the_adapter_can_reach_ros():
    srcs = dict(_package_sources())
    for name in ('m6_live_contract.py', 'm6_live_playback.py', 'm6_goal_fingerprint.py',
                 'm6_live_readiness.py', 'm6_live_mock.py'):
        assert 'import rclpy' not in srcs[name] and 'from rclpy' not in srcs[name], name
    assert 'import rclpy' in srcs['m6_live_adapter.py']
