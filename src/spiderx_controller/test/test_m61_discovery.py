"""colcon test: M6.1 bounded action-server discovery, the complete readiness record and the
read-only readiness rehearsal (fake clocks, the in-memory fake transport, isolated ROS domains).

No Gazebo, launch, controller or bridge. The committed gates stay False; the gated --live wiring
is exercised only with the gate set inside a test (monkeypatch) against the in-memory fake, as in
test_m61_gait_replay. The isolated-domain tests use an explicit domain 150-199 with
ROS_LOCALHOST_ONLY=1 and in-test nodes only, through RehearsalTransport, which cannot send.

Covers: delayed discovery; permanent absence (and how it is classified); the exact bound and
spin overruns; competing commanders observed after the wait; the readiness expiry, stream
freshness and sim-progress checks unchanged; the trajectory identity unchanged; the complete
per-check report in readiness_before/after.json; both gate modes (fakes only).
"""
import ast
import json
import os
import threading
import time

import pytest

from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_gait_replay as gr
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_live_preflight as lpf
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m61_discovery as dsc
from spiderx_controller import m61_evidence as evd
from spiderx_controller import m61_live_contract as c61
from spiderx_controller import m61_mock
from spiderx_controller import m61_readiness_rehearsal as rh
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller import m61a_fixed_base as fb
from spiderx_controller.config_check import load_urdf

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC_CONFIG = os.path.join(PKG, 'config')
TRAJECTORY_ID = '241760e7dfd5ef12'
CONTENT_SHA256 = '94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64'
FINGERPRINT = '9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a'
WORD = c61.CONFIRMATION_WORD + '\n'
HOST = '/leg_trajectory_controller'
SERVER = [(HOST, [env.ACTION_TYPE])]


@pytest.fixture(scope='module')
def plan():
    p = tc.load_plan(SRC_CONFIG, load_urdf())
    assert p.errors == (), p.errors
    return p


@pytest.fixture(scope='module')
def cfg():
    return gr.load_fixed_base_config(SRC_CONFIG)


def word():
    return lambda: WORD


# ==================================================================== the pure wait (fake clock)
class Clock:
    """A fake monotonic clock; spin(s) advances it by s (+ overrun, as a slow live spin)."""

    def __init__(self, overrun=0.0, start=1000.0):
        self.t = self.start = start
        self.overrun = overrun
        self.spins = []

    def now(self):
        return self.t

    def spin(self, s):
        self.spins.append(s)
        self.t += s + self.overrun


def graph(clock, visible_at=None, host=True, endpoints=False, fail_until=None, fail=False):
    """A scripted graph: the server is visible from `visible_at` (elapsed s) on."""
    def snapshot():
        el = clock.t - clock.start
        if fail or (fail_until is not None and el < fail_until):
            raise RuntimeError('NodeNameNonExistentError: node vanished between queries')
        vis = visible_at is not None and el >= visible_at
        return {'servers': list(SERVER) if vis else [],
                'nodes': ['/m61_readiness_rehearsal'] + ([HOST] if host or vis else []),
                'status_publishers': 1 if (vis or endpoints) else 0,
                'client_matched': bool(vis or endpoints)}
    return snapshot


def wait(clock, snapshot, **kw):
    return dsc.wait_for_action_server(snapshot, clock.spin, clock.now, env.ACTION_NAME,
                                      domain_id=7, **kw)


def test_the_bound_is_the_existing_10_s_server_wait():
    assert dsc.TIMEOUT_S == lc.SERVER_WAIT_S == 10.0
    assert dsc.POLL_S == 0.25
    import inspect
    sig = inspect.signature(dsc.wait_for_action_server)
    assert sig.parameters['timeout_s'].default == lc.SERVER_WAIT_S
    assert dsc.expected_host_node(env.ACTION_NAME) == HOST
    assert dsc.expected_host_node('/x') is None


def test_a_visible_server_is_found_without_waiting():
    c = Clock()
    rec = wait(c, graph(c, visible_at=0.0))
    assert rec['outcome'] == dsc.FOUND and rec['found_after_s'] == 0.0
    assert c.spins == [] and len(rec['polls']) == 1 and rec['duration_s'] == 0.0
    assert rec['servers'] == [[HOST, [env.ACTION_TYPE]]] and rec['classification'] is None


@pytest.mark.parametrize('visible_at, found_after', [
    (0.1, 0.25), (3.0, 3.0), (9.75, 9.75), (9.8, 10.0), (10.0, 10.0)])
def test_delayed_discovery_is_found_at_the_first_poll_that_sees_it(visible_at, found_after):
    c = Clock()
    rec = wait(c, graph(c, visible_at=visible_at))
    assert rec['outcome'] == dsc.FOUND and rec['found_after_s'] == found_after
    assert rec['duration_s'] == found_after and sum(c.spins) == found_after
    assert all(p['servers'] == 0 for p in rec['polls'][:-1]) and rec['polls'][-1]['servers'] == 1
    assert all(s <= dsc.POLL_S for s in c.spins)


@pytest.mark.parametrize('visible_at', [10.0001, 10.25, None])
def test_a_server_not_seen_by_the_bound_times_out_at_exactly_the_bound(visible_at):
    c = Clock()
    rec = wait(c, graph(c, visible_at=visible_at))
    assert rec['outcome'] == dsc.TIMEOUT and rec['found_after_s'] is None
    assert rec['duration_s'] == 10.0 and sum(c.spins) == 10.0
    assert rec['polls'][-1]['t_s'] == 10.0 and len(rec['polls']) == 41
    assert rec['servers'] == []


def test_a_spin_overrun_never_accepts_a_server_after_the_bound():
    c = Clock(overrun=0.3)                              # a slow live spin: polls 0.55 s apart
    rec = wait(c, graph(c, visible_at=10.2))
    assert rec['outcome'] == dsc.TIMEOUT
    last = rec['polls'][-1]
    assert last['t_s'] > 10.0 and last['servers'] == 1          # seen, but after the bound
    assert rec['duration_s'] <= 10.0 + dsc.POLL_S + 0.3
    c = Clock(overrun=0.3)
    rec = wait(c, graph(c, visible_at=9.0))
    assert rec['outcome'] == dsc.FOUND and rec['found_after_s'] <= 10.0


@pytest.mark.parametrize('kw, classification', [
    ({'host': True}, dsc.SERVER_ABSENT),
    ({'host': False}, dsc.NOT_DISCOVERED),
    ({'host': False, 'endpoints': True}, dsc.UNATTRIBUTED),
    ({'host': True, 'endpoints': True}, dsc.UNATTRIBUTED),
    ({'fail': True}, dsc.QUERY_FAILED),
])
def test_a_timeout_says_whether_the_server_looks_absent_or_undiscovered(kw, classification):
    c = Clock()
    rec = wait(c, graph(c, **kw))
    assert rec['outcome'] == dsc.TIMEOUT and rec['classification'] == classification
    assert rec['classification_text'] == dsc.CLASSIFICATION_TEXT[classification]
    assert 'TIMEOUT' in dsc.summary_line(rec) and classification in dsc.summary_line(rec)


def test_an_operator_interrupt_ends_the_wait_early():
    c = Clock()
    rec = wait(c, graph(c, visible_at=None), interrupted=lambda: c.t - c.start >= 1.0)
    assert rec['outcome'] == dsc.INTERRUPTED and rec['duration_s'] == 1.0
    assert rec['classification'] is None and 'INTERRUPTED' in dsc.summary_line(rec)
    c = Clock()                                          # a server already visible still counts
    rec = wait(c, graph(c, visible_at=0.0), interrupted=lambda: True)
    assert rec['outcome'] == dsc.FOUND


def test_transient_query_errors_are_recorded_and_polling_continues():
    c = Clock()
    rec = wait(c, graph(c, visible_at=1.0, fail_until=0.6))
    assert rec['outcome'] == dsc.FOUND and rec['found_after_s'] == 1.0
    errors = [p['error'] for p in rec['polls']]
    assert errors[:3] == ['RuntimeError: NodeNameNonExistentError: node vanished between '
                          'queries'] * 3 and errors[-1] is None


def test_the_record_is_complete_and_strict_json():
    c = Clock()
    rec = wait(c, graph(c, visible_at=2.0))
    for key in ('schema', 'action_name', 'expected_host_node', 'domain_id', 'timeout_s',
                'poll_s', 'started_monotonic', 'ended_monotonic', 'duration_s', 'outcome',
                'found_after_s', 'servers', 'classification', 'last_snapshot', 'polls'):
        assert key in rec, key
    assert rec['action_name'] == env.ACTION_NAME and rec['domain_id'] == 7
    assert rec['started_monotonic'] == 1000.0 and rec['ended_monotonic'] == 1002.0
    assert {'t_s', 'monotonic', 'servers', 'nodes', 'host_node_visible', 'status_publishers',
            'client_matched', 'error'} <= set(rec['polls'][0])
    json.dumps(rec, allow_nan=False)


# ==================================================================== the composition (fake)
def fake(plan, cfg, **kw):
    return m61_mock.M61FakeTransport(tc.build_trajectory(plan), FINGERPRINT, fixed_base=cfg, **kw)


def provider(t, plan, cfg, base=None):
    names = plan.sources.joint_names
    base = base or (lambda: rd.assess(m61_mock.readiness_report(t, names, plan.neutral),
                                      t.wall_now()))
    return gr.compose_readiness(base, t, plan, cfg)


def test_the_observation_starts_after_the_wait_so_its_expiry_is_unchanged(plan, cfg):
    t = fake(plan, cfg, action_server_at=9.75)
    res = provider(t, plan, cfg)()
    disc = res.report['m61_action_discovery']
    assert disc['outcome'] == dsc.FOUND and disc['found_after_s'] == 9.75
    assert disc['waited_before_this_observation'] is True
    assert res.ready and res.observed_monotonic >= disc['ended_monotonic']
    start = res.observed_monotonic
    assert rd.dispatch_permitted(res, start + lc.READINESS_MAX_AGE_S) == (True, None)
    assert rd.dispatch_permitted(res, start + lc.READINESS_MAX_AGE_S + 0.01) == \
        (False, 'readiness_stale')


def test_later_observations_observe_afresh_without_waiting_again(plan, cfg):
    t = fake(plan, cfg, action_server_at=3.0)
    provide = provider(t, plan, cfg)
    first = provide()
    n, spun = t.discovery_snapshots, t.spun_s
    t.t += 1.0
    second = provide()
    assert (t.discovery_snapshots, t.spun_s) == (n, spun) == (13, 3.0)
    assert second.report['m61_action_discovery']['waited_before_this_observation'] is False
    assert second.observed_monotonic == first.observed_monotonic + 1.0      # a new observation
    assert t.owner_snapshots == 2


@pytest.mark.parametrize('kw, classification', [
    ({'action_server_at': None}, dsc.SERVER_ABSENT),
    ({'action_server_at': None, 'action_host_visible': False}, dsc.NOT_DISCOVERED),
    ({'action_endpoints_only': True}, dsc.UNATTRIBUTED),
    ({'action_server_at': 10.1}, dsc.SERVER_ABSENT),
])
def test_a_missing_server_still_fails_after_the_bounded_wait(plan, cfg, kw, classification):
    t = fake(plan, cfg, **kw)
    res = provider(t, plan, cfg)()
    assert not res.ready and res.label == rd.INCOMPATIBLE
    assert 'action_server_missing' in res.failure_codes
    assert dsc.DISCOVERY_TIMEOUT in res.failure_codes
    assert res.report['m61_action_discovery']['classification'] == classification
    assert t.spun_s == 10.0
    assert rd.dispatch_permitted(res, t.wall_now()) == (False, 'stack_incompatible')


def test_the_bound_holds_even_if_the_fresh_observation_then_sees_a_server(plan, cfg):
    """A server first seen after the bound is not accepted by the observation that follows."""
    t = fake(plan, cfg, action_server_at=None)
    report = m61_mock.m6mock.mock_readiness_report(plan.sources.joint_names, plan.neutral)
    res = provider(t, plan, cfg, base=lambda: rd.assess(report, t.wall_now()))()
    assert res.failure_codes == (dsc.DISCOVERY_TIMEOUT,) and not res.ready
    assert rd.dispatch_permitted(res, t.wall_now()) == (False, 'readiness_not_ready')


@pytest.mark.parametrize('kw, code', [
    ({'action_server_at': 3.0, 'foreign_client_at': 1.0}, fb.COMMAND_ACTION_CLIENTS),
    ({'action_server_at': 3.0, 'foreign_clients': 1}, fb.COMMAND_ACTION_CLIENTS),
    ({'action_server_at': 3.0, 'command_publishers': 1}, fb.COMMAND_PUBLISHERS),
])
def test_competing_commanders_are_observed_after_the_wait(plan, cfg, kw, code):
    t = fake(plan, cfg, **kw)
    res = provider(t, plan, cfg)()
    assert res.report['m61_action_discovery']['outcome'] == dsc.FOUND
    assert not res.ready and code in res.failure_codes
    assert t.owner_snapshots == 1                       # one snapshot, taken after the wait
    snap = res.report['m61a_command_owner']['snapshot']
    assert snap['foreign_action_clients'] + snap['command_publishers'] >= 1


@pytest.mark.parametrize('kw, code', [
    ({'stamp_freeze_at': -10.0}, fb.POSE_SIM_NOT_ADVANCING),
    ({'no_pose': True}, 'body_pose_missing'),
    ({'description': 'free_base'}, None),
])
def test_pose_plant_and_sim_progress_still_apply_after_a_late_server(plan, cfg, kw, code):
    t = fake(plan, cfg, action_server_at=3.0, **kw)
    res = provider(t, plan, cfg)()
    assert res.report['m61_action_discovery']['found_after_s'] == 3.0
    assert not res.ready
    if code is not None:
        assert code in res.failure_codes, res.failure_codes


def test_the_record_layer_stamps_context_and_monotonic_times(plan, cfg):
    t = fake(plan, cfg, action_server_at=1.0)
    t.domain_id = 42
    res = provider(t, plan, cfg)()
    rec = res.report['m61_readiness_record']
    assert rec['domain_id'] == 42 and rec['action_name'] == env.ACTION_NAME
    assert rec['requested_monotonic'] <= res.report['m61_action_discovery']['started_monotonic']
    assert rec['observation_started_monotonic'] >= \
        res.report['m61_action_discovery']['ended_monotonic']
    assert rec['completed_monotonic'] >= rec['observation_started_monotonic']
    for layer in ('checks', 'failures', 'm61_body_pose', 'm61a_fixed_base', 'm61a_sim_progress',
                  'm61a_command_owner', 'm61_action_discovery', 'm61_readiness_record'):
        assert layer in res.report, layer


# ==================================================================== session and evidence (mock)
def test_a_late_server_run_keeps_identity_and_records_discovery(plan):
    out, session, traj = gr.run_mock(plan, 'late_action_server', word())
    assert out['state'] == lpb.SUCCEEDED and out['goals_sent'] == 1
    assert (out['trajectory_id'], out['trajectory_content_sha256'], out['goal_fingerprint']) == \
        (TRAJECTORY_ID, CONTENT_SHA256, FINGERPRINT)
    before, after = (r['result'] for r in out['readiness'])
    assert before['report']['m61_action_discovery']['found_after_s'] == 3.0
    assert after['report']['m61_action_discovery']['waited_before_this_observation'] is False
    assert before['observed_monotonic'] <= before['evaluated_monotonic']
    assert before['age_s'] < 1.0 and out['freshness_at_send']['permitted'] is True
    assert out['freshness_at_send']['streams_now']['ok'] is True


def test_a_late_server_does_not_mask_streams_stopping_before_the_send(plan):
    """Fakes only: discovery takes 3 s, readiness passes, then the joint states stop during the
    2 s server wait: the pre-send check still refuses (plan E5)."""
    m61_mock.SCENARIOS['_late_then_js_stop'] = dict(action_server_at=3.0, server_wait_s=2.0,
                                                    js_stop_at=4.0)
    try:
        out, session, _ = gr.run_mock(plan, '_late_then_js_stop', word())
    finally:
        del m61_mock.SCENARIOS['_late_then_js_stop']
    assert out['state'] == lpb.REFUSED and out['reason'] == fb.JOINT_STATES_STALE
    assert out['goals_sent'] == 0 and session.transport.inner.sent == []


@pytest.mark.parametrize('scenario, classification', [
    ('action_server_absent', dsc.SERVER_ABSENT),
    ('action_server_not_discovered', dsc.NOT_DISCOVERED),
    ('action_server_unattributed', dsc.UNATTRIBUTED),
    ('action_server_after_bound', dsc.SERVER_ABSENT),
])
def test_a_missing_server_is_refused_with_zero_goals_and_a_complete_report(plan, tmp_path,
                                                                           scenario,
                                                                           classification):
    out, session, traj = gr.run_mock(plan, scenario, word())
    assert out['state'] == lpb.REFUSED and out['reason'] == 'stack_incompatible'
    assert out['goals_sent'] == 0 and out['mock_server_goals_received'] == 0
    run_dir = gr._write_run(str(tmp_path), scenario, evd.MOCK_OUTCOME, out, session, traj)
    before = json.loads(open(os.path.join(run_dir, 'readiness_before.json')).read())
    res = before['result']
    assert {'action_server_missing', dsc.DISCOVERY_TIMEOUT} <= set(res['failure_codes'])
    rep = res['report']
    assert rep['checks']['action_server'] == 'failed'
    assert {f['code'] for f in rep['failures']} >= {'action_server_missing'}
    assert rep['m61_action_discovery']['classification'] == classification
    assert rep['m61_action_discovery']['duration_s'] == 10.0
    assert json.loads(open(os.path.join(run_dir, 'readiness_after.json')).read()) is None


def test_an_interrupt_during_the_wait_refuses_with_zero_goals(plan):
    m61_mock.SCENARIOS['_interrupt_in_discovery'] = dict(action_server_at=None,
                                                         interrupts_at=(2.0,))
    try:
        out, session, _ = gr.run_mock(plan, '_interrupt_in_discovery', word())
    finally:
        del m61_mock.SCENARIOS['_interrupt_in_discovery']
    assert out['state'] == lpb.REFUSED and out['goals_sent'] == 0
    assert session.transport.inner.sent == []
    res = out['readiness'][0]['result']
    assert dsc.DISCOVERY_INTERRUPTED in res['failure_codes']
    disc = res['report']['m61_action_discovery']
    assert disc['outcome'] == dsc.INTERRUPTED and disc['duration_s'] == 2.0


# ==================================================================== gate modes (fakes only)
class LateFactories:
    """--live factories over the fake: the probe observation sees the server only when the fake
    graph shows it (as the live per-node query would)."""

    def __init__(self, plan, **kw):
        self.plan, self.kw, self.made = plan, kw, []

    def transport(self, fp, domain_id, fixed_base):
        t = m61_mock.M61FakeTransport(tc.build_trajectory(self.plan), fp, fixed_base=fixed_base,
                                      **self.kw)
        t.domain_id = domain_id
        self.made.append(t)
        return t

    def collect(self, transport):
        names, neutral = self.plan.sources.joint_names, self.plan.neutral

        def observe():
            obs = lpf.Observations()
            rep = m61_mock.readiness_report(transport, names, neutral)['observed']
            obs.action_servers = rep['action_servers'] or []
            obs.controllers = rep['controllers']
            obs.joint_state_publishers = rep['joint_state_publishers']
            obs.joint_state_messages = [(1.0 + 0.01 * i, list(names), list(neutral))
                                        for i in range(4)]
            return obs
        return observe


@pytest.fixture
def enabled(monkeypatch):
    monkeypatch.setattr(c61, 'M61_LIVE_DISPATCH_ENABLED', True)       # tests only


def live(plan, tmp_path, f, utc):
    args = gr.parse_args(['--live', '--domain-id', '42', '--out', str(tmp_path)])
    return gr._live_main(args, plan, word(), transport_factory=f.transport,
                         collect_factory=f.collect, utc_stamp=utc)


def test_disabled_gate_refuses_live_before_any_discovery(plan, tmp_path, monkeypatch):
    """Mode-explicit: the gate is set False here (tests only), whatever this build carries;
    the build's own value is pinned by test/m61_gate.py."""
    monkeypatch.setattr(c61, 'M61_LIVE_DISPATCH_ENABLED', False)
    f = LateFactories(plan, action_server_at=3.0)
    rc = gr.main(['m6_gait_replay', '--live', '--domain-id', '42', '--out', str(tmp_path)],
                 plan=plan, reader=word())
    assert rc == gr.EXIT_DISABLED and f.made == [] and os.listdir(tmp_path) == []
    with pytest.raises(PermissionError):
        gr._run_live(plan, f.transport, f.collect, word(), lpb.InterruptLatch(), 42)
    assert f.made == []


def test_enabled_live_with_a_late_server_sends_once_and_records_discovery(enabled, plan,
                                                                          tmp_path):
    f = LateFactories(plan, action_server_at=3.0)
    assert live(plan, tmp_path, f, '20261012T100000Z') == gr.EXIT_OK
    d = tmp_path / '20261012T100000Z'
    out = json.loads((d / evd.LIVE_OUTCOME).read_text())
    assert out['goals_sent'] == 1 and len(f.made[0].sent) == 1
    assert out['goal_fingerprint'] == FINGERPRINT
    before = json.loads((d / 'readiness_before.json').read_text())['result']
    rec = before['report']['m61_readiness_record']
    assert rec['domain_id'] == 42 and rec['action_name'] == env.ACTION_NAME
    assert before['report']['m61_action_discovery']['found_after_s'] == 3.0
    assert before['report']['checks']['action_server'] == 'passed'


def test_enabled_live_with_an_absent_server_is_refused_with_zero_goals(enabled, plan, tmp_path):
    f = LateFactories(plan, action_server_at=None)
    assert live(plan, tmp_path, f, '20261012T100001Z') == gr.EXIT_REFUSED
    d = tmp_path / '20261012T100001Z'
    out = json.loads((d / evd.LIVE_OUTCOME).read_text())
    assert out['goals_sent'] == 0 and f.made[0].sent == [] and out['reason'] == \
        'stack_incompatible'
    before = json.loads((d / 'readiness_before.json').read_text())['result']
    assert before['report']['m61_action_discovery']['classification'] == dsc.SERVER_ABSENT
    assert set(before['failure_codes']) >= {'action_server_missing', dsc.DISCOVERY_TIMEOUT}


# ==================================================================== read-only rehearsal
REHEARSAL_SRC = os.path.join(PKG, 'spiderx_controller', 'm61_readiness_rehearsal.py')
FORBIDDEN_CALLS = {'send_goal', 'send_goal_async', 'cancel_goal', 'cancel_goal_async',
                   '_dispatch', 'run', 'parse_confirmation', 'interruptible_line_reader',
                   'build_live_goal', '_run_live', '_live_main', 'publish'}
FORBIDDEN_NAMES = {'M61Session', 'LiveSession', 'build_live_goal', '_run_live', '_live_main',
                   'M61_LIVE_DISPATCH_ENABLED_MESSAGE', 'create_publisher'}


def test_the_rehearsal_module_contains_no_dispatch_path():
    tree = ast.parse(open(REHEARSAL_SRC).read())
    calls = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            calls.add(fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, 'id', None))
        if isinstance(node, (ast.Name, ast.Attribute)):
            name = node.id if isinstance(node, ast.Name) else node.attr
            assert name not in FORBIDDEN_NAMES, name
        if isinstance(node, ast.Assign):                  # nothing assigns a gate
            for tgt in node.targets:
                assert 'DISPATCH_ENABLED' not in ast.dump(tgt)
    assert not calls & FORBIDDEN_CALLS, calls & FORBIDDEN_CALLS


def test_the_rehearsal_transport_cannot_send_or_cancel(cfg):
    t = rh.RehearsalTransport(FINGERPRINT, 151, cfg)
    assert t.node_name == rh.NODE_NAME
    with pytest.raises(PermissionError):
        t.send_goal(object(), object())
    with pytest.raises(PermissionError):
        t.cancel_goal()
    assert t.dispatch_calls == 2 and t.client is None


def rehearse(plan, cfg, **kw):
    made = []

    def factory(fp, domain_id, fixed_base):
        t = fake(plan, cfg, **kw)
        t.domain_id = domain_id
        made.append(t)
        return t
    rec = rh.run_rehearsal(plan, 0, transport_factory=factory,
                           collect_factory=LateFactories(plan).collect)
    return rec, made[0]


def test_a_rehearsal_with_a_late_server_is_ready_and_sends_nothing(plan, cfg):
    rec, t = rehearse(plan, cfg, action_server_at=3.0)
    assert rec['verdict'] == 'READY' and [s['when'] for s in rec['steps']] == \
        ['before_confirmation', 'after_confirmation']
    assert rec['discovery']['found_after_s'] == 3.0
    assert rec['streams_at_send']['ok'] and rec['age_check_at_send']['permitted']
    assert t.sent == [] and t.cancels == 0 and rec['goals_sent'] == 0
    assert rec['dispatch_possible'] is False and rec['transport_closed'] is True
    assert (rec['trajectory_id'], rec['goal_fingerprint']) == (TRAJECTORY_ID, FINGERPRINT)
    json.dumps(evd.json_safe(rec), allow_nan=False)


@pytest.mark.parametrize('kw, stopped', [
    ({'action_server_at': None}, 'before_confirmation'),
    ({'action_server_at': 1.0, 'foreign_clients': 1}, 'before_confirmation'),
    ({'action_server_at': 1.0, 'competing_after_calls': 1}, 'after_confirmation'),
    ({'action_server_at': 1.0, 'js_stop_at': 0.0}, 'at_send'),
])
def test_a_rehearsal_stops_where_live_would_refuse(plan, cfg, kw, stopped):
    rec, t = rehearse(plan, cfg, **kw)
    assert rec['verdict'] == 'NOT READY' and rec['stopped_at'] == stopped
    assert t.sent == [] and t.cancels == 0


@pytest.mark.parametrize('gate', [False, True])
def test_the_rehearsal_is_identical_in_both_gate_modes(plan, cfg, tmp_path, monkeypatch, gate):
    monkeypatch.setattr(c61, 'M61_LIVE_DISPATCH_ENABLED', gate)        # tests only
    made = []

    def factory(fp, domain_id, fixed_base):
        made.append(fake(plan, cfg, action_server_at=2.0))
        return made[-1]
    rc = rh.main(['m61_readiness_rehearsal', '--domain-id', '0', '--out', str(tmp_path)],
                 plan=plan, transport_factory=factory,
                 collect_factory=LateFactories(plan).collect, utc='20261012T110000Z')
    assert rc == rh.EXIT_READY and made[0].sent == [] and made[0].cancels == 0
    rec = json.loads((tmp_path / '20261012T110000Z' / 'rehearsal.json').read_text())
    assert rec['gates']['m61_live_dispatch_enabled'] is gate and rec['goals_sent'] == 0
    assert rec['discovery']['found_after_s'] == 2.0
    # evidence is never overwritten
    assert rh.main(['m61_readiness_rehearsal', '--domain-id', '0', '--out', str(tmp_path)],
                   plan=plan, transport_factory=factory, utc='20261012T110000Z') == \
        rh.EXIT_REFUSED
    assert len(made) == 1


def test_rehearsal_arguments_are_checked(plan, tmp_path):
    assert rh.main(['m61_readiness_rehearsal', '--out', str(tmp_path)], plan=plan) == \
        rh.EXIT_REFUSED
    assert rh.main(['m61_readiness_rehearsal', '--domain-id', '999', '--out', str(tmp_path)],
                   plan=plan) == rh.EXIT_REFUSED
    assert os.listdir(tmp_path) == []


# ==================================================================== isolated ROS domain
DOMAIN = 150 + (os.getpid() + 31) % 50


@pytest.fixture
def isolated(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    assert 150 <= DOMAIN < 200 and DOMAIN != int(os.environ.get('ROS_DOMAIN_ID', '0') or 0)


class InTestNode:
    """A node named like the controller in its own context, optionally with the action server
    (no goal is ever sent to it; it records any it receives)."""

    def __init__(self, with_action):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionServer
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions
        self.rclpy, self.received = rclpy, []
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=DOMAIN,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('leg_trajectory_controller', context=self.ctx,
                                      enable_rosout=False, start_parameter_services=False)
        self.server = None
        if with_action:
            self.server = ActionServer(self.node, FollowJointTrajectory, env.ACTION_NAME,
                                       execute_callback=self._execute)

    def _execute(self, handle):                       # never expected to run
        self.received.append(handle)
        handle.abort()
        from control_msgs.action import FollowJointTrajectory
        return FollowJointTrajectory.Result()

    def stop(self):
        if self.server is not None:
            self.server.destroy()
        self.node.destroy_node()
        self.rclpy.try_shutdown(context=self.ctx)


def open_isolated(cfg):
    t = rh.RehearsalTransport(FINGERPRINT, DOMAIN, cfg, use_sim_time=False).open()
    t.spin_for(1.0)
    nodes = t.node.get_node_names_and_namespaces()
    assert nodes == [(rh.NODE_NAME, '/')], f'ROS domain {DOMAIN} is not isolated: {nodes}'
    return t


def later(delay_s, fn, box):
    timer = threading.Timer(delay_s, lambda: box.append(fn()))
    timer.start()
    return timer


def test_isolated_a_server_that_appears_late_is_found(isolated, cfg):
    t = open_isolated(cfg)
    box = []
    timer = later(2.5, lambda: InTestNode(with_action=True), box)
    try:
        rec = dsc.wait_for_action_server(t.action_discovery_snapshot, t.spin_for, t.wall_now,
                                         env.ACTION_NAME, domain_id=DOMAIN)
        assert rec['outcome'] == dsc.FOUND, rec['polls'][-3:]
        assert 2.0 <= rec['found_after_s'] <= 10.0
        assert rec['servers'] == [[HOST, [env.ACTION_TYPE]]]
        assert rec['polls'][0]['servers'] == 0
        assert t.dispatch_calls == 0 and box[0].received == []
    finally:
        timer.join()
        for n in box:
            n.stop()
        t.close()


@pytest.mark.parametrize('host, classification', [
    (True, dsc.SERVER_ABSENT), (False, dsc.NOT_DISCOVERED)])
def test_isolated_a_permanently_absent_server_times_out_and_is_classified(isolated, cfg, host,
                                                                          classification):
    t = open_isolated(cfg)
    node = InTestNode(with_action=False) if host else None
    try:
        t0 = time.monotonic()
        rec = dsc.wait_for_action_server(t.action_discovery_snapshot, t.spin_for, t.wall_now,
                                         env.ACTION_NAME, domain_id=DOMAIN)
        took = time.monotonic() - t0
        assert rec['outcome'] == dsc.TIMEOUT and rec['classification'] == classification
        assert 10.0 <= rec['duration_s'] <= 10.0 + dsc.POLL_S + 0.5 and took < 11.5
        assert rec['polls'][-1]['host_node_visible'] is host
        assert rec['last_snapshot']['status_publishers'] == 0
        assert rec['last_snapshot']['client_matched'] is False
    finally:
        if node is not None:
            node.stop()
        t.close()


def test_isolated_production_readiness_finds_a_late_stack_and_observes_afresh(isolated, plan,
                                                                              cfg):
    """The --live readiness composition (live_readiness_provider) on a real rclpy graph: the
    test-double controller stack (list_controllers, the action server, /clock, /joint_states)
    starts 2.5 s after the transport. Discovery waits for it; the observation that follows sees
    the server and the controllers itself. No pose bridge runs, so the M6.1 layers refuse - this
    test is about discovery and freshness, and nothing can be sent."""
    import m6d_isolated_stack as stack_mod
    t = open_isolated(cfg)
    box = []
    timer = later(2.5, lambda: stack_mod.IsolatedFakeStack(DOMAIN, plan.sources.joint_names,
                                                           plan.neutral), box)
    try:
        res = gr.live_readiness_provider(t, plan, cfg, gr._default_collect_factory)()
        rep = res.report
        disc = rep['m61_action_discovery']
        assert disc['outcome'] == dsc.FOUND and 2.0 <= disc['found_after_s'] <= 10.0
        assert rep['checks']['action_server'] == 'passed'
        assert rep['checks']['controllers'] == 'passed'
        assert 'action_server_missing' not in res.failure_codes
        assert dsc.DISCOVERY_TIMEOUT not in res.failure_codes
        assert res.observed_monotonic >= disc['ended_monotonic']
        assert rep['m61_readiness_record']['domain_id'] == DOMAIN
        assert not res.ready                              # no body pose: the M6.1 layers refuse
        json.dumps(evd.json_safe(rep), allow_nan=False)
        assert t.dispatch_calls == 0 and box[0].received == [] and box[0].cancel_requests == 0
    finally:
        timer.join()
        for s in box:
            s.stop()
        t.close()


def test_isolated_production_readiness_still_refuses_a_missing_server(isolated, plan, cfg):
    t = open_isolated(cfg)
    try:
        res = gr.live_readiness_provider(t, plan, cfg, gr._default_collect_factory)()
        disc = res.report['m61_action_discovery']
        assert disc['outcome'] == dsc.TIMEOUT and disc['classification'] == dsc.NOT_DISCOVERED
        assert {'action_server_missing', dsc.DISCOVERY_TIMEOUT} <= set(res.failure_codes)
        assert res.label == rd.INCOMPATIBLE and not res.ready
        assert res.observed_monotonic >= disc['ended_monotonic']
        assert t.dispatch_calls == 0
    finally:
        t.close()
