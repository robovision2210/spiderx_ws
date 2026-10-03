"""colcon test: M6.0-D Batch C - RclpyLiveTransport against an in-process ACTION test double.

Isolation (owner decision D12): every ROS context here uses an explicit, non-default ROS domain
(150-199, never the process's ROS_DOMAIN_ID) with ROS_LOCALHOST_ONLY=1; the tests first prove that
no other node exists on that domain. No Gazebo, launch, controller, bridge or external process is
used, and nothing publishes /joint_states or any command topic. Each test shuts its contexts down.
"""
import copy
import os
import signal
import time

import m6d_isolated_action_server as fake
import pytest

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_adapter as la
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
DOMAIN = 150 + os.getpid() % 50


@pytest.fixture(scope='module', autouse=True)
def isolated_env():
    old = {k: os.environ.get(k) for k in ('ROS_LOCALHOST_ONLY',)}
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    yield
    for k, v in old.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


@pytest.fixture(scope='module')
def approved(sources):
    traj = m6t.build_trajectory(sources)
    goal, _, spec, fp = gf.build_live_goal(traj, sources)
    return traj, goal, fp


def open_transport(fp):
    return la.RclpyLiveTransport(fp, DOMAIN, node_name='m6d_test_client',
                                 use_sim_time=False).open()


def goal_responses(events):
    """[(accepted, goal_id)]: the adapter reports the client-generated goal UUID as 32 hex."""
    out = [(e[1], e[2]) for e in events if e[0] == 'goal_response']
    assert all(isinstance(g, str) and len(g) == 32 and int(g, 16) >= 0 for _, g in out), out
    return out


def wait_events(transport, kinds, timeout=10.0):
    got = []
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        got += transport.poll(0.05)
        if all(any(e[0] == k for e in got) for k in kinds):
            break
    return got


# ---------------------------------------------------------------- isolation
def test_domain_is_explicit_and_not_default():
    assert DOMAIN != 0 and 150 <= DOMAIN < 200
    assert DOMAIN != int(os.environ.get('ROS_DOMAIN_ID', '0') or 0)
    with pytest.raises(la.TransportError):
        la.RclpyLiveTransport('x', None)


def test_no_external_node_or_endpoint_is_visible(approved):
    t = open_transport(approved[2])
    try:
        assert t.context.get_domain_id() == DOMAIN
        t.poll(1.5)                                           # let discovery settle
        nodes = t.node.get_node_names_and_namespaces()
        assert nodes == [('m6d_test_client', '/')], nodes
        from rclpy.action import get_action_server_names_and_types_by_node
        assert all(get_action_server_names_and_types_by_node(t.node, n, ns) == []
                   for n, ns in nodes)                       # no action server anywhere
        assert t.node.count_publishers('/joint_states') == 0
        assert not t.server_ready(0.5)                        # nothing to talk to
    finally:
        t.close()


def test_rclpy_signal_handlers_are_disabled(approved):
    from rclpy.signals import get_current_signal_handlers_options, SignalHandlerOptions
    t = open_transport(approved[2])
    try:
        assert get_current_signal_handlers_options() == SignalHandlerOptions.NO
    finally:
        t.close()


# ---------------------------------------------------------------- goal over the wire
def test_one_approved_goal_reaches_the_server_with_every_field(approved):
    traj, goal, fp = approved
    srv = fake.IsolatedFakeServer(DOMAIN)
    t = open_transport(fp)
    try:
        assert t.server_ready(10.0)
        t.send_goal(goal, gf.binding(traj))
        ev = wait_events(t, ['goal_response', 'result'])
        assert [a for a, _ in goal_responses(ev)] == [True]
        res = [e for e in ev if e[0] == 'result'][0]
        assert res[1] == ac.STATUS_SUCCEEDED and res[2] == 0
        assert any(e[0] == 'feedback' for e in ev)
        assert len(srv.received) == 1
        got = srv.received[0]
        # every field survived real serialization
        assert gf.verify_goal(got, gf.binding(traj), fp) == fp
        assert (got.trajectory.header.stamp.sec, got.trajectory.header.stamp.nanosec) == (0, 0)
        assert all(x.velocity == 0.05 and x.position == 0.05 for x in got.goal_tolerance)
        assert all(x.position == 0.05 and x.velocity == 0.0 for x in got.path_tolerance)
        assert (got.goal_time_tolerance.sec, got.goal_time_tolerance.nanosec) == (1, 0)
        with pytest.raises(ac.SecondGoalForbidden):
            t.send_goal(goal, gf.binding(traj))
        assert len(srv.received) == 1
    finally:
        t.close()
        srv.stop()


def test_non_approved_goal_never_reaches_the_server(approved):
    traj, goal, fp = approved
    srv = fake.IsolatedFakeServer(DOMAIN)
    t = open_transport(fp)
    try:
        assert t.server_ready(10.0)
        bad = copy.deepcopy(goal)
        del bad.trajectory.points[1:]                         # a "return to neutral" goal
        with pytest.raises(gf.FingerprintError):
            t.send_goal(bad, gf.binding(traj))
        t.poll(0.5)
        assert srv.received == []
    finally:
        t.close()
        srv.stop()


def test_reject_and_abort_map_to_events(approved):
    traj, goal, fp = approved
    for mode, code, expect in (('reject', 0, None), ('abort', -4, ac.STATUS_ABORTED)):
        srv = fake.IsolatedFakeServer(DOMAIN, mode=mode, error_code=code)
        t = open_transport(fp)
        try:
            assert t.server_ready(10.0)
            t.send_goal(goal, gf.binding(traj))
            ev = wait_events(t, ['goal_response'] + (['result'] if expect else []))
            if expect is None:
                assert [a for a, _ in goal_responses(ev)] == [False]
            else:
                res = [e for e in ev if e[0] == 'result'][0]
                assert (res[1], res[2]) == (expect, -4)
        finally:
            t.close()
            srv.stop()


def test_cancel_only_once_and_goal_ends_canceled(approved):
    traj, goal, fp = approved
    srv = fake.IsolatedFakeServer(DOMAIN, mode='wait')
    t = open_transport(fp)
    try:
        assert t.server_ready(10.0)
        t.send_goal(goal, gf.binding(traj))
        assert [a for a, _ in goal_responses(wait_events(t, ['goal_response']))] == [True]
        t.cancel_goal()
        ev = wait_events(t, ['cancel_response', 'result'])
        assert ('cancel_response', 0) in ev
        assert [e for e in ev if e[0] == 'result'][0][1] == ac.STATUS_CANCELED
        with pytest.raises(ac.SecondGoalForbidden):
            t.cancel_goal()
        assert srv.cancel_requests == 1 and len(srv.received) == 1
    finally:
        t.close()
        srv.stop()


def test_sigint_keeps_the_context_alive_for_exactly_one_cancel(approved):
    traj, goal, fp = approved
    srv = fake.IsolatedFakeServer(DOMAIN, mode='wait')
    t = open_transport(fp)
    latch = lpb.InterruptLatch()
    latch.install()
    try:
        assert t.server_ready(10.0)
        t.send_goal(goal, gf.binding(traj))
        assert [a for a, _ in goal_responses(wait_events(t, ['goal_response']))] == [True]
        os.kill(os.getpid(), signal.SIGINT)                   # an operator Ctrl+C
        end = time.monotonic() + 2.0
        while latch.count == 0 and time.monotonic() < end:
            time.sleep(0.01)
        assert latch.count == 1
        assert t.context.ok()                                 # rclpy did not shut it down
        t.cancel_goal()
        ev = wait_events(t, ['cancel_response', 'result'])
        assert ('cancel_response', 0) in ev
        assert srv.cancel_requests == 1 and len(srv.received) == 1
    finally:
        latch.uninstall()
        t.close()
        srv.stop()


def test_close_shuts_the_context_down(approved):
    t = open_transport(approved[2])
    ctx = t.context
    t.close()
    assert not ctx.ok()
    with pytest.raises(la.TransportError):
        t.poll(0.01)


@pytest.mark.parametrize('fail_at', ['create_node', 'action_client', 'subscription'])
def test_partial_open_releases_everything_it_created(approved, monkeypatch, fail_at):
    """open() fails after rclpy.init: the context and any node are released, error re-raised."""
    import rclpy
    import rclpy.action
    from rclpy.node import Node

    def boom(*a, **k):
        raise RuntimeError(f'injected {fail_at} failure')
    if fail_at == 'create_node':
        monkeypatch.setattr(rclpy, 'create_node', boom)
    elif fail_at == 'action_client':
        monkeypatch.setattr(rclpy.action, 'ActionClient', boom)
    else:
        monkeypatch.setattr(Node, 'create_subscription', boom)
    t = la.RclpyLiveTransport(approved[2], DOMAIN, node_name='m6d_partial', use_sim_time=False)
    with pytest.raises(RuntimeError, match='injected'):
        t.open()
    assert t.context is not None and not t.context.ok()
    assert (t.node, t.executor, t.client, t.sub) == (None, None, None, None)
    with pytest.raises(la.TransportError):
        t.poll(0.01)


def test_adapter_source_is_command_free():
    import inspect
    src = inspect.getsource(la)
    assert src.count('send_goal_async(') == 1 and src.count('cancel_goal_async(') == 1
    for word in ('create_publisher', '.publish(', 'switch_controller', 'set_parameters',
                 'cmd_vel', 'subprocess', 'SignalHandlerOptions.ALL'):
        assert word not in src


# ---------------------------------------------------------------- live-enabling wiring (isolated)
def test_graph_collector_is_read_only_and_sees_only_the_test_double(approved):
    traj, goal, fp = approved
    srv = fake.IsolatedFakeServer(DOMAIN)
    t = open_transport(fp)
    try:
        obs = t.graph_collector(timeout_s=0.5, window_s=0.3, discovery_s=1.0)()
        assert obs.action_servers == [('/m6d_test_double_fjt_server', [
            'control_msgs/action/FollowJointTrajectory'])]
        assert obs.controllers is None                         # no controller manager exists
        assert obs.joint_state_publishers == [] and obs.joint_state_messages == []
        assert t.node.count_publishers('/leg_trajectory_controller/joint_trajectory') == 0
        assert srv.received == [] and srv.cancel_requests == 0
    finally:
        t.close()
        srv.stop()


def test_enabled_live_wiring_refuses_without_a_controller_stack(approved, monkeypatch, tmp_path):
    """Gate set True IN THIS TEST ONLY, real rclpy transport and collector, isolated domain:
    the readiness gate refuses (no controller manager, no /joint_states) and no goal is sent."""
    from spiderx_controller import m6_live_contract as lc
    traj, goal, fp = approved
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', True)
    srv = fake.IsolatedFakeServer(DOMAIN)
    asked = []
    try:
        args = lpb.parse_args(['--live', '--domain-id', str(DOMAIN), '--out', str(tmp_path)])
        rc = lpb._live_main(
            args, None, lambda: asked.append(1) or lc.CONFIRMATION_WORD + '\n',
            collect_factory=lambda tr: tr.graph_collector(timeout_s=0.5, window_s=0.3,
                                                          discovery_s=1.0),
            utc_stamp='isolated')
        assert rc == lpb.EXIT_REFUSED
        assert asked == []                                    # refused before the confirmation
        assert srv.received == [] and srv.cancel_requests == 0
        import json
        data = json.loads((tmp_path / 'live' / 'isolated' / 'live_outcome.json').read_text())
        # no /joint_states on the isolated domain: the joint contract is unobserved, which the
        # D4 classification treats as incompatible (refused before any confirmation)
        assert data['state'] == lpb.REFUSED and data['reason'] == 'stack_incompatible'
        assert 'joint contract not observed' in data['readiness'][0]['result']['reasons']
        codes = data['readiness'][0]['result']['failure_codes']
        assert 'controller_manager_unavailable' in codes and 'joint_states_no_publisher' in codes
        assert data['dispatch']['attempted'] is False and data['transport']['closed']
    finally:
        srv.stop()
