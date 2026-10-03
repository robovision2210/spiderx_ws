"""colcon test: PR #16 corrective batch 3 - the gate-enabled live CLI, end to end, on an ISOLATED
ROS domain against an in-process fake controller stack (tests only).

Isolation (owner decision D12): an explicit non-default domain (150-199, never the process's
ROS_DOMAIN_ID) with ROS_LOCALHOST_ONLY=1; before anything is started, a probe proves that no
other node, service, publisher or action server exists on it (otherwise the test is SKIPPED with
the reason - a real graph is never used to make it pass). No Gazebo, launch, controller manager,
controller, bridge or external process. The gate is set True only inside this test
(monkeypatch); the committed gate stays False.

What runs for real: main() and its argument handling, the evidence file, the default
interruptible confirmation reader (stdin is a pipe holding the exact word), RclpyLiveTransport
(own context, rclpy signal handlers off), the read-only readiness collector with its
list_controllers call completed on the transport's own executor, both readiness checks, the
send-time freshness check, the fingerprint check, one send_goal, feedback, the result, the
in-flight and final tracking against /joint_states stamped with /clock simulation time, and
teardown. What is faked: the controller stack (m6d_isolated_stack) - it proves nothing about a
real controller.
"""
import json
import os
import time

import m6d_isolated_stack as fake_stack
import pytest

from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
DOMAIN = 150 + os.getpid() % 50
TRAJECTORY_ID = '44f0a7ad52e5c330'
FINGERPRINT = '0d6ef4171f2d338a01e76b934be94d1a4386e7c0fc68fe74262fb3c65005ff83'


@pytest.fixture(scope='module', autouse=True)
def isolated_env():
    old = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    yield
    if old is None:
        os.environ.pop('ROS_LOCALHOST_ONLY', None)
    else:
        os.environ['ROS_LOCALHOST_ONLY'] = old


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


def graph_snapshot(domain, settle_s=1.5):
    """Everything visible on `domain` except a short-lived probe (no rosout, no param services)."""
    import rclpy
    from rclpy.action import get_action_server_names_and_types_by_node
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    node = executor = None
    try:
        assert ctx.get_domain_id() == domain
        node = rclpy.create_node('m6d_isolation_probe', context=ctx, enable_rosout=False,
                                 start_parameter_services=False)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        end = time.monotonic() + settle_s
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
        others = [n for n in node.get_node_names_and_namespaces()
                  if n != ('m6d_isolation_probe', '/')]
        return {
            'nodes': others,
            'services': node.get_service_names_and_types(),
            'publishers': [t for t, _ in node.get_topic_names_and_types()
                           if any(i.node_name != 'm6d_isolation_probe'
                                  for i in node.get_publishers_info_by_topic(t))],
            'action_servers': [a for n, ns in others
                               for a in get_action_server_names_and_types_by_node(node, n, ns)],
        }
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown(context=ctx)


def piped_stdin(monkeypatch, text):
    r, w = os.pipe()
    os.write(w, text.encode())
    os.close(w)
    monkeypatch.setattr('sys.stdin', os.fdopen(r, 'r'))


def test_isolation_preconditions():
    assert 150 <= DOMAIN < 200
    assert DOMAIN != int(os.environ.get('ROS_DOMAIN_ID', '0') or 0)
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    assert lc.LIVE_DISPATCH_ENABLED is False             # committed gate, outside the test patch


def test_gate_enabled_main_succeeds_end_to_end_with_one_goal(sources, monkeypatch, tmp_path):
    before = graph_snapshot(DOMAIN)
    if any(before.values()):
        pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {before}')
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', True)         # test-local only
    traj = m6t.build_trajectory(sources)
    stack = fake_stack.IsolatedFakeStack(DOMAIN, traj['joint_names'],
                                         traj['points'][0]['positions'])
    try:
        piped_stdin(monkeypatch, lc.CONFIRMATION_WORD + '\n')
        rc = lpb.main(['m6_live_playback', '--live', '--domain-id', str(DOMAIN),
                       '--out', str(tmp_path)], sources=sources)
        received, results = list(stack.received), list(stack.results)
        cancels, list_calls = stack.cancel_requests, stack.list_calls
    finally:
        stack.stop()
    after = graph_snapshot(DOMAIN)

    files = sorted((tmp_path / 'live').glob('*/live_outcome.json'))
    assert len(files) == 1
    data = json.loads(files[0].read_text())
    assert rc == lpb.EXIT_OK, (data['state'], data['reason'], data['errors'])
    assert data['state'] == lpb.SUCCEEDED and data['passed']
    assert data['evidence']['phase'] == 'final' and data['domain_id'] == DOMAIN

    # identity: the one approved goal, and only it, reached the fake server
    assert data['trajectory_id'] == TRAJECTORY_ID and data['goal_fingerprint'] == FINGERPRINT
    assert len(received) == 1 and cancels == 0 and results == ['succeeded']
    assert gf.verify_goal(received[0], gf.binding(traj), FINGERPRINT) == FINGERPRINT
    assert (data['goals_sent'], data['cancels_sent'], data['retries'],
            data['automatic_return_goals']) == (1, 0, 0, 0)

    # readiness ran twice through the real service-completion path, and was fresh at the send
    assert list_calls >= 2
    assert [r['when'] for r in data['readiness']] == ['before_confirmation', 'after_confirmation']
    for r in data['readiness']:
        assert r['result']['ready'] and r['result']['classification'] in ('compatible', 'warning')
        assert r['result']['collection_s'] > 1.0          # discovery + sample window, counted
    fresh = data['freshness_at_send']
    assert fresh['permitted'] and data['readiness'][1]['result']['collection_s'] <= \
        fresh['age_s'] <= lc.READINESS_MAX_AGE_S

    # acceptance, result and tracking evidence
    d = data['dispatch']
    assert d['acceptance'] == 'accepted' and len(d['goal_id']) == 32
    assert d['final_result_known'] and d['final_goal_status'] == 4
    assert not d['goal_may_still_be_executing'] and not d['cancel_attempted']
    assert data['channels'] == {'action': 'passed', 'tracking': 'passed',
                                'goal_tolerances': 'passed'}
    assert data['tracking']['samples'] > 50
    assert data['tracking']['max_inflight_error_rad'] < lc.CLIENT_TRACKING_ABORT_RAD
    assert data['errors'] == []

    # clean teardown: the transport closed itself; nothing is left on the domain
    assert data['transport']['closed'] and data['transport']['close_error'] is None
    assert not any(after.values()), after
