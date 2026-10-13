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

Diagnostics: a pass-through recorder keeps every readiness report in full (each check, each
failure message, the /joint_states sample stamps and their age, the nodes visible on the domain)
and puts it in the assertion message, so a failure explains itself in the xunit/colcon logs even
after pytest has rotated its temporary directories. It returns the real results unchanged.
"""
import json
import os
import threading
import time

import m6d_gate
import m6d_isolated_stack as fake_stack
import pytest

from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_live_preflight as lpf
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


class ReadinessRecorder:
    """Pass-through wrappers around the readiness probe and evaluation (diagnostics only).

    Records, per readiness evaluation: monotonic time, the verdict, every check and failure (code
    and message), the number of /joint_states samples, their first/last stamps, any index where a
    stamp did not increase, the age of the last sample against the fake stack's simulation clock,
    and the nodes visible to the probe. The real functions run and their results are returned
    unchanged.
    """

    def __init__(self, monkeypatch, sim_now):
        self.entries = []
        orig_collect, orig_evaluate = lpf.GraphProbe.collect, lpf.evaluate
        peers = {}

        def collect(probe):
            obs = orig_collect(probe)
            peers['last'] = sorted(ns.rstrip('/') + '/' + n for n, ns in
                                   probe.node.get_node_names_and_namespaces())
            return obs

        def evaluate(obs, names, neutral):
            report = orig_evaluate(obs, names, neutral)
            stamps = [m[0] for m in (obs.joint_state_messages or [])]
            now = sim_now()
            self.entries.append({
                'monotonic': round(time.monotonic(), 6), 'ready': report['ready'],
                'checks': report['checks'], 'failures': report['failures'],
                'samples': len(stamps), 'first_stamp': stamps[0] if stamps else None,
                'last_stamp': stamps[-1] if stamps else None,
                'non_increasing_at': [i for i in range(1, len(stamps))
                                      if stamps[i] <= stamps[i - 1]][:10],
                'last_sample_age_sim_s': (now - stamps[-1]) if stamps else None,
                'peers': peers.pop('last', None)})
            return report

        monkeypatch.setattr(lpf.GraphProbe, 'collect', collect)
        monkeypatch.setattr(lpf, 'evaluate', evaluate)


def explain(data, recorder, path):
    """Everything needed to diagnose a failed run, as one JSON string for the assertion."""
    return json.dumps({'state': data.get('state'), 'reason': data.get('reason'),
                       'errors': data.get('errors'), 'outcome_file': str(path),
                       'readiness_outcome': data.get('readiness'),
                       'readiness_reports': recorder.entries,
                       'freshness_at_send': data.get('freshness_at_send'),
                       'channels': data.get('channels'), 'tracking': data.get('tracking')},
                      indent=1, default=repr)


def piped_stdin(monkeypatch, text):
    r, w = os.pipe()
    os.write(w, text.encode())
    os.close(w)
    monkeypatch.setattr('sys.stdin', os.fdopen(r, 'r'))


def test_isolation_preconditions():
    assert 150 <= DOMAIN < 200
    assert DOMAIN != int(os.environ.get('ROS_DOMAIN_ID', '0') or 0)
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    # committed gate, outside the test patch; pinned only in m6d_gate
    assert lc.LIVE_DISPATCH_ENABLED is m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED


def test_gate_enabled_main_succeeds_end_to_end_with_one_goal(sources, monkeypatch, tmp_path):
    before = graph_snapshot(DOMAIN)
    if any(before.values()):
        pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {before}')
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', True)         # test-local only
    traj = m6t.build_trajectory(sources)
    stack = fake_stack.IsolatedFakeStack(DOMAIN, traj['joint_names'],
                                         traj['points'][0]['positions'])
    recorder = ReadinessRecorder(monkeypatch, stack.sim_now)
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
    why = explain(data, recorder, files[0])
    assert rc == lpb.EXIT_OK, why
    assert len(recorder.entries) == 2, why
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


# ---------------------------------------------------------------- the fake stack itself
def test_stack_stamp_never_steps_back_at_a_second_boundary():
    # 117.9999999996 s: rounding only the fraction gave nanosec 1e9 -> 0, i.e. 117.0 s
    for t in (117.9999999996, 117.9999999995, 99.99999999951, 118.0, 118.0000000004):
        st = fake_stack._stamp(t)
        assert 0 <= st.nanosec < 1_000_000_000
        assert abs(st.sec + st.nanosec * 1e-9 - t) <= 1e-9, t
    steps = [117.0 + k * 1e-10 for k in range(9_999_990_000, 10_000_010_000, 7)]
    secs = [fake_stack._secs(fake_stack._stamp(t)) for t in steps]
    assert all(b >= a for a, b in zip(secs, secs[1:]))


def test_stack_ticks_never_overlap_and_stamps_leave_in_order(sources):
    """A slow publish (0-25 ms between computing a tick's stamp and handing it to DDS, longer than
    the 10 ms period) must neither overlap two ticks nor reorder their stamps. In a reentrant group
    rclpy starts the next tick while one is still running, and the later stamp can leave first;
    in its own mutually exclusive group one tick runs at a time and every received stamp
    increases."""
    import random

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    before = graph_snapshot(DOMAIN)
    if any(before.values()):
        pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {before}')
    state = {'active': 0, 'max': 0, 'ticks': 0}
    lock = threading.Lock()
    rng = random.Random(7)

    class SlowPublishStack(fake_stack.IsolatedFakeStack):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            real = self.js_pub

            class SlowPublisher:
                def publish(self, msg):
                    time.sleep(rng.uniform(0.0, 0.025))
                    real.publish(msg)
            self.js_pub = SlowPublisher()

        def _tick(self):
            with lock:
                state['active'] += 1
                state['ticks'] += 1
                state['max'] = max(state['max'], state['active'])
            try:
                super()._tick()
            finally:
                with lock:
                    state['active'] -= 1

    traj = m6t.build_trajectory(sources)
    stack = SlowPublishStack(DOMAIN, traj['joint_names'], traj['points'][0]['positions'])
    ctx = Context()
    rclpy.init(context=ctx, domain_id=DOMAIN, signal_handler_options=SignalHandlerOptions.NO)
    stamps = []
    try:
        node = rclpy.create_node('m6d_stack_order_probe', context=ctx, enable_rosout=False,
                                 start_parameter_services=False)
        node.create_subscription(JointState, '/joint_states',
                                 lambda m: stamps.append(fake_stack._secs(m.header.stamp)), 50)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.05)
        ex.shutdown()
        node.destroy_node()
    finally:
        rclpy.try_shutdown(context=ctx)
        stack.stop()
    back = [i for i in range(1, len(stamps)) if stamps[i] <= stamps[i - 1]]
    assert state['ticks'] >= 20 and state['max'] == 1, state
    assert len(stamps) >= 20 and not back, (len(stamps), back[:10])
    assert not any(graph_snapshot(DOMAIN).values())


def test_stack_stop_leaves_no_callback_running_against_destroyed_handles(monkeypatch):
    """Start and stop the stack repeatedly; no rclpy Task may end with an exception nobody
    retrieves. Before the fix, Humble's executor shutdown destroyed its guard condition while a
    pool callback could still trigger it (12 such Tasks in 150 cycles in Cloud), which rclpy
    prints as 'The following exception was never retrieved: cannot use Destroyable ...'."""
    import gc
    import random

    from rclpy import task as rtask

    before = graph_snapshot(DOMAIN)
    if any(before.values()):
        pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {before}')
    orphans = []

    def record_orphan(fut):
        if fut._exception is not None and not fut._exception_fetched:
            orphans.append(repr(fut._exception))
    monkeypatch.setattr(rtask.Future, '__del__', record_orphan)
    rng = random.Random(11)
    names = [f'j{i}' for i in range(12)]
    for _ in range(30):
        stack = fake_stack.IsolatedFakeStack(DOMAIN, names, [0.0] * 12)
        time.sleep(rng.uniform(0.05, 0.2))
        stack.stop()
        del stack
        gc.collect()
    assert orphans == []
    assert not any(graph_snapshot(DOMAIN).values())
