"""colcon test: M6.0-B live read-only preflight, with mocks only (no ROS graph is contacted).

Proves: the decision logic for every requirement; timeout behaviour; the read-only probe touches
only graph queries, one list_controllers client and one /joint_states subscription (no goal, no
publisher, no action client, no launch); reports are written under log/ and never overwritten.
"""
import datetime
import inspect
import json
import os
import subprocess
import sys

import pytest

from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_live_preflight as lp

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
NAMES, NEUTRAL = lp.contract_from_config(SRC_CONFIG)
GOOD_INTERFACE = {
    'action_file_sha256': 'x',
    'goal_fields': ['component_goal_tolerance', 'component_path_tolerance', 'goal_time_tolerance',
                    'goal_tolerance', 'multi_dof_trajectory', 'path_tolerance', 'trajectory'],
    'tolerance_fields': ['acceleration', 'name', 'position', 'velocity'],
    'error_codes': dict(lp.REQUIRED_ERROR_CODES),
}


def good_obs(**kw):
    obs = lp.Observations(
        action_servers=[('/leg_trajectory_controller', [env.ACTION_TYPE])],
        controllers=[('joint_state_broadcaster', 'joint_state_broadcaster/JointStateBroadcaster',
                      'active'),
                     ('leg_trajectory_controller', lp.JTC_TYPE, 'active')],
        joint_state_publishers=[('/joint_state_broadcaster', lp.JOINT_STATE_TYPE)],
        joint_state_messages=[(10.0 + 0.02 * i, list(reversed(NAMES)), [0.001] * 12)
                              for i in range(5)],
        versions=dict(lp.REFERENCE_VERSIONS),
        interface=dict(GOOD_INTERFACE))
    for k, v in kw.items():
        setattr(obs, k, v)
    return obs


def codes(obs):
    return lp.evaluate(obs, NAMES, NEUTRAL)['failure_codes']


# ---------------------------------------------------------------- decision logic
def test_contract_comes_from_the_controller_yaml():
    assert NAMES[0] == 'lf_hip' and len(NAMES) == 12 and NEUTRAL == [0.0] * 12


def test_all_requirements_met_is_ready():
    r = lp.evaluate(good_obs(), NAMES, NEUTRAL)
    assert r['ready'] and r['verdict'] == 'READY' and r['failures'] == []
    assert set(r['checks'].values()) == {'passed'}
    assert r['goals_sent'] == 0 and r['messages_published'] == 0
    assert 'does NOT prove tracking, movement, contact' in r['not_proof']
    json.dumps(r, allow_nan=False)


CASES = {
    'action_server_missing': dict(action_servers=[]),
    'action_server_ambiguous': dict(action_servers=[('/a', [env.ACTION_TYPE]),
                                                    ('/b', [env.ACTION_TYPE])]),
    'action_type_mismatch': dict(action_servers=[('/a', ['control_msgs/action/Other'])]),
    'controller_manager_unavailable': dict(controllers=None),
    'controller_missing': dict(controllers=[('leg_trajectory_controller', lp.JTC_TYPE,
                                             'active')]),
    'controller_not_active': dict(controllers=[
        ('joint_state_broadcaster', 'x', 'active'),
        ('leg_trajectory_controller', lp.JTC_TYPE, 'inactive')]),
    'controller_type_mismatch': dict(controllers=[
        ('joint_state_broadcaster', 'x', 'active'),
        ('leg_trajectory_controller', 'forward_command_controller/X', 'active')]),
    'joint_states_no_publisher': dict(joint_state_publishers=[]),
    'joint_states_multiple_publishers': dict(joint_state_publishers=[
        ('/jsb', lp.JOINT_STATE_TYPE), ('/other', lp.JOINT_STATE_TYPE)]),
    'joint_states_type_mismatch': dict(joint_state_publishers=[('/jsb', 'std_msgs/msg/String')]),
    'joint_states_no_messages': dict(joint_state_messages=[(1.0, NAMES, [0.0] * 12)]),
    'joint_states_stale': dict(joint_state_messages=[(5.0, NAMES, [0.0] * 12)] * 3),
    'joint_names_mismatch': dict(joint_state_messages=[(1.0 + i, NAMES[:11] + ['extra'],
                                                        [0.0] * 12) for i in range(3)]),
    'joint_states_incomplete': dict(joint_state_messages=[(1.0 + i, NAMES, [0.0] * 11)
                                                          for i in range(3)]),
    'start_pose_not_neutral': dict(joint_state_messages=[(1.0 + i, NAMES, [0.0] * 11 + [0.06])
                                                         for i in range(3)]),
    'interface_contract_mismatch': dict(interface={**GOOD_INTERFACE,
                                                   'goal_fields': ['trajectory']}),
    'probe_error': dict(probe_errors=['controllers: RuntimeError: boom']),
}


@pytest.mark.parametrize('code', sorted(CASES))
def test_each_missing_requirement_is_not_ready(code):
    r = lp.evaluate(good_obs(**CASES[code]), NAMES, NEUTRAL)
    assert not r['ready'] and r['verdict'] == 'NOT READY'
    assert r['failure_codes'] == [code], r['failures']


def test_every_failure_code_is_exercised():
    assert set(CASES) == set(lp.FAILURE_CODES)


def test_zero_stamps_are_stale():
    assert codes(good_obs(joint_state_messages=[(0.0, NAMES, [0.0] * 12)] * 3)) == \
        ['joint_states_stale']


def test_nan_positions_are_incomplete_and_report_is_valid_json():
    msgs = [(1.0 + i, NAMES, [float('nan')] + [0.0] * 11) for i in range(3)]
    r = lp.evaluate(good_obs(joint_state_messages=msgs), NAMES, NEUTRAL)
    assert r['failure_codes'] == ['joint_states_incomplete']
    json.dumps(r, allow_nan=False)


def test_start_pose_tolerance_is_its_own_quantity():
    at_edge = [(1.0 + i, NAMES, [0.0] * 11 + [0.05]) for i in range(3)]
    assert codes(good_obs(joint_state_messages=at_edge)) == []
    assert env.START_POSE_TOLERANCE_RAD == 0.05
    assert 'TRACKING_TOLERANCE' not in inspect.getsource(lp)
    assert 'M6_0_D_MAX_DISPLACEMENT' not in inspect.getsource(lp)


def test_version_differences_are_visible_but_not_a_failure():
    versions = dict(lp.REFERENCE_VERSIONS, joint_trajectory_controller='2.43.0',
                    gz_ros2_control=None)
    r = lp.evaluate(good_obs(versions=versions), NAMES, NEUTRAL)
    assert r['ready']
    vc = r['version_check']
    assert vc['status'] == 'differs'
    assert vc['packages']['joint_trajectory_controller']['status'] == 'differs'
    assert vc['packages']['gz_ros2_control']['status'] == 'unavailable'
    assert vc['packages']['control_msgs']['status'] == 'matches'


def test_interface_only_never_claims_ready():
    obs = lp.Observations(versions=dict(lp.REFERENCE_VERSIONS), interface=dict(GOOD_INTERFACE),
                          graph_probed=False)
    r = lp.evaluate(obs, NAMES, NEUTRAL)
    assert r['verdict'] == 'INTERFACE ONLY' and r['ready'] is False and r['failures'] == []
    assert r['checks']['action_server'] == 'not_run'


def test_installed_interface_and_versions_are_collected_offline():
    itf = lp.collect_interface()
    versions = lp.collect_versions()
    r = lp.evaluate(lp.Observations(versions=versions, interface=itf, graph_probed=False),
                    NAMES, NEUTRAL)
    assert r['failures'] == []
    assert set(lp.REFERENCE_VERSIONS) <= set(versions)


# ---------------------------------------------------------------- read-only probe with fakes
class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeFuture:
    def __init__(self, result, done=True):
        self._result, self._done = result, done

    def done(self):
        return self._done

    def result(self):
        return self._result if self._done else None


class FakeClient:
    def __init__(self, node, ready, response, done):
        self.node, self.ready, self.response, self.done = node, ready, response, done

    def wait_for_service(self, timeout_sec):
        self.node.log.append(('wait_for_service', timeout_sec))
        return self.ready

    def call_async(self, request):
        self.node.log.append(('call_async', type(request).__name__))
        return FakeFuture(self.response, self.done)


class Ctrl:
    def __init__(self, name, type_, state):
        self.name, self.type, self.state = name, type_, state


class PubInfo:
    def __init__(self, node_name, ns, topic_type):
        self.node_name, self.node_namespace, self.topic_type = node_name, ns, topic_type


class Stamp:
    def __init__(self, t):
        self.sec, self.nanosec = int(t), int(round((t - int(t)) * 1e9))


class Msg:
    def __init__(self, t, names, positions):
        self.header = type('H', (), {'stamp': Stamp(t)})()
        self.name, self.position = names, positions


ALLOWED_NODE_CALLS = {'get_node_names_and_namespaces', 'create_client', 'destroy_client',
                      'get_publishers_info_by_topic', 'create_subscription',
                      'destroy_subscription'}


class FakeNode:
    """Records every call. Any method outside the read-only allow-list raises."""

    def __init__(self, service_ready=True, service_done=True, messages=5):
        self.log = []
        self.callback = None
        self.service_ready, self.service_done, self.messages = service_ready, service_done, \
            messages

    def __getattr__(self, name):
        raise AssertionError(f'probe called a non-read-only node method: {name}')

    def get_node_names_and_namespaces(self):
        self.log.append(('get_node_names_and_namespaces',))
        return [('leg_trajectory_controller', '/'), ('robot_state_publisher', '/')]

    def create_client(self, srv_type, name):
        self.log.append(('create_client', srv_type.__name__, name))
        resp = type('R', (), {'controller': [
            Ctrl('joint_state_broadcaster', 'jsb', 'active'),
            Ctrl('leg_trajectory_controller', lp.JTC_TYPE, 'active')]})()
        return FakeClient(self, self.service_ready, resp, self.service_done)

    def destroy_client(self, client):
        self.log.append(('destroy_client',))

    def get_publishers_info_by_topic(self, topic):
        self.log.append(('get_publishers_info_by_topic', topic))
        return [PubInfo('joint_state_broadcaster', '/', lp.JOINT_STATE_TYPE)]

    def create_subscription(self, msg_type, topic, callback, qos):
        self.log.append(('create_subscription', msg_type.__name__, topic))
        self.callback = callback
        return object()

    def destroy_subscription(self, sub):
        self.log.append(('destroy_subscription',))
        self.callback = None


class FakeRos:
    class ListControllers:
        class Request:
            pass

    class JointState:
        pass

    def __init__(self, node, clock):
        self.node, self.clock, self.sent = node, clock, 0

    def spin_once(self, node, timeout_sec):
        self.clock.t += timeout_sec
        if node.callback is not None and self.sent < node.messages:
            self.sent += 1
            node.callback(Msg(100.0 + 0.02 * self.sent, list(NAMES), [0.0] * 12))

    def spin_until_future_complete(self, node, future, timeout_sec):
        self.clock.t += 0.01

    def get_action_server_names_and_types_by_node(self, node, name, ns):
        node.log.append(('get_action_server_names_and_types_by_node', name))
        if name == 'leg_trajectory_controller':
            return [(env.ACTION_NAME, [env.ACTION_TYPE])]
        return []


def probe(**node_kw):
    clock = FakeClock()
    node = FakeNode(**node_kw)
    return node, lp.GraphProbe(node, FakeRos(node, clock), timeout_s=3.0, window_s=1.0,
                               discovery_s=0.5, monotonic=clock).collect()


def test_probe_collects_everything_read_only():
    node, obs = probe()
    assert obs.probe_errors == []
    assert obs.action_servers == [('/leg_trajectory_controller', [env.ACTION_TYPE])]
    assert [c[0] for c in obs.controllers] == list(env.REQUIRED_CONTROLLERS)
    assert obs.joint_state_publishers == [('/joint_state_broadcaster', lp.JOINT_STATE_TYPE)]
    assert len(obs.joint_state_messages) == 5
    called = {entry[0] for entry in node.log}
    assert called <= ALLOWED_NODE_CALLS | {'wait_for_service', 'call_async',
                                           'get_action_server_names_and_types_by_node'}
    assert ('create_client', 'ListControllers', lp.LIST_CONTROLLERS_SERVICE) in node.log
    assert ('create_subscription', 'JointState', env.JOINT_STATES_TOPIC) in node.log
    assert ('destroy_client',) in node.log and ('destroy_subscription',) in node.log
    obs.versions, obs.interface = dict(lp.REFERENCE_VERSIONS), dict(GOOD_INTERFACE)
    assert lp.evaluate(obs, NAMES, NEUTRAL)['ready']


def test_probe_service_timeout_is_not_ready():
    _, obs = probe(service_ready=False)
    assert obs.controllers is None
    obs.versions, obs.interface = dict(lp.REFERENCE_VERSIONS), dict(GOOD_INTERFACE)
    assert codes(obs) == ['controller_manager_unavailable']


def test_probe_unanswered_query_is_not_ready():
    _, obs = probe(service_done=False)
    assert obs.controllers is None


def test_probe_without_joint_states_is_not_ready():
    _, obs = probe(messages=0)
    obs.versions, obs.interface = dict(lp.REFERENCE_VERSIONS), dict(GOOD_INTERFACE)
    assert codes(obs) == ['joint_states_no_messages']


def test_probe_window_is_bounded_by_the_clock():
    clock = FakeClock()
    node = FakeNode(messages=10 ** 6)
    obs = lp.GraphProbe(node, FakeRos(node, clock), window_s=1.0, discovery_s=0.0,
                        monotonic=clock).collect()
    assert len(obs.joint_state_messages) <= lp.MAX_KEPT_MESSAGES
    assert clock.t < 30.0


def test_probe_errors_are_recorded_not_hidden():
    class Exploding(FakeNode):
        def get_publishers_info_by_topic(self, topic):
            raise RuntimeError('graph error')
    clock = FakeClock()
    node = Exploding()
    obs = lp.GraphProbe(node, FakeRos(node, clock), discovery_s=0.0, window_s=0.2,
                        monotonic=clock).collect()
    assert any('graph error' in e for e in obs.probe_errors)


# ---------------------------------------------------------------- no-dispatch guarantee
def test_module_has_no_dispatch_publish_or_launch_path():
    src = inspect.getsource(lp)
    for word in ('send_goal', 'ActionClient', 'create_publisher', '.publish(', 'subprocess',
                 'Popen', 'os.system', 'switch_controller', 'load_controller', 'set_parameters',
                 'm6_action_client', 'FollowJointTrajectory.Goal(', 'cmd_vel'):
        assert word not in src, word


def test_importing_the_tool_imports_no_ros_client():
    code = ('import sys, spiderx_controller.m6_live_preflight;'
            'bad=[m for m in sys.modules if m.split(".")[0] in ("rclpy","control_msgs")'
            ' or m.endswith("m6_action_client")]; print(bad); sys.exit(1 if bad else 0)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr


# ---------------------------------------------------------------- CLI
class FakeGraph:
    def __init__(self, obs):
        self.__dict__.update(obs.__dict__)


NOW = datetime.datetime(2026, 10, 2, 12, 0, 0, tzinfo=datetime.timezone.utc)


def cli(tmp_path, obs=None, extra=(), factory=None):
    calls = []

    def default_factory(args):
        calls.append(args)
        return FakeGraph(obs or good_obs())
    rc = lp.main(['m6_live_preflight', '--config-dir', SRC_CONFIG, '--out', str(tmp_path),
                  *extra], probe_factory=factory or default_factory,
                 versions=dict(lp.REFERENCE_VERSIONS), interface=dict(GOOD_INTERFACE), now=NOW)
    return rc, calls


def test_cli_ready_writes_report_under_out(tmp_path, capsys):
    rc, calls = cli(tmp_path)
    assert rc == lp.EXIT_READY and len(calls) == 1
    path = tmp_path / '20261002T120000Z' / 'live_preflight.json'
    report = json.loads(path.read_text())
    assert report['verdict'] == 'READY' and report['goals_sent'] == 0
    out = capsys.readouterr().out
    assert 'does NOT prove tracking, movement, contact' in out
    assert 'Sends no goal' in out


def test_cli_not_ready_is_nonzero(tmp_path, capsys):
    rc, _ = cli(tmp_path, obs=good_obs(action_servers=[]))
    assert rc == lp.EXIT_NOT_READY
    assert 'NOT READY action_server_missing' in capsys.readouterr().out


def test_cli_never_overwrites_a_report(tmp_path, capsys):
    assert cli(tmp_path)[0] == lp.EXIT_READY
    assert cli(tmp_path)[0] == lp.EXIT_REFUSED
    assert 'never overwritten' in capsys.readouterr().out


def test_cli_probe_exception_is_not_ready(tmp_path):
    def boom(args):
        raise RuntimeError('rclpy init failed')
    rc, _ = cli(tmp_path, factory=boom, extra=('--no-write',))
    assert rc == lp.EXIT_NOT_READY


def test_cli_interrupt_is_not_ready(tmp_path):
    def interrupted(args):
        raise KeyboardInterrupt
    rc, _ = cli(tmp_path, factory=interrupted, extra=('--no-write',))
    assert rc == lp.EXIT_NOT_READY


def test_cli_interface_only_creates_no_probe(tmp_path, capsys):
    rc, calls = cli(tmp_path, extra=('--interface-only', '--no-write'))
    assert rc == lp.EXIT_READY and calls == []
    assert 'INTERFACE ONLY' in capsys.readouterr().out


@pytest.mark.parametrize('arg', ['--timeout=0', '--window=-1'])
def test_cli_rejects_bad_timeouts(arg, tmp_path):
    assert cli(tmp_path, extra=(arg,))[0] == lp.EXIT_REFUSED
