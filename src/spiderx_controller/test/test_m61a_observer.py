"""M6.1-A read-only observer, the fixed-base launch file and the fixed-base rclpy transport.

No Gazebo, no launch is run, no goal or command anywhere. The rclpy tests use an explicit,
non-default, localhost-only ROS domain (150-199) with in-test fake peers standing in for
robot_state_publisher, the Gazebo pose bridge, /clock, /joint_states and the controller manager,
and they skip (with the reason) if the domain is not isolated.
"""
import importlib.util
import json
import math
import os
import re
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
import pytest

from spiderx_controller import m61a_fixed_base as fb
from spiderx_controller import m61a_observer as ob

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'spiderx_controller')
CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
JOINTS = ['lf_hip', 'lf_thigh_joint', 'lf_foot_joint', 'rf_hip', 'rf_thigh_joint',
          'rf_foot_joint', 'lr_hip', 'lr_thigh_joint', 'lr_foot_joint', 'rr_hip',
          'rr_thigh_joint', 'rr_foot_joint']
DOMAIN = 150 + (os.getpid() + 33) % 50


@pytest.fixture(scope='module')
def cfg():
    return fb.load_config(CONFIG_DIR)


# ==================================================================== read-only by construction
@pytest.mark.parametrize('module', ['m61a_observer.py', 'm61a_live.py', 'm61a_fixed_base.py',
                                    'm61a_clearance.py'])
def test_m61a_sources_create_no_publisher_goal_or_action_client(module):
    src = open(os.path.join(PKG, module)).read()
    for forbidden in ('create_publisher', 'ActionClient(', 'send_goal', 'send_goal_async',
                      'call_async(FollowJointTrajectory', 'JointTrajectory('):
        assert forbidden not in src, (module, forbidden)
    if module in ('m61a_fixed_base.py', 'm61a_clearance.py'):
        assert 'import rclpy' not in src and 'from rclpy' not in src


def test_interface_only_creates_no_node(cfg, capsys):
    assert ob.main(['--interface-only'], joint_names=JOINTS, config_dir=CONFIG_DIR) == \
        ob.EXIT_READY
    out = capsys.readouterr().out
    data = json.loads(out[out.index('{'):])
    assert data['publishers'] == {} and data['action_clients'] == {}
    assert set(data['subscriptions']) == {'/spiderx/sim/world_poses', '/joint_states', '/clock',
                                          '/robot_description'}
    assert data['service_clients'] == {'/controller_manager/list_controllers':
                                       'controller_manager_msgs/srv/ListControllers'}


@pytest.mark.parametrize('argv', [[], ['--domain-id', '-1'], ['--domain-id', '999'],
                                  ['--domain-id', '1', '--duration', '0'],
                                  ['--domain-id', '1', '--duration', 'nan']])
def test_usage_errors(argv):
    assert ob.main(argv + ['--no-write'], joint_names=JOINTS, config_dir=CONFIG_DIR) == \
        ob.EXIT_USAGE


# ==================================================================== pure statistics
def test_statistics_of_a_stationary_weld(cfg):
    weld = fb.parse_robot_description(fb.minimal_description(cfg.mount))
    st = ob.Stats()
    for i in range(100):
        wall = 10.0 + 0.02 * i + (0.3 if i == 50 else 0.0)
        body = fb.Pose.from_xyz_rpy(0, 0, cfg.mount.p[2] + 1e-5 * math.sin(i))
        model = body * weld.model_to_dummy.inverse()
        sel = fb.select_entries([fb.make_transform('spiderx', model),
                                 fb.make_transform('dummy_link', weld.model_to_dummy)])
        st.on_pose(wall, sel, fb.evaluate_sample(sel, weld, cfg, wall, 0.0))
        st.on_clock(wall, 100.0 + 0.5 * (wall - 10.0))
        st.on_joint_state(wall, 100.0 + 0.5 * (wall - 10.0))
    s = st.summary(cfg)
    assert s['body']['z_range_m'] < 2.1e-5 and s['body']['z_mean_m'] == pytest.approx(0.125)
    assert s['attachment']['max_translation_m'] < 1.1e-5
    assert s['attachment']['fraction_of_tolerance'] < 0.01
    assert s['pose_receipt']['max_s'] == pytest.approx(0.32) and s['pose_receipt']['count'] == 100
    assert s['clock']['real_time_factor'] == pytest.approx(0.5)
    assert s['header_frame_ids'] == {'': 200}


# ==================================================================== CLI with a fake observer
class FakeObserver(ob.FixedBaseObserver):
    """The real assessment, without ROS: evidence is injected."""

    def __init__(self, cfg, joint_names, domain_id, description=None, controllers=None,
                 open_error=None, paused=False):
        super().__init__(cfg, joint_names, domain_id)
        self._paused = paused        # a paused world: /clock and pose/info arrive, time stands
        self._desc = fb.minimal_description(cfg.mount) if description is None else description
        self._ctrl = controllers or {'joint_state_broadcaster': 'active',
                                     'leg_trajectory_controller': 'active'}
        self._open_error = open_error
        self.calls = []

    def open(self):
        self.calls.append('open')
        if self._open_error:
            raise self._open_error
        self.closed = False
        return self

    def close(self):
        self.calls.append('close')
        self.closed = True

    def spin(self, duration_s, stop=None):
        now = time.monotonic()
        self.tracker.on_description(self._desc, now)
        weld = fb.parse_robot_description(fb.minimal_description(self.cfg.mount))
        msg = [fb.make_transform('spiderx', fb.Pose()),
               fb.make_transform('dummy_link', weld.model_to_dummy)]
        rate = 0.0 if self._paused else 0.7                      # sim s per wall s
        self._sim = getattr(self, '_sim', 1.0) + 0.5 * rate
        for i in range(21):                                      # the last 0.5 s of pose/info
            w = now - 0.5 + 0.025 * i
            self.tracker.on_transforms(msg, w, self._sim - rate * (now - w))
        self.clock.on_clock(now - 0.2, self._sim - 0.2 * rate)
        self.clock.on_clock(now, self._sim)
        self.latest_joint_states = (now, {j: 0.0 for j in self.joint_names})

    def controllers(self, timeout_s=5.0):
        return self._ctrl

    def graph(self):
        return {'joint_state_publishers': 1, 'command_publishers': 0, 'action_clients': 0}


def test_cli_ready_and_evidence_never_overwritten(tmp_path, capsys):
    fac = lambda c, j, d: FakeObserver(c, j, d)                              # noqa: E731
    argv = ['--domain-id', '42', '--preflight', '--out', str(tmp_path)]
    assert ob.main(argv, observer_factory=fac, joint_names=JOINTS, config_dir=CONFIG_DIR,
                   utc='20261008T000000Z') == ob.EXIT_READY
    data = json.loads((tmp_path / '20261008T000000Z' / 'observation.json').read_text())
    assert data['ready'] and data['goals_sent'] == 0 and data['publishers_created'] == 0
    assert data['mode'] == 'preflight' and data['domain_id'] == 42
    assert re.fullmatch(r'[0-9a-f]{64}', data['config_sha256'])
    assert 'READY: all fixed-base checks pass' in capsys.readouterr().out
    assert ob.main(argv, observer_factory=fac, joint_names=JOINTS, config_dir=CONFIG_DIR,
                   utc='20261008T000000Z') == ob.EXIT_USAGE                # never overwritten


@pytest.mark.parametrize('kw, code', [
    ({'description': fb.minimal_description(fixed_base=False)}, fb.DESCRIPTION_NOT_FIXED_BASE),
    ({'controllers': {'joint_state_broadcaster': 'active'}}, fb.CONTROLLERS_NOT_ACTIVE),
    # a paused world: every stream is fresh by receipt, but simulation time does not advance
    ({'paused': True}, fb.CLOCK_NOT_ADVANCING),
    ({'paused': True}, fb.POSE_SIM_NOT_ADVANCING),
])
def test_cli_not_ready_names_the_code(kw, code, capsys):
    fac = lambda c, j, d: FakeObserver(c, j, d, **kw)                         # noqa: E731
    assert ob.main(['--domain-id', '1', '--preflight', '--no-write'], observer_factory=fac,
                   joint_names=JOINTS, config_dir=CONFIG_DIR) == ob.EXIT_NOT_READY
    assert code in capsys.readouterr().out


def test_cli_partial_initialization_is_reported_and_cleaned_up(capsys):
    made = []

    def fac(c, j, d):
        made.append(FakeObserver(c, j, d, open_error=RuntimeError('rmw init failed')))
        return made[-1]
    assert ob.main(['--domain-id', '1', '--no-write'], observer_factory=fac, joint_names=JOINTS,
                   config_dir=CONFIG_DIR) == ob.EXIT_NOT_READY
    # the failed open is reported, and close() still runs to release anything half-created
    assert 'observer_failed' in capsys.readouterr().out and made[0].calls == ['open', 'close']


# ==================================================================== the fixed-base launch file
def _load_launch():
    path = os.path.join(get_package_share_directory('spiderx_bringup'), 'launch',
                        'fortress_m61a_fixed_base.launch.py')
    spec = importlib.util.spec_from_file_location('m61a_launch', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m, m.generate_launch_description().entities


def _text(x):
    """Readable text of a launch argument: str, TextSubstitution, Command or a list of them."""
    if isinstance(x, str):
        return x
    if isinstance(x, TextSubstitution):
        return x.text
    if hasattr(x, 'command'):                       # launch.substitutions.Command
        return _text(x.command)
    if hasattr(x, 'value') and not isinstance(x, (list, tuple)):   # ParameterValue
        return _text(x.value)
    if isinstance(x, (list, tuple)):
        return ''.join(_text(p) for p in x)
    return f'<{type(x).__name__}>'


def _params(node):
    return {_text(k): v for d in node._Node__parameters for k, v in d.items()}


def test_fixed_base_launch_spawns_the_wrapper_at_identity():
    m, ents = _load_launch()
    nodes = [e for e in ents if isinstance(e, Node)]
    by_name = {n._Node__node_name: n for n in nodes}
    assert set(by_name) == {'robot_state_publisher', 'spawn_spiderx', 'spiderx_gz_bridge',
                            'spiderx_sim_ground_truth_bridge', 'rviz2'}
    spawn = by_name['spawn_spiderx']
    args = [_text(a) for a in spawn._Node__arguments]
    assert args[:4] == ['-name', 'spiderx', '-topic', 'robot_description']
    assert args[4:] == m.IDENTITY_SPAWN == ['-x', '0', '-y', '0', '-z', '0', '-R', '0', '-P',
                                            '0', '-Y', '0']
    cmd = _text(_params(by_name['robot_state_publisher'])['robot_description'])
    assert 'spiderx_fixed_base.urdf.xacro sim_backend:=fortress enable_control:=true' in cmd
    assert 'mount' not in cmd                                     # no mount override
    gt = by_name['spiderx_sim_ground_truth_bridge']
    assert [_text(a) for a in gt._Node__arguments] == [
        '/world/spiderx_fortress/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']
    remaps = [(_text(a), _text(b)) for a, b in gt._Node__remappings]
    assert remaps == [('/world/spiderx_fortress/pose/info', '/spiderx/sim/world_poses')]
    cfgfile = _text(_params(by_name['spiderx_gz_bridge'])['config_file']).split('\n')[0]
    assert cfgfile.endswith('/config/fortress_bridge_control.yaml')   # launch_ros YAML-encodes
    declared = {e.name for e in ents if type(e).__name__ == 'DeclareLaunchArgument'}
    assert declared == {'headless', 'rviz', 'gz_verbosity'}       # no mount or spawn argument


def test_fixed_base_launch_starts_no_motion_tool():
    path = os.path.join(get_package_share_directory('spiderx_bringup'), 'launch',
                        'fortress_m61a_fixed_base.launch.py')
    src = open(path).read()
    code = src[src.index('def generate_launch_description'):]
    for tool in ('m6_gait_replay', 'm6_live_playback', 'test_one_joint', 'run_posture_hold_test',
                 'teleop', 'm55_', "'/tf'"):
        assert tool not in code, tool
    m, ents = _load_launch()
    includes = [e for e in ents if type(e).__name__ == 'IncludeLaunchDescription']
    paths = [_text(e.launch_description_source._LaunchDescriptionSource__location)
             for e in includes]
    assert sum(p.endswith('controller.launch.py') for p in paths) == 1
    assert sum(p.endswith('gz_sim.launch.py') for p in paths) == 2       # GUI or headless


# ==================================================================== isolated rclpy (private domain)
@pytest.fixture
def isolated_env(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')


class FakePeers:
    """robot_state_publisher, the pose bridge, /clock, /joint_states and list_controllers."""

    def __init__(self, domain, description, cfg, model=None):
        import rclpy
        from controller_manager_msgs.msg import ControllerState
        from controller_manager_msgs.srv import ListControllers
        from geometry_msgs.msg import TransformStamped
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from spiderx_controller import m61a_live
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=domain,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('m61a_fake_peers', context=self.ctx)
        n = self.node
        self.desc_pub = n.create_publisher(String, m61a_live.DESCRIPTION_TOPIC,
                                           m61a_live.description_qos())
        self.desc_pub.publish(String(data=description))
        self.pose_pub = n.create_publisher(TFMessage, cfg.pose_topic, 10)
        self.js_pub = n.create_publisher(JointState, '/joint_states', 10)
        self.clock_pub = n.create_publisher(Clock, '/clock', 10)

        def list_ctrl(req, resp):
            for name in cfg.controllers:
                resp.controller.append(ControllerState(name=name, state='active'))
            return resp
        self.srv = n.create_service(ListControllers, '/controller_manager/list_controllers',
                                    list_ctrl)
        weld = fb.parse_robot_description(fb.minimal_description(cfg.mount))
        self.model = model or fb.Pose()

        def entry(child, pose, frame=''):
            t = TransformStamped()
            t.child_frame_id, t.header.frame_id = child, frame
            t.transform.translation.x, t.transform.translation.y, \
                t.transform.translation.z = pose.p
            r = t.transform.rotation
            r.x, r.y, r.z, r.w = pose.q
            return t
        self.msg = TFMessage(transforms=[entry('ground_plane', fb.Pose()),
                                         entry(cfg.model_name, self.model),
                                         entry(cfg.body_link, weld.model_to_dummy,
                                               cfg.model_name)])
        self.sim = 100.0
        self.Clock, self.JointState = Clock, JointState
        self.timer = n.create_timer(0.02, self._tick)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.running = True
        self.thread.start()

    def _tick(self):
        from builtin_interfaces.msg import Time
        self.sim += 0.02
        t = Time(sec=int(self.sim), nanosec=int((self.sim % 1) * 1e9))
        self.clock_pub.publish(self.Clock(clock=t))
        js = self.JointState(name=JOINTS, position=[0.0] * 12)
        js.header.stamp = t
        self.js_pub.publish(js)
        self.pose_pub.publish(self.msg)

    def _spin(self):
        while self.running and self.ctx.ok():
            self.executor.spin_once(timeout_sec=0.05)

    def stop(self):
        import rclpy
        self.running = False
        self.thread.join(5.0)
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.ctx)


def _isolated_or_skip(observer):
    names = observer.node.get_node_names_and_namespaces()
    if [n for n in names if n[0] not in ('m61a_fixed_base_observer', 'm61a_fake_peers')]:
        pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {names}')


@pytest.mark.parametrize('description, model, ready, code', [
    ('fixed', None, True, None),
    ('free', None, False, fb.DESCRIPTION_NOT_FIXED_BASE),
    ('fixed', fb.Pose.from_xyz_rpy(z=0.075), False, fb.FRAME_SPAWN_NOT_IDENTITY),
])
def test_observer_on_an_isolated_domain(isolated_env, cfg, description, model, ready, code):
    assert 150 <= DOMAIN < 200 and DOMAIN != int(os.environ.get('ROS_DOMAIN_ID', '0') or 0)
    desc = fb.minimal_description(cfg.mount, fixed_base=(description == 'fixed'))
    peers = observer = None
    try:
        observer = ob.FixedBaseObserver(cfg, JOINTS, DOMAIN).open()
        _isolated_or_skip(observer)
        peers = FakePeers(DOMAIN, desc, cfg, model)
        rep = ob.run_observation(observer, cfg, duration_s=1.5, discovery_s=2.0)
        assert rep['ready'] is ready, rep['failure_codes']
        if code:
            assert code in rep['failure_codes']
        assert rep['graph']['end']['command_publishers'] == 0
        assert rep['graph']['end']['pose_publishers'] == ['/m61a_fake_peers']
        assert rep['statistics']['pose_receipt']['count'] > 20
        assert observer.node.count_publishers('/joint_states') == 1
        own = observer.node.get_publisher_names_and_types_by_node('m61a_fixed_base_observer', '/')
        assert [t for t, _ in own if t not in ('/rosout', '/parameter_events')] == []
    finally:
        if peers is not None:
            peers.stop()
        if observer is not None:
            observer.close()
    assert observer.closed and observer.node is None and observer.context is None


def test_fixed_base_transport_composes_the_body_on_an_isolated_domain(isolated_env, cfg):
    from spiderx_controller import m61a_live
    t = m61a_live.M61AFixedBaseTransport('x' * 64, DOMAIN, cfg, node_name='m61a_transport_test',
                                         use_sim_time=False).open()
    peers = None
    try:
        t.poll(1.0)
        names = t.node.get_node_names_and_namespaces()
        if names != [('m61a_transport_test', '/')]:
            pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {names}')
        peers = FakePeers(DOMAIN, fb.minimal_description(cfg.mount), cfg)
        got = []
        end = time.monotonic() + 15.0
        while time.monotonic() < end and not any(e[0] == 'body_pose' for e in got):
            got += t.poll(0.1)
        body = [e for e in got if e[0] == 'body_pose']
        assert body, 'no composed body pose'
        assert body[0][3] == pytest.approx(cfg.mount.xyz_rpy())
        fbe = [e for e in got if e[0] == 'fixed_base']
        assert fbe and fbe[-1][3] == ()
        snap = t.fixed_base_snapshot()
        ok, codes, _ = fb.assess_plant(snap['description'], snap['latest_usable_pose'], cfg)
        assert ok and codes == []
        assert not t._goal_sent and not t._cancel_sent
    finally:
        if peers is not None:
            peers.stop()
        t.close()
    assert t.node is None and t.description_sub is None and t.pose_sub is None


def test_fixed_base_transport_streams_now_on_an_isolated_domain(isolated_env, cfg):
    """The live input of the M6.1 pre-send check: a short spin delivers what is current, the
    stream events it received are dropped (tracking starts at the dispatch), nothing is sent.
    When the peers stop, the same check names the stale streams."""
    from spiderx_controller import m61a_live
    t = m61a_live.M61AFixedBaseTransport('x' * 64, DOMAIN, cfg, node_name='m61a_transport_test',
                                         use_sim_time=False).open()
    peers = None
    try:
        t.poll(1.0)
        names = t.node.get_node_names_and_namespaces()
        if names != [('m61a_transport_test', '/')]:
            pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {names}')
        peers = FakePeers(DOMAIN, fb.minimal_description(cfg.mount), cfg)
        codes, end = None, time.monotonic() + 15.0
        while time.monotonic() < end and codes != []:
            cur = t.streams_now()
            codes, _ = fb.streams_at_send(cur['tracker'], cur['joint_states'], cur['now'],
                                          JOINTS, cfg)
        assert codes == [], codes
        assert not [e for e in t._events if e[0] in m61a_live.STREAM_EVENT_KINDS]
        peers.stop()
        peers = None
        time.sleep(1.2)
        cur = t.streams_now()
        codes, _ = fb.streams_at_send(cur['tracker'], cur['joint_states'], cur['now'], JOINTS,
                                      cfg)
        assert {fb.JOINT_STATES_STALE, fb.POSE_STALE, fb.POSE_SIM_NOT_ADVANCING} <= set(codes)
        assert not t._goal_sent and not t._cancel_sent
    finally:
        if peers is not None:
            peers.stop()
        t.close()


def test_observer_script_interface_only_runs_without_ros(tmp_path):
    """The installed entry point, in a subprocess: --interface-only creates no node."""
    exe = os.path.join(os.path.dirname(get_package_share_directory('spiderx_controller')), '..',
                       'lib', 'spiderx_controller', 'm61a_observe_fixed_base')
    r = subprocess.run([exe, '--interface-only'], capture_output=True, text=True, timeout=120,
                       env=dict(os.environ, ROS_LOCALHOST_ONLY='1'))
    assert r.returncode == 0, r.stderr
    assert '"publishers": {}' in r.stdout


# ==================================================================== late competing commanders
@pytest.mark.parametrize('kind', ['publisher', 'action_client'])
def test_observer_sees_a_late_competing_commander_after_discovery(isolated_env, cfg, kind):
    """A commander that appears AFTER a clean snapshot is only seen by a later snapshot, once
    discovery has propagated: graph counts are point-in-time observations, not a lock."""
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectory
    desc = fb.minimal_description(cfg.mount)
    peers = observer = other = None
    try:
        observer = ob.FixedBaseObserver(cfg, JOINTS, DOMAIN).open()
        _isolated_or_skip(observer)
        peers = FakePeers(DOMAIN, desc, cfg)
        observer.spin(1.0)
        before = observer.graph()
        assert before['command_publishers'] == 0 and before['action_clients'] == 0
        t0 = time.monotonic()
        if kind == 'publisher':
            other = peers.node.create_publisher(JointTrajectory, cfg.command_topic, 10)
            key, code = 'command_publishers', fb.COMMAND_PUBLISHERS
        else:
            other = ActionClient(peers.node, FollowJointTrajectory,
                                 '/leg_trajectory_controller/follow_joint_trajectory')
            key, code = 'action_clients', fb.COMMAND_ACTION_CLIENTS
        seen = None
        while time.monotonic() - t0 < 10.0:
            observer.spin(0.05)
            if observer.graph()[key] == 1:
                seen = time.monotonic() - t0
                break
        assert seen is not None, 'never discovered within 10 s'
        ready, codes, _ = fb.assess(observer.evidence(observer.controllers(), observer.graph()),
                                    cfg, time.monotonic(), JOINTS)
        assert not ready and code in codes
        print(f'{kind} discovered after {seen:.3f} s')       # recorded by pytest -s
    finally:
        if other is not None and kind == 'action_client':
            other.destroy()
        if peers is not None:
            peers.stop()
        if observer is not None:
            observer.close()
