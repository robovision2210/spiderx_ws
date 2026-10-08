"""M5.5 ROS side on a PRIVATE, localhost-only ROS domain (150-199) with in-test fake peers: the
locomotion node in shadow mode (services, /cmd_vel, status, graph ownership checks, evidence),
the keyboard teleop node, the ground-truth body pose, the gated transport and the launch file.

No simulator, no controller, no external graph: the peers stand in for /joint_states and the
pose bridge. Tests skip (with the reason) if the private domain is not isolated.
"""

import json
import os
import threading
import time

import m55_fixtures as fx
import pytest

from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_locomotion as loc
from spiderx_controller import m55_rclpy as m55r
from spiderx_controller import m55_teleop as tp

DOMAIN = 150 + (os.getpid() + 41) % 50
ACTION = '/leg_trajectory_controller/follow_joint_trajectory'


@pytest.fixture(scope='module')
def cfg():
    return fx.config()


@pytest.fixture(scope='module')
def lib():
    return fx.synthetic_library()


@pytest.fixture
def isolated_env(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')


def _context(domain):
    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    return ctx


class Spinner:
    """One context, one executor thread, any number of nodes."""

    def __init__(self, domain):
        from rclpy.executors import SingleThreadedExecutor
        self.ctx = _context(domain)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.running = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.started = False

    def add(self, node):
        self.executor.add_node(node)
        if not self.started:
            self.thread.start()
            self.started = True
        return node

    def _spin(self):
        while self.running and self.ctx.ok():
            self.executor.spin_once(timeout_sec=0.02)

    def stop(self, *nodes):
        import rclpy
        self.running = False
        if self.started:
            self.thread.join(5.0)
        for n in nodes:
            try:
                n.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        self.executor.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown(context=self.ctx)


class Peers:
    """/joint_states (synthetic neutral) and /spiderx/sim/world_poses, plus a client side:
    Trigger clients, a /cmd_vel publisher and the status subscription."""

    def __init__(self, spinner, cfg, joints, tilt_ok=True):
        import rclpy
        from geometry_msgs.msg import TransformStamped, Twist
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage
        self.node = rclpy.create_node('m55_fake_peers', context=spinner.ctx)
        n = self.node
        self.joints = joints
        self.JointState, self.Twist, self.Trigger = JointState, Twist, Trigger
        self.js_pub = n.create_publisher(JointState, cfg.monitor.joint_states_topic, 10)
        self.pose_pub = n.create_publisher(TFMessage, cfg.monitor.pose_topic, 10)
        self.cmd_pub = n.create_publisher(Twist, cfg.command.topic, 10)
        self.clients = {s: n.create_client(Trigger, f'{c55.SERVICE_PREFIX}/{s}')
                        for s in c55.SERVICES}
        self.status = None
        self.statuses = 0
        n.create_subscription(String, c55.STATUS_TOPIC, self._on_status, 10)

        def entry(child, z, frame=''):
            t = TransformStamped()
            t.child_frame_id, t.header.frame_id = child, frame
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            return t
        self.poses = TFMessage(transforms=[entry('ground_plane', 0.0),
                                           entry(cfg.monitor.model_name, 0.1),
                                           entry(cfg.monitor.body_link, 0.0,
                                                 cfg.monitor.model_name)])
        self.sim = 100.0
        self.cmd = None
        n.create_timer(0.02, self._tick)
        n.create_timer(0.1, self._heartbeat)
        spinner.add(n)

    def _on_status(self, msg):
        self.status = json.loads(msg.data)
        self.statuses += 1

    def _tick(self):
        from builtin_interfaces.msg import Time
        self.sim += 0.02
        js = self.JointState(name=list(self.joints), position=[0.0] * len(self.joints))
        js.header.stamp = Time(sec=int(self.sim), nanosec=int((self.sim % 1) * 1e9))
        self.js_pub.publish(js)
        self.pose_pub.publish(self.poses)

    def _heartbeat(self):
        if self.cmd is not None:
            t = self.Twist()
            t.linear.x, t.linear.y, t.angular.z = self.cmd
            self.cmd_pub.publish(t)

    def call(self, name, timeout=5.0):
        c = self.clients[name]
        end = time.monotonic() + timeout
        while not c.service_is_ready() and time.monotonic() < end:
            time.sleep(0.05)
        fut = c.call_async(self.Trigger.Request())
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        assert fut.done(), f'{name}: no response'
        r = fut.result()
        return r.success, r.message

    def wait(self, pred, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(0.02)
        return False


def _isolated_or_skip(node, allowed):
    names = [n for n, _ in node.get_node_names_and_namespaces()]
    if [n for n in names if n not in allowed]:
        pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {names}')


@pytest.fixture
def stack(isolated_env, cfg, lib, tmp_path):
    """The locomotion node (own context) + peers (own context) on the private domain."""
    assert 150 <= DOMAIN < 200 and DOMAIN != int(os.environ.get('ROS_DOMAIN_ID', '0') or 0)
    node_side = Spinner(DOMAIN)
    peer_side = Spinner(DOMAIN)
    ln = m55r.LocomotionNode(cfg, lib, context=node_side.ctx, evidence_dir=str(tmp_path / 'ev'))
    node_side.add(ln.node)
    peers = Peers(peer_side, cfg, lib.joint_names)
    time.sleep(1.0)
    _isolated_or_skip(peers.node, {'spiderx_locomotion', 'm55_fake_peers'})
    yield ln, peers, peer_side, tmp_path / 'ev'
    node_side.running = False
    node_side.thread.join(5.0)
    ln.close()
    node_side.executor.shutdown(timeout_sec=1.0)
    import rclpy
    rclpy.try_shutdown(context=node_side.ctx)
    peer_side.stop(peers.node)


# ==================================================================== the node (shadow mode)
def test_node_runs_shadow_mode_end_to_end(stack, lib):
    ln, peers, _, ev = stack
    assert ln.transport.mode == 'shadow' and c55.M55_LOCOMOTION_DISPATCH_ENABLED is False
    assert peers.wait(lambda: peers.status is not None, 5.0)
    assert peers.status['mode'] == 'shadow' and peers.status['dispatch_gate'] is False
    assert peers.status['levels_m_s'] == pytest.approx(lib.speeds_m_s)
    time.sleep(1.2)                                           # one graph period
    peers.cmd = (0.0, 0.0, 0.0)
    time.sleep(0.3)
    ok, msg = peers.call('arm')
    assert ok, msg
    peers.cmd = (lib.speeds_m_s[0], 0.0, 0.0)
    assert peers.wait(lambda: (peers.status or {}).get('goals_recorded', 0) >= 3, 8.0)
    st = peers.status
    assert st['state'] in ('STARTING', 'WALKING') and st['faults'] == []
    assert st['continuity_reference'].startswith('planned')
    # single owner: nothing reached (or listens for) the controller's command interfaces
    n = peers.node
    assert n.count_subscribers(m55r.action_status_topic(ACTION)) == 0
    assert n.count_publishers('/leg_trajectory_controller/joint_trajectory') == 0
    assert n.count_subscribers('/leg_trajectory_controller/joint_trajectory') == 0
    # turning is rejected and stops the walk
    peers.cmd = (lib.speeds_m_s[0], 0.0, 0.2)
    assert peers.wait(lambda: peers.status['command_rejections'].get(
        'command_unsupported_yaw', 0) > 0, 3.0)
    peers.cmd = (0.0, 0.0, 0.0)
    assert peers.wait(lambda: peers.status['state'] == 'READY', 5.0)
    assert peers.call('estop')[0]
    assert peers.wait(lambda: peers.status['state'] == 'FAULTED', 2.0)
    assert peers.call('arm') == (False, loc.RESET_REQUIRED)
    assert peers.call('reset')[0]
    assert peers.wait(lambda: peers.status['state'] == 'DISARMED', 2.0)
    ln._write_evidence()
    events = [json.loads(line) for line in (ev / 'events.jsonl').read_text().splitlines()]
    assert {'goal_sent', 'phase_done', 'fault', 'reset'} <= {e['event'] for e in events}
    lib_doc = json.loads((ev / 'library.json').read_text())
    assert lib_doc['library']['goals'] == len(lib.goals) and lib_doc['non_claims']


def test_node_lease_expiry_disarms_and_reconnect_does_not_rearm(stack, lib):
    ln, peers, _, _ = stack
    time.sleep(1.2)
    peers.cmd = (0.0, 0.0, 0.0)
    time.sleep(0.3)
    assert peers.call('arm')[0]
    peers.cmd = (lib.speeds_m_s[0], 0.0, 0.0)
    assert peers.wait(lambda: (peers.status or {}).get('state') == 'WALKING', 5.0)
    peers.cmd = None                                          # the teleop disappears
    assert peers.wait(lambda: peers.status['state'] == 'DISARMED', 5.0)
    peers.cmd = (lib.speeds_m_s[0], 0.0, 0.0)                 # it comes back
    time.sleep(1.5)
    assert peers.status['state'] == 'DISARMED'


def test_node_refuses_arm_with_a_foreign_trajectory_client(stack):
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    ln, peers, _, _ = stack
    other = ActionClient(peers.node, FollowJointTrajectory, ACTION)
    try:
        assert peers.wait(lambda: ln.graph_status()['foreign_action_clients'] == 1, 5.0)
        time.sleep(1.2)
        peers.cmd = (0.0, 0.0, 0.0)
        time.sleep(0.3)
        assert peers.call('arm') == (False, loc.OWNER_CONFLICT)
    finally:
        other.destroy()


def test_node_refuses_arm_with_a_trajectory_topic_publisher(stack):
    from trajectory_msgs.msg import JointTrajectory
    ln, peers, _, _ = stack
    pub = peers.node.create_publisher(JointTrajectory,
                                      '/leg_trajectory_controller/joint_trajectory', 10)
    try:
        assert peers.wait(lambda: ln.graph_status()['command_topic_publishers'] == 1, 5.0)
        time.sleep(1.2)
        peers.cmd = (0.0, 0.0, 0.0)
        time.sleep(0.3)
        assert peers.call('arm') == (False, loc.TOPIC_PUBLISHER)
    finally:
        peers.node.destroy_publisher(pub)


def test_teleop_node_against_the_locomotion_node(stack, cfg, lib):
    ln, peers, peer_side, _ = stack
    import rclpy
    lines = []
    tn = tp.TeleopNode(cfg, context=peer_side.ctx, out=lines.append)
    peer_side.add(tn.node)
    try:
        assert peers.wait(lambda: tn.state.levels is not None, 5.0)
        assert tn.state.levels == pytest.approx(lib.speeds_m_s)
        time.sleep(1.2)
        beats = threading.Event()

        def heartbeat():
            while not beats.is_set():
                tn.publish(time.monotonic())
                time.sleep(0.1)
        t = threading.Thread(target=heartbeat, daemon=True)
        t.start()
        time.sleep(0.3)
        assert tn.handle(tp.ARM, time.monotonic())
        assert peers.wait(lambda: any(r[0] == 'arm' for r in tn.responses), 5.0)
        assert [r for r in tn.responses if r[0] == 'arm'][0][1] is True, tn.responses
        for _ in range(15):                                   # hold w (key auto-repeat)
            tn.handle(tp.FORWARD, time.monotonic())
            time.sleep(0.1)
        assert peers.status['intent']['direction'] == 1
        assert peers.status['state'] in ('STARTING', 'WALKING')
        tn.handle(tp.TURN, time.monotonic())
        assert tp.TURN_MESSAGE in lines
        assert not tn.handle(tp.QUIT, time.monotonic())
        assert peers.wait(lambda: {'stop', 'disarm'} <= {r[0] for r in tn.responses}, 5.0)
        assert peers.wait(lambda: peers.status['state'] == 'DISARMED', 5.0)
        beats.set()
        t.join(2.0)
    finally:
        peer_side.executor.remove_node(tn.node)
        tn.close()
    assert rclpy is not None


# ==================================================================== gated transport, pose
def test_dispatch_transport_refuses_to_exist_while_the_gate_is_false(isolated_env, cfg, lib):
    import rclpy
    ctx = _context(DOMAIN)
    node = rclpy.create_node('m55_gate_probe', context=ctx)
    try:
        with pytest.raises(loc.DispatchDisabled):
            m55r.RclpyPhaseTransport(node, cfg, lib.fingerprints)
        assert node.count_subscribers(m55r.action_status_topic(ACTION)) == 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown(context=ctx)


def _tf(child, frame, xyz=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0)):
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.child_frame_id, t.header.frame_id = child, frame
    tr, r = t.transform.translation, t.transform.rotation
    tr.x, tr.y, tr.z = xyz
    r.x, r.y, r.z, r.w = q
    return t


def test_body_pose_sample_composes_model_and_link(cfg):
    import math
    s = math.sin(0.1)                                         # 0.2 rad about x
    codes, pose = m55r.body_pose_sample(
        [_tf('spiderx', '', (1.0, 2.0, 0.1), (s, 0.0, 0.0, math.cos(0.1))),
         _tf('dummy_link', 'spiderx', (0.0, 0.0, 0.02))], cfg)
    assert codes == () and pose['tilt'] == pytest.approx(0.2)
    assert pose['xy'][0] == pytest.approx(1.0) and pose['z'] == pytest.approx(
        0.1 + 0.02 * math.cos(0.2))
    codes, pose = m55r.body_pose_sample([_tf('dummy_link', 'spiderx')], cfg)
    assert pose is None and 'pose_missing_model' in codes
    codes, pose = m55r.body_pose_sample([_tf('spiderx', ''), _tf('spiderx', ''),
                                         _tf('dummy_link', 'spiderx')], cfg)
    assert pose is None and codes


# ==================================================================== launch file
def test_walking_launch_is_free_base_with_the_locomotion_node():
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('spiderx_bringup'), 'launch',
                        'fortress_m55_walking.launch.py')
    import importlib.util
    spec = importlib.util.spec_from_file_location('m55_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    args = {e.name: e.default_value for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert set(args) == {'rviz', 'headless', 'evidence_dir'}
    includes = [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]
    locs = [''.join(getattr(x, 'text', str(x)) for x in
                    e.launch_description_source._LaunchDescriptionSource__location)
            for e in includes]
    assert len(locs) == 1 and locs[0].endswith('fortress_posture_hold.launch.py')
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    assert len(nodes) == 1
    text = open(path).read()
    assert "executable='m55_locomotion_node'" in text and 'fixed_base' not in text
    for w in ('enable', 'dispatch:=', 'gate', 'live'):
        assert w not in ''.join(args), w


# ==================================================================== review: late commanders
def test_arm_uses_a_fresh_graph_snapshot(stack):
    """The node takes a graph snapshot for every request: a client that appeared after the last
    periodic snapshot is already seen by arm (once DDS discovery has propagated)."""
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    ln, peers, _, _ = stack
    time.sleep(1.2)
    peers.cmd = (0.0, 0.0, 0.0)
    time.sleep(0.3)
    other = ActionClient(peers.node, FollowJointTrajectory, ACTION)
    try:
        time.sleep(0.3)                                # discovery, but no periodic poll needed
        assert peers.call('arm') == (False, loc.OWNER_CONFLICT)
    finally:
        other.destroy()


def test_a_late_foreign_client_faults_an_armed_node(stack, lib):
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    ln, peers, _, _ = stack
    time.sleep(1.2)
    peers.cmd = (0.0, 0.0, 0.0)
    time.sleep(0.3)
    assert peers.call('arm')[0]
    peers.cmd = (lib.speeds_m_s[0], 0.0, 0.0)
    assert peers.wait(lambda: (peers.status or {}).get('state') == 'WALKING', 5.0)
    t0 = time.monotonic()
    other = ActionClient(peers.node, FollowJointTrajectory, ACTION)
    try:
        assert peers.wait(lambda: peers.status['state'] == 'FAULTED', 6.0)
        latency = time.monotonic() - t0
        assert peers.status['faults'] == [loc.OWNER_CONFLICT]
        # one graph period (1 s) + discovery + status publication (2 Hz); observed, not bounded
        assert latency < 4.0, latency
        print(f'late foreign client -> FAULTED after {latency:.2f} s')
    finally:
        other.destroy()
