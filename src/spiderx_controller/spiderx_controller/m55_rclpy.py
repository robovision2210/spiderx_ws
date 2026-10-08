"""M5.5 ROS 2 side: the locomotion node (`spiderx_locomotion`) and the gated dispatch transport.

Importing this module starts nothing. The node:
  * subscribes to /cmd_vel (Twist), /joint_states and (when required) the simulator ground truth
    /spiderx/sim/world_poses; queries the graph once per graph_period_s;
  * serves /spiderx/locomotion/{arm,disarm,stop,estop,reset,home} (std_srvs/Trigger);
  * publishes /spiderx/locomotion/status (std_msgs/String, JSON) at status_hz and on every
    state change;
  * runs the LocomotionSession tick at tick_hz on time.monotonic().

It is the SINGLE OWNER of leg motion commands: one action client, created only by
RclpyPhaseTransport, and RclpyPhaseTransport refuses to exist while
m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED is False. In this build the node therefore always
uses the ShadowTransport and creates NO action client and NO publisher towards the controller.
The graph check faults (or refuses arm) if anything else publishes on the controller's topic
interface or holds another FollowJointTrajectory client.

Shutdown never cancels: a goal in flight is one validated phase that ends at rest, so the
controller finishes it and holds (a controlled stop by construction). The emergency stop is the
explicit `estop` service (the teleop's x / Esc).
"""

import json
import os
import time

from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_locomotion as loc
from spiderx_controller import m61a_fixed_base as fb
from spiderx_controller import m6_action_client as ac


def action_status_topic(action):
    return action.rstrip('/') + '/_action/status'


# ======================================================================== dispatch transport
class RclpyPhaseTransport:
    """The ONLY code that can send a locomotion goal. Refuses to exist while the gate is False."""

    mode = 'dispatch'

    def __init__(self, node, cfg, allowed_fingerprints):
        if not c55.M55_LOCOMOTION_DISPATCH_ENABLED:
            raise loc.DispatchDisabled(c55.M55_DISPATCH_DISABLED_MESSAGE)
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        self.cfg = cfg
        self.allowed = frozenset(allowed_fingerprints)
        self.Goal = FollowJointTrajectory.Goal
        self.client = ActionClient(node, FollowJointTrajectory, cfg.dispatch.action)
        self.sent = 0
        self._handle = None
        self._seq = None
        self._cancel_pending = False
        self._events = []

    def ready(self):
        return bool(self.client.server_is_ready())

    def _message(self, goal):
        from builtin_interfaces.msg import Duration, Time
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectoryPoint
        msg = self.Goal()
        msg.trajectory.header.stamp = Time(sec=0, nanosec=0)          # start on receipt
        msg.trajectory.joint_names = list(goal.joint_names)
        for t, q, v in goal.points:
            sec, nsec = ac.seconds_to_duration_fields(t)
            msg.trajectory.points.append(JointTrajectoryPoint(
                positions=[float(x) for x in q], velocities=[float(x) for x in v],
                time_from_start=Duration(sec=sec, nanosec=nsec)))
        tol = self.cfg.dispatch.tracking_tolerance_rad
        msg.path_tolerance = [JointTolerance(name=n, position=tol, velocity=0.0, acceleration=0.0)
                              for n in goal.joint_names]
        msg.goal_tolerance = [JointTolerance(name=n, position=tol, velocity=0.0, acceleration=0.0)
                              for n in goal.joint_names]
        sec, nsec = ac.seconds_to_duration_fields(self.cfg.dispatch.goal_time_tolerance_s)
        msg.goal_time_tolerance = Duration(sec=sec, nanosec=nsec)
        return msg

    def send(self, goal, wall):
        if not c55.M55_LOCOMOTION_DISPATCH_ENABLED:
            raise loc.DispatchDisabled(c55.M55_DISPATCH_DISABLED_MESSAGE)
        loc.verify_goal(goal, self.allowed)
        if self._seq is not None:
            raise loc.LocomotionError('a goal is already in flight (one goal at a time)')
        self.sent += 1
        seq = self._seq = self.sent
        self._cancel_pending = False
        fut = self.client.send_goal_async(self._message(goal),
                                          feedback_callback=lambda m: self._on_feedback(seq, m))
        fut.add_done_callback(lambda f: self._on_goal_response(seq, f))
        return seq

    def cancel(self, wall):
        if self._seq is None:
            return
        if self._handle is None:
            self._cancel_pending = True            # cancel as soon as it is accepted
            return
        seq = self._seq
        self._handle.cancel_goal_async().add_done_callback(
            lambda f: self._on_cancel_response(seq, f))

    def poll(self, wall):
        events, self._events = self._events, []
        return events

    def close(self):
        self.client.destroy()

    # callbacks (executor thread = the node's executor)
    def _on_goal_response(self, seq, future):
        handle = future.result()
        accepted = bool(handle is not None and handle.accepted)
        self._events.append(('goal_response', seq, accepted))
        if not accepted:
            self._seq = None
            return
        self._handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(seq, f))
        if self._cancel_pending:
            self._cancel_pending = False
            handle.cancel_goal_async().add_done_callback(
                lambda f: self._on_cancel_response(seq, f))

    def _on_feedback(self, seq, msg):
        err = list(msg.feedback.error.positions)
        self._events.append(('feedback', seq, max((abs(e) for e in err), default=0.0)))

    def _on_cancel_response(self, seq, future):
        resp = future.result()
        self._events.append(('cancel_response', seq, -1 if resp is None else resp.return_code))

    def _on_result(self, seq, future):
        res = future.result()
        self._handle = None
        self._seq = None
        if res is None:
            self._events.append(('result', seq, -1, -1, 'no result'))
            return
        self._events.append(('result', seq, res.status, res.result.error_code,
                             res.result.error_string))


# ======================================================================== ground truth
def body_pose_sample(transforms, cfg):
    """(codes, pose) from one /spiderx/sim/world_poses message (simulator ground truth).

    T_world_body = T_world_model * T_model_dummy (the dummy_link entry is model-relative) *
    T_dummy_base (identity: dummy_joint has no origin). Development monitor, NOT odometry."""
    sel = fb.select_entries(transforms, cfg.monitor.model_name, cfg.monitor.body_link)
    if sel.codes:
        return sel.codes, None
    body = sel.model * sel.body_link
    roll, pitch, yaw = body.rpy()
    return (), {'xy': (body.p[0], body.p[1]), 'z': body.p[2], 'roll': roll, 'pitch': pitch,
                'yaw': yaw, 'tilt': body.tilt()}


# ======================================================================== the node
class LocomotionNode:
    """Owns the rclpy node, its interfaces and the session. Construct, spin its executor, close."""

    def __init__(self, cfg, library, context=None, node_name=c55.NODE_NAME, evidence_dir=None,
                 clock=time.monotonic):
        import rclpy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.cfg = cfg
        self.lib = library
        self.clock = clock
        self.node = rclpy.create_node(node_name, context=context)
        n = self.node
        self.transport = None
        try:
            if c55.M55_LOCOMOTION_DISPATCH_ENABLED:
                self.transport = RclpyPhaseTransport(n, cfg, library.fingerprints)
            else:
                self.transport = loc.ShadowTransport(library.fingerprints)
            self.session = loc.LocomotionSession(cfg, library, self.transport)
            self.String = String
            self.status_pub = n.create_publisher(String, c55.STATUS_TOPIC, 10)
            n.create_subscription(Twist, cfg.command.topic, self._on_twist, 10)
            n.create_subscription(JointState, cfg.monitor.joint_states_topic,
                                  self._on_joint_state, 10)
            if cfg.monitor.body_pose_required:
                from tf2_msgs.msg import TFMessage
                n.create_subscription(TFMessage, cfg.monitor.pose_topic, self._on_pose, 10)
            self.services = [
                n.create_service(Trigger, f'{c55.SERVICE_PREFIX}/{name}',
                                 lambda req, resp, name=name: self._on_request(name, resp))
                for name in c55.SERVICES]
            n.create_timer(1.0 / cfg.node.tick_hz, self._tick)
            n.create_timer(1.0 / cfg.node.status_hz, self.publish_status)
            n.create_timer(cfg.monitor.graph_period_s, self._graph)
        except BaseException:
            self.close()
            raise
        self.evidence_dir = evidence_dir
        self._written = 0
        self._last_state = self.session.state
        if evidence_dir:
            os.makedirs(evidence_dir, exist_ok=True)
            with open(os.path.join(evidence_dir, 'library.json'), 'w') as f:
                json.dump({'config': cfg.to_dict(), 'library': library.describe(),
                           'non_claims': list(c55.NON_CLAIMS)}, f, indent=1, sort_keys=True)
        n.get_logger().info(
            f'{c55.MILESTONE} locomotion node up in {self.transport.mode.upper()} mode '
            f'(dispatch gate {c55.M55_LOCOMOTION_DISPATCH_ENABLED}); levels '
            f'{[round(v * 1000, 2) for v in library.speeds_m_s]} mm/s')
        if self.transport.mode == 'shadow':
            n.get_logger().warning(c55.M55_DISPATCH_DISABLED_MESSAGE)

    # -------------------------------------------------------------- callbacks
    def _on_twist(self, msg):
        lin, ang = msg.linear, msg.angular
        self.session.on_command(self.clock(), (lin.x, lin.y, lin.z), (ang.x, ang.y, ang.z))

    def _on_joint_state(self, msg):
        st = msg.header.stamp
        self.session.on_joint_state(self.clock(), st.sec + st.nanosec * 1e-9, list(msg.name),
                                    list(msg.position))

    def _on_pose(self, msg):
        codes, pose = body_pose_sample(msg.transforms, self.cfg)
        self.session.on_body_pose(self.clock(), codes, pose)

    def _on_request(self, name, resp):
        ok, message = self.session.request(name, self.clock())
        resp.success, resp.message = bool(ok), str(message)
        self._after()
        return resp

    def graph_status(self):
        n, d = self.node, self.cfg.dispatch
        status_topic = action_status_topic(d.action)
        own = 1 if self.transport.mode == 'dispatch' else 0
        if self.transport.mode == 'dispatch':
            server = self.transport.ready()
        else:
            server = n.count_publishers(status_topic) > 0
        return {'joint_state_publishers': n.count_publishers(self.cfg.monitor.joint_states_topic),
                'command_topic_publishers': n.count_publishers(d.command_topic),
                'foreign_action_clients': max(0, n.count_subscribers(status_topic) - own),
                'action_server_present': bool(server)}

    def _graph(self):
        self.session.on_graph(self.clock(), self.graph_status())

    def _tick(self):
        self.session.step(self.clock())
        self._after()

    def _after(self):
        if self.session.state != self._last_state:
            self._last_state = self.session.state
            self.publish_status()
        self._write_evidence()

    def status(self):
        return self.session.status(self.clock())

    def publish_status(self):
        self.status_pub.publish(self.String(data=json.dumps(self.status(), sort_keys=True)))

    def _write_evidence(self):
        if not self.evidence_dir:
            return
        log = list(self.session.log)
        new = [e for e in log if e['n'] > self._written]
        if not new:
            return
        with open(os.path.join(self.evidence_dir, 'events.jsonl'), 'a') as f:
            for e in new:
                f.write(json.dumps(e, sort_keys=True, default=repr) + '\n')
        self._written = new[-1]['n']

    def close(self):
        """Release local resources. Never cancels: a phase in flight completes at rest."""
        node, transport = self.node, self.transport
        self.transport = None
        if self.evidence_dir and getattr(self, 'session', None) is not None:
            try:
                self._write_evidence()
                with open(os.path.join(self.evidence_dir, 'final_status.json'), 'w') as f:
                    json.dump(self.session.status(self.clock()), f, indent=1, sort_keys=True,
                              default=repr)
                records = list(getattr(transport, 'records', []))
                if records:
                    with open(os.path.join(self.evidence_dir, 'shadow_goals.json'), 'w') as f:
                        json.dump(records, f, indent=1, sort_keys=True)
            except OSError:
                pass
        if transport is not None:
            transport.close()
        if node is not None:
            node.destroy_node()
            self.node = None


def parse_args(argv):
    import argparse
    p = argparse.ArgumentParser(
        prog='m55_locomotion_node',
        description='SpiderX M5.5 locomotion node. Continuous locomotion dispatch is '
                    'hard-disabled in this build: the node runs in SHADOW mode and sends '
                    'nothing to the controller.')
    p.add_argument('--evidence-dir', default='',
                   help='write events.jsonl, library.json and final_status.json here')
    return p.parse_args(argv)


def main(argv=None):
    import sys
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.utilities import remove_ros_args
    args = parse_args(remove_ros_args(sys.argv if argv is None else argv)[1:])
    cfg = loc.load_config()
    print(f'{c55.MILESTONE}: building and validating the crawl templates '
          f'({len(cfg.gait.stride_levels_m)} levels, ~20 s each) ...', flush=True)
    library = loc.build_library(cfg, progress=lambda m: print('  ' + m, flush=True))
    rclpy.init(args=sys.argv if argv is None else argv)
    node = None
    try:
        node = LocomotionNode(cfg, library, evidence_dir=args.evidence_dir or None)
        executor = SingleThreadedExecutor()
        executor.add_node(node.node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    finally:
        if node is not None:
            node.close()
        rclpy.try_shutdown()
    return 0


__all__ = ['RclpyPhaseTransport', 'LocomotionNode', 'body_pose_sample', 'action_status_topic',
           'main']
