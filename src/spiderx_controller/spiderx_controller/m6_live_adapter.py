"""M6.0-D rclpy LiveTransport (Batch C). Importing this module starts nothing.

RclpyLiveTransport is the only code that can put the approved M6.0-D goal on a ROS graph:
  - open() creates its OWN rclpy Context with rclpy's signal handlers DISABLED
    (SignalHandlerOptions.NO, D2), so a Ctrl+C cannot shut the context down before the one cancel;
    the caller owns SIGINT/SIGTERM (m6_live_playback.InterruptLatch);
  - send_goal() is single-use and verifies the goal fingerprint BEFORE any ROS call; a goal that
    is not the approved M6.0-D goal (e.g. a return-to-neutral) is refused;
  - cancel_goal() is single-use (cancel-only; never a new goal);
  - it publishes nothing: one action client, one /joint_states subscription, graph queries.

Live use against the Fortress graph is HARD-DISABLED by the CLI (m6_live_contract). In this build
the transport is exercised only by tests on an explicit, non-default, localhost-only ROS domain
against an in-process test double.
"""

import time

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_goal_fingerprint as gf


class TransportError(RuntimeError):
    """The transport was used outside its contract (not open, wrong domain, ...)."""


def _secs(t):
    return t.sec + t.nanosec * 1e-9


class RclpyLiveTransport:
    """LiveTransport over rclpy. Construct, then open(); always close()."""

    def __init__(self, expected_fingerprint, domain_id, node_name='m6d_live_playback',
                 use_sim_time=True, action_name=env.ACTION_NAME,
                 joint_states_topic=env.JOINT_STATES_TOPIC):
        if not isinstance(domain_id, int) or isinstance(domain_id, bool) or domain_id < 0:
            raise TransportError(f'an explicit ROS domain id is required, got {domain_id!r}')
        self.expected_fingerprint = expected_fingerprint
        self.domain_id = domain_id
        self.node_name = node_name
        self.use_sim_time = use_sim_time
        self.action_name = action_name
        self.joint_states_topic = joint_states_topic
        self.context = None
        self.node = None
        self._events = []
        self._handle = None
        self._goal_sent = False
        self._cancel_sent = False
        self._last_stamp = None

    # ---------------------------------------------------------------- lifecycle
    def open(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from control_msgs.action import FollowJointTrajectory
        from sensor_msgs.msg import JointState

        self._rclpy = rclpy
        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.domain_id,
                   signal_handler_options=SignalHandlerOptions.NO)
        if self.context.get_domain_id() != self.domain_id:
            raise TransportError('context domain id differs from the requested one')
        self.node = rclpy.create_node(
            self.node_name, context=self.context,
            parameter_overrides=[Parameter('use_sim_time', value=self.use_sim_time)])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.client = ActionClient(self.node, FollowJointTrajectory, self.action_name)
        self.sub = self.node.create_subscription(JointState, self.joint_states_topic,
                                                 self._on_joint_state, 10)
        return self

    def close(self):
        try:
            if self.node is not None:
                self.executor.remove_node(self.node)
                self.client.destroy()
                self.node.destroy_subscription(self.sub)
                self.node.destroy_node()
                self.executor.shutdown()
        finally:
            if self.context is not None:
                self._rclpy.try_shutdown(context=self.context)
            self.node = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def _require_open(self):
        if self.node is None or not self.context.ok():
            raise TransportError('transport is not open')

    # ---------------------------------------------------------------- clocks
    def wall_now(self):
        return time.monotonic()

    def sim_now(self):
        if self.node is None:
            return None
        ns = self.node.get_clock().now().nanoseconds
        if ns > 0:
            return ns * 1e-9
        return self._last_stamp

    # ---------------------------------------------------------------- action
    def server_ready(self, timeout_s):
        self._require_open()
        return bool(self.client.wait_for_server(timeout_sec=timeout_s))

    def send_goal(self, goal, binding):
        """Single-use; the fingerprint is verified before anything reaches ROS."""
        if self._goal_sent:
            raise ac.SecondGoalForbidden('this transport already sent its one goal')
        gf.verify_goal(goal, binding, self.expected_fingerprint)
        self._require_open()
        self._goal_sent = True
        future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def cancel_goal(self):
        """Single-use cancel request; never sends a goal."""
        if self._cancel_sent:
            raise ac.SecondGoalForbidden('this transport already sent its one cancel')
        self._require_open()
        self._cancel_sent = True
        if self._handle is None:
            self._events.append(('cancel_response', -1))      # nothing accepted to cancel
            return
        self._handle.cancel_goal_async().add_done_callback(self._on_cancel_response)

    def poll(self, timeout_s):
        self._require_open()
        end = time.monotonic() + timeout_s
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            self.executor.spin_once(timeout_sec=remaining)
        events, self._events = self._events, []
        return events

    def graph_status(self):
        self._require_open()
        return {'action_server_present': bool(self.client.server_is_ready()),
                'joint_state_publishers': self.node.count_publishers(self.joint_states_topic)}

    # ---------------------------------------------------------------- callbacks
    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None:
            return
        self._events.append(('goal_response', bool(handle.accepted)))
        if handle.accepted:
            self._handle = handle
            handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        res = future.result()
        if res is None:
            return
        self._events.append(('result', res.status, res.result.error_code,
                             res.result.error_string))

    def _on_cancel_response(self, future):
        resp = future.result()
        self._events.append(('cancel_response', -1 if resp is None else resp.return_code))

    def _on_feedback(self, msg):
        fb = msg.feedback
        self._events.append(('feedback', {
            'stamp': _secs(fb.header.stamp),
            'desired_time_from_start': _secs(fb.desired.time_from_start),
            'error': list(fb.error.positions)}))

    def _on_joint_state(self, msg):
        stamp = _secs(msg.header.stamp)
        self._last_stamp = stamp
        self._events.append(('joint_state', time.monotonic(), stamp, list(msg.name),
                             [float(p) for p in msg.position]))


__all__ = ['RclpyLiveTransport', 'TransportError']
