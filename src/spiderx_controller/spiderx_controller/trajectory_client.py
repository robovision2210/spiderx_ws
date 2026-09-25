"""Minimal ROS 2 helpers for the M1 joint-position tests: joint states, controller list,
and one FollowJointTrajectory goal to leg_trajectory_controller."""

import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ACTION = '/leg_trajectory_controller/follow_joint_trajectory'


class JointTestNode(Node):
    """Runs on simulation time (/clock): durations and settle times are simulated seconds, so
    results do not depend on the real-time factor of the machine."""

    def __init__(self, name):
        super().__init__(name, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.positions = None
        self.active_goal = None
        self.create_subscription(JointState, '/joint_states', self._on_state, 10)
        self.action = ActionClient(self, FollowJointTrajectory, ACTION)
        self.list_srv = self.create_client(ListControllers, '/controller_manager/list_controllers')

    def _on_state(self, msg):
        self.positions = dict(zip(msg.name, msg.position))

    def spin_for(self, seconds, wall_timeout=120.0):
        """Spin for `seconds` of SIMULATION time (bounded by wall_timeout)."""
        wall_end = time.monotonic() + wall_timeout
        while self.get_clock().now().nanoseconds == 0 and time.monotonic() < wall_end:
            rclpy.spin_once(self, timeout_sec=0.05)   # wait for the first /clock message
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end and time.monotonic() < wall_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_joint_states(self, timeout=20.0):
        end = time.monotonic() + timeout
        self.positions = None
        while self.positions is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.positions

    def fresh_positions(self, settle=0.3):
        """Joint positions from a message received after `settle` seconds."""
        self.spin_for(settle)
        return self.wait_for_joint_states()

    def controller_states(self, timeout=10.0):
        """[(name, type, state)] from /controller_manager/list_controllers, or None."""
        if not self.list_srv.wait_for_service(timeout_sec=timeout):
            return None
        future = self.list_srv.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            return None
        return [(c.name, c.type, c.state) for c in future.result().controller]

    def send_positions(self, joint_names, targets, duration_s, timeout=None):
        """Send ONE trajectory with a single point reached after duration_s (simulation time)
        and wait for the controller's result. Returns (accepted, error_code or None if no result
        arrived within the wall-clock timeout)."""
        if not self.action.wait_for_server(timeout_sec=10.0):
            return False, None
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(t) for t in targets]
        point.velocities = [0.0] * len(targets)
        sec = int(duration_s)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_s - sec) * 1e9))
        goal.trajectory.points = [point]
        send = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, None
        self.active_goal = handle
        result = handle.get_result_async()
        # Wall-clock timeout generous enough for slow machines (real-time factor ~0.2).
        rclpy.spin_until_future_complete(
            self, result, timeout_sec=timeout if timeout is not None else 10.0 * duration_s + 30.0)
        if result.result() is None:
            return True, None
        self.active_goal = None
        return True, result.result().result.error_code

    def cancel_active_goal(self, timeout=5.0):
        """Cancel the goal still running (for Ctrl-C). Returns True if a cancel was sent."""
        if self.active_goal is None:
            return False
        future = self.active_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        self.active_goal = None
        return True
