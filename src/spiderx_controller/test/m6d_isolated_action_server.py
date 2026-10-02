"""In-process FollowJointTrajectory ACTION test double for M6.0-D adapter tests (tests only).

Runs in its own rclpy Context on an explicit non-default, localhost-only ROS domain chosen by the
test. It is not a controller: it moves nothing, publishes no /joint_states and no command topic;
it only answers the action (goal response, feedback, result, cancel) and records what it got.
"""

import threading
import time

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from spiderx_controller import m6_envelope as env


class IsolatedFakeServer:
    def __init__(self, domain_id, mode='succeed', error_code=0, duration_s=0.3, feedback_n=3):
        self.mode, self.error_code = mode, error_code
        self.duration_s, self.feedback_n = duration_s, feedback_n
        self.received = []
        self.cancel_requests = 0
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('m6d_test_double_fjt_server', context=self.context,
                                      enable_rosout=False, start_parameter_services=False)
        self.server = ActionServer(self.node, FollowJointTrajectory, env.ACTION_NAME,
                                   execute_callback=self._execute, goal_callback=self._goal,
                                   cancel_callback=self._cancel,
                                   callback_group=ReentrantCallbackGroup())
        self.executor = MultiThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _goal(self, goal_request):
        self.received.append(goal_request)
        return GoalResponse.REJECT if self.mode == 'reject' else GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        self.cancel_requests += 1
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        step = self.duration_s / max(self.feedback_n, 1)
        for i in range(self.feedback_n):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            fb = FollowJointTrajectory.Feedback()
            fb.header.stamp = Time(sec=100 + i, nanosec=0)
            fb.desired.time_from_start.sec = i
            goal_handle.publish_feedback(fb)
            time.sleep(step)
        if self.mode == 'wait':
            end = time.monotonic() + 10.0
            while time.monotonic() < end and not goal_handle.is_cancel_requested:
                time.sleep(0.02)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
        result = FollowJointTrajectory.Result(error_code=self.error_code,
                                              error_string='test double')
        if self.mode == 'abort':
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return result

    def stop(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.server.destroy()
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.context)
