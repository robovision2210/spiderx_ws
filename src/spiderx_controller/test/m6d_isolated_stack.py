"""In-process fake controller STACK for the M6.0-D isolated success-path test (tests only).

One node in its own rclpy Context on an explicit, non-default, localhost-only ROS domain chosen by
the test. It imitates, for the readiness and supervision paths of m6_live_playback, exactly what
the live tool observes - nothing more:
  - /controller_manager/list_controllers: joint_state_broadcaster and leg_trajectory_controller,
    both active (a real service the tool calls through its own executor);
  - the FollowJointTrajectory action server at the controller's action name;
  - /clock (simulation time running `rtf` times faster than wall time) and a single /joint_states
    publisher stamped with that simulation time.
While a goal executes, the published joint positions follow the received goal's own waypoints
with the controller's cubic (zero-velocity) interpolation, then hold the last waypoint. It is NOT
a controller, moves nothing physical and proves no controller behaviour; it records every goal and
cancel request it receives.
"""

import threading
import time

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_live_preflight as lpf

SIM_START_S = 100.0


def _stamp(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def _secs(d):
    return d.sec + d.nanosec * 1e-9


class IsolatedFakeStack:
    def __init__(self, domain_id, joint_names, start_positions, rtf=2.0, rate_hz=100.0):
        self.joint_names = list(joint_names)
        self.positions = list(start_positions)
        self.rtf = rtf
        self.received, self.cancel_requests, self.list_calls = [], 0, 0
        self.results = []
        self._wall0 = time.monotonic()
        self._lock = threading.Lock()
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('m6d_test_double_controller_stack', context=self.context,
                                      enable_rosout=False, start_parameter_services=False)
        group = ReentrantCallbackGroup()
        self.service = self.node.create_service(ListControllers,
                                                lpf.LIST_CONTROLLERS_SERVICE, self._list,
                                                callback_group=group)
        self.server = ActionServer(self.node, FollowJointTrajectory, env.ACTION_NAME,
                                   execute_callback=self._execute, goal_callback=self._goal,
                                   cancel_callback=self._cancel, callback_group=group)
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.js_pub = self.node.create_publisher(JointState, env.JOINT_STATES_TOPIC, 10)
        self.timer = self.node.create_timer(1.0 / rate_hz, self._tick, callback_group=group)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    # ---------------------------------------------------------------- simulation time
    def sim_now(self):
        return SIM_START_S + self.rtf * (time.monotonic() - self._wall0)

    def _tick(self):
        now = self.sim_now()
        self.clock_pub.publish(Clock(clock=_stamp(now)))
        msg = JointState()
        msg.header.stamp = _stamp(now)
        msg.name = list(self.joint_names)
        with self._lock:
            msg.position = [float(p) for p in self.positions]
        self.js_pub.publish(msg)

    # ---------------------------------------------------------------- controller manager
    def _list(self, request, response):
        self.list_calls += 1
        response.controller = [
            ControllerState(name='joint_state_broadcaster', state='active',
                            type='joint_state_broadcaster/JointStateBroadcaster'),
            ControllerState(name=env.CONTROLLER_NAME, state='active', type=lpf.JTC_TYPE)]
        return response

    # ---------------------------------------------------------------- action server
    def _goal(self, goal_request):
        self.received.append(goal_request)
        return GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        self.cancel_requests += 1
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        names = list(goal.trajectory.joint_names)
        plan = {'points': [{'time_from_start_s': _secs(p.time_from_start),
                            'positions': list(p.positions)} for p in goal.trajectory.points]}
        end = plan['points'][-1]['time_from_start_s']
        start = self.sim_now()
        while True:
            elapsed = self.sim_now() - start
            ref = ac.reference_positions(plan, elapsed)
            if ref is not None:
                by_name = dict(zip(names, ref))
                with self._lock:
                    self.positions = [by_name.get(n, q) for n, q in
                                      zip(self.joint_names, self.positions)]
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.results.append('canceled')
                return FollowJointTrajectory.Result()
            fb = FollowJointTrajectory.Feedback()
            fb.header.stamp = _stamp(self.sim_now())
            fb.joint_names = names
            fb.desired.time_from_start.sec = int(elapsed)
            fb.desired.time_from_start.nanosec = int((elapsed - int(elapsed)) * 1e9)
            goal_handle.publish_feedback(fb)
            if elapsed >= end:
                break
            time.sleep(0.02)
        goal_handle.succeed()
        self.results.append('succeeded')
        return FollowJointTrajectory.Result(error_code=0, error_string='test double')

    def stop(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.node.destroy_timer(self.timer)
        self.server.destroy()
        self.node.destroy_service(self.service)
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.context)
