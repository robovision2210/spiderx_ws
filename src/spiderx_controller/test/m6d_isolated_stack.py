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

Like a real joint_state_broadcaster, it publishes from ONE update loop: the 100 Hz tick runs in its
own mutually exclusive callback group, so two ticks never overlap and the /clock and /joint_states
stamps leave in strictly increasing order. (In a reentrant group, rclpy releases a timer before its
callback runs, so a slow tick could overlap the next one.) stop() first stops scheduling, then
lets every scheduled callback finish, and only then shuts the executor down and destroys anything
(see SpinThread), so no callback can touch a destroyed entity or guard condition.
"""

import threading
import time

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_live_preflight as lpf

SIM_START_S = 100.0


class SpinThread:
    """Spins a MultiThreadedExecutor in a daemon thread until stop() (tests only).

    rclpy 3.3 (Humble) Executor.shutdown() waits only for callbacks already running and then
    destroys the executor's guard condition; a callback the pool starts just afterwards still
    triggers that guard (executors.py, handler: `gc.trigger()`), and a callback still queued can
    run after its entity is destroyed. Both raise InvalidHandle ('cannot use Destroyable because
    destruction was requested') into a Task nobody reads, which rclpy prints as 'The following
    exception was never retrieved'. stop() therefore: stops scheduling (no spin_once after the
    flag), joins the spin thread, waits for every scheduled callback (the pool; later rclpy
    releases do this inside shutdown()), and only then shuts the executor down.
    """

    def __init__(self, executor, period_s=0.05):
        self.executor = executor
        self._period_s = period_s
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self._stop.is_set():
            self.executor.spin_once(timeout_sec=self._period_s)

    def stop(self, timeout_s=2.0):
        self._stop.set()
        self.thread.join(timeout=timeout_s)
        pool = getattr(self.executor, '_executor', None)   # rclpy 3.3 MultiThreadedExecutor pool
        if pool is not None:
            pool.shutdown(wait=True)
        self.executor.shutdown(timeout_sec=timeout_s)


def _stamp(t):
    # integer nanoseconds first: rounding only the fraction could yield nanosec = 1e9, which the
    # old '% 1e9' turned into a stamp one whole second in the past
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _secs(d):
    return d.sec + d.nanosec * 1e-9


class IsolatedFakeStack:
    def __init__(self, domain_id, joint_names, start_positions, rtf=2.0, rate_hz=100.0):
        self.joint_names = list(joint_names)
        self.positions = list(start_positions)
        self.rtf = rtf
        self.received, self.cancel_requests, self.list_calls = [], 0, 0
        self.results = []
        self._stopping = False
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
        self.timer = self.node.create_timer(1.0 / rate_hz, self._tick,
                                            callback_group=MutuallyExclusiveCallbackGroup())
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.spinner = SpinThread(self.executor)

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
            if self._stopping:                 # stop() mid-goal: end it, never report success
                goal_handle.abort()
                self.results.append('aborted_by_stop')
                return FollowJointTrajectory.Result(error_code=-1, error_string='test double stop')
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
        self._stopping = True
        self.spinner.stop()
        self.node.destroy_timer(self.timer)
        self.server.destroy()
        self.node.destroy_service(self.service)
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.context)
