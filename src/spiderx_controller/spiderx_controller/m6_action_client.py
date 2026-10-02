"""M6.0 single-goal FollowJointTrajectory client behind a testable adapter (Batch B).

The session sends AT MOST ONE goal in its lifetime, and only a goal built from a trajectory
whose M6.0 preflight (m6_trajectory.preflight) passes. Owner decisions (docs/M6 plan section 14):
  - D3: three evidence channels, recorded separately as passed / failed / timed_out /
    unavailable: 'action' (accepted + successful result), 'tracking' (independent /joint_states
    comparison at 0.05 rad) and 'goal_tolerances' (explicit 0.05 rad path/goal tolerances in the
    goal message; no controller-YAML change). Action success alone is not tracking evidence.
  - D5: cancel only; the controller holds. No automatic return-to-neutral after cancellation,
    rejection, timeout, tracking failure or error. No retry. No second goal.

Batch B has NO live adapter: the session talks only to an ActionAdapter. Tests use a deterministic
test double. A live adapter for M6.0-D needs separate owner approval.

ROS message types (control_msgs, trajectory_msgs, builtin_interfaces) are imported only inside
build_goal(); the rest of this module is pure Python.
"""

from dataclasses import dataclass, field
import math

from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_trajectory as m6t

OUTCOME_SCHEMA = 'spiderx.m6.playback_outcome/1'

# action_msgs/msg/GoalStatus values (pinned against the installed message by tests)
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

# control_msgs/action/FollowJointTrajectory result error codes (pinned by tests)
ERROR_CODES = {
    0: 'SUCCESSFUL',
    -1: 'INVALID_GOAL',
    -2: 'INVALID_JOINTS',
    -3: 'OLD_HEADER_TIMESTAMP',
    -4: 'PATH_TOLERANCE_VIOLATED',
    -5: 'GOAL_TOLERANCE_VIOLATED',
}
TOLERANCE_ERRORS = (-4, -5)

PASSED, FAILED, TIMED_OUT, UNAVAILABLE = 'passed', 'failed', 'timed_out', 'unavailable'
CHANNEL_VALUES = (PASSED, FAILED, TIMED_OUT, UNAVAILABLE)

DEFAULT_SERVER_TIMEOUT_S = 10.0


class PreflightRefused(ValueError):
    """The trajectory failed preflight; no goal object was constructed."""

    def __init__(self, report):
        super().__init__('preflight refused: ' + ', '.join(report.codes))
        self.report = report


class SecondGoalForbidden(RuntimeError):
    """A PlaybackSession sends at most one goal in its lifetime."""


class ActionAdapter:
    """What the session needs from an action transport. Implementations must not retry.

    server_ready(timeout_s) -> bool
    send_goal(goal) -> handle with a boolean .accepted, or None if no answer arrived
    wait_result(handle, timeout_s) -> (status, error_code, error_string), or None on timeout
    cancel(handle) -> bool                      (cancel request only; never sends a new goal)
    joint_state_samples() -> [(t_s, {joint: position})], t_s relative to goal acceptance
    """

    def server_ready(self, timeout_s):
        raise NotImplementedError

    def send_goal(self, goal):
        raise NotImplementedError

    def wait_result(self, handle, timeout_s):
        raise NotImplementedError

    def cancel(self, handle):
        raise NotImplementedError

    def joint_state_samples(self):
        raise NotImplementedError


# ---------------------------------------------------------------- goal construction
def seconds_to_duration_fields(t):
    """(sec, nanosec) for a non-negative time in seconds, rounded to the nanosecond."""
    ns = int(round(t * 1e9))
    return divmod(ns, 1_000_000_000)


def tolerance_entries(joint_names):
    """Per-joint path/goal tolerance values placed in the goal: position = the tracking tolerance.

    velocity/acceleration 0 = 'unspecified' (controller default). The installed controller falls
    back to its (unchecked) defaults if any name is unknown or any value is illegal, so every
    name must be one of the controller's joints and every position must be > 0.
    """
    return [{'name': n, 'position': env.TRACKING_TOLERANCE_RAD, 'velocity': 0.0,
             'acceleration': 0.0} for n in joint_names]


def build_goal(trajectory, sources):
    """FollowJointTrajectory.Goal for ONE preflighted trajectory. Raises PreflightRefused.

    The preflight runs here again, so no caller can obtain a goal for a refused trajectory.
    """
    report = m6t.preflight(trajectory, sources)
    if not report.ok:
        raise PreflightRefused(report)
    from builtin_interfaces.msg import Duration, Time
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from trajectory_msgs.msg import JointTrajectoryPoint

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.header.stamp = Time(sec=0, nanosec=0)      # start on receipt
    goal.trajectory.joint_names = list(trajectory['joint_names'])
    for p in trajectory['points']:
        sec, nsec = seconds_to_duration_fields(p['time_from_start_s'])
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=[float(v) for v in p['positions']],
            velocities=[float(v) for v in p['velocities']],
            time_from_start=Duration(sec=sec, nanosec=nsec)))
    names = goal.trajectory.joint_names
    goal.path_tolerance = [JointTolerance(**t) for t in tolerance_entries(names)]
    goal.goal_tolerance = [JointTolerance(**t) for t in tolerance_entries(names)]
    sec, nsec = seconds_to_duration_fields(env.GOAL_TIME_TOLERANCE_S)
    goal.goal_time_tolerance = Duration(sec=sec, nanosec=nsec)
    return goal, report


def _dispatch_allowed(report, trajectory):
    """THE dispatch gate: a passing preflight bound to this exact trajectory content."""
    return (report.ok and isinstance(trajectory, dict)
            and report.trajectory_id == trajectory.get('trajectory_id')
            == m6t.compute_trajectory_id(trajectory))


# ---------------------------------------------------------------- tracking (D3 channel 2)
def reference_positions(trajectory, t):
    """Commanded reference at time t (s after acceptance), or None before the first waypoint.

    Between zero-velocity waypoints the installed controller's 'splines' interpolation with
    positions and velocities is a cubic Hermite: q = qa + (qb - qa)(3s^2 - 2s^3).
    After the last waypoint the reference is the last waypoint.
    """
    pts = trajectory['points']
    times = [p['time_from_start_s'] for p in pts]
    if t < times[0]:
        return None
    if t >= times[-1]:
        return list(pts[-1]['positions'])
    for a, b in zip(range(len(pts)), range(1, len(pts))):
        if times[a] <= t < times[b]:
            s = (t - times[a]) / (times[b] - times[a])
            h = 3 * s * s - 2 * s * s * s
            return [qa + (qb - qa) * h
                    for qa, qb in zip(pts[a]['positions'], pts[b]['positions'])]
    return None


def evaluate_tracking(trajectory, samples, tolerance=None):
    """D3 channel 2: observed /joint_states vs commanded reference.

    samples: [(t_s, {joint: position})]. passed iff every joint of every sample from the first
    waypoint on is within the tracking tolerance, and every segment plus the end is covered.
    """
    tol = env.TRACKING_TOLERANCE_RAD if tolerance is None else tolerance
    names = list(trajectory['joint_names'])
    times = [p['time_from_start_s'] for p in trajectory['points']]
    out = {'tolerance_rad': tol, 'samples_used': 0, 'max_error_rad': None,
           'worst_joint': None, 'worst_time_s': None, 'incomplete_samples': 0,
           'uncovered_intervals': [], 'reason': None}
    if not samples:
        out['reason'] = 'no /joint_states samples'
        return UNAVAILABLE, out
    worst = (-1.0, None, None)
    covered = [False] * len(times)        # [segment 0..n-2] + [after end]
    for t, obs in samples:
        ref = reference_positions(trajectory, t)
        if ref is None:
            continue
        if not isinstance(obs, dict) or any(n not in obs for n in names) or not all(
                isinstance(obs[n], (int, float)) and math.isfinite(obs[n]) for n in names):
            out['incomplete_samples'] += 1
            continue
        out['samples_used'] += 1
        idx = len(times) - 1 if t >= times[-1] else max(i for i in range(len(times) - 1)
                                                        if times[i] <= t)
        covered[idx] = True
        for n, r in zip(names, ref):
            e = abs(obs[n] - r)
            if e > worst[0]:
                worst = (e, n, t)
    out['uncovered_intervals'] = [i for i, c in enumerate(covered) if not c]
    if out['samples_used'] == 0:
        out['reason'] = 'no complete sample after the first waypoint'
        return UNAVAILABLE, out
    out['max_error_rad'], out['worst_joint'], out['worst_time_s'] = worst
    if out['incomplete_samples']:
        out['reason'] = f'{out["incomplete_samples"]} incomplete or non-finite samples'
        return FAILED, out
    if out['uncovered_intervals']:
        out['reason'] = f'no samples in intervals {out["uncovered_intervals"]}'
        return FAILED, out
    if worst[0] > tol:
        out['reason'] = f'{worst[1]} error {worst[0]:.4f} rad > {tol} rad at t = {worst[2]} s'
        return FAILED, out
    return PASSED, out


# ---------------------------------------------------------------- the session
@dataclass
class Outcome:
    # result: refused | rejected | succeeded | failed | timed_out | canceled | error
    result: str
    state: str
    trajectory_id: object
    goals_sent: int
    channels: dict
    hold_position: bool
    preflight: dict
    status: object = None
    error_code: object = None
    error_string: str = ''
    tracking: dict = field(default_factory=dict)
    events: list = field(default_factory=list)
    reason: str = ''

    @property
    def passed(self):
        return all(v == PASSED for v in self.channels.values())

    def to_dict(self):
        return {
            'schema': OUTCOME_SCHEMA, 'result': self.result, 'state': self.state,
            'passed': self.passed, 'trajectory_id': self.trajectory_id,
            'goals_sent': self.goals_sent, 'retries': 0, 'automatic_return_goals': 0,
            'hold_position': self.hold_position, 'channels': dict(self.channels),
            'status': self.status, 'error_code': self.error_code,
            'error_name': ERROR_CODES.get(self.error_code), 'error_string': self.error_string,
            'tracking': dict(self.tracking), 'events': list(self.events), 'reason': self.reason,
            'preflight': self.preflight, 'non_claims': list(env.NON_CLAIMS),
        }


def _channels(action=UNAVAILABLE, tracking=UNAVAILABLE, goal_tolerances=UNAVAILABLE):
    return {'action': action, 'tracking': tracking, 'goal_tolerances': goal_tolerances}


class PlaybackSession:
    """One trajectory, at most one goal, never a retry, never an automatic return goal.

    States: idle -> preflight -> refused
                              -> dispatched -> rejected | no_response
                                            -> executing -> succeeded | failed | timed_out
                                                         -> canceled_holding | error_holding
    After any terminal state the session is spent: run() raises SecondGoalForbidden.
    """

    def __init__(self, adapter, sources, server_timeout_s=DEFAULT_SERVER_TIMEOUT_S,
                 result_timeout_s=None):
        self.adapter = adapter
        self.sources = sources
        self.server_timeout_s = server_timeout_s
        self.result_timeout_s = result_timeout_s
        self.state = 'idle'
        self.goals_sent = 0
        self.events = []
        self._handle = None
        self._trajectory = None

    # -- helpers
    def _event(self, text):
        self.events.append(text)

    def result_timeout_for(self, trajectory):
        """Wall-clock result wait: 10 x duration + 30 s (the existing trajectory_client rule)."""
        if self.result_timeout_s is not None:
            return self.result_timeout_s
        return 10.0 * trajectory['points'][-1]['time_from_start_s'] + 30.0

    def _outcome(self, result, report, channels, hold, **kw):
        return Outcome(result=result, state=self.state,
                       trajectory_id=report.trajectory_id if report else None,
                       goals_sent=self.goals_sent, channels=channels, hold_position=hold,
                       preflight=report.to_dict() if report else {}, events=list(self.events),
                       **kw)

    def _dispatch(self, goal):
        if self.goals_sent:
            raise SecondGoalForbidden('this session already sent its one goal')
        self.goals_sent += 1
        self.state = 'dispatched'
        self._event('goal sent (the only goal of this session)')
        return self.adapter.send_goal(goal)

    def _cancel_and_hold(self, why):
        """D5: cancel only; the controller holds its last command. No goal is generated."""
        if self._handle is not None:
            ok = self.adapter.cancel(self._handle)
            self._event(f'cancel requested ({why}); accepted by adapter: {bool(ok)}')
        self._event('holding position: no return-to-neutral goal (D5)')

    def _handle_rejection(self, report):
        self.state = 'rejected'
        self._event('goal rejected by the action server; no retry, no further goal')
        return self._outcome('rejected', report, _channels(action=FAILED), hold=True,
                             reason='goal rejected')

    def _handle_timeout(self, report):
        self._cancel_and_hold('result timeout')
        self.state = 'timed_out'
        return self._outcome('timed_out', report,
                             _channels(action=TIMED_OUT, goal_tolerances=TIMED_OUT), hold=True,
                             reason='no result within the watchdog')

    def _handle_interrupt(self, report):
        self._cancel_and_hold('operator interrupt')
        self.state = 'canceled_holding'
        return self._outcome('canceled', report, _channels(), hold=True,
                             reason='canceled by operator; controller holds')

    def _handle_error(self, report, exc):
        self._cancel_and_hold(f'error: {type(exc).__name__}')
        self.state = 'error_holding'
        return self._outcome('error', report, _channels(), hold=True,
                             reason=f'{type(exc).__name__}: {exc}')

    # -- public
    def cancel(self):
        """External cancel (e.g. Ctrl+C handler): cancel request only, then hold."""
        if self.state in ('dispatched', 'executing'):
            self._cancel_and_hold('external cancel')
            self.state = 'canceled_holding'
        return self.state

    def run(self, trajectory):
        if self.state != 'idle':
            raise SecondGoalForbidden(f'session is {self.state}; a new session and owner '
                                      'approval are needed for any further goal')
        self.state = 'preflight'
        self._trajectory = trajectory
        report = m6t.preflight(trajectory, self.sources)
        if not _dispatch_allowed(report, trajectory):
            self.state = 'refused'
            self._event('preflight refused: ' + ', '.join(report.codes or ['identity']))
            return self._outcome('refused', report, _channels(), hold=False,
                                 reason='preflight refused; no goal constructed')
        try:
            goal, report = build_goal(trajectory, self.sources)
        except PreflightRefused as e:
            self.state = 'refused'
            return self._outcome('refused', e.report, _channels(), hold=False,
                                 reason='preflight refused at goal construction')
        try:
            if not self.adapter.server_ready(self.server_timeout_s):
                self.state = 'refused'
                self._event('action server not available; nothing sent')
                return self._outcome('refused', report, _channels(), hold=False,
                                     reason='action server unavailable')
            self._handle = self._dispatch(goal)
            if self._handle is None:
                self.state = 'no_response'
                self._event('no answer to the goal request; nothing to cancel')
                return self._outcome('timed_out', report,
                                     _channels(action=TIMED_OUT, goal_tolerances=TIMED_OUT),
                                     hold=True, reason='goal request unanswered')
            if not self._handle.accepted:
                return self._handle_rejection(report)
            self.state = 'executing'
            self._event('goal accepted')
            res = self.adapter.wait_result(self._handle, self.result_timeout_for(trajectory))
            if res is None:
                return self._handle_timeout(report)
            status, error_code, error_string = res
            samples = self.adapter.joint_state_samples()
        except KeyboardInterrupt:
            return self._handle_interrupt(report)
        except SecondGoalForbidden:
            raise
        except Exception as e:  # noqa: BLE001 - any adapter failure ends in cancel + hold
            return self._handle_error(report, e)

        action = PASSED if (status == STATUS_SUCCEEDED and error_code == 0) else FAILED
        if error_code in TOLERANCE_ERRORS:
            tolerances = FAILED
        elif status == STATUS_SUCCEEDED and error_code == 0:
            tolerances = PASSED
        else:
            tolerances = UNAVAILABLE
        tracking, tdetail = evaluate_tracking(trajectory, samples)
        channels = _channels(action=action, tracking=tracking, goal_tolerances=tolerances)
        self.state = 'succeeded' if action == PASSED else 'failed'
        self._event(f'result status {status}, error {error_code} '
                    f'({ERROR_CODES.get(error_code, "unknown")})')
        if self.state == 'failed' or tracking != PASSED:
            self._event('holding position: no return-to-neutral goal (D5)')
        result = 'succeeded' if all(v == PASSED for v in channels.values()) else 'failed'
        return self._outcome(result, report, channels, hold=result != 'succeeded',
                             status=status, error_code=error_code,
                             error_string=error_string or '', tracking=tdetail,
                             reason='' if result == 'succeeded' else 'a D3 channel did not pass')


__all__ = ['ActionAdapter', 'PlaybackSession', 'Outcome', 'PreflightRefused',
           'SecondGoalForbidden', 'build_goal', 'evaluate_tracking', 'reference_positions',
           'tolerance_entries', 'seconds_to_duration_fields', 'ERROR_CODES', 'STATUS_SUCCEEDED',
           'STATUS_CANCELED', 'STATUS_ABORTED', 'CHANNEL_VALUES']
