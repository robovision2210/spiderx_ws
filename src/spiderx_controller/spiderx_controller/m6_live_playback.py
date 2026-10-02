"""M6.0-D single-goal live-playback state machine (Batch B). Inert unless explicitly driven.

LiveSession sends AT MOST ONE goal in its lifetime: the one approved, fingerprinted
neutral -> crouch_10mm -> neutral goal, and only after
  1. the offline M6.0 preflight passes and the approved spec/fingerprint is computed (PREFLIGHTED);
  2. a same-process readiness result is fresh (<= 10 s) and not `incompatible` (D3, D4);
  3. the operator typed the exact confirmation word (D10);
  4. readiness is re-observed and still permitted after the confirmation;
  5. the action server is ready, and the transport verifies the goal fingerprint (adapter gate).
Everything after dispatch is cancel-only: one cancel at most, never a retry, a preemption, an
automatic return-to-neutral or a second goal (D2, D5-D9, D11).

The transport is injected (LiveTransport). Tests use a deterministic fake; the rclpy transport is
m6_live_adapter (Batch C). Live dispatch is hard-disabled at the CLI (m6_live_contract).

This is a poll-based state machine rather than the blocking m6_action_client.PlaybackSession,
because the live envelope needs per-second controller checks, stream monitoring and an
in-flight tracking abort while the goal runs. It reuses the M6.0 preflight, goal builder,
tracking reference, error-code tables and SecondGoalForbidden.
"""

import math

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m6_trajectory as m6t

OUTCOME_SCHEMA = 'spiderx.m6d.live_outcome/1'
POLL_S = 0.05

# ---- states ----
NEW = 'NEW'
PREFLIGHTED = 'PREFLIGHTED'
CONFIRMED = 'CONFIRMED'
DISPATCHING = 'DISPATCHING'
GOAL_PENDING = 'GOAL_PENDING'
ACTIVE = 'ACTIVE'
CANCEL_REQUESTED = 'CANCEL_REQUESTED'
CANCEL_CONFIRMED = 'CANCEL_CONFIRMED'
SUCCEEDED = 'SUCCEEDED'
REJECTED = 'REJECTED'
ABORTED = 'ABORTED'
TIMED_OUT = 'TIMED_OUT'
TRACKING_FAILED = 'TRACKING_FAILED'
READINESS_LOST = 'READINESS_LOST'
CANCEL_UNCONFIRMED = 'CANCEL_UNCONFIRMED'
HELD_ERROR = 'HELD_ERROR'
REFUSED = 'REFUSED'
TERMINAL = frozenset({CANCEL_CONFIRMED, SUCCEEDED, REJECTED, ABORTED, TIMED_OUT, TRACKING_FAILED,
                      READINESS_LOST, CANCEL_UNCONFIRMED, HELD_ERROR, REFUSED})

# cancel reasons -> terminal state when the cancel is resolved (confirmed or not)
REASON_STATE = {
    'operator_interrupt': CANCEL_CONFIRMED,       # CANCEL_UNCONFIRMED if not confirmed
    'tracking_error': TRACKING_FAILED,
    'sample_gap': TRACKING_FAILED,
    'joint_states_stale': READINESS_LOST,
    'sim_time_stalled': READINESS_LOST,
    'extra_joint_state_publisher': READINESS_LOST,
    'controller_lost': HELD_ERROR,
    'result_watchdog': TIMED_OUT,
    'transport_error': HELD_ERROR,
}

# action GoalStatus values (pinned in m6_action_client tests)
STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = (ac.STATUS_SUCCEEDED, ac.STATUS_CANCELED,
                                                     ac.STATUS_ABORTED)
# action_msgs/srv/CancelGoal return codes
CANCEL_ERROR_NONE = 0


class LiveTransport:
    """What LiveSession needs. Implementations never retry and never send a goal on their own.

    server_ready(timeout_s) -> bool
    send_goal(goal, binding) -> None    asynchronous; MUST verify the goal fingerprint and refuse
                                        (raise) a second call or a non-approved goal before any
                                        ROS call
    cancel_goal() -> None               asynchronous; at most once
    poll(timeout_s) -> [event]          ('goal_response', accepted)
                                        ('feedback', {'stamp': s, 'desired_time_from_start': s,
                                                      'error': [..]})
                                        ('result', status, error_code, error_string)
                                        ('cancel_response', return_code)
                                        ('joint_state', wall_s, stamp_s, names, positions)
    graph_status() -> {'action_server_present': bool, 'joint_state_publishers': int}
    wall_now() -> monotonic seconds;   sim_now() -> sim seconds or None
    """


class InterruptLatch:
    """Counts operator interrupts (SIGINT/SIGTERM). trigger() is what a signal handler calls."""

    def __init__(self):
        self.count = 0

    def trigger(self, *_):
        self.count += 1

    def install(self):                      # live CLI only (hard-disabled in this build)
        import signal
        self._previous = {s: signal.signal(s, self.trigger)
                          for s in (signal.SIGINT, signal.SIGTERM)}

    def uninstall(self):
        import signal
        for s, h in getattr(self, '_previous', {}).items():
            signal.signal(s, h)


class StreamMonitor:
    """D6/D7 decisions on the live /joint_states stream. Pure; times are injected."""

    def __init__(self, start_wall):
        self.last_msg_wall = start_wall
        self.last_stamp = None
        self.last_stamp_change_wall = start_wall
        self.gap = None

    def on_sample(self, wall, stamp):
        if self.last_stamp is not None and stamp - self.last_stamp > lc.SAMPLE_GAP_S + 1e-9:
            self.gap = (self.last_stamp, stamp)
        if self.last_stamp is None or stamp > self.last_stamp:
            self.last_stamp_change_wall = wall
        self.last_stamp = stamp if self.last_stamp is None else max(self.last_stamp, stamp)
        self.last_msg_wall = wall

    def check(self, now_wall):
        """The first violated rule as a cancel reason, or None."""
        if self.gap is not None:
            return 'sample_gap'
        if now_wall - self.last_msg_wall > lc.JOINT_STATES_STALE_S:
            return 'joint_states_stale'
        if now_wall - self.last_stamp_change_wall > lc.SIM_STALL_S:
            return 'sim_time_stalled'
        return None


def graph_violation(status):
    if not status.get('action_server_present', False):
        return 'controller_lost'
    if status.get('joint_state_publishers', 0) > 1:
        return 'extra_joint_state_publisher'
    return None


class LiveSession:
    """One approved goal, at most once. Use run(); it returns a structured outcome dict."""

    def __init__(self, transport, sources, readiness_provider, confirm_reader, latch=None):
        self.transport = transport
        self.sources = sources
        self.readiness_provider = readiness_provider   # () -> rd.ReadinessResult
        self.confirm_reader = confirm_reader           # () -> str (one line)
        self.latch = latch or InterruptLatch()
        self.state = NEW
        self.goals_sent = 0
        self.cancels_sent = 0
        self.events = []
        self.reason = None
        self._t0 = None
        self._trajectory = None
        self._spec = None
        self._fingerprint = None
        self._readiness = []
        self._samples = []
        self._feedback_offsets = []
        self._result = None
        self._cancel_response = None
        self._cancel_reason = None
        self._t_accept_sim = None
        self._max_inflight_error = None
        self._handled_interrupts = 0
        self._dispatch_attempted = False

    # ------------------------------------------------------------ helpers
    def _now(self):
        return self.transport.wall_now()

    def _event(self, text):
        self.events.append((round(self._now() - self._t0, 6), self.state, text))

    def _set(self, state, text=None):
        self.state = state
        if text:
            self._event(text)

    def _refuse(self, code, text):
        self.reason = code
        self._set(REFUSED, f'refused: {code}: {text}')
        return self.outcome()

    def _readiness_ok(self, label):
        result = self.readiness_provider()
        now = self._now()
        entry = result.to_dict(now) if isinstance(result, rd.ReadinessResult) else None
        self._readiness.append((label, entry))
        return rd.dispatch_permitted(result, now)

    # ------------------------------------------------------------ public
    def run(self):
        if self.state != NEW:
            raise ac.SecondGoalForbidden(f'session is {self.state}; a new owner approval and '
                                         'a new session are required for any further goal')
        self._t0 = self._now()
        self._event('session start')
        # 1. offline preflight + approved spec (no goal object yet)
        try:
            self._trajectory = m6t.build_trajectory(self.sources)
            self._spec = gf.approved_spec(self._trajectory, self.sources)
            self._fingerprint = gf.fingerprint(self._spec)
        except (m6t.TrajectoryBuildError, gf.FingerprintError) as e:
            return self._refuse('preflight_refused', str(e))
        self._set(PREFLIGHTED, f'preflighted trajectory {self._trajectory["trajectory_id"]}, '
                               f'fingerprint {self._fingerprint[:16]}')
        # 2. readiness (same process, fresh, not incompatible)
        ok, code = self._readiness_ok('before_confirmation')
        if not ok:
            return self._refuse(code, 'readiness gate before confirmation')
        if self.latch.count:
            return self._refuse('operator_interrupt', 'interrupted before confirmation')
        # 3. typed confirmation
        if not lc.parse_confirmation(self.confirm_reader):
            return self._refuse('confirmation_refused', 'confirmation word not typed exactly')
        if self.latch.count:
            return self._refuse('operator_interrupt', 'interrupted at confirmation')
        self._set(CONFIRMED, 'operator confirmation accepted')
        # 4. readiness re-observed after confirmation
        ok, code = self._readiness_ok('after_confirmation')
        if not ok:
            return self._refuse(code, 'readiness gate after confirmation')
        # 5. goal construction + server + dispatch (transport verifies the fingerprint)
        try:
            goal, _, _, fp = gf.build_live_goal(self._trajectory, self.sources)
        except (gf.FingerprintError, ac.PreflightRefused) as e:
            return self._refuse('goal_construction_refused', str(e))
        if fp != self._fingerprint:
            return self._refuse('goal_fingerprint_mismatch', 'goal differs from preflighted spec')
        if not self.transport.server_ready(lc.SERVER_WAIT_S):
            return self._refuse('action_server_unavailable', 'server not ready within 10 s')
        if self.latch.count:
            return self._refuse('operator_interrupt', 'interrupted before dispatch')
        self._set(DISPATCHING, 'dispatching the one approved goal')
        try:
            self._dispatch(goal)
        except gf.FingerprintError as e:
            return self._refuse('goal_fingerprint_mismatch', str(e))
        except ac.SecondGoalForbidden:
            raise
        except Exception as e:  # noqa: BLE001 - a failed send is reported, never retried
            self.reason = 'transport_error'
            self._set(HELD_ERROR, f'send failed: {type(e).__name__}: {e}')
            return self.outcome()
        return self._supervise()

    def _dispatch(self, goal):
        if self._dispatch_attempted:
            raise ac.SecondGoalForbidden('this session already used its one dispatch')
        self._dispatch_attempted = True
        self.transport.send_goal(goal, gf.binding(self._trajectory))   # verifies fingerprint
        self.goals_sent += 1
        self._dispatch_wall = self._now()
        self._set(GOAL_PENDING, 'goal sent (the only goal of this session)')

    # ------------------------------------------------------------ supervision
    def _request_cancel(self, reason):
        if self._cancel_reason is not None:
            return
        self._cancel_reason = reason
        self.reason = reason
        if self.cancels_sent == 0:
            self.cancels_sent += 1
            self.transport.cancel_goal()
        self._cancel_wall = self._now()
        self._set(CANCEL_REQUESTED, f'one cancel requested ({reason}); holding, no new goal')

    def _ref_time(self, stamp):
        t0 = self._t_accept_sim
        est = self._start_estimate()
        if est is not None and abs(est - t0) > lc.FEEDBACK_OFFSET_LIMIT_S:
            t0 = est
        return stamp - t0

    def _start_estimate(self):
        if not self._feedback_offsets:
            return None
        vals = sorted(self._feedback_offsets)
        return vals[len(vals) // 2]

    def _inflight_tracking(self, stamp, names, positions):
        if self._t_accept_sim is None:
            return None
        ref = ac.reference_positions(self._trajectory, self._ref_time(stamp))
        if ref is None:
            return None
        pos = dict(zip(names, positions))
        errs = []
        for n, r in zip(self._trajectory['joint_names'], ref):
            v = pos.get(n)
            if v is None or not math.isfinite(v):
                return 'tracking_error'
            errs.append(abs(v - r))
        worst = max(errs)
        self._max_inflight_error = max(worst, self._max_inflight_error or 0.0)
        return 'tracking_error' if worst > lc.CLIENT_TRACKING_ABORT_RAD else None

    def _supervise(self):
        monitor = StreamMonitor(self._now())
        next_check = self._now()
        settle_until_stamp = None
        while True:
            now = self._now()
            # interrupts: first -> one cancel; second -> stop waiting
            if self.latch.count >= 2 and self._handled_interrupts < 2:
                self._handled_interrupts = 2
                self.reason = self._cancel_reason or 'operator_interrupt'
                self._set(CANCEL_UNCONFIRMED, 'second interrupt: stopped waiting; '
                                              'no further command')
                return self.outcome()
            if self.latch.count >= 1 and self._handled_interrupts < 1:
                self._handled_interrupts = 1
                if self.state == ACTIVE:
                    self._request_cancel('operator_interrupt')
                else:
                    self._event('interrupt while goal pending: cancel on acceptance')
            # timers
            pending_for = now - self._dispatch_wall
            if self.state == GOAL_PENDING and pending_for > lc.GOAL_RESPONSE_TIMEOUT_S:
                self.reason = 'goal_response_timeout'
                self._set(TIMED_OUT, 'no goal response within 10 s; nothing to cancel')
                return self.outcome()
            if self.state in (ACTIVE, GOAL_PENDING) and \
                    now - self._dispatch_wall > lc.RESULT_WATCHDOG_S:
                self._request_cancel('result_watchdog')
            if self.state == CANCEL_REQUESTED:
                waited = now - self._cancel_wall
                if self._cancel_response is None and waited > lc.CANCEL_RESPONSE_TIMEOUT_S:
                    return self._finish_cancel(confirmed=False, why='no cancel response in 5 s')
                if self._cancel_response is not None and waited > (
                        lc.CANCEL_RESPONSE_TIMEOUT_S + lc.FINAL_STATUS_AFTER_CANCEL_S):
                    return self._finish_cancel(confirmed=False, why='no final status in 5 s')
            if self.state == ACTIVE and self._result is None:
                reason = monitor.check(now)
                if reason is None and now >= next_check:
                    next_check = now + lc.CONTROLLER_CHECK_PERIOD_S
                    reason = graph_violation(self.transport.graph_status())
                if reason is not None:
                    self._request_cancel(reason)
            # events
            try:
                events = self.transport.poll(POLL_S)
            except Exception as e:  # noqa: BLE001
                if self.state == ACTIVE:
                    self._request_cancel('transport_error')
                    continue
                self.reason = 'transport_error'
                self._set(HELD_ERROR, f'transport error: {type(e).__name__}: {e}')
                return self.outcome()
            for ev in events:
                done = self._on_event(ev, monitor)
                if done is not None:
                    return done
            # post-result settle (sim time), then evaluate
            if self._result is not None and self.state == ACTIVE:
                last = monitor.last_stamp
                if settle_until_stamp is None:
                    settle_until_stamp = (last or 0.0) + lc.POST_RESULT_SETTLE_S
                if last is not None and last >= settle_until_stamp:
                    return self._finish_result()
                settle_limit = lc.POST_RESULT_SETTLE_S + lc.FINAL_STATUS_AFTER_CANCEL_S
                if now - self._result_wall > settle_limit:
                    return self._finish_result()

    def _on_event(self, ev, monitor):
        kind = ev[0]
        if kind == 'goal_response':
            if self.state != GOAL_PENDING:
                return None
            if not ev[1]:
                self.reason = 'goal_rejected'
                self._set(REJECTED, 'goal rejected by the server; no retry')
                return self.outcome()
            self._t_accept_sim = self.transport.sim_now()
            if self._t_accept_sim is None:
                self._t_accept_sim = monitor.last_stamp or 0.0
            self._set(ACTIVE, 'goal accepted')
            if self._handled_interrupts >= 1:
                self._request_cancel('operator_interrupt')
        elif kind == 'feedback':
            fb = ev[1]
            try:
                self._feedback_offsets.append(float(fb['stamp']) -
                                              float(fb['desired_time_from_start']))
            except (KeyError, TypeError, ValueError):
                pass
        elif kind == 'joint_state':
            _, wall, stamp, names, positions = ev
            monitor.on_sample(wall, stamp)
            if self._t_accept_sim is not None:
                self._samples.append((stamp, dict(zip(names, positions))))
            if self.state == ACTIVE and self._result is None:
                reason = self._inflight_tracking(stamp, names, positions)
                if reason:
                    self._request_cancel(reason)
        elif kind == 'cancel_response':
            self._cancel_response = ev[1]
            self._event(f'cancel response {ev[1]}')
            if self.state == CANCEL_REQUESTED and ev[1] != CANCEL_ERROR_NONE:
                return self._finish_cancel(confirmed=False, why=f'cancel refused ({ev[1]})')
        elif kind == 'result':
            _, status, code, text = ev
            self._result = (status, code, text or '')
            self._result_wall = self._now()
            self._event(f'result status {status}, error {code} '
                        f'({ac.ERROR_CODES.get(code, "unknown")})')
            if self.state == CANCEL_REQUESTED:
                return self._finish_cancel(confirmed=(status == STATUS_CANCELED),
                                           why=f'final status {status}')
            if self.state == GOAL_PENDING:
                self._set(ACTIVE)
            if status != STATUS_SUCCEEDED or code != 0:
                self.reason = 'action_aborted' if status == STATUS_ABORTED else 'action_failed'
                self._set(ABORTED, 'controller ended the goal without success; holding')
                return self.outcome()
        return None

    def _finish_cancel(self, confirmed, why):
        reason = self._cancel_reason
        if reason == 'operator_interrupt':
            state = CANCEL_CONFIRMED if confirmed else CANCEL_UNCONFIRMED
        else:
            state = REASON_STATE.get(reason, HELD_ERROR)
        self._cancel_confirmed = confirmed
        self._set(state, f'cancel {"confirmed" if confirmed else "unconfirmed"} ({why}); '
                         'controller holds; no new goal')
        return self.outcome()

    def _finish_result(self):
        status, code, _ = self._result
        tracking, detail = self._tracking()
        self._tracking_detail = detail
        if tracking == ac.PASSED:
            self.reason = None
            self._set(SUCCEEDED, 'result succeeded and independent tracking passed')
        else:
            self.reason = 'tracking_' + tracking
            self._set(TRACKING_FAILED, f'independent tracking {tracking}')
        return self.outcome()

    def _tracking(self):
        if self._t_accept_sim is None:
            return ac.UNAVAILABLE, {'reason': 'goal never accepted'}
        shifted = [(self._ref_time(stamp), obs) for stamp, obs in self._samples]
        return ac.evaluate_tracking(self._trajectory, shifted)

    # ------------------------------------------------------------ report
    def channels(self):
        status, code = (self._result or (None, None, None))[:2]
        action = ac.UNAVAILABLE
        tol = ac.UNAVAILABLE
        if self._result is not None:
            action = ac.PASSED if (status == STATUS_SUCCEEDED and code == 0) else ac.FAILED
            tol = (ac.FAILED if code in ac.TOLERANCE_ERRORS
                   else ac.PASSED if action == ac.PASSED else ac.UNAVAILABLE)
        elif self.state == TIMED_OUT:
            action = tol = ac.TIMED_OUT
        tracking = ac.UNAVAILABLE
        if self.state in (SUCCEEDED,):
            tracking = ac.PASSED
        elif self.state == TRACKING_FAILED:
            tracking = ac.FAILED
        return {'action': action, 'tracking': tracking, 'goal_tolerances': tol}

    def outcome(self):
        status, code, text = self._result or (None, None, '')
        est = self._start_estimate()
        return {
            'schema': OUTCOME_SCHEMA,
            'state': self.state,
            'terminal': self.state in TERMINAL,
            'passed': self.state == SUCCEEDED,
            'reason': self.reason,
            'trajectory_id': (self._trajectory or {}).get('trajectory_id'),
            'goal_fingerprint': self._fingerprint,
            'goals_sent': self.goals_sent,
            'cancels_sent': self.cancels_sent,
            'cancel_reason': self._cancel_reason,
            'cancel_response': self._cancel_response,
            'retries': 0,
            'automatic_return_goals': 0,
            'channels': self.channels(),
            'result': {'status': status, 'error_code': code,
                       'error_name': ac.ERROR_CODES.get(code), 'error_string': text},
            'readiness': [{'when': w, 'result': r} for w, r in self._readiness],
            'tracking': {
                'max_inflight_error_rad': self._max_inflight_error,
                'samples': len(self._samples),
                'accept_sim_s': self._t_accept_sim,
                'feedback_start_estimate_s': est,
                'feedback_offset_s': (None if est is None or self._t_accept_sim is None
                                      else round(est - self._t_accept_sim, 6)),
                'detail': getattr(self, '_tracking_detail', {}),
            },
            'events': [list(e) for e in self.events],
            'limits': lc.as_dict(),
            'envelope': env.as_dict(),
            'non_claims': list(env.NON_CLAIMS),
        }


__all__ = ['LiveSession', 'LiveTransport', 'InterruptLatch', 'StreamMonitor', 'graph_violation',
           'TERMINAL', 'REASON_STATE', 'OUTCOME_SCHEMA']
