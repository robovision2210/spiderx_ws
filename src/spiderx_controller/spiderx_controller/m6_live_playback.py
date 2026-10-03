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

Evidence of uncertainty (corrective batch 1): the outcome's `dispatch` block distinguishes
dispatch not attempted / attempted with acceptance unknown / accepted (with the goal ID) /
rejected, whether the one cancel was attempted and answered, and whether the final goal status is
known. A transport or processing failure after dispatch is recorded, requests at most the one
cancel through the same guard as every other cancel reason, and ends the session with the final
goal status marked unknown. Closing the client never stops or cancels an accepted goal.

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


def dispatch_record(send='not_attempted', acceptance=None, goal_id=None, cancels_sent=0,
                    cancel_error=None, cancel_response=None, result=None):
    """What is known about the one goal. Pure; shared by sessions and pre-session records.

    send: not_attempted | refused_by_transport (verified before any ROS call; nothing sent)
          | attempted (the send call started but did not complete: it may have reached the
          server) | sent.
    """
    attempted = send in ('attempted', 'sent')
    if not attempted:
        accept = 'not_attempted'
    elif acceptance is None:
        accept = 'unknown'
    else:
        accept = 'accepted' if acceptance else 'rejected'
    known = result is not None
    if known:
        final = result[0]
    elif not attempted:
        final = 'not_applicable'
    else:
        final = 'rejected' if accept == 'rejected' else 'unknown'
    return {
        'send': send,
        'attempted': attempted,
        'acceptance': accept,
        'goal_id': goal_id,
        'cancel_attempted': cancels_sent > 0,
        'cancel_error': cancel_error,
        'cancel_response_known': cancel_response is not None,
        'final_result_known': known,
        'final_goal_status': final,
        'goal_may_still_be_executing': attempted and accept != 'rejected' and not known,
    }


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

    def __init__(self, transport, sources, readiness_provider, confirm_reader, latch=None,
                 checkpoint=None):
        self.transport = transport
        self.sources = sources
        self.readiness_provider = readiness_provider   # () -> rd.ReadinessResult
        self.confirm_reader = confirm_reader           # () -> str (one line)
        self.latch = latch or InterruptLatch()
        self.checkpoint = checkpoint                   # (outcome) -> None; raises = not saved
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
        self._send = 'not_attempted'
        self._acceptance = None                        # None = unknown; True / False = answered
        self._goal_id = None
        self._cancel_error = None
        self._cancel_on_accept = None
        self._supervision_errors = 0
        self._errors = []

    # ------------------------------------------------------------ helpers
    def _now(self):
        return self.transport.wall_now()

    def _event(self, text):
        self.events.append((round(self._now() - self._t0, 6), self.state, text))

    def _set(self, state, text=None):
        self.state = state
        if text:
            self._event(text)

    def _record_error(self, where, exc):
        self._errors.append({'where': where, 'type': type(exc).__name__, 'message': str(exc),
                             'state': self.state,
                             't': None if self._t0 is None else round(self._now() - self._t0, 6)})

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
        try:
            return self._run()
        except ac.SecondGoalForbidden:
            raise
        except Exception as e:  # noqa: BLE001 - recorded as evidence; never retried
            return self.outcome_after_exception(e)

    def outcome_after_exception(self, exc, where='session'):
        """A truthful outcome for an exception that escaped the session logic."""
        self._record_error(where, exc)
        if self.state not in TERMINAL:
            if self._send in ('attempted', 'sent'):
                if self.state == ACTIVE and self._result is None and \
                        self._cancel_reason is None:
                    self._request_cancel('transport_error')     # the one cancel, same guard
                self.reason = 'transport_error'
                self._set(HELD_ERROR, f'{where} failed after dispatch ({type(exc).__name__}); '
                                      'final goal status unknown; no new command')
            else:
                self._refuse('session_exception', f'{type(exc).__name__}: {exc}; nothing sent')
        return self.outcome()

    def _run(self):
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
        if self.checkpoint is not None:
            try:
                self.checkpoint(self.outcome())          # pre-send evidence, or no send at all
            except Exception as e:  # noqa: BLE001
                self._record_error('evidence_checkpoint', e)
                return self._refuse('evidence_persistence_failed',
                                    'pre-dispatch evidence could not be saved; nothing sent')
        try:
            self._dispatch(goal)
        except gf.FingerprintError as e:
            self._send = 'refused_by_transport'          # verified before any ROS call
            return self._refuse('goal_fingerprint_mismatch', str(e))
        except ac.SecondGoalForbidden:
            raise
        except Exception as e:  # noqa: BLE001 - a failed send is reported, never retried
            self._record_error('send_goal', e)
            self.reason = 'transport_error'
            self._set(HELD_ERROR, f'send failed: {type(e).__name__}: {e}; the goal may have '
                                  'reached the server; acceptance unknown; no retry')
            return self.outcome()
        return self._supervise()

    def _dispatch(self, goal):
        if self._dispatch_attempted:
            raise ac.SecondGoalForbidden('this session already used its one dispatch')
        self._dispatch_attempted = True
        self._send = 'attempted'                    # from here the goal may reach the server
        self.transport.send_goal(goal, gf.binding(self._trajectory))   # verifies fingerprint
        self._send = 'sent'
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
            self.cancels_sent += 1                  # counted as attempted before the call
            try:
                self.transport.cancel_goal()
            except Exception as e:  # noqa: BLE001 - recorded; the one cancel is never retried
                self._cancel_error = f'{type(e).__name__}: {e}'
                self._record_error('cancel_goal', e)
        self._cancel_wall = self._now()
        self._set(CANCEL_REQUESTED, f'one cancel requested ({reason}); no new goal')

    def _supervision_error(self, where, exc):
        """A failure after dispatch: at most the one cancel (same guard), never a retry."""
        self._record_error(where, exc)
        self._supervision_errors += 1
        if self._supervision_errors == 1 and self._cancel_reason is None:
            if self.state == ACTIVE:
                self._request_cancel('transport_error')
                return None                        # wait for the cancel outcome (bounded)
            if self.state == GOAL_PENDING:
                self._cancel_on_accept = 'transport_error'
                self._event(f'{where} failed while the goal is pending: cancel on acceptance')
                return None
        self.reason = 'transport_error'
        self._set(HELD_ERROR, f'{where} failed after dispatch ({type(exc).__name__}); '
                              'supervision stopped; final goal status unknown; no new command')
        return self.outcome()

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
                self._set(TIMED_OUT, 'no goal response within 10 s; acceptance unknown; no goal '
                                     'handle to cancel; final goal status unknown')
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
                    try:
                        reason = graph_violation(self.transport.graph_status())
                    except Exception as e:  # noqa: BLE001
                        done = self._supervision_error('graph_status', e)
                        if done is not None:
                            return done
                        continue
                if reason is not None:
                    self._request_cancel(reason)
            # events
            try:
                events = self.transport.poll(POLL_S)
            except Exception as e:  # noqa: BLE001
                done = self._supervision_error('poll', e)
                if done is not None:
                    return done
                continue
            for ev in events:
                try:
                    done = self._on_event(ev, monitor)
                except Exception as e:  # noqa: BLE001 - the remaining events are still read
                    done = self._supervision_error(f'event {ev[0] if ev else None}', e)
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
            self._acceptance = bool(ev[1])
            if len(ev) > 2 and ev[2]:
                self._goal_id = str(ev[2])
            if not ev[1]:
                self.reason = 'goal_rejected'
                self._set(REJECTED, 'goal rejected by the server; no retry')
                return self.outcome()
            self._set(ACTIVE, 'goal accepted')        # recorded before anything else can fail
            if self._handled_interrupts >= 1:
                self._request_cancel('operator_interrupt')
            elif self._cancel_on_accept is not None:
                self._request_cancel(self._cancel_on_accept)
            try:
                t_accept = self.transport.sim_now()
            except Exception as e:  # noqa: BLE001 - no trustworthy time base: cancel (guarded)
                self._record_error('sim_now', e)
                self._request_cancel('transport_error')
                t_accept = None
            self._t_accept_sim = ((monitor.last_stamp or 0.0) if t_accept is None
                                  else t_accept)
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
                if self._acceptance is None:
                    self._acceptance = True             # a result implies acceptance
                self._set(ACTIVE)
            if status != STATUS_SUCCEEDED or code != 0:
                self.reason = 'action_aborted' if status == STATUS_ABORTED else 'action_failed'
                self._set(ABORTED, 'controller ended the goal without success; no new goal')
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
                         'no new goal')
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
            'dispatch': dispatch_record(self._send, self._acceptance, self._goal_id,
                                        self.cancels_sent, self._cancel_error,
                                        self._cancel_response, self._result),
            'errors': [dict(e) for e in self._errors],
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
           'TERMINAL', 'REASON_STATE', 'OUTCOME_SCHEMA', 'dispatch_record']


# ==================================================================== CLI (Batch D)
# ros2 run spiderx_controller m6_live_playback --dry-run       offline goal contract, no ROS graph
# ros2 run spiderx_controller m6_live_playback --mock [...]    full state machine, in-memory mock
# ros2 run spiderx_controller m6_live_playback --live --domain-id N
#                                                    one goal; refused (exit 3) while the single
#                                                    gate m6_live_contract.LIVE_DISPATCH_ENABLED
#                                                    is False, which it is in this build
#
# There is no option to supply a trajectory, skip the confirmation, retry, repeat or force, and
# no option or environment variable that changes the gate.
# Exit codes: 0 succeeded / dry-run PASS; 1 terminal failure; 2 refused; 3 live dispatch disabled.

EXIT_OK, EXIT_FAILED, EXIT_REFUSED, EXIT_DISABLED = 0, 1, 2, 3
DEFAULT_OUT = 'log/m6d_playback'
LIVE_STATE = ('ENABLED for exactly one goal' if lc.LIVE_DISPATCH_ENABLED
              else 'HARD-DISABLED in this build')
CLI_SCOPE = (f'M6.0-D live-playback tool. Live dispatch is {LIVE_STATE}; --dry-run '
             'and --mock never touch a ROS graph. Nothing here shows motion, tracking, contact, '
             'balance or walking.')


def parse_args(argv):
    import argparse
    p = argparse.ArgumentParser(prog='m6_live_playback', description=CLI_SCOPE)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--dry-run', action='store_true',
                      help='build and fingerprint the one approved goal offline; send nothing')
    mode.add_argument('--mock', action='store_true',
                      help='run the full state machine against the in-memory mock transport; '
                           'reads the confirmation word from stdin')
    mode.add_argument('--live', action='store_true',
                      help=f'one live goal to the running simulation ({LIVE_STATE})')
    p.add_argument('--scenario', default='success',
                   help='--mock scenario: success, interrupt, tracking_error, '
                        'stale_joint_states, controller_lost, rejected')
    p.add_argument('--out', default=DEFAULT_OUT,
                   help=f'report root under the git-ignored log/ (default: {DEFAULT_OUT})')
    p.add_argument('--no-write', action='store_true', help='print only')
    p.add_argument('--domain-id', type=int, default=None,
                   help='--live only: the ROS domain id of the running simulation, typed '
                        'explicitly by the operator (never read from the environment)')
    return p.parse_args(argv)


def _write(path, data):
    import json
    import os
    if os.path.exists(path):
        raise FileExistsError(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(json.dumps(data, indent=1, sort_keys=True, allow_nan=False, default=str) + '\n')
    return path


def dry_run_report(sources):
    traj = m6t.build_trajectory(sources)
    goal, report, spec, fp = gf.build_live_goal(traj, sources)
    return {
        'schema': 'spiderx.m6d.dry_run/1',
        'mode': 'dry-run',
        'verdict': 'PASS',
        'trajectory_id': traj['trajectory_id'],
        'goal_fingerprint': fp,
        'goal_spec': spec,
        'preflight': report.to_dict(),
        'live_dispatch_enabled': lc.LIVE_DISPATCH_ENABLED,
        'limits': lc.as_dict(),
        'envelope': env.as_dict(),
        'goals_sent': 0,
        'non_claims': list(env.NON_CLAIMS),
    }


def run_mock(sources, scenario, reader):
    from spiderx_controller import m6_live_mock as mock
    if scenario not in mock.SCENARIOS:
        raise ValueError(f'unknown scenario {scenario!r}; choose from {sorted(mock.SCENARIOS)}')
    traj = m6t.build_trajectory(sources)
    fp = gf.fingerprint(gf.approved_spec(traj, sources))
    latch = InterruptLatch()
    transport = mock.FakeTransport(traj, fp, latch=latch, **mock.SCENARIOS[scenario])
    neutral = list(sources.poses[env.NEUTRAL_LABEL])
    report = mock.mock_readiness_report(sources.joint_names, neutral)
    session = LiveSession(transport, sources,
                          lambda: rd.assess(report, transport.wall_now()), reader, latch=latch)
    out = session.run()
    out['mode'] = 'mock'
    out['scenario'] = scenario
    out['mock_server_goals_received'] = len(transport.sent)
    out['mock_server_cancels_received'] = transport.cancels
    return out


TRANSPORT_CLOSE_NOTE = ('closing the client releases local ROS resources only; it does not '
                        'cancel or stop an accepted goal on the controller')


def _close_transport(transport, out):
    """Close the transport; a close failure is recorded, never allowed to hide the outcome."""
    try:
        transport.close()
        closed, error = True, None
    except Exception as e:  # noqa: BLE001
        closed, error = False, f'{type(e).__name__}: {e}'
    if isinstance(out, dict):
        out['transport'] = {'closed': closed, 'close_error': error, 'note': TRANSPORT_CLOSE_NOTE}


def _run_live(sources, transport_factory, collect_factory, reader, latch, domain_id,
              checkpoint=None):
    """The future live path. Refuses unless LIVE_DISPATCH_ENABLED (hard-disabled in this build).

    Every dependency is injected so the wiring can be audited and tested without ROS. An exception
    that escapes this function was raised before any session ran, so nothing was sent; anything
    raised while the session runs becomes part of its outcome (LiveSession.run).
    """
    if not lc.LIVE_DISPATCH_ENABLED:
        raise PermissionError(lc.LIVE_DISPATCH_DISABLED_MESSAGE)
    traj = m6t.build_trajectory(sources)
    fp = gf.fingerprint(gf.approved_spec(traj, sources))
    transport = transport_factory(fp, domain_id)
    out = None
    try:
        provide = rd.make_readiness_provider(collect_factory(transport),
                                             sources.joint_names,
                                             list(sources.poses[env.NEUTRAL_LABEL]),
                                             transport.wall_now)       # the session's clock
        out = LiveSession(transport, sources, provide, reader, latch=latch,
                          checkpoint=checkpoint).run()
        return out
    finally:
        _close_transport(transport, out)


MAX_DOMAIN_ID = 232                                  # largest valid ROS 2 domain id


def _exit_code(state):
    return EXIT_OK if state == SUCCEEDED else EXIT_REFUSED if state == REFUSED else EXIT_FAILED


def _default_transport_factory(fp, domain_id):
    from spiderx_controller import m6_live_adapter as la
    return la.RclpyLiveTransport(fp, domain_id).open()


def _default_collect_factory(transport):
    return transport.graph_collector()


class EvidenceFile:
    """One live run's evidence file: created exclusively, then rewritten only through its own fd.

    reserve() creates the file with O_CREAT | O_EXCL, so an existing file (another run's evidence)
    refuses. Every later write goes through the descriptor this run created - never a path-based
    rename or reopen - so no other file can be overwritten. Each write serializes first, then
    rewrites the file in place and fsyncs. A failed write may leave the file incomplete; the
    caller then emits the outcome to stderr and never claims it was saved.
    """

    def __init__(self, path, fd):
        self.path = path
        self._fd = fd

    @classmethod
    def reserve(cls, path):
        import os
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        return cls(path, os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644))

    def write(self, doc):
        import json
        import os
        data = (json.dumps(doc, indent=1, sort_keys=True, allow_nan=False, default=str)
                + '\n').encode()
        if self._fd is None:
            raise ValueError(f'evidence file {self.path} is closed')
        os.lseek(self._fd, 0, os.SEEK_SET)
        view = memoryview(data)
        while view:
            n = os.write(self._fd, view)
            if n <= 0:
                raise OSError(f'short write to {self.path}')
            view = view[n:]
        os.ftruncate(self._fd, len(data))
        os.fsync(self._fd)

    def checkpoint(self, outcome):
        """The pre-send record: if it is the last one saved, the goal status is UNKNOWN."""
        self.write(dict(outcome, evidence={'path': self.path, 'phase': 'pre_send'}))

    def close(self):
        import os
        fd, self._fd = self._fd, None
        if fd is not None:
            os.close(fd)


EVIDENCE_PHASES = {
    'reserved': 'written before any ROS initialization; if it is the last record, nothing was '
                'sent',
    'pre_send': 'every gate passed except the final latch and freshness checks; a send may have '
                'followed; if it is the last record, the goal status is UNKNOWN',
    'final': 'the complete outcome of the run',
}


def _reserved_record(args, utc_stamp, path):
    return {'schema': OUTCOME_SCHEMA, 'mode': 'live', 'state': 'NOT_DISPATCHED',
            'terminal': False, 'passed': False, 'reason': None, 'run_utc': utc_stamp,
            'domain_id': args.domain_id, 'goals_sent': 0, 'cancels_sent': 0,
            'dispatch': dispatch_record(), 'errors': [],
            'evidence': {'path': path, 'phase': 'reserved'}, 'evidence_phases': EVIDENCE_PHASES,
            'limits': lc.as_dict(), 'non_claims': list(env.NON_CLAIMS)}


def _setup_failure_outcome(exc):
    """Outcome when the run failed before any session ran (so nothing was sent)."""
    return {'schema': OUTCOME_SCHEMA, 'state': REFUSED, 'terminal': True, 'passed': False,
            'reason': 'session_setup_failed', 'goals_sent': 0, 'cancels_sent': 0,
            'retries': 0, 'automatic_return_goals': 0, 'dispatch': dispatch_record(),
            'errors': [{'where': 'setup', 'type': type(exc).__name__, 'message': str(exc)}],
            'limits': lc.as_dict(), 'non_claims': list(env.NON_CLAIMS)}


def _fallback_json(data):
    import json
    try:
        return json.dumps(data, indent=1, sort_keys=True, allow_nan=True, default=repr)
    except Exception:  # noqa: BLE001 - the fallback itself must not fail
        return repr(data)


def _finish_live(evidence, data):
    """Persist the final record. On failure: stderr fallback, exit 1, never 'saved', no retry."""
    import sys
    dispatch = data.get('dispatch') or {}
    print(f'Live: final state {data.get("state")}, reason {data.get("reason")}, goals sent '
          f'{data.get("goals_sent")}, cancels sent {data.get("cancels_sent")}; no retry, '
          'no second goal')
    if dispatch.get('goal_may_still_be_executing'):
        print('WARNING: the final goal status is UNKNOWN; the controller may still be executing '
              f'the goal ({TRANSPORT_CLOSE_NOTE}).')
    data['evidence'] = {'path': evidence.path, 'phase': 'final'}
    try:
        evidence.write(data)
        saved, error = True, None
    except Exception as e:  # noqa: BLE001
        saved, error = False, f'{type(e).__name__}: {e}'
    finally:
        try:
            evidence.close()
        except Exception:  # noqa: BLE001 - nothing more can be done about a close failure
            pass
    if saved:
        print(f'Report: {evidence.path}')
        return _exit_code(data.get('state'))
    data['evidence'] = {'path': evidence.path, 'phase': 'final', 'saved': False,
                        'persistence_error': error}
    print(f'EVIDENCE NOT SAVED: writing {evidence.path} failed ({error}). The outcome below '
          '(stderr) is the only complete record. The goal is never retried.', file=sys.stderr)
    print(_fallback_json(data), file=sys.stderr)
    return EXIT_FAILED


def _live_main(args, sources, reader, transport_factory=None, collect_factory=None,
               latch=None, utc_stamp=None, evidence_factory=None):
    """--live after the gate: one goal through _run_live, persisted evidence, never an overwrite.

    Reached only when LIVE_DISPATCH_ENABLED is True. Dependencies are injectable so this wiring is
    tested without ROS; the defaults are the rclpy transport and its read-only graph collector.
    Before any ROS initialization the evidence file is reserved exclusively and a
    NOT_DISPATCHED record is saved; if either fails, nothing starts.
    """
    import datetime
    import sys
    if not lc.LIVE_DISPATCH_ENABLED:
        print(f'REFUSED: {lc.LIVE_DISPATCH_DISABLED_MESSAGE}')
        return EXIT_DISABLED
    if args.no_write:
        print('REFUSED: --live always records evidence; --no-write is not allowed with --live')
        return EXIT_REFUSED
    if args.domain_id is None or not 0 <= args.domain_id <= MAX_DOMAIN_ID:
        print(f'REFUSED: --live needs an explicit --domain-id in 0..{MAX_DOMAIN_ID}')
        return EXIT_REFUSED
    if utc_stamp is None:
        utc_stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    target = f'{args.out}/live/{utc_stamp}/live_outcome.json'
    try:
        evidence = (evidence_factory or EvidenceFile.reserve)(target)
    except FileExistsError:
        print(f'REFUSED: {target} already exists (evidence is never overwritten); nothing '
              'started')
        return EXIT_REFUSED
    except OSError as e:
        print(f'REFUSED: cannot reserve live evidence at {target} ({type(e).__name__}: {e}); '
              'nothing started')
        return EXIT_REFUSED
    try:
        evidence.write(_reserved_record(args, utc_stamp, target))
    except Exception as e:  # noqa: BLE001
        evidence.close()
        print(f'REFUSED: the initial NOT_DISPATCHED evidence could not be saved '
              f'({type(e).__name__}: {e}); nothing started')
        return EXIT_REFUSED
    if sources is None:
        sources = m6t.load_sources()
    if reader is None:
        def reader():              # asked only after readiness #1 passed (not incompatible)
            print(f'Readiness passed (compatible or warning). Type {lc.CONFIRMATION_WORD} to send '
                  'ONE goal (neutral -> crouch_10mm -> neutral) to the running simulation on ROS '
                  f'domain {args.domain_id}; anything else refuses:', flush=True)
            return sys.stdin.readline()
    latch = latch or InterruptLatch()
    latch.install()
    try:
        data = _run_live(sources, transport_factory or _default_transport_factory,
                         collect_factory or _default_collect_factory, reader, latch,
                         args.domain_id, checkpoint=evidence.checkpoint)
    except Exception as e:  # noqa: BLE001 - raised before any session ran: nothing was sent
        data = _setup_failure_outcome(e)
    finally:
        latch.uninstall()
    data.update(mode='live', domain_id=args.domain_id, run_utc=utc_stamp,
                evidence_phases=EVIDENCE_PHASES)
    return _finish_live(evidence, data)


def main(argv=None, sources=None, reader=None):
    import sys
    argv = list(argv if argv is not None else sys.argv)
    args = parse_args(argv[1:])
    print('SpiderX M6.0-D live-playback tool')
    print(CLI_SCOPE)
    if args.live:
        # the single gate, checked before any configuration, ROS import or input
        if not lc.LIVE_DISPATCH_ENABLED:
            print(f'REFUSED: {lc.LIVE_DISPATCH_DISABLED_MESSAGE}')
            return EXIT_DISABLED
        return _live_main(args, sources, reader)
    if args.domain_id is not None:
        print('REFUSED: --domain-id is only valid with --live')
        return EXIT_REFUSED
    if sources is None:
        sources = m6t.load_sources()
    if args.dry_run:
        try:
            data = dry_run_report(sources)
        except (m6t.TrajectoryBuildError, gf.FingerprintError, ac.PreflightRefused) as e:
            print(f'REFUSED: {e}')
            return EXIT_REFUSED
        print(f'Dry run PASS: trajectory {data["trajectory_id"]}, goal fingerprint '
              f'{data["goal_fingerprint"]}; goal velocity tolerance '
              f'{lc.GOAL_VELOCITY_TOLERANCE_RAD_S} rad/s; nothing sent')
        target = f'{args.out}/dry_run/{data["trajectory_id"]}/dry_run_report.json'
        code = EXIT_OK
    else:
        if reader is None:
            print(f'Type {lc.CONFIRMATION_WORD} to continue the MOCK run (nothing is sent '
                  'to any ROS graph):')
            reader = sys.stdin.readline
        try:
            data = run_mock(sources, args.scenario, reader)
        except ValueError as e:
            print(f'REFUSED: {e}')
            return EXIT_REFUSED
        print(f'Mock {args.scenario}: final state {data["state"]}, reason {data["reason"]}, '
              f'goals at mock server {data["mock_server_goals_received"]}, cancels '
              f'{data["mock_server_cancels_received"]}')
        target = f'{args.out}/mock/{args.scenario}/mock_report.json'
        code = _exit_code(data['state'])
    if not args.no_write:
        try:
            print(f'Report: {_write(target, data)}')
        except FileExistsError:
            print(f'REFUSED: {target} already exists (reports are never overwritten)')
            return EXIT_REFUSED
    return code
