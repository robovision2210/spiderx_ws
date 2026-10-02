"""Deterministic fake LiveTransport for M6.0-D tests (no ROS, no graph, simulated clocks).

It models a FollowJointTrajectory server and a /joint_states stream on a fake wall clock that
advances only inside poll(). Every goal and cancel it receives is recorded, so tests can prove
what was (not) dispatched.
"""

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_playback as lp


class FakeTransport(lp.LiveTransport):
    def __init__(self, trajectory, expected_fp, *, latch=None, ready=True, respond=True,
                 accept=True, response_latency=0.1, result='success', result_at=None,
                 result_code=0, rtf=1.0, js_rate=50.0, js_offset=0.0, error_after=None,
                 error_value=0.0, js_stop_at=None, stamp_freeze_at=None, gap_at=None,
                 extra_pub_at=None, server_lost_at=None, cancel_response=0,
                 cancel_latency=0.1, cancel_final=True, interrupts_at=(), feedback_offset=0.0,
                 verify=True, single_use=True, poll_raises_at=None, start_wall=1000.0):
        self.traj = trajectory
        self.expected_fp = expected_fp
        self.latch = latch
        self.ready = ready
        self.respond, self.accept, self.response_latency = respond, accept, response_latency
        self.result_kind, self.result_at, self.result_code = result, result_at, result_code
        self.rtf, self.js_period = rtf, 1.0 / js_rate
        self.js_offset, self.error_after, self.error_value = js_offset, error_after, error_value
        self.js_stop_at, self.stamp_freeze_at, self.gap_at = js_stop_at, stamp_freeze_at, gap_at
        self.extra_pub_at, self.server_lost_at = extra_pub_at, server_lost_at
        self.cancel_response, self.cancel_latency = cancel_response, cancel_latency
        self.cancel_final = cancel_final
        self.interrupts_at = sorted(interrupts_at)
        self.feedback_offset = feedback_offset
        self.verify, self.single_use = verify, single_use
        self.poll_raises_at = poll_raises_at
        self.t = start_wall
        self.t_start = start_wall
        self.sent = []            # goals that reached the "server"
        self.cancels = 0
        self.calls = []
        self.queue = []           # (time, event)
        self.ctrl_start_sim = None
        self.held_at = None
        self.next_js = start_wall
        self.next_fb = None
        self.finished = False

    # ---------------------------------------------------------------- clocks
    def wall_now(self):
        return self.t

    def _rel(self):
        return self.t - self.t_start

    def _stamp(self, wall):
        rel = wall - self.t_start
        if self.stamp_freeze_at is not None and rel >= self.stamp_freeze_at:
            rel = self.stamp_freeze_at
        return 100.0 + rel * self.rtf

    def sim_now(self):
        return self._stamp(self.t)

    # ---------------------------------------------------------------- server
    def server_ready(self, timeout_s):
        self.calls.append(('server_ready', timeout_s))
        return self.ready

    def send_goal(self, goal, binding):
        self.calls.append(('send_goal',))
        if self.single_use and self.sent:
            raise ac.SecondGoalForbidden('transport is single-use')
        if self.verify:
            gf.verify_goal(goal, binding, self.expected_fp)
        self.sent.append(goal)
        if self.respond:
            self.queue.append((self.t + self.response_latency, ('goal_response', self.accept)))
            if self.accept:
                start = self._stamp(self.t + self.response_latency)
                self.ctrl_start_sim = start
                end_wall = self.t + self.response_latency + 9.0 / self.rtf
                if self.result_kind is not None:
                    at = (end_wall + 0.05 if self.result_at is None
                          else self.t_start + self.result_at)
                    status = {'success': ac.STATUS_SUCCEEDED, 'abort': ac.STATUS_ABORTED,
                              'canceled': ac.STATUS_CANCELED}[self.result_kind]
                    self.queue.append((at, ('result', status, self.result_code, '')))
                self.next_fb = self.t + self.response_latency

    def cancel_goal(self):
        self.calls.append(('cancel_goal',))
        self.cancels += 1
        if self.cancel_response is not None:
            self.queue.append((self.t + self.cancel_latency, ('cancel_response',
                                                              self.cancel_response)))
            if self.cancel_response == 0 and self.cancel_final:
                # drop any pending result; the goal ends CANCELED and the robot holds
                self.queue = [(t, e) for t, e in self.queue if e[0] != 'result']
                self.queue.append((self.t + 2 * self.cancel_latency,
                                   ('result', ac.STATUS_CANCELED, 0, 'canceled')))
                self.held_at = self.t

    def graph_status(self):
        rel = self._rel()
        present = self.server_lost_at is None or rel < self.server_lost_at
        pubs = 2 if (self.extra_pub_at is not None and rel >= self.extra_pub_at) else 1
        return {'action_server_present': present, 'joint_state_publishers': pubs}

    # ---------------------------------------------------------------- stream
    def _positions(self, wall):
        stamp = self._stamp(wall)
        if self.held_at is not None:
            stamp = self._stamp(self.held_at)
        ref = None
        if self.ctrl_start_sim is not None:
            ref = ac.reference_positions(self.traj, stamp - self.ctrl_start_sim)
        if ref is None:
            ref = list(self.traj['points'][0]['positions'])
        off = self.js_offset
        if self.error_after is not None and self.ctrl_start_sim is not None and \
                stamp - self.ctrl_start_sim >= self.error_after:
            off += self.error_value
        return [q + off for q in ref]

    def poll(self, timeout_s):
        if self.poll_raises_at is not None and self._rel() >= self.poll_raises_at:
            raise RuntimeError('transport failure')
        end = self.t + timeout_s
        events = []
        while self.next_js <= end:
            rel = self.next_js - self.t_start
            skip = (self.js_stop_at is not None and rel >= self.js_stop_at) or (
                self.gap_at is not None and self.gap_at <= rel < self.gap_at + 0.4)
            if not skip:
                events.append((self.next_js, ('joint_state', self.next_js,
                                              self._stamp(self.next_js),
                                              list(self.traj['joint_names']),
                                              self._positions(self.next_js))))
            self.next_js += self.js_period
        while self.next_fb is not None and self.next_fb <= end and self.held_at is None and \
                not any(e[0] == 'result' for _, e in events):
            st = self._stamp(self.next_fb)
            events.append((self.next_fb, ('feedback', {
                'stamp': st,
                'desired_time_from_start': st - self.ctrl_start_sim + self.feedback_offset})))
            self.next_fb += 0.05
        due = [(t, e) for t, e in self.queue if t <= end]
        self.queue = [(t, e) for t, e in self.queue if t > end]
        events += due
        for at in list(self.interrupts_at):
            if self.t_start + at <= end:
                self.interrupts_at.remove(at)
                if self.latch is not None:
                    self.latch.trigger()
        self.t = end
        events.sort(key=lambda te: te[0])
        return [e for _, e in events]
