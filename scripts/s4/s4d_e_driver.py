#!/usr/bin/env python3
"""S4d-E driver (docs/S4D_E_CONTRACT.md, frozen before this file): the level-2 emergency stop.

Built on the S4c driver (scripts/s4/s4c_driver.py, unchanged), which is built on the S4a, S3 and S2
drivers (unchanged). Reused as they are: the teleop pseudo-terminal and its key repeat; the
status and /joint_states subscriptions; fresh samples, the 2-sample debounces, the settle check,
the joint-limit stop and the tracking abort and finding (S3); the full-precision key-write bracket
(S4a); the level selection with attributed teleop confirmations (S4c); the stop handling (S3: any
stop releases the held key and sends x as the emergency response).

Script: exactly ONE planned goal and ONE planned cancel. No retry, recovery, second goal or reset.
  settle   before any key (S3: >= 10 s of sim, z >= 0.045 m, z range <= 5 mm, tilt <= 0.10 rad)
  level    '+ +' while DISARMED with no goal, before arm; the teleop confirms 2/3, then 3/3
  arm      'a'; READY at boundary 0 (the node is at level 0 before goal 1)
  goal 1   hold 'w': level 2, 0 -> 1, the S1 shift (1.9 s of sim in the frozen library)
  x        only while goal 1 is active (reported, accepted, no result) and within 0.95 s of sim
           of the driver's first sight of it accepted; LIVE: only after at least one attributed
           real feedback sample of goal 1 from the driver's own feedback subscription. SHADOW:
           no real feedback can exist, so x is not delayed for it (and any feedback stops).
           The repeat is released at once. If a condition fails, the script stops: the stop
           handling sends x as the emergency response; no second goal is ever sent.
  hold     FAULTED (only the planned estop), goal 1 resolved, then 10 s of sim measured (S3's
           hold observation: joint motion, tilt, z), FAULTED with no goal throughout
  close    'q': the teleop's stop and disarm, twice; the node accepts the stops and refuses the
           disarms (faulted_reset_required); the state stays FAULTED; the teleop exits 0 within
           15 s of wall
Stops (S3/S4a/S4c, unchanged): tilt > 0.10 rad or z < 0.045 m (2 fresh samples), travel > 0.15 m
from the arm pose (NOT S4d-M's 0.40 m), a joint within 0.02 rad of its limit, tracking > 0.05 rad,
command rejections, an unplanned fault, any goal but goal 1 (level 2, 0 -> 1), a level other
than 2 for goal 1, a completed cycle, the wait caps, and the 30-minute script wall cap. LIVE: the
node must report DISPATCH with the gate True in every status; SHADOW: SHADOW with the gate False.

Usage: s4d_e_driver.py --out <dir> [--domain-id 0] [--shadow-rehearsal]
Exit: 0 script complete, 2 stop condition, 3 driver error, 4 interrupted, 5 refused (no start)."""
import argparse
import json
import math
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import s4c_driver as s4c  # noqa: E402  (unchanged; brings the S4a, S3 and S2 drivers)

s4a, s3, s2 = s4c.s4a, s4c.s3, s4c.s2

LEVEL = 2                                   # internal index: 60 mm stride, teleop label 3/3
E_GOAL = (0, 1, 1)                          # (b_from, b_to, direction) at level 2: the S1 shift
E_GOAL_KIND = 'shift'
E_GOAL_DURATION_S = 1.9                     # sim, the frozen library (library.json 917fc4d7...)
X_DEADLINE_SIM_S = 0.95                     # half the phase, from the first sight of goal 1
EXPECTED_GOALS, EXPECTED_CANCELS = 1, 1
SCRIPT_CAP_WALL_S = 30 * 60.0
FEEDBACK_TOPIC = '/leg_trajectory_controller/follow_joint_trajectory/_action/feedback'
MAIN_KEYS = ["'+'", "'+'", "'a'", "'w'", "'x'", "'q'"]
FAULTED_STOP_MESSAGE = 'not moving (FAULTED); held until a zero command'
RESET_REQUIRED = 'faulted_reset_required'
CLOSE_REQUESTS = {'stop': 2, 'disarm': 2}
TELEOP_EXIT_WALL_S = 15.0
GOAL_SIM_S, FAULTED_SIM_S, RESOLVED_SIM_S, AFTER_CLOSE_WALL_S = 5.0, 5.0, 5.0, 2.0
SHADOW_NA = 'not applicable: dispatch disabled (shadow rehearsal)'
LIVE_GATES = {'M55_LOCOMOTION_DISPATCH_ENABLED': True, 'M61_LIVE_DISPATCH_ENABLED': False,
              'LIVE_DISPATCH_ENABLED': False}


def gate_values(gates):
    """{gate: value} from s3.fresh_gates()/imported_gates() output ({name: [value, file]})."""
    return {k: (v[0] if isinstance(v, (list, tuple)) else v) for k, v in (gates or {}).items()}


class S4dEDriver(s4c.S4cDriver):
    def __init__(self, out, domain_id, limits, shadow_rehearsal=False, gates=None):
        self._init_s4de()
        super().__init__(out, domain_id, limits, LEVEL, shadow_rehearsal, gates)

    def _init_s4de(self):
        self.t_start = time.monotonic()
        self.feedback = []                  # [(t_receipt, goal id, max |error|)], own subscription
        self.feedback_violation = None
        self.last_js_sim = None             # the latest joint-state stamp (sim time)
        self.goal1_seen = None
        self.x_record = None
        self.estop_status = None
        self.resolved = None
        self.after_close = None
        self.hold_summary = None

    # ------------------------------------------------------------------ inputs
    def _ros(self, domain_id):
        super()._ros(domain_id)
        from control_msgs.action import FollowJointTrajectory
        self.node.create_subscription(FollowJointTrajectory.Impl.FeedbackMessage, FEEDBACK_TOPIC,
                                      self._on_feedback_msg, 50)

    def _on_feedback_msg(self, msg):
        t = time.monotonic()
        err = max((abs(e) for e in msg.feedback.error.positions), default=0.0)
        self._on_feedback_sample(bytes(msg.goal_id.uuid).hex(), err, t)

    def _on_feedback_sample(self, goal_id, err, t_receipt):
        """One action feedback sample as the driver received it. Attribution (contract section 3):
        one goal id in the whole run, received after the write of 'w'. Anything else stops at
        the next check (the decision is not taken in the callback)."""
        w = next((k['t_after_write'] for k in self.key_writes if k['key'] == "'w'"), None)
        with self.lock:
            self.feedback.append((t_receipt, goal_id, err))
            ids = {g for _, g, _ in self.feedback}
            n = len(self.feedback)
            why = None
            if self.shadow_rehearsal:
                why = 'action feedback received in a shadow rehearsal (no real goal may exist)'
            elif w is None or t_receipt < w:
                why = 'action feedback before goal 1 could exist (before the write of w)'
            elif len(ids) > 1:
                why = f'action feedback with {len(ids)} goal ids (one goal only)'
            if why and self.feedback_violation is None:
                self.feedback_violation = why
        self.log(feedback={'n': n, 'goal_id': goal_id, 'max_abs_error_rad': err,
                           't_receipt': t_receipt})

    def _on_joint_state(self, names, positions, stamp):
        self.last_js_sim = stamp
        super()._on_joint_state(names, positions, stamp)

    def _sim_now(self, st=None):
        if self.last_js_sim is not None:
            return self.last_js_sim
        return ((st or self.st() or {}).get('monitors') or {}).get('sim_time_s')

    def attributed_feedback(self):
        """(n samples, goal id) attributed to goal 1, or (0, None)."""
        w = next((k['t_after_write'] for k in self.key_writes if k['key'] == "'w'"), None)
        with self.lock:
            fb = list(self.feedback)
        ids = {g for _, g, _ in fb}
        if w is None or len(ids) != 1 or any(t < w for t, _, _ in fb):
            return 0, None
        return len(fb), next(iter(ids))

    # ------------------------------------------------------------------ stops
    def check(self):
        if self.feedback_violation:
            raise s2.Abort(self.feedback_violation)
        if time.monotonic() - self.t_start > SCRIPT_CAP_WALL_S:
            raise s2.Abort(f'script wall cap {SCRIPT_CAP_WALL_S / 60:.0f} min exceeded')
        super().check()

    def _check_status(self, st):
        g = st.get('goal')
        if self.shadow_rehearsal:
            if st.get('mode') != 'shadow' or st.get('dispatch_gate') is not False:
                raise s2.Abort(f'shadow rehearsal: node status mode {st.get("mode")!r}, '
                               f'dispatch_gate {st.get("dispatch_gate")!r} (SHADOW with the gate '
                               'False required throughout)')
        elif st.get('mode') != 'dispatch' or st.get('dispatch_gate') is not True:
            raise s2.Abort(f'live: node status mode {st.get("mode")!r}, dispatch_gate '
                           f'{st.get("dispatch_gate")!r} (DISPATCH with the gate True required)')
        if g is not None and isinstance(g.get('seq'), int):
            got = (g.get('b_from'), g.get('b_to'), g.get('direction'))
            if g['seq'] != 1 or got != E_GOAL or g.get('level') != LEVEL:
                raise s2.Abort(f'goal {g["seq"]} {got} at level {g.get("level")}: the plan is '
                               f'one goal, level {LEVEL}, {E_GOAL[0]} -> {E_GOAL[1]}')
            self.checked_max_seq = max(self.checked_max_seq, g['seq'])
        rec = st.get('goals_recorded')
        if isinstance(rec, int) and rec > EXPECTED_GOALS:
            raise s2.Abort(f'{rec} goals recorded by the node (the plan is one)')
        lvl = st.get('level')
        if self.checked_max_seq == 0 and lvl != 0:
            raise s2.Abort(f'node level {lvl} before goal 1 (it arms at level 0)')
        if self.checked_max_seq >= 1 and lvl != LEVEL:
            raise s2.Abort(f'node level {lvl} after goal 1; the selected level is {LEVEL}')
        il = (st.get('intent') or {}).get('level')
        if il is not None and il != LEVEL:
            raise s2.Abort(f'intent level {il}: the teleop is not at the selected level {LEVEL}')
        state = st['state']
        if state == 'FAULTED':
            if not self.planned_estop:
                raise s2.Abort(f'unplanned FAULTED: {st.get("faults")}')
            if st.get('faults') != ['estop']:
                raise s2.Abort(f'FAULTED with {st.get("faults")}: only the planned estop')
        if self.stage in ('hold', 'closing', 'closed'):
            if state != 'FAULTED' or g is not None:
                raise s2.Abort(f'after the estop: state {state}, goal {g} (FAULTED with no goal '
                               'must hold: no reset, re-arm, home or recovery)')
        if st.get('command_rejections'):
            raise s2.Abort(f'command rejections {st["command_rejections"]}')
        if self.max_seq > EXPECTED_GOALS:
            raise s2.Abort(f'goal seq {self.max_seq} above the plan ({EXPECTED_GOALS})')
        self._tracking(g)
        fp = s3.fresh_pose(st, self.last_fresh_sim)
        if fp is None:
            return
        gt, sim = fp
        self.last_fresh_sim = sim
        self.fresh_n += 1
        if self.in_hold:
            self._hold_sample(gt, sim)
            return
        if not self.armed:
            if self.settle_state is not None and self.settle_state.done is None:
                self.settle_state.add(sim, gt['z'], gt['tilt'],
                                      (st.get('monitors') or {}).get('graph'))
            return
        if self.tilt_db.update(gt['tilt'] > s3.TILT_MAX_RAD):
            raise s2.Abort(f'tilt {gt["tilt"]:.4f} rad > {s3.TILT_MAX_RAD} in '
                           f'{s3.DEBOUNCE_SAMPLES} consecutive fresh samples')
        if self.fall_db.update(gt['z'] < s3.Z_MIN_M):
            raise s2.Abort(f'body z {gt["z"]:.4f} m < {s3.Z_MIN_M} in {s3.DEBOUNCE_SAMPLES} '
                           'consecutive fresh samples')
        d = gt.get('delta_since_arm_xy')
        if d is not None and math.hypot(d[0], d[1]) > s3.TRAVEL_MAX_M:
            raise s2.Abort(f'travel {math.hypot(d[0], d[1]):.4f} m from the arm pose > '
                           f'{s3.TRAVEL_MAX_M} (S3 cap; S4d-M\'s 0.40 m does not apply)')
        c = st.get('cycles')
        if self.cycles_checked is None:
            self.cycles_checked = c
        elif c != self.cycles_checked:
            raise s2.Abort(f'cycle count {self.cycles_checked} -> {c}: no cycle may complete')

    # ------------------------------------------------------------------ script
    def _goal1_active(self, st):
        g = st.get('goal') or {}
        return (g.get('seq') == 1 and g.get('accepted') is True
                and st.get('state') in ('STARTING', 'WALKING'))

    def script(self, levels=None):
        mode, gate = ('shadow', False) if self.shadow_rehearsal else ('dispatch', True)
        st = self.wait(f'node status {mode.upper()} with the dispatch gate {gate}',
                       lambda st: st.get('mode') == mode and st.get('dispatch_gate') is gate, 30.0)
        self.shadow_verified = self.shadow_rehearsal
        self.log(mode_check={'node_mode': st.get('mode'),
                             'node_dispatch_gate': st.get('dispatch_gate'),
                             'driver_gates': self.gates})
        self.spawn()
        self.wait('status with levels at level 0',
                  lambda st: bool(st.get('levels_m_s')) and st.get('level') == 0, 30.0)
        time.sleep(3.0)                               # the teleop receives the levels too
        self.settle()
        self.select_level()                           # '+ +': 2/3, then 3/3 (S4c, unchanged)
        self.key('a', 'arm')
        self.wait('READY at boundary 0 after arm', self.ready_at(0), 5.0)
        self.armed, self.stage = True, 'armed'
        self.expected_goals = EXPECTED_GOALS
        self.cycle_direction = s4a.FORWARD            # goal 1 is 0 -> 1; no cycle may end
        self.hold('w', 'S4d-E: hold w for goal 1 (level 2, 0 -> 1, the S1 shift)')
        st = self.wait('goal 1 in flight and accepted', self._goal1_active, GOAL_SIM_S)
        self.stage = 'goal'
        sim0 = self._sim_now(st)
        self.goal1_seen = {'mono': time.monotonic(), 'sim': sim0}
        self.log(goal1_seen=self.goal1_seen)
        if sim0 is None:
            raise s2.Abort('no sim time when goal 1 was seen: the x deadline cannot be kept')
        if self.shadow_rehearsal:
            self.log(feedback_precondition=SHADOW_NA + ': no real feedback can exist; x is not '
                     'delayed for it')
        else:
            self.wait('at least one attributed feedback sample of goal 1',
                      lambda st: self.attributed_feedback()[0] >= 1
                      or self._sim_now(st) - sim0 > X_DEADLINE_SIM_S, X_DEADLINE_SIM_S)
            if self.attributed_feedback()[0] < 1:
                raise s2.Abort(f'no attributed feedback for goal 1 within {X_DEADLINE_SIM_S} s '
                               'of sim: the experiment is not performed (no second goal)')
        self.check()
        st = self.st() or {}
        elapsed = self._sim_now(st) - sim0
        if not self._goal1_active(st):
            raise s2.Abort(f'goal 1 is no longer active before x (state {st.get("state")}, '
                           f'goal {st.get("goal")}): no second goal is sent')
        if not elapsed <= X_DEADLINE_SIM_S:
            raise s2.Abort(f'x deadline passed: {elapsed:.3f} s of sim since goal 1 was seen '
                           f'(at most {X_DEADLINE_SIM_S})')
        n_fb, gid = self.attributed_feedback()
        self.planned_estop = True
        self.stage = 'estop'
        self.key('x', 'S4d-E: x while goal 1 is active')
        kw = self.key_writes[-1]
        self.release('S4d-E: the repeat stops right after x')
        self.x_record = {'t_before_write': kw['t_before_write'],
                         't_after_write': kw['t_after_write'], 'write_result': kw['write_result'],
                         'sim_elapsed_since_goal1_seen_s': elapsed,
                         'attributed_feedback_before_x': n_fb, 'feedback_goal_id': gid,
                         'feedback_precondition': SHADOW_NA if self.shadow_rehearsal
                         else 'required: at least one attributed sample'}
        self.log(x_record=self.x_record)
        st = self.wait('FAULTED after x', lambda st: st['state'] == 'FAULTED', FAULTED_SIM_S)
        self.estop_status = {'sim': (st.get('monitors') or {}).get('sim_time_s'),
                             'faults': st.get('faults'), 'goal': st.get('goal')}
        st = self.wait('goal 1 resolved after the cancel (FAULTED, no goal)',
                       lambda st: st['state'] == 'FAULTED' and st.get('goal') is None,
                       RESOLVED_SIM_S)
        self.resolved = {'sim': (st.get('monitors') or {}).get('sim_time_s'),
                         'goals_recorded': st.get('goals_recorded')}
        self.log(estop=self.estop_status, resolved=self.resolved)
        self.stage = 'hold'
        self.estop_sim = self.resolved['sim']         # the hold is measured from the result
        self.hold_observation()                       # S3: 10 s of sim (recorded, A6 limits)
        self.close_after_estop()

    def close_after_estop(self):
        """'q' in FAULTED: the teleop's stop and disarm, twice; the disarms are refused (reset
        required); the state stays FAULTED; the teleop's own exit (0) within 15 s of wall."""
        st = self.st() or {}
        if st.get('state') != 'FAULTED' or st.get('goal') is not None or self.hold_key:
            raise s2.Abort(f'close only in FAULTED with no goal and no held key (state '
                           f'{st.get("state")}, goal {st.get("goal")})')
        self.stage = 'closing'
        self.key('q', 'S4d-E: q (stop and disarm twice; disarm is refused: reset required)')
        end = time.monotonic() + TELEOP_EXIT_WALL_S
        status = self._teleop_exit_status()
        while status is None:
            self.check()
            if time.monotonic() >= end:
                raise s2.Abort(f'the teleop was still running {TELEOP_EXIT_WALL_S:.0f} s after q')
            time.sleep(0.05)
            status = self._teleop_exit_status()
        self.teleop_alive = False                   # a stop now uses the node's estop service
        code = os.waitstatus_to_exitcode(status) if isinstance(status, int) else None
        self.close_record = {'event': 'teleop_closed', 'pid': self.pid, 'step': 'q',
                             'wait_status': status if isinstance(status, int) else repr(status),
                             'exit_code': code, 't_exit_seen': time.monotonic()}
        self.log(**self.close_record)
        if code != 0:
            raise s2.Abort(f'the teleop exited with {code!r} after q (0 expected)')
        self.stage = 'closed'
        t_end = time.monotonic() + AFTER_CLOSE_WALL_S
        st = self.wait('FAULTED with no goal after the close',
                       lambda st: time.monotonic() >= t_end and st['state'] == 'FAULTED'
                       and st.get('goal') is None, AFTER_CLOSE_WALL_S)
        self.after_close = {'state': st['state'], 'goals_recorded': st.get('goals_recorded'),
                            'max_seq': self.max_seq, 'last_refusal': st.get('last_refusal')}
        self.log(after_close=self.after_close)

    def summary(self):
        s = super().summary()
        for k in ('plan', 'scripted_keys', 'level_spec', 'net_return_recorded_only',
                  'close_observation'):
            s.pop(k, None)
        with self.lock:
            fb = list(self.feedback)
        s.update({'expected_goals': EXPECTED_GOALS, 'expected_cancels': EXPECTED_CANCELS,
                  'plan': [[LEVEL, *E_GOAL]], 'main_keys': MAIN_KEYS,
                  'x_deadline_sim_s': X_DEADLINE_SIM_S, 'travel_cap_m': s3.TRAVEL_MAX_M,
                  'script_cap_wall_s': SCRIPT_CAP_WALL_S, 'goal1_seen': self.goal1_seen,
                  'x_record': self.x_record, 'estop': self.estop_status,
                  'resolved': self.resolved, 'hold': self.hold_summary,
                  'close': self.close_record, 'after_close': self.after_close,
                  'feedback_samples': len(fb),
                  'feedback_goal_ids': sorted({g for _, g, _ in fb}),
                  'feedback_violation': self.feedback_violation, 'stage': self.stage})
        if self.shadow_rehearsal:
            s['shadow_rehearsal'] = {
                'driver_gates': self.gates, 'node_mode_verified': self.shadow_verified,
                'feedback_precondition': SHADOW_NA, 'tracking': SHADOW_NA,
                'note': 'dispatch disabled: one planned shadow goal and its modeled '
                        'cancellation; nothing reaches the controller; not live cancel-and-hold '
                        'validation'}
        return s


def run(d):
    """The script with its stop handling (S3's): (exit code, result)."""
    rc, why = s3.run(d)
    if rc == 0 and d.shadow_rehearsal:
        why = 'script complete (shadow rehearsal: dispatch disabled; not S4d-E acceptance)'
    return rc, why


def gate_refusal(gates, shadow):
    """None when the imported gates are exactly the mode's (shadow: all False; live: M55 True,
    M61 and M6.0-D False), else the reason."""
    want = {k: False for k in LIVE_GATES} if shadow else LIVE_GATES
    got = gate_values(gates)
    if got != want:
        return f'gates {got}, required {want} ({"shadow" if shadow else "live"})'
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--domain-id', type=int, default=0)
    ap.add_argument('--shadow-rehearsal', action='store_true',
                    help='dispatch disabled: verify SHADOW throughout; no feedback wait; tracking '
                         'not applicable (see the module docstring)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    from spiderx_controller import joint_safety
    limits, _ = joint_safety.load_limits()
    try:
        gates = {'fresh_interpreter': s3.fresh_gates(), 'driver_process': s3.imported_gates()}
        why = (gate_refusal(gates['fresh_interpreter'], a.shadow_rehearsal)
               or gate_refusal(gates['driver_process'], a.shadow_rehearsal))
    except Exception as e:  # noqa: BLE001  (no gate evidence: refuse)
        gates, why = None, f'gate import failed: {e!r}'
    if why:
        s4c.refuse(a.out, why, {'shadow_rehearsal': a.shadow_rehearsal, 'gates': gates})

    def interrupted(signum, frame):
        raise s3.Interrupted(signal.Signals(signum).name)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    d = S4dEDriver(a.out, a.domain_id, limits, a.shadow_rehearsal, gates)
    rc, why = run(d)
    scope = ('S4d-E shadow rehearsal: one planned shadow goal (level 2, 0 -> 1) and its modeled '
             'cancellation; dispatch disabled; not acceptance' if a.shadow_rehearsal else
             'S4d-E: one goal (level 2, 0 -> 1) and one cancel at most, the estop, the measured '
             'hold and the close')
    summary = {'result': why, 'scope': scope, **d.summary()}
    with open(os.path.join(a.out, 'driver_summary.json'), 'w') as f:
        json.dump(summary, f, indent=1, sort_keys=True, default=str)
    d.log(summary=summary)
    print(json.dumps(summary, sort_keys=True, default=str), flush=True)
    if d.pid and getattr(d, 'teleop_alive', False):
        try:
            os.killpg(d.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    try:
        d.close()
    except Exception as e:  # noqa: BLE001
        print(f'close: {e!r}', flush=True)
    sys.stdout.flush()
    os._exit(rc)      # skip interpreter teardown: the C++ executor can abort there (exit 134)


if __name__ == '__main__':
    main()
