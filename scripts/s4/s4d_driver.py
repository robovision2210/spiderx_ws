#!/usr/bin/env python3
"""S4d-M script driver (docs/S4D_ACCEPTANCE_PLAN.md, sections 5, 9 and 12): about 0.3 m forward and
back at internal level 2, the speed change while walking, release, reversal, home, a close while
moving and a reconnect probe, in ONE launch. Offline preparation: written and bench-tested only.

Built on the S4c driver (scripts/s4/s4c_driver.py), which builds on the S4a, S3 and S2 drivers; all
are unchanged. Reused as they are:
- the teleop pseudo-terminal, its key repeat and the S4a full-precision key-write brackets;
- S3's fresh samples, 2-sample debounce, settle check, joint-limit and tracking stops and the 10 s
  sim observation;
- S4b's timed repeat stop (time.monotonic() immediately before and after) and its key switch;
- S4c's teleop output with chunk receipt times, its one-confirmation-per-key attribution (a cursor
  from the write, lines rebuilt across chunks; stale, duplicate or wrong lines stop), the teleop's
  exit status and the post-close observation.
NOT reused: S4c's DISARMED-only level selection (here only the first '+' is pressed before arm; the
second is pressed while walking), and every level-0 assumption.

Plan entries are (level, b_from, b_to, direction); goal seq k is PLANNED[k - 1]. 111 goals, 0
planned cancels (OD5):
  settle     before arming, as S3: >= 10 s of sim, z >= 0.045 m, z range <= 0.005 m, tilt <= 0.10
  select     '+' while DISARMED, before arm: teleop "speed level 2/3" (internal level 1)
  arm        'a'; READY at boundary 0 (the node arms at level 0)
  SC + F     hold 'w': goals 1-9 at level 1 (level_change 0 -> 1 with goal 1). '+' while goal 2
             is in flight: teleop 3/3. The node keeps level 1 to boundary 0 and adopts level 2
             there, while walking (level_change 1 -> 2 with goal 10). Goals 10-54: five level-2
             cycles.
             Space while goal 54 (8->9) is in flight; READY at 0; the cycle checked
  R          hold 's': goals 55-99, five level-2 reverse cycles; space while goal 99 (1->0) is in
             flight; READY at 0; checked
  X          hold 'w': goals 100-104 (0->1 ... 4->5), released while 104 is in flight: READY at 5.
             Hold 'w': goal 105 (5->6); 's' instead while it is in flight (S4b switch): goal 106
             (6->5) at boundary 6; released while 106 is in flight: READY at 5. 'h': goals 107-110
             (5->6 ... 8->9), homed at 0; the composite cycle checked
  Q          hold 'w': goal 111 (0->1). The repeat stops; 0.2 s later (inside the 0.8 s hold window)
             'q': the teleop calls stop and disarm, publishes its zero heartbeat, calls them again
             and exits 0. STOPPING, then DISARMED at boundary 1 (the node's own lease decision:
             'lease_expired' or 'disarm'; scored offline, M10)
  observe    10 s of sim DISARMED, as the S3 hold
  P          a second teleop on the DISARMED node (OD11): '+ +', hold 'w' for 5 s, release, 'q'. It
             never presses 'a'. Bounds (wall): startup 30 s, each confirmation 5 s, the release 5 s
             after the hold, intent back to stop within 5 s, exit within 15 s of the 'q'
Keys: + a w + space s space w w s h w q, then + + w q. Requests before 'q': arm, stop, stop, home.

Live stops LS1-LS10 (docs section 9); every one is an emergency response: the repeat stops, 'x' is
written (or, once the teleop has read 'q', the node's estop service is called), exit 2.
- LS1 plan: a goal beyond 111 or above the step's budget, off the plan, a node level other than the
  plan's, a moving intent at another level (1 before the second '+'; 1 or 2 once it is written; 2
  once the node reports 2; 2 in the probe).
- LS2 state: FAULTED; STOPPING or READY while a motion key is held with no scripted stop; after 'q',
  anything but WALKING or STOPPING with goal 111 (or none, awaiting a joint sample), or DISARMED
  with no goal; after DISARMED, anything but DISARMED with no goal.
- LS3 tilt > 0.10 rad or z < 0.045 m in 2 consecutive fresh samples (S3, unchanged).
- LS4 travel: Euclidean XY from the arm pose > 0.40 m (OD3, S4d only; the reference is never reset;
  S3's 0.15 m constant is unchanged).
- LS5 per cycle, at the first fresh status after each cycle-count change: SC [0, 60] mm, F and the X
  composite [0, 80] mm, R [-80, 0] mm along body +y in the frame of the cycle's start check;
  lateral > 20 mm or |yaw| > 0.10 rad; a count change that is not one cycle in the step's direction.
- LS6 cumulative (OD4), at the same checks during F and R: |lateral| > 50 mm or |heading change| >
  0.15 rad from B9,live (the SC cycle's own check), in its frame. Set once, never reset.
- LS7 a joint within 0.02 rad of its URDF limit; tracking > 0.05 rad (S3, unchanged).
- LS8 command rejections; with --shadow-rehearsal, any status not SHADOW with the gate False.
- LS9 teleop: a missing, stale, duplicate or wrong level confirmation; no exit within 15 s or a
  non-zero exit; any probe bound or probe evidence.
- LS10 time: each wait's backstop (sim cap / 0.1 + 60 s wall, S2); the 90-minute script cap (OD8),
  from the driver's start. SIGINT and SIGTERM are handled at once (exit 4).
Shadow rehearsal (verified SHADOW, gates False): the signed per-cycle bounds are not applied
("Signed progress condition not applicable: dispatch disabled."); |progress| above the cycle's
outer bound (60 or 80 mm), the drift, cumulative and every other stop still apply.

Usage: s4d_driver.py --out <dir> [--domain-id 0] [--shadow-rehearsal]
Exit: 0 script complete, 2 stop condition, 3 driver error, 4 interrupted, 5 refused before start."""
import argparse
import json
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import s4b_driver as s4b  # noqa: E402  (unchanged: the timed repeat stop and the key switch)
import s4c_driver as s4c  # noqa: E402  (unchanged: confirmations, exit status, observation)
s4a, s3, s2 = s4c.s4a, s4c.s3, s4c.s2

P, FORWARD, REVERSE = s4a.P, s4a.FORWARD, s4a.REVERSE
STRIDE_M = {1: 0.040, 2: 0.060}                     # internal level -> stride (node strides_m)


def _forward(level, b0, b1):
    return tuple((level, b, b + 1, FORWARD) for b in range(b0, b1))


SC = _forward(1, 0, P)                                            # goals 1-9
F = _forward(2, 0, P) * 5                                         # 10-54
R = tuple((2, b, b - 1, REVERSE) for b in range(P, 0, -1)) * 5    # 55-99
X_RELEASE = _forward(2, 0, 5)                                     # 100-104
X_REVERSAL = ((2, 5, 6, FORWARD), (2, 6, 5, REVERSE))             # 105-106
X_HOME = _forward(2, 5, P)                                        # 107-110
Q = ((2, 0, 1, FORWARD),)                                         # 111
PLANNED = SC + F + R + X_RELEASE + X_REVERSAL + X_HOME + Q
EXPECTED_GOALS, EXPECTED_CANCELS = len(PLANNED), 0                # 111, 0 (OD5)
MAIN_KEYS = ["'+'", "'a'", "'w'", "'+'", "' '", "'s'", "' '", "'w'", "'w'", "'s'", "'h'", "'w'",
             "'q'"]
PROBE_KEYS = ["'+'", "'+'", "'w'", "'q'"]
SCRIPTED_KEYS = MAIN_KEYS + PROBE_KEYS
REQUESTS_BEFORE_Q = ('arm', 'stop', 'stop', 'home')
# the cycle checks in order: (name, level, direction, live stop band along body +y, m)
CYCLE_CHECKS = ((('SC', 1, FORWARD, (0.0, 0.060)),)
                + tuple((f'F{j}', 2, FORWARD, (0.0, 0.080)) for j in range(1, 6))
                + tuple((f'R{j}', 2, REVERSE, (-0.080, 0.0)) for j in range(1, 6))
                + (('X', 2, FORWARD, (0.0, 0.080)),))
CUMULATIVE_CHECKS = tuple(c[0] for c in CYCLE_CHECKS[1:11])       # F1-F5, R1-R5
# S4d-only limits (the plan's owner decisions); shared and historical constants are unchanged
TRAVEL_MAX_M = 0.40                  # OD3: Euclidean XY from the arm pose (S3: 0.15 elsewhere)
CUM_LATERAL_STOP_M = 0.050           # OD4: cumulative live stops from B9,live
CUM_YAW_STOP_RAD = 0.15
SCRIPT_CAP_WALL_S = 90 * 60.0        # OD8: from the driver's start
LATERAL_MAX_M, YAW_MAX_RAD = s4a.LATERAL_MAX_M, s4a.YAW_MAX_RAD    # 0.020 m, 0.10 rad (S3)
SIM_MARGIN_S = 1.0                   # READY or DISARMED within the in-flight goal + 1.0 s sim
LONGEST, CYCLE_S = s2.LONGEST_GOAL_S, s2.CYCLE_S
CONFIRM_SIM_S, TELEOP_EXIT_WALL_S = s4c.CONFIRM_SIM_S, s4c.TELEOP_EXIT_WALL_S   # 2.0 sim, 15 s
REPEAT_QUIESCE_S = s4b.REPEAT_QUIESCE_S                           # 0.2 s (< the 0.8 s hold)
PROBE_START_WALL_S = 30.0            # OD11 probe bounds (wall, driver clock)
PROBE_CONFIRM_WALL_S = 5.0
PROBE_HOLD_WALL_S = 5.0
PROBE_RELEASE_WALL_S = 5.0
TELEOP_STATUS_LINE = 'state DISARMED'  # m55_teleop.status_line, once the teleop has a node status
SHADOW_SIGN_NOTE = s4a.SHADOW_SIGN_NOTE
RUN_NAME = s4a.RUN_NAME


class S4dDriver(s4c.S4cDriver):
    # S4b's timed repeat stop and key switch, unchanged (they use S4b's own module clock)
    _stop_repeat = s4b.S4bDriver._stop_repeat
    _stop_repeat_timed = s4b.S4bDriver._stop_repeat_timed
    switch_held_key = s4b.S4bDriver.switch_held_key

    def __init__(self, out, domain_id, limits, shadow_rehearsal=False, gates=None):
        self._init_s4d()
        s4a.S4aDriver.__init__(self, out, domain_id, limits, shadow_rehearsal, gates)

    def _init_s4d(self):
        s4b.S4bDriver._init_s4b(self)       # releases, switches (kills stays empty)
        self.t_start = time.monotonic()     # the 90-minute script cap counts from here (OD8)
        self.teleop_out = []                # [(t_receipt, text)] as the pty delivered it (S4c)
        self.stage = 'pre'      # pre arming ready hold stopping home closing closed probe done
        self.hold_first_seq, self.hold_started = None, False
        self.checked_max_seq = 0
        self.levels = None
        self.plus2_written, self.intent2_seen = False, False
        self.arm_ref, self.b9_ref, self.net_return = None, None, None
        self.selection, self.speed_change = None, None
        self.repeat_stop_before_q, self.close_record, self.close_observation = None, None, None
        self.probe, self.probe_phase = None, None

    # ------------------------------------------------------------------ time
    def check(self):
        if time.monotonic() - self.t_start > SCRIPT_CAP_WALL_S:
            raise s2.Abort(f'script wall cap {SCRIPT_CAP_WALL_S / 60:.0f} min exceeded (OD8)')
        super().check()

    def wait_wall(self, what, pred, wall_s):
        """S2's wait with a wall bound (the probe's bounds are wall time: the teleop and the lease
        run on wall clocks)."""
        end = time.monotonic() + wall_s
        while time.monotonic() < end:
            self.check()
            st = self.st()
            if st is not None and pred(st):
                self.log(reached=what)
                return st
            time.sleep(0.05)
        raise s2.Abort(f'wall bound {wall_s:.0f} s exceeded waiting for: {what}')

    # ------------------------------------------------------------------ stops
    def _check_status(self, st):
        g = st.get('goal')
        self._check_plan(st, g)
        self._check_state(st, g)
        self._probe_evidence(st)
        if self.shadow_rehearsal and (st.get('mode') != 'shadow'
                                      or st.get('dispatch_gate') is not False):
            raise s2.Abort(f'shadow rehearsal: node status mode {st.get("mode")!r}, dispatch_gate '
                           f'{st.get("dispatch_gate")!r} (SHADOW with the gate False required '
                           'throughout)')
        if st.get('command_rejections'):
            raise s2.Abort(f'command rejections {st["command_rejections"]}')
        if self.max_seq > min(self.expected_goals, EXPECTED_GOALS):
            raise s2.Abort(f'goal seq {self.max_seq} above the step budget ({self.expected_goals})')
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
        if d is not None and (d[0] ** 2 + d[1] ** 2) ** 0.5 > TRAVEL_MAX_M:
            raise s2.Abort(f'travel {(d[0] ** 2 + d[1] ** 2) ** 0.5:.4f} m from the arm pose > '
                           f'{TRAVEL_MAX_M} (S4d cap, OD3)')
        self._cycle_check(st, gt, sim)

    def _check_plan(self, st, g):
        """LS1: every goal on the plan; the node level and a moving intent's level as planned."""
        lvl = st.get('level')
        if g is not None and isinstance(g.get('seq'), int):
            s = g['seq']
            got = (g.get('level'), g.get('b_from'), g.get('b_to'), g.get('direction'))
            if s > EXPECTED_GOALS:
                raise s2.Abort(f'goal {s} {got} beyond the S4d-M plan ({EXPECTED_GOALS} goals)')
            if s >= 1 and got != PLANNED[s - 1]:
                raise s2.Abort(f'goal {s} is {got}, planned {PLANNED[s - 1]}')
            if s >= 1 and lvl != PLANNED[s - 1][0]:
                raise s2.Abort(f'node level {lvl} while goal {s} is in flight (planned level '
                               f'{PLANNED[s - 1][0]})')
            self.checked_max_seq = max(self.checked_max_seq, s)
        else:
            k = self.checked_max_seq
            want = 0 if k == 0 else PLANNED[k - 1][0]
            if lvl != want:
                raise s2.Abort(f'node level {lvl} with no goal in flight after goal {k} (planned '
                               f'{want})')
        it = st.get('intent') or {}
        il = it.get('level')
        if il is not None:
            allowed = ((2,) if self.stage in ('probe', 'done') else (1,) if not self.plus2_written
                       else (2,) if self.intent2_seen else (1, 2))
            if il not in allowed:
                raise s2.Abort(f'intent level {il}; allowed now {allowed} (the teleop is not at '
                               'the scripted level)')
            if il == 2 and self.stage not in ('probe', 'done'):
                self.intent2_seen = True

    def _check_state(self, st, g):
        """LS2: the states each scripted step allows."""
        state, stage = st['state'], self.stage
        seq = g.get('seq') if g is not None else None
        if state == 'FAULTED':
            raise s2.Abort(f'FAULTED {st.get("faults")} (S4d-M has no planned cancel)')
        if stage == 'hold' and seq is not None and seq >= self.hold_first_seq:
            self.hold_started = True
        ok = {
            'pre': state == 'DISARMED' and g is None,
            'arming': state in ('DISARMED', 'READY') and g is None,
            'ready': state == 'READY' and g is None,
            'hold': state in ('STARTING', 'WALKING')
            or (state == 'READY' and g is None and not self.hold_started),
            'stopping': state in ('WALKING', 'STOPPING', 'READY'),
            'home': state in ('STARTING', 'WALKING') or (state == 'READY' and g is None),
            'closing': (state in ('WALKING', 'STOPPING') and seq in (None, EXPECTED_GOALS))
            or (state == 'DISARMED' and g is None),
        }.get(stage, state == 'DISARMED' and g is None)        # closed, probe, done
        if not ok:
            raise s2.Abort(f'state {state} with goal {seq} not allowed in step {stage!r}')

    def _probe_evidence(self, st):
        """OD11: the probe's delivery and release, from every status in arrival order."""
        if self.stage != 'probe' or self.probe is None:
            return
        it = st.get('intent') or {}
        rec = {'intent': it, 'commands_accepted': st.get('commands_accepted'),
               'lease_age_s': st.get('lease_age_s'),
               'sim': (st.get('monitors') or {}).get('sim_time_s'), 't_seen': time.monotonic()}
        moving_at_2 = (it.get('direction') == FORWARD and it.get('level') == 2
                       and it.get('reason') is None)
        if self.probe_phase == 'hold' and self.probe.get('delivery') is None and moving_at_2:
            self.probe['delivery'] = rec
            self.log(probe_delivery=rec)
        if self.probe_phase == 'released' and self.probe.get('stopped_after_release') is None \
                and it.get('direction') == 0:
            self.probe['stopped_after_release'] = rec
            self.log(probe_stopped_after_release=rec)

    def _cycle_check(self, st, gt, sim):
        """LS5 per cycle and LS6 cumulative, at the first fresh status after each count change."""
        pose = {'xy': tuple(gt['xy']), 'yaw': gt['yaw']}
        c = st.get('cycles')
        if self.cycle_ref is None:
            self.cycle_ref, self.cycles_checked, self.arm_ref = pose, c, pose
            self.log(cycle_reference=pose, cycles=c, sim=sim)
            return
        if c is None or c == self.cycles_checked:
            return
        i = len(self.cycle_log)
        if i >= len(CYCLE_CHECKS):
            raise s2.Abort(f'cycle count {self.cycles_checked} -> {c}: more cycles than planned')
        name, level, dirn, (lo, hi) = CYCLE_CHECKS[i]
        if c != self.cycles_checked + dirn:
            raise s2.Abort(f'cycle count {self.cycles_checked} -> {c}: not one {RUN_NAME[dirn]} '
                           f'cycle ({name})')
        fwd, lat, dyaw = s3.body_delta(self.cycle_ref, pose)
        rec = {'cycle': i + 1, 'name': name, 'level': level, 'direction': dirn,
               'cycles_count': c, 'sim': sim, 'forward_m': fwd, 'lateral_m': lat,
               'yaw_rad': dyaw, 'live_stop_band_m': [lo, hi]}
        shadow = (self.shadow_verified and st.get('mode') == 'shadow'
                  and st.get('dispatch_gate') is False)
        if shadow:
            rec['progress_sign_bound'] = SHADOW_SIGN_NOTE
        cum = None
        if name == 'SC':
            self.b9_ref = pose
            rec['cumulative_reference'] = 'B9,live: this check, set once, never reset'
        elif name in CUMULATIVE_CHECKS:
            _, cl, cy = s3.body_delta(self.b9_ref, pose)
            cum = rec['cumulative_from_b9'] = {'lateral_m': cl, 'yaw_rad': cy}
        if name == 'R5':
            f9, l9, y9 = s3.body_delta(self.b9_ref, pose)
            f0, l0, y0 = s3.body_delta(self.arm_ref, pose)
            self.net_return = rec['net_return_recorded_only'] = {
                'from_b9': {'forward_m': f9, 'lateral_m': l9, 'yaw_rad': y9},
                'from_arm': {'forward_m': f0, 'lateral_m': l0, 'yaw_rad': y0},
                'note': 'live samples; recorded only, never a band'}
        self.cycle_log.append(rec)
        self.log(cycle_check=rec)
        self.cycle_ref, self.cycles_checked = pose, c
        if shadow:
            outer = max(abs(lo), abs(hi))
            if not abs(fwd) <= outer:
                raise s2.Abort(f'{name} cycle moved {fwd * 1000:.1f} mm along body +y (shadow '
                               f'rehearsal: |progress| at most {outer * 1000:.0f})')
        elif not lo <= fwd <= hi:
            raise s2.Abort(f'{name} cycle moved {fwd * 1000:.1f} mm along body +y at level {level} '
                           f'(allowed {lo * 1000:.0f}..{hi * 1000:.0f})')
        if abs(lat) > LATERAL_MAX_M or abs(dyaw) > YAW_MAX_RAD:
            raise s2.Abort(f'{name} cycle drifted {lat * 1000:.1f} mm laterally, {dyaw:.4f} rad '
                           'yaw')
        if cum is not None and (abs(cum['lateral_m']) > CUM_LATERAL_STOP_M
                                or abs(cum['yaw_rad']) > CUM_YAW_STOP_RAD):
            raise s2.Abort(f'{name}: cumulative drift from B9 {cum["lateral_m"] * 1000:.1f} mm '
                           f'laterally, {cum["yaw_rad"]:.4f} rad heading (stop above '
                           f'{CUM_LATERAL_STOP_M * 1000:.0f} mm or {CUM_YAW_STOP_RAD} rad, OD4)')

    # ------------------------------------------------------------------ keys
    def release(self, why, step=None):
        """S4b's timed release (the repeat stops; nothing is written), tagged with its step."""
        rec = self._stop_repeat_timed(why)
        rec['step'] = step
        self.releases.append(rec)
        self.log(release=rec['key'], **{k: v for k, v in rec.items() if k != 'key'})

    @staticmethod
    def goal_in_flight(seq):
        want = PLANNED[seq - 1]

        def pred(st):
            g = st.get('goal')
            return g is not None and g.get('seq') == seq and \
                (g.get('level'), g.get('b_from'), g.get('b_to'), g.get('direction')) == want
        return pred

    def _press_level_key(self, i, why, wall_s=None):
        """One '+' and its own confirmation, "speed level <i + 1>/3: <speed> mm/s" (S4c's cursor
        rules). wall_s: a wall bound instead of the sim-based wait (the probe)."""
        want = (str(i + 1), str(len(self.levels)), f'{self.levels[i] * 1000:.2f}')
        with self.lock:
            k0 = len(self.teleop_out)
        self.key('+', why)
        kw = self.key_writes[-1]                    # this press's full-precision write bracket
        tb = kw['t_before_write']

        def pred(st):
            return self._confirmation(k0, tb) is not None
        what = f'teleop confirms speed level {want[0]}/{want[1]}'
        if wall_s is None:
            self.wait(what, pred, CONFIRM_SIM_S)
        else:
            self.wait_wall(what, pred, wall_s)
        got = self._confirmation(k0, tb)
        if got[:3] != want:
            raise s2.Abort(f'teleop confirmed speed level {got[0]}/{got[1]} at {got[2]} mm/s; '
                           f'expected {want[0]}/{want[1]} at {want[2]} mm/s')
        return {'label': f'{got[0]}/{got[1]}', 'speed_mm_s': got[2], 't_before_write': tb,
                't_after_write': kw['t_after_write'], 't_receipt': got[3],
                'received_before_the_write_returned': got[3] < kw['t_after_write']}

    def _begin_hold(self, k, first_seq, why):
        self.stage, self.hold_first_seq, self.hold_started = 'hold', first_seq, False
        self.hold(k, why)

    # ------------------------------------------------------------------ script steps
    def select_level_1(self):
        """Row 1: one '+' while DISARMED with no goal, before arm: teleop 2/3 (internal level 1)."""
        st = self.st() or {}
        if (self.stage != 'pre' or self.armed or st.get('state') != 'DISARMED'
                or st.get('goal') is not None or self.max_seq != 0 or self.hold_key is not None):
            raise s2.Abort(f'the first level key only while DISARMED with no goal, before arm '
                           f'(stage {self.stage}, state {st.get("state")}, goal {st.get("goal")})')
        levels, strides = st.get('levels_m_s') or [], st.get('strides_m') or []
        if len(levels) != 3 or len(strides) != 3 or \
                any(abs(strides[k] - v) > 1e-6 for k, v in STRIDE_M.items()):
            raise s2.Abort(f'node levels {levels}, strides {strides}: internal levels 1 and 2 must '
                           'be the 40 and 60 mm strides of three')
        self.levels = list(levels)
        conf = self._press_level_key(1, 'select: + while DISARMED before arm (teleop 2/3, '
                                        'internal level 1)')
        self.selection = {'level': 1, 'teleop_label': '2/3', 'stride_m': strides[1],
                          'speed_m_s': levels[1], 'confirmations': [{'press': 1, **conf}]}
        self.log(selection=self.selection)

    def speed_change_while_walking(self):
        """Row 4: '+' while goal 2 (level 1, 1->2) is in flight. The teleop goes to 3/3 at once;
        the node keeps level 1 to boundary 0 and adopts level 2 there, walking (goal 10)."""
        st = self.wait('SC: goal 2 (level 1, 1->2) in flight', self.goal_in_flight(2),
                       2 * LONGEST + 2.0)
        if st.get('state') != 'WALKING' or self.hold_key != 'w':
            raise s2.Abort(f'speed change only while walking with w held (state '
                           f'{st.get("state")}, held {self.hold_key!r})')
        self.plus2_written = True
        conf = self._press_level_key(2, 'speed change: + while goal 2 is in flight (teleop 3/3; '
                                        'the node adopts level 2 at boundary 0)')
        self.speed_change = {'press': 2, 'goal_in_flight_at_write': 2,
                             'status_goal_before_write': st.get('goal'), **conf}
        self.log(speed_change=self.speed_change)

    def _stop_at_boundary_0(self, seq, name, n_checks):
        """Rows 5 and 7: space while goal `seq` is in flight; READY ('stopped') at boundary 0; the
        last cycle checked at a fresh sample before the next key."""
        self.stage = 'stopping'
        self.key(' ', f'{name}: space while goal {seq} is in flight (stop at boundary 0)')
        self.release(f'{name}: released after the space', step=f'{name}_space')
        self.wait(f'{name}: READY at boundary 0', self.ready_at(0), LONGEST + 2.0)
        self.stage = 'ready'
        self.wait(f'{name}: the last cycle checked at a fresh sample',
                  lambda st: len(self.cycle_log) >= n_checks, s4a.CYCLE_CHECK_SIM_S)
        time.sleep(1.5)                              # zeros after the hold window clear the latch

    def sc_and_forward_leg(self):
        """Rows 3-5: goals 1-54 in one hold, the speed change inside it."""
        self.expected_goals = 54
        self._begin_hold('w', 1, 'SC + F: hold w (one level-1 cycle, then five level-2 cycles)')
        self.speed_change_while_walking()
        self.wait('F: goal 54 (8->9) in flight', self.goal_in_flight(54),
                  CYCLE_S[1] + 5 * CYCLE_S[2])
        self._stop_at_boundary_0(54, 'F', 6)

    def return_leg(self):
        """Rows 6-7: goals 55-99."""
        self.expected_goals = 99
        self._begin_hold('s', 55, 'R: hold s (five level-2 reverse cycles)')
        self.wait('R: goal 99 (1->0) in flight', self.goal_in_flight(99), 5 * CYCLE_S[2])
        self._stop_at_boundary_0(99, 'R', 11)

    def x_release(self):
        """Rows 8-9: goals 100-104; released (no space) while 104 is in flight; READY at 5."""
        self.expected_goals = 104
        self._begin_hold('w', 100, 'X release: hold w')
        self.wait('X release: goal 104 (4->5) in flight', self.goal_in_flight(104),
                  5 * LONGEST + 2.0)
        self.stage = 'stopping'
        self.release('X release: w released while 4->5 is in flight (no space)', step='X_release')
        self.wait('X release: READY at boundary 5', self.ready_at(5), LONGEST + 2.0)
        self.stage = 'ready'
        time.sleep(1.5)

    def x_reversal(self):
        """Rows 10-12: goal 105 (5->6); 's' instead of 'w' while it is in flight (S4b's switch);
        goal 106 (6->5) at boundary 6; released while 106 is in flight; READY at 5."""
        self.expected_goals = 105
        self._begin_hold('w', 105, 'X reversal: hold w')
        self.wait('X reversal: goal 105 (5->6) in flight', self.goal_in_flight(105),
                  LONGEST + 2.0)
        self.expected_goals = 106
        self.switch_held_key('s', 'X reversal: hold s while 5->6 is in flight')
        self.wait('X reversal: goal 106 (6->5) in flight (the same phase played back)',
                  self.goal_in_flight(106), 2 * LONGEST + 2.0)
        self.stage = 'stopping'
        self.release('X reversal: s released while 6->5 is in flight (no space)',
                     step='X_reversal_release')
        self.wait('X reversal: READY at boundary 5', self.ready_at(5), LONGEST + 2.0)
        self.stage = 'ready'
        time.sleep(1.5)

    def x_home(self):
        """Row 13: 'h' from READY at 5: goals 107-110 forward, homed at 0; the composite cycle."""
        self.expected_goals = 110
        n0 = len(self.cycle_log)
        self.stage = 'home'
        self.key('h', 'X home: from READY at boundary 5 (four phases forward)')
        self.wait('X home: READY at boundary 0 (homed)', self.ready_at(0), 4 * LONGEST + 2.0)
        self.stage = 'ready'
        self.wait('X home: the composite cycle checked at a fresh sample',
                  lambda st: len(self.cycle_log) > n0, s4a.CYCLE_CHECK_SIM_S)
        time.sleep(1.5)

    def _await_teleop_exit(self, name, start):
        """The teleop's own exit within 15 s of `start`, code 0 (recorded)."""
        end = start + TELEOP_EXIT_WALL_S
        status = self._teleop_exit_status()
        while status is None:
            self.check()
            if time.monotonic() >= end:
                raise s2.Abort(f'{name}: the teleop was still running {TELEOP_EXIT_WALL_S:.0f} s '
                               'later')
            time.sleep(0.05)
            status = self._teleop_exit_status()
        code = os.waitstatus_to_exitcode(status) if isinstance(status, int) else None
        rec = {'event': 'teleop_closed', 'step': name, 'pid': self.pid,
               'wait_status': status if isinstance(status, int) else repr(status),
               'exit_code': code, 't_exit_seen': time.monotonic(), 't_bound_from': start}
        self.log(**rec)
        if code != 0:
            raise s2.Abort(f'{name}: the teleop exited with {code!r} (0 expected)')
        return rec

    def close_while_moving(self):
        """Rows 14-15: goal 111 (0->1); the repeat stops and 'q' follows 0.2 s later, inside the
        0.8 s hold window, so the close happens while moving."""
        self.expected_goals = EXPECTED_GOALS
        self._begin_hold('w', 111, 'Q: hold w')
        st = self.wait('Q: goal 111 (0->1) in flight', self.goal_in_flight(111), LONGEST + 2.0)
        d111 = (st.get('goal') or {}).get('duration_s')
        if isinstance(d111, bool) or not isinstance(d111, (int, float)):
            raise s2.Abort(f'goal 111 duration {d111!r} not reported')
        rec = self._stop_repeat_timed('Q: the repeat stops; q follows within the hold window')
        self.repeat_stop_before_q = rec
        self.log(repeat_stop_before_q=rec)
        time.sleep(REPEAT_QUIESCE_S)
        self.stage = 'closing'
        self.key('q', 'Q: q while goal 111 is in flight (the close while moving)')
        self.teleop_alive = False       # it reads no key after q: a stop now uses the estop service
        st = self.wait('Q: DISARMED with no goal', lambda st: st['state'] == 'DISARMED'
                       and st.get('goal') is None, d111 + SIM_MARGIN_S)
        self.stage = 'closed'
        self.close_record = {**self._await_teleop_exit('Q', time.monotonic()),
                             'disarmed_sim_s': (st.get('monitors') or {}).get('sim_time_s'),
                             'goal_111_duration_s': d111}
        self.log(close=self.close_record)
        return st

    def reconnect_probe(self):
        """Row 17 (OD11): a second teleop on the DISARMED node. It never presses 'a'."""
        st0 = self.st() or {}
        n0 = st0.get('commands_accepted')
        if st0.get('state') != 'DISARMED' or st0.get('goal') is not None or \
                isinstance(n0, bool) or not isinstance(n0, int):
            raise s2.Abort(f'probe only on the DISARMED node (state {st0.get("state")}, '
                           f'commands_accepted {n0!r})')
        self.stage, self.probe_phase = 'probe', 'start'
        self.probe = {'commands_accepted_before': n0}
        with self.lock:
            k0 = len(self.teleop_out)
        old_fd, t0 = self.fd, time.monotonic()
        self.spawn()
        self.probe['t_spawn'] = t0
        if old_fd is not None:      # the first teleop's pty (it has exited), closed only now so
            try:                    # that the new pty can never reuse its number
                os.close(old_fd)
            except OSError:
                pass

        def started(st):
            with self.lock:
                txt = ''.join(x for _, x in self.teleop_out[k0:])
            n = st.get('commands_accepted')
            return TELEOP_STATUS_LINE in txt and isinstance(n, int) and n > n0
        st = self.wait_wall('probe: the teleop shows a node status and the node counts its '
                            'heartbeats', started, PROBE_START_WALL_S)
        self.probe['started'] = {'t': time.monotonic(),
                                 'commands_accepted': st.get('commands_accepted')}
        self.probe['confirmations'] = [
            {'press': i, **self._press_level_key(i, f'probe: + ({i + 1}/3)',
                                                 PROBE_CONFIRM_WALL_S)} for i in (1, 2)]
        self.probe_phase = 'hold'
        self.hold('w', 'probe: hold w for 5 s on the DISARMED node (no a)')
        kw = self.key_writes[-1]
        end = kw['t_after_write'] + PROBE_HOLD_WALL_S
        while time.monotonic() < end:
            self.check()
            time.sleep(0.05)
        self.check()
        if self.probe.get('delivery') is None:
            raise s2.Abort('probe: no status showed the held w delivered (intent forward at level '
                           '2 with a live lease)')
        self.probe_phase = 'released'
        self.release('probe: w released after 5 s', step='probe_release')
        self.wait_wall('probe: intent back to stop after the release',
                       lambda st: self.probe.get('stopped_after_release') is not None,
                       PROBE_RELEASE_WALL_S)
        self.probe_phase = 'closing'
        self.key('q', 'probe: q (stop and disarm are accepted no-ops while DISARMED)')
        self.teleop_alive = False
        self.probe['close'] = self._await_teleop_exit('probe', self.key_writes[-1]['t_after_write'])
        n_end = self.status_n
        self.wait_wall('probe: statuses after the exit', lambda st: self.status_n >= n_end + 2,
                       PROBE_RELEASE_WALL_S)
        self.probe['commands_accepted_after'] = (self.st() or {}).get('commands_accepted')
        self.probe_phase = 'done'
        self.stage = 'done'
        self.log(probe=self.probe)

    def script(self, levels=None):
        if self.shadow_rehearsal:                     # before the teleop exists: no key yet
            st = self.wait('node status SHADOW with the dispatch gate False',
                           lambda st: st.get('mode') == 'shadow'
                           and st.get('dispatch_gate') is False, 30.0)
            self.shadow_verified = True
            self.log(shadow_rehearsal={'node_mode': st.get('mode'),
                                       'node_dispatch_gate': st.get('dispatch_gate'),
                                       'driver_gates': self.gates,
                                       'progress_sign_bound': SHADOW_SIGN_NOTE})
        self.spawn()
        self.wait('status with levels at level 0',
                  lambda st: bool(st.get('levels_m_s')) and st.get('level') == 0, 30.0)
        time.sleep(3.0)                               # the teleop receives the levels too
        self.settle()
        self.select_level_1()
        self.stage = 'arming'
        self.key('a', 'arm')
        self.wait('READY at boundary 0 after arm', self.ready_at(0), 5.0)
        self.armed, self.stage = True, 'ready'
        self.sc_and_forward_leg()
        self.return_leg()
        self.x_release()
        self.x_reversal()
        self.x_home()
        st = self.close_while_moving()
        self.observe_after_close(st)
        self.reconnect_probe()

    def summary(self):
        s = s4a.S4aDriver.summary(self)
        s.pop('estop_hold', None)                     # no estop: the hold is after the close
        s.update({
            'plan': [list(g) for g in PLANNED], 'expected_cancels': EXPECTED_CANCELS,
            'scripted_keys': SCRIPTED_KEYS, 'main_keys': MAIN_KEYS, 'probe_keys': PROBE_KEYS,
            'requests_before_q': list(REQUESTS_BEFORE_Q), 'stage': self.stage,
            'selection': self.selection, 'speed_change': self.speed_change,
            'releases': self.releases, 'switches': self.switches,
            'repeat_stop_before_q': self.repeat_stop_before_q, 'close': self.close_record,
            'close_observation': self.close_observation, 'probe': self.probe,
            'cumulative_reference_b9_live': self.b9_ref,
            'net_return_recorded_only': self.net_return,
            'limits': {'travel_max_m': TRAVEL_MAX_M, 'cumulative_lateral_stop_m':
                       CUM_LATERAL_STOP_M, 'cumulative_yaw_stop_rad': CUM_YAW_STOP_RAD,
                       'cycle_lateral_stop_m': LATERAL_MAX_M, 'cycle_yaw_stop_rad': YAW_MAX_RAD,
                       'cycle_stop_bands_m': {c[0]: list(c[3]) for c in CYCLE_CHECKS},
                       'script_cap_wall_s': SCRIPT_CAP_WALL_S,
                       'probe_wall_s': {'start': PROBE_START_WALL_S,
                                        'confirm': PROBE_CONFIRM_WALL_S,
                                        'hold': PROBE_HOLD_WALL_S,
                                        'release': PROBE_RELEASE_WALL_S,
                                        'exit': TELEOP_EXIT_WALL_S}}})
        return s


def run(d):
    """The script with its stop handling (S3's, through S4a's): (exit code, result)."""
    return s4a.run(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--domain-id', type=int, default=0)
    ap.add_argument('--shadow-rehearsal', action='store_true',
                    help='dispatch disabled: verify SHADOW throughout; the signed progress bounds '
                         'are not applied (see the module docstring)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    from spiderx_controller import joint_safety
    limits, _ = joint_safety.load_limits()
    gates = None
    if a.shadow_rehearsal:
        try:
            gates = {'fresh_interpreter': s3.fresh_gates(),
                     'driver_process': s3.imported_gates()}
            why = (s3.gate_refusal(gates['fresh_interpreter'])
                   or s3.gate_refusal(gates['driver_process']))
        except Exception as e:  # noqa: BLE001  (no gate evidence: refuse)
            why = f'gate import failed: {e!r}'
        if why:
            s4c.refuse(a.out, why, {'shadow_rehearsal': True, 'gates': gates})

    def interrupted(signum, frame):
        raise s3.Interrupted(signal.Signals(signum).name)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    d = S4dDriver(a.out, a.domain_id, limits, a.shadow_rehearsal, gates)
    rc, why = run(d)
    scope = (f'S4d-M shadow rehearsal: at most {EXPECTED_GOALS} planned shadow goals and 0 '
             'cancels, the close and the probe; dispatch disabled' if a.shadow_rehearsal else
             f'S4d-M: {EXPECTED_GOALS} goals and 0 cancels at most, the close while moving and the '
             'reconnect probe')
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
