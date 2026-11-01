#!/usr/bin/env python3
"""S4a script driver (docs/S4_KEYBOARD_TELEOP_PLAN.md, S4a): level 0, first contact in REVERSE.

Built on the S3 driver (scripts/s3/s3_driver.py, unchanged), which is built on the S2 driver
(scripts/s2/s2_driver.py, unchanged). Reused as they are:
- the teleop pseudo-terminal and its key repeat (0.5 s delay, then 30 Hz);
- the status and /joint_states subscriptions;
- fresh samples, the 2-sample debounce, the settle check, the travel cap and the joint-limit stop;
- the tracking abort and finding;
- the estop exercise and the 10 s sim hold observation;
- the wall caps, the stop handling and --shadow-rehearsal (gates and mode).

Script, at most 19 goals and 1 cancel; level 0 only; no other level, extra cycle, recovery or
retry:
  settle  before arming (as S3: >= 10 s of sim, z >= 0.045 m, z range <= 0.005 m, tilt <= 0.10)
  arm     'a'; READY at boundary 0 (the status must report level 0)
  R       hold 's': one REVERSE cycle (goals 9->8 ... 1->0); space during its last phase
          (b_to = 0); READY at boundary 0. The node then reports cycles - 1
  F       hold 'w': one FORWARD return cycle (0->1 ... 8->9); space during its last phase
          (b_to = 9); READY at boundary 0. The node then reports cycles + 1
  E       hold 'w', 'x' while that goal (0->1) is in flight (the 1 cancel); the 10 s sim hold;
          then 'q'

Per-cycle checks at the first fresh sample after the node's cycle count changes, in the frame of the
previous boundary's yaw (body +y forward; yaw wrapped):
- the count must change by exactly the run's direction (-1 for R, +1 for F);
- the script waits for that check before its next key, so a cycle is never judged in the next
  cycle's direction; if no fresh sample comes, the wait's wall cap stops the script;
- live stops: progress along body +y outside [-40, 0] mm (reverse) or [0, 40] mm (forward);
  lateral > 20 mm; |yaw| > 0.10 rad;
- shadow rehearsal (verified SHADOW, gates False): no trajectory runs, so the signed bounds are not
  applied ("Signed progress condition not applicable: dispatch disabled."). |progress| > 40 mm,
  lateral and yaw still stop. Displacement is always the measured one, never the planned one.
All other stops are S3's, unchanged.

Key writes: each scripted key is logged with time.monotonic() taken immediately BEFORE and AFTER
os.write(), at full precision, and with the write result (bytes written, or the error). The
record's own 'mono' is the post-write log time. The held key's repeats are unchanged and not
logged one by one.

Usage: s4a_driver.py --out <dir> [--domain-id 0] [--shadow-rehearsal]
Exit: 0 script complete, 2 stop condition, 3 driver error, 4 interrupted, 5 refused before start."""
import argparse
import json
import math
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..', 's3'), os.path.join(HERE, '..', 's2')]
import s3_driver as s3  # noqa: E402
s2 = s3.s2

P = s3.P
EXPECTED_GOALS = 2 * P + 1                 # 19: run R (9), run F (9), the estop goal (1)
REVERSE, FORWARD = -1, 1
STOP_BAND_M = {REVERSE: (-0.040, 0.0), FORWARD: (0.0, 0.040)}       # live stop: outside
SHADOW_MAX_ABS_M = 0.040                   # shadow: |progress| above this still stops
LATERAL_MAX_M, YAW_MAX_RAD = s3.LATERAL_MAX_M, s3.YAW_MAX_RAD          # 0.020 m, 0.10 rad
SHADOW_SIGN_NOTE = 'Signed progress condition not applicable: dispatch disabled.'
RUN_NAME = {REVERSE: 'reverse', FORWARD: 'forward'}
CYCLE_CHECK_SIM_S = 2 * s3.POSE_STALE_S    # at rest after a cycle: wait for its fresh sample


class S4aDriver(s3.S3Driver):
    def __init__(self, out, domain_id, limits, shadow_rehearsal=False, gates=None):
        self._init_s4a()
        super().__init__(out, domain_id, limits, shadow_rehearsal, gates)

    def _init_s4a(self):
        self.cycle_direction = None            # the running cycle's direction (set by the script)
        self.key_writes = []

    # ------------------------------------------------------------------ keys
    def _write(self, data):
        """The pty write (the bench replaces only this). Returns the bytes written."""
        return os.write(self.fd, data)

    def _press(self, k, why):
        t_before = time.monotonic()
        try:
            n = self._write(k.encode())
        except OSError as e:
            t_after = time.monotonic()
            self.key_writes.append({'key': repr(k), 't_before_write': t_before,
                                    't_after_write': t_after, 'write_result': repr(e)})
            self.log(key=repr(k), why=why, t_before_write=t_before, t_after_write=t_after,
                     write_result=repr(e))
            raise
        t_after = time.monotonic()
        self.key_writes.append({'key': repr(k), 't_before_write': t_before,
                                't_after_write': t_after, 'write_result': n})
        self.log(key=repr(k), why=why, t_before_write=t_before, t_after_write=t_after,
                 write_result=n)

    # ------------------------------------------------------------------ stops
    def _check_status(self, st):
        """S3's checks, with the cycle check made direction-aware (S3's is forward-only)."""
        if self.shadow_rehearsal and (st.get('mode') != 'shadow'
                                      or st.get('dispatch_gate') is not False):
            raise s2.Abort(f'shadow rehearsal: node status mode {st.get("mode")!r}, dispatch_gate '
                           f'{st.get("dispatch_gate")!r} (SHADOW with the gate False required '
                           'throughout)')
        if st.get('command_rejections'):
            raise s2.Abort(f'command rejections {st["command_rejections"]}')
        if st['state'] == 'FAULTED' and not self.planned_estop:
            raise s2.Abort(f'unplanned FAULTED: {st.get("faults")}')
        if self.max_seq > min(self.expected_goals, EXPECTED_GOALS):
            raise s2.Abort(f'goal seq {self.max_seq} above the script ({self.expected_goals})')
        if st.get('level') not in (None, 0):
            raise s2.Abort(f'level {st.get("level")} reported: S4a is level 0 only')
        self._tracking(st.get('goal'))
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
                           f'{s3.TRAVEL_MAX_M}')
        pose = {'xy': tuple(gt['xy']), 'yaw': gt['yaw']}
        c = st.get('cycles')
        if self.cycle_ref is None:
            self.cycle_ref, self.cycles_checked = pose, c
            self.log(cycle_reference=pose, cycles=c, sim=sim)
            return
        if c is None or c == self.cycles_checked:
            return
        dirn = self.cycle_direction
        if dirn not in (REVERSE, FORWARD) or c != self.cycles_checked + dirn:
            raise s2.Abort(f'cycle count {self.cycles_checked} -> {c}: not one '
                           f'{RUN_NAME.get(dirn, "scripted")} cycle')
        fwd, lat, dyaw = s3.body_delta(self.cycle_ref, pose)
        rec = {'cycle': len(self.cycle_log) + 1, 'direction': dirn, 'cycles_count': c,
               'sim': sim, 'forward_m': fwd, 'lateral_m': lat, 'yaw_rad': dyaw}
        shadow = (self.shadow_verified and st.get('mode') == 'shadow'
                  and st.get('dispatch_gate') is False)
        if shadow:
            rec['progress_sign_bound'] = SHADOW_SIGN_NOTE
        self.cycle_log.append(rec)
        self.log(cycle_check=rec)
        self.cycle_ref, self.cycles_checked = pose, c
        lo, hi = STOP_BAND_M[dirn]
        if shadow:
            if not abs(fwd) <= SHADOW_MAX_ABS_M:
                raise s2.Abort(f'{RUN_NAME[dirn]} cycle moved {fwd * 1000:.1f} mm along body +y '
                               f'(shadow rehearsal: |progress| at most '
                               f'{SHADOW_MAX_ABS_M * 1000:.0f})')
        elif not lo <= fwd <= hi:
            raise s2.Abort(f'{RUN_NAME[dirn]} cycle moved {fwd * 1000:.1f} mm along body +y '
                           f'(allowed {lo * 1000:.0f}..{hi * 1000:.0f})')
        if abs(lat) > LATERAL_MAX_M or abs(dyaw) > YAW_MAX_RAD:
            raise s2.Abort(f'{RUN_NAME[dirn]} cycle drifted {lat * 1000:.1f} mm laterally, '
                           f'{dyaw:.4f} rad yaw')

    # ------------------------------------------------------------------ script
    def run_cycle(self, direction, why):
        """One cycle in `direction`, stopped at its boundary (space during its last phase). Returns
        only once this cycle has been checked at a fresh sample, so it is never judged in the next
        cycle's direction; without a fresh sample the wait's wall cap stops the script."""
        self.expected_goals += P
        self.cycle_direction = direction
        c0, n0 = self.st()['cycles'], len(self.cycle_log)
        self.hold('s' if direction == REVERSE else 'w', why)
        self.wait(f'{why}: last phase in flight',
                  lambda st: st['cycles'] == c0 and self.last_phase_in_flight(direction)(st),
                  s3.CYCLE_S0)
        self.key(' ', f'{why}: stop at the cycle boundary')
        self.release(why)
        self.wait(f'{why}: READY at boundary 0', self.ready_at(0), s2.LONGEST_GOAL_S + 2.0)
        self.wait(f'{why}: cycle checked at a fresh sample',
                  lambda st: len(self.cycle_log) > n0, CYCLE_CHECK_SIM_S)
        time.sleep(1.5)                              # zeros after the hold window clear the latch

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
        self.key('a', 'arm')
        self.wait('READY at boundary 0 after arm', self.ready_at(0), 5.0)
        self.armed = True
        self.run_cycle(REVERSE, 'run R: one reverse cycle')
        self.run_cycle(FORWARD, 'run F: one forward return cycle')
        self.cycle_direction = FORWARD                # the estop goal is 0 -> 1; no cycle ends
        self.estop_exercise()
        self.hold_observation()
        self.key('q', 'quit teleop')
        time.sleep(3.0)

    def summary(self):
        s = super().summary()
        s['key_writes'] = self.key_writes
        s.pop('shadow_rehearsal', None)
        if self.shadow_rehearsal:
            s['shadow_rehearsal'] = {
                'driver_gates': self.gates, 'node_mode_verified': self.shadow_verified,
                'progress_sign_bound': SHADOW_SIGN_NOTE,
                'note': 'dispatch disabled: no trajectory was sent; per-cycle displacements are '
                        'measured and recorded, not acceptance'}
        return s


def run(d):
    """The script with its stop handling (S3's): (exit code, result)."""
    rc, why = s3.run(d)
    if rc == 0 and d.shadow_rehearsal:
        why = 'script complete (shadow rehearsal: dispatch disabled; not acceptance)'
    return rc, why


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
            summary = {'result': f'REFUSED: {why}', 'shadow_rehearsal': True, 'gates': gates}
            with open(os.path.join(a.out, 'driver_summary.json'), 'w') as f:
                json.dump(summary, f, indent=1, sort_keys=True, default=str)
            print(json.dumps(summary, sort_keys=True, default=str), flush=True)
            sys.exit(5)

    def interrupted(signum, frame):
        raise s3.Interrupted(signal.Signals(signum).name)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    d = S4aDriver(a.out, a.domain_id, limits, a.shadow_rehearsal, gates)
    rc, why = run(d)
    scope = ('S4a shadow rehearsal: level 0, one reverse and one forward cycle, at most 19 planned '
             'shadow goals and 1 shadow cancel; dispatch disabled' if a.shadow_rehearsal else
             'S4a: level 0, one reverse cycle, one forward return cycle, the estop; 19 goals and '
             '1 cancel at most')
    summary = {'result': why, 'scope': scope, **d.summary()}
    with open(os.path.join(a.out, 'driver_summary.json'), 'w') as f:
        json.dump(summary, f, indent=1, sort_keys=True, default=str)
    d.log(summary=summary)
    print(json.dumps(summary, sort_keys=True, default=str), flush=True)
    if d.pid:
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
