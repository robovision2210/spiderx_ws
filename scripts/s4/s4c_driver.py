#!/usr/bin/env python3
"""S4c script driver (docs/S4_KEYBOARD_TELEOP_PLAN.md, section 3.3): first contact at ONE faster
level, selected explicitly for each invocation (--level 1 or --level 2).

Built on the S4a driver (scripts/s4/s4a_driver.py), which builds on the S3 and S2 drivers; all three
are unchanged. Reused as they are:
- the teleop pseudo-terminal and its key repeat (0.5 s delay, then 30 Hz);
- the key-write brackets: time.monotonic() immediately before and after os.write(), at full
  precision, with the write result;
- fresh samples, the 2-sample debounce, the settle check, the travel cap, the joint-limit and
  tracking stops, and the 10 s sim observation;
- the wall caps and the stop handling: any stop presses x. An emergency stop ends the attempt and
  is never suppressed to fit the budget;
- --shadow-rehearsal: the gates are checked in a fresh interpreter and in this process, and the
  check fails closed.
S4a's cycle check and cycle wait are level 0 only. They are restated here with the selected level's
stop bands and cycle time.

Levels: the CLI value is the node's INTERNAL level index. The teleop shows index + 1 of 3.
  --level 1   40 mm stride   teleop "speed level 2/3"   one '+'
  --level 2   60 mm stride   teleop "speed level 3/3"   two '+'
Level 0 (20 mm, teleop 1/3) belongs to S3, S4a and S4b and is refused here.

Level selection, derived from m55_teleop and m55_locomotion:
- '+' raises the teleop's own level (TeleopState.level, from 0), once the teleop has the node's
  levels. It prints "speed level <index + 1>/<n>: <speed> mm/s". No service is called and nothing
  moves.
- A held 'w' or 's' then publishes that level's speed. The node maps the speed to the level and
  switches only at a cycle boundary (b = 0), logging 'level_change' before it dispatches.
- So this script selects while DISARMED, before arm, and checks every confirmation. The node arms
  at level 0 (its default), and its one level_change (0 -> L) comes with goal 1.

Script for each level: 18 phase goals and 0 planned cancels. There is no warm-up goal, home,
estop, reset or recovery.
  settle   before arming, as S3: >= 10 s of sim, z >= 0.045 m, z range <= 0.005 m, tilt <= 0.10
  select   '+' x L while DISARMED, each confirmed by the teleop's own line
  arm      'a'; READY at boundary 0
  F        hold 'w': one FORWARD cycle at level L, goals 1-9 (0->1 ... 8->9). Space while 8->9 is
           in flight. READY ('stopped') at boundary 0, and the cycle checked at a fresh sample
  R        hold 's': one REVERSE return cycle, goals 10-18 (9->8 ... 1->0). Space while 1->0 is in
           flight. READY at boundary 0, checked at a fresh sample
  close    'q' at READY: the stopped-state teleop close (m55_teleop). The teleop calls stop and
           disarm, then stop and disarm again in its shutdown sequence, and exits 0. The node
           goes READY -> DISARMED ('disarm').
  observe  10 s of sim after DISARMED: still DISARMED, with no goal; measured as the S3 hold.
No key is written after 'q'.

Per-cycle checks, at the first fresh sample after the node's cycle count changes; the next key waits
for it. Measured along body +y in the frame of the cycle's start boundary, yaw wrapped. The reverse
return is measured from the end of the forward cycle; the net return is recorded only.
- Live stops: progress outside [0, 60] mm forward or [-60, 0] mm reverse at level 1, [0, 80] or
  [-80, 0] at level 2; lateral > 20 mm; |yaw| > 0.10 rad (S3, unchanged).
- Shadow (verified SHADOW, gates False): the signed bounds are not applied ("Signed progress
  condition not applicable: dispatch disabled."). |progress| above the level's outer bound (60 or
  80 mm), lateral and yaw still stop.

Stops added to every S3 and S4a stop:
- a goal beyond the 18, different from the plan (b_from, b_to, direction), or not at level L;
- the node's level other than 0 before goal 1, or other than L from goal 1 on; an intent level
  other than L;
- any FAULTED state (no planned cancel);
- a level key unless DISARMED with no goal before arm; a missing, stale (received before the '+'
  write began), duplicate or different confirmation. Each output chunk's receipt time is
  time.monotonic() right after os.read(), logged at full precision with the chunk; a line split
  across chunks is rebuilt, and receipt during the key's write bracket is accepted;
- after 'q': a goal, or a state other than READY or DISARMED; after DISARMED: anything but
  DISARMED with no goal; the teleop still running 15 s after DISARMED, or exiting non-zero.

Usage: s4c_driver.py --level {1,2} --out <dir> [--domain-id 0] [--shadow-rehearsal]
Exit: 0 script complete, 2 stop condition, 3 driver error, 4 interrupted, 5 refused before start."""
import argparse
import json
import math
import os
import re
import select
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import s4a_driver as s4a  # noqa: E402  (unchanged; brings the S3 and S2 drivers)
s3, s2 = s4a.s3, s4a.s2

P, FORWARD, REVERSE = s4a.P, s4a.FORWARD, s4a.REVERSE
LEVELS = {                     # internal index -> stride, teleop label, live stop bands (outside)
    1: {'stride_m': 0.040, 'teleop_label': '2/3',
        'stop_m': {FORWARD: (0.0, 0.060), REVERSE: (-0.060, 0.0)}},
    2: {'stride_m': 0.060, 'teleop_label': '3/3',
        'stop_m': {FORWARD: (0.0, 0.080), REVERSE: (-0.080, 0.0)}},
}
FORWARD_CYCLE = tuple((b, b + 1, FORWARD) for b in range(P))          # 0->1 ... 8->9
REVERSE_CYCLE = tuple((b, b - 1, REVERSE) for b in range(P, 0, -1))   # 9->8 ... 1->0
PLANNED = FORWARD_CYCLE + REVERSE_CYCLE           # goal seq k (1-based) is PLANNED[k - 1]
EXPECTED_GOALS, EXPECTED_CANCELS = len(PLANNED), 0                   # 18, 0
CLOSE_REQUESTS = ('stop', 'disarm', 'stop', 'disarm')   # m55_teleop: handle(q), then shutdown
CONFIRM_RE = re.compile(r'speed level (\d+)/(\d+): ([0-9.]+) mm/s')
UNKNOWN_LEVELS = 'speed levels unknown'           # m55_teleop, before it has the node's levels
CONFIRM_SIM_S = 2.0           # wait cap for one confirmation (wall: 2 / 0.1 + 60 s)
CLOSE_SIM_S = 2.0             # 'q' -> DISARMED (one request; wall cap 80 s)
TELEOP_EXIT_WALL_S = 15.0     # its shutdown waits at most 1 s for the replies
LONGEST = s2.LONGEST_GOAL_S
SHADOW_SIGN_NOTE = s4a.SHADOW_SIGN_NOTE


def scripted_keys(level):
    """The exact key writes of a complete script at `level` (repr, as the driver logs them)."""
    return ["'+'"] * level + ["'a'", "'w'", "' '", "'s'", "' '", "'q'"]


def outer_bound_m(level):
    lo, hi = LEVELS[level]['stop_m'][REVERSE][0], LEVELS[level]['stop_m'][FORWARD][1]
    return max(abs(lo), abs(hi))


class S4cDriver(s4a.S4aDriver):
    def __init__(self, out, domain_id, limits, level, shadow_rehearsal=False, gates=None):
        self._init_s4c(level)
        super().__init__(out, domain_id, limits, shadow_rehearsal, gates)

    def _init_s4c(self, level):
        if level not in LEVELS:
            raise ValueError(f'S4c runs level 1 or 2 (internal index), not {level!r}')
        self.level_index = level
        self.spec = LEVELS[level]
        self.teleop_out = []                # [(t_receipt, text)] as the pty delivered it
        self.selection = None
        self.stage = 'pre'                  # pre, selected, armed, closing, closed
        self.checked_max_seq = 0            # the highest goal seq among the statuses checked
        self.arm_ref = None
        self.net_return = None
        self.close_record = None
        self.close_observation = None

    # ------------------------------------------------------------------ the teleop's output
    def _pump(self, fd):
        """S2's pump (the teleop's output, logged), also kept for the level confirmations. Each
        chunk's receipt time is time.monotonic() immediately after os.read() returns, logged at
        full precision (the record's own 'mono' is rounded)."""
        while True:
            try:
                r, _, _ = select.select([fd], [], [], 0.1)
                if r:
                    data = os.read(fd, 4096)
                    t_receipt = time.monotonic()
                    if not data:
                        return
                    self._on_teleop_out(data.decode(errors='replace'), t_receipt)
            except (OSError, ValueError):
                return

    def _on_teleop_out(self, text, t_receipt):
        with self.lock:
            self.teleop_out.append((t_receipt, text))
        self.log(teleop_out=text, t_receipt=t_receipt)

    def _teleop_exit_status(self):
        """The teleop's wait status once it has exited, else None (the bench replaces only this)."""
        try:
            pid, status = os.waitpid(self.pid, os.WNOHANG)
        except ChildProcessError as e:
            return repr(e)
        return None if pid == 0 else status

    # ------------------------------------------------------------------ stops
    def _check_status(self, st):
        L, g = self.level_index, st.get('goal')
        if g is not None and isinstance(g.get('seq'), int):
            s = g['seq']
            got = (g.get('b_from'), g.get('b_to'), g.get('direction'))
            if s > EXPECTED_GOALS:
                raise s2.Abort(f'goal {s} {got} beyond the S4c plan ({EXPECTED_GOALS} goals)')
            if s >= 1 and got != PLANNED[s - 1]:
                raise s2.Abort(f'goal {s} is {got}, planned {PLANNED[s - 1]}')
            if g.get('level') != L:
                raise s2.Abort(f'goal {s} at level {g.get("level")}; the selected level is {L}')
            self.checked_max_seq = max(self.checked_max_seq, s)
        lvl = st.get('level')
        if self.checked_max_seq == 0 and lvl != 0:
            raise s2.Abort(f'node level {lvl} before goal 1 (it arms at level 0)')
        if self.checked_max_seq >= 1 and lvl != L:
            raise s2.Abort(f'node level {lvl} after goal 1; the selected level is {L}')
        il = (st.get('intent') or {}).get('level')
        if il is not None and il != L:
            raise s2.Abort(f'intent level {il}: the teleop is not at the selected level {L}')
        if st['state'] == 'FAULTED':
            raise s2.Abort(f'FAULTED {st.get("faults")} (S4c has no planned cancel)')
        if self.stage == 'closing' and (st['state'] not in ('READY', 'DISARMED')
                                        or g is not None):
            raise s2.Abort(f'after q: state {st["state"]}, goal {g} (the close only disarms)')
        if self.stage == 'closed' and (st['state'] != 'DISARMED' or g is not None):
            raise s2.Abort(f'after the close: state {st["state"]}, goal {g} (no new motion)')
        if self.shadow_rehearsal and (st.get('mode') != 'shadow'
                                      or st.get('dispatch_gate') is not False):
            raise s2.Abort(f'shadow rehearsal: node status mode {st.get("mode")!r}, dispatch_gate '
                           f'{st.get("dispatch_gate")!r} (SHADOW with the gate False required '
                           'throughout)')
        if st.get('command_rejections'):
            raise s2.Abort(f'command rejections {st["command_rejections"]}')
        if self.max_seq > min(self.expected_goals, EXPECTED_GOALS):
            raise s2.Abort(f'goal seq {self.max_seq} above the script ({self.expected_goals})')
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
                           f'{s3.TRAVEL_MAX_M}')
        self._cycle_check(st, gt, sim)

    def _cycle_check(self, st, gt, sim):
        pose = {'xy': tuple(gt['xy']), 'yaw': gt['yaw']}
        c = st.get('cycles')
        if self.cycle_ref is None:
            self.cycle_ref, self.cycles_checked, self.arm_ref = pose, c, pose
            self.log(cycle_reference=pose, cycles=c, sim=sim)
            return
        if c is None or c == self.cycles_checked:
            return
        dirn = self.cycle_direction
        if dirn not in (REVERSE, FORWARD) or c != self.cycles_checked + dirn:
            raise s2.Abort(f'cycle count {self.cycles_checked} -> {c}: not one '
                           f'{s4a.RUN_NAME.get(dirn, "scripted")} cycle')
        fwd, lat, dyaw = s3.body_delta(self.cycle_ref, pose)
        rec = {'cycle': len(self.cycle_log) + 1, 'direction': dirn, 'level': self.level_index,
               'cycles_count': c, 'sim': sim, 'forward_m': fwd, 'lateral_m': lat,
               'yaw_rad': dyaw}
        shadow = (self.shadow_verified and st.get('mode') == 'shadow'
                  and st.get('dispatch_gate') is False)
        if shadow:
            rec['progress_sign_bound'] = SHADOW_SIGN_NOTE
        if dirn == REVERSE and self.arm_ref is not None:
            f, la, y = s3.body_delta(self.arm_ref, pose)
            self.net_return = {'forward_m': f, 'lateral_m': la, 'yaw_rad': y,
                               'note': 'from the arm pose; recorded only, never a band'}
            rec['net_return_recorded_only'] = self.net_return
        self.cycle_log.append(rec)
        self.log(cycle_check=rec)
        self.cycle_ref, self.cycles_checked = pose, c
        lo, hi = self.spec['stop_m'][dirn]
        name = s4a.RUN_NAME[dirn]
        if shadow:
            if not abs(fwd) <= outer_bound_m(self.level_index):
                raise s2.Abort(f'{name} cycle moved {fwd * 1000:.1f} mm along body +y (shadow '
                               f'rehearsal: |progress| at most '
                               f'{outer_bound_m(self.level_index) * 1000:.0f})')
        elif not lo <= fwd <= hi:
            raise s2.Abort(f'{name} cycle moved {fwd * 1000:.1f} mm along body +y at level '
                           f'{self.level_index} (allowed {lo * 1000:.0f}..{hi * 1000:.0f})')
        if abs(lat) > s4a.LATERAL_MAX_M or abs(dyaw) > s4a.YAW_MAX_RAD:
            raise s2.Abort(f'{name} cycle drifted {lat * 1000:.1f} mm laterally, {dyaw:.4f} rad '
                           'yaw')

    # ------------------------------------------------------------------ script
    def select_level(self):
        """'+' x L while DISARMED with no goal, before arm; each confirmed by the teleop."""
        L = self.level_index
        st = self.st() or {}
        if (self.stage != 'pre' or self.armed or st.get('state') != 'DISARMED'
                or st.get('goal') is not None or self.max_seq != 0
                or self.hold_key is not None):
            raise s2.Abort(f'level keys only while DISARMED with no goal, before arm (stage '
                           f'{self.stage}, state {st.get("state")}, goal {st.get("goal")})')
        levels, strides = st.get('levels_m_s') or [], st.get('strides_m') or []
        n = len(levels)
        if n <= L or len(strides) != n or abs(strides[L] - self.spec['stride_m']) > 1e-6 \
                or f'{L + 1}/{n}' != self.spec['teleop_label']:
            raise s2.Abort(f'node levels {levels}, strides {strides}: level {L} must be the '
                           f'{self.spec["stride_m"] * 1000:.0f} mm stride, teleop label '
                           f'{self.spec["teleop_label"]}')
        confirmations = []
        for i in range(1, L + 1):
            want = (str(i + 1), str(n), f'{levels[i] * 1000:.2f}')
            with self.lock:
                k0 = len(self.teleop_out)
            self.key('+', f'select level {L} (internal index; teleop label '
                          f'{self.spec["teleop_label"]}): + {i} of {L}')
            kw = self.key_writes[-1]                # this press's full-precision write bracket
            tb = kw['t_before_write']
            self.wait(f'teleop confirms speed level {i + 1}/{n}',
                      lambda st, k0=k0, tb=tb: self._confirmation(k0, tb) is not None,
                      CONFIRM_SIM_S)
            got = self._confirmation(k0, tb)
            if got[:3] != want:
                raise s2.Abort(f'teleop confirmed speed level {got[0]}/{got[1]} at {got[2]} mm/s; '
                               f'expected {want[0]}/{want[1]} at {want[2]} mm/s')
            confirmations.append({'press': i, 'label': f'{got[0]}/{got[1]}',
                                  'speed_mm_s': got[2], 't_before_write': tb,
                                  't_after_write': kw['t_after_write'], 't_receipt': got[3],
                                  'received_before_the_write_returned':
                                  got[3] < kw['t_after_write']})
        self.selection = {'level': L, 'teleop_label': self.spec['teleop_label'],
                          'stride_m': strides[L], 'speed_m_s': levels[L],
                          'confirmations': confirmations}
        self.stage = 'selected'
        self.log(selection=self.selection)

    def _confirmation(self, k0, t_before):
        """The one confirmation line in the teleop's output from chunk k0 on (a cursor, never a
        time), rebuilt across chunks: (label index, count, speed, receipt time of the chunk that
        completed it), or None while there is none. Received during the key's write bracket is
        fine; received before the write began is stale. Stale, duplicate or unknown-level output
        stops the script."""
        with self.lock:
            chunks = list(self.teleop_out[k0:])
        txt = ''.join(x for _, x in chunks)
        if UNKNOWN_LEVELS in txt:
            raise s2.Abort('the teleop had no speed levels yet: the level key was not applied')
        ms = list(CONFIRM_RE.finditer(txt))
        if len(ms) > 1:
            raise s2.Abort(f'{len(ms)} level confirmations for one key: '
                           f'{[m.group(0) for m in ms]}')
        if not ms:
            return None
        end, t_receipt = 0, None
        for t, x in chunks:
            end += len(x)
            if end >= ms[0].end():
                t_receipt = t
                break
        if not isinstance(t_receipt, (int, float)) or t_receipt < t_before:
            raise s2.Abort(f'stale level confirmation: received at {t_receipt!r}, before the + '
                           f'write began at {t_before!r}')
        return ms[0].group(1), ms[0].group(2), ms[0].group(3), t_receipt

    def run_cycle(self, direction, why):
        """S4a's run_cycle with the selected level's cycle time: one cycle in `direction`, stopped
        at its boundary (space during its last phase). Returns only once this cycle has been
        checked at a fresh sample; without one the wait's wall cap stops the script."""
        self.expected_goals += P
        self.cycle_direction = direction
        c0, n0 = self.st()['cycles'], len(self.cycle_log)
        self.hold('s' if direction == REVERSE else 'w', why)
        self.wait(f'{why}: last phase in flight',
                  lambda st: st['cycles'] == c0 and self.last_phase_in_flight(direction)(st),
                  s2.CYCLE_S[self.level_index])
        self.key(' ', f'{why}: stop at the cycle boundary')
        self.release(why)
        self.wait(f'{why}: READY at boundary 0', self.ready_at(0), LONGEST + 2.0)
        self.wait(f'{why}: cycle checked at a fresh sample',
                  lambda st: len(self.cycle_log) > n0, s4a.CYCLE_CHECK_SIM_S)
        time.sleep(1.5)                              # zeros after the hold window clear the latch

    def close_teleop(self):
        """'q' at READY at boundary 0, after both cycles were checked: the stopped-state close.
        Waits for DISARMED and for the teleop's own exit (recorded, 0 required)."""
        st = self.st() or {}
        if not (st.get('state') == 'READY' and st.get('boundary') == 0 and st.get('goal') is None
                and len(self.cycle_log) == 2 and self.hold_key is None):
            raise s2.Abort(f'close only at READY at boundary 0 after both cycle checks (state '
                           f'{st.get("state")}, boundary {st.get("boundary")}, cycles checked '
                           f'{len(self.cycle_log)})')
        self.stage = 'closing'
        self.key('q', 'close: q at READY (the stopped-state teleop close: stop, disarm, exit)')
        st = self.wait('close: DISARMED with no goal',
                       lambda st: st['state'] == 'DISARMED' and st.get('goal') is None,
                       CLOSE_SIM_S)
        self.stage = 'closed'
        end = time.monotonic() + TELEOP_EXIT_WALL_S
        status = self._teleop_exit_status()
        while status is None:
            self.check()
            if time.monotonic() >= end:
                raise s2.Abort(f'the teleop was still running {TELEOP_EXIT_WALL_S:.0f} s after '
                               'the close')
            time.sleep(0.05)
            status = self._teleop_exit_status()
        self.teleop_alive = False                   # a stop now uses the node's estop service
        code = os.waitstatus_to_exitcode(status) if isinstance(status, int) else None
        self.close_record = {'event': 'teleop_closed', 'pid': self.pid,
                             'wait_status': status if isinstance(status, int) else repr(status),
                             'exit_code': code, 't_exit_seen': time.monotonic(),
                             'disarmed_sim_s': (st.get('monitors') or {}).get('sim_time_s')}
        self.log(**self.close_record)
        if code != 0:
            raise s2.Abort(f'the teleop exited with {code!r} after q (0 expected)')
        return st

    def observe_after_close(self, st):
        """10 s of sim after DISARMED, measured as the S3 hold (no key, no command)."""
        self.estop_sim = (st.get('monitors') or {}).get('sim_time_s')
        self.hold_observation()
        last = self.st() or {}
        self.close_observation = {**(self.hold_summary or {}), 'state_after': last.get('state'),
                                  'goals_recorded_after': last.get('goals_recorded'),
                                  'max_seq': self.max_seq}
        self.log(close_observation=self.close_observation)

    def script(self, levels=None):
        L = self.level_index
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
        self.select_level()
        self.key('a', 'arm')
        self.wait('READY at boundary 0 after arm', self.ready_at(0), 5.0)
        self.armed, self.stage = True, 'armed'
        self.run_cycle(FORWARD, f'run F: one forward cycle at level {L}')
        self.run_cycle(REVERSE, f'run R: one reverse return cycle at level {L}')
        st = self.close_teleop()
        self.observe_after_close(st)

    def summary(self):
        s = super().summary()
        s.pop('estop_hold', None)                     # no estop: the hold is after the close
        s.update({'level': self.level_index, 'level_spec': {
                      'stride_m': self.spec['stride_m'], 'teleop_label': self.spec['teleop_label'],
                      'stop_m': {'forward': list(self.spec['stop_m'][FORWARD]),
                                 'reverse': list(self.spec['stop_m'][REVERSE])}},
                  'plan': [list(g) for g in PLANNED], 'expected_cancels': EXPECTED_CANCELS,
                  'scripted_keys': scripted_keys(self.level_index), 'selection': self.selection,
                  'stage': self.stage, 'close': self.close_record,
                  'close_observation': self.close_observation,
                  'net_return_recorded_only': self.net_return})
        return s


def run(d):
    """The script with its stop handling (S3's, through S4a's): (exit code, result)."""
    return s4a.run(d)


def refuse(out, why, extra=None):
    summary = {'result': f'REFUSED: {why}', **(extra or {})}
    with open(os.path.join(out, 'driver_summary.json'), 'w') as f:
        json.dump(summary, f, indent=1, sort_keys=True, default=str)
    print(json.dumps(summary, sort_keys=True, default=str), flush=True)
    sys.exit(5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--level', type=int,
                    help='internal level index: 1 (40 mm stride, teleop 2/3) or 2 (60 mm, 3/3)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--domain-id', type=int, default=0)
    ap.add_argument('--shadow-rehearsal', action='store_true',
                    help='dispatch disabled: verify SHADOW throughout; the signed progress bounds '
                         'are not applied (see the module docstring)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.level not in LEVELS:
        refuse(a.out, f'--level must be 1 or 2 (internal index), got {a.level!r}',
               {'level': a.level})
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
            refuse(a.out, why, {'shadow_rehearsal': True, 'gates': gates, 'level': a.level})

    def interrupted(signum, frame):
        raise s3.Interrupted(signal.Signals(signum).name)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    d = S4cDriver(a.out, a.domain_id, limits, a.level, a.shadow_rehearsal, gates)
    rc, why = run(d)
    scope = (f'S4c shadow rehearsal: level {a.level} only, one forward and one reverse cycle, the '
             f'close; at most {EXPECTED_GOALS} planned shadow goals and 0 cancels; dispatch '
             'disabled' if a.shadow_rehearsal else
             f'S4c: level {a.level} only, one forward cycle, one reverse return cycle, the close; '
             f'{EXPECTED_GOALS} goals and 0 cancels at most')
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
