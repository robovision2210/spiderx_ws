#!/usr/bin/env python3
"""S4b script driver (docs/S4_KEYBOARD_TELEOP_PLAN.md, S4b): operator input on contact, level 0.

Built on the S4a driver (scripts/s4/s4a_driver.py), which builds on the S3 and S2 drivers; all
three are unchanged. Reused as they are:
- the teleop pseudo-terminal and its key repeat (0.5 s delay, then 30 Hz);
- the key-write brackets: time.monotonic() immediately before and after os.write(), with the
  result;
- fresh samples, the debounce, the settle check;
- the direction-aware cycle check and its fresh-sample wait;
- the travel cap, the joint-limit and tracking stops;
- the wall caps, the stop handling and --shadow-rehearsal (gates and mode, fail closed).

Script: level 0 only; 12 phase goals and 0 planned cancels. It has no estop exercise and never
resets or re-arms. It was derived from m55_locomotion (release = zero intent -> STOPPING
'intent_stop'; reversal at the next phase boundary; home = the fewest phases, so forward from
boundary 5; lease expiry -> STOPPING 'lease_expired' -> DISARMED) and checked on the in-memory
bench:
  settle, arm  'a'; READY at boundary 0
  1 release    hold 'w': goals 1-5, 0->1 ... 4->5. While 4->5 is in flight (2.8 s sim, the
               longest level-0 phase), the key is RELEASED, with no space. The teleop's hold
               window expires and the node stops after the phase ('intent_stop'): READY at
               boundary 5.
  2 reversal   hold 'w': goal 6, 5->6. While it is in flight, the repeat stops and 's' is held
               0.2 s later (within the 0.8 s hold window, so the motion never lapses). The node
               reverses at the next phase boundary and plays the same phase back: goal 7, 6->5.
               Released while 6->5 is in flight: READY at boundary 5.
  3 home       'h' at READY: the fewest phases to the cycle boundary, forward from boundary 5:
               goals 8-11, 5->6 ... 8->9. READY at boundary 0 ('homed').
               Exercises 1-3 make one complete forward cycle: the node's cycle count changes
               once, by +1, and the S4a forward cycle check and its bands apply.
  4 lease loss hold 'w': goal 12, 0->1. While it is in flight, the teleop is killed with
               SIGKILL (no clean quit). The lease expires and the node stops after the phase
               ('lease_expired'), then disarms: DISARMED at boundary 1. No key is written after
               the kill.
  observe      10 s of sim after DISARMED: still DISARMED, no goal, no re-arm. Joint and body
               motion are measured as in the S3 hold.

Determinism. Every release, and the kill, happens while a goal is in flight, as soon as the
status shows it (statuses come every 0.5 s wall).
- Release detection takes 0.8-0.95 s wall after the last repeated character; lease detection at
  most 0.55 s after the last command (M5.5 section 16.1).
- The chosen phases (2.8, 2.5 and 1.7 s sim) end later than that up to an RTF of about 1.6. The
  Cloud runs had 0.17-0.64.
- An extra goal, from a boundary inside a detection window, is never hidden. The count is exact,
  and any goal beyond it or different from the plan stops the script.
- The emergency stop then sent is a safety response, never part of the budget.

Stops added to every S4a and S3 stop (all unchanged):
- a goal different from the plan (b_from, b_to, direction), or beyond it;
- after the kill: READY, or any state other than WALKING/STARTING (until the lease expires),
  STOPPING or DISARMED;
- after DISARMED: any goal in flight, or any state other than DISARMED;
- any FAULTED state: S4b has no planned cancel.

Usage: s4b_driver.py --out <dir> [--domain-id 0] [--shadow-rehearsal]
Exit: 0 script complete, 2 stop condition, 3 driver error, 4 interrupted, 5 refused before start."""
import argparse
import json
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import s4a_driver as s4a  # noqa: E402  (unchanged; brings the S3 and S2 drivers)
s3, s2 = s4a.s3, s4a.s2

P, FORWARD, REVERSE = s4a.P, s4a.FORWARD, s4a.REVERSE
PLAN = (                                           # (exercise, ((b_from, b_to, direction), ...))
    ('release', ((0, 1, 1), (1, 2, 1), (2, 3, 1), (3, 4, 1), (4, 5, 1))),
    ('reversal', ((5, 6, 1), (6, 5, -1))),
    ('home', ((5, 6, 1), (6, 7, 1), (7, 8, 1), (8, 9, 1))),
    ('lease', ((0, 1, 1),)),
)
PLANNED = [g for _, gs in PLAN for g in gs]       # goal seq k (1-based) is PLANNED[k - 1]
FIRST, _k = {}, 1                                   # the first goal seq of each exercise
for _name, _gs in PLAN:
    FIRST[_name], _k = _k, _k + len(_gs)            # release 1, reversal 6, home 8, lease 12
EXPECTED_GOALS, EXPECTED_CANCELS = len(PLANNED), 0  # 12, 0
REPEAT_QUIESCE_S = 0.2       # after stopping the repeat: no stale character can follow (< 0.8 s)
OBSERVE_SIM_S = s3.HOLD_SIM_S                       # 10 s of sim after DISARMED
LONGEST = s2.LONGEST_GOAL_S


class S4bDriver(s4a.S4aDriver):
    def __init__(self, out, domain_id, limits, shadow_rehearsal=False, gates=None):
        self._init_s4b()
        super().__init__(out, domain_id, limits, shadow_rehearsal, gates)

    def _init_s4b(self):
        self.releases, self.switches, self.kills = [], [], []
        self.lease_phase = None                     # None, 'killed', 'disarmed'
        self.lease_observation = None

    # ------------------------------------------------------------------ keys and the teleop
    def _stop_repeat(self):
        """Stop the emulated key repeat (the bench replaces only this)."""
        self.hold_key = None

    def _stop_repeat_timed(self, why):
        key = self.hold_key
        t_before = time.monotonic()
        self._stop_repeat()
        t_after = time.monotonic()
        return {'key': repr(key), 'why': why, 't_before_stop': t_before, 't_after_stop': t_after}

    def release(self, why):
        """S2's release (the repeat stops; nothing is written), with its time recorded."""
        rec = self._stop_repeat_timed(why)
        self.releases.append(rec)
        self.log(release=rec['key'], **{k: v for k, v in rec.items() if k != 'key'})

    def switch_held_key(self, k, why):
        """Hold k instead of the held key, without a release: the repeat stops, and
        REPEAT_QUIESCE_S later (well inside the 0.8 s hold window) k is written, so no stale
        character of the old key can follow it and the motion never lapses."""
        rec = self._stop_repeat_timed(why)
        self.switches.append(rec)
        self.log(switch=rec['key'], to=repr(k), **{k2: v for k2, v in rec.items() if k2 != 'key'})
        time.sleep(REPEAT_QUIESCE_S)
        self.hold(k, why)

    def _signal_teleop(self):
        """SIGKILL to the teleop's whole pty session (the bench replaces only this)."""
        os.killpg(self.pid, signal.SIGKILL)

    def _reap_teleop(self):
        try:
            os.waitpid(self.pid, 0)
        except ChildProcessError:
            pass

    def kill_teleop(self, why='lease: SIGKILL'):
        """No clean quit: the teleop dies, its heartbeats stop and the node's lease expires.
        time.monotonic() is recorded immediately before and after the kill."""
        self.hold_key = None
        t_before = time.monotonic()
        try:
            self._signal_teleop()
        except OSError as e:
            t_after = time.monotonic()
            rec = {'event': 'teleop_kill_failed', 'why': why, 'pid': self.pid,
                   't_before_kill': t_before, 't_after_kill': t_after, 'result': repr(e)}
            self.kills.append(rec)
            self.log(**rec)
            raise
        t_after = time.monotonic()
        self.teleop_alive = False                   # a stop now uses the node's estop service
        self._reap_teleop()
        rec = {'event': 'teleop_killed', 'why': why, 'pid': self.pid, 'signal': 'SIGKILL',
               't_before_kill': t_before, 't_after_kill': t_after, 'result': 'delivered',
               't_reaped': time.monotonic()}
        self.kills.append(rec)
        self.log(**rec)

    # ------------------------------------------------------------------ stops
    def _check_status(self, st):
        g = st.get('goal')
        if g is not None and isinstance(g.get('seq'), int):
            s = g['seq']
            got = (g.get('b_from'), g.get('b_to'), g.get('direction'))
            if s > EXPECTED_GOALS:
                raise s2.Abort(f'goal {s} {got} beyond the S4b plan ({EXPECTED_GOALS} goals)')
            if s >= 1 and got != PLANNED[s - 1]:
                raise s2.Abort(f'goal {s} is {got}, planned {PLANNED[s - 1]}')
        if self.lease_phase == 'killed' and st['state'] not in (
                'WALKING', 'STARTING', 'STOPPING', 'DISARMED'):
            raise s2.Abort(f'after the teleop kill: state {st["state"]} (the lease path must '
                           'stop and disarm)')
        if self.lease_phase == 'disarmed' and (st['state'] != 'DISARMED'
                                               or st.get('goal') is not None):
            raise s2.Abort(f'after lease loss: state {st["state"]}, goal {st.get("goal")} '
                           '(no dispatch or re-arm is allowed)')
        super()._check_status(st)

    # ------------------------------------------------------------------ script
    @staticmethod
    def goal_in_flight(seq):
        want = PLANNED[seq - 1]

        def pred(st):
            g = st.get('goal')
            return g is not None and g.get('seq') == seq and \
                (g.get('b_from'), g.get('b_to'), g.get('direction')) == want
        return pred

    def release_exercise(self):
        k = FIRST['release']
        self.expected_goals += 5
        self.hold('w', 'release: hold w')
        self.wait('release: goal 4->5 in flight', self.goal_in_flight(k + 4), 5 * LONGEST + 2.0)
        self.release('release: w released while 4->5 is in flight (no space)')
        self.wait('release: READY at boundary 5', self.ready_at(5), LONGEST + 2.0)
        time.sleep(1.5)

    def reversal_exercise(self):
        k = FIRST['reversal']
        self.expected_goals += 2
        self.hold('w', 'reversal: hold w')
        self.wait('reversal: goal 5->6 in flight', self.goal_in_flight(k), LONGEST + 2.0)
        self.switch_held_key('s', 'reversal: hold s while 5->6 is in flight')
        self.wait('reversal: goal 6->5 in flight (the same phase played back)',
                  self.goal_in_flight(k + 1), 2 * LONGEST + 2.0)
        self.release('reversal: s released while 6->5 is in flight (no space)')
        self.wait('reversal: READY at boundary 5', self.ready_at(5), LONGEST + 2.0)
        time.sleep(1.5)

    def home_exercise(self):
        self.expected_goals += 4
        n0 = len(self.cycle_log)
        self.key('h', 'home: from READY at boundary 5')
        self.wait('home: READY at boundary 0 (homed)', self.ready_at(0), 4 * LONGEST + 2.0)
        self.wait('home: the forward cycle checked at a fresh sample',
                  lambda st: len(self.cycle_log) > n0, s4a.CYCLE_CHECK_SIM_S)
        time.sleep(1.5)

    def lease_exercise(self):
        k = FIRST['lease']
        self.expected_goals += 1
        self.hold('w', 'lease: hold w')
        self.wait('lease: goal 0->1 in flight', self.goal_in_flight(k), LONGEST + 2.0)
        self.kill_teleop('lease: SIGKILL while 0->1 is in flight (no clean quit)')
        self.lease_phase = 'killed'
        st = self.wait('lease: DISARMED at boundary 1',
                       lambda st: st['state'] == 'DISARMED' and st['boundary'] == 1
                       and st.get('goal') is None, LONGEST + 2.0)
        self.lease_phase = 'disarmed'
        return st

    def observe_after_lease_loss(self, st):
        """10 s of sim after DISARMED, measured as the S3 hold (no key, no command)."""
        self.estop_sim = (st.get('monitors') or {}).get('sim_time_s')
        self.hold_observation()
        last = self.st() or {}
        self.lease_observation = {**(self.hold_summary or {}), 'state_after': last.get('state'),
                                  'goals_recorded_after': last.get('goals_recorded'),
                                  'max_seq': self.max_seq}
        self.log(lease_observation=self.lease_observation)

    def script(self, levels=None):
        if self.shadow_rehearsal:                     # before the teleop exists: no key yet
            st = self.wait('node status SHADOW with the dispatch gate False',
                           lambda st: st.get('mode') == 'shadow'
                           and st.get('dispatch_gate') is False, 30.0)
            self.shadow_verified = True
            self.log(shadow_rehearsal={'node_mode': st.get('mode'),
                                       'node_dispatch_gate': st.get('dispatch_gate'),
                                       'driver_gates': self.gates,
                                       'progress_sign_bound': s4a.SHADOW_SIGN_NOTE})
        self.spawn()
        self.wait('status with levels at level 0',
                  lambda st: bool(st.get('levels_m_s')) and st.get('level') == 0, 30.0)
        time.sleep(3.0)                               # the teleop receives the levels too
        self.settle()
        self.key('a', 'arm')
        self.wait('READY at boundary 0 after arm', self.ready_at(0), 5.0)
        self.armed = True
        self.cycle_direction = FORWARD                # the only cycle-count change: +1 (home)
        self.release_exercise()
        self.reversal_exercise()
        self.home_exercise()
        st = self.lease_exercise()
        self.observe_after_lease_loss(st)

    def summary(self):
        s = super().summary()
        s.update({'plan': [list(g) for g in PLANNED], 'expected_cancels': EXPECTED_CANCELS,
                  'releases': self.releases, 'switches': self.switches, 'kills': self.kills,
                  'lease_observation': self.lease_observation})
        s.pop('estop_hold', None)                     # S4b has no estop: the hold is post-lease
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
                         'are not applied (see the S4a driver)')
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
    d = S4bDriver(a.out, a.domain_id, limits, a.shadow_rehearsal, gates)
    rc, why = run(d)
    scope = ('S4b shadow rehearsal: level 0, release, reversal, home, lease loss; at most 12 '
             'planned shadow goals and 0 cancels; dispatch disabled' if a.shadow_rehearsal else
             'S4b: level 0, release, reversal, home, lease loss; 12 goals and 0 cancels at most')
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
