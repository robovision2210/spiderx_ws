#!/usr/bin/env python3
"""S3 script driver (docs/S3_FREE_BASE_CRAWL_DESIGN.md section 3): level 0, forward only, free base.

Reuses the S2 driver (scripts/s2/s2_driver.py, unchanged): its teleop pseudo-terminal and key
repeat, the 2 Hz status subscription, the estop key with the node's estop service as fallback, and
the wall caps (sim duration / min RTF 0.1 + 60 s). Not its script, and not its 0.02 rad tracking
stop. Every goal is built and sent by the node; this driver presses teleop keys and reads
/spiderx/locomotion/status and /joint_states only.

Script, at most 55 goals and 1 cancel, no reverse, no other level, no recovery or retry:
  settle  before arming: >= 10 s of sim with body z >= 0.045 m, z range <= 0.005 m, tilt <= 0.10
          rad, the simulation advancing and graph ownership 1/0/0 (fresh samples only)
  A       a, hold w: one cycle forward; space while its last phase is in flight (9 goals)
  C       hold w: five consecutive cycles; space during the fifth cycle's last phase (45 goals)
  E       hold w, x while its goal is in flight (1 goal, the 1 cancel), then a 10 s SIM-time
          hold observation (recorded for acceptance A6; no further keys but q)

Stops (the driver releases the key, presses x and exits 2). Approved limits, kept as they are:
  tilt > 0.10 rad, or body z < 0.045 m, each in 2 consecutive FRESH status samples;
  travel |delta since arm| > 0.15 m; a completed cycle moving < 0 or > 40 mm along body +y, or
  drifting > 20 mm along body x or > 0.10 rad in yaw (first fresh sample after the node's cycle
  count changes); a measured joint within 0.02 rad of its URDF limit; tracking > 0.05 rad (the
  node's abort; > 0.02 rad is recorded as a finding, not a stop); and the S2 protections: an
  unplanned FAULTED (ownership, freshness, sim stall, RTF and every other node fault), a command
  rejection, a goal count above the script, a wall cap. Any other driver error presses x (exit 3);
  SIGINT/SIGTERM press x (exit 4). The travel and fall stops are stricter than the acceptance
  bands (design section 3.2): they can end a run whose cycles all lie within the bands.

Fresh sample: a status whose body pose is at most 1.0 s old (monitor.pose_stale_s) and whose sim
time is later than the last counted sample's; a duplicate publication or a stalled simulation
repeats the sim time and is not counted. Tilt is the node's ground-truth tilt: the angle between
the body (dummy_link = base_link) z axis and the world z axis, independent of yaw. Body frame:
+y forward, +x right (spiderx_legs.yaml); cycle displacements use the previous boundary's yaw,
and yaw differences are wrapped to (-pi, pi].

--shadow-rehearsal (dispatch disabled; nothing moves the robot): refused (exit 5, before any ROS
call) unless the three dispatch gates are False both in a FRESH interpreter (a subprocess) and in
this process. Before the teleop starts, and in every status after that, the node must report mode
'shadow' with dispatch_gate False; a missing, conflicting or changed mode stops the run like any
other stop condition. Only then is the lower forward-progress bound (< 0 m) not applied: a shadow
cycle moves nothing, so the measured displacement is the standing body's creep. It is still
measured and recorded, with "Minimum forward-progress condition not applicable: dispatch
disabled."; the 40 mm upper bound, the lateral and yaw bounds and every other stop are unchanged.
Without the flag (the default), the behaviour is exactly as above. Shadow results are not crawl
acceptance.

Usage: s3_driver.py --out <dir> [--domain-id 0] [--shadow-rehearsal]
Writes <dir>/driver.jsonl and <dir>/driver_summary.json. Exit: 0 script complete, 2 stop condition,
3 driver error, 4 interrupted, 5 refused before start (--shadow-rehearsal)."""
import argparse
import collections
import json
import math
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 's2'))
import s2_driver as s2  # noqa: E402

LEVEL = 0
P = s2.P
CYCLES_C = 5
EXPECTED_GOALS = P + CYCLES_C * P + 1           # 55: runs A, C and E
CYCLE_S0 = s2.CYCLE_S[LEVEL]                    # 18.8 s sim per level-0 cycle with lead-ins
TILT_MAX_RAD = 0.10
Z_MIN_M = 0.045
TRAVEL_MAX_M = 0.15
PROGRESS_MIN_M, PROGRESS_MAX_M = 0.0, 0.040
LATERAL_MAX_M = 0.020
YAW_MAX_RAD = 0.10
JOINT_LIMIT_STOP_RAD = 0.02
TRACKING_ABORT_RAD = 0.05                       # = dispatch.tracking_tolerance_rad (node abort)
TRACKING_FINDING_RAD = 0.02
DEBOUNCE_SAMPLES = 2
POSE_STALE_S = 1.0                              # = monitor.pose_stale_s
SETTLE_SIM_S = 10.0
SETTLE_RANGE_M = 0.005
SETTLE_CAP_SIM_S = 60.0
HOLD_SIM_S = 10.0
JOINT_STATES_TOPIC = '/joint_states'
SHADOW_PROGRESS_NOTE = 'Minimum forward-progress condition not applicable: dispatch disabled.'
GATES = (('m55_contract', 'M55_LOCOMOTION_DISPATCH_ENABLED'),
         ('m61_live_contract', 'M61_LIVE_DISPATCH_ENABLED'),
         ('m6_live_contract', 'LIVE_DISPATCH_ENABLED'))


class Interrupted(Exception):
    pass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def body_delta(p0, p1):
    """(forward, lateral, yaw change) from pose p0 to p1 ({'xy', 'yaw'} in the world), in p0's
    body frame: body +y is forward, body +x is right."""
    dx, dy = p1['xy'][0] - p0['xy'][0], p1['xy'][1] - p0['xy'][1]
    c, s = math.cos(p0['yaw']), math.sin(p0['yaw'])
    return -s * dx + c * dy, c * dx + s * dy, wrap(p1['yaw'] - p0['yaw'])


def fresh_pose(st, last_sim):
    """(pose, sim time) if the status carries a fresh ground-truth pose, else None."""
    gt = st.get('ground_truth')
    mon = st.get('monitors') or {}
    sim, age = mon.get('sim_time_s'), mon.get('body_pose_age_s')
    if not gt or sim is None or age is None or age > POSE_STALE_S:
        return None
    if any(not isinstance(gt.get(k), (int, float)) for k in ('z', 'tilt', 'yaw')):
        return None
    if last_sim is not None and not sim > last_sim:
        return None
    return gt, sim


def imported_gates():
    """{gate: [value, module file]} as imported in this process."""
    import importlib
    out = {}
    for mod, name in GATES:
        m = importlib.import_module('spiderx_controller.' + mod)
        out[name] = [getattr(m, name), m.__file__]
    return out


def fresh_gates(python=None):
    """{gate: [value, module file]} as a FRESH interpreter imports them (a subprocess started in
    /, same environment). Raises if the import fails."""
    code = ('import json, sys; sys.path.insert(0, sys.argv[1]); import s3_driver; '
            'print(json.dumps(s3_driver.imported_gates()))')
    r = subprocess.run([python or sys.executable, '-c', code, os.path.dirname(os.path.abspath(
        __file__))], cwd='/', capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        raise RuntimeError(f'fresh gate import failed: {r.stderr.strip()[-300:]}')
    return json.loads(r.stdout.strip().splitlines()[-1])


def gate_refusal(gates):
    """None if all three gates are present and exactly False, else the reason to refuse."""
    vals = {n: (gates.get(n) or [None])[0] for _, n in GATES}
    bad = {n: v for n, v in vals.items() if v is not False}
    return None if not bad else f'dispatch gates not all False: {bad}'


class Debounce:
    """True once `n` consecutive counted samples exceed; any counted sample within resets."""

    def __init__(self, n=DEBOUNCE_SAMPLES):
        self.n, self.count = n, 0

    def update(self, exceeded):
        self.count = self.count + 1 if exceeded else 0
        return self.count >= self.n


class Settle:
    """The pre-arm settle window over fresh samples (design section 2, M2 criteria)."""

    def __init__(self):
        self.window = collections.deque()
        self.done = None

    def add(self, sim, z, tilt, graph):
        ok = (z >= Z_MIN_M and tilt <= TILT_MAX_RAD and isinstance(graph, dict)
              and graph.get('joint_state_publishers') == 1
              and graph.get('foreign_action_clients') == 0
              and graph.get('command_topic_publishers') == 0)
        if not ok:
            self.window.clear()
            return False
        self.window.append((sim, z, tilt))
        while max(s[1] for s in self.window) - min(s[1] for s in self.window) > SETTLE_RANGE_M:
            self.window.popleft()
        if self.window[-1][0] - self.window[0][0] >= SETTLE_SIM_S:
            zs = [s[1] for s in self.window]
            self.done = {'sim_from_s': self.window[0][0], 'sim_to_s': self.window[-1][0],
                         'samples': len(zs), 'z_mean_m': sum(zs) / len(zs),
                         'z_range_m': max(zs) - min(zs),
                         'max_tilt_rad': max(s[2] for s in self.window)}
            return True
        return False


class S3Driver(s2.Driver):
    def __init__(self, out, domain_id, limits, shadow_rehearsal=False, gates=None):
        self._init_s3(limits, shadow_rehearsal, gates)
        super().__init__(out, domain_id)

    def _init_s3(self, limits, shadow_rehearsal=False, gates=None):
        self.shadow_rehearsal = shadow_rehearsal    # requested (gates checked by main)
        self.shadow_verified = False        # node reported SHADOW, gate False, before the teleop
        self.gates = gates
        self.limits = dict(limits)          # {joint: (lower, upper)}, URDF
        self.pending = []                   # statuses not yet checked, in arrival order
        self.last_fresh_sim = None
        self.fresh_n = 0
        self.tilt_db, self.fall_db = Debounce(), Debounce()
        self.armed = False
        self.cycle_ref = None
        self.cycles_checked = None
        self.cycle_log = []
        self.findings = {}                  # goal seq -> max tracking error above 0.02 rad
        self.settle_state = None
        self.joint_stop = None
        self.min_joint_margin = None
        self.in_hold = False
        self.hold_obs = None
        self.estop_sim = None

    # ------------------------------------------------------------------ inputs
    def _ros(self, domain_id):
        super()._ros(domain_id)
        from sensor_msgs.msg import JointState
        self.node.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state_msg,
                                      50)

    def _on_status(self, msg):
        super()._on_status(msg)
        with self.lock:
            self.pending.append(self.status)

    def _on_joint_state_msg(self, msg):
        st = msg.header.stamp
        self._on_joint_state(list(msg.name), list(msg.position), st.sec + st.nanosec * 1e-9)

    def _on_joint_state(self, names, positions, stamp):
        worst = None
        for n, q in zip(names, positions):
            lim = self.limits.get(n)
            if lim is None:
                continue
            m = min(q - lim[0], lim[1] - q)
            if worst is None or m < worst[0]:
                worst = (m, n, q)
        if worst is None:
            return
        with self.lock:
            if self.min_joint_margin is None or worst[0] < self.min_joint_margin[0]:
                self.min_joint_margin = worst
            if self.in_hold and self.hold_obs is not None:
                ref = self.hold_obs.setdefault('joint_ref', dict(zip(names, positions)))
                mv = max((abs(q - ref[n]) for n, q in zip(names, positions) if n in ref),
                         default=0.0)
                self.hold_obs['max_joint_motion_rad'] = max(
                    self.hold_obs.get('max_joint_motion_rad', 0.0), mv)
            elif worst[0] < JOINT_LIMIT_STOP_RAD and self.joint_stop is None:
                self.joint_stop = (f'joint {worst[1]} at {worst[2]:.4f} rad is {worst[0]:.4f} rad '
                                   f'from its URDF limit (< {JOINT_LIMIT_STOP_RAD})')

    # ------------------------------------------------------------------ keys
    def key(self, k, why):
        """Shadow rehearsal: the statuses received so far are checked before every scripted key
        (a changed mode stops the script before its next key). x is never delayed."""
        if self.shadow_rehearsal and k != 'x':
            self.check()
        self._press(k, why)

    def _press(self, k, why):
        super().key(k, why)

    # ------------------------------------------------------------------ stops
    def check(self):
        with self.lock:
            batch, self.pending = self.pending, []
            joint_stop = self.joint_stop
        for st in batch:
            self._check_status(st)
        if joint_stop is not None:
            raise s2.Abort(joint_stop)

    def _check_status(self, st):
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
        self._tracking(st.get('goal'))
        fp = fresh_pose(st, self.last_fresh_sim)
        if fp is None:
            return
        gt, sim = fp
        self.last_fresh_sim = sim
        self.fresh_n += 1
        if self.in_hold:
            self._hold_sample(gt, sim)
            return
        if not self.armed:                  # settling after the drop: judged by the settle window
            if self.settle_state is not None and self.settle_state.done is None:
                self.settle_state.add(sim, gt['z'], gt['tilt'],
                                      (st.get('monitors') or {}).get('graph'))
            return
        if self.tilt_db.update(gt['tilt'] > TILT_MAX_RAD):
            raise s2.Abort(f'tilt {gt["tilt"]:.4f} rad > {TILT_MAX_RAD} in {DEBOUNCE_SAMPLES} '
                           'consecutive fresh samples')
        if self.fall_db.update(gt['z'] < Z_MIN_M):
            raise s2.Abort(f'body z {gt["z"]:.4f} m < {Z_MIN_M} in {DEBOUNCE_SAMPLES} '
                           'consecutive fresh samples')
        d = gt.get('delta_since_arm_xy')
        if d is not None and math.hypot(d[0], d[1]) > TRAVEL_MAX_M:
            raise s2.Abort(f'travel {math.hypot(d[0], d[1]):.4f} m from the arm pose > '
                           f'{TRAVEL_MAX_M}')
        pose = {'xy': tuple(gt['xy']), 'yaw': gt['yaw']}
        c = st.get('cycles')
        if self.cycle_ref is None:
            self.cycle_ref, self.cycles_checked = pose, c
            self.log(cycle_reference=pose, cycles=c, sim=sim)
            return
        if c is None or c == self.cycles_checked:
            return
        if c != self.cycles_checked + 1:
            raise s2.Abort(f'cycle count {self.cycles_checked} -> {c}: not one forward cycle')
        fwd, lat, dyaw = body_delta(self.cycle_ref, pose)
        rec = {'cycle': c, 'sim': sim, 'forward_m': fwd, 'lateral_m': lat, 'yaw_rad': dyaw}
        shadow = (self.shadow_verified and st.get('mode') == 'shadow'
                  and st.get('dispatch_gate') is False)
        if shadow:
            rec['progress_lower_bound'] = SHADOW_PROGRESS_NOTE
        self.cycle_log.append(rec)
        self.log(cycle_check=rec)
        self.cycle_ref, self.cycles_checked = pose, c
        if shadow:
            if not fwd <= PROGRESS_MAX_M:
                raise s2.Abort(f'cycle {c} moved {fwd * 1000:.1f} mm along body +y (shadow '
                               f'rehearsal: at most {PROGRESS_MAX_M * 1000:.0f})')
        elif not PROGRESS_MIN_M <= fwd <= PROGRESS_MAX_M:
            raise s2.Abort(f'cycle {c} moved {fwd * 1000:.1f} mm along body +y (allowed '
                           f'{PROGRESS_MIN_M * 1000:.0f}..{PROGRESS_MAX_M * 1000:.0f})')
        if abs(lat) > LATERAL_MAX_M or abs(dyaw) > YAW_MAX_RAD:
            raise s2.Abort(f'cycle {c} drifted {lat * 1000:.1f} mm laterally, {dyaw:.4f} rad yaw')

    def _tracking(self, g):
        err = None if g is None else g.get('max_tracking_error_rad')
        if err is None:
            return
        if err > TRACKING_ABORT_RAD:
            raise s2.Abort(f'tracking error {err} rad above {TRACKING_ABORT_RAD} on goal '
                           f'{g["seq"]}')
        if err > TRACKING_FINDING_RAD and err > self.findings.get(g['seq'], 0.0):
            self.findings[g['seq']] = err
            self.log(finding={'goal': g['seq'], 'max_tracking_error_rad': err,
                              'note': 'above 0.02 rad: recorded, not a stop'})

    def _hold_sample(self, gt, sim):
        h = self.hold_obs
        h['samples'] += 1
        h['last_sim_s'] = sim
        h['min_z_m'] = min(h['min_z_m'], gt['z'])
        h['max_tilt_rad'] = max(h['max_tilt_rad'], gt['tilt'])

    # ------------------------------------------------------------------ script
    def settle(self):
        self.settle_state = Settle()
        self.wait('settled before arm (>= 10 s sim)', lambda st: self.settle_state.done is not None,
                  SETTLE_CAP_SIM_S)
        self.log(settle=self.settle_state.done)

    def run_cycles(self, n, why):
        self.expected_goals += n * P
        c0 = self.st()['cycles']
        self.hold('w', why)
        self.wait(f'{why}: last phase of cycle {n} in flight',
                  lambda st: st['cycles'] == c0 + n - 1 and self.last_phase_in_flight(1)(st),
                  n * CYCLE_S0)
        self.key(' ', f'{why}: stop at the cycle boundary')
        self.release(why)
        self.wait(f'{why}: READY at boundary 0', self.ready_at(0), s2.LONGEST_GOAL_S + 2.0)
        time.sleep(1.5)                              # zeros after the hold window clear the latch

    def estop_exercise(self):
        self.expected_goals += 1
        seq = self.max_seq
        self.hold('w', 'estop exercise')
        self.wait('estop exercise: goal in flight', self.first_goal_in_flight(seq), 5.0)
        self.planned_estop = True
        self.key('x', 'estop exercise: x')
        self.release('estop exercise')
        st = self.wait('estop exercise: FAULTED', lambda st: st['state'] == 'FAULTED', 5.0)
        self.estop_sim = (st.get('monitors') or {}).get('sim_time_s')

    def hold_observation(self):
        """10 s of sim after the estop: recorded for acceptance A6, never a stop by itself."""
        t0 = self.estop_sim
        with self.lock:
            self.hold_obs = {'from_sim_s': t0, 'samples': 0, 'min_z_m': math.inf,
                             'max_tilt_rad': -math.inf, 'last_sim_s': None}
            self.in_hold = True
        self.wait('estop hold: 10 s of sim',
                  lambda st: t0 is not None and (st.get('monitors') or {}).get(
                      'sim_time_s', -math.inf) >= t0 + HOLD_SIM_S, HOLD_SIM_S)
        with self.lock:
            self.in_hold = False
            h = dict(self.hold_obs)
        h.pop('joint_ref', None)
        h['within_A6_limits'] = (h['samples'] > 0 and h['max_tilt_rad'] <= TILT_MAX_RAD
                                 and h['min_z_m'] >= Z_MIN_M
                                 and h.get('max_joint_motion_rad', math.inf) <= 0.01)
        self.hold_summary = h
        self.log(hold=h)

    def script(self, levels=None):
        if self.shadow_rehearsal:                     # before the teleop exists: no key yet
            st = self.wait('node status SHADOW with the dispatch gate False',
                           lambda st: st.get('mode') == 'shadow'
                           and st.get('dispatch_gate') is False, 30.0)
            self.shadow_verified = True
            self.log(shadow_rehearsal={'node_mode': st.get('mode'),
                                       'node_dispatch_gate': st.get('dispatch_gate'),
                                       'driver_gates': self.gates,
                                       'progress_lower_bound': SHADOW_PROGRESS_NOTE})
        self.spawn()
        self.wait('status with levels', lambda st: bool(st.get('levels_m_s')), 30.0)
        time.sleep(3.0)                               # the teleop receives the levels too
        self.settle()
        self.key('a', 'arm')
        self.wait('READY at boundary 0 after arm', self.ready_at(0), 5.0)
        self.armed = True
        self.run_cycles(1, 'run A: one cycle forward')
        self.run_cycles(CYCLES_C, 'run C: five cycles forward')
        self.estop_exercise()
        self.hold_observation()
        self.key('q', 'quit teleop')
        time.sleep(3.0)

    def summary(self):
        st = self.st() or {}
        return {'expected_goals': self.expected_goals, 'goal_seqs_seen': sorted(self.seen_seqs),
                'max_seq': self.max_seq, 'final_state': st.get('state'),
                'faults': st.get('faults'), 'command_rejections': st.get('command_rejections'),
                'goals_recorded': st.get('goals_recorded'), 'status_messages': self.status_n,
                'fresh_samples': self.fresh_n, 'settle': getattr(self.settle_state, 'done', None),
                'cycles': self.cycle_log,
                'tracking_findings': {str(k): v for k, v in sorted(self.findings.items())},
                'min_joint_margin': self.min_joint_margin,
                'estop_hold': getattr(self, 'hold_summary', None),
                **({'shadow_rehearsal': {
                    'driver_gates': self.gates, 'node_mode_verified': self.shadow_verified,
                    'progress_lower_bound': SHADOW_PROGRESS_NOTE,
                    'note': 'dispatch disabled: no trajectory was sent; the per-cycle '
                            'displacements are measured and recorded, not crawl acceptance'}}
                   if self.shadow_rehearsal else {})}


def run(d):
    """The script with its stop handling: (exit code, result). Every stop releases the held key
    and presses x (the node cancels a goal in flight once and holds)."""
    try:
        d.script()
        return 0, ('script complete (shadow rehearsal: dispatch disabled; not crawl acceptance)'
                   if d.shadow_rehearsal else 'script complete')
    except s2.Abort as e:
        rc, why = 2, f'STOP CONDITION: {e}'
    except (Interrupted, KeyboardInterrupt) as e:
        rc, why = 4, f'INTERRUPTED: {e!r}'
    except Exception as e:  # noqa: BLE001  (a driver bug must still stop the robot)
        rc, why = 3, f'DRIVER ERROR: {e!r}'
    d.hold_key = None
    try:
        d.emergency_stop('stop condition: emergency stop')
    except Exception as e:  # noqa: BLE001
        d.log(event='emergency_stop_failed', error=repr(e))
    return rc, why


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--domain-id', type=int, default=0)
    ap.add_argument('--shadow-rehearsal', action='store_true',
                    help='dispatch disabled: verify SHADOW throughout; the lower progress bound '
                         'is not applied (see the module docstring)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    from spiderx_controller import joint_safety
    limits, _ = joint_safety.load_limits()
    gates = None
    if a.shadow_rehearsal:
        try:
            gates = {'fresh_interpreter': fresh_gates(), 'driver_process': imported_gates()}
            why = (gate_refusal(gates['fresh_interpreter'])
                   or gate_refusal(gates['driver_process']))
        except Exception as e:  # noqa: BLE001  (no gate evidence: refuse)
            why = f'gate import failed: {e!r}'
        if why:
            summary = {'result': f'REFUSED: {why}', 'shadow_rehearsal': True, 'gates': gates}
            with open(os.path.join(a.out, 'driver_summary.json'), 'w') as f:
                json.dump(summary, f, indent=1, sort_keys=True, default=str)
            print(json.dumps(summary, sort_keys=True, default=str), flush=True)
            sys.exit(5)

    def interrupted(signum, frame):
        raise Interrupted(signal.Signals(signum).name)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    d = S3Driver(a.out, a.domain_id, limits, a.shadow_rehearsal, gates)
    rc, why = run(d)
    scope = ('S3 shadow rehearsal: level 0, forward only, at most 55 planned shadow goals and 1 '
             'shadow cancel; dispatch disabled' if a.shadow_rehearsal else
             'S3: level 0, forward only, 55 goals and 1 cancel at most')
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
