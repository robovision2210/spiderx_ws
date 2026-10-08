"""M5.5 continuous locomotion: configuration, phase goals and the locomotion state machine.

Pure Python: times are injected (monotonic wall seconds) and the action side is an injected
PhaseTransport. Importing this module starts nothing and sends nothing.

Architecture (docs/M55_KEYBOARD_WALKING.md)
-------------------------------------------
    /cmd_vel (Twist, REP-103) --> CommandLease (m55_command) --> intent (direction, speed level)
    services arm/disarm/stop/estop/reset/home --------------------------------+
    /joint_states, sim ground truth, graph status --> monitors                |
                                                          v                   v
                                     LocomotionSession (this module, the ONLY decision maker)
                                                          |
                         one PhaseGoal at a time (validated crawl phase, fingerprinted)
                                                          v
          ShadowTransport (gate False: records, sends nothing) | dispatch transport (gate True)

Dispatch unit: ONE crawl phase per FollowJointTrajectory goal. Every phase of a validated
m55_crawl template starts and ends at rest with four feet down, so between goals the robot is at
rest in a validated configuration. A goal is
    lead-in (phase_lead_s, from the measured state to the phase start; continuity-checked)
    + the phase's waypoints (the validated spline, unchanged)
No preemption: the next goal is sent only after the previous one SUCCEEDED; the horizon is one
phase (<= a few seconds). Only a fault or an emergency stop cancels a goal (once), and then the
session holds: it never sends a recovery motion on its own.

States
------
    DISARMED  nothing in flight; motion commands are ignored; `arm` checks readiness
    READY     armed, at rest at a phase boundary; a moving intent starts walking
    STARTING  the first phase from rest has been sent and awaits acceptance
    WALKING   phases are dispatched one after another while the intent stays moving
    STOPPING  controlled stop: the phase in flight finishes, nothing more is sent
    FAULTED   a fault or an emergency stop: the goal in flight was cancelled once; hold.
              Leaving FAULTED needs `reset` (-> DISARMED) and then a fresh `arm`.

Controlled stop, direction and speed
------------------------------------
* stop / key release / zero command: STOPPING (the current phase completes), then READY.
* lease expiry (no valid command for lease_s, e.g. the teleop died): stop, then DISARMED; a
  reconnecting teleop never re-arms silently.
* reversal: at the next phase boundary the phases are played backwards (the reversed phase is the
  same validated motion in reverse time).
* speed level changes take effect at a cycle boundary only (mid-cycle phase boundaries depend on
  the stride); until then the current level continues.
* home: from a mid-cycle boundary, walk the fewest phases to the nearest cycle boundary (neutral
  stance), then READY.

Shadow mode (m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED = False, the only mode in this build):
the same state machine runs against ShadowTransport, which verifies and records every goal the
session would send and reports it complete after its planned duration. Continuity is then checked
against the PLANNED positions (nothing moved), which the status labels; the measured joint states
must still be present and fresh, and `arm` still checks the measured posture.
"""

import collections
from dataclasses import dataclass
import hashlib
import json
import math
import os

import yaml

from spiderx_controller import m55_command as cmd
from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_crawl as cr
from spiderx_controller import m6_action_client as ac

CONFIG_SCHEMA = 'spiderx.m55.locomotion/1'
STATUS_SCHEMA = 'spiderx.m55.status/1'
GOAL_SCHEMA = 'spiderx.m55.phase_goal/1'
CONFIG_FILE = 'm55_locomotion.yaml'

DISARMED, READY, STARTING, WALKING, STOPPING, FAULTED = (
    'DISARMED', 'READY', 'STARTING', 'WALKING', 'STOPPING', 'FAULTED')
STATES = (DISARMED, READY, STARTING, WALKING, STOPPING, FAULTED)
ARMED_STATES = frozenset({READY, STARTING, WALKING, STOPPING})
MOTION_STATES = frozenset({STARTING, WALKING, STOPPING})

STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = (ac.STATUS_SUCCEEDED, ac.STATUS_CANCELED,
                                                     ac.STATUS_ABORTED)

# fault reasons (each enters FAULTED; the first one is the primary reason)
ESTOP = 'estop'
JOINT_STATES_STALE = 'joint_states_stale'
JOINT_STATES_INVALID = 'joint_states_invalid'
SIM_TIME_STALLED = 'sim_time_stalled'
SIM_TIME_RESET = 'sim_time_reset'
BODY_POSE_STALE = 'body_pose_stale'
BODY_POSE_INVALID = 'body_pose_invalid'
BODY_TILT_EXCEEDED = 'body_tilt_exceeded'
GRAPH_STALE = 'graph_status_stale'
OWNER_CONFLICT = 'command_owner_conflict'
TOPIC_PUBLISHER = 'command_topic_publisher_present'
JOINT_STATE_PUBLISHERS = 'joint_state_publishers'
SERVER_MISSING = 'action_server_missing'
CONTINUITY = 'continuity_error'
TRACKING = 'tracking_error'
GOAL_REJECTED = 'goal_rejected'
GOAL_ABORTED = 'goal_aborted'
GOAL_UNEXPECTED = 'goal_result_unexpected'
RESPONSE_TIMEOUT = 'goal_response_timeout'
RESULT_WATCHDOG = 'phase_result_watchdog'
TRANSPORT_ERROR = 'transport_error'
DISPATCH_DISABLED = 'dispatch_disabled'
FAULT_REASONS = (ESTOP, JOINT_STATES_STALE, JOINT_STATES_INVALID, SIM_TIME_STALLED, SIM_TIME_RESET,
                 BODY_POSE_STALE, BODY_POSE_INVALID, BODY_TILT_EXCEEDED, GRAPH_STALE,
                 OWNER_CONFLICT, TOPIC_PUBLISHER, JOINT_STATE_PUBLISHERS, SERVER_MISSING,
                 CONTINUITY, TRACKING, GOAL_REJECTED, GOAL_ABORTED, GOAL_UNEXPECTED,
                 RESPONSE_TIMEOUT, RESULT_WATCHDOG, TRANSPORT_ERROR, DISPATCH_DISABLED)

# refusal reasons for requests (no state change)
NOT_DISARMED = 'not_disarmed'
RESET_REQUIRED = 'faulted_reset_required'
NOT_FAULTED = 'not_faulted'
GOAL_UNRESOLVED = 'goal_in_flight_unresolved'
NO_JOINT_STATES = 'joint_states_missing'
POSTURE = 'posture_not_at_phase_boundary'
MOVING_COMMAND = 'arm_requires_stop_command'
SERVER_UNAVAILABLE = 'action_server_unavailable'
NOT_READY = 'not_ready'
NO_BODY_POSE = 'body_pose_missing'
NO_GRAPH = 'graph_status_missing'
UNKNOWN_REQUEST = 'unknown_request'

LEASE_EXPIRED = 'lease_expired'


class LocomotionError(ValueError):
    """Invalid configuration, templates or transport use."""


class GoalRefused(LocomotionError):
    """A goal that is not one of the library's validated phase goals."""


class DispatchDisabled(LocomotionError):
    """A dispatching transport was used while the M5.5 gate is False."""


# ======================================================================== configuration
_POS, _NONNEG, _STR, _BOOL, _LEVELS = 'pos', 'nonneg', 'str', 'bool', 'levels'
_SPEC = {
    'command': {'topic': _STR, 'lease_s': _POS, 'deadband_m_s': _POS, 'max_rate_hz': _POS,
                'unsupported_tolerance': _POS},
    'gait': {'stride_levels_m': _LEVELS, 'lift_m': _POS, 'min_shift_s': _POS, 'min_swing_s': _POS,
             'design_margin_m': _POS, 'min_margin_m': _POS, 'max_joint_speed_rad_s': _POS,
             'speed_use': _POS, 'point_dt_s': _POS, 'check_dt_s': _POS,
             'spline_tolerance_rad': _POS},
    'dispatch': {'action': _STR, 'command_topic': _STR, 'phase_lead_s': _POS,
                 'continuity_tolerance_rad': _POS, 'tracking_tolerance_rad': _POS,
                 'goal_time_tolerance_s': _POS, 'goal_response_timeout_s': _POS,
                 'result_margin_s': _POS},
    'monitor': {'joint_states_topic': _STR, 'joint_states_stale_s': _POS, 'sim_stall_s': _POS,
                'body_pose_required': _BOOL, 'pose_topic': _STR, 'model_name': _STR,
                'body_link': _STR, 'pose_stale_s': _POS, 'max_tilt_rad': _POS,
                'graph_period_s': _POS, 'graph_stale_s': _POS},
    'node': {'tick_hz': _POS, 'status_hz': _POS},
    'teleop': {'heartbeat_hz': _POS, 'hold_s': _POS},
}


@dataclass(frozen=True)
class Section:
    """One validated config section (attribute access to its keys)."""
    values: tuple

    def __getattr__(self, name):
        for k, v in object.__getattribute__(self, 'values'):
            if k == name:
                return v
        raise AttributeError(name)

    def as_dict(self):
        return {k: (list(v) if isinstance(v, tuple) else v) for k, v in self.values}


@dataclass(frozen=True)
class LocomotionConfig:
    command: Section
    gait: Section
    dispatch: Section
    monitor: Section
    node: Section
    teleop: Section
    status: str
    sha256: str = None

    def crawl_params(self, stride_m):
        g = self.gait
        return cr.CrawlParams(
            stride_m=float(stride_m), lift_m=g.lift_m, min_shift_s=g.min_shift_s,
            min_swing_s=g.min_swing_s, design_margin_m=g.design_margin_m,
            min_margin_m=g.min_margin_m, max_joint_speed_rad_s=g.max_joint_speed_rad_s,
            speed_use=g.speed_use, point_dt_s=g.point_dt_s, check_dt_s=g.check_dt_s,
            spline_tolerance_rad=g.spline_tolerance_rad)

    def to_dict(self):
        return {'schema': CONFIG_SCHEMA, 'status': self.status, 'sha256': self.sha256,
                **{name: getattr(self, name).as_dict() for name in _SPEC}}


def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _value(section, key, kind, v):
    where = f'{section}.{key}'
    if kind == _STR:
        if not isinstance(v, str) or not v:
            raise LocomotionError(f'{where} must be a non-empty string')
        return v
    if kind == _BOOL:
        if not isinstance(v, bool):
            raise LocomotionError(f'{where} must be true or false')
        return v
    if kind == _LEVELS:
        if not isinstance(v, list) or not v or not all(_number(x) and x > 0 for x in v):
            raise LocomotionError(f'{where} must be a non-empty list of numbers > 0')
        if any(b <= a for a, b in zip(v, v[1:])):
            raise LocomotionError(f'{where} must be strictly increasing')
        return tuple(float(x) for x in v)
    if not _number(v) or (kind == _POS and not v > 0) or (kind == _NONNEG and v < 0):
        raise LocomotionError(f'{where} must be a finite number '
                              f'{"> 0" if kind == _POS else ">= 0"}')
    return float(v)


def parse_config(data, sha256=None):
    """Strictly validated LocomotionConfig (unknown, missing or ill-typed keys are errors)."""
    if not isinstance(data, dict):
        raise LocomotionError('the M5.5 config must be a mapping')
    expected = {'schema', 'status', *_SPEC}
    if set(data) != expected:
        raise LocomotionError(f'config keys {sorted(data)} != {sorted(expected)}')
    if data['schema'] != CONFIG_SCHEMA:
        raise LocomotionError(f'schema {data["schema"]!r} != {CONFIG_SCHEMA!r}')
    if data['status'] != 'provisional':
        raise LocomotionError('status must be "provisional" until local acceptance evidence '
                              'exists')
    sections = {}
    for name, spec in _SPEC.items():
        sec = data[name]
        if not isinstance(sec, dict) or set(sec) != set(spec):
            got = sorted(sec) if isinstance(sec, dict) else sec
            raise LocomotionError(f'{name} keys {got} != {sorted(spec)}')
        sections[name] = Section(tuple((k, _value(name, k, kind, sec[k]))
                                       for k, kind in spec.items()))
    cfg = LocomotionConfig(status=data['status'], sha256=sha256, **sections)
    d, m, t = cfg.dispatch, cfg.monitor, cfg.teleop
    try:
        for stride in cfg.gait.stride_levels_m:
            cfg.crawl_params(stride).check()
    except cr.CrawlError as e:
        raise LocomotionError(f'gait: {e}') from None
    if d.continuity_tolerance_rad / d.phase_lead_s > cfg.gait.max_joint_speed_rad_s:
        raise LocomotionError('dispatch: continuity_tolerance_rad / phase_lead_s must not exceed '
                              'gait.max_joint_speed_rad_s (the lead-in would be too fast)')
    if d.tracking_tolerance_rad < d.continuity_tolerance_rad:
        raise LocomotionError('dispatch: tracking_tolerance_rad must be >= '
                              'continuity_tolerance_rad')
    if cfg.command.lease_s < 2.0 / t.heartbeat_hz:
        raise LocomotionError('command.lease_s must cover at least two teleop heartbeats')
    if cfg.command.max_rate_hz <= t.heartbeat_hz:
        raise LocomotionError('command.max_rate_hz must exceed teleop.heartbeat_hz')
    if t.hold_s <= 1.0 / t.heartbeat_hz:
        raise LocomotionError('teleop.hold_s must exceed one heartbeat period')
    if m.graph_stale_s <= m.graph_period_s:
        raise LocomotionError('monitor.graph_stale_s must exceed monitor.graph_period_s')
    return cfg


def config_path(config_dir=None):
    if config_dir is None:
        from ament_index_python.packages import get_package_share_directory
        config_dir = os.path.join(get_package_share_directory('spiderx_controller'), 'config')
    return os.path.join(config_dir, CONFIG_FILE)


def load_config(config_dir=None):
    path = config_path(config_dir)
    with open(path, 'rb') as f:
        raw = f.read()
    return parse_config(yaml.safe_load(raw), hashlib.sha256(raw).hexdigest())


# ======================================================================== phase goals
def _r(x):
    return round(float(x), 9)


def goal_fingerprint(joint_names, points):
    """sha256 over the exact dispatched content (names, times, positions, velocities)."""
    payload = {'schema': GOAL_SCHEMA, 'joint_names': list(joint_names),
               'points': [[t, list(q), list(v)] for t, q, v in points]}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'))
                          .encode()).hexdigest()


@dataclass(frozen=True)
class PhaseGoal:
    """One validated crawl phase as a FollowJointTrajectory goal (times from goal start)."""
    level: int
    phase: int                  # forward phase index k (0 .. P-1)
    direction: int              # +1 forward, -1 reverse
    b_from: int                 # boundary index before (0 .. P; P is the next cycle's 0)
    b_to: int
    kind: str                   # 'shift' | 'swing'
    leg: str                    # swinging leg or None
    joint_names: tuple
    points: tuple               # ((t, positions, velocities), ...); first point at phase_lead_s
    body_delta: tuple           # planned body displacement (x, y) in base_link (m)
    fingerprint: str

    @property
    def duration_s(self):
        return self.points[-1][0]

    @property
    def start(self):
        return dict(zip(self.joint_names, self.points[0][1]))

    @property
    def end(self):
        return dict(zip(self.joint_names, self.points[-1][1]))

    def summary(self):
        return {'level': self.level, 'phase': self.phase, 'direction': self.direction,
                'b_from': self.b_from, 'b_to': self.b_to, 'kind': self.kind, 'leg': self.leg,
                'duration_s': self.duration_s, 'points': len(self.points),
                'fingerprint': self.fingerprint}

    def to_dict(self):
        return {'schema': GOAL_SCHEMA, **self.summary(), 'joint_names': list(self.joint_names),
                'points': [{'time_from_start_s': t, 'positions': list(q), 'velocities': list(v)}
                           for t, q, v in self.points]}


def verify_goal(goal, allowed):
    """Raise GoalRefused unless `goal` is intact and one of the allowed (validated) goals."""
    if not isinstance(goal, PhaseGoal):
        raise GoalRefused(f'not a PhaseGoal: {type(goal).__name__}')
    fp = goal_fingerprint(goal.joint_names, goal.points)
    if fp != goal.fingerprint:
        raise GoalRefused('goal content does not match its fingerprint')
    if fp not in allowed:
        raise GoalRefused('goal is not one of the validated phase goals')


class PhaseLibrary:
    """Every dispatchable phase goal, built from validated forward crawl templates."""

    NEUTRAL_TOLERANCE_RAD = 1e-6

    def __init__(self, templates, lead_s):
        if not templates:
            raise LocomotionError('at least one crawl template is required')
        temps = sorted(templates, key=lambda t: t.params.stride_m)
        first = temps[0]
        if not (_number(lead_s) and lead_s > 0):
            raise LocomotionError('lead_s must be > 0')
        for t in temps:
            if not isinstance(t, cr.CrawlTemplate) or t.direction != 1:
                raise LocomotionError('templates must be FORWARD m55_crawl.CrawlTemplate objects')
            if t.report.get('failures'):
                raise LocomotionError('a template with validation failures is not dispatchable')
            if t.boundaries is None or t.phase_info is None or t.boundary_body is None:
                raise LocomotionError('template has no phase metadata')
            if list(t.joint_names) != list(first.joint_names):
                raise LocomotionError('templates use different joint orders')
            if [k for k, _ in t.phase_info] != [k for k, _ in first.phase_info] or \
                    [lg for _, lg in t.phase_info] != [lg for _, lg in first.phase_info]:
                raise LocomotionError('templates use different phase sequences')
            if t.boundaries[0] != 0 or t.boundaries[-1] != len(t.points) - 1:
                raise LocomotionError('template boundaries do not span its points')
            for i in t.boundaries:
                if any(v != 0.0 for v in t.points[i]['velocities']):
                    raise LocomotionError('template phase boundaries must be rest points')
            a, b = t.points[0]['positions'], t.points[-1]['positions']
            n0 = first.points[0]['positions']
            if max(abs(x - y) for x, y in zip(a, b)) > self.NEUTRAL_TOLERANCE_RAD or \
                    max(abs(x - y) for x, y in zip(a, n0)) > self.NEUTRAL_TOLERANCE_RAD:
                raise LocomotionError('every template must start and end in the same neutral '
                                      'stance')
        self.templates = temps
        self.lead_s = float(lead_s)
        self.joint_names = tuple(first.joint_names)
        self.n_phases = len(first.phase_info)
        self.strides_m = [t.params.stride_m for t in temps]
        self.cycle_s = [t.cycle_s for t in temps]
        # nominal speed including the per-phase lead-in (result latency is not included)
        self.speeds_m_s = [t.params.stride_m / (t.cycle_s + self.n_phases * self.lead_s)
                           for t in temps]
        if any(b <= a for a, b in zip(self.speeds_m_s, self.speeds_m_s[1:])):
            raise LocomotionError(f'level speeds {self.speeds_m_s} are not strictly increasing')
        self.goals = {}
        for level, t in enumerate(temps):
            for k in range(self.n_phases):
                for direction in (1, -1):
                    self.goals[(level, k, direction)] = self._make(level, t, k, direction)
        self.fingerprints = frozenset(g.fingerprint for g in self.goals.values())

    def _make(self, level, t, k, direction):
        i0, i1 = t.boundaries[k], t.boundaries[k + 1]
        seg = t.points[i0:i1 + 1]
        t0, t1 = seg[0]['time_from_start_s'], seg[-1]['time_from_start_s']
        if direction > 0:
            pts = [(_r(self.lead_s + p['time_from_start_s'] - t0),
                    tuple(_r(q) for q in p['positions']),
                    tuple(_r(v) for v in p['velocities'])) for p in seg]
            b_from, b_to = k, k + 1
            delta = tuple(_r(b - a) for a, b in zip(t.boundary_body[k], t.boundary_body[k + 1]))
        else:
            pts = [(_r(self.lead_s + t1 - p['time_from_start_s']),
                    tuple(_r(q) for q in p['positions']),
                    tuple(_r(-v) + 0.0 for v in p['velocities'])) for p in reversed(seg)]
            b_from, b_to = k + 1, k
            delta = tuple(_r(a - b) for a, b in zip(t.boundary_body[k], t.boundary_body[k + 1]))
        kind, leg = t.phase_info[k]
        return PhaseGoal(level, k, direction, b_from, b_to, kind, leg, self.joint_names,
                         tuple(pts), delta, goal_fingerprint(self.joint_names, pts))

    def goal(self, level, b, direction):
        """The goal that leaves boundary b (0..P-1; 0 == P) in `direction`."""
        P = self.n_phases
        if direction > 0:
            return self.goals[(level, b % P, 1)]
        return self.goals[(level, (b - 1) % P, -1)]

    def boundary_positions(self, level, b):
        t = self.templates[level]
        return dict(zip(self.joint_names, t.points[t.boundaries[b % self.n_phases]]['positions']))

    def match(self, measured, tol, prefer_level=0):
        """(level, b, max_error) of a validated boundary within tol of `measured`, or None.

        b = 0 (neutral) is the same for every level and reports prefer_level. Mid-cycle
        boundaries of DIFFERENT levels can be close (the same phase of two strides; the real
        templates come within ~0.023 rad), so a boundary of prefer_level (the level last
        executed) wins whenever it is within tol; otherwise the closest one. Any match is within
        tol of a validated rest configuration, which is all the next phase's continuity needs.
        None if nothing is within tol or a joint is missing."""
        if any(n not in measured for n in self.joint_names):
            return None
        cands = []
        for level in range(len(self.templates)):
            for b in range(self.n_phases):
                if b == 0 and level > 0:
                    continue                                   # one shared neutral
                ref = self.boundary_positions(level, b)
                err = max(abs(measured[n] - ref[n]) for n in self.joint_names)
                if err <= tol:
                    cands.append((prefer_level if b == 0 else level, b, err))
        if not cands:
            return None
        preferred = [c for c in cands if c[0] == prefer_level]
        return min(preferred or cands, key=lambda c: c[2])

    def describe(self):
        return {'levels': [{'level': i, 'stride_m': t.params.stride_m, 'cycle_s': t.cycle_s,
                            'speed_m_s': self.speeds_m_s[i],
                            'template_speed_m_s': t.report.get('speed_m_s'),
                            'min_static_margin_m': t.report.get('min_static_margin_m'),
                            'max_joint_speed_rad_s': t.report.get('max_joint_speed_rad_s'),
                            'phase_durations_s': t.report.get('phase_durations_s')}
                           for i, t in enumerate(self.templates)],
                'phases_per_cycle': self.n_phases,
                'phase_sequence': [list(x) for x in self.templates[0].phase_info],
                'lead_s': self.lead_s, 'goals': len(self.goals),
                'joint_names': list(self.joint_names)}


def build_library(cfg, designer=None, progress=None):
    """Design and validate one template per stride level (slow: ~20 s per level), then the
    library. Raises CrawlError if any level fails validation (nothing is dispatchable then)."""
    designer = designer or cr.load_designer()
    temps = []
    for stride in cfg.gait.stride_levels_m:
        if progress:
            progress(f'designing and validating the {stride * 1000:.0f} mm crawl template')
        temps.append(cr.build_template(designer, cfg.crawl_params(stride)))
    return PhaseLibrary(temps, cfg.dispatch.phase_lead_s)


# ======================================================================== transports
class ShadowTransport:
    """Gate-False transport: verifies and RECORDS each goal, reports it complete after its
    planned duration, and sends nothing anywhere."""

    mode = 'shadow'

    def __init__(self, allowed_fingerprints, max_records=1000):
        self.allowed = frozenset(allowed_fingerprints)
        self.records = collections.deque(maxlen=max_records)
        self.sent = 0
        self._active = None
        self._events = []

    def ready(self):
        return True

    def send(self, goal, wall):
        verify_goal(goal, self.allowed)
        if self._active is not None:
            raise LocomotionError('a goal is already active (one goal at a time)')
        self.sent += 1
        self.records.append({'seq': self.sent, 'wall': wall, **goal.summary()})
        self._active = (self.sent, wall + goal.duration_s)
        self._events.append(('goal_response', self.sent, True))
        return self.sent

    def cancel(self, wall):
        if self._active is not None:
            seq = self._active[0]
            self._active = None
            self._events += [('cancel_response', seq, 0),
                             ('result', seq, STATUS_CANCELED, 0, 'shadow cancel')]

    def poll(self, wall):
        if self._active is not None and wall >= self._active[1]:
            self._events.append(('result', self._active[0], STATUS_SUCCEEDED, 0, ''))
            self._active = None
        events, self._events = self._events, []
        return events

    def close(self):
        self._active = None


# ======================================================================== the session
@dataclass
class _Goal:
    seq: int
    goal: PhaseGoal
    sent_wall: float
    accepted: bool = None
    accept_wall: float = None
    result: tuple = None
    cancel_sent: bool = False
    cancel_response: int = None
    max_error: float = 0.0


class LocomotionSession:
    """The locomotion state machine. Feed it inputs, call step(wall) periodically."""

    def __init__(self, cfg, library, transport, log_size=500):
        if transport.mode not in ('shadow', 'dispatch'):
            raise LocomotionError(f'unknown transport mode {transport.mode!r}')
        if transport.mode == 'dispatch' and not c55.M55_LOCOMOTION_DISPATCH_ENABLED:
            raise DispatchDisabled(c55.M55_DISPATCH_DISABLED_MESSAGE)
        if cfg.command.deadband_m_s >= library.speeds_m_s[0]:
            raise LocomotionError('command.deadband_m_s must be below the slowest level speed')
        self.cfg = cfg
        self.lib = library
        self.transport = transport
        self.shadow = transport.mode == 'shadow'
        self.lease = cmd.CommandLease(
            cmd.CommandMapper(library.speeds_m_s, cfg.command.deadband_m_s,
                              cfg.command.unsupported_tolerance),
            cfg.command.lease_s, cfg.command.max_rate_hz)
        self.state = DISARMED
        self.level = 0
        self.b = 0
        self.direction = 0
        self.goal = None
        self.homing = 0                   # 0, or the direction of the homing walk
        self.stop_pending = None          # reason of a requested controlled stop
        self.stop_latched = False         # a stop request holds until a zero command is seen
        self.rest_wall = None             # when the last phase reported its result
        self.awaiting_js = False          # at rest, waiting for a joint state newer than that
        self.disarm_after_stop = False
        self.armed_wall = None
        self.faults = []
        self.virtual_q = None             # shadow mode: planned joint positions
        self.phases_completed = 0
        self.cycles = 0
        self.planned_xy = [0.0, 0.0]
        self.last_refusal = None
        self.log = collections.deque(maxlen=log_size)
        self.n_events = 0
        # monitors
        self.js = None                    # (wall, stamp, {name: q})
        self.js_problem = None
        self.stamp_change_wall = None
        self.clock_reset_wall = None
        self.pose = None                  # (wall, codes, pose dict)
        self.pose_at_arm = None
        self.graph = None                 # (wall, status dict)

    # ------------------------------------------------------------------ helpers
    def _event(self, wall, event, **detail):
        self.n_events += 1
        self.log.append({'n': self.n_events, 'wall': wall, 'event': event, 'state': self.state,
                         **detail})

    def _set(self, wall, state, why):
        if state != self.state:
            prev = self.state
            self.state = state
            self._event(wall, 'transition', previous=prev, why=why)

    def _fault(self, wall, reason, **detail):
        if reason not in self.faults:
            self.faults.append(reason)
        self._event(wall, 'fault', reason=reason, **detail)
        self._set(wall, FAULTED, reason)
        self.lease.clear()
        self.homing = 0
        self.stop_pending = None
        self.disarm_after_stop = False
        self.awaiting_js = False
        g = self.goal
        if g is not None and g.result is None and not g.cancel_sent:
            g.cancel_sent = True                       # once, never retried
            try:
                self.transport.cancel(wall)
                self._event(wall, 'cancel_sent', seq=g.seq)
            except Exception as e:  # noqa: BLE001 - the fault stands; the goal stays unresolved
                self._event(wall, 'cancel_failed', seq=g.seq, error=repr(e))

    def _measured(self):
        return None if self.js is None else self.js[2]

    def _lease(self, wall):
        """(intent, expired). Expired: no valid command for lease_s (counted from the arm when
        no command has arrived since)."""
        intent, reason = self.lease.current(wall)
        expired = reason == LEASE_EXPIRED or (
            reason == 'no_command_yet' and self.armed_wall is not None and
            wall - self.armed_wall > self.cfg.command.lease_s)
        return intent, expired

    # ------------------------------------------------------------------ inputs
    def on_command(self, wall, linear, angular):
        """A received /cmd_vel (receipt time). Returns the rejection code or None."""
        code = self.lease.on_command(wall, linear, angular)
        if code is not None:
            self._event(wall, 'command_rejected', code=code)
        return code

    def on_joint_state(self, wall, stamp, names, positions):
        names, positions = list(names), list(positions)
        problem = None
        if len(names) != len(positions) or len(set(names)) != len(names):
            problem = 'malformed'
        elif any(n not in names for n in self.lib.joint_names):
            problem = 'missing_joints'
        elif not all(_number(p) for p in positions):
            problem = 'nonfinite'
        if problem is not None:
            self.js_problem = (wall, problem)
            if self.state in ARMED_STATES:
                self._fault(wall, JOINT_STATES_INVALID, problem=problem)
            return
        if self.js is not None and stamp is not None and self.js[1] is not None:
            if stamp < self.js[1] - 1e-9:
                self.clock_reset_wall = wall
                if self.state in ARMED_STATES:
                    self._fault(wall, SIM_TIME_RESET, previous=self.js[1], stamp=stamp)
            if stamp > self.js[1]:
                self.stamp_change_wall = wall
        elif stamp is not None:
            self.stamp_change_wall = wall
        self.js = (wall, stamp, dict(zip(names, (float(p) for p in positions))))
        self.js_problem = None

    def on_body_pose(self, wall, codes, pose=None):
        """Simulator ground truth (development monitor): codes = problems with this sample;
        pose = {'xy': (x, y), 'z': z, 'yaw': rad, 'tilt': rad} of the body in the world."""
        self.pose = (wall, tuple(codes), pose)

    def on_graph(self, wall, status):
        """status: {'joint_state_publishers', 'command_topic_publishers', 'foreign_action_clients',
        'action_server_present'} from the node's graph queries."""
        self.graph = (wall, dict(status))

    # ------------------------------------------------------------------ readiness checks
    def _joint_problem(self, wall):
        if self.js_problem is not None:
            return JOINT_STATES_INVALID
        if self.js is None:
            return NO_JOINT_STATES
        if wall - self.js[0] > self.cfg.monitor.joint_states_stale_s:
            return JOINT_STATES_STALE
        if self.stamp_change_wall is None or \
                wall - self.stamp_change_wall > self.cfg.monitor.sim_stall_s:
            return SIM_TIME_STALLED
        return None

    def _pose_problem(self, wall):
        m = self.cfg.monitor
        if not m.body_pose_required:
            return None
        if self.pose is None:
            return NO_BODY_POSE
        pw, codes, pose = self.pose
        if wall - pw > m.pose_stale_s:
            return BODY_POSE_STALE
        if codes or pose is None:
            return BODY_POSE_INVALID
        if pose['tilt'] > m.max_tilt_rad:
            return BODY_TILT_EXCEEDED
        return None

    def _graph_problem(self, wall):
        if self.graph is None:
            return NO_GRAPH
        gw, st = self.graph
        if wall - gw > self.cfg.monitor.graph_stale_s:
            return GRAPH_STALE
        if st.get('command_topic_publishers', 0) != 0:
            return TOPIC_PUBLISHER
        if st.get('foreign_action_clients', 0) != 0:
            return OWNER_CONFLICT
        if st.get('joint_state_publishers') != 1:
            return JOINT_STATE_PUBLISHERS
        if not self.shadow and not st.get('action_server_present', False):
            return SERVER_MISSING
        return None

    # ------------------------------------------------------------------ requests
    def request(self, name, wall):
        """Service request -> (ok, message). Never raises for a known request."""
        handler = {'arm': self._arm, 'disarm': self._disarm, 'stop': self._stop,
                   'estop': self._estop, 'reset': self._reset, 'home': self._home}.get(name)
        if handler is None:
            return self._refuse(wall, name, UNKNOWN_REQUEST)
        return handler(wall)

    def _refuse(self, wall, name, reason, detail=''):
        self.last_refusal = {'request': name, 'reason': reason, 'wall': wall}
        self._event(wall, 'refused', request=name, reason=reason)
        return False, reason + (f': {detail}' if detail else '')

    def _accept(self, wall, name, message):
        self._event(wall, 'request', request=name, message=message)
        return True, message

    def _arm(self, wall):
        if self.state == FAULTED:
            return self._refuse(wall, 'arm', RESET_REQUIRED)
        if self.state != DISARMED:
            return self._refuse(wall, 'arm', NOT_DISARMED, self.state)
        if not self.shadow:
            if not c55.M55_LOCOMOTION_DISPATCH_ENABLED:
                return self._refuse(wall, 'arm', DISPATCH_DISABLED)
            if not self.transport.ready():
                return self._refuse(wall, 'arm', SERVER_UNAVAILABLE)
        for check in (self._joint_problem, self._graph_problem, self._pose_problem):
            problem = check(wall)
            if problem is not None:
                return self._refuse(wall, 'arm', problem)
        if self.clock_reset_wall is not None and \
                wall - self.clock_reset_wall < self.cfg.monitor.sim_stall_s:
            return self._refuse(wall, 'arm', SIM_TIME_RESET)
        intent, _ = self.lease.current(wall)
        if intent.moving:
            return self._refuse(wall, 'arm', MOVING_COMMAND)
        tol = self.cfg.dispatch.continuity_tolerance_rad
        found = self.lib.match(self._measured(), tol, prefer_level=self.level)
        if found is None:
            return self._refuse(wall, 'arm', POSTURE,
                                f'no validated phase boundary within {tol} rad')
        self.level, self.b, err = found
        self.direction = 0
        self.stop_latched = False
        self.lease.clear()
        self.armed_wall = wall
        self.virtual_q = self.lib.boundary_positions(self.level, self.b)
        self.pose_at_arm = None if self.pose is None else self.pose[2]
        self._set(wall, READY, 'arm')
        return self._accept(wall, 'arm', f'armed at boundary {self.b} (level {self.level}, '
                                         f'posture error {err:.4f} rad)')

    def _disarm(self, wall):
        if self.state == DISARMED:
            return self._accept(wall, 'disarm', 'already disarmed')
        if self.state == FAULTED:
            return self._refuse(wall, 'disarm', RESET_REQUIRED)
        if self.state == READY:
            self._to_disarmed(wall, 'disarm')
            return self._accept(wall, 'disarm', 'disarmed')
        self.disarm_after_stop = True
        self._begin_stop(wall, 'disarm')
        return self._accept(wall, 'disarm', 'stopping after the current phase, then disarming')

    def _stop(self, wall):
        self.stop_latched = True
        if self.state in MOTION_STATES:
            self._begin_stop(wall, 'stop')
            return self._accept(wall, 'stop', 'stopping after the current phase (held until a '
                                              'zero command)')
        return self._accept(wall, 'stop', f'not moving ({self.state}); held until a zero '
                                          f'command')

    def _estop(self, wall):
        self._fault(wall, ESTOP)
        return self._accept(wall, 'estop', 'emergency stop: goal cancelled (if any), holding; '
                                           'reset and arm to continue')

    def _reset(self, wall):
        if self.state != FAULTED:
            return self._refuse(wall, 'reset', NOT_FAULTED, self.state)
        if self.goal is not None:
            return self._refuse(wall, 'reset', GOAL_UNRESOLVED,
                                'the cancelled goal has not reported a result')
        self._event(wall, 'reset', faults=list(self.faults))
        self.faults = []
        self.lease.clear()
        self._set(wall, DISARMED, 'reset')
        return self._accept(wall, 'reset', 'reset: disarmed (arm again to continue)')

    def _home(self, wall):
        if self.state != READY:
            return self._refuse(wall, 'home', NOT_READY, self.state)
        if self.b == 0:
            return self._accept(wall, 'home', 'already at a cycle boundary')
        P = self.lib.n_phases
        self.homing = 1 if P - self.b < self.b else -1
        self._event(wall, 'homing', direction=self.homing,
                    phases=P - self.b if self.homing > 0 else self.b)
        self._dispatch(wall, self.homing, STARTING)
        if self.state == FAULTED:
            return False, f'faulted: {self.faults[-1]}'
        return self._accept(wall, 'home', f'homing {"forward" if self.homing > 0 else "backward"}'
                                          f' to the cycle boundary')

    # ------------------------------------------------------------------ transitions
    def _to_disarmed(self, wall, why):
        self.lease.clear()
        self.homing = 0
        self.stop_pending = None
        self.disarm_after_stop = False
        self.direction = 0
        self._set(wall, DISARMED, why)

    def _begin_stop(self, wall, why):
        if self.state in (STARTING, WALKING):
            self.stop_pending = why
            self.homing = 0
            self._set(wall, STOPPING, why)

    def _dispatch(self, wall, direction, state):
        """Send the next phase from boundary self.b in `direction` (continuity checked)."""
        goal = self.lib.goal(self.level, self.b, direction)
        if not self.shadow and not c55.M55_LOCOMOTION_DISPATCH_ENABLED:
            self._fault(wall, DISPATCH_DISABLED)
            return
        problem = self._joint_problem(wall)
        if problem is not None:
            self._fault(wall, JOINT_STATES_STALE if problem == NO_JOINT_STATES else problem)
            return
        ref = self.virtual_q if self.shadow else self._measured()
        start = goal.start
        err = max(abs(ref[n] - start[n]) for n in self.lib.joint_names)
        if err > self.cfg.dispatch.continuity_tolerance_rad:
            self._fault(wall, CONTINUITY, error_rad=err, phase=goal.phase, b=self.b,
                        reference='planned' if self.shadow else 'measured')
            return
        try:
            seq = self.transport.send(goal, wall)
        except Exception as e:  # noqa: BLE001 - nothing was accepted; fault and hold
            self._fault(wall, TRANSPORT_ERROR, error=repr(e))
            return
        self.goal = _Goal(seq, goal, wall)
        self.direction = direction
        self._event(wall, 'goal_sent', seq=seq, continuity_error_rad=err, **goal.summary())
        self._set(wall, state, 'dispatch')

    def _phase_done(self, wall):
        g = self.goal.goal
        self.goal = None
        P = self.lib.n_phases
        self.b = g.b_to % P
        self.phases_completed += 1
        self.rest_wall = wall
        self.planned_xy = [self.planned_xy[0] + g.body_delta[0],
                           self.planned_xy[1] + g.body_delta[1]]
        if (g.direction > 0 and g.b_to == P) or (g.direction < 0 and g.b_to == 0):
            self.cycles += g.direction
        if self.shadow:
            self.virtual_q = g.end
        self._event(wall, 'phase_done', b=self.b, phase=g.phase, direction=g.direction)
        self._after_rest(wall)

    def _after_rest(self, wall):
        """At rest at boundary self.b with nothing in flight: decide what happens next.

        Dispatch mode first waits for a /joint_states sample received AFTER the phase result, so
        the continuity check never uses a sample taken while the phase was still finishing (the
        two streams are independent); the joint-state monitor bounds that wait."""
        if not self.shadow and (self.js is None or self.js[0] <= self.rest_wall):
            self.awaiting_js = True
            return
        self.awaiting_js = False
        intent, expired = self._lease(wall)
        if self.state == STOPPING or expired:
            disarm = self.disarm_after_stop or expired
            self.stop_pending = None
            self.direction = 0
            if disarm:
                self._to_disarmed(wall, LEASE_EXPIRED if expired else 'disarm')
            else:
                self._set(wall, READY, 'stopped')
            return
        if self.homing:
            if self.b == 0:
                self.homing = 0
                self.direction = 0
                self._set(wall, READY, 'homed')
            else:
                self._dispatch(wall, self.homing, WALKING)
            return
        if intent.moving and not self.stop_latched:
            if self.b == 0 and intent.level != self.level:
                self._event(wall, 'level_change', previous=self.level, level=intent.level)
                self.level = intent.level
            self._dispatch(wall, intent.direction, WALKING)
            return
        self.direction = 0
        self._set(wall, READY, 'intent_stop')

    # ------------------------------------------------------------------ the tick
    def step(self, wall):
        try:
            events = self.transport.poll(wall)
        except Exception as e:  # noqa: BLE001
            events = []
            if self.state in ARMED_STATES:
                self._fault(wall, TRANSPORT_ERROR, error=repr(e))
        for ev in events:
            self._on_transport_event(wall, ev)
        if self.state in ARMED_STATES:
            self._monitor(wall)
        if self.awaiting_js and self.goal is None and self.state in (WALKING, STOPPING):
            self._after_rest(wall)
        g = self.goal
        if g is not None and g.result is None:
            self._watchdogs(wall, g)
        if self.state == READY:
            self._ready_tick(wall)
        elif self.state in (STARTING, WALKING):
            intent, expired = self._lease(wall)
            if expired:
                self.disarm_after_stop = True
                self._begin_stop(wall, LEASE_EXPIRED)
            elif not self.homing and not intent.moving:
                self._begin_stop(wall, 'intent_stop')

    def _ready_tick(self, wall):
        intent, expired = self._lease(wall)
        if expired:
            self._event(wall, 'lease', reason=LEASE_EXPIRED)
            self._to_disarmed(wall, LEASE_EXPIRED)
            return
        if not intent.moving:
            self.stop_latched = False
        elif not self.stop_latched:
            if self.b == 0 and intent.level != self.level:
                self._event(wall, 'level_change', previous=self.level, level=intent.level)
                self.level = intent.level
            self._dispatch(wall, intent.direction, STARTING)

    def _monitor(self, wall):
        for check in (self._joint_problem, self._pose_problem, self._graph_problem):
            problem = check(wall)
            if problem is not None:
                reason = {NO_JOINT_STATES: JOINT_STATES_STALE, NO_BODY_POSE: BODY_POSE_STALE,
                          NO_GRAPH: GRAPH_STALE}.get(problem, problem)
                self._fault(wall, reason)
                return

    def _watchdogs(self, wall, g):
        d = self.cfg.dispatch
        if g.accepted is None and wall - g.sent_wall > d.goal_response_timeout_s:
            if self.state != FAULTED:
                self._fault(wall, RESPONSE_TIMEOUT, seq=g.seq)
        elif g.accepted and wall - g.accept_wall > (g.goal.duration_s + d.goal_time_tolerance_s +
                                                    d.result_margin_s):
            if self.state != FAULTED:
                self._fault(wall, RESULT_WATCHDOG, seq=g.seq)

    def _on_transport_event(self, wall, ev):
        kind = ev[0]
        g = self.goal
        if g is None or ev[1] != g.seq:
            self._event(wall, 'stray_transport_event', transport_event=list(ev))
            return
        if kind == 'goal_response':
            g.accepted = bool(ev[2])
            g.accept_wall = wall
            self._event(wall, 'goal_response', seq=g.seq, accepted=g.accepted)
            if not g.accepted:
                self.goal = None
                if self.state != FAULTED:
                    self._fault(wall, GOAL_REJECTED, seq=g.seq)
            elif self.state == STARTING:
                self._set(wall, WALKING, 'accepted')
        elif kind == 'feedback':
            err = float(ev[2])
            g.max_error = max(g.max_error, err) if math.isfinite(err) else math.inf
            if not err <= self.cfg.dispatch.tracking_tolerance_rad and self.state != FAULTED:
                self._fault(wall, TRACKING, seq=g.seq, error_rad=err)
        elif kind == 'cancel_response':
            g.cancel_response = ev[2]
            self._event(wall, 'cancel_response', seq=g.seq, code=ev[2])
        elif kind == 'result':
            status, code = ev[2], ev[3]
            g.result = (status, code)
            self._event(wall, 'result', seq=g.seq, status=status, error_code=code,
                        error_string=ev[4] if len(ev) > 4 else '')
            if self.state == FAULTED:
                self.goal = None                       # the cancelled goal is resolved
            elif status == STATUS_SUCCEEDED:
                self._phase_done(wall)
            else:
                self._fault(wall, GOAL_ABORTED if status == STATUS_ABORTED else GOAL_UNEXPECTED,
                            seq=g.seq, status=status, error_code=code)
                self.goal = None
        else:
            self._event(wall, 'unknown_transport_event', transport_event=list(ev))

    # ------------------------------------------------------------------ status
    def status(self, wall):
        intent, reason = self.lease.current(wall)
        g = self.goal
        lv = self.lease.last_valid_wall
        mon = {
            'joint_states_age_s': None if self.js is None else wall - self.js[0],
            'sim_time_s': None if self.js is None else self.js[1],
            'body_pose_age_s': None if self.pose is None else wall - self.pose[0],
            'graph_age_s': None if self.graph is None else wall - self.graph[0],
            'graph': None if self.graph is None else self.graph[1],
        }
        gt = None
        if self.pose is not None and self.pose[2] is not None:
            p = self.pose[2]
            gt = {'label': 'simulator ground truth (development monitor), not odometry',
                  'xy': list(p['xy']), 'z': p.get('z'), 'yaw': p.get('yaw'),
                  'tilt': p.get('tilt')}
            if self.pose_at_arm is not None:
                a = self.pose_at_arm
                gt['delta_since_arm_xy'] = [p['xy'][0] - a['xy'][0], p['xy'][1] - a['xy'][1]]
        return {
            'schema': STATUS_SCHEMA, 'milestone': c55.MILESTONE, 'state': self.state,
            'mode': self.transport.mode, 'dispatch_gate': c55.M55_LOCOMOTION_DISPATCH_ENABLED,
            'levels_m_s': list(self.lib.speeds_m_s), 'strides_m': list(self.lib.strides_m),
            'level': self.level, 'boundary': self.b, 'phases_per_cycle': self.lib.n_phases,
            'direction': self.direction, 'homing': self.homing,
            'stop_pending': self.stop_pending, 'disarm_pending': self.disarm_after_stop,
            'stop_latched': self.stop_latched, 'awaiting_joint_state': self.awaiting_js,
            'intent': {'direction': intent.direction, 'level': intent.level,
                       'requested_m_s': intent.requested_m_s,
                       'granted_m_s': intent.granted_m_s, 'reason': reason},
            'lease_age_s': None if lv is None else wall - lv,
            'commands_accepted': self.lease.accepted,
            'command_rejections': dict(self.lease.rejections),
            'goal': None if g is None else {
                'seq': g.seq, **g.goal.summary(), 'accepted': g.accepted,
                'elapsed_s': wall - g.sent_wall, 'cancel_sent': g.cancel_sent,
                'max_tracking_error_rad': g.max_error},
            'faults': list(self.faults), 'last_refusal': self.last_refusal,
            'phases_completed': self.phases_completed, 'cycles': self.cycles,
            'planned_displacement_xy_m': list(self.planned_xy),
            'continuity_reference': 'planned (shadow: nothing moves)' if self.shadow
            else 'measured',
            'goals_recorded': getattr(self.transport, 'sent', None),
            'monitors': mon, 'ground_truth': gt,
        }


__all__ = ['LocomotionConfig', 'LocomotionError', 'GoalRefused', 'DispatchDisabled',
           'parse_config', 'load_config', 'config_path', 'PhaseGoal', 'PhaseLibrary',
           'build_library', 'goal_fingerprint', 'verify_goal', 'ShadowTransport',
           'LocomotionSession', 'STATES', 'DISARMED', 'READY', 'STARTING', 'WALKING', 'STOPPING',
           'FAULTED', 'FAULT_REASONS', 'CONFIG_SCHEMA', 'STATUS_SCHEMA', 'GOAL_SCHEMA']
