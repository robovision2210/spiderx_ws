"""M6.1 trot cycle: generator, strict loader, content hash, re-derivation and offline preflight.

Pure Python (no ROS graph). The ONE approved trajectory is stored in config/m61_trot_cycle.yaml
(owner decision); it is accepted only if, independently,
  1. its canonical content hashes to the stored and the approved content_sha256 (design-note
     scheme, docs/M61_TROT_REPLAY_DESIGN.md section 2.3);
  2. every waypoint equals the waypoint re-derived from the approved parameters by the design
     generator below (offline IK, leg_kinematics) within m61_limits.yaml source_match_rad;
  3. it passes the M6.1 envelope (m61_limits.yaml): point count, segment and total duration,
     neutral endpoints with zero velocity, the 0.1223 rad displacement cap, URDF soft limits, the
     planned joint speed and the G3 limit fraction - checked on the waypoints AND on the dense
     cubic-Hermite spline the controller will interpolate (JTC 'splines', positions + velocities).

The trajectory is a start-stop single trot cycle: neutral -> pair A (LF + RR) swings while pair B
(RF + LR) is in stance -> pair B swings while pair A is in stance -> neutral. Frame base_link:
+x right, +y front, +z up. Nothing here sends or builds a goal message (see m61_goal).
"""

from dataclasses import dataclass, field
import hashlib
import json
import math
import os

from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m61_limits
from spiderx_controller import m61_live_contract as c61
from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_trajectory as m6t

FILE_NAME = 'm61_trot_cycle.yaml'
FILE_SCHEMA = 'spiderx.m61.trot_cycle/1'
DESIGN_HASH_SCHEMA = 'spiderx.m61.trot_cycle.design/0'
TRAJECTORY_SCHEMA = 'spiderx.m61.trajectory/1'
REPORT_SCHEMA = 'spiderx.m61.preflight/1'
MILESTONE = 'M6.1'
MODE = 'single_trot_cycle'
GENERATOR = 'm61_trot_cycle.generate_points (design note section 2.3)'
FD_EPS_S = 1e-5                      # central-difference step of the generator velocities
STORE_DECIMALS = 9                   # stored waypoint precision
NEUTRAL_EXACT_RAD = 1e-9             # endpoints must equal neutral (they are +/-0.0 when stored)

GAIT_KEYS = ('gait_name', 'variant', 'num_cycles', 'num_waypoints', 'cycle_duration_s',
             'step_length_m', 'lift_height_m', 'stance_height_offset_m', 'lead_in_s', 'segments',
             'duty_factor', 'swing_pair_first', 'swing_pair_second', 'base_constraint')
FILE_KEYS = (('schema', 'milestone', 'simulation_only') + GAIT_KEYS
             + ('content_sha256', 'design_note_reference_sha256', 'joint_names', 'waypoints'))
WAYPOINT_KEYS = ('label', 'time_from_start_s', 'positions', 'velocities')
TRAJECTORY_KEYS = ('schema', 'milestone', 'mode', 'joint_names', 'points', 'gait', 'provenance',
                   'trajectory_id')
PROVENANCE_KEYS = ('trajectory_source', 'content_sha256', 'design_note_reference_sha256',
                   'generator', 'inputs_sha256', 'm4_config_version')

FAILURE_CODES = {
    'sources_unavailable': 'a source file, the limits file or the trot-cycle file is unusable',
    'limits_not_approved': 'm61_limits.yaml differs from the owner-approved values',
    'limits_inconsistent_with_m6d_monitor': 'a limit differs from the re-used M6.0-D monitor value',
    'gait_not_approved': 'the gait parameters differ from the owner-approved values',
    'schema_invalid': 'the trajectory structure is not the M6.1 schema',
    'joint_names_not_canonical': 'joint names differ from the canonical 12-joint controller order',
    'point_count_mismatch': 'the number of points differs from num_waypoints',
    'too_many_points': 'more points than max_trajectory_points',
    'point_invalid': 'a point is malformed (keys, lengths or types)',
    'non_finite_value': 'a NaN or infinite number',
    'first_time_not_lead_in': 'the first point is not at lead_in_s',
    'times_not_increasing': 'time_from_start is not strictly increasing',
    'segment_too_short': 'a segment is shorter than min_segment_duration_s',
    'duration_exceeds_max': 'the goal is longer than max_duration_s',
    'cycle_duration_mismatch': 'last minus first point time differs from cycle_duration_s',
    'start_not_neutral': 'the first point is not CAD neutral',
    'end_not_neutral': 'the last point is not CAD neutral',
    'endpoint_velocity_nonzero': 'the first or last point velocity is not zero',
    'displacement_exceeds_cap': '|q - neutral| above max_joint_displacement_rad (+ epsilon)',
    'joint_limit_violation': 'the planned path leaves the URDF limits minus the soft margin',
    'planned_limit_fraction_exceeded': 'the planned path would itself trip the G3 gate',
    'joint_speed_exceeds_limit': 'the planned spline speed is above max_planned_joint_speed',
    'content_hash_mismatch': 'the content hash differs from the stored content_sha256',
    'content_not_approved': 'the content hash differs from the approved content_sha256',
    'rederivation_failed': 'the generator could not re-derive the waypoints (IK)',
    'rederivation_mismatch': 'a waypoint differs from its re-derived value',
    'trajectory_id_mismatch': 'trajectory_id does not match the trajectory content',
}

CHECKS = ('sources', 'limits', 'gait', 'schema', 'joints', 'structure', 'numeric', 'timing',
          'endpoints', 'displacement_cap', 'joint_limits', 'joint_speed', 'content_hash',
          'rederivation', 'identity')


class TrotCycleError(ValueError):
    """The trot-cycle file is malformed or cannot be generated; nothing may be built or sent."""

    def __init__(self, code, message):
        super().__init__(f'{code}: {message}')
        self.code = code


class TrajectoryBuildError(ValueError):
    """The M6.1 sources are not usable; no trajectory is built."""


# ---------------------------------------------------------------- parameters and generator
@dataclass(frozen=True)
class GaitParams:
    cycle_duration_s: float
    step_length_m: float
    lift_height_m: float
    stance_height_offset_m: float
    lead_in_s: float
    segments: int
    pair_a: tuple = ('front_left', 'rear_right')       # swings in the first half cycle

    @classmethod
    def from_gait(cls, gait):
        return cls(gait['cycle_duration_s'], gait['step_length_m'], gait['lift_height_m'],
                   gait['stance_height_offset_m'], gait['lead_in_s'], gait['segments'],
                   tuple(gait['swing_pair_first']))

    def design_params(self):
        """The params block of the design-note content hash (key names as in the design)."""
        return {'cycle_period_s': self.cycle_duration_s, 'step_length_m': self.step_length_m,
                'step_height_m': self.lift_height_m,
                'stance_height_offset_m': self.stance_height_offset_m,
                'lead_in_s': self.lead_in_s, 'segments': self.segments}


def body_advance(p, t):
    """Body advance along +y at cycle time t; cycloid, zero speed at 0 and T."""
    T = p.cycle_duration_s
    return p.step_length_m * (t / T - math.sin(2 * math.pi * t / T) / (2 * math.pi))


def foot_offset(p, leg, t):
    """Foot offset (x, y, z) from the CAD-neutral tip, base_link, at cycle time t.

    Exactly the design-note generator (section 2.3): the arithmetic order is kept so that the
    re-derived values reproduce the stored ones bit-for-bit on the reference platform.
    """
    STEP, LIFT, Z0 = p.step_length_m, p.lift_height_m, p.stance_height_offset_m
    tau, a = p.cycle_duration_s / 2, leg in p.pair_a
    t0, y0 = (0.0, 0.0) if a else (tau, -STEP / 2)
    if t0 <= t <= t0 + tau:                      # swing: world cycloid, STEP forward, LIFT high
        u = (t - t0) / tau
        y = (y0 + STEP * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
             - (body_advance(p, t) - body_advance(p, t0)))
        return (0.0, y, Z0 + LIFT * (1 - math.cos(2 * math.pi * u)) / 2)
    return (0.0, (STEP / 2 - (body_advance(p, t) - body_advance(p, tau))) if a
            else -body_advance(p, t), Z0)                                       # stance


def joint_positions(p, geoms, t, order):
    """12 joint angles (IK) at cycle time t, in `order`. Raises TrotCycleError on IK failure."""
    out = {}
    for leg in lk.ALL_LEGS:
        g = geoms[leg]
        r = lk.inverse(g, tuple(a + o for a, o in zip(g.tip0, foot_offset(p, leg, t))),
                       margin=lk.DEFAULT_MARGIN_RAD)
        if not r['ok']:
            raise TrotCycleError('rederivation_failed', f'IK failed for {leg} at t = {t} s: '
                                                        f'{r["reason"]}')
        out.update(zip(g.joint_names, r['solution']))
    return [out[n] for n in order]


def generate_points(p, geoms, order):
    """The segments + 1 waypoints {time_from_start_s, positions, velocities}, rounded as stored."""
    pts = []
    for k in range(p.segments + 1):
        t = k * p.cycle_duration_s / p.segments
        q = joint_positions(p, geoms, t, order)
        if k in (0, p.segments):
            v = [0.0] * len(order)
        else:
            a = joint_positions(p, geoms, t - FD_EPS_S, order)
            b = joint_positions(p, geoms, t + FD_EPS_S, order)
            v = [(y - x) / (2 * FD_EPS_S) for x, y in zip(a, b)]
        pts.append({'time_from_start_s': round(p.lead_in_s + t, 6),
                    'positions': [round(x, STORE_DECIMALS) for x in q],
                    'velocities': [round(x, STORE_DECIMALS) for x in v]})
    return pts


def content_doc(params, joint_names, points):
    """The design-note hash document (section 2.3)."""
    return {'schema': DESIGN_HASH_SCHEMA, 'joint_names': list(joint_names),
            'params': dict(params),
            'points': [{'time_from_start_s': p['time_from_start_s'],
                        'positions': list(p['positions']),
                        'velocities': list(p['velocities'])} for p in points]}


def content_sha256(params, joint_names, points):
    blob = json.dumps(content_doc(params, joint_names, points), sort_keys=True,
                      separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(blob).hexdigest()


# ---------------------------------------------------------------- the stored file
def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def parse_file(data):
    """Validate the structure of a parsed m61_trot_cycle.yaml. Returns it. Raises TrotCycleError."""
    if not isinstance(data, dict):
        raise TrotCycleError('schema_invalid', 'top level must be a mapping')
    unknown, missing = sorted(set(data) - set(FILE_KEYS)), sorted(set(FILE_KEYS) - set(data))
    if unknown or missing:
        raise TrotCycleError('schema_invalid', f'unknown keys {unknown}, missing keys {missing}')
    if data['schema'] != FILE_SCHEMA or data['milestone'] != MILESTONE or \
            data['simulation_only'] is not True:
        raise TrotCycleError('schema_invalid', 'schema/milestone/simulation_only differ')
    for k in ('cycle_duration_s', 'step_length_m', 'lift_height_m', 'stance_height_offset_m',
              'lead_in_s', 'duty_factor'):
        if not _is_number(data[k]) or not math.isfinite(data[k]):
            raise TrotCycleError('schema_invalid', f'{k} must be a finite number')
    for k in ('num_cycles', 'num_waypoints', 'segments'):
        if not isinstance(data[k], int) or isinstance(data[k], bool) or data[k] <= 0:
            raise TrotCycleError('schema_invalid', f'{k} must be a positive integer')
    for k in ('content_sha256', 'design_note_reference_sha256'):
        v = data[k]
        if not (isinstance(v, str) and len(v) == 64 and all(c in '0123456789abcdef' for c in v)):
            raise TrotCycleError('schema_invalid', f'{k} must be 64 lowercase hex characters')
    for k in ('swing_pair_first', 'swing_pair_second'):
        if not (isinstance(data[k], list) and len(data[k]) == 2
                and all(leg in lk.ALL_LEGS for leg in data[k])):
            raise TrotCycleError('schema_invalid', f'{k} must name two legs of {lk.ALL_LEGS}')
    if sorted(data['swing_pair_first'] + data['swing_pair_second']) != sorted(lk.ALL_LEGS):
        raise TrotCycleError('schema_invalid', 'the two swing pairs must cover all four legs')
    if not (isinstance(data['joint_names'], list)
            and all(isinstance(n, str) for n in data['joint_names'])):
        raise TrotCycleError('schema_invalid', 'joint_names must be a list of names')
    wps = data['waypoints']
    if not isinstance(wps, list) or not wps:
        raise TrotCycleError('schema_invalid', 'waypoints must be a non-empty list')
    for i, w in enumerate(wps):
        if not isinstance(w, dict) or sorted(w) != sorted(WAYPOINT_KEYS):
            raise TrotCycleError('schema_invalid', f'waypoint {i} keys must be {WAYPOINT_KEYS}')
        if not isinstance(w['label'], str):
            raise TrotCycleError('schema_invalid', f'waypoint {i} label must be a string')
        if not _is_number(w['time_from_start_s']):
            raise TrotCycleError('schema_invalid', f'waypoint {i} time must be a number')
        for k in ('positions', 'velocities'):
            if not (isinstance(w[k], list) and len(w[k]) == len(data['joint_names'])
                    and all(isinstance(v, float) for v in w[k])):
                raise TrotCycleError('schema_invalid', f'waypoint {i} {k} must be '
                                     f'{len(data["joint_names"])} floats')
    return data


def file_path(config_dir=None):
    return os.path.join(config_dir or m6t.default_config_dir(), FILE_NAME)


def load_file(config_dir=None):
    from spiderx_controller.posture_config import PostureConfigError, load_yaml_strict
    try:
        data = load_yaml_strict(file_path(config_dir))
    except PostureConfigError as e:
        raise TrotCycleError('schema_invalid', str(e)) from e
    return parse_file(data)


# ---------------------------------------------------------------- the plan (all inputs)
@dataclass(frozen=True)
class Plan:
    """Everything the M6.1 build and preflight read. errors: ((code, message), ...)."""

    config_dir: str
    sources: object = None                 # m6_trajectory.Sources (joint order, limits, neutral)
    limits: object = None                  # m61_limits.Limits
    cycle: dict = None                     # parsed m61_trot_cycle.yaml
    geoms: dict = None                     # {leg: LegGeometry} for the re-derivation
    inputs_sha256: dict = field(default_factory=dict)
    errors: tuple = ()

    @property
    def neutral(self):
        return list(self.sources.poses[env.NEUTRAL_LABEL])


def load_plan(config_dir=None, urdf_root=None):
    """Read every M6.1 input. Never raises for a bad input: problems become Plan.errors."""
    config_dir = config_dir or m6t.default_config_dir()
    errors = []
    if urdf_root is None:
        try:
            from spiderx_controller.config_check import load_urdf
            urdf_root = load_urdf()
        except Exception as e:  # noqa: BLE001 - recorded; nothing may be built
            return Plan(config_dir=config_dir,
                        errors=(('sources_unavailable', f'URDF: {type(e).__name__}: {e}'),))
    sources = m6t.load_sources(config_dir, urdf_root)
    errors += [('sources_unavailable', f'{c}: {m}') for c, m in sources.errors]
    hashes = dict(sources.inputs_sha256)
    limits = cycle = geoms = None
    for name in (m61_limits.FILE_NAME, FILE_NAME):
        path = os.path.join(config_dir, name)
        if os.path.isfile(path):
            hashes[name] = m6t.sha256_file(path)
    try:
        limits = m61_limits.load(config_dir)
    except m61_limits.LimitsError as e:
        errors.append(('sources_unavailable', f'{m61_limits.FILE_NAME}: {e}'))
    try:
        cycle = load_file(config_dir)
    except TrotCycleError as e:
        errors.append(('sources_unavailable', f'{FILE_NAME}: {e}'))
    try:
        geoms = lk.load_all_geometries(urdf_root=urdf_root, config_dir=config_dir)
    except Exception as e:  # noqa: BLE001
        errors.append(('sources_unavailable', f'geometry: {type(e).__name__}: {e}'))
    return Plan(config_dir=config_dir, sources=sources, limits=limits, cycle=cycle, geoms=geoms,
                inputs_sha256=dict(sorted(hashes.items())), errors=tuple(errors))


def build_trajectory(plan):
    """The ONE M6.1 trajectory dict (the stored waypoints, M6 format). Raises TrajectoryBuildError.

    Building does not prove anything: preflight() decides whether it may be used.
    """
    if plan.errors or plan.cycle is None or plan.sources is None:
        raise TrajectoryBuildError('M6.1 inputs are not usable: ' + '; '.join(
            f'{c}: {m}' for c, m in plan.errors))
    cyc = plan.cycle
    traj = {
        'schema': TRAJECTORY_SCHEMA,
        'milestone': MILESTONE,
        'mode': MODE,
        'joint_names': list(cyc['joint_names']),
        'points': [{'label': w['label'], 'time_from_start_s': w['time_from_start_s'],
                    'positions': list(w['positions']), 'velocities': list(w['velocities'])}
                   for w in cyc['waypoints']],
        'gait': {k: (list(cyc[k]) if isinstance(cyc[k], list) else cyc[k]) for k in GAIT_KEYS},
        'provenance': {
            'trajectory_source': FILE_NAME,
            'content_sha256': cyc['content_sha256'],
            'design_note_reference_sha256': cyc['design_note_reference_sha256'],
            'generator': GENERATOR,
            'inputs_sha256': dict(plan.inputs_sha256),
            'm4_config_version': plan.sources.m4_config_version,
        },
    }
    traj['trajectory_id'] = m6t.compute_trajectory_id(traj)
    return traj


# ---------------------------------------------------------------- spline (JTC 'splines')
def _segment(pa, pb, t):
    """Cubic Hermite (positions + velocities) between two points: (q, qdot) at time t."""
    ta, tb = pa['time_from_start_s'], pb['time_from_start_s']
    dt = tb - ta
    s = (t - ta) / dt
    s2, s3 = s * s, s * s * s
    h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + s, -2 * s3 + 3 * s2, s3 - s2
    d00, d10, d01, d11 = 6 * s2 - 6 * s, 3 * s2 - 4 * s + 1, -6 * s2 + 6 * s, 3 * s2 - 2 * s
    q = [h00 * qa + h10 * dt * va + h01 * qb + h11 * dt * vb
         for qa, va, qb, vb in zip(pa['positions'], pa['velocities'], pb['positions'],
                                   pb['velocities'])]
    qd = [(d00 * qa + d01 * qb) / dt + d10 * va + d11 * vb
          for qa, va, qb, vb in zip(pa['positions'], pa['velocities'], pb['positions'],
                                    pb['velocities'])]
    return q, qd


def reference_state(trajectory, t):
    """(positions, velocities) of the commanded spline at time t (s after acceptance), or None
    before the first point. After the last point: the last point at rest."""
    pts = trajectory['points']
    times = [p['time_from_start_s'] for p in pts]
    if t < times[0]:
        return None
    if t >= times[-1]:
        return list(pts[-1]['positions']), [0.0] * len(pts[-1]['positions'])
    for a in range(len(pts) - 1):
        if times[a] <= t < times[a + 1]:
            return _segment(pts[a], pts[a + 1], t)
    return None


def reference_positions(trajectory, t):
    """Commanded positions at time t (cubic Hermite with the waypoint velocities), or None."""
    st = reference_state(trajectory, t)
    return None if st is None else st[0]


def evaluate_tracking(trajectory, samples, tolerance):
    """Independent tracking (M6.0-D channel 2) against the M6.1 Hermite reference.

    samples: [(t_s, {joint: position})]. passed iff every joint of every sample from the first
    point on is within tolerance, and every segment plus the end is covered by samples.
    """
    names = list(trajectory['joint_names'])
    times = [p['time_from_start_s'] for p in trajectory['points']]
    out = {'tolerance_rad': tolerance, 'reference': 'cubic Hermite (positions + velocities)',
           'samples_used': 0, 'max_error_rad': None, 'worst_joint': None, 'worst_time_s': None,
           'incomplete_samples': 0, 'uncovered_intervals': [], 'reason': None}
    if not samples:
        out['reason'] = 'no /joint_states samples'
        return ac.UNAVAILABLE, out
    worst = (-1.0, None, None)
    covered = [False] * len(times)
    for t, obs in samples:
        ref = reference_positions(trajectory, t)
        if ref is None:
            continue
        if not isinstance(obs, dict) or any(n not in obs for n in names) or not all(
                _is_number(obs[n]) and math.isfinite(obs[n]) for n in names):
            out['incomplete_samples'] += 1
            continue
        out['samples_used'] += 1
        idx = len(times) - 1 if t >= times[-1] else max(i for i in range(len(times) - 1)
                                                        if times[i] <= t)
        covered[idx] = True
        for n, r in zip(names, ref):
            e = abs(obs[n] - r)
            if e > worst[0]:
                worst = (e, n, t)
    out['uncovered_intervals'] = [i for i, c in enumerate(covered) if not c]
    if out['samples_used'] == 0:
        out['reason'] = 'no complete sample after the first point'
        return ac.UNAVAILABLE, out
    out['max_error_rad'], out['worst_joint'], out['worst_time_s'] = worst
    if out['incomplete_samples']:
        out['reason'] = f'{out["incomplete_samples"]} incomplete or non-finite samples'
        return ac.FAILED, out
    if out['uncovered_intervals']:
        out['reason'] = f'no samples in intervals {out["uncovered_intervals"]}'
        return ac.FAILED, out
    if worst[0] > tolerance:
        out['reason'] = (f'{worst[1]} error {worst[0]:.4f} rad > {tolerance} rad at '
                         f't = {worst[2]} s')
        return ac.FAILED, out
    return ac.PASSED, out


def limit_fraction(q, lower, upper):
    """How far q is toward the URDF limit on its own side (0 at 0 rad, 1 at the limit)."""
    if q > 0:
        return q / upper if upper > 0 else math.inf
    if q < 0:
        return q / lower if lower < 0 else math.inf
    return 0.0


def spline_extremes(trajectory, neutral, limits_by_joint, soft_margin, samples_per_segment):
    """Dense check of the commanded spline: per-joint extremes, speeds and limit use."""
    names = list(trajectory['joint_names'])
    pts = trajectory['points']
    per = {n: {'max_displacement_rad': 0.0, 'max_speed_rad_s': 0.0, 'max_limit_fraction': 0.0,
               'min_soft_limit_margin_rad': math.inf} for n in names}
    for a in range(len(pts) - 1):
        ta, tb = pts[a]['time_from_start_s'], pts[a + 1]['time_from_start_s']
        for i in range(samples_per_segment + 1):
            t = ta + (tb - ta) * i / samples_per_segment
            q, qd = _segment(pts[a], pts[a + 1], t)
            for n, v, vd, q0 in zip(names, q, qd, neutral):
                lo, hi = limits_by_joint[n]
                r = per[n]
                r['max_displacement_rad'] = max(r['max_displacement_rad'], abs(v - q0))
                r['max_speed_rad_s'] = max(r['max_speed_rad_s'], abs(vd))
                r['max_limit_fraction'] = max(r['max_limit_fraction'], limit_fraction(v, lo, hi))
                r['min_soft_limit_margin_rad'] = min(r['min_soft_limit_margin_rad'],
                                                     v - (lo + soft_margin),
                                                     (hi - soft_margin) - v)
    return per


# ---------------------------------------------------------------- preflight
@dataclass(frozen=True)
class PreflightReport:
    ok: bool
    trajectory_id: object
    failures: tuple
    checks: dict
    summary: dict

    @property
    def codes(self):
        return sorted({c for c, _ in self.failures})

    def to_dict(self):
        return {'schema': REPORT_SCHEMA, 'ok': self.ok,
                'verdict': 'PASS' if self.ok else 'REFUSED',
                'trajectory_id': self.trajectory_id, 'failure_codes': self.codes,
                'failures': [{'code': c, 'message': m} for c, m in self.failures],
                'checks': dict(self.checks), 'summary': dict(self.summary),
                'non_claims': list(c61.NON_CLAIMS)}


class _Collector:
    def __init__(self):
        self.failures = []
        self.checks = {c: 'skipped' for c in CHECKS}

    def fail(self, check, code, message):
        if code not in FAILURE_CODES:
            raise KeyError(f'undocumented failure code {code!r}')
        self.failures.append((code, message))
        self.checks[check] = 'failed'

    def passed(self, check):
        if self.checks[check] != 'failed':
            self.checks[check] = 'passed'

    def failed(self, *checks):
        return any(self.checks[c] == 'failed' for c in checks)


def _check_gait(gait, c):
    if not isinstance(gait, dict):
        c.fail('gait', 'gait_not_approved', 'gait block missing')
        return
    for k, v in c61.APPROVED_GAIT.items():
        got = gait.get(k)
        if type(got) is not type(v) or got != v:
            c.fail('gait', 'gait_not_approved', f'{k} = {got!r}, approved {v!r}')
    unknown = sorted(set(gait) - set(c61.APPROVED_GAIT))
    if unknown:
        c.fail('gait', 'gait_not_approved', f'unknown gait keys {unknown}')
    c.passed('gait')


def _check_structure(traj, plan, c):
    if not isinstance(traj, dict) or sorted(traj) != sorted(TRAJECTORY_KEYS):
        c.fail('schema', 'schema_invalid', 'trajectory keys differ from the M6.1 schema')
        return False
    if (traj['schema'], traj['milestone'], traj['mode']) != (TRAJECTORY_SCHEMA, MILESTONE, MODE):
        c.fail('schema', 'schema_invalid', 'schema / milestone / mode differ')
    prov = traj['provenance']
    if not isinstance(prov, dict) or sorted(prov) != sorted(PROVENANCE_KEYS):
        c.fail('schema', 'schema_invalid', 'provenance keys differ')
    c.passed('schema')
    if list(traj['joint_names']) != list(plan.sources.joint_names) or \
            len(traj['joint_names']) != 12:
        c.fail('joints', 'joint_names_not_canonical', f'{traj["joint_names"]!r}')
    c.passed('joints')
    pts = traj['points']
    if not isinstance(pts, list) or not pts:
        c.fail('structure', 'point_invalid', 'points must be a non-empty list')
        return False
    gait = traj['gait'] if isinstance(traj['gait'], dict) else {}
    if len(pts) != gait.get('num_waypoints'):
        c.fail('structure', 'point_count_mismatch',
               f'{len(pts)} points, num_waypoints {gait.get("num_waypoints")!r}')
    if len(pts) > plan.limits.max_trajectory_points:
        c.fail('structure', 'too_many_points',
               f'{len(pts)} > max_trajectory_points {plan.limits.max_trajectory_points}')
    for i, p in enumerate(pts):
        if not isinstance(p, dict) or sorted(p) != sorted(WAYPOINT_KEYS):
            c.fail('structure', 'point_invalid', f'point {i} keys')
            continue
        for k in ('positions', 'velocities'):
            if not (isinstance(p[k], list) and len(p[k]) == 12
                    and all(_is_number(v) for v in p[k])):
                c.fail('structure', 'point_invalid', f'point {i} {k} must be 12 numbers')
        if not _is_number(p['time_from_start_s']):
            c.fail('structure', 'point_invalid', f'point {i} time must be a number')
    c.passed('structure')
    if c.failed('structure'):
        return False
    for i, p in enumerate(pts):
        if not all(math.isfinite(v) for v in p['positions'] + p['velocities']
                   + [p['time_from_start_s']]):
            c.fail('numeric', 'non_finite_value', f'point {i}')
    c.passed('numeric')
    return not c.failed('numeric', 'joints')


def _check_timing(pts, gait, lim, c):
    times = [p['time_from_start_s'] for p in pts]
    if abs(times[0] - lim.lead_in_s) > 1e-9:
        c.fail('timing', 'first_time_not_lead_in', f'first point at {times[0]} s, lead-in '
                                                   f'{lim.lead_in_s} s')
    for a, b in zip(times, times[1:]):
        if not b > a:
            c.fail('timing', 'times_not_increasing', f'{a} -> {b}')
        elif b - a < lim.min_segment_duration_s - 1e-9:
            c.fail('timing', 'segment_too_short', f'{a} -> {b} ({b - a:.6f} s < '
                                                  f'{lim.min_segment_duration_s} s)')
    if times[-1] > lim.max_duration_s:
        c.fail('timing', 'duration_exceeds_max', f'{times[-1]} s > {lim.max_duration_s} s')
    if abs((times[-1] - times[0]) - gait.get('cycle_duration_s', math.nan)) > 1e-9:
        c.fail('timing', 'cycle_duration_mismatch',
               f'{times[-1] - times[0]} s vs cycle_duration_s {gait.get("cycle_duration_s")!r}')
    c.passed('timing')


def _check_endpoints(pts, neutral, c):
    for label, p, code in (('first', pts[0], 'start_not_neutral'),
                           ('last', pts[-1], 'end_not_neutral')):
        worst = max(abs(a - b) for a, b in zip(p['positions'], neutral))
        if worst > NEUTRAL_EXACT_RAD:
            c.fail('endpoints', code, f'{label} point differs from neutral by {worst:.3e} rad')
        if any(v != 0.0 for v in p['velocities']):
            c.fail('endpoints', 'endpoint_velocity_nonzero', f'{label} point velocities')
    c.passed('endpoints')


def _check_rederivation(traj, plan, lim, c):
    """Re-derive the waypoints from the gait parameters (IK) and compare. Returns the max diff."""
    try:
        params = GaitParams.from_gait(traj['gait'])
        regen = generate_points(params, plan.geoms, list(plan.sources.joint_names))
    except TrotCycleError as e:
        c.fail('rederivation', 'rederivation_failed', str(e))
        return None
    except (KeyError, TypeError, ValueError, ZeroDivisionError) as e:
        c.fail('rederivation', 'rederivation_failed', f'{type(e).__name__}: {e}')
        return None
    if len(regen) != len(traj['points']):
        c.fail('rederivation', 'rederivation_mismatch',
               f'{len(regen)} re-derived points vs {len(traj["points"])} stored')
        return None
    worst = 0.0
    for i, (r, p) in enumerate(zip(regen, traj['points'])):
        if abs(r['time_from_start_s'] - p['time_from_start_s']) > 1e-9:
            c.fail('rederivation', 'rederivation_mismatch', f'point {i} time')
        for k in ('positions', 'velocities'):
            d = max(abs(a - b) for a, b in zip(r[k], p[k]))
            worst = max(worst, d)
            if d > lim.source_match_rad:
                c.fail('rederivation', 'rederivation_mismatch',
                       f'point {i} {k} differ from the re-derived values by {d:.3e} '
                       f'> {lim.source_match_rad}')
    c.passed('rederivation')
    return worst


def preflight(trajectory, plan):
    """Every M6.1 offline check, in order. Never raises for bad content."""
    c = _Collector()
    summary = {}
    if plan.errors or plan.limits is None or plan.sources is None or plan.geoms is None:
        for code, msg in plan.errors or (('sources_unavailable', 'plan incomplete'),):
            c.fail('sources', 'sources_unavailable', msg)
        return PreflightReport(False, (trajectory or {}).get('trajectory_id')
                               if isinstance(trajectory, dict) else None,
                               tuple(c.failures), c.checks, summary)
    c.passed('sources')
    lim = plan.limits
    for code, msg in c61.limits_problems(lim):
        c.fail('limits', code, msg)
    c.passed('limits')
    tid = trajectory.get('trajectory_id') if isinstance(trajectory, dict) else None
    if not _check_structure(trajectory, plan, c):
        return PreflightReport(False, tid, tuple(c.failures), c.checks, summary)
    gait = trajectory['gait'] if isinstance(trajectory['gait'], dict) else {}
    _check_gait(trajectory['gait'], c)
    pts = trajectory['points']
    _check_timing(pts, gait, lim, c)
    neutral = plan.neutral
    _check_endpoints(pts, neutral, c)
    # displacement cap, limits and speed on the waypoints AND the dense spline
    names = list(trajectory['joint_names'])
    wp_disp = max(abs(v - q0) for p in pts for v, q0 in zip(p['positions'], neutral))
    per = spline_extremes(trajectory, neutral, plan.sources.limits, plan.sources.soft_margin_rad,
                          lim.spline_check_samples_per_segment)
    worst_disp = max(per, key=lambda n: per[n]['max_displacement_rad'])
    worst_speed = max(per, key=lambda n: per[n]['max_speed_rad_s'])
    worst_frac = max(per, key=lambda n: per[n]['max_limit_fraction'])
    worst_margin = min(per, key=lambda n: per[n]['min_soft_limit_margin_rad'])
    if not lim.displacement_within_cap(wp_disp):
        c.fail('displacement_cap', 'displacement_exceeds_cap',
               f'waypoint displacement {wp_disp:.6f} rad > {lim.max_joint_displacement_rad}')
    if not lim.displacement_within_cap(per[worst_disp]['max_displacement_rad']):
        c.fail('displacement_cap', 'displacement_exceeds_cap',
               f'spline displacement {per[worst_disp]["max_displacement_rad"]:.6f} rad '
               f'({worst_disp}) > {lim.max_joint_displacement_rad}')
    c.passed('displacement_cap')
    if per[worst_margin]['min_soft_limit_margin_rad'] < 0:
        c.fail('joint_limits', 'joint_limit_violation',
               f'{worst_margin} leaves the URDF limits minus the soft margin by '
               f'{-per[worst_margin]["min_soft_limit_margin_rad"]:.6f} rad')
    if per[worst_frac]['max_limit_fraction'] >= lim.joint_limit_fraction:
        c.fail('joint_limits', 'planned_limit_fraction_exceeded',
               f'{worst_frac} reaches {per[worst_frac]["max_limit_fraction"]:.4f} of its limit '
               f'(G3 trips at {lim.joint_limit_fraction})')
    c.passed('joint_limits')
    if per[worst_speed]['max_speed_rad_s'] > lim.max_planned_joint_speed_rad_s:
        c.fail('joint_speed', 'joint_speed_exceeds_limit',
               f'{worst_speed} {per[worst_speed]["max_speed_rad_s"]:.4f} rad/s > '
               f'{lim.max_planned_joint_speed_rad_s} rad/s')
    c.passed('joint_speed')
    # content identity
    digest = None
    try:
        digest = content_sha256(GaitParams.from_gait(gait).design_params(), names, pts)
    except (KeyError, TypeError, ValueError) as e:
        c.fail('content_hash', 'content_hash_mismatch', f'cannot hash: {type(e).__name__}: {e}')
    if digest is not None:
        stored = trajectory['provenance'].get('content_sha256')
        if digest != stored:
            c.fail('content_hash', 'content_hash_mismatch',
                   f'content hashes to {digest[:16]}, stored {str(stored)[:16]}')
        if digest != c61.APPROVED_CONTENT_SHA256:
            c.fail('content_hash', 'content_not_approved',
                   f'content hashes to {digest[:16]}, approved '
                   f'{c61.APPROVED_CONTENT_SHA256[:16]}')
    c.passed('content_hash')
    rederive = _check_rederivation(trajectory, plan, lim, c) if not c.failed('gait') else None
    if tid != m6t.compute_trajectory_id(trajectory):
        c.fail('identity', 'trajectory_id_mismatch', f'{tid!r}')
    c.passed('identity')
    summary.update({
        'point_count': len(pts),
        'duration_s': pts[-1]['time_from_start_s'],
        'cycle_duration_s': gait.get('cycle_duration_s'),
        'content_sha256': digest,
        'max_waypoint_displacement_rad': wp_disp,
        'max_spline_displacement_rad': per[worst_disp]['max_displacement_rad'],
        'max_spline_displacement_joint': worst_disp,
        'max_spline_speed_rad_s': per[worst_speed]['max_speed_rad_s'],
        'max_spline_speed_joint': worst_speed,
        'max_limit_fraction': per[worst_frac]['max_limit_fraction'],
        'max_limit_fraction_joint': worst_frac,
        'min_soft_limit_margin_rad': per[worst_margin]['min_soft_limit_margin_rad'],
        'rederivation_max_diff': rederive,
        'per_joint': per,
    })
    return PreflightReport(not c.failures, tid, tuple(c.failures), c.checks, summary)


__all__ = ['GaitParams', 'Plan', 'PreflightReport', 'TrotCycleError', 'TrajectoryBuildError',
           'FAILURE_CODES', 'body_advance', 'foot_offset', 'generate_points', 'content_sha256',
           'load_file', 'parse_file', 'load_plan', 'build_trajectory', 'preflight',
           'reference_state', 'reference_positions', 'evaluate_tracking', 'limit_fraction',
           'spline_extremes']
