"""M6.0 offline conversion and trajectory preflight (Batch A). Pure Python: no ROS graph.

Converts the M4-validated crouch_10mm pose into exactly ONE canonical all-12-joint trajectory

    neutral -> crouch_10mm -> neutral

and preflights any such trajectory against the fixed M6.0 envelope (m6_envelope). The preflight
runs before any action goal can exist: the M6 action client (m6_action_client) builds a goal only
from a trajectory whose preflight passes.

Sources (all read, never written):
  - spiderx_poses.yaml cad_neutral          the verified neutral (all zeros); cross-checked against
                                            the M4 IK neutral_stance command
  - m4_pose_targets.yaml crouch_10mm        the exact M4-validated IK command (m4_pose_targets)
  - spiderx_ros2_controllers.yaml           canonical controller joint order (must equal
                                            leg_kinematics.all_joint_names and spiderx_legs.yaml)
  - spiderx_legs.yaml                       URDF limits, soft margin, joint speed placeholder
  - the expanded URDF                       hashed for provenance

Every failure has a machine-readable code (FAILURE_CODES). Nothing here sends or builds a goal.
"""

from dataclasses import dataclass, field
import hashlib
import json
import math
import os
import xml.etree.ElementTree as ET

from spiderx_controller import m6_envelope as env

SCHEMA = 'spiderx.m6.trajectory/1'
REPORT_SCHEMA = 'spiderx.m6.preflight/1'
MILESTONE = 'M6.0-D'
MODE = 'single'
TRAJECTORY_KEYS = ('schema', 'milestone', 'mode', 'joint_names', 'points', 'provenance',
                   'trajectory_id')
POINT_KEYS = ('label', 'time_from_start_s', 'positions', 'velocities')
PROVENANCE_KEYS = ('pose', 'pose_source', 'neutral_source', 'joint_order_source',
                   'inputs_sha256', 'm4_config_version')
INPUT_FILES = ('m4_pose_targets.yaml', 'spiderx_legs.yaml', 'spiderx_poses.yaml',
               'spiderx_ros2_controllers.yaml')
POSE_SOURCE = 'm4_pose_targets.yaml#poses.crouch_10mm (m4_pose_targets.pose_command)'
NEUTRAL_SOURCE = 'spiderx_poses.yaml#poses.cad_neutral'
JOINT_ORDER_SOURCE = 'spiderx_ros2_controllers.yaml#leg_trajectory_controller.joints'
NEUTRAL_AGREEMENT_RAD = 1e-9      # M4 IK neutral_stance vs CAD neutral (float round-off only)
SOURCE_MATCH_RAD = 1e-12          # trajectory waypoint vs recomputed source pose

FAILURE_CODES = {
    # sources
    'source_unavailable': 'a configuration file, the URDF or the M4 evaluation could not be loaded',
    'neutral_source_missing': 'spiderx_poses.yaml cad_neutral is missing or incomplete',
    'neutral_source_inconsistent': 'CAD neutral and the M4 neutral_stance IK command disagree',
    'source_pose_unavailable': 'the M4 crouch_10mm pose is not available or not safe',
    'canonical_order_inconsistent':
        'controller YAML, URDF and spiderx_legs.yaml joint orders differ',
    'soft_margin_inconsistent': 'spiderx_legs.yaml soft margin differs from joint_safety',
    # structure
    'schema_invalid': 'not a trajectory of the M6.0-D schema (unknown or missing fields)',
    'mode_not_single': 'only one single, non-repeating trajectory is allowed',
    'joint_count': 'the trajectory does not name exactly 12 joints',
    'joint_names_not_unique': 'a joint name appears more than once',
    'joint_names_unknown': 'joint names differ from the 12 SpiderX controller joints',
    'joint_order_not_canonical': 'joints are not in the canonical controller order',
    'empty_trajectory': 'the trajectory has no points',
    'too_many_points': f'more than {env.MAX_POINTS} points',
    'positions_incomplete': 'a point does not give exactly 12 positions',
    'velocities_incomplete': 'a point does not give exactly 12 velocities',
    'non_numeric_value': 'a time, position or velocity is not a number',
    'non_finite_value': 'a time, position or velocity is NaN or infinite',
    # timing
    'start_delay_missing': 'the first point is at time_from_start <= 0 (a start delay is required)',
    'start_delay_too_short': 'the start delay is shorter than the lead-in rule',
    'time_not_strictly_increasing': 'time_from_start is not strictly increasing',
    'duration_exceeds_max': f'total duration exceeds {env.MAX_DURATION_S} s',
    'segment_too_fast': 'a segment is shorter than max(3.0 s, 2 x max|dq| / v_max)',
    'nonzero_velocity': 'every waypoint velocity must be exactly 0',
    # content and envelope
    'waypoint_sequence_not_approved':
        'the waypoints are not exactly neutral -> crouch_10mm -> neutral',
    'source_pose_mismatch': 'a waypoint differs from its recomputed source pose',
    'displacement_exceeds_cap': 'a commanded displacement from neutral exceeds the M6.0-D cap',
    'joint_limit_violation': 'a position is outside the URDF limits minus the soft margin',
    # provenance and identity
    'provenance_missing': 'provenance is missing or incomplete',
    'provenance_mismatch': 'provenance names a different pose or source',
    'source_stale': 'a source file changed since the trajectory was converted',
    'trajectory_id_mismatch': 'trajectory_id does not match the trajectory content',
}


class TrajectoryBuildError(ValueError):
    """The sources are not usable; no trajectory is built."""


@dataclass(frozen=True)
class Sources:
    """Everything the conversion and the preflight read. errors: ((code, message), ...)."""

    config_dir: str
    joint_names: tuple = ()
    limits: dict = field(default_factory=dict)
    soft_margin_rad: float = None
    max_joint_velocity_rad_s: float = None
    poses: dict = field(default_factory=dict)
    inputs_sha256: dict = field(default_factory=dict)
    m4_config_version: object = None
    errors: tuple = ()


@dataclass(frozen=True)
class PreflightReport:
    ok: bool
    trajectory_id: object
    failures: tuple            # ((code, message), ...) in check order
    checks: dict               # check name -> 'passed' | 'failed' | 'skipped'
    summary: dict

    @property
    def codes(self):
        return sorted({c for c, _ in self.failures})

    def to_dict(self):
        return {
            'schema': REPORT_SCHEMA,
            'ok': self.ok,
            'verdict': 'PASS' if self.ok else 'REFUSED',
            'trajectory_id': self.trajectory_id,
            'failure_codes': self.codes,
            'failures': [{'code': c, 'message': m} for c, m in self.failures],
            'checks': dict(self.checks),
            'summary': dict(self.summary),
            'envelope': env.as_dict(),
            'non_claims': list(env.NON_CLAIMS),
        }


# ---------------------------------------------------------------- hashing / serialisation
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 16), b''):
            h.update(chunk)
    return h.hexdigest()


def canonical_json(data):
    """Byte-stable JSON (sorted keys, no whitespace, NaN refused)."""
    return json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False)


def compute_trajectory_id(traj):
    """16-hex content hash. Tolerates NaN and odd types so that a malformed trajectory gets an
    identity too (the preflight refuses it for its own reasons)."""
    body = {k: v for k, v in traj.items() if k != 'trajectory_id'}
    text = json.dumps(body, sort_keys=True, separators=(',', ':'), allow_nan=True, default=repr)
    return hashlib.sha256(text.encode()).hexdigest()[:16]


def dumps(data):
    """Deterministic file serialisation used for trajectory.json and preflight.json."""
    return json.dumps(data, indent=1, sort_keys=True, allow_nan=False) + '\n'


def write_json(path, data):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write(dumps(data))


def load_trajectory_file(path):
    """Parse a trajectory JSON file (NaN/Infinity literals parse; the preflight refuses them)."""
    with open(path) as f:
        return json.load(f)


# ---------------------------------------------------------------- sources
def default_config_dir():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('spiderx_controller'), 'config')


def _canonical_neutral(config_dir, names):
    """CAD neutral as a 12-tuple in canonical order, or raise LookupError."""
    from spiderx_controller.joint_safety import UnsafeCommandError, load_pose
    try:
        pose = load_pose('cad_neutral', config_dir)
    except (OSError, KeyError, TypeError, UnsafeCommandError) as e:
        raise LookupError(f'cad_neutral not available: {e}') from e
    if set(pose) != set(names):
        raise LookupError(f'cad_neutral must give exactly the 12 joints {list(names)}, '
                          f'got {sorted(pose)}')
    return tuple(float(pose[n]) for n in names)


def load_sources(config_dir=None, urdf_root=None):
    """Read every source. Never raises for a bad source: problems become Sources.errors."""
    from spiderx_controller import leg_kinematics as lk
    from spiderx_controller import m4_pose_targets as m4
    from spiderx_controller.joint_safety import DEFAULT_MARGIN_RAD, load_limits
    from spiderx_controller.posture_config import PostureConfigError, load_yaml_strict

    config_dir = config_dir or default_config_dir()
    errors = []
    hashes = {}
    for name in INPUT_FILES:
        path = os.path.join(config_dir, name)
        if os.path.isfile(path):
            hashes[name] = sha256_file(path)
        else:
            code = ('neutral_source_missing' if name == 'spiderx_poses.yaml'
                    else 'source_unavailable')
            errors.append((code, f'{name} not found in {config_dir}'))
    if errors:
        return Sources(config_dir=config_dir, inputs_sha256=hashes, errors=tuple(errors))

    try:
        legs = load_yaml_strict(os.path.join(config_dir, 'spiderx_legs.yaml'))
        margin = float(legs['soft_limit_margin_rad'])
        if abs(margin - DEFAULT_MARGIN_RAD) > 1e-12:
            errors.append(('soft_margin_inconsistent', f'spiderx_legs.yaml soft_limit_margin_rad '
                           f'{margin} != joint_safety.DEFAULT_MARGIN_RAD {DEFAULT_MARGIN_RAD}'))
        v_max = float(legs['motion_constraints']['max_joint_velocity_rad_s'])
        limits, legs_order = load_limits(config_dir)
        ctrl = load_yaml_strict(os.path.join(config_dir, 'spiderx_ros2_controllers.yaml'))
        ctrl_order = list(ctrl['leg_trajectory_controller']['ros__parameters']['joints'])
        if urdf_root is None:
            from spiderx_controller.config_check import load_urdf
            urdf_root = load_urdf()
        hashes['urdf_expanded_xml'] = hashlib.sha256(ET.tostring(urdf_root)).hexdigest()
        cfg, geoms, _, plan = m4.load_and_evaluate(config_dir=config_dir, urdf_root=urdf_root)
        urdf_order = lk.all_joint_names(geoms)
    except (PostureConfigError, m4.PoseTargetError, OSError, KeyError, TypeError,
            ValueError) as e:
        errors.append(('source_unavailable', f'{type(e).__name__}: {e}'))
        return Sources(config_dir=config_dir, inputs_sha256=hashes, errors=tuple(errors))

    if not (ctrl_order == list(urdf_order) == list(legs_order)) or len(ctrl_order) != 12:
        errors.append(('canonical_order_inconsistent',
                       f'controller {ctrl_order} / URDF {list(urdf_order)} / '
                       f'spiderx_legs {list(legs_order)}'))
    names = tuple(ctrl_order)
    poses = {}
    try:
        poses[env.CROUCH_POSE] = tuple(float(v) for v in m4.pose_command(plan, env.CROUCH_POSE))
    except m4.PoseTargetError as e:
        errors.append(('source_pose_unavailable', str(e)))
    try:
        neutral = _canonical_neutral(config_dir, names)
        poses[env.NEUTRAL_LABEL] = neutral
        m4_neutral = m4.pose_command(plan, 'neutral_stance')
        worst = max(abs(a - b) for a, b in zip(neutral, m4_neutral))
        if worst > NEUTRAL_AGREEMENT_RAD:
            errors.append(('neutral_source_inconsistent', f'CAD neutral and M4 neutral_stance '
                           f'differ by {worst:.3e} rad > {NEUTRAL_AGREEMENT_RAD}'))
    except LookupError as e:
        errors.append(('neutral_source_missing', str(e)))
    except m4.PoseTargetError as e:
        errors.append(('neutral_source_inconsistent', f'M4 neutral_stance unavailable: {e}'))
    return Sources(config_dir=config_dir, joint_names=names, limits=dict(limits),
                   soft_margin_rad=margin, max_joint_velocity_rad_s=v_max, poses=poses,
                   inputs_sha256=hashes, m4_config_version=cfg['config_version'],
                   errors=tuple(errors))


# ---------------------------------------------------------------- conversion
def lead_in_s(sources):
    """Start delay: the M3/M4 duration rule for a move of up to the start-pose tolerance."""
    return max(env.MIN_SEGMENT_S,
               env.SPEED_FACTOR * env.START_POSE_TOLERANCE_RAD / sources.max_joint_velocity_rad_s)


def segment_min_s(q_from, q_to, v_max):
    dq = max(abs(a - b) for a, b in zip(q_from, q_to))
    return max(env.MIN_SEGMENT_S, env.SPEED_FACTOR * dq / v_max)


def build_trajectory(sources):
    """The ONE canonical M6.0-D trajectory: neutral -> crouch_10mm -> neutral (deterministic)."""
    if sources.errors:
        raise TrajectoryBuildError('sources are not usable: '
                                   + '; '.join(f'{c}: {m}' for c, m in sources.errors))
    n = len(sources.joint_names)
    points, t, prev = [], lead_in_s(sources), None
    for label in env.WAYPOINT_LABELS:
        q = sources.poses[label]
        if prev is not None:
            t += segment_min_s(prev, q, sources.max_joint_velocity_rad_s)
        points.append({'label': label, 'time_from_start_s': t, 'positions': list(q),
                       'velocities': [0.0] * n})
        prev = q
    traj = {
        'schema': SCHEMA,
        'milestone': MILESTONE,
        'mode': MODE,
        'joint_names': list(sources.joint_names),
        'points': points,
        'provenance': {
            'pose': env.CROUCH_POSE,
            'pose_source': POSE_SOURCE,
            'neutral_source': NEUTRAL_SOURCE,
            'joint_order_source': JOINT_ORDER_SOURCE,
            'inputs_sha256': dict(sorted(sources.inputs_sha256.items())),
            'm4_config_version': sources.m4_config_version,
        },
    }
    traj['trajectory_id'] = compute_trajectory_id(traj)
    return traj


# ---------------------------------------------------------------- preflight
def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


class _Collector:
    def __init__(self, check_names):
        self.failures = []
        self.checks = {name: 'skipped' for name in check_names}

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


CHECKS = ('sources', 'schema', 'joints', 'points_structure', 'numeric', 'timing', 'velocities',
          'waypoints', 'displacement_cap', 'joint_limits', 'provenance', 'identity')


def _check_structure(traj, c, sources):
    if not isinstance(traj, dict):
        c.fail('schema', 'schema_invalid',
               f'trajectory must be a mapping, got {type(traj).__name__}')
        return None, None
    unknown = sorted(set(traj) - set(TRAJECTORY_KEYS))
    missing = sorted(set(TRAJECTORY_KEYS) - set(traj) - {'provenance', 'trajectory_id'})
    if unknown or missing:
        c.fail('schema', 'schema_invalid', f'unknown fields {unknown}, missing fields {missing} '
               '(there is no repeat, cycle or concatenation field)')
    if traj.get('schema') != SCHEMA or traj.get('milestone') != MILESTONE:
        c.fail('schema', 'schema_invalid', f'schema/milestone must be {SCHEMA}/{MILESTONE}, got '
               f'{traj.get("schema")!r}/{traj.get("milestone")!r}')
    if traj.get('mode') != MODE:
        c.fail('schema', 'mode_not_single', f'mode must be "{MODE}", got {traj.get("mode")!r}')
    c.passed('schema')

    names = traj.get('joint_names')
    canonical = list(sources.joint_names)
    if not isinstance(names, list) or not all(isinstance(n, str) for n in names):
        c.fail('joints', 'joint_count', f'joint_names must be a list of 12 names, got {names!r}')
        names = None
    else:
        if len(names) != 12:
            c.fail('joints', 'joint_count', f'{len(names)} joint names, expected 12')
        if len(set(names)) != len(names):
            dup = sorted({n for n in names if names.count(n) > 1})
            c.fail('joints', 'joint_names_not_unique', f'duplicate joint names {dup}')
        if set(names) != set(canonical):
            c.fail('joints', 'joint_names_unknown',
                   f'missing {sorted(set(canonical) - set(names))}, '
                   f'unknown {sorted(set(names) - set(canonical))}')
        elif names != canonical:
            c.fail('joints', 'joint_order_not_canonical',
                   f'order {names} != canonical controller order {canonical}')
    c.passed('joints')

    points = traj.get('points')
    if not isinstance(points, list):
        c.fail('points_structure', 'schema_invalid', 'points must be a list')
        return names, None
    if not points:
        c.fail('points_structure', 'empty_trajectory', 'the trajectory has no points')
    if len(points) > env.MAX_POINTS:
        c.fail('points_structure', 'too_many_points',
               f'{len(points)} points > maximum {env.MAX_POINTS}')
    for i, p in enumerate(points):
        if not isinstance(p, dict) or set(p) != set(POINT_KEYS):
            c.fail('points_structure', 'schema_invalid', f'point {i} must have exactly the fields '
                   f'{list(POINT_KEYS)}')
            continue
        if not isinstance(p['label'], str):
            c.fail('points_structure', 'schema_invalid', f'point {i}: label must be a string')
        if not isinstance(p['positions'], list) or len(p['positions']) != 12:
            c.fail('points_structure', 'positions_incomplete',
                   f'point {i}: positions must be a list of 12 values')
        if not isinstance(p['velocities'], list) or len(p['velocities']) != 12:
            c.fail('points_structure', 'velocities_incomplete',
                   f'point {i}: velocities must be a list of 12 values')
    c.passed('points_structure')
    return names, points


def _check_numeric(points, c):
    for i, p in enumerate(points):
        values = [('time_from_start_s', p['time_from_start_s'])]
        values += [(f'positions[{j}]', v) for j, v in enumerate(p['positions'])]
        values += [(f'velocities[{j}]', v) for j, v in enumerate(p['velocities'])]
        for what, v in values:
            if not _is_number(v):
                c.fail('numeric', 'non_numeric_value', f'point {i} {what} = {v!r} is not a number')
            elif not math.isfinite(v):
                c.fail('numeric', 'non_finite_value', f'point {i} {what} = {v!r} is not finite')
    c.passed('numeric')


def _check_timing(points, sources, c):
    times = [float(p['time_from_start_s']) for p in points]
    if times[0] <= 0.0:
        c.fail('timing', 'start_delay_missing',
               f'first point at {times[0]} s; a positive start delay is required')
    elif times[0] < lead_in_s(sources) - 1e-9:
        c.fail('timing', 'start_delay_too_short',
               f'start delay {times[0]} s < lead-in {lead_in_s(sources)} s')
    increasing = all(b > a for a, b in zip(times, times[1:]))
    if not increasing:
        c.fail('timing', 'time_not_strictly_increasing', f'times {times}')
    if times[-1] > env.MAX_DURATION_S:
        c.fail('timing', 'duration_exceeds_max',
               f'duration {times[-1]} s > maximum {env.MAX_DURATION_S} s')
    if increasing:
        for i in range(1, len(points)):
            need = segment_min_s(points[i - 1]['positions'], points[i]['positions'],
                                 sources.max_joint_velocity_rad_s)
            dt = times[i] - times[i - 1]
            if dt < need - 1e-9:
                c.fail('timing', 'segment_too_fast',
                       f'segment {i - 1}->{i}: {dt} s < required {need} s')
    c.passed('timing')
    for i, p in enumerate(points):
        if any(v != 0.0 for v in p['velocities']):
            c.fail('velocities', 'nonzero_velocity', f'point {i} velocities {p["velocities"]}')
    c.passed('velocities')


def _check_content(points, sources, c):
    labels = [p['label'] for p in points]
    if labels != list(env.WAYPOINT_LABELS):
        c.fail('waypoints', 'waypoint_sequence_not_approved',
               f'waypoints {labels} != approved {list(env.WAYPOINT_LABELS)}')
    for i, p in enumerate(points):
        src = sources.poses.get(p['label'])
        if src is None:
            continue
        worst = max(abs(a - b) for a, b in zip(p['positions'], src))
        if worst > SOURCE_MATCH_RAD:
            c.fail('waypoints', 'source_pose_mismatch',
                   f'point {i} ({p["label"]}) differs from its source pose by {worst:.3e} rad')
    c.passed('waypoints')

    from spiderx_controller.joint_safety import UnsafeCommandError, check_target
    neutral = sources.poses[env.NEUTRAL_LABEL]
    names = list(sources.joint_names)
    for i, p in enumerate(points):
        for j, (q, q0) in enumerate(zip(p['positions'], neutral)):
            d = q - q0
            if not env.displacement_within_cap(d):
                c.fail('displacement_cap', 'displacement_exceeds_cap',
                       f'point {i} {names[j]}: |{d:.10f}| rad > cap '
                       f'{env.M6_0_D_MAX_DISPLACEMENT_RAD} (+ epsilon '
                       f'{env.DISPLACEMENT_EPSILON_RAD})')
            try:
                check_target(names[j], q, sources.limits, sources.soft_margin_rad)
            except UnsafeCommandError as e:
                c.fail('joint_limits', 'joint_limit_violation', f'point {i}: {e}')
    c.passed('displacement_cap')
    c.passed('joint_limits')


def _check_provenance(traj, sources, c):
    prov = traj.get('provenance')
    if not isinstance(prov, dict) or set(prov) != set(PROVENANCE_KEYS) \
            or not isinstance(prov.get('inputs_sha256'), dict):
        c.fail('provenance', 'provenance_missing',
               f'provenance must have exactly {list(PROVENANCE_KEYS)}')
    else:
        expected = {'pose': env.CROUCH_POSE, 'pose_source': POSE_SOURCE,
                    'neutral_source': NEUTRAL_SOURCE, 'joint_order_source': JOINT_ORDER_SOURCE,
                    'm4_config_version': sources.m4_config_version}
        wrong = {k: prov.get(k) for k, v in expected.items() if prov.get(k) != v}
        if wrong:
            c.fail('provenance', 'provenance_mismatch', f'unexpected provenance {wrong}')
        recorded, current = prov['inputs_sha256'], sources.inputs_sha256
        if set(recorded) != set(current):
            c.fail('provenance', 'provenance_missing',
                   f'inputs_sha256 names {sorted(recorded)} != {sorted(current)}')
        else:
            stale = sorted(k for k in current if recorded[k] != current[k])
            if stale:
                c.fail('provenance', 'source_stale',
                       f'changed since conversion: {stale}; re-run the offline conversion')
    c.passed('provenance')
    if traj.get('trajectory_id') != compute_trajectory_id(traj):
        c.fail('identity', 'trajectory_id_mismatch',
               f'trajectory_id {traj.get("trajectory_id")!r} != content hash')
    c.passed('identity')


def preflight(traj, sources):
    """Check one trajectory against the M6.0 envelope. Returns a PreflightReport (never raises).

    Structural failures skip the numeric checks that depend on them (reported as 'skipped').
    """
    c = _Collector(CHECKS)
    tid = traj.get('trajectory_id') if isinstance(traj, dict) else None
    summary = {}
    if sources.errors:
        for code, msg in sources.errors:
            c.fail('sources', code, msg)
        return PreflightReport(False, tid, tuple(c.failures), c.checks, summary)
    c.passed('sources')

    _, points = _check_structure(traj, c, sources)
    if points and not c.failed('points_structure'):
        _check_numeric(points, c)
        if not c.failed('numeric'):
            _check_timing(points, sources, c)
            if not c.failed('joints'):
                _check_content(points, sources, c)
            neutral = sources.poses[env.NEUTRAL_LABEL]
            disp = [(abs(q - q0), j, i) for i, p in enumerate(points)
                    for j, (q, q0) in enumerate(zip(p['positions'], neutral))]
            worst = max(disp)
            summary = {
                'points': len(points),
                'duration_s': float(points[-1]['time_from_start_s']),
                'start_delay_s': float(points[0]['time_from_start_s']),
                'max_displacement_rad': worst[0],
                'max_displacement_joint': (sources.joint_names[worst[1]]
                                           if worst[1] < len(sources.joint_names) else None),
                'labels': [p['label'] for p in points],
            }
    if isinstance(traj, dict):
        _check_provenance(traj, sources, c)
    ok = not c.failures
    return PreflightReport(ok, tid, tuple(c.failures), c.checks, summary)


__all__ = ['SCHEMA', 'REPORT_SCHEMA', 'FAILURE_CODES', 'Sources', 'PreflightReport',
           'TrajectoryBuildError', 'load_sources', 'build_trajectory', 'preflight',
           'compute_trajectory_id', 'canonical_json', 'dumps', 'write_json',
           'load_trajectory_file', 'lead_in_s', 'segment_min_s']
