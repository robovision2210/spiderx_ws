"""M6.0-D goal contract and fingerprint: binds the ONE approved goal to a deterministic hash.

The approved goal is the preflighted neutral -> crouch_10mm -> neutral trajectory
(m6_trajectory.build_trajectory) as a FollowJointTrajectory goal with:
  - header stamp 0 (D5), 12 canonical joints, 3 points at 3/6/9 s, point velocities 0.0;
  - path_tolerance position 0.05 rad per joint (velocity/acceleration unspecified);
  - goal_tolerance position 0.05 rad AND an explicit velocity of 0.05 rad/s per joint (D16);
  - goal_time_tolerance 1.0 s; no multi-DOF or component tolerances.

The goal is built by composition: m6_action_client.build_goal (unchanged; it re-runs the M6.0
preflight) and then the D16 goal velocity tolerance is set on the returned message. The M6.0-C
mock goal is unaffected.

A goal SPEC is a plain dict of every field the controller would act on, plus the trajectory ID,
source hashes, mode and goal count. Its fingerprint is the SHA-256 of the canonical JSON. The live
adapter refuses any goal whose recomputed fingerprint differs from the approved one, so a
return-to-neutral or any other second goal cannot be sent through it.

ROS message types are imported only inside build_live_goal(); spec_from_goal() reads attributes.
"""

import hashlib
import json
import math

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_trajectory as m6t

SPEC_SCHEMA = 'spiderx.m6d.goal_spec/1'


class FingerprintError(ValueError):
    """A goal or trajectory does not match the one approved M6.0-D goal; nothing may be sent."""

    def __init__(self, code, message):
        super().__init__(f'{code}: {message}')
        self.code = code


def _ns(sec, nanosec):
    return int(sec) * 1_000_000_000 + int(nanosec)


def _tol(names, velocity):
    return [{'name': n, 'position': lc.GOAL_POSITION_TOLERANCE_RAD, 'velocity': velocity,
             'acceleration': 0.0} for n in names]


def binding(trajectory):
    """The non-message identity fields bound into the fingerprint."""
    prov = trajectory.get('provenance') or {}
    return {
        'trajectory_id': trajectory.get('trajectory_id'),
        'inputs_sha256': dict(sorted((prov.get('inputs_sha256') or {}).items())),
        'mode': trajectory.get('mode'),
        'goal_count': lc.APPROVED_GOAL_COUNT,
    }


def _check_approved_content(trajectory, sources):
    report = m6t.preflight(trajectory, sources)
    if not report.ok:
        raise FingerprintError('preflight_refused', ', '.join(report.codes))
    pts = trajectory['points']
    if [p['label'] for p in pts] != list(lc.APPROVED_LABELS):
        raise FingerprintError('not_approved_content', 'waypoints differ')
    if tuple(p['time_from_start_s'] for p in pts) != lc.APPROVED_POINT_TIMES_S:
        raise FingerprintError('not_approved_content',
                               f'times {[p["time_from_start_s"] for p in pts]}')
    if trajectory.get('mode') != m6t.MODE:
        raise FingerprintError('not_approved_content', 'mode is not single')
    return report


def approved_spec(trajectory, sources):
    """The spec of the one approved goal, from a preflighted trajectory. Raises FingerprintError."""
    _check_approved_content(trajectory, sources)
    names = list(trajectory['joint_names'])
    points = []
    for p in trajectory['points']:
        sec, nsec = ac.seconds_to_duration_fields(p['time_from_start_s'])
        points.append({'time_ns': _ns(sec, nsec),
                       'positions': [float(v) for v in p['positions']],
                       'velocities': [float(v) for v in p['velocities']],
                       'accelerations': [], 'effort': []})
    sec, nsec = ac.seconds_to_duration_fields(lc.GOAL_TIME_TOLERANCE_S)
    spec = {
        'schema': SPEC_SCHEMA,
        'binding': binding(trajectory),
        'header_stamp_ns': 0,
        'joint_names': names,
        'points': points,
        'path_tolerance': _tol(names, 0.0),
        'goal_tolerance': _tol(names, lc.GOAL_VELOCITY_TOLERANCE_RAD_S),
        'goal_time_tolerance_ns': _ns(sec, nsec),
        'component_path_tolerance': 0,
        'component_goal_tolerance': 0,
        'multi_dof_joint_names': [],
        'multi_dof_points': 0,
    }
    validate_spec(spec)
    return spec


def spec_from_goal(goal, bind):
    """The spec of an arbitrary FollowJointTrajectory goal (attribute access only)."""
    traj = goal.trajectory
    return {
        'schema': SPEC_SCHEMA,
        'binding': dict(bind),
        'header_stamp_ns': _ns(traj.header.stamp.sec, traj.header.stamp.nanosec),
        'joint_names': list(traj.joint_names),
        'points': [{'time_ns': _ns(p.time_from_start.sec, p.time_from_start.nanosec),
                    'positions': [float(v) for v in p.positions],
                    'velocities': [float(v) for v in p.velocities],
                    'accelerations': [float(v) for v in p.accelerations],
                    'effort': [float(v) for v in p.effort]} for p in traj.points],
        'path_tolerance': [{'name': t.name, 'position': float(t.position),
                            'velocity': float(t.velocity),
                            'acceleration': float(t.acceleration)}
                           for t in goal.path_tolerance],
        'goal_tolerance': [{'name': t.name, 'position': float(t.position),
                            'velocity': float(t.velocity),
                            'acceleration': float(t.acceleration)}
                           for t in goal.goal_tolerance],
        'goal_time_tolerance_ns': _ns(goal.goal_time_tolerance.sec,
                                      goal.goal_time_tolerance.nanosec),
        'component_path_tolerance': len(goal.component_path_tolerance),
        'component_goal_tolerance': len(goal.component_goal_tolerance),
        'multi_dof_joint_names': list(goal.multi_dof_trajectory.joint_names),
        'multi_dof_points': len(goal.multi_dof_trajectory.points),
    }


def _walk_numbers(node):
    if isinstance(node, dict):
        for v in node.values():
            yield from _walk_numbers(v)
    elif isinstance(node, (list, tuple)):
        for v in node:
            yield from _walk_numbers(v)
    elif isinstance(node, float):
        yield node


def validate_spec(spec):
    """Refuse non-finite values anywhere in a spec."""
    for v in _walk_numbers(spec):
        if not math.isfinite(v):
            raise FingerprintError('non_finite_value', f'{v!r} in goal spec')
    return spec


def canonical(spec):
    validate_spec(spec)
    return json.dumps(spec, sort_keys=True, separators=(',', ':'), allow_nan=False)


def fingerprint(spec):
    """SHA-256 (hex) of the canonical spec. Raises FingerprintError for non-finite values."""
    return hashlib.sha256(canonical(spec).encode()).hexdigest()


def verify_goal(goal, bind, expected_fingerprint):
    """Raise FingerprintError unless the goal (with its binding) is the approved goal."""
    got = fingerprint(spec_from_goal(goal, bind))
    if got != expected_fingerprint:
        raise FingerprintError('goal_fingerprint_mismatch',
                               f'goal fingerprint {got[:16]} != approved '
                               f'{expected_fingerprint[:16]}')
    return got


def build_live_goal(trajectory, sources):
    """(goal, report, spec, fingerprint) for the one approved M6.0-D goal. Raises on any mismatch.

    Composition: the unchanged M6.0 build_goal (which re-runs the preflight) followed by the
    explicit D16 goal velocity tolerance on goal_tolerance only.
    """
    expected = approved_spec(trajectory, sources)
    goal, report = ac.build_goal(trajectory, sources)
    for t in goal.goal_tolerance:
        t.velocity = lc.GOAL_VELOCITY_TOLERANCE_RAD_S
    fp = fingerprint(expected)
    verify_goal(goal, binding(trajectory), fp)
    return goal, report, expected, fp


__all__ = ['SPEC_SCHEMA', 'FingerprintError', 'binding', 'approved_spec', 'spec_from_goal',
           'validate_spec', 'canonical', 'fingerprint', 'verify_goal', 'build_live_goal']
