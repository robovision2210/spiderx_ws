"""colcon test: M6.0-A offline conversion and trajectory preflight (ROS-free).

Proves: the canonical crouch trajectory passes; every invalid condition fails on its own with a
machine-readable code; the 0.1223 rad displacement cap uses a documented 1e-9 rad epsilon; the
0.05 rad tracking tolerance is never used as a command cap; the conversion is deterministic.
"""
import copy
import dataclasses
import inspect
import json
import math
import os
import shutil
import subprocess
import sys

import pytest
import yaml

from spiderx_controller import m4_pose_targets as m4
from spiderx_controller import m4_pose_validation as m4v
from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf
from spiderx_controller.joint_safety import DEFAULT_MARGIN_RAD

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
CANONICAL = ['lf_hip', 'lf_thigh_joint', 'lf_foot_joint', 'rf_hip', 'rf_thigh_joint',
             'rf_foot_joint', 'lr_hip', 'lr_thigh_joint', 'lr_foot_joint', 'rr_hip',
             'rr_thigh_joint', 'rr_foot_joint']
CROUCH_MAX_DISPLACEMENT_RAD = 0.12229413600889982   # docs/M6 plan section 14.2 [RESULT]


@pytest.fixture(scope='module')
def urdf():
    return load_urdf()


@pytest.fixture(scope='module')
def sources(urdf):
    return m6t.load_sources(SRC_CONFIG, urdf)


@pytest.fixture
def traj(sources):
    return m6t.build_trajectory(sources)


@pytest.fixture
def cfg_dir(tmp_path):
    for name in os.listdir(SRC_CONFIG):
        if name.endswith('.yaml'):
            shutil.copy(os.path.join(SRC_CONFIG, name), tmp_path / name)
    return tmp_path


def restamp(t):
    t['trajectory_id'] = m6t.compute_trajectory_id(t)
    return t


def codes(t, sources):
    return m6t.preflight(t, sources).codes


# ------------------------------------------------------------ the valid trajectory
def test_sources_load_without_errors(sources):
    assert sources.errors == ()
    assert list(sources.joint_names) == CANONICAL
    assert sources.soft_margin_rad == DEFAULT_MARGIN_RAD == 0.05
    assert sources.max_joint_velocity_rad_s == 0.5


def test_valid_crouch_trajectory_passes_every_check(traj, sources):
    report = m6t.preflight(traj, sources)
    assert report.ok, report.failures
    assert report.failures == ()
    assert set(report.checks.values()) == {'passed'}
    assert tuple(report.checks) == m6t.CHECKS


def test_trajectory_is_exactly_neutral_crouch_neutral(traj, sources, urdf):
    assert traj['mode'] == 'single'
    assert traj['joint_names'] == CANONICAL
    assert [p['label'] for p in traj['points']] == ['neutral', 'crouch_10mm', 'neutral']
    assert [p['time_from_start_s'] for p in traj['points']] == [3.0, 6.0, 9.0]
    for p in traj['points']:
        assert p['velocities'] == [0.0] * 12
    assert traj['points'][0]['positions'] == [0.0] * 12
    assert traj['points'][2]['positions'] == [0.0] * 12
    _, _, _, plan = m4.load_and_evaluate(config_dir=SRC_CONFIG, urdf_root=urdf)
    assert traj['points'][1]['positions'] == m4.pose_command(plan, 'crouch_10mm')   # exact reuse


def test_crouch_pose_values_match_the_m4_record(traj):
    q = traj['points'][1]['positions']
    expected = [0, 0.0555, 0.1223, 0, -0.0555, -0.1223, 0, 0.0555, -0.1223, 0, -0.0555, -0.1223]
    assert all(abs(a - b) < 1e-4 for a, b in zip(q, expected))     # docs/M4_PLAN.md:189
    assert max(abs(v) for v in q) == pytest.approx(CROUCH_MAX_DISPLACEMENT_RAD, abs=1e-15)


def test_structural_envelope_limits(traj):
    assert len(traj['points']) <= env.MAX_POINTS == 5
    assert traj['points'][-1]['time_from_start_s'] <= env.MAX_DURATION_S == 30.0
    assert traj['points'][0]['time_from_start_s'] > 0


def test_canonical_order_matches_controller_yaml():
    with open(os.path.join(SRC_CONFIG, 'spiderx_ros2_controllers.yaml')) as f:
        ctrl = yaml.safe_load(f)
    assert ctrl['leg_trajectory_controller']['ros__parameters']['joints'] == CANONICAL


# ------------------------------------------------------------ determinism and identity
def test_conversion_is_deterministic(sources, urdf):
    a = m6t.dumps(m6t.build_trajectory(sources))
    b = m6t.dumps(m6t.build_trajectory(m6t.load_sources(SRC_CONFIG, urdf)))
    assert a == b
    t = m6t.build_trajectory(sources)
    assert len(t['trajectory_id']) == 16 and int(t['trajectory_id'], 16) >= 0
    assert m6t.dumps(m6t.preflight(t, sources).to_dict()) == \
        m6t.dumps(m6t.preflight(m6t.build_trajectory(sources), sources).to_dict())


def test_json_round_trip_is_exact_and_still_passes(traj, sources, tmp_path):
    path = tmp_path / 't.json'
    m6t.write_json(str(path), traj)
    back = m6t.load_trajectory_file(str(path))
    assert back == traj
    assert m6t.preflight(back, sources).ok


def test_trajectory_id_changes_with_content(traj):
    other = copy.deepcopy(traj)
    other['points'][1]['time_from_start_s'] = 6.5
    assert m6t.compute_trajectory_id(other) != traj['trajectory_id']


# ------------------------------------------------------------ every condition fails on its own
def _set(path, value):
    def mutate(t):
        node = t
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value
    return mutate


def _swap_joints(t):
    t['joint_names'][0], t['joint_names'][1] = t['joint_names'][1], t['joint_names'][0]


def _extra_points(t):
    t['points'] = t['points'] + [copy.deepcopy(t['points'][-1]) for _ in range(3)]
    for i, p in enumerate(t['points']):
        p['time_from_start_s'] = 3.0 * (i + 1)


MUTATIONS = {
    'joint_count': lambda t: t['joint_names'].pop(),
    'joint_names_not_unique': _set(['joint_names', 1], 'lf_hip'),
    'joint_names_unknown': _set(['joint_names', 0], 'lf_hip_typo'),
    'joint_order_not_canonical': _swap_joints,
    'positions_incomplete': lambda t: t['points'][1]['positions'].pop(),
    'velocities_incomplete': lambda t: t['points'][1]['velocities'].pop(),
    'non_finite_value': _set(['points', 1, 'positions', 4], float('nan')),
    'non_numeric_value': _set(['points', 1, 'positions', 4], '0.05'),
    'time_not_strictly_increasing': _set(['points', 2, 'time_from_start_s'], 6.0),
    'too_many_points': _extra_points,
    'duration_exceeds_max': _set(['points', 2, 'time_from_start_s'], 30.5),
    'start_delay_missing': _set(['points', 0, 'time_from_start_s'], 0.0),
    'start_delay_too_short': _set(['points', 0, 'time_from_start_s'], 1.0),
    'segment_too_fast': _set(['points', 1, 'time_from_start_s'], 4.0),
    'nonzero_velocity': _set(['points', 2, 'velocities', 0], 0.1),
    'mode_not_single': _set(['mode'], 'cyclic'),
    'schema_invalid': _set(['repeat'], 2),
    'empty_trajectory': _set(['points'], []),
    'waypoint_sequence_not_approved': _set(['points', 1, 'label'], 'neutral'),
    'source_pose_mismatch': _set(['points', 1, 'positions', 1], 0.06),
    'displacement_exceeds_cap': _set(['points', 1, 'positions', 2], 0.1224),
    'provenance_missing': lambda t: t.pop('provenance'),
    'provenance_mismatch': _set(['provenance', 'pose'], 'lift_lf_15mm'),
    'source_stale': _set(['provenance', 'inputs_sha256', 'spiderx_poses.yaml'], '0' * 64),
}


@pytest.mark.parametrize('code', sorted(MUTATIONS))
def test_each_invalid_condition_is_refused_with_its_code(code, traj, sources):
    t = copy.deepcopy(traj)
    MUTATIONS[code](t)
    restamp(t)
    report = m6t.preflight(t, sources)
    assert not report.ok
    assert code in report.codes, report.failures
    assert 'trajectory_id_mismatch' not in report.codes


INDEPENDENT = ('joint_order_not_canonical', 'positions_incomplete', 'velocities_incomplete',
               'non_finite_value', 'non_numeric_value', 'time_not_strictly_increasing',
               'duration_exceeds_max', 'start_delay_missing', 'start_delay_too_short',
               'segment_too_fast', 'nonzero_velocity', 'mode_not_single', 'schema_invalid',
               'empty_trajectory', 'provenance_missing', 'provenance_mismatch', 'source_stale')


@pytest.mark.parametrize('code', INDEPENDENT)
def test_single_mutation_yields_only_its_own_code(code, traj, sources):
    t = copy.deepcopy(traj)
    MUTATIONS[code](t)
    restamp(t)
    assert codes(t, sources) == [code]


def test_every_documented_code_is_exercised():
    covered = set(MUTATIONS) | {'trajectory_id_mismatch', 'joint_limit_violation',
                                'source_unavailable', 'neutral_source_missing',
                                'neutral_source_inconsistent', 'canonical_order_inconsistent',
                                'soft_margin_inconsistent', 'source_pose_unavailable'}
    assert covered == set(m6t.FAILURE_CODES)


def test_tampered_trajectory_id_is_refused(traj, sources):
    t = copy.deepcopy(traj)
    t['points'][2]['time_from_start_s'] = 9.5     # valid change, but id not restamped
    assert codes(t, sources) == ['trajectory_id_mismatch']


def test_missing_trajectory_id_is_refused(traj, sources):
    t = copy.deepcopy(traj)
    del t['trajectory_id']
    assert codes(t, sources) == ['trajectory_id_mismatch']


@pytest.mark.parametrize('bad', [float('inf'), float('-inf')])
def test_infinite_values_and_times_are_refused(bad, traj, sources):
    for path in (['points', 1, 'positions', 0], ['points', 1, 'time_from_start_s'],
                 ['points', 1, 'velocities', 3]):
        t = copy.deepcopy(traj)
        _set(path, bad)(t)
        assert 'non_finite_value' in codes(restamp(t), sources)


@pytest.mark.parametrize('bad', [None, True, [0.0]])
def test_non_numeric_values_are_refused(bad, traj, sources):
    t = copy.deepcopy(traj)
    t['points'][1]['positions'][3] = bad
    assert 'non_numeric_value' in codes(restamp(t), sources)


@pytest.mark.parametrize('bad', [None, 'x', 3, []])
def test_non_mapping_or_malformed_trajectory_never_crashes(bad, sources):
    report = m6t.preflight(bad, sources)
    assert not report.ok and 'schema_invalid' in report.codes


def test_missing_point_field_is_refused(traj, sources):
    t = copy.deepcopy(traj)
    del t['points'][1]['velocities']
    assert 'schema_invalid' in codes(restamp(t), sources)


def test_negative_start_time_is_refused(traj, sources):
    t = copy.deepcopy(traj)
    t['points'][0]['time_from_start_s'] = -1.0
    assert 'start_delay_missing' in codes(restamp(t), sources)


def test_repeated_cycle_is_refused(traj, sources):
    """neutral -> crouch -> neutral -> crouch -> neutral: 5 points, a repeat, never approved."""
    t = copy.deepcopy(traj)
    t['points'] = t['points'] + copy.deepcopy(t['points'][1:])
    for i, p in enumerate(t['points']):
        p['time_from_start_s'] = 3.0 * (i + 1)
    assert codes(restamp(t), sources) == ['waypoint_sequence_not_approved']


# ------------------------------------------------------------ cap, epsilon and separation
def _sources_with_crouch_value(sources, value, joint=2):
    q = list(sources.poses['crouch_10mm'])
    q[joint] = value
    return dataclasses.replace(sources, poses=dict(sources.poses, crouch_10mm=tuple(q)))


def test_cap_function_epsilon_handling():
    cap, eps = env.M6_0_D_MAX_DISPLACEMENT_RAD, env.DISPLACEMENT_EPSILON_RAD
    assert (cap, eps) == (0.1223, 1e-9)
    assert env.displacement_within_cap(cap)
    assert env.displacement_within_cap(-cap)
    assert env.displacement_within_cap(cap + 0.5 * eps)
    assert not env.displacement_within_cap(cap + 2 * eps)
    assert not env.displacement_within_cap(-(cap + 2 * eps))
    assert not env.displacement_within_cap(0.1224)
    assert env.displacement_within_cap(CROUCH_MAX_DISPLACEMENT_RAD)


@pytest.mark.parametrize('value', [0.1223, 0.1223 + 0.5e-9, -0.1223])
def test_trajectory_at_the_cap_is_accepted_only_within_epsilon(value, sources):
    s = _sources_with_crouch_value(sources, value)
    report = m6t.preflight(m6t.build_trajectory(s), s)
    assert report.ok, report.failures


@pytest.mark.parametrize('value', [0.1223 + 2e-9, 0.1224, 0.2, -0.13])
def test_trajectory_beyond_the_cap_is_refused(value, sources):
    s = _sources_with_crouch_value(sources, value)
    report = m6t.preflight(m6t.build_trajectory(s), s)
    assert report.codes == ['displacement_exceeds_cap']


def test_tracking_tolerance_is_not_a_command_cap(traj, sources, monkeypatch):
    # The approved crouch moves 0.0555 / 0.1223 rad, well beyond the 0.05 rad tracking tolerance.
    assert CROUCH_MAX_DISPLACEMENT_RAD > env.TRACKING_TOLERANCE_RAD
    assert m6t.preflight(traj, sources).ok
    s = _sources_with_crouch_value(sources, 0.09)
    assert m6t.preflight(m6t.build_trajectory(s), s).ok
    monkeypatch.setattr(env, 'TRACKING_TOLERANCE_RAD', 1e-6)
    assert m6t.preflight(traj, sources).ok
    assert 'TRACKING_TOLERANCE' not in inspect.getsource(m6t)


def test_soft_margin_is_the_limit_check_only(traj, sources):
    # Tighten lf_foot_joint so its soft upper limit (upper - 0.05) is below the 0.1222 command:
    # only the joint-limit check fires; the displacement cap is untouched.
    limits = dict(sources.limits, lf_foot_joint=(-0.698132, 0.15))
    s = dataclasses.replace(sources, limits=limits)
    assert m6t.preflight(traj, s).codes == ['joint_limit_violation']
    s2 = dataclasses.replace(sources, limits=limits, soft_margin_rad=0.0)
    assert m6t.preflight(traj, s2).ok          # same limit without the margin: inside


def test_three_quantities_are_distinct_constants():
    assert env.M6_0_D_MAX_DISPLACEMENT_RAD == 0.1223
    assert env.TRACKING_TOLERANCE_RAD == 0.05
    assert DEFAULT_MARGIN_RAD == 0.05
    assert not hasattr(env, 'SOFT_LIMIT_MARGIN_RAD')     # the margin is loaded, never redefined


def test_envelope_values_are_pinned_to_their_sources():
    with open(os.path.join(SRC_CONFIG, 'm3_kinematics_targets.yaml')) as f:
        motion = yaml.safe_load(f)['motion']
    assert env.MIN_SEGMENT_S == motion['min_duration_s'] == m4v.MIN_DURATION_S
    assert env.SPEED_FACTOR == motion['speed_factor'] == m4v.SPEED_FACTOR
    assert env.SETTLE_S == motion['settle_s'] == m4v.SETTLE_S
    assert env.START_POSE_TOLERANCE_RAD == m4v.START_POSE_TOL_RAD
    with open(os.path.join(SRC_CONFIG, 'm2_simulation_postures.yaml')) as f:
        m2 = yaml.safe_load(f)
    found = []

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == 'max_joint_error_rad':
                    found.append(v)
                walk(v)
    walk(m2)
    assert found and all(v == env.TRACKING_TOLERANCE_RAD for v in found)


# ------------------------------------------------------------ source failures
def _write_yaml(path, data):
    path.write_text(yaml.safe_dump(data, sort_keys=False))


def test_missing_neutral_source_is_refused(cfg_dir, urdf):
    os.remove(cfg_dir / 'spiderx_poses.yaml')
    s = m6t.load_sources(str(cfg_dir), urdf)
    assert [c for c, _ in s.errors] == ['neutral_source_missing']
    with pytest.raises(m6t.TrajectoryBuildError):
        m6t.build_trajectory(s)
    assert m6t.preflight({}, s).codes == ['neutral_source_missing']


def test_incomplete_neutral_source_is_refused(cfg_dir, urdf):
    data = yaml.safe_load((cfg_dir / 'spiderx_poses.yaml').read_text())
    del data['poses']['cad_neutral']['joints']['rr_hip']
    _write_yaml(cfg_dir / 'spiderx_poses.yaml', data)
    assert [c for c, _ in m6t.load_sources(str(cfg_dir), urdf).errors] == \
        ['neutral_source_missing']


def test_inconsistent_neutral_source_is_refused(cfg_dir, urdf):
    data = yaml.safe_load((cfg_dir / 'spiderx_poses.yaml').read_text())
    data['poses']['cad_neutral']['joints']['lf_hip'] = 0.01
    _write_yaml(cfg_dir / 'spiderx_poses.yaml', data)
    assert [c for c, _ in m6t.load_sources(str(cfg_dir), urdf).errors] == \
        ['neutral_source_inconsistent']


def test_controller_order_change_is_refused(cfg_dir, urdf):
    path = cfg_dir / 'spiderx_ros2_controllers.yaml'
    data = yaml.safe_load(path.read_text())
    j = data['leg_trajectory_controller']['ros__parameters']['joints']
    j[0], j[3] = j[3], j[0]
    _write_yaml(path, data)
    assert 'canonical_order_inconsistent' in \
        [c for c, _ in m6t.load_sources(str(cfg_dir), urdf).errors]


def test_unsafe_crouch_source_is_refused(cfg_dir, urdf):
    path = cfg_dir / 'm4_pose_targets.yaml'
    data = yaml.safe_load(path.read_text())
    data['safety']['max_joint_change_rad'] = 0.1           # M4 refuses its own crouch now
    _write_yaml(path, data)
    assert 'source_pose_unavailable' in \
        [c for c, _ in m6t.load_sources(str(cfg_dir), urdf).errors]


def test_unreadable_config_is_refused(cfg_dir, urdf):
    (cfg_dir / 'spiderx_legs.yaml').write_text('legs: [unclosed')
    assert [c for c, _ in m6t.load_sources(str(cfg_dir), urdf).errors] == ['source_unavailable']


def test_changed_margin_is_refused(cfg_dir, urdf):
    text = (cfg_dir / 'spiderx_legs.yaml').read_text()
    (cfg_dir / 'spiderx_legs.yaml').write_text(
        text.replace('soft_limit_margin_rad: 0.05', 'soft_limit_margin_rad: 0.04'))
    errs = [c for c, _ in m6t.load_sources(str(cfg_dir), urdf).errors]
    assert 'soft_margin_inconsistent' in errs


def test_source_change_after_conversion_is_stale(cfg_dir, urdf):
    s1 = m6t.load_sources(str(cfg_dir), urdf)
    t = m6t.build_trajectory(s1)
    assert m6t.preflight(t, s1).ok
    with open(cfg_dir / 'spiderx_poses.yaml', 'a') as f:
        f.write('# edited after conversion\n')
    s2 = m6t.load_sources(str(cfg_dir), urdf)
    report = m6t.preflight(t, s2)
    assert report.codes == ['source_stale']
    assert 'spiderx_poses.yaml' in report.failures[0][1]


# ------------------------------------------------------------ report and isolation
def test_report_is_machine_readable(traj, sources):
    t = copy.deepcopy(traj)
    t['points'][1]['positions'][2] = 0.5
    d = m6t.preflight(restamp(t), sources).to_dict()
    json.dumps(d, allow_nan=False)
    assert d['verdict'] == 'REFUSED' and d['ok'] is False
    assert {'displacement_exceeds_cap', 'joint_limit_violation'} <= set(d['failure_codes'])
    assert all(f['code'] in m6t.FAILURE_CODES for f in d['failures'])
    assert d['envelope']['max_commanded_displacement_rad'] == 0.1223
    assert d['envelope']['tracking_tolerance_rad'] == 0.05


def test_envelope_is_never_read_from_the_trajectory(traj, sources):
    t = copy.deepcopy(traj)
    t['envelope'] = {'max_commanded_displacement_rad': 9.9}
    t['points'][1]['positions'][2] = 0.5
    assert {'schema_invalid', 'displacement_exceeds_cap'} <= set(codes(restamp(t), sources))


def test_offline_modules_import_no_ros_client():
    code = ('import sys, spiderx_controller.m6_offline_preflight, spiderx_controller.m6_trajectory;'
            'bad=[m for m in sys.modules if m.split(".")[0] in ("rclpy","control_msgs",'
            '"trajectory_msgs","action_msgs","sensor_msgs")]; print(bad);'
            'sys.exit(1 if bad else 0)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr


def test_report_never_contains_nan(traj, sources):
    t = copy.deepcopy(traj)
    t['points'][1]['positions'][0] = float('nan')
    d = m6t.preflight(restamp(t), sources).to_dict()
    json.dumps(d, allow_nan=False)
    assert not any(isinstance(v, float) and math.isnan(v) for v in d['summary'].values())
