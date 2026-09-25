"""colcon test: deterministic M2 metrics and pass/fail evaluation (no ROS graph)."""
import math
from types import SimpleNamespace as NS

import pytest

from spiderx_controller import posture_metrics as pm

JOINTS = ['lf_hip', 'lf_thigh_joint', 'lf_foot_joint', 'rf_hip', 'rf_thigh_joint',
          'rf_foot_joint', 'lr_hip', 'lr_thigh_joint', 'lr_foot_joint', 'rr_hip',
          'rr_thigh_joint', 'rr_foot_joint']
POSTURE = {
    'name': 'cad_neutral_simulation_hold', 'config_version': 1, 'date': '2026-09-25',
    'targets': {j: 0.0 for j in JOINTS}, 'margin': 0.05, 'command_duration_s': 6.0,
    'hold_duration_s': 10.0, 'max_joint_velocity_rad_s': 0.5,
    'thresholds': {'max_joint_error_rad': 0.05, 'rms_joint_error_rad': 0.02,
                   'min_body_height_m': 0.045, 'max_body_height_range_m': 0.005,
                   'max_abs_roll_rad': 0.10, 'max_abs_pitch_rad': 0.10,
                   'min_pose_samples': 20, 'min_joint_state_samples': 20},
}
LEVEL = (0.0, 0.0, 0.0545, 0.0, 0.0, 0.0, 1.0)


def quat_x(angle):
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def quat_y(angle):
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


def good_run(**over):
    run = {
        'controller_checks': [(t, ph, {'joint_state_broadcaster': 'active',
                                       'leg_trajectory_controller': 'active'})
                              for t, ph in ((1.0, 'start'), (12.0, 'hold'), (20.0, 'end'))],
        'joint_state_publishers': [1, 1],
        'action': {'server_available': True, 'accepted': True, 'error_code': 0},
        'peak_joint_speed_rad_s': 0.03,
        'hold': {'start_s': 8.0, 'end_s': 18.0},
        'joint_samples': [{j: 0.001 for j in JOINTS} for _ in range(50)],
        'pose_samples': [(8.0 + 0.2 * i, LEVEL) for i in range(50)],
        'observation_errors': [],
    }
    run.update(over)
    return run


def test_quat_to_rpy_known_rotations():
    assert pm.quat_to_rpy(0, 0, 0, 1) == (0.0, 0.0, 0.0)
    assert pm.quat_to_rpy(*quat_x(0.1))[0] == pytest.approx(0.1)
    assert pm.quat_to_rpy(*quat_y(-0.2))[1] == pytest.approx(-0.2)
    assert pm.tilt_from_quat(*quat_x(0.3)) == pytest.approx(0.3)
    with pytest.raises(pm.ObservationError):
        pm.quat_to_rpy(0, 0, 0, 0)


def test_joint_error_stats_deterministic():
    targets = {'a': 0.0, 'b': 0.1}
    samples = [{'a': 0.03, 'b': 0.1}, {'a': 0.0, 'b': 0.14}]
    s = pm.joint_error_stats(samples, targets)
    assert s['per_joint_max_abs_error_rad'] == pytest.approx({'a': 0.03, 'b': 0.04})
    assert s['max_abs_error_rad'] == pytest.approx(0.04)
    assert s['rms_error_rad'] == pytest.approx(math.sqrt((0.03 ** 2 + 0.04 ** 2) / 4))


def test_body_stats_frame_and_values():
    samples = [(0.0, LEVEL), (1.0, (0.01, 0.0, 0.0535) + quat_x(0.02))]
    b = pm.body_stats(samples)
    assert b['height_m']['min'] == pytest.approx(0.0535)
    assert b['height_m']['range'] == pytest.approx(0.001)
    assert b['roll_rad']['max_abs'] == pytest.approx(0.02)
    assert b['xy_drift_m'] == pytest.approx(0.01)
    assert 'world frame' in b['frame'] and 'base_link' in b['frame']


def test_extract_model_pose():
    def tf(name, z):
        return NS(child_frame_id=name, transform=NS(
            translation=NS(x=0.0, y=0.0, z=z), rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))
    assert pm.extract_model_pose([tf('ground_plane', 0), tf('spiderx', 0.05)])[2] == 0.05
    with pytest.raises(pm.ObservationError, match='not found'):
        pm.extract_model_pose([tf('ground_plane', 0)])


def test_joint_positions_validation():
    assert pm.joint_positions(['a', 'b'], [0.1, 0.2], ['a']) == {'a': 0.1}
    with pytest.raises(pm.ObservationError, match='malformed'):
        pm.joint_positions(['a', 'b'], [0.1], ['a'])
    with pytest.raises(pm.ObservationError, match='missing'):
        pm.joint_positions(['a'], [0.1], ['a', 'b'])
    with pytest.raises(pm.ObservationError, match='non-finite'):
        pm.joint_positions(['a'], [float('nan')], ['a'])


def test_good_run_is_verified():
    passed, failures, metrics = pm.evaluate(POSTURE, good_run())
    assert passed and failures == []
    assert metrics['hold_duration_s'] == pytest.approx(10.0)
    report = pm.build_report(POSTURE, good_run(), passed, failures, metrics)
    assert report['outcome'] == 'Simulation posture hold verified.'
    assert report['simulation_only'] is True
    assert report['foot_contact']['status'] == 'unavailable'
    assert len(report['limitations']) == 7


@pytest.mark.parametrize('over,reason', [
    ({'pose_samples': [(8.0 + 0.2 * i, (0, 0, 0.012, 0, 0, 0, 1)) for i in range(50)]},
     'body height fell'),
    ({'pose_samples': [(8.0 + 0.2 * i, (0, 0, 0.0545) + quat_x(0.2)) for i in range(50)]},
     '|roll|'),
    ({'pose_samples': [(8.0 + 0.2 * i, (0, 0, 0.0545) + quat_y(-0.15)) for i in range(50)]},
     '|pitch|'),
    ({'pose_samples': [(8.0 + 0.2 * i, (0, 0, 0.0545 + 0.001 * (i % 10), 0, 0, 0, 1))
                       for i in range(50)]}, 'body height varied'),
    ({'joint_samples': [{**{j: 0.0 for j in JOINTS}, 'rr_foot_joint': 0.08}] * 50},
     'max joint error'),
    ({'joint_samples': [{j: 0.03 for j in JOINTS}] * 50}, 'RMS joint error'),
    ({'controller_checks': [(12.0, 'hold', {'joint_state_broadcaster': 'active',
                                            'leg_trajectory_controller': 'inactive'})]},
     'leg_trajectory_controller'),
    ({'controller_checks': []}, 'never read'),
    ({'joint_state_publishers': [2, 2]}, 'publisher counts'),
    ({'action': {'server_available': False, 'accepted': False, 'error_code': None}},
     'unavailable'),
    ({'action': {'server_available': True, 'accepted': True, 'error_code': None}}, 'timed out'),
    ({'action': {'server_available': True, 'accepted': True, 'error_code': -5}}, 'error_code -5'),
    ({'hold': {'start_s': 8.0, 'end_s': 12.0}}, 'hold lasted'),
    ({'pose_samples': [(8.0, LEVEL)] * 5}, 'body-pose samples'),
    ({'joint_samples': []}, 'joint-state samples'),
    ({'peak_joint_speed_rad_s': 0.9}, 'peak joint speed'),
    ({'observation_errors': ['no /clock received']}, 'no /clock'),
])
def test_failed_conditions_are_not_verified(over, reason):
    """Negative tests: each unsafe or failed measured condition returns failure."""
    run = good_run(**over)
    passed, failures, metrics = pm.evaluate(POSTURE, run)
    assert not passed
    assert any(reason in f for f in failures), failures
    report = pm.build_report(POSTURE, run, passed, failures, metrics)
    assert report['outcome'] == 'Simulation posture hold not verified.'
