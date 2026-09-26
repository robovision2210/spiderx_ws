"""colcon test: M4 four-leg static pose targets - strict loader and atomic evaluator (ROS-free).

Tolerance: FK of every returned command must reproduce each leg's target within
leg_kinematics.IK_POSITION_TOL_M = 1e-6 m (the config's ik_position_tol_m, checked equal).
"""
import copy
import math
import os
import shutil

import pytest
import yaml

from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m4_pose_targets as m4
from spiderx_controller.config_check import load_urdf
from spiderx_controller.joint_safety import load_limits

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
POSES = ('neutral_stance', 'crouch_10mm', 'lift_lf_15mm')


@pytest.fixture(scope='module')
def urdf():
    return load_urdf()


@pytest.fixture(scope='module')
def shipped(urdf):
    return m4.load_and_evaluate(os.path.join(SRC_CONFIG, m4.CONFIG_FILE), SRC_CONFIG, urdf)


@pytest.fixture
def cfg_dir(tmp_path):
    for name in os.listdir(SRC_CONFIG):
        if name.endswith('.yaml'):
            shutil.copy(os.path.join(SRC_CONFIG, name), tmp_path / name)
    return tmp_path


def _raw():
    with open(os.path.join(SRC_CONFIG, m4.CONFIG_FILE)) as f:
        return yaml.safe_load(f)


def _load(cfg_dir, urdf, data):
    path = cfg_dir / m4.CONFIG_FILE
    path.write_text(data if isinstance(data, str) else yaml.safe_dump(data))
    return m4.load_and_evaluate(str(path), str(cfg_dir), urdf)


# ------------------------------------------------------------ shipped poses
def test_shipped_config_has_exactly_the_three_poses(shipped):
    cfg, _, errors, plan = shipped
    assert errors == []
    assert tuple(cfg['poses']) == POSES
    assert set(plan['negative']) == {'rr_unreachable_300mm_below', 'lr_thigh_outside_limits'}


@pytest.mark.parametrize('name', POSES)
def test_pose_yields_12_finite_joint_safe_angles_reproducing_targets(shipped, name):
    _, geoms, _, plan = shipped
    rec = plan['poses'][name]
    q12 = m4.pose_command(plan, name)
    assert rec['ok'] and len(q12) == 12 and all(math.isfinite(v) for v in q12)
    assert rec['joint_names'] == lk.all_joint_names(geoms)
    limits, _ = load_limits(SRC_CONFIG)
    for joint, v in zip(rec['joint_names'], q12):
        lo, hi = limits[joint]
        assert lo + 0.05 <= v <= hi - 0.05
    fk = lk.forward_all(geoms, q12)
    for leg in lk.ALL_LEGS:
        assert math.dist(fk[leg]['tip_position'], rec['targets_m'][leg]) <= lk.IK_POSITION_TOL_M


def test_neutral_stance_is_cad_neutral(shipped):
    assert m4.pose_command(shipped[3], 'neutral_stance') == pytest.approx([0.0] * 12, abs=1e-9)


def test_crouch_moves_every_target_up_10mm(shipped):
    _, geoms, _, plan = shipped
    for leg in lk.ALL_LEGS:
        d = [a - b for a, b in zip(plan['poses']['crouch_10mm']['targets_m'][leg], geoms[leg].tip0)]
        assert d == pytest.approx([0.0, 0.0, 0.010], abs=1e-12)
    rec = plan['poses']['crouch_10mm']
    assert rec['support_plane_z_m'] == pytest.approx(geoms['front_left'].tip0[2] + 0.010)
    assert 'NOT a measured' in rec['geometric_expected_body_height_m']['note']


def test_lift_changes_only_front_left(shipped):
    _, geoms, _, plan = shipped
    rec = plan['poses']['lift_lf_15mm']
    for leg in lk.ALL_LEGS:
        d = [a - b for a, b in zip(rec['targets_m'][leg], geoms[leg].tip0)]
        assert d == pytest.approx([0.0, 0.0, 0.015 if leg == 'front_left' else 0.0], abs=1e-12)
    q12 = m4.pose_command(plan, 'lift_lf_15mm')
    assert any(abs(v) > 0.01 for v in q12[:3]) and q12[3:] == pytest.approx([0.0] * 9, abs=1e-9)
    assert rec['support_legs'] == ['front_right', 'rear_left', 'rear_right']   # 3-foot support


def test_crouch_sign_pattern_in_the_command(shipped):
    q = m4.pose_command(shipped[3], 'crouch_10mm')
    signs = [(math.copysign(1, q[i + 1]), math.copysign(1, q[i + 2])) for i in (0, 3, 6, 9)]
    assert signs == [(1, 1), (-1, -1), (1, -1), (-1, -1)]      # lf, rf, lr, rr


def test_negative_tests_name_leg_and_reason_with_no_command(shipped):
    plan = shipped[3]
    rr = plan['negative']['rr_unreachable_300mm_below']
    assert rr['failing_legs'] == {'rear_right': 'unreachable_knee'} and rr['command_rad'] is None
    lr = plan['negative']['lr_thigh_outside_limits']
    assert lr['failing_legs'] == {'rear_left': 'joint_limits'} and lr['command_rad'] is None
    assert any('rear_left: joint_limits' in f for f in lr['failures'])


# ------------------------------------------------------------ atomic refusal of unsafe poses
def test_unreachable_leg_in_a_safe_pose_refuses_the_whole_pose(cfg_dir, urdf):
    data = _raw()
    data['safety']['max_offset_m'] = 0.5
    data['poses']['crouch_10mm']['offsets_m']['rear_right'] = [0.0, 0.0, -0.30]
    _, _, errors, plan = _load(cfg_dir, urdf, data)
    rec = plan['poses']['crouch_10mm']
    assert not rec['ok'] and rec['command_rad'] is None
    assert rec['failing_legs'] == {'rear_right': 'unreachable_knee'}
    assert any('pose crouch_10mm refused (ik_failed): rear_right: unreachable_knee' in e
               for e in errors)
    with pytest.raises(m4.PoseTargetError, match='not safe to command'):
        m4.pose_command(plan, 'crouch_10mm')
    assert plan['poses']['neutral_stance']['ok']          # other poses unaffected


def test_margin_violation_refused_not_clamped(cfg_dir, urdf):
    """RF knee 0.02 rad inside its URDF limit but inside the 0.05 margin -> refused."""
    data = _raw()
    data['safety']['max_offset_m'] = 0.5
    _, geoms, _, _ = _load(cfg_dir, urdf, _raw())
    g = geoms['front_right']
    lo, _ = g.limits[2]
    tip = lk.forward(g, [0.0, 0.0, lo + 0.02])['tip_position']
    data['poses']['neutral_stance']['offsets_m']['front_right'] = [a - b for a, b in
                                                                  zip(tip, g.tip0)]
    data['poses']['neutral_stance']['support_legs'] = ['front_left', 'rear_left', 'rear_right']
    _, _, errors, plan = _load(cfg_dir, urdf, data)
    rec = plan['poses']['neutral_stance']
    assert not rec['ok'] and rec['command_rad'] is None
    assert rec['failing_legs'] == {'front_right': 'joint_limits'}


def test_non_coplanar_support_and_unlifted_foot_refused(cfg_dir, urdf):
    data = _raw()
    data['poses']['crouch_10mm']['offsets_m']['rear_left'] = [0.0, 0.0, 0.0]   # not coplanar
    data['poses']['lift_lf_15mm']['offsets_m']['front_left'] = [0.0, 0.0, 0.002]  # < 5 mm lift
    _, _, errors, plan = _load(cfg_dir, urdf, data)
    assert not plan['poses']['crouch_10mm']['ok']
    assert any('support feet not coplanar' in f for f in plan['poses']['crouch_10mm']['failures'])
    lift = plan['poses']['lift_lf_15mm']
    assert not lift['ok'] and lift['failing_legs'] == {'front_left': 'not_lifted'}
    assert lift['command_rad'] is None and len(errors) == 2


def test_joint_change_bound_refuses(cfg_dir, urdf):
    data = _raw()
    data['safety']['max_joint_change_rad'] = 0.1          # crouch knees need 0.1223 rad
    _, _, errors, plan = _load(cfg_dir, urdf, data)
    assert not plan['poses']['crouch_10mm']['ok'] and plan['poses']['neutral_stance']['ok']
    assert any('joint change' in f for f in plan['poses']['crouch_10mm']['failures'])


def test_negative_test_that_is_accepted_is_an_error(cfg_dir, urdf):
    data = _raw()
    data['negative_tests']['rr_unreachable_300mm_below']['offsets_m']['rear_right'] = [0, 0, 0]
    _, _, errors, _ = _load(cfg_dir, urdf, data)
    assert any('rr_unreachable_300mm_below: pose was unexpectedly accepted' in e for e in errors)


# ------------------------------------------------------------ invalid configs
@pytest.mark.parametrize('mutate,match', [
    (lambda d: d.pop('simulation_only'), 'simulation_only'),
    (lambda d: d.update(simulation_only=False), 'simulation_only'),
    (lambda d: d.update(simulation_only='true'), 'simulation_only'),
    (lambda d: d.update(frame='world'), 'frame must be'),
    (lambda d: d.update(frame='odom'), 'frame must be'),
    (lambda d: d.update(units={'length': 'mm', 'angle': 'rad'}), 'units'),
    (lambda d: d.update(units={'length': 'm', 'angle': 'deg'}), 'units'),
    (lambda d: d.update(validation_margin_rad=0.0), 'M1 margin'),
    (lambda d: d.update(ik_position_tol_m=1e-3), 'IK_POSITION_TOL_M'),
    (lambda d: d.update(reference={'pose': 'tuned', 'point': 'derived_foot_tip'}), 'reference'),
    (lambda d: d['safety'].pop('max_offset_m'), 'missing'),
    (lambda d: d['safety'].update(extra=1), 'unknown'),
    (lambda d: d['safety'].update(min_lift_above_support_m=0), '> 0'),
    (lambda d: d.update(poses={}), 'non-empty'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].pop('rear_left'), 'missing legs'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].update(middle_left=[0, 0, 0]),
     'unknown leg'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].update(lf=[0, 0, 0.01]), 'given twice'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].update(front_left=[0, 0]), 'list of 3'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].update(front_left=[0, float('nan'), 0]),
     'finite number'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].update(front_left=[0, float('inf'), 0]),
     'finite number'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].update(front_left=['0', 0, 0.01]),
     'finite number'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].update(front_left=[True, 0, 0.01]),
     'finite number'),
    (lambda d: d['poses']['crouch_10mm']['offsets_m'].update(front_left=[0, 0, 0.05]),
     'max_offset_m'),
    (lambda d: d['poses']['crouch_10mm'].update(support_legs=['front_left', 'rear_left']),
     '>= 3 support legs'),
    (lambda d: d['poses']['crouch_10mm'].update(support_legs=['lf', 'front_left', 'rr']),
     'listed twice'),
    (lambda d: d['poses']['crouch_10mm'].pop('support_legs'), 'support_legs'),
    (lambda d: d['poses']['crouch_10mm'].update(negative_test_only=True), 'negative_tests'),
    (lambda d: d['negative_tests']['lr_thigh_outside_limits'].pop('negative_test_only'),
     'negative_test_only'),
    (lambda d: d['negative_tests']['lr_thigh_outside_limits'].update(
        expect={'leg': 'rear_left', 'reason': 'fine'}), 'expect.reason'),
    (lambda d: d['negative_tests']['lr_thigh_outside_limits'].update(
        expect={'leg': 'left_wheel', 'reason': 'joint_limits'}), 'expect.leg'),
])
def test_invalid_config_is_rejected(cfg_dir, urdf, mutate, match):
    data = copy.deepcopy(_raw())
    mutate(data)
    with pytest.raises(m4.PoseTargetError, match=match):
        _load(cfg_dir, urdf, data)


def test_malformed_and_duplicate_key_yaml_rejected(cfg_dir, urdf):
    with pytest.raises(m4.PoseTargetError, match='malformed YAML'):
        _load(cfg_dir, urdf, 'poses: [unclosed\n  x: {')
    text = open(os.path.join(SRC_CONFIG, m4.CONFIG_FILE)).read()
    text = text.replace('      rear_right: [0.0, 0.0, 0.010]\n',
                        '      rear_right: [0.0, 0.0, 0.010]\n      rear_right: [0.0, 0.0, 0.0]\n',
                        1)
    with pytest.raises(m4.PoseTargetError, match='duplicate key "rear_right"'):
        _load(cfg_dir, urdf, text)


def test_pose_command_unknown_pose(shipped):
    with pytest.raises(m4.PoseTargetError, match='unknown pose'):
        m4.pose_command(shipped[3], 'stand_tall_10mm')


def test_evaluation_is_deterministic(urdf):
    a = m4.load_and_evaluate(os.path.join(SRC_CONFIG, m4.CONFIG_FILE), SRC_CONFIG, urdf)[3]
    b = m4.load_and_evaluate(os.path.join(SRC_CONFIG, m4.CONFIG_FILE), SRC_CONFIG, urdf)[3]
    assert a == b


def test_module_is_ros_free():
    src = open(m4.__file__).read()
    for mod in ('import rclpy', 'from rclpy', 'tf2_ros', 'control_msgs', 'trajectory_msgs'):
        assert mod not in src
