"""M1 safety rules: invalid joints and out-of-limit targets must be rejected before commanding."""
import pytest

from spiderx_controller.joint_safety import (
    UnsafeCommandError, check_pose, check_target, load_limits, load_pose)


@pytest.fixture(scope='module')
def limits():
    return load_limits()[0]


def test_twelve_joints(limits):
    assert len(limits) == 12


def test_default_test_target_is_safe_for_every_joint(limits):
    for joint in limits:
        check_target(joint, 0.2, limits)   # the documented default target


@pytest.mark.parametrize('bad', ['left_wheel_joint', 'Rigid 1', 'lf_hip ', '', 'dummy_joint'])
def test_rejects_unknown_joint(limits, bad):
    with pytest.raises(UnsafeCommandError, match='not one of the 12'):
        check_target(bad, 0.0, limits)


def test_rejects_beyond_urdf_limit(limits):
    with pytest.raises(UnsafeCommandError, match='outside the allowed range'):
        check_target('lf_foot_joint', 0.5, limits)          # URDF upper is 0.436332


def test_rejects_inside_urdf_but_inside_margin(limits):
    with pytest.raises(UnsafeCommandError):
        check_target('lf_hip', 0.68, limits, margin=0.05)   # URDF upper 0.698132


def test_accepts_edge_of_margin(limits):
    check_target('lf_hip', 0.698132 - 0.05, limits, margin=0.05)


def test_neutral_pose_valid(limits):
    check_pose(load_pose('cad_neutral'), limits)


def test_rejects_incomplete_or_bad_pose(limits):
    pose = load_pose('cad_neutral')
    del pose['rr_hip']
    with pytest.raises(UnsafeCommandError, match='missing'):
        check_pose(pose, limits)
    pose = load_pose('cad_neutral')
    pose['rf_thigh_joint'] = -1.0
    with pytest.raises(UnsafeCommandError, match='outside'):
        check_pose(pose, limits)


def test_rejects_unknown_pose():
    with pytest.raises(UnsafeCommandError, match='unknown pose'):
        load_pose('walk')
