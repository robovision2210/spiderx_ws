"""colcon test: the controller configs must agree with the SpiderX URDF."""
import copy

from spiderx_controller.config_check import check, load_configs, load_urdf


def test_configs_match_urdf():
    errors, _ = check(load_urdf(), *load_configs())
    assert errors == []


def test_detects_wrong_axis():
    legs, poses, ctrl = load_configs()
    legs = copy.deepcopy(legs)
    legs['legs']['rear_left']['joints']['foot']['axis'] = [1, 0, 0]
    errors, _ = check(load_urdf(), legs, poses, ctrl)
    assert any('lr_foot_joint: axis' in e for e in errors)


def test_detects_pose_outside_limits():
    legs, poses, ctrl = load_configs()
    poses = copy.deepcopy(poses)
    poses['poses']['cad_neutral']['joints']['rf_hip'] = 1.0
    errors, _ = check(load_urdf(), legs, poses, ctrl)
    assert any('rf_hip=1.0 outside soft limits' in e for e in errors)
