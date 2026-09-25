"""colcon test: the M2 simulation-only posture config is validated strictly and invalid configs
are refused BEFORE anything is sent."""
import copy
import importlib.util
import os
import shutil

import pytest
import yaml

from spiderx_controller.config_check import check_m2_posture, load_configs
from spiderx_controller.posture_config import (
    CONFIG_FILE, DEFAULT_POSTURE, PostureConfigError, load_posture)

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts',
                      'run_posture_hold_test.py')


@pytest.fixture
def cfg_dir(tmp_path):
    for name in os.listdir(SRC_CONFIG):
        if name.endswith('.yaml'):
            shutil.copy(os.path.join(SRC_CONFIG, name), tmp_path / name)
    return tmp_path


def _raw():
    with open(os.path.join(SRC_CONFIG, CONFIG_FILE)) as f:
        return yaml.safe_load(f)


def _write(cfg_dir, data):
    path = cfg_dir / CONFIG_FILE
    path.write_text(data if isinstance(data, str) else yaml.safe_dump(data))
    return str(path)


def _load(cfg_dir, data):
    return load_posture(DEFAULT_POSTURE, _write(cfg_dir, data), str(cfg_dir))


def _posture(data):
    return data['postures'][DEFAULT_POSTURE]


def test_shipped_config_is_valid_cad_neutral(cfg_dir):
    posture, order = load_posture(DEFAULT_POSTURE, None, str(cfg_dir))
    assert len(order) == 12 and sorted(posture['targets']) == sorted(order)
    assert all(v == 0.0 for v in posture['targets'].values())
    assert posture['margin'] == 0.05
    assert posture['hold_duration_s'] == 10.0


def test_rejects_missing_joint(cfg_dir):
    data = _raw()
    del _posture(data)['targets_rad']['rr_foot_joint']
    with pytest.raises(PostureConfigError, match='missing'):
        _load(cfg_dir, data)


@pytest.mark.parametrize('joint', ['left_wheel_joint', 'lf_hipp', 'dummy_joint', 'Rigid 1'])
def test_rejects_extra_or_unknown_joint(cfg_dir, joint):
    data = _raw()
    _posture(data)['targets_rad'][joint] = 0.0
    with pytest.raises(PostureConfigError, match='unknown'):
        _load(cfg_dir, data)


def test_rejects_duplicate_joint_name(cfg_dir):
    text = open(os.path.join(SRC_CONFIG, CONFIG_FILE)).read()
    text = text.replace('      lf_hip: 0.0\n', '      lf_hip: 0.0\n      lf_hip: 0.1\n', 1)
    with pytest.raises(PostureConfigError, match='duplicate key "lf_hip"'):
        _load(cfg_dir, text)


def test_rejects_target_outside_urdf_limit(cfg_dir):
    data = _raw()
    _posture(data)['targets_rad']['lf_foot_joint'] = 0.5     # URDF upper 0.436332
    with pytest.raises(PostureConfigError, match='outside the allowed range'):
        _load(cfg_dir, data)


def test_rejects_target_inside_limit_but_violating_margin(cfg_dir):
    data = _raw()
    _posture(data)['targets_rad']['lf_foot_joint'] = 0.40    # < 0.436332 but > 0.386332
    with pytest.raises(PostureConfigError, match='outside the allowed range'):
        _load(cfg_dir, data)


def test_rejects_altered_posture_even_if_safe(cfg_dir):
    data = _raw()
    _posture(data)['targets_rad']['rf_hip'] = 0.1            # safe, but not cad_neutral
    with pytest.raises(PostureConfigError, match='differs from cad_neutral'):
        _load(cfg_dir, data)


@pytest.mark.parametrize('value', ['0.0', True, None, [0.0], float('nan')])
def test_rejects_wrong_target_type(cfg_dir, value):
    data = _raw()
    _posture(data)['targets_rad']['lr_hip'] = value
    with pytest.raises(PostureConfigError):
        _load(cfg_dir, data)


def test_rejects_degrees_like_units(cfg_dir):
    data = _raw()
    _posture(data)['targets_rad']['lr_hip'] = 30               # degrees mistaken for radians
    with pytest.raises(PostureConfigError, match='outside the allowed range'):
        _load(cfg_dir, data)


@pytest.mark.parametrize('key', ['max_joint_error_rad', 'min_body_height_m', 'max_abs_pitch_rad',
                                 'min_pose_samples'])
def test_rejects_missing_threshold(cfg_dir, key):
    data = _raw()
    del _posture(data)['thresholds'][key]
    with pytest.raises(PostureConfigError, match='missing'):
        _load(cfg_dir, data)


@pytest.mark.parametrize('key,value', [
    ('max_joint_error_rad', -0.05), ('min_body_height_m', 0), ('max_abs_roll_rad', 'small'),
    ('min_pose_samples', 20.5), ('rms_joint_error_rad', 0.5), ('unknown_threshold', 1.0)])
def test_rejects_invalid_threshold(cfg_dir, key, value):
    data = _raw()
    _posture(data)['thresholds'][key] = value
    with pytest.raises(PostureConfigError):
        _load(cfg_dir, data)


@pytest.mark.parametrize('where', ['top', 'posture'])
@pytest.mark.parametrize('value', [None, False, 'true'])
def test_requires_simulation_only_true(cfg_dir, where, value):
    data = _raw()
    target = data if where == 'top' else _posture(data)
    if value is None:
        del target['simulation_only']
    else:
        target['simulation_only'] = value
    with pytest.raises(PostureConfigError, match='simulation_only'):
        _load(cfg_dir, data)


def test_rejects_malformed_yaml(cfg_dir):
    with pytest.raises(PostureConfigError, match='malformed YAML'):
        _load(cfg_dir, 'postures: [unclosed\n  targets_rad: {')


def test_rejects_margin_different_from_m1(cfg_dir):
    data = _raw()
    _posture(data)['validation_margin_rad'] = 0.0
    with pytest.raises(PostureConfigError, match='M1 safety margin'):
        _load(cfg_dir, data)


def test_rejects_fast_command_and_bad_hold(cfg_dir):
    data = _raw()
    _posture(data)['command_duration_s'] = 1.0
    with pytest.raises(PostureConfigError, match='command_duration_s'):
        _load(cfg_dir, data)
    data = _raw()
    _posture(data)['hold_duration_s'] = 0
    with pytest.raises(PostureConfigError, match='hold_duration_s'):
        _load(cfg_dir, data)


def test_rejects_unknown_posture_name(cfg_dir):
    with pytest.raises(PostureConfigError, match='unknown posture'):
        load_posture('tuned_stand', None, str(cfg_dir))


def test_static_check_passes_on_shipped_config(cfg_dir):
    errors, info = check_m2_posture(load_configs(str(cfg_dir))[0], str(cfg_dir))
    assert errors == [] and info


def test_static_check_rejects_second_posture(cfg_dir):
    data = _raw()
    data['postures']['cad_neutral_copy'] = copy.deepcopy(_posture(data))
    _write(cfg_dir, data)
    errors, _ = check_m2_posture(load_configs(str(cfg_dir))[0], str(cfg_dir))
    assert any('only the CAD neutral posture' in e for e in errors)


def test_tool_refuses_invalid_config_before_any_ros_call(cfg_dir, monkeypatch, capsys):
    """Negative test: an invalid config returns exit 2 and never initialises ROS or sends."""
    spec = importlib.util.spec_from_file_location('run_posture_hold_test', SCRIPT)
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)

    def forbidden(*_a, **_k):
        raise AssertionError('ROS was initialised although the config is invalid')
    monkeypatch.setattr(tool.rclpy, 'init', forbidden)
    monkeypatch.setattr(tool, 'PostureHoldNode', forbidden)
    data = _raw()
    _posture(data)['targets_rad']['lf_foot_joint'] = 0.6
    path = _write(cfg_dir, data)
    monkeypatch.setattr(tool, 'load_posture',
                        lambda name, cfg: load_posture(name, cfg, str(cfg_dir)))
    rc = tool.main(['run_posture_hold_test.py', '--config', path,
                    '--output', str(cfg_dir / 'r.json')])
    out = capsys.readouterr().out
    assert rc == 2
    assert 'REFUSED' in out and 'Nothing was sent' in out
    assert not (cfg_dir / 'r.json').exists()
