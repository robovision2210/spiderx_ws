"""colcon test: M3 target config validation, target evaluation and runtime decision logic.

Includes negative tests: invalid configs are refused before anything is sent, and failed
measured conditions produce "M3 kinematics validation not verified."
"""
import importlib.machinery
import importlib.util
import os
import shutil

import pytest
import yaml

from spiderx_controller import kinematics_validation as kv
from spiderx_controller import leg_kinematics as lk
from spiderx_controller.config_check import check_m3_kinematics, load_configs, load_urdf
from spiderx_controller.kinematics_targets import (
    CONFIG_FILE, TargetConfigError, evaluate, load_and_evaluate, validate_config)

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts',
                      'validate_leg_kinematics')


@pytest.fixture(scope='module')
def urdf():
    return load_urdf()


@pytest.fixture(scope='module')
def geom(urdf):
    return lk.LegGeometry.from_urdf(urdf, load_configs(SRC_CONFIG)[0], 'front_left')


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


# ------------------------------------------------------------ shipped config
def test_shipped_config_is_valid_and_all_targets_pass(cfg_dir, urdf):
    cfg, geom, errors, plan = load_and_evaluate(str(cfg_dir / CONFIG_FILE), str(cfg_dir), urdf)
    assert errors == []
    assert len(plan['safe']) == 5 and len(plan['negative']) == 2
    assert plan['reference_tip_m'] == pytest.approx(list(geom.tip0))
    for rec in plan['safe'].values():
        assert rec['ik']['ok'] and rec['target_m'][2] >= geom.tip0[2] + 0.005   # lifted
        assert not lk.check_limits(geom, rec['ik']['solution'], 0.05)
    reasons = {n: r['ik']['reason'] for n, r in plan['negative'].items()}
    assert reasons == {'unreachable_300mm_below': 'unreachable_knee',
                       'outside_joint_limits': 'joint_limits'}


def test_static_check_passes(cfg_dir):
    errors, info = check_m3_kinematics(load_configs(str(cfg_dir))[0], str(cfg_dir))
    assert errors == [] and len(info) == 2


# ------------------------------------------------------------ invalid configs
@pytest.mark.parametrize('mutate,match', [
    (lambda d: d.pop('simulation_only'), 'simulation_only'),
    (lambda d: d.update(simulation_only=False), 'simulation_only'),
    (lambda d: d.update(leg='front_right'), 'unsupported leg'),
    (lambda d: d.update(frame='world'), 'frame must be'),
    (lambda d: d.update(validation_margin_rad=0.0), 'M1 margin'),
    (lambda d: d['tolerances'].update(ik_position_tol_m=1e-3), 'IK_POSITION_TOL_M'),
    (lambda d: d['tolerances'].pop('fk_vs_gazebo_position_m'), 'missing'),
    (lambda d: d['tolerances'].update(target_reached_position_m=-0.002), '> 0'),
    (lambda d: d['tolerances'].update(fk_vs_tf_position_m='tiny'), 'finite number'),
    (lambda d: d['safety'].update(extra=1.0), 'unknown'),
    (lambda d: d['motion'].update(min_duration_s=1.0), 'min_duration_s'),
    (lambda d: d['safe_targets']['lift_10mm'].update(offset_m=[0.0, 0.0, 0.0]), 'min_lift_m'),
    (lambda d: d['safe_targets']['lift_10mm'].update(offset_m=[0.0, 0.0, 0.05]), 'max_offset_m'),
    (lambda d: d['safe_targets']['lift_10mm'].update(offset_m=[0.0, 0.01]), 'list of 3'),
    (lambda d: d['safe_targets']['lift_10mm'].update(offset_m=[0.0, float('nan'), 0.01]),
     'finite number'),
    (lambda d: d['safe_targets']['lift_10mm'].pop('rationale'), 'rationale'),
    (lambda d: d['negative_tests']['outside_joint_limits'].pop('negative_test_only'),
     'negative_test_only'),
    (lambda d: d['negative_tests']['outside_joint_limits'].update(expect='fine'), 'expect'),
    (lambda d: d['negative_tests']['outside_joint_limits'].update(offset_m=[0, 0, 0.01]),
     'exactly one'),
    (lambda d: d.update(reference={'pose': 'tuned_stand', 'point': 'derived_foot_tip'}),
     'reference'),
])
def test_invalid_config_is_rejected(cfg_dir, urdf, mutate, match):
    data = _raw()
    mutate(data)
    with pytest.raises(TargetConfigError, match=match):
        load_and_evaluate(_write(cfg_dir, data), str(cfg_dir), urdf)


def test_malformed_and_duplicate_yaml_rejected(cfg_dir, urdf):
    with pytest.raises(TargetConfigError):
        load_and_evaluate(_write(cfg_dir, 'safe_targets: [oops\n  x: {'), str(cfg_dir), urdf)
    text = open(os.path.join(SRC_CONFIG, CONFIG_FILE)).read()
    text = text.replace('  lift_10mm:\n', '  lift_10mm:\n    offset_m: [0.0, 0.0, 0.02]\n', 1)
    with pytest.raises(TargetConfigError, match='duplicate key'):
        load_and_evaluate(_write(cfg_dir, text), str(cfg_dir), urdf)


def test_evaluation_flags_unsafe_and_wrongly_accepted_targets(geom):
    data = _raw()
    data['safety']['max_joint_change_rad'] = 0.01          # every real target now too large
    data['negative_tests']['bogus_negative'] = {'negative_test_only': True,
                                                'offset_m': [0.0, 0.0, 0.01],
                                                'expect': 'unreachable'}
    cfg = validate_config(data, 0.05)
    errors, _ = evaluate(geom, cfg)
    assert any('joint change' in e for e in errors)
    assert any('bogus_negative: IK unexpectedly succeeded' in e for e in errors)


# ------------------------------------------------------------ runtime decision logic (pure)
TOL = {'ik_position_tol_m': 1e-6, 'fk_vs_tf_position_m': 1e-6, 'fk_vs_tf_orientation_rad': 1e-6,
       'fk_vs_gazebo_position_m': 1e-3, 'fk_vs_gazebo_orientation_rad': 5e-3,
       'target_reached_position_m': 2e-3}


def _obs(geom, q, dpos=(0.0, 0.0, 0.0)):
    fk = lk.forward(geom, q)
    return (tuple(a + b for a, b in zip(fk['foot_link_position'], dpos)),
            fk['foot_link_quaternion'])


def test_fk_check_passes_on_exact_observation_and_fails_on_offset(geom):
    q = [0.05, 0.1, 0.2]
    good = kv.fk_check(geom, q, _obs(geom, q), _obs(geom, q), TOL, 'x')
    assert good['passed'] and good['vs_tf']['link_origin_error_m']['euclidean'] < 1e-12
    bad = kv.fk_check(geom, q, _obs(geom, q, (0, 0, 1e-5)), _obs(geom, q, (0.002, 0, 0)), TOL, 'x')
    assert not bad['passed']
    assert any('TF: foot-link origin' in f for f in bad['failures'])
    assert any('Gazebo: foot-link origin' in f for f in bad['failures'])
    missing = kv.fk_check(geom, q, None, _obs(geom, q), TOL, 'x')
    assert not missing['passed'] and 'no observation' in missing['failures'][0]


def _ik_rec(geom, name, target_err=(0.0, 0.0, 0.0), code=0, returned=True):
    target = tuple(a + b for a, b in zip(geom.tip0, (0, 0, 0.01)))
    ik = lk.inverse(geom, target)
    q = ik['solution']
    fk = kv.fk_check(geom, q, _obs(geom, q), _obs(geom, q), TOL, name)
    pos, quat = _obs(geom, q, target_err)
    return kv.ik_target_check(geom, name, target, ik, q, (pos, quat), fk,
                              {'accepted': True, 'error_code': code}, returned, TOL)


def test_ik_target_check_pass_and_failures(geom):
    assert _ik_rec(geom, 'ok')['passed']
    assert 'mm from target' in _ik_rec(geom, 'far', target_err=(0.003, 0, 0))['failures'][0]
    assert any('trajectory result' in f for f in _ik_rec(geom, 'code', code=-5)['failures'])
    assert any('did not return' in f for f in _ik_rec(geom, 'stay', returned=False)['failures'])


def test_negative_check_detects_sent_goals_and_motion():
    ik = {'ok': False, 'reason': 'unreachable_knee'}
    assert kv.negative_check('n', ('unreachable_knee',), ik, 3, 3, 0.0)['passed']
    assert not kv.negative_check('n', ('unreachable_knee',), ik, 3, 4, 0.0)['passed']
    assert not kv.negative_check('n', ('unreachable_knee',), ik, 3, 3, 0.2)['passed']
    assert not kv.negative_check('n', ('joint_limits',), ik, 3, 3, 0.0)['passed']
    assert not kv.negative_check('n', ('unreachable_knee',), {'ok': True, 'reason': 'ok'},
                                 3, 3, 0.0)['passed']


def _run(geom, **over):
    q = [0.0, 0.0, 0.0]
    run = {'precondition_failures': [],
           'fk_checks': [kv.fk_check(geom, q, _obs(geom, q), _obs(geom, q), TOL, 'start')],
           'ik_targets': [_ik_rec(geom, 'a'), _ik_rec(geom, 'b')], 'expected_safe_targets': 2,
           'negative_tests': [kv.negative_check('n', ('unreachable_knee',),
                                                {'ok': False, 'reason': 'unreachable_knee'},
                                                0, 0, 0.0)],
           'final_return_ok': True}
    run.update(over)
    return run


def test_outcome_verified_and_not_verified(geom):
    fk, ik, lines, failures = kv.outcome(_run(geom))
    assert fk and ik and lines == [kv.FK_VERIFIED, kv.IK_VERIFIED] and failures == []
    # failed IK target -> FK still verified, overall not verified
    fk, ik, lines, _ = kv.outcome(_run(geom, ik_targets=[_ik_rec(geom, 'a', (0.01, 0, 0))]
                                       + [_ik_rec(geom, 'b')]))
    assert fk and not ik and lines == [kv.FK_VERIFIED, kv.NOT_VERIFIED]
    # precondition failure -> nothing verified
    fk, ik, lines, failures = kv.outcome(_run(geom, precondition_failures=['no /clock']))
    assert not fk and not ik and lines == [kv.NOT_VERIFIED] and 'no /clock' in failures
    # a skipped safe target or a missing final return is not verified
    assert not kv.outcome(_run(geom, expected_safe_targets=3))[1]
    assert not kv.outcome(_run(geom, final_return_ok=False))[1]
    report = kv.build_report(geom, validate_config(_raw(), 0.05), _run(geom))
    assert report['simulation_only'] is True and report['passed'] is True
    assert report['geometry']['tip_definition'].startswith('derived')


def test_tool_refuses_invalid_config_before_any_ros_call(cfg_dir, urdf, monkeypatch, capsys):
    """Negative test: an invalid target config exits 2 and never initialises ROS or sends."""
    loader = importlib.machinery.SourceFileLoader('validate_leg_kinematics', SCRIPT)
    spec = importlib.util.spec_from_loader('validate_leg_kinematics', loader)
    tool = importlib.util.module_from_spec(spec)
    loader.exec_module(tool)

    def forbidden(*_a, **_k):
        raise AssertionError('ROS was initialised although the config is invalid')
    monkeypatch.setattr(tool.rclpy, 'init', forbidden)
    monkeypatch.setattr(tool, 'KinematicsNode', forbidden)
    data = _raw()
    data['safe_targets']['lift_10mm']['offset_m'] = [0.0, 0.0, -0.01]     # pushes into ground
    path = _write(cfg_dir, data)
    monkeypatch.setattr(tool, 'load_and_evaluate',
                        lambda p: load_and_evaluate(p, str(cfg_dir), urdf))
    rc = tool.main(['validate_leg_kinematics', '--config', path,
                    '--output', str(cfg_dir / 'r.json')])
    out = capsys.readouterr().out
    assert rc == 2 and 'REFUSED' in out and 'Nothing was sent' in out
    assert not (cfg_dir / 'r.json').exists()

