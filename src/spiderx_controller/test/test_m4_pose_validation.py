"""colcon test: M4 runtime tool - non-Gazebo decision logic (ROS graph never started)."""
import os

import pytest
import yaml

from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m4_pose_validation as v
from spiderx_controller.config_check import load_urdf
from spiderx_controller.m4_pose_targets import load_and_evaluate, pose_command

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')


@pytest.fixture(scope='module')
def shipped():
    return load_and_evaluate(os.path.join(SRC_CONFIG, 'm4_pose_targets.yaml'), SRC_CONFIG,
                             load_urdf())


def _obs(geoms, q12, dpos=(0.0, 0.0, 0.0)):
    fk = lk.forward_all(geoms, q12)
    return {leg: (tuple(a + b for a, b in zip(fk[leg]['foot_link_position'], dpos)),
                  fk[leg]['foot_link_quaternion']) for leg in lk.ALL_LEGS}


def _body(height, n=100, tilt_q=(0.0, 0.0, 0.0, 1.0)):
    return v.body_stats([(0.05 * i, (0.0, 0.0, height) + tilt_q) for i in range(n)])


def _pose_rec(geoms, plan, name, tip_d=(0.0, 0.0, 0.0), height=None, code=0, returned=True,
              body=None):
    cmd = pose_command(plan, name)
    targets = {leg: tuple(t) for leg, t in plan['poses'][name]['targets_m'].items()}
    exp = plan['poses'][name]['geometric_expected_body_height_m']['value']
    return {'expected_body_height_m': exp, 'action': {'accepted': True, 'error_code': code},
            'fk_checks': [v.fk_check_all(geoms, cmd, _obs(geoms, cmd), _obs(geoms, cmd), s)
                          for s in ('start', 'end')],
            'tip_errors': v.tip_errors(geoms, targets, _obs(geoms, cmd, tip_d)),
            'returned_to_neutral': returned, 'hold_s': 5.0,
            'body': body if body is not None else _body(exp if height is None else height)}


def _run(shipped, **over):
    _, geoms, _, plan = shipped
    run = {'precondition_failures': [],
           'start_fk': [v.fk_check_all(geoms, [0.0] * 12, _obs(geoms, [0.0] * 12),
                                       _obs(geoms, [0.0] * 12), 'start')],
           'poses': {n: _pose_rec(geoms, plan, n) for n in v.POSE_ORDER},
           'negative_poses': {n: {'accepted': False, 'commanded': False, 'failing_legs': {}}
                              for n in plan['negative']},
           'final_return_ok': True}
    run.update(over)
    return run


# ------------------------------------------------------------ provenance of thresholds
def test_tolerances_equal_the_verified_m3_values():
    with open(os.path.join(SRC_CONFIG, 'm3_kinematics_targets.yaml')) as f:
        m3 = yaml.safe_load(f)
    assert v.TOLERANCES == m3['tolerances']
    assert (v.SETTLE_S, v.MIN_DURATION_S, v.SPEED_FACTOR) == (
        m3['motion']['settle_s'], m3['motion']['min_duration_s'], m3['motion']['speed_factor'])


def test_hold_thresholds_equal_the_verified_m2_values():
    with open(os.path.join(SRC_CONFIG, 'm2_simulation_postures.yaml')) as f:
        th = yaml.safe_load(f)['postures']['cad_neutral_simulation_hold']['thresholds']
    assert (v.MAX_ABS_ROLL_RAD, v.MAX_ABS_PITCH_RAD, v.MIN_HOLD_SAMPLES) == (
        th['max_abs_roll_rad'], th['max_abs_pitch_rad'], th['min_pose_samples'])
    assert v.HEIGHT_TOL_M == 0.003 and v.HOLD_S == 5.0          # docs/M4_PLAN.md


def test_pose_order_matches_config(shipped):
    assert tuple(shipped[3]['poses']) == v.POSE_ORDER


# ------------------------------------------------------------ decisions
def test_all_good_run_is_verified(shipped):
    _, geoms, _, plan = shipped
    fk, ik, hold, lines, failures = v.outcome(_run(shipped))
    assert (fk, ik, hold) == (True, True, True) and failures == []
    assert lines == [v.FK_VERIFIED, v.IK_VERIFIED, v.HOLD_VALIDATED]


def test_fk_check_all_detects_one_bad_leg(shipped):
    _, geoms, _, _ = shipped
    q = pose_command(shipped[3], 'crouch_10mm')
    gz = _obs(geoms, q)
    gz['rear_left'] = ((gz['rear_left'][0][0] + 0.002,) + gz['rear_left'][0][1:], gz['rear_left'][1])
    rec = v.fk_check_all(geoms, q, _obs(geoms, q), gz, 'x')
    assert not rec['passed'] and rec['legs']['front_left']['passed']
    assert any('rear_left' in f and 'Gazebo' in f for f in rec['failures'])
    missing = v.fk_check_all(geoms, q, {}, gz, 'x')
    assert not missing['passed'] and any('no observation' in f for f in missing['failures'])


@pytest.mark.parametrize('name,over,expect_ik,expect_hold,match', [
    ('crouch_10mm', {'tip_d': (0.003, 0.0, 0.0)}, False, True, 'mm from target'),
    ('crouch_10mm', {'code': -5}, False, True, 'trajectory result'),
    ('lift_lf_15mm', {'returned': False}, False, True, 'did not return'),
    ('crouch_10mm', {'height': 0.0545}, True, False, 'deviates'),     # body did not lower
    ('neutral_stance', {'height': 0.0500}, True, False, 'deviates'),
    ('lift_lf_15mm', {}, True, True, None),
])
def test_pose_criteria(shipped, name, over, expect_ik, expect_hold, match):
    """FK checks use exact observations here, so only the IK/hold criteria can fail."""
    _, geoms, _, plan = shipped
    rec = _pose_rec(geoms, plan, name, **over)
    fk, ik, hold, failures = v.evaluate_pose(name, rec, rec['expected_body_height_m'])
    assert (fk, ik, hold) == (True, expect_ik, expect_hold)
    if match:
        assert any(match in f for f in failures)
    else:
        assert failures == []


def test_tilt_and_too_few_samples_fail_the_hold(shipped):
    _, geoms, _, plan = shipped
    s = 0.0998334166                                   # 0.2 rad about x
    tilted = _pose_rec(geoms, plan, 'lift_lf_15mm',
                       body=_body(0.054543, tilt_q=(s, 0.0, 0.0, 0.9950041653)))
    assert any('|roll|' in f for f in v.evaluate_pose('lift_lf_15mm', tilted, 0.054543)[3])
    few = _pose_rec(geoms, plan, 'neutral_stance', body=_body(0.054543, n=5))
    assert any('body samples' in f for f in v.evaluate_pose('neutral_stance', few, 0.054543)[3])
    short = _pose_rec(geoms, plan, 'neutral_stance')
    short['hold_s'] = 3.0
    assert any('hold lasted' in f for f in v.evaluate_pose('neutral_stance', short, 0.054543)[3])


def test_outcome_lines_for_failures(shipped):
    _, geoms, _, plan = shipped
    run = _run(shipped)
    run['poses']['crouch_10mm'] = _pose_rec(geoms, plan, 'crouch_10mm', height=0.0545)
    fk, ik, hold, lines, _ = v.outcome(run)
    assert (fk, ik, hold) == (True, True, False)
    assert lines == [v.FK_VERIFIED, v.IK_VERIFIED, v.NOT_VERIFIED]
    fk, ik, hold, lines, failures = v.outcome(_run(shipped, precondition_failures=['no /clock']))
    assert lines == [v.NOT_VERIFIED] and 'no /clock' in failures
    assert v.outcome(_run(shipped, final_return_ok=False))[3][-1] == v.NOT_VERIFIED
    missing = _run(shipped)
    del missing['poses']['lift_lf_15mm']
    assert v.outcome(missing)[3] == [v.NOT_VERIFIED]
    commanded = _run(shipped)
    next(iter(commanded['negative_poses'].values()))['commanded'] = True
    assert v.NOT_VERIFIED in v.outcome(commanded)[3]


def test_report_fields(shipped):
    cfg, geoms, _, _ = shipped
    r = v.build_report(geoms, cfg, _run(shipped))
    assert r['simulation_only'] is True and r['passed'] is True
    assert r['legs'] == list(lk.ALL_LEGS) and len(r['joint_names']) == 12
    for name in v.POSE_ORDER:
        p = r['poses'][name]
        assert p['passed'] and 'height_deviation_max_m' in p and 'tip_errors' in p
    assert r['hold_criteria']['max_abs_roll_rad'] == 0.10


def test_duration_rule():
    assert v.duration_for([0.0] * 12, [0.0] * 12, 0.5) == 3.0
    assert v.duration_for([0.0] * 12, [1.0] + [0.0] * 11, 0.5) == pytest.approx(4.0)


def test_tool_refuses_invalid_config_before_ros(tmp_path, monkeypatch, capsys):
    data = yaml.safe_load(open(os.path.join(SRC_CONFIG, 'm4_pose_targets.yaml')))
    data['simulation_only'] = False
    cfg = tmp_path / 'm4_pose_targets.yaml'
    cfg.write_text(yaml.safe_dump(data))

    def forbidden(*_a, **_k):
        raise AssertionError('ROS node created although the config is invalid')
    monkeypatch.setattr(v, '_make_node', forbidden)
    import spiderx_controller.m4_pose_targets as t
    monkeypatch.setattr(t, 'default_config_dir', lambda: SRC_CONFIG)
    rc = v.main(['m4_pose_validation', '--config', str(cfg), '--output', str(tmp_path / 'r.json')])
    out = capsys.readouterr().out
    assert rc == 2 and 'REFUSED' in out and 'simulation_only' in out and 'Nothing was sent' in out
    assert not (tmp_path / 'r.json').exists()


def test_check_only_validates_without_ros(monkeypatch, capsys):
    def forbidden(*_a, **_k):
        raise AssertionError('ROS node created in --check-only mode')
    monkeypatch.setattr(v, '_make_node', forbidden)
    import spiderx_controller.m4_pose_targets as t
    monkeypatch.setattr(t, 'default_config_dir', lambda: SRC_CONFIG)
    assert v.main(['m4_pose_validation', '--check-only']) == 0
    assert "Poses ['neutral_stance', 'crouch_10mm', 'lift_lf_15mm'] validated" in \
        capsys.readouterr().out


def test_report_is_complete_when_the_tool_stops_early(shipped):
    """An early refusal (e.g. controllers inactive) still yields every report field."""
    cfg, geoms, _, _ = shipped
    r = v.build_report(geoms, cfg, {'precondition_failures': ['M1 controllers not all active']})
    assert r['joint_state_publishers'] == [] and r['passed'] is False
    assert r['outcome'] == [v.NOT_VERIFIED]
