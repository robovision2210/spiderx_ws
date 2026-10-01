"""colcon test: M4.5 Batch F - regression pins for the SHIPPED gait configuration.

These pin the reported offline results of config/m4_5_gaits.yaml (config_version 1) at the
configured 200 samples per cycle, so that any change to the trajectory, IK, mass model or checks
that moves a verdict or a key metric is noticed and has to be explained in docs/M4_5_TEST_RESULTS.md.
They are REPORTED results, not targets: the crawl gaits failing static stability is a finding
(the URDF COM sits ~5.8 mm behind the foot centre), and must not be "fixed" by loosening a check.
Stability values are quasi-static approximations; energy values are heuristic proxies.
"""
import os

import pytest

from spiderx_controller import gait_metrics as gm

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')

EXPECTED_ORDER = ['wave', 'tripod_crawl', 'ripple', 'amble', 'pace', 'trot']
EXPECTED_FAILED = {
    'wave': ['static_stability', 'joint_speed'],
    'tripod_crawl': ['static_stability'],
    'ripple': [], 'amble': [], 'pace': [], 'trot': [],
}
# gait: (min static margin m | None, fraction statically stable, max joint speed rad/s,
#        lift work proxy J/m, joint travel rad/m)
EXPECTED_METRICS = {
    'wave': (-0.002582, 0.845, 0.5405, 5.124, 90.72),
    'tripod_crawl': (-0.004073, 0.770, 0.3048, 4.642, 81.38),
    'ripple': (-0.002098, 0.505, 0.2029, 4.174, 72.25),
    'amble': (0.000842, 0.200, 0.1465, 3.719, 63.37),
    'pace': (None, 0.0, 0.1269, 3.498, 59.01),
    'trot': (None, 0.0, 0.1269, 3.498, 59.01),
}


@pytest.fixture(scope='module')
def results():
    cfg, geoms, mass = gm.load_inputs(config_dir=SRC_CONFIG)
    assert cfg.config_version == 1 and cfg.analysis.samples_per_cycle == 200
    return {ev.spec.name: ev for ev in gm.evaluate_all(cfg, geoms, mass)}


def test_gait_set_and_order(results):
    assert list(results) == EXPECTED_ORDER


@pytest.mark.parametrize('name', EXPECTED_ORDER)
def test_verdict_and_failed_checks(results, name):
    ev = results[name]
    assert ev.failed_checks == EXPECTED_FAILED[name]
    assert ev.passed == (not EXPECTED_FAILED[name])


@pytest.mark.parametrize('name', EXPECTED_ORDER)
def test_key_metrics(results, name):
    m = results[name].metrics
    margin, stable, speed, lift, travel = EXPECTED_METRICS[name]
    if margin is None:
        assert m['min_static_margin_m'] is None
    else:
        assert m['min_static_margin_m'] == pytest.approx(margin, abs=2e-6)
    assert m['fraction_statically_stable'] == pytest.approx(stable, abs=1e-9)
    assert m['max_joint_speed_rad_s'] == pytest.approx(speed, abs=1e-4)
    assert m['lift_work_proxy_j_per_m'] == pytest.approx(lift, abs=1e-3)
    assert m['joint_travel_rad_per_m'] == pytest.approx(travel, abs=1e-2)


@pytest.mark.parametrize('name', EXPECTED_ORDER)
def test_every_gait_is_kinematically_feasible(results, name):
    """All six shipped gaits solve IK at every sample with margin to singularity and limits."""
    m, failed = results[name].metrics, results[name].failed_checks
    for check in ('ik_feasible', 'fk_residual', 'singularity_margin', 'joint_limit_margin',
                  'joint_continuity', 'transition_velocity_jump'):
        assert check not in failed
    assert m['min_singularity_margin_rad'] == pytest.approx(1.2116, abs=1e-3)
    assert m['min_joint_limit_margin_rad'] == pytest.approx(0.205, abs=1e-3)


def test_equal_body_speed_and_stride(results):
    for ev in results.values():
        assert ev.metrics['body_speed_m_s'] == pytest.approx(0.005)
        assert ev.metrics['stride_length_m'] == pytest.approx(0.04 / ev.spec.duty_factor)
