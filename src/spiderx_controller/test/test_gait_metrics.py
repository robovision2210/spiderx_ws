"""colcon test: M4.5 Batch D - mass model, static-stability approximation, metrics, verdict."""
import dataclasses
import math
import os

import pytest

from spiderx_controller import gait_metrics as gm
from spiderx_controller import gait_trajectory as gt

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
N_FAST = 40


@pytest.fixture(scope='module')
def inputs():
    return gm.load_inputs(config_dir=SRC_CONFIG)


@pytest.fixture(scope='module')
def shipped(inputs):
    cfg, geoms, mass = inputs
    return {ev.spec.name: ev for ev in gm.evaluate_all(cfg, geoms, mass)}


def _check(ev, name):
    return next(c for c in ev.checks if c.name == name)


# ------------------------------------------------------------ mass model
def test_mass_model_reproduces_the_documented_com(inputs):
    _, _, mass = inputs
    assert mass.total_mass == pytest.approx(7.458, abs=1e-3)
    # docs/M3_LEG_KINEMATICS_PLAN.md: whole-robot COM (0.051, -0.0576, 0.0717) m at q = 0
    assert mass.com({}) == pytest.approx((0.051, -0.0576, 0.0717), abs=1e-4)
    assert all(m == pytest.approx(1.234, abs=1e-3) for m in mass.leg_mass.values())
    assert sum(mass.leg_mass.values()) < mass.total_mass


def test_moving_one_leg_moves_only_that_leg_com(inputs):
    _, geoms, mass = inputs
    names = geoms['front_left'].joint_names
    moved = {names[1]: 0.4}
    assert mass.leg_com('front_left', moved) != pytest.approx(mass.leg_com('front_left', {}))
    for leg in ('front_right', 'rear_left', 'rear_right'):
        assert mass.leg_com(leg, moved) == pytest.approx(mass.leg_com(leg, {}))
    shift = math.dist(mass.com(moved), mass.com({}))
    assert 0 < shift < 0.05


# ------------------------------------------------------------ 2-D geometry
def test_convex_hull_drops_interior_and_collinear_points():
    hull = gm.convex_hull([(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0.5, 0)])
    assert hull == [(0, 0), (1, 0), (1, 1), (0, 1)]              # counter-clockwise
    assert gm.convex_hull([(0, 0), (1, 1), (2, 2)]) == [(0, 0), (2, 2)]


@pytest.mark.parametrize('com,expected', [
    ((0.5, 0.5), 0.5), ((0.5, 0.1), 0.1), ((0.5, 0.0), 0.0), ((0.5, -0.2), -0.2),
    ((1.5, 0.5), -0.5),
])
def test_stability_margin_of_a_unit_square(com, expected):
    assert gm.stability_margin(com, [(0, 0), (1, 0), (1, 1), (0, 1)]) == pytest.approx(expected)


def test_stability_margin_needs_a_polygon():
    assert gm.stability_margin((0, 0), [(0, 0), (1, 0)]) is None
    assert gm.stability_margin((0, 0), [(0, 0), (1, 1), (2, 2)]) is None   # collinear
    tri = [(0, 0), (2, 0), (0, 2)]
    assert gm.stability_margin((0.5, 0.5), tri) == pytest.approx(0.5)      # nearest: the axes


# ------------------------------------------------------------ the shipped gaits (findings)
def test_crawl_gaits_fail_static_stability_when_the_rear_left_foot_swings(shipped):
    """Finding: the URDF COM sits ~5.8 mm behind the foot centre, so with LR lifted it lies
    outside the FL-FR-RR triangle. The tool must report this, not hide it."""
    for name in ('wave', 'tripod_crawl'):
        c = _check(shipped[name], 'static_stability')
        assert c.applicable and not c.passed and c.value < 0
        assert "'front_left', 'front_right', 'rear_right'" in c.failures[0]
        assert 'static_stability' in shipped[name].failed_checks


def test_dynamic_gaits_report_stability_for_information_only(shipped):
    for name in ('ripple', 'amble', 'pace', 'trot'):
        c = _check(shipped[name], 'static_stability')
        assert not c.applicable and 'static_stability' not in shipped[name].failed_checks
    for name in ('pace', 'trot'):                       # always two feet: no support polygon
        assert all(m is None for m in shipped[name].series['static_margin_m'])
        assert shipped[name].metrics['fraction_statically_stable'] == 0.0


def test_wave_exceeds_the_placeholder_joint_speed(shipped):
    c = _check(shipped['wave'], 'joint_speed')
    assert not c.passed and c.value > 0.5 and 'rad/s' in c.failures[0]
    assert _check(shipped['trot'], 'joint_speed').passed


def test_transitions_are_velocity_continuous(shipped):
    for ev in shipped.values():
        c = _check(ev, 'transition_velocity_jump')
        assert c.passed and c.value < 1e-6
        assert ev.metrics['max_transition_acceleration_jump_m_s2'] > 0   # reported, not hidden


def test_energy_proxies_are_consistent(shipped):
    for ev in shipped.values():
        m, spec = ev.metrics, ev.spec
        assert m['steps_per_m'] == pytest.approx(4.0 / spec.stride_length_m)
        swing = gm.swing_path_length(spec)
        assert spec.stride_length_m < swing < spec.stride_length_m + 2 * spec.step_height_m
        assert m['lift_work_proxy_j_per_m'] > 0 and m['joint_travel_rad_per_m'] > 0
    # the proxies depend on each leg's own trajectory, not on the footfall pattern
    for key in ('foot_path_per_m', 'joint_travel_rad_per_m', 'lift_work_proxy_j_per_m'):
        assert shipped['pace'].metrics[key] == pytest.approx(shipped['trot'].metrics[key])


def test_flat_swing_path_equals_the_stride():
    spec = dataclasses.replace(next(iter(gm.load_inputs(config_dir=SRC_CONFIG)[0].gaits)),
                               step_height_m=1e-12)
    assert gm.swing_path_length(spec) == pytest.approx(spec.stride_length_m, rel=1e-6)


# ------------------------------------------------------------ the checks must bite
def test_requiring_static_stability_of_a_trot_fails_on_two_foot_support(inputs):
    cfg, geoms, mass = inputs
    spec = dataclasses.replace(cfg.gait('trot'), requires_static_stability=True)
    ev = gm.evaluate_gait(spec, cfg, geoms, mass, n=N_FAST)
    c = _check(ev, 'static_stability')
    assert not c.passed and 'only 2 feet in stance' in c.failures[0]
    assert not ev.passed


def test_faster_body_speed_fails_the_joint_speed_check(inputs):
    cfg, geoms, mass = inputs
    spec = cfg.gait('trot')
    fast = dataclasses.replace(spec, body_speed_m_s=spec.body_speed_m_s * 10,
                               cycle_period_s=spec.cycle_period_s / 10)
    assert not _check(gm.evaluate_gait(fast, cfg, geoms, mass, n=N_FAST), 'joint_speed').passed


def test_velocity_jump_check_bites(inputs, monkeypatch):
    cfg = inputs[0]
    real = gt.foot_sample

    def jumpy(spec, leg, u, tip0=None):
        fs = real(spec, leg, u, tip0)
        return dataclasses.replace(fs, velocity=(0.0, 0.0, 1.0)) if fs.phase == 'swing' else fs
    monkeypatch.setattr(gt, 'foot_sample', jumpy)
    c = gm.check_transition_velocity(cfg.gait('trot'), cfg.analysis.velocity_jump_tol_m_s)
    assert not c.passed and c.failure_count == 8                  # 4 legs x 2 transitions


def test_unsolvable_gait_fails_with_metrics_marked_unknown(inputs):
    cfg, geoms, mass = inputs
    spec = dataclasses.replace(cfg.gait('trot'), step_length_m=0.30)
    ev = gm.evaluate_gait(spec, cfg, geoms, mass, n=N_FAST)
    assert 'ik_feasible' in ev.failed_checks
    assert ev.metrics['lift_work_proxy_j_per_m'] is None


def test_evaluation_is_deterministic(inputs):
    cfg, geoms, mass = inputs
    a = gm.evaluate_gait(cfg.gait('ripple'), cfg, geoms, mass, n=N_FAST)
    b = gm.evaluate_gait(cfg.gait('ripple'), cfg, geoms, mass, n=N_FAST)
    assert a.metrics == b.metrics and a.series == b.series
    assert [c.as_dict() for c in a.checks] == [c.as_dict() for c in b.checks]


def test_every_check_carries_an_honesty_label(shipped):
    for ev in shipped.values():
        labels = {c.name: c.label for c in ev.checks}
        assert labels['static_stability'] == 'approximation'
        assert labels['ik_feasible'] == 'exact' and labels['joint_speed'] == 'sampled'
        assert {c.name for c in ev.checks} >= {'ik_feasible', 'fk_residual', 'singularity_margin',
                                               'joint_limit_margin', 'joint_continuity',
                                               'static_stability', 'joint_speed',
                                               'transition_velocity_jump'}
