"""colcon test: M4.5 Batch B - phase engine and foot-trajectory generator (pure, no URDF/ROS)."""
import dataclasses
import math
import os

import pytest

from spiderx_controller import gait_config as gc
from spiderx_controller import gait_phase as gp
from spiderx_controller import gait_trajectory as gt
from spiderx_controller import leg_kinematics as lk

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
CFG = gc.load_gait_config(config_dir=SRC_CONFIG)
N = CFG.analysis.samples_per_cycle
EPS = 1e-9


def _spec(name):
    return CFG.gait(name)


# ------------------------------------------------------------ phase engine
@pytest.mark.parametrize('name,min_support,max_support', [
    ('wave', 3, 4),          # one leg in swing at most, long four-foot overlap
    ('tripod_crawl', 3, 3),  # the tripod-support boundary: exactly three feet at every sample
    ('ripple', 2, 3),        # alternates three- and two-foot support
    ('amble', 2, 3),
    ('pace', 2, 2),
    ('trot', 2, 2),
])
def test_support_counts(name, min_support, max_support):
    s = gp.support_summary(_spec(name), N)
    assert (s['min_support'], s['max_support']) == (min_support, max_support)


@pytest.mark.parametrize('name', [g.name for g in CFG.gaits])
def test_each_leg_swings_one_minus_beta_of_the_cycle(name):
    spec = _spec(name)
    for leg, frac in gp.support_summary(spec, N)['swing_fraction'].items():
        assert frac == pytest.approx(1.0 - spec.duty_factor, abs=1.0 / N), leg


def test_phase_boundaries_are_half_open():
    spec = _spec('trot')                     # beta 0.5; front_left offset 0.0
    assert gp.leg_phase(spec, 'front_left', 0.0) == gp.LegPhase(gp.SWING, 0.0, 0.0)
    assert gp.leg_phase(spec, 'front_left', 0.5).phase == gp.STANCE   # touch-down instant
    assert gp.leg_phase(spec, 'front_left', 0.75).progress == pytest.approx(0.5)


def test_wave_swing_order_follows_the_config():
    spec = _spec('wave')
    first_swing = {leg: next(u for u in gp.sample_us(N) if gp.local_phase(spec, leg, u) == 0.0)
                   for leg in lk.ALL_LEGS}
    order = sorted(lk.ALL_LEGS, key=lambda leg: first_swing[leg])
    assert tuple((leg,) for leg in order) == spec.swing_order


@pytest.mark.parametrize('n', [0, -1, 2.5, True])
def test_bad_sample_counts_are_refused(n):
    with pytest.raises(ValueError):
        gp.sample_us(n)


# ------------------------------------------------------------ foot trajectory
def _offset(spec, leg, u):
    return gt.foot_offset_and_velocity(spec, leg, u)[1]


@pytest.mark.parametrize('name', [g.name for g in CFG.gaits])
def test_trajectory_is_periodic_and_continuous(name):
    """Position continuity everywhere, including both stance/swing transitions and u -> 1."""
    spec = _spec(name)
    for leg in lk.ALL_LEGS:
        phi, beta = spec.phase_offset(leg), spec.duty_factor
        for u_event in (phi, (phi + 1.0 - beta) % 1.0, 0.0):     # lift-off, touch-down, wrap
            before, after = _offset(spec, leg, (u_event - 1e-9) % 1.0), _offset(spec, leg, u_event)
            assert math.dist(before, after) < 1e-8, (leg, u_event)


@pytest.mark.parametrize('name', [g.name for g in CFG.gaits])
def test_stance_and_swing_geometry(name):
    spec = _spec(name)
    L, h = spec.step_length_m, spec.step_height_m
    for leg in lk.ALL_LEGS:
        phi, beta = spec.phase_offset(leg), spec.duty_factor
        lift_off = _offset(spec, leg, phi)
        touch_down = _offset(spec, leg, (phi + 1.0 - beta) % 1.0)
        assert lift_off[1] == pytest.approx(-L / 2) and touch_down[1] == pytest.approx(L / 2)
        assert lift_off[2] == pytest.approx(0.0) and touch_down[2] == pytest.approx(0.0)
        mid_swing = _offset(spec, leg, (phi + (1.0 - beta) / 2) % 1.0)
        assert mid_swing[2] == pytest.approx(h)                   # peak lift at mid-swing
        for u in gp.sample_us(N):
            fs = gt.foot_sample(spec, leg, u)
            if fs.phase == gp.STANCE:
                assert fs.offset[2] == 0.0 and -L / 2 - EPS <= fs.offset[1] <= L / 2 + EPS
            else:
                assert -EPS <= fs.offset[2] <= h + EPS


@pytest.mark.parametrize('name', [g.name for g in CFG.gaits])
def test_world_velocity_is_zero_in_stance_and_continuous(name):
    spec = _spec(name)
    for leg in lk.ALL_LEGS:
        phi, beta = spec.phase_offset(leg), spec.duty_factor
        for u in gp.sample_us(N):
            fs = gt.foot_sample(spec, leg, u)
            if fs.phase == gp.STANCE:
                assert gt.world_velocity(spec, fs.velocity) == pytest.approx((0, 0, 0), abs=1e-15)
        for u_event in (phi, (phi + 1.0 - beta) % 1.0):
            v_before = gt.foot_sample(spec, leg, (u_event - 1e-9) % 1.0).velocity
            v_after = gt.foot_sample(spec, leg, u_event).velocity
            assert math.dist(v_before, v_after) < 1e-6, (leg, u_event)


@pytest.mark.parametrize('name', ['wave', 'trot'])
def test_analytic_velocity_matches_finite_differences(name):
    spec = _spec(name)
    dt = 1e-6
    du = dt / spec.cycle_period_s
    for leg in lk.ALL_LEGS:
        for u in gp.sample_us(40):
            if abs(gp.leg_phase(spec, leg, u).progress) < 1e-3:
                continue                       # skip samples straddling a phase boundary
            a, b = _offset(spec, leg, u), _offset(spec, leg, u + du)
            fd = tuple((y - x) / dt for x, y in zip(a, b))
            assert fd == pytest.approx(gt.foot_sample(spec, leg, u).velocity, abs=1e-6)


def test_swing_advances_one_stride_in_the_world_frame():
    spec = _spec('wave')
    leg, phi = 'front_left', spec.phase_offset('front_left')
    tau = (1 - spec.duty_factor) * spec.cycle_period_s
    start, end = _offset(spec, leg, phi), _offset(spec, leg, (phi + 1 - spec.duty_factor) % 1.0)
    # world displacement = body-frame displacement + body travel during the swing
    assert (end[1] - start[1]) + spec.body_speed_m_s * tau == pytest.approx(spec.stride_length_m)


def test_offsets_apply_stance_centre_and_height():
    spec = dataclasses.replace(_spec('trot'), stance_center_offset_m=(0.003, -0.01),
                               stance_height_offset_m=0.008)
    fs = gt.foot_sample(spec, 'front_left', 0.5)                  # touch-down: y = +L/2
    assert fs.offset == pytest.approx((0.003, -0.01 + spec.step_length_m / 2, 0.008))


def test_targets_are_tip_plus_offset_and_sampling_is_deterministic():
    tips = {leg: (i * 0.1, 1.0 + i, -0.05) for i, leg in enumerate(lk.ALL_LEGS)}
    a = gt.sample_cycle(_spec('ripple'), 50, tips)
    assert a == gt.sample_cycle(_spec('ripple'), 50, tips)
    for sample in a:
        for leg, fs in sample['feet'].items():
            assert fs.target == pytest.approx(tuple(t + o for t, o in zip(tips[leg], fs.offset)))
        assert sample['support'] == gp.support_set(_spec('ripple'), sample['u'])
    assert [s['t'] for s in a][:2] == [0.0, _spec('ripple').cycle_period_s / 50]
