"""colcon test: M4.5 Batch C - sampled IK of gait cycles via the unchanged M3/M4 kinematics."""
import dataclasses
import math
import os

import pytest

from spiderx_controller import gait_config as gc
from spiderx_controller import gait_kinematics as gk
from spiderx_controller import leg_kinematics as lk
from spiderx_controller.gait_results import Check

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
CFG = gc.load_gait_config(config_dir=SRC_CONFIG)
N_FAST = 40                      # fewer samples where only the mechanism is tested


@pytest.fixture(scope='module')
def geoms():
    return lk.load_all_geometries(config_dir=SRC_CONFIG)


@pytest.fixture(scope='module')
def shipped(geoms):
    return {g.name: gk.solve_cycle(g, geoms, CFG.analysis.samples_per_cycle, CFG.margin_rad)
            for g in CFG.gaits}


def _by_name(checks):
    return {c.name: c for c in checks}


# ------------------------------------------------------------ the shipped gaits
def test_every_shipped_gait_is_kinematically_feasible(shipped):
    for name, kin in shipped.items():
        checks = _by_name(gk.kinematic_checks(kin, CFG))
        assert kin.all_solved, name
        assert all(c.passed for c in checks.values()), (name, [c.as_dict() for c in checks.values()
                                                               if not c.passed])


def test_ik_reproduces_every_target(shipped, geoms):
    for kin in shipped.values():
        for leg, s in kin.legs.items():
            for k, q in enumerate(s.q):
                tip = lk.forward(geoms[leg], q)['tip_position']
                assert math.dist(tip, kin.samples[k]['feet'][leg].target) <= lk.IK_POSITION_TOL_M


def test_mid_stance_at_neutral_height_is_cad_neutral(shipped):
    """Mid-stance (s = 0.5) puts the foot exactly on its neutral tip, so IK must return ~zeros."""
    kin = shipped['trot']
    for leg, s in kin.legs.items():
        for k, sample in enumerate(kin.samples):
            fs = sample['feet'][leg]
            if fs.phase == 'stance' and abs(fs.progress - 0.5) < 1e-12:
                assert s.q[k] == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
                break
        else:
            pytest.fail(f'{leg}: no mid-stance sample')


def test_reference_keeps_one_branch(shipped):
    for name, kin in shipped.items():
        for s in kin.legs.values():
            steps = [max(st) for st in gk.joint_steps(s.q)]
            assert max(steps) < CFG.analysis.max_joint_step_rad, name


def test_solving_is_deterministic(geoms):
    a = gk.solve_cycle(CFG.gait('ripple'), geoms, N_FAST, CFG.margin_rad)
    b = gk.solve_cycle(CFG.gait('ripple'), geoms, N_FAST, CFG.margin_rad)
    assert a.legs == b.legs


# ------------------------------------------------------------ the checks must bite
def test_over_long_stroke_is_unreachable_and_named(geoms):
    spec = dataclasses.replace(CFG.gait('trot'), step_length_m=0.30)
    kin = gk.solve_cycle(spec, geoms, N_FAST, CFG.margin_rad)
    c = _by_name(gk.kinematic_checks(kin, CFG))['ik_feasible']
    assert not c.passed and c.value > 0
    assert any(r in c.failures[0] for r in ('unreachable', 'joint_limits'))
    assert c.failures[0].split()[0] in lk.ALL_LEGS and 'sample' in c.failures[0]


def test_stance_height_beyond_reach_fails(geoms):
    spec = dataclasses.replace(CFG.gait('wave'), stance_height_offset_m=0.06)   # > +33 mm reach
    kin = gk.solve_cycle(spec, geoms, N_FAST, CFG.margin_rad)
    assert not kin.all_solved
    assert not gk.check_ik_feasible(kin).passed


def test_failed_samples_do_not_poison_later_ones(geoms):
    """A failing sample keeps the last good reference; later reachable samples still solve."""
    spec = dataclasses.replace(CFG.gait('trot'), step_length_m=0.17)
    kin = gk.solve_cycle(spec, geoms, N_FAST, CFG.margin_rad)
    reasons = kin.legs['front_left'].reason
    assert 'ok' in reasons and any(r != 'ok' for r in reasons)


def test_singularity_threshold_bites(shipped):
    c = gk.check_min_series(shipped['wave'], 'singularity_margin_rad', 'singularity_margin', 2.0,
                            'rad', 'x')
    assert not c.passed and c.failure_count > 0 and c.value < 2.0


# ------------------------------------------------------------ pure helpers
def test_joint_steps_and_speeds_are_cyclic_and_none_safe():
    qs = [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), None, (0.0, 0.2, 0.0)]
    steps = gk.joint_steps(qs)
    assert steps[0] == (0.0, 0.2, 0.0)            # wrap: sample 0 vs the last sample
    assert steps[1] == pytest.approx((0.1, 0.0, 0.0))
    assert steps[2] is None and steps[3] is None
    speeds = gk.joint_speeds(qs, 0.5)
    assert speeds[0] == pytest.approx((0.1, 0.2, 0.0))     # (q1 - q_last) / (2 dt)
    assert speeds[1] is None and speeds[3] is None


def test_continuity_check_flags_a_branch_flip():
    class _Kin:                                   # minimal stand-in with the fields the check uses
        samples = [{'u': k / 4, 't': k} for k in range(4)]
        legs = {'front_left': gk.LegSeries('front_left', ('a', 'b', 'c'),
                                           [(0, 0, 0), (0.01, 0, 0), (1.5, 0, 0), (0.01, 0, 0)],
                                           ['ok'] * 4, [0.0] * 4, [1.0] * 4, [0.1] * 4)}
    c = gk.check_joint_continuity(_Kin, 0.05)
    assert not c.passed and c.value == pytest.approx(1.49) and c.failure_count == 2
    assert 'a jumps' in c.failures[0]


def test_check_labels_are_enforced():
    with pytest.raises(ValueError):
        Check('x', True, 1, 1, 'm', 'measured', 'bad label')
