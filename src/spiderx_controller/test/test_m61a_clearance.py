"""M6.1-A ground clearance of the welded SpiderX (offline; no Gazebo).

Proves the analysis is consistent with the existing kinematics (M3/M4 foot tips and FK), that the
cubic-Hermite speed bound is exact, that every "bound" is conservative against dense sampling,
and that config/m61a_fixed_base.yaml agrees with a fresh computation.
"""
import math
import random
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller import m61a_clearance as cl
from spiderx_controller import m61a_fixed_base as fb


@pytest.fixture(scope='module')
def inputs():
    return cl.load_default_inputs()


@pytest.fixture(scope='module')
def model(inputs):
    return cl.CollisionModel(inputs[0])


def test_every_collision_is_loaded(model):
    assert len(model.geometry) == 31
    assert model.geometry['lidar_link'].cylinders and not len(model.geometry['lidar_link'].vertices)
    assert all(len(g.vertices) > 100 for n, g in model.geometry.items() if n != 'lidar_link')
    assert model.root == 'dummy_link' and sorted(model.revolute) == sorted(
        j for leg in ('lf', 'rf', 'lr', 'rr')
        for j in (f'{leg}_hip', f'{leg}_thigh_joint', f'{leg}_foot_joint'))


def test_neutral_foot_minimum_equals_the_m3_derived_foot_tip(inputs, model):
    geoms = lk.load_all_geometries(urdf_root=inputs[0])
    z = model.link_min_z({})
    for g in geoms.values():
        assert z[g.foot_link] == pytest.approx(g.tip0[2], abs=1e-12)
    assert min(z.values()) == pytest.approx(-0.0545, abs=1e-4)      # the M2/URDF-audit figure


def test_collision_fk_matches_leg_kinematics(inputs, model):
    geoms = lk.load_all_geometries(urdf_root=inputs[0])
    rng = random.Random(3)
    for g in geoms.values():
        for _ in range(20):
            q = [rng.uniform(-0.4, 0.4) for _ in range(3)]
            R, p = model.pose(g.foot_link, dict(zip(g.joint_names, q)))
            fk = lk.forward(g, q)
            assert np.allclose(p, fk['foot_link_position'], atol=1e-12)
            assert np.allclose(R, np.array(fk['foot_link_rotation']), atol=1e-12)


def test_hermite_positions_and_exact_speed_bounds(inputs):
    traj = inputs[1]
    pts = traj['points']
    for a in range(len(pts) - 1):
        bounds = cl.hermite_speed_bounds(pts[a], pts[a + 1])
        ta, tb = pts[a]['time_from_start_s'], pts[a + 1]['time_from_start_s']
        dense = [0.0] * len(bounds)
        for i in range(2001):
            t = ta + (tb - ta) * i / 2000
            q, qd = tc._segment(pts[a], pts[a + 1], t)
            assert cl.hermite_q(pts[a], pts[a + 1], t) == pytest.approx(q, abs=1e-15)
            dense = [max(d, abs(v)) for d, v in zip(dense, qd)]
        for b, d in zip(bounds, dense):
            assert d <= b + 1e-12 and b - d < 1e-6          # exact up to the sampling
    assert max(max(cl.hermite_speed_bounds(pts[a], pts[a + 1])) for a in range(8)) == \
        pytest.approx(0.139, abs=0.001)                      # the documented M6.1 peak


def test_lipschitz_box_bound_is_conservative(model):
    rng = random.Random(5)
    links = ['rr_foot_1', 'lf_thigh_1', 'rf_holder_1']
    for _ in range(25):
        c = {j: rng.uniform(-0.3, 0.3) for j in model.revolute}
        d = rng.uniform(0.01, 0.08)
        z0 = model.link_min_z(c, links)
        for n in links:
            bound = z0[n] - model.lipschitz(n, {j: d for j in model.revolute})
            for _ in range(30):
                q = {j: v + rng.uniform(-d, d) for j, v in c.items()}
                assert model.link_min_z(q, [n])[n] >= bound - 1e-12


def test_branch_and_bound_brackets_the_true_minimum(model):
    links = cl.leg_groups(model)[('rr_foot_joint', 'rr_hip', 'rr_thigh_joint')]
    lo = {'rr_foot_joint': 0.3, 'rr_hip': -0.3, 'rr_thigh_joint': 0.0}
    hi = {'rr_foot_joint': 0.6, 'rr_hip': 0.0, 'rr_thigh_joint': 0.3}
    r = cl.branch_and_bound_min(model, links, lo, hi, eps_m=1e-3)
    assert r['bound'] <= r['sampled'] and r['proven_within_m'] <= 1e-3 + 1e-12
    rng = random.Random(11)
    samples = [min(model.link_min_z({j: rng.uniform(lo[j], hi[j]) for j in lo}, links).values())
               for _ in range(300)]
    assert r['bound'] <= min(samples) + 1e-12
    assert min(model.link_min_z(r['configuration'], links).values()) == pytest.approx(
        r['sampled'])


def test_commanded_envelope_bound_is_below_dense_samples(inputs, model):
    traj = inputs[1]
    env = cl.commanded_envelope(model, traj, samples_per_segment=10)
    dense = cl.commanded_envelope(model, traj, samples_per_segment=100)
    assert env['bound_min_z_body_m'] <= dense['sampled_min_z_body_m'] + 1e-12
    assert env['bound_min_z_body_m'] <= env['sampled_min_z_body_m']
    assert dense['sampled_limiting_link'].endswith('_foot_1')


def test_config_agrees_with_a_fresh_passive_bound(inputs, model):
    """The stored bound, minimum height and mount follow from the geometry (no hand edits)."""
    cfg = fb.load_config()
    jl = cl.joint_limit_envelope(model)
    assert jl['proven_within_m'] <= cl.BNB_EPS_M + 1e-12
    assert jl['sampled_limiting_link'].endswith('_foot_1')
    assert cl.check_config(cfg.raw['clearance'], cfg.mount.p[2], jl['bound_min_z_body_m']) == []
    rep = {'joint_limit_box': jl, 'neutral': {'min_z_body_m': -0.0545},
           'commanded': {'sampled_min_z_body_m': -0.0546, 'bound_min_z_body_m': -0.0547},
           'run_envelope_bound_min_z_body_m': -0.067}
    rec = cl.recommend(rep, cfg.raw['clearance']['margin_m'])
    assert rec['recommended_height_m'] == cfg.mount.p[2] == cfg.min_mount_height_m
    assert rec['clearance_at_recommended_m']['joint_limit_box_bound'] >= \
        cfg.raw['clearance']['margin_m']
    assert rec['previous_candidate_0_075_m']['joint_limit_box_bound'] < 0   # 0.075 m can touch


def test_check_config_reports_inconsistencies():
    good = {'passive_bound_min_z_body_m': -0.10906, 'margin_m': 0.015,
            'min_mount_height_m': 0.125}
    assert cl.check_config(good, 0.125, -0.10906) == []
    assert any('differs' in p for p in cl.check_config(good, 0.125, -0.12))
    assert any('required' in p for p in cl.check_config(dict(good, min_mount_height_m=0.12),
                                                        0.125, -0.10906))
    assert any('mount z' in p for p in cl.check_config(good, 0.11, -0.10906))
    assert cl.required_height(-0.1, 0.02) == pytest.approx(0.12)


def test_unsupported_collision_geometry_is_refused():
    urdf = ET.fromstring('<robot name="r"><link name="a"><collision><geometry><capsule/>'
                         '</geometry></collision></link></robot>')
    with pytest.raises(cl.ClearanceError, match='unsupported'):
        cl.CollisionModel(urdf, body_link='a')


def test_primitive_lowest_points_are_exact():
    g = cl.Geometry(vertices=np.zeros((0, 3)),
                    cylinders=[(np.eye(3), np.zeros(3), 0.035, 0.04)],
                    boxes=[(np.eye(3), np.array([0, 0, 1.0]), np.array([0.1, 0.2, 0.3]))],
                    spheres=[(np.array([0, 0, -1.0]), 0.05)])
    assert g.min_z(np.eye(3), np.zeros(3)) == pytest.approx(-1.05)
    tilt = np.array(lk.rot_rpy(math.pi / 2, 0, 0))                # cylinder axis horizontal
    g2 = cl.Geometry(vertices=np.zeros((0, 3)), cylinders=[(np.eye(3), np.zeros(3), 0.035, 0.04)])
    assert g2.min_z(tilt, np.zeros(3)) == pytest.approx(-0.035)
    assert g2.min_z(np.eye(3), np.zeros(3)) == pytest.approx(-0.02)
