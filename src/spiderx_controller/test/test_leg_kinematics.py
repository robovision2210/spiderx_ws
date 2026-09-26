"""colcon test: M3 single-leg (front_left) forward/inverse kinematics against the expanded URDF.

The reference implementation here is an INDEPENDENT numpy URDF walk (quaternion rotations, its own
XML and STL parsing), so FK is not only compared with itself.
"""
import copy
import math
import os
import random

import numpy as np
import pytest
import yaml

from spiderx_controller import leg_kinematics as lk
from spiderx_controller.config_check import load_urdf

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
LF = ['lf_hip', 'lf_thigh_joint', 'lf_foot_joint']


@pytest.fixture(scope='module')
def urdf():
    return load_urdf()


@pytest.fixture(scope='module')
def legs():
    with open(os.path.join(SRC_CONFIG, 'spiderx_legs.yaml')) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def geom(urdf, legs):
    return lk.LegGeometry.from_urdf(urdf, legs, 'lf')


# ------------------------------------------------------------ independent reference (numpy)
def _quat(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[math.cos(angle / 2)], math.sin(angle / 2) * a])   # (w, x, y, z)


def _qmul(p, q):
    w1, v1, w2, v2 = p[0], p[1:], q[0], q[1:]
    return np.concatenate([[w1 * w2 - v1 @ v2], w1 * v2 + w2 * v1 + np.cross(v1, v2)])


def _qrot(q, v):
    return _qmul(_qmul(q, np.concatenate([[0.0], v])), q * np.array([1, -1, -1, -1]))[1:]


def _rpy_quat(r, p, y):
    return _qmul(_quat([0, 0, 1], y), _qmul(_quat([0, 1, 0], p), _quat([1, 0, 0], r)))


def reference_walk(urdf, q):
    """(position, quaternion wxyz) of lf_foot_1 in base_link by walking the raw URDF XML."""
    joints = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    chain, link = [], 'lf_foot_1'
    while link != 'base_link':
        j = joints[link]
        chain.append(j)
        link = j.find('parent').get('link')
    pos, rot = np.zeros(3), np.array([1.0, 0, 0, 0])
    angles = dict(zip(LF, q))
    for j in reversed(chain):
        o = j.find('origin')
        xyz = np.array([float(v) for v in o.get('xyz').split()]) if o is not None else np.zeros(3)
        rpy = [float(v) for v in o.get('rpy').split()] if o is not None else [0, 0, 0]
        pos = pos + _qrot(rot, xyz)
        rot = _qmul(rot, _rpy_quat(*rpy))
        if j.get('type') == 'revolute':
            axis = [float(v) for v in j.find('axis').get('xyz').split()]
            rot = _qmul(rot, _quat(axis, angles[j.get('name')]))
    return pos, rot


def reference_tip_from_mesh():
    """Independent derivation of the documented foot tip (base_link, q = 0) from the STL."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                        'spiderx_description', 'meshes', 'lf_foot_1.stl')
    data = open(path, 'rb').read()
    n = int(np.frombuffer(data[80:84], '<u4')[0])
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=[('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    v = rec['v'].reshape(-1, 3).astype(float) * 0.001   # mesh is in CAD = base_link coordinates
    zmin = v[:, 2].min()
    low = np.unique(np.round(v[v[:, 2] <= zmin + 1e-4], 9), axis=0)
    return np.array([low[:, 0].mean(), low[:, 1].mean(), zmin])


def _rand_q(geom, rng, margin=0.05):
    return [rng.uniform(lo + margin, hi - margin) for lo, hi in geom.limits]


# ------------------------------------------------------------ geometry
def test_selected_chain_and_joint_order(geom):
    assert list(geom.joint_names) == LF
    assert [j['name'] for j in geom.chain] == [
        'Rigid 1', 'lf_hip', 'Rigid 12', 'Rigid 16', 'lf_thigh_joint', 'Rigid 25', 'lf_foot_joint']
    assert geom.foot_link == 'lf_foot_1' and geom.frame == 'base_link'


def test_extracted_chain_matches_legs_yaml(geom, legs):
    """Changing the URDF without spiderx_legs.yaml (or vice versa) must fail."""
    fl = legs['legs']['front_left']['joints']
    for role, name, (lo, hi) in zip(legs['joint_order'], geom.joint_names, geom.limits):
        cfg = fl[role]
        j = [c for c in geom.chain if c['name'] == name][0]
        assert cfg['name'] == name
        assert (cfg['parent'], cfg['child']) == (j['parent'], j['child'])
        assert list(map(float, cfg['axis'])) == pytest.approx(list(j['axis']), abs=1e-12)
        assert (cfg['lower'], cfg['upper']) == pytest.approx((lo, hi), abs=1e-12)


def test_axes_and_analytic_preconditions(geom):
    assert geom.axes[0] == pytest.approx((0, 1, 0), abs=1e-12)    # lf_hip  [0 1 0]
    assert geom.axes[1] == pytest.approx((-1, 0, 0), abs=1e-12)   # lf_thigh_joint [-1 0 0]
    assert geom.axes[2] == pytest.approx((1, 0, 0), abs=1e-12)    # lf_foot_joint  [1 0 0]
    assert geom.structure_problems == []


def test_axis_points_equal_urdf_origin_sums(geom):
    H, T, K = geom.axis_points
    assert H == pytest.approx((0.010, 0.0161, 0.07725), abs=1e-12)
    assert T == pytest.approx((-0.0237, 0.03885, 0.08875), abs=1e-12)
    assert K == pytest.approx((-0.0367, -0.038932, 0.010968), abs=1e-12)


def test_tip_matches_independent_mesh_derivation(geom):
    assert np.allclose(geom.tip0, reference_tip_from_mesh(), atol=1e-9)
    assert geom.tip0[2] == pytest.approx(-0.054543, abs=1e-6)     # audit: feet at z = -0.0545


def test_mismatched_legs_config_is_rejected(urdf, legs):
    bad = copy.deepcopy(legs)
    j = bad['legs']['front_left']['joints']
    j['hip']['name'], j['thigh']['name'] = j['thigh']['name'], j['hip']['name']
    with pytest.raises(lk.KinematicsError, match='differ'):
        lk.LegGeometry.from_urdf(urdf, bad, 'front_left')


def test_broken_structure_disables_ik(urdf, legs):
    mutated = copy.deepcopy(urdf)
    j = [x for x in mutated.findall('joint') if x.get('name') == 'lf_foot_joint'][0]
    j.find('axis').set('xyz', '1 0.2 0')          # knee no longer parallel to thigh
    g = lk.LegGeometry.from_urdf(mutated, legs, 'lf')
    assert g.structure_problems
    r = lk.inverse(g, g.tip0)
    assert not r['ok'] and r['reason'] == 'analytic_preconditions'


def test_urdf_origin_change_changes_fk(urdf, legs, geom):
    mutated = copy.deepcopy(urdf)
    j = [x for x in mutated.findall('joint') if x.get('name') == 'lf_foot_joint'][0]
    j.find('origin').set('xyz', '-0.012 -0.087782 -0.077782')   # 1 cm longer thigh
    g = lk.LegGeometry.from_urdf(mutated, legs, 'lf')
    assert g.axis_points[2][1] == pytest.approx(geom.axis_points[2][1] - 0.01, abs=1e-12)


@pytest.mark.parametrize('leg', ['rf', 'front_right', 'rr', 'left_wheel', '', None])
def test_unknown_or_unsupported_leg_is_rejected(urdf, legs, leg):
    with pytest.raises(lk.KinematicsError, match='unsupported leg'):
        lk.LegGeometry.from_urdf(urdf, legs, leg)


# ------------------------------------------------------------ forward kinematics
def test_fk_cad_neutral(geom):
    fk = lk.forward(geom, [0.0, 0.0, 0.0])
    assert fk['foot_link_position'] == pytest.approx((-0.0367, -0.038932, 0.010968), abs=1e-12)
    assert fk['foot_link_quaternion'] == pytest.approx((0, 0, 0, 1), abs=1e-12)
    assert fk['tip_position'] == pytest.approx(geom.tip0, abs=1e-15)
    assert fk['frame'] == 'base_link' and fk['joint_names'] == LF


def test_fk_matches_runtime_observation(geom):
    """Regression anchor: TF and Gazebo both reported this pose with lf_thigh_joint = 0.2."""
    fk = lk.forward(geom, [0.0, 0.2, 0.0])
    assert fk['foot_link_position'] == pytest.approx((-0.0367, -0.052834, 0.027971), abs=1e-6)
    assert fk['foot_link_quaternion'] == pytest.approx((-0.099833, 0, 0, 0.995004), abs=1e-6)


def test_fk_matches_independent_urdf_walk(urdf, geom):
    rng = random.Random(7)
    for _ in range(300):
        q = _rand_q(geom, rng, margin=0.0)
        fk = lk.forward(geom, q)
        pos, quat = reference_walk(urdf, q)
        assert np.allclose(fk['foot_link_position'], pos, atol=1e-12)
        w, x, y, z = quat / np.linalg.norm(quat)
        ref_q = np.array([x, y, z, w]) * (1 if w >= 0 else -1)
        assert np.allclose(fk['foot_link_quaternion'], ref_q, atol=1e-9)
        tip = pos + _qrot(quat, np.array(geom.tip_local))
        assert np.allclose(fk['tip_position'], tip, atol=1e-12)


def test_transform_chain_equals_product_of_exponentials(geom):
    rng = random.Random(3)
    for _ in range(300):
        q = _rand_q(geom, rng, margin=0.0)
        assert lk.forward(geom, q)['tip_position'] == pytest.approx(
            lk.forward_tip_poe(geom, q), abs=1e-12)


def test_fk_is_repeatable(geom):
    q = [0.1, -0.2, 0.3]
    assert lk.forward(geom, q) == lk.forward(geom, list(q)) == lk.forward(geom, tuple(q))


@pytest.mark.parametrize('q', [[0.0, 0.0], [0.0] * 4, [0.0, float('nan'), 0.0],
                               [0.0, float('inf'), 0.0], ['0', 0.0, 0.0], [True, 0.0, 0.0],
                               None, 'abc', {'lf_hip': 0.0}, 0.3])
def test_fk_rejects_malformed_input(geom, q):
    with pytest.raises(lk.KinematicsError):
        lk.forward(geom, q)


def test_sign_conventions(geom):
    tip0 = np.array(geom.tip0)
    hip = np.array(lk.forward(geom, [0.2, 0, 0])['tip_position']) - tip0
    thigh = np.array(lk.forward(geom, [0, 0.2, 0])['tip_position']) - tip0
    foot = np.array(lk.forward(geom, [0, 0, 0.2])['tip_position']) - tip0
    assert hip[0] < -0.01           # +lf_hip: tip outward (-x) for the left leg
    assert thigh[1] < -0.02         # +lf_thigh_joint: tip rearward (-y)
    assert foot[1] > 0.01 and foot[2] > 0.01   # +lf_foot_joint: tip forward (+y) and up


# ------------------------------------------------------------ inverse kinematics
def test_ik_cad_neutral_returns_cad_neutral(geom):
    r = lk.inverse(geom, geom.tip0)
    assert r['ok'] and r['solution'] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert r['joint_names'] == LF and r['frame'] == 'base_link'
    assert len(r['candidates']) == 4 and sum(c['valid'] for c in r['candidates']) == 1


def test_ik_round_trip_many_configurations(geom):
    rng = random.Random(11)
    for _ in range(500):
        q = _rand_q(geom, rng)
        target = lk.forward(geom, q)['tip_position']
        r = lk.inverse(geom, target, reference=q)
        assert r['ok'], r['message']
        assert r['solution'] == pytest.approx(q, abs=1e-9)
        assert math.dist(lk.forward(geom, r['solution'])['tip_position'], target) <= \
            lk.IK_POSITION_TOL_M
        # default reference (CAD neutral) must also return a valid, target-reproducing solution
        r0 = lk.inverse(geom, target)
        assert r0['ok'] and not lk.check_limits(geom, r0['solution'])
        assert math.dist(lk.forward(geom, r0['solution'])['tip_position'], target) <= 1e-6


def test_ik_selects_nearest_valid_candidate(geom):
    rng = random.Random(5)
    for _ in range(100):
        target = lk.forward(geom, _rand_q(geom, rng))['tip_position']
        ref = _rand_q(geom, rng)
        r = lk.inverse(geom, target, reference=ref)
        valid = [c for c in r['candidates'] if c['valid']]
        best = min(c['distance_to_reference_rad'] for c in valid)
        assert r['candidates'][r['selected_index']]['distance_to_reference_rad'] == best


def test_ik_solutions_respect_limits_and_margin(geom):
    rng = random.Random(13)
    for _ in range(200):
        r = lk.inverse(geom, lk.forward(geom, _rand_q(geom, rng))['tip_position'])
        for (lo, hi), v in zip(geom.limits, r['solution']):
            assert lo + 0.05 <= v <= hi - 0.05


def test_unreachable_target_is_rejected(geom):
    far = (geom.tip0[0], geom.tip0[1], geom.tip0[2] - 0.30)
    r = lk.inverse(geom, far)
    assert not r['ok'] and r['reason'] == 'unreachable_knee' and r['solution'] is None


def test_target_too_close_to_hip_axis_is_rejected(geom):
    H = geom.axis_points[0]
    w = geom.axes[1]
    r = lk.inverse(geom, tuple(h + 0.01 * c for h, c in zip(H, w)))
    assert not r['ok'] and r['reason'] == 'unreachable_hip'


def test_target_on_hip_axis_is_singular(geom):
    H, u = geom.axis_points[0], geom.axes[0]
    r = lk.inverse(geom, tuple(h + 0.05 * c for h, c in zip(H, u)))
    assert not r['ok'] and r['reason'] == 'singular'


def test_out_of_limit_target_is_refused_not_clamped(geom):
    """Negative test: reachable only outside the limits -> failure, never a clamped 'solution'."""
    q_bad = [0.0, 1.2, 0.0]                        # lf_thigh_joint 1.2 > URDF upper 0.785398
    r = lk.inverse(geom, lk.forward(geom, q_bad)['tip_position'])
    assert not r['ok'] and r['reason'] == 'joint_limits' and r['solution'] is None
    exact = [c for c in r['candidates'] if c['q'] == pytest.approx(q_bad, abs=1e-9)]
    assert exact and exact[0]['limit_violations'] and not exact[0]['valid']
    for c in r['candidates']:                      # every candidate still reproduces the target
        assert c['fk_residual_m'] <= lk.IK_POSITION_TOL_M


def test_margin_is_enforced(geom):
    """lf_foot_joint 0.41 is inside the URDF limit (0.436) but inside the 0.05 margin."""
    target = lk.forward(geom, [0.0, 0.0, 0.41])['tip_position']
    assert lk.inverse(geom, target)['reason'] == 'joint_limits'
    r = lk.inverse(geom, target, margin=0.0)
    assert r['ok'] and r['solution'] == pytest.approx([0.0, 0.0, 0.41], abs=1e-9)


@pytest.mark.parametrize('target', [[0.0, 0.0], [0.0, float('nan'), 0.0], 'xyz', None,
                                    [0.0, 0.0, float('-inf')], [0.0, 0.0, '1']])
def test_ik_rejects_malformed_target(geom, target):
    r = lk.inverse(geom, target)
    assert not r['ok'] and r['reason'] == 'invalid_input' and r['solution'] is None


@pytest.mark.parametrize('frame', ['world', 'odom', 'dummy_link', 'lf_foot_1', ''])
def test_ik_rejects_other_frames(geom, frame):
    r = lk.inverse(geom, geom.tip0, frame=frame)
    assert not r['ok'] and r['reason'] == 'invalid_input'


def test_ik_rejects_bad_reference_and_margin(geom):
    assert lk.inverse(geom, geom.tip0, reference=[0.0, 0.0])['reason'] == 'invalid_input'
    assert lk.inverse(geom, geom.tip0, margin=-0.1)['reason'] == 'invalid_input'


@pytest.mark.parametrize('angle', [1e-9, 1e-6, 0.3, 3.0])
def test_rotation_error_is_accurate_for_small_angles(angle):
    r = lk.rot_axis_angle((0.3, -1.0, 0.2), angle)
    assert lk.rotation_angle_between(lk.IDENTITY, r) == pytest.approx(angle, rel=1e-6)
