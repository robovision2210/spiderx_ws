"""colcon test: M4 all-leg forward/inverse kinematics (opt-in) against the expanded URDF.

Reference implementation: an INDEPENDENT numpy URDF walk (quaternions, own XML/STL parsing) for
every leg. M3's front_left-only default is checked to be unchanged.
"""
import math
import os
import random

import numpy as np
import pytest
import yaml

from spiderx_controller import leg_kinematics as lk
from spiderx_controller.config_check import load_urdf

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_CONFIG = os.path.join(HERE, '..', 'config')
MESHES = os.path.join(HERE, '..', '..', 'spiderx_description', 'meshes')
PREFIX = {'front_left': 'lf', 'front_right': 'rf', 'rear_left': 'lr', 'rear_right': 'rr'}


@pytest.fixture(scope='module')
def urdf():
    return load_urdf()


@pytest.fixture(scope='module')
def legs_cfg():
    with open(os.path.join(SRC_CONFIG, 'spiderx_legs.yaml')) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def geoms(urdf, legs_cfg):
    return lk.load_all_geometries(urdf, legs_cfg)


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


def _floats(s):
    return [float(v) for v in s.split()]


def reference_walk(urdf, leg, q):
    """(position, quaternion wxyz) of {prefix}_foot_1 in base_link from the raw URDF XML."""
    pre = PREFIX[leg]
    names = [f'{pre}_hip', f'{pre}_thigh_joint', f'{pre}_foot_joint']
    joints = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    chain, link = [], f'{pre}_foot_1'
    while link != 'base_link':
        chain.append(joints[link])
        link = joints[link].find('parent').get('link')
    pos, rot = np.zeros(3), np.array([1.0, 0, 0, 0])
    angles = dict(zip(names, q))
    for j in reversed(chain):
        o = j.find('origin')
        pos = pos + _qrot(rot, np.array(_floats(o.get('xyz'))))
        rot = _qmul(rot, _rpy_quat(*_floats(o.get('rpy'))))
        if j.get('type') == 'revolute':
            rot = _qmul(rot, _quat(_floats(j.find('axis').get('xyz')), angles[j.get('name')]))
    return pos, rot


def reference_tip(leg):
    data = open(os.path.join(MESHES, f'{PREFIX[leg]}_foot_1.stl'), 'rb').read()
    n = int(np.frombuffer(data[80:84], '<u4')[0])
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=[('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    v = rec['v'].reshape(-1, 3).astype(float) * 0.001    # CAD coordinates = base_link at q = 0
    zmin = v[:, 2].min()
    low = np.unique(np.round(v[v[:, 2] <= zmin + 1e-4], 9), axis=0)
    return np.array([low[:, 0].mean(), low[:, 1].mean(), zmin])


def _rand_q(g, rng, margin=0.05):
    return [rng.uniform(lo + margin, hi - margin) for lo, hi in g.limits]


# ------------------------------------------------------------ M3 default is unchanged
@pytest.mark.parametrize('leg', ['rf', 'front_right', 'lr', 'rear_left', 'rr', 'rear_right'])
def test_m3_default_still_rejects_other_legs(urdf, legs_cfg, leg):
    with pytest.raises(lk.KinematicsError, match='unsupported leg'):
        lk.resolve_leg(leg)
    with pytest.raises(lk.KinematicsError, match='unsupported leg'):
        lk.LegGeometry.from_urdf(urdf, legs_cfg, leg)
    assert lk.resolve_leg(leg, allow_all_legs=True) in lk.ALL_LEGS


def test_m3_default_front_left_unchanged(urdf, legs_cfg, geoms):
    m3 = lk.LegGeometry.from_urdf(urdf, legs_cfg, 'lf')          # M3 call, no flag
    assert m3.summary() == geoms['front_left'].summary()
    assert lk.SUPPORTED_LEGS == ('front_left',)


@pytest.mark.parametrize('leg', ['left_wheel', 'lf ', '', None, 'LF', 'front'])
def test_unknown_leg_rejected_even_with_opt_in(leg):
    with pytest.raises(lk.KinematicsError, match='unknown leg'):
        lk.resolve_leg(leg, allow_all_legs=True)


# ------------------------------------------------------------ geometry per leg
def test_joint_names_and_controller_order(geoms):
    for leg, pre in PREFIX.items():
        assert list(geoms[leg].joint_names) == [f'{pre}_hip', f'{pre}_thigh_joint',
                                                f'{pre}_foot_joint']
    with open(os.path.join(SRC_CONFIG, 'spiderx_ros2_controllers.yaml')) as f:
        ctrl = yaml.safe_load(f)['leg_trajectory_controller']['ros__parameters']['joints']
    assert lk.all_joint_names(geoms) == ctrl        # same 12-joint order as the M1 controller


@pytest.mark.parametrize('leg,hip,thigh,knee,knee_dot_thigh', [
    ('front_left', (0, 1, 0), (-1, 0, 0), (1, 0, 0), -1.0),
    ('front_right', (0, 1, 0), (1, 0, 0), (-1, 0, 0), -1.0),
    ('rear_left', (0, -1, 0), (-1, 0, 0), (-1, 0, 0), 1.0),     # the odd one
    ('rear_right', (0, -1, 0), (1, 0, 0), (-1, 0, 0), -1.0),
])
def test_axes_sign_table_and_preconditions(geoms, leg, hip, thigh, knee, knee_dot_thigh):
    g = geoms[leg]
    assert g.axes[0] == pytest.approx(hip, abs=1e-12)
    assert g.axes[1] == pytest.approx(thigh, abs=1e-12)
    assert g.axes[2] == pytest.approx(knee, abs=1e-12)
    assert g.structure['knee_sign_vs_thigh'] == knee_dot_thigh
    assert g.structure_problems == []           # hip ⟂ thigh, thigh ∥ knee on every leg


@pytest.mark.parametrize('leg', lk.ALL_LEGS)
def test_axis_points_equal_independent_walk(urdf, geoms, leg):
    """H, T, K at q = 0 = origins of the hip, thigh and knee child frames (independent walk)."""
    joints = {j.get('name'): j for j in urdf.findall('joint')}
    by_child = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    for name, point in zip(geoms[leg].joint_names, geoms[leg].axis_points):
        p, link = np.zeros(3), joints[name].find('child').get('link')
        while link != 'base_link':
            p += np.array(_floats(by_child[link].find('origin').get('xyz')))
            link = by_child[link].find('parent').get('link')
        assert np.allclose(point, p, atol=1e-12)


@pytest.mark.parametrize('leg', lk.ALL_LEGS)
def test_tip_matches_independent_mesh_derivation(geoms, leg):
    assert np.allclose(geoms[leg].tip0, reference_tip(leg), atol=1e-9)


def test_tips_coplanar_and_mirror_symmetric(geoms):
    """Measured facts about the current CAD (not assumptions used by the code)."""
    t = {leg: geoms[leg].tip0 for leg in lk.ALL_LEGS}
    zs = [p[2] for p in t.values()]
    assert max(zs) - min(zs) < 1e-12 and zs[0] == pytest.approx(-0.054543, abs=1e-6)
    cx = (t['front_left'][0] + t['front_right'][0]) / 2
    assert (t['rear_left'][0] + t['rear_right'][0]) / 2 == pytest.approx(cx, abs=1e-9)
    assert t['front_left'][1] == pytest.approx(t['front_right'][1], abs=1e-9)


def test_front_and_rear_legs_are_not_mirror_copies(geoms):
    """Why every leg is read from the URDF: thigh offsets differ front vs rear."""
    front = np.subtract(geoms['front_left'].axis_points[1], geoms['front_left'].axis_points[0])
    rear = np.subtract(geoms['rear_left'].axis_points[1], geoms['rear_left'].axis_points[0])
    assert front[1] == pytest.approx(0.02275, abs=1e-9)
    assert rear[1] == pytest.approx(-0.04325, abs=1e-9)


# ------------------------------------------------------------ forward kinematics
@pytest.mark.parametrize('leg', lk.ALL_LEGS)
def test_fk_matches_independent_urdf_walk(urdf, geoms, leg):
    g, rng = geoms[leg], random.Random(lk.ALL_LEGS.index(leg))
    for _ in range(200):
        q = _rand_q(g, rng, margin=0.0)
        fk = lk.forward(g, q)
        pos, quat = reference_walk(urdf, leg, q)
        assert np.allclose(fk['foot_link_position'], pos, atol=1e-12)
        w, x, y, z = quat / np.linalg.norm(quat)
        assert np.allclose(fk['foot_link_quaternion'], np.array([x, y, z, w]) * np.sign(w),
                           atol=1e-9)
        assert np.allclose(fk['tip_position'], pos + _qrot(quat, np.array(g.tip_local)),
                           atol=1e-12)


@pytest.mark.parametrize('leg', lk.ALL_LEGS)
def test_fk_chain_equals_poe(geoms, leg):
    g, rng = geoms[leg], random.Random(3)
    for _ in range(200):
        q = _rand_q(g, rng, margin=0.0)
        assert lk.forward(g, q)['tip_position'] == pytest.approx(lk.forward_tip_poe(g, q),
                                                                 abs=1e-12)


def test_forward_all_equals_per_leg_fk(geoms):
    rng = random.Random(9)
    q12 = [v for leg in lk.ALL_LEGS for v in _rand_q(geoms[leg], rng)]
    out = lk.forward_all(geoms, q12)
    for i, leg in enumerate(lk.ALL_LEGS):
        assert out[leg] == lk.forward(geoms[leg], q12[3 * i:3 * i + 3])


@pytest.mark.parametrize('q12', [[0.0] * 11, [0.0] * 13, [0.0] * 11 + [float('nan')], None,
                                 'zeros', {'lf_hip': 0.0}])
def test_forward_all_rejects_malformed(geoms, q12):
    with pytest.raises(lk.KinematicsError):
        lk.forward_all(geoms, q12)


# ------------------------------------------------------------ sign conventions
def test_crouch_sign_pattern_per_leg(geoms):
    """The SAME physical motion (every foot 10 mm up toward the body) needs different signs."""
    expected = {'front_left': (1, 1), 'front_right': (-1, -1),
                'rear_left': (1, -1), 'rear_right': (-1, -1)}
    targets = {leg: tuple(a + b for a, b in zip(geoms[leg].tip0, (0, 0, 0.010)))
               for leg in lk.ALL_LEGS}
    r = lk.inverse_all(geoms, targets)
    assert r['ok']
    for leg, (thigh_sign, knee_sign) in expected.items():
        hip, thigh, knee = r['legs'][leg]['solution']
        assert abs(hip) < 1e-9
        assert math.copysign(1, thigh) == thigh_sign and abs(thigh) == pytest.approx(0.0555,
                                                                                    abs=1e-4)
        assert math.copysign(1, knee) == knee_sign and abs(knee) == pytest.approx(0.1223,
                                                                                  abs=1e-4)


def test_hip_outward_sign_per_leg(geoms):
    """+0.2 rad on each hip: left legs have outward = -x, right legs outward = +x."""
    dx = {leg: lk.forward(geoms[leg], [0.2, 0, 0])['tip_position'][0] - geoms[leg].tip0[0]
          for leg in lk.ALL_LEGS}
    assert dx['front_left'] < -0.01      # +y axis, left leg: outward
    assert dx['front_right'] < -0.01     # +y axis, right leg: inward (outward needs -0.2)
    assert dx['rear_left'] > 0.01        # -y axis, left leg: inward
    assert dx['rear_right'] > 0.01       # -y axis, right leg: outward


def test_physical_joint_ranges_are_symmetric(geoms):
    """Limits flip together with the axis sign, so the physical range about +x / +y matches."""
    def physical(g, i):
        axis = g.axes[i]
        s = axis[0] + axis[1] + axis[2]          # each axis is +/- a unit basis vector
        lo, hi = g.limits[i]
        return tuple(sorted((s * lo, s * hi)))
    left_hip = physical(geoms['front_left'], 0)
    assert physical(geoms['rear_left'], 0) == pytest.approx(left_hip)
    right_hip = physical(geoms['front_right'], 0)
    assert physical(geoms['rear_right'], 0) == pytest.approx(right_hip)
    assert right_hip == pytest.approx(tuple(sorted((-left_hip[0], -left_hip[1]))))
    for i in (1, 2):                             # thigh and knee: identical on all four legs
        ranges = [physical(geoms[leg], i) for leg in lk.ALL_LEGS]
        assert all(r == pytest.approx(ranges[0]) for r in ranges)


# ------------------------------------------------------------ inverse kinematics
@pytest.mark.parametrize('leg', lk.ALL_LEGS)
def test_ik_round_trip_per_leg(geoms, leg):
    g, rng = geoms[leg], random.Random(11)
    for _ in range(300):
        q = _rand_q(g, rng)
        target = lk.forward(g, q)['tip_position']
        r = lk.inverse(g, target, reference=q)
        assert r['ok'], r['message']
        assert r['solution'] == pytest.approx(q, abs=1e-9)
        r0 = lk.inverse(g, target)                       # default reference: CAD neutral
        assert r0['ok'] and not lk.check_limits(g, r0['solution'])
        assert math.dist(lk.forward(g, r0['solution'])['tip_position'], target) <= \
            lk.IK_POSITION_TOL_M


def test_inverse_all_round_trip(geoms):
    rng = random.Random(21)
    for _ in range(200):
        q12 = [v for leg in lk.ALL_LEGS for v in _rand_q(geoms[leg], rng)]
        fk = lk.forward_all(geoms, q12)
        targets = {leg: fk[leg]['tip_position'] for leg in lk.ALL_LEGS}
        r = lk.inverse_all(geoms, targets, reference=q12)
        assert r['ok'] and r['solution'] == pytest.approx(q12, abs=1e-9)
        assert r['joint_names'] == lk.all_joint_names(geoms)


def test_inverse_all_cad_neutral_is_all_zeros(geoms):
    r = lk.inverse_all(geoms, {leg: geoms[leg].tip0 for leg in lk.ALL_LEGS})
    assert r['ok'] and r['solution'] == pytest.approx([0.0] * 12, abs=1e-9)
    for leg in lk.ALL_LEGS:                          # exactly one valid candidate per leg
        assert sum(c['valid'] for c in r['legs'][leg]['candidates']) == 1


# ------------------------------------------------------------ limits and refusals
@pytest.mark.parametrize('leg', lk.ALL_LEGS)
def test_margin_enforced_per_leg(geoms, leg):
    """Knee 0.02 rad inside its URDF limit: allowed with margin 0, refused with 0.05."""
    g = geoms[leg]
    lo, hi = g.limits[2]
    q = [0.0, 0.0, hi - 0.02 if hi < abs(lo) else lo + 0.02]   # the smaller (forward/up) side
    target = lk.forward(g, q)['tip_position']
    refused = lk.inverse(g, target)
    assert not refused['ok'] and refused['reason'] == 'joint_limits'
    assert refused['solution'] is None
    ok = lk.inverse(g, target, margin=0.0)
    assert ok['ok'] and ok['solution'] == pytest.approx(q, abs=1e-9)


@pytest.mark.parametrize('leg', lk.ALL_LEGS)
def test_out_of_limit_target_refused_not_clamped(geoms, leg):
    g = geoms[leg]
    lo, hi = g.limits[1]
    q_bad = [0.0, hi + 0.4, 0.0]                     # thigh 0.4 rad beyond its URDF limit
    r = lk.inverse(g, lk.forward(g, q_bad)['tip_position'])
    assert not r['ok'] and r['reason'] == 'joint_limits' and r['solution'] is None
    exact = [c for c in r['candidates'] if c['q'] == pytest.approx(q_bad, abs=1e-9)]
    assert exact and exact[0]['limit_violations'] and not exact[0]['valid']


@pytest.mark.parametrize('leg', lk.ALL_LEGS)
def test_unreachable_target_rejected(geoms, leg):
    g = geoms[leg]
    r = lk.inverse(g, (g.tip0[0], g.tip0[1], g.tip0[2] - 0.30))
    assert not r['ok'] and r['reason'] == 'unreachable_knee' and r['solution'] is None


def test_inverse_all_never_returns_a_partial_solution(geoms):
    targets = {leg: geoms[leg].tip0 for leg in lk.ALL_LEGS}
    rr = geoms['rear_right'].tip0
    targets['rear_right'] = (rr[0], rr[1], rr[2] - 0.30)          # unreachable
    r = lk.inverse_all(geoms, targets)
    assert not r['ok'] and r['reason'] == 'leg_failed' and r['solution'] is None
    assert r['legs']['front_left']['ok'] and not r['legs']['rear_right']['ok']
    assert 'rear_right' in r['message']


@pytest.mark.parametrize('targets,match', [
    ('not a dict', 'mapping'),
    ({'front_left': (0, 0, 0)}, 'missing legs'),
    ({'lf': (0, 0, 0), 'front_left': (0, 0, 0), 'rf': (0, 0, 0), 'lr': (0, 0, 0),
      'rr': (0, 0, 0)}, 'given twice'),
    ({'lf': (0, 0, 0), 'rf': (0, 0, 0), 'lr': (0, 0, 0), 'left_wheel': (0, 0, 0)}, 'unknown leg'),
])
def test_inverse_all_rejects_malformed_targets(geoms, targets, match):
    r = lk.inverse_all(geoms, targets)
    assert not r['ok'] and r['reason'] == 'invalid_input' and match in r['message']


def test_inverse_all_rejects_bad_reference_and_nan_target(geoms):
    good = {leg: geoms[leg].tip0 for leg in lk.ALL_LEGS}
    assert lk.inverse_all(geoms, good, reference=[0.0] * 11)['reason'] == 'invalid_input'
    bad = dict(good, front_right=(0.0, float('nan'), 0.0))
    r = lk.inverse_all(geoms, bad)
    assert not r['ok'] and r['legs']['front_right']['reason'] == 'invalid_input'
