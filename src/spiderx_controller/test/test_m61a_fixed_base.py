"""M6.1-A fixed base: frames, weld identity, the converter rule, observations and readiness.

Offline only (no Gazebo, no ROS graph). The converter checks run `ign sdf -p` (sdformat) on the
real wrapper expansion and are skipped, with the reason, where `ign` is not installed.
"""
import math
import os
import random
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import yaml

from spiderx_controller import config_check as cc
from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m61a_fixed_base as fb
from spiderx_controller import posture_metrics as pm

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
WRAPPER = os.path.join(get_package_share_directory('spiderx_description'), 'urdf',
                       'spiderx_fixed_base.urdf.xacro')
PLAIN = os.path.join(get_package_share_directory('spiderx_description'), 'urdf',
                     'spiderx.urdf.xacro')
CONTROLLERS = os.path.join(get_package_share_directory('spiderx_controller'), 'config',
                           'spiderx_ros2_controllers.yaml')
IGN = shutil.which('ign')


def xacro(path, *args):
    return subprocess.run(['xacro', path, *args], check=True, capture_output=True,
                          text=True).stdout


def fortress_args(control):
    return (['sim_backend:=fortress', 'enable_control:=true', f'controllers_file:={CONTROLLERS}']
            if control else ['sim_backend:=fortress'])


@pytest.fixture(scope='module')
def cfg():
    return fb.load_config(CONFIG_DIR)


# ==================================================================== rigid transforms
def matrix(p):
    R = np.array(lk.rot_rpy(*p.rpy()))
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p.p
    return T


def rand_pose(rng):
    return fb.Pose.from_xyz_rpy(*(rng.uniform(-0.5, 0.5) for _ in range(3)),
                                rng.uniform(-math.pi, math.pi), rng.uniform(-1.4, 1.4),
                                rng.uniform(-math.pi, math.pi))


def test_composition_matches_independent_matrices_for_general_rotations():
    """T_a_c = T_a_b * T_b_c, checked against 4x4 matrices built from leg_kinematics.rot_rpy."""
    rng = random.Random(61)
    for _ in range(200):
        a, b = rand_pose(rng), rand_pose(rng)
        assert np.allclose(matrix(a * b), matrix(a) @ matrix(b), atol=1e-12)
        assert np.allclose(matrix(a.inverse()), np.linalg.inv(matrix(a)), atol=1e-12)
        v = tuple(rng.uniform(-1, 1) for _ in range(3))
        assert np.allclose(a.apply(v), (matrix(a) @ np.array(v + (1.0,)))[:3], atol=1e-12)


def test_composition_order_is_not_a_height_addition():
    """A yawed, translated mount composed with an offset body frame is not mount + offset."""
    mount = fb.Pose.from_xyz_rpy(0.31, -0.17, 0.43, 0, 0, 0.7)
    off = fb.Pose.from_xyz_rpy(0.051, -0.045, 0.141, 0, 0, math.pi / 2)
    c = mount * off
    assert c.p == pytest.approx((0.31 + math.cos(0.7) * 0.051 + math.sin(0.7) * 0.045,
                                 -0.17 + math.sin(0.7) * 0.051 - math.cos(0.7) * 0.045,
                                 0.43 + 0.141), abs=1e-12)
    assert c.rpy()[2] == pytest.approx(0.7 + math.pi / 2)
    assert (off * mount).p != pytest.approx(c.p)                    # order matters


def test_euler_and_tilt_conventions_match_posture_metrics():
    rng = random.Random(7)
    for _ in range(100):
        r, p, y = rng.uniform(-3, 3), rng.uniform(-1.5, 1.5), rng.uniform(-3, 3)
        q = fb.quat_from_rpy(r, p, y)
        assert fb.quat_to_rpy(q) == pytest.approx(pm.quat_to_rpy(*q), abs=1e-12)
        assert fb.Pose((0, 0, 0), q).tilt() == pytest.approx(pm.tilt_from_quat(*q), abs=1e-12)
        assert np.allclose(matrix(fb.Pose((0, 0, 0), q))[:3, :3], np.array(lk.rot_rpy(r, p, y)),
                           atol=1e-12)


def test_deviation_and_rotation_angle():
    a = fb.Pose.from_xyz_rpy(0.0, 0.0, 0.125)
    b = fb.Pose.from_xyz_rpy(0.003, -0.004, 0.125, 0.0, 0.02, 0.0)
    t, r, dz, dxy = fb.deviation(b, a)
    assert t == pytest.approx(0.005) and dxy == pytest.approx(0.005) and dz == pytest.approx(0)
    assert r == pytest.approx(0.02)
    q = fb.quat_from_rpy(0.1, 0, 0)
    assert fb.rotation_angle(q, tuple(-c for c in q)) == pytest.approx(0.0, abs=1e-7)


# ==================================================================== configuration
def test_config_loads_and_equals_the_wrapper_defaults(cfg):
    root = ET.parse(WRAPPER).getroot()
    xa = '{http://www.ros.org/wiki/xacro}arg'
    defaults = {a.get('name'): float(a.get('default')) for a in root.findall(xa)}
    assert defaults == {'mount_x': cfg.mount.p[0], 'mount_y': cfg.mount.p[1],
                        'mount_z': cfg.mount.p[2], 'mount_yaw': cfg.mount.rpy()[2]}
    assert cfg.mount.p[2] >= cfg.min_mount_height_m
    assert cfg.raw['status'] == 'provisional' and cfg.raw['simulation_only'] is True
    assert (cfg.model_name, cfg.body_link, cfg.body_frame, cfg.weld_joint) == (
        'spiderx', 'dummy_link', 'base_link', 'spiderx_fixed_base_weld')
    assert cfg.pose_topic == pm.POSE_TOPIC


def test_config_freshness_equals_the_existing_monitor_values(cfg):
    from spiderx_controller import m6_live_contract as m6d
    from spiderx_controller import m61_live_contract as c61
    assert cfg.pose_stale_s == c61.APPROVED_LIMITS['body_pose_stale_s']
    assert cfg.joint_states_stale_s == m6d.JOINT_STATES_STALE_S
    assert cfg.sim_stall_s == m6d.SIM_STALL_S


def _raw():
    with open(os.path.join(CONFIG_DIR, fb.CONFIG_FILE)) as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize('mutate, needle', [
    (lambda d: d.update(extra=1), 'unknown keys'),
    (lambda d: d.pop('attachment'), 'missing keys'),
    (lambda d: d.update(simulation_only=False), 'simulation_only'),
    (lambda d: d['mount'].update(z_m=float('nan')), 'finite'),
    (lambda d: d['mount'].update(roll_rad=0.1), 'roll and pitch'),
    (lambda d: d['mount'].update(z_m=0.10), 'below'),
    (lambda d: d['attachment'].update(translation_tolerance_m=0.0), '> 0'),
    (lambda d: d['attachment'].update(debounce_samples=True), 'debounce'),
    (lambda d: d.update(schema='x/0'), 'schema'),
])
def test_config_refuses_malformed_values(mutate, needle):
    d = _raw()
    mutate(d)
    with pytest.raises(fb.FixedBaseError, match=needle):
        fb.parse_config(d)


# ==================================================================== the wrapper description
@pytest.fixture(scope='module')
def expansions():
    out = {}
    for control in (False, True):
        out[('wrapper', control)] = xacro(WRAPPER, *fortress_args(control))
        out[('plain', control)] = xacro(PLAIN, *fortress_args(control))
    return out


@pytest.mark.parametrize('control', [False, True])
def test_wrapper_is_the_plain_model_plus_world_and_one_weld(expansions, control, cfg):
    wrap = ET.fromstring(expansions[('wrapper', control)])
    plain = ET.fromstring(expansions[('plain', control)])
    assert fb.same_model_except_weld(wrap, plain)
    weld = fb.parse_robot_description(wrap)
    assert weld.fixed_base and weld.weld_joint == 'spiderx_fixed_base_weld'
    assert fb.check_mount(weld, cfg) == []
    assert fb.deviation(weld.model_to_body, cfg.mount)[:2] == (0.0, 0.0)
    assert fb.parse_robot_description(plain).fixed_base is False
    roots = ({lk.get('name') for lk in wrap.findall('link')}
             - {j.find('child').get('link') for j in wrap.findall('joint')})
    assert roots == {'world'}


@pytest.mark.parametrize('control', [False, True])
def test_wrapper_passes_the_existing_configuration_checks(expansions, control):
    legs, poses, ctrl = cc.load_configs()
    errors, _ = cc.check(ET.fromstring(expansions[('wrapper', control)]), legs, poses, ctrl)
    assert errors == []
    if control:
        errors, _ = cc.check_ros2_control(ET.fromstring(expansions[('wrapper', True)]), legs,
                                          ET.fromstring(expansions[('wrapper', False)]))
        assert errors == []


def test_wrapper_check_urdf(expansions, tmp_path):
    exe = shutil.which('check_urdf')
    if exe is None:
        pytest.skip('check_urdf (urdfdom) is not installed')
    p = tmp_path / 'w.urdf'
    p.write_text(expansions[('wrapper', True)])
    r = subprocess.run([exe, str(p)], capture_output=True, text=True)
    assert r.returncode == 0 and 'root Link: world has 1 child(ren)' in r.stdout


def test_wrapper_mount_arguments_reach_the_weld():
    text = xacro(WRAPPER, 'sim_backend:=none', 'mount_x:=0.31', 'mount_y:=-0.17',
                 'mount_z:=0.43', 'mount_yaw:=0.7')
    weld = fb.parse_robot_description(text)
    assert weld.weld_origin.xyz_rpy() == pytest.approx((0.31, -0.17, 0.43, 0, 0, 0.7))


def test_free_base_model_and_launches_do_not_contain_the_weld():
    plain = xacro(PLAIN, 'sim_backend:=fortress')
    assert 'spiderx_fixed_base_weld' not in plain and '<link name="world"' not in plain
    bringup = get_package_share_directory('spiderx_bringup')
    desc = get_package_share_directory('spiderx_description')
    for path in (os.path.join(bringup, 'launch', 'fortress.launch.py'),
                 os.path.join(bringup, 'launch', 'fortress_control.launch.py'),
                 os.path.join(bringup, 'launch', 'fortress_posture_hold.launch.py'),
                 os.path.join(desc, 'launch', 'fortress.launch.py')):
        assert 'fixed_base' not in open(path).read(), path


# -------------------------------------------------------------------- negative descriptions
BASE = ('<robot name="spiderx"><link name="dummy_link"/><link name="base_link"/>'
        '<joint name="dummy_joint" type="fixed"><parent link="dummy_link"/>'
        '<child link="base_link"/></joint>{}</robot>')
WELD = ('<joint name="{name}" type="{type}"><parent link="world"/><child link="{child}"/>'
        '<origin xyz="0 0 0.125" rpy="{rpy}"/>{axis}</joint>')


def weld_xml(name='spiderx_fixed_base_weld', type='fixed', child='dummy_link', rpy='0 0 0',
             world='<link name="world"/>', extra=''):
    axis = '<axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/>' \
        if type == 'revolute' else ''
    return BASE.format(world + WELD.format(name=name, type=type, child=child, rpy=rpy,
                                           axis=axis) + extra)


@pytest.mark.parametrize('xml, needle', [
    (weld_xml(world='<link name="world"><inertial/></link>'), 'must be empty'),
    (weld_xml(name='other'), 'expected spiderx_fixed_base_weld'),
    (weld_xml(type='revolute'), 'must be fixed'),
    (weld_xml(child='base_link'), 'must be fixed'),
    (weld_xml(rpy='0.1 0 0'), 'roll/pitch'),
    (weld_xml(rpy='0 -0.2 0'), 'roll/pitch'),
    (weld_xml(extra='<joint name="w2" type="fixed"><parent link="world"/>'
                    '<child link="base_link"/></joint>'), 'exactly one joint from world'),
    (weld_xml(extra='<link name="dummy_link"/>'), 'duplicate'),
    ('<model/>', 'not robot'),
])
def test_malformed_fixed_base_descriptions_are_refused(xml, needle):
    with pytest.raises(fb.FixedBaseError, match=needle):
        fb.parse_robot_description(xml)


def test_mount_checks(cfg):
    low = fb.parse_robot_description(weld_xml().replace('0 0 0.125', '0 0 0.100'))
    assert fb.check_mount(low, cfg) == [fb.MOUNT_TOO_LOW, fb.MOUNT_MISMATCH]
    other = fb.parse_robot_description(weld_xml().replace('0 0 0.125', '0 0 0.150'))
    assert fb.check_mount(other, cfg) == [fb.MOUNT_MISMATCH]
    free = fb.parse_robot_description(BASE.format(''))
    assert fb.check_mount(free, cfg) == [fb.DESCRIPTION_NOT_FIXED_BASE]


# ==================================================================== the converter rule (sdformat)
def ign_convert(urdf_text, tmp_path, name):
    if IGN is None:
        pytest.skip('ign (sdformat command line) is not installed; converter rule not checked')
    p = tmp_path / f'{name}.urdf'
    p.write_text(urdf_text)
    r = subprocess.run([IGN, 'sdf', '-p', str(p)], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0 and r.stderr == '', r.stderr
    return r.stdout


@pytest.mark.parametrize('mount', [(0.0, 0.0, 0.125, 0.0), (0.31, -0.17, 0.43, 0.7)])
def test_converter_places_the_weld_origin_between_model_and_body(tmp_path, mount):
    """T_model_dummy = weld origin and T_model_body = weld * dummy_joint, from `ign sdf -p`."""
    x, y, z, yaw = mount
    text = xacro(WRAPPER, *fortress_args(True), f'mount_x:={x}', f'mount_y:={y}',
                 f'mount_z:={z}', f'mount_yaw:={yaw}')
    frames, facts = fb.resolve_sdf_model(ign_convert(text, tmp_path, 'wrapper'))
    weld = fb.parse_robot_description(text)
    assert facts['model_name'] == 'spiderx' and not facts['model_has_pose']
    assert facts['canonical_link'] == 'dummy_link'
    assert facts['world_joints'] == [{'name': 'spiderx_fixed_base_weld', 'type': 'fixed',
                                      'child': 'dummy_link'}]
    for name, expected in (('dummy_link', weld.model_to_dummy), ('base_link', weld.model_to_body)):
        t, r, _, _ = fb.deviation(frames[name], expected)
        assert t < 1e-9 and r < 1e-6, (name, frames[name], expected)
    # body-relative geometry is unchanged by the weld (hip joint frame relative to base_link)
    plain = fb.resolve_sdf_model(ign_convert(xacro(PLAIN, *fortress_args(True)), tmp_path,
                                             'plain'))[0]
    for name in ('lf_hip', 'rr_foot_joint', 'lidar_joint'):
        a = frames['base_link'].inverse() * frames[name]
        b = plain['base_link'].inverse() * plain[name]
        assert fb.deviation(a, b)[0] < 1e-9 and fb.deviation(a, b)[1] < 1e-6, name


def test_converter_rule_also_holds_for_general_rotation(tmp_path):
    """Not allowed by the wrapper (roll = pitch = 0), but the frame rule is general: a temporary
    description with roll and pitch in the weld converts to T_model_dummy = weld origin too."""
    plain = ET.fromstring(xacro(PLAIN, 'sim_backend:=none'))
    ET.SubElement(plain, 'link', name='world')
    j = ET.SubElement(plain, 'joint', name='w', type='fixed')
    ET.SubElement(j, 'parent', link='world')
    ET.SubElement(j, 'child', link='dummy_link')
    ET.SubElement(j, 'origin', xyz='0.1 0.2 0.3', rpy='0.2 -0.3 0.4')
    frames, facts = fb.resolve_sdf_model(ign_convert(ET.tostring(plain, encoding='unicode'),
                                                     tmp_path, 'general'))
    assert facts['world_joints'][0]['child'] == 'dummy_link'
    t, r, _, _ = fb.deviation(frames['dummy_link'],
                              fb.Pose.from_xyz_rpy(0.1, 0.2, 0.3, 0.2, -0.3, 0.4))
    assert t < 1e-9 and r < 1e-6


# ==================================================================== pose observations
def tf(child, pose, frame_id=''):
    return fb.make_transform(child, pose, frame_id)


@pytest.fixture
def weld(cfg):
    return fb.parse_robot_description(fb.minimal_description(cfg.mount))


def good(weld):
    return [tf('ground_plane', fb.Pose()), tf('spiderx', fb.Pose()),
            tf('dummy_link', weld.model_to_dummy, 'spiderx')]


def test_selection_of_the_model_and_body_link_entries(weld):
    s = fb.select_entries(good(weld))
    assert s.codes == () and s.frame_ids == ('', 'spiderx')
    assert s.model == fb.Pose() and s.body_link == weld.model_to_dummy


@pytest.mark.parametrize('edit, code', [
    (lambda t: [x for x in t if x.child_frame_id != 'spiderx'], fb.POSE_MISSING_MODEL),
    (lambda t: [x for x in t if x.child_frame_id != 'dummy_link'], fb.POSE_MISSING_BODY_LINK),
    (lambda t: t + [tf('spiderx', fb.Pose())], fb.POSE_AMBIGUOUS_MODEL),
    (lambda t: t + [tf('dummy_link', fb.Pose())], fb.POSE_AMBIGUOUS_BODY_LINK),
])
def test_missing_and_ambiguous_entries(weld, edit, code):
    assert code in fb.select_entries(edit(good(weld))).codes


@pytest.mark.parametrize('field, value, code', [
    ('x', float('nan'), fb.POSE_NONFINITE), ('z', float('inf'), fb.POSE_NONFINITE),
    ('qw', 0.5, fb.POSE_BAD_QUATERNION), ('qw', float('nan'), fb.POSE_NONFINITE),
])
def test_nonfinite_and_non_unit_entries_are_refused(weld, field, value, code):
    t = good(weld)
    m = t[1].transform
    if field.startswith('q'):
        setattr(m.rotation, field[1], value)
    else:
        setattr(m.translation, field, value)
    assert fb.select_entries(t).codes == (code,)


def test_nearly_unit_quaternions_are_normalized(weld):
    t = good(weld)
    t[1].transform.rotation.w = 1.0005
    s = fb.select_entries(t)
    assert s.codes == () and math.isclose(sum(c * c for c in s.model.q), 1.0)


def test_intact_weld_composes_the_body_at_the_mount(cfg, weld):
    obs = fb.evaluate_sample(fb.select_entries(good(weld)), weld, cfg, 1.0, 2.0)
    assert obs.codes == () and obs.pose6() == pytest.approx(cfg.mount.xyz_rpy())
    assert obs.attachment == pytest.approx((0, 0, 0, 0))


def test_full_composition_with_nonzero_mount_and_body_offset(cfg):
    """T_world_body = T_world_model * T_model_dummy * T_dummy_body with every factor non-trivial."""
    mount = fb.Pose.from_xyz_rpy(0.31, -0.17, 0.43, 0, 0, 0.7)
    db = fb.Pose.from_xyz_rpy(0.01, 0.02, -0.03, 0.0, 0.0, 0.3)
    w = fb.parse_robot_description(fb.minimal_description(mount, dummy_to_body=db))
    model = fb.Pose.from_xyz_rpy(0.002, -0.001, 0.0005, 0.001, -0.002, 0.003)
    sel = fb.select_entries([tf('spiderx', model), tf('dummy_link', mount)])
    obs = fb.evaluate_sample(sel, w, cfg, 0.0, 0.0)
    assert np.allclose(matrix(obs.body), matrix(model) @ matrix(mount) @ matrix(db), atol=1e-12)
    assert obs.attachment[0] == pytest.approx(
        np.linalg.norm((matrix(model) @ matrix(mount) @ matrix(db))[:3, 3]
                       - (matrix(mount) @ matrix(db))[:3, 3]))


def test_double_counted_spawn_is_detected(cfg, weld):
    t = good(weld)
    t[1] = tf('spiderx', fb.Pose.from_xyz_rpy(z=0.075))        # fortress.launch.py default -z
    obs = fb.evaluate_sample(fb.select_entries(t), weld, cfg, 0, 0)
    assert fb.FRAME_SPAWN_NOT_IDENTITY in obs.codes and fb.ATTACHMENT_DISPLACED in obs.codes
    assert obs.pose6()[2] == pytest.approx(0.2)


def test_description_and_spawned_link_mismatch_is_detected(cfg, weld):
    t = good(weld)
    t[2] = tf('dummy_link', fb.Pose.from_xyz_rpy(z=0.075))     # Gazebo spawned another mount
    obs = fb.evaluate_sample(fb.select_entries(t), weld, cfg, 0, 0)
    assert fb.FRAME_LINK_INCONSISTENT in obs.codes


@pytest.mark.parametrize('dz, displaced', [(0.0029, False), (0.0031, True), (-0.07, True)])
def test_attachment_translation_threshold(cfg, weld, dz, displaced):
    body = fb.Pose.from_xyz_rpy(0, 0, cfg.mount.p[2] + dz)
    model = body * weld.dummy_to_body.inverse() * weld.model_to_dummy.inverse()
    obs = fb.evaluate_sample(fb.select_entries([tf('spiderx', model),
                                                tf('dummy_link', weld.model_to_dummy)]),
                             weld, cfg, 0, 0)
    assert (fb.ATTACHMENT_DISPLACED in obs.codes) is displaced
    assert obs.attachment[2] == pytest.approx(dz)


def test_attachment_rotation_threshold(cfg, weld):
    for ang, displaced in ((0.0099, False), (0.0101, True)):
        model = fb.Pose.from_xyz_rpy(0, 0, 0, ang, 0, 0)
        obs = fb.evaluate_sample(fb.select_entries([tf('spiderx', model),
                                                    tf('dummy_link', weld.model_to_dummy)]),
                                 weld, cfg, 0, 0)
        assert (fb.ATTACHMENT_DISPLACED in obs.codes) is displaced


def test_a_detached_body_resting_above_the_height_gate_is_still_caught(cfg, weld):
    """The 0.045 m G1 threshold misses a body resting at 0.0545 m; the attachment check does not."""
    from spiderx_controller import m61_live_contract as c61
    body = fb.Pose.from_xyz_rpy(0, 0, 0.0545)
    model = body * weld.model_to_dummy.inverse()
    obs = fb.evaluate_sample(fb.select_entries([tf('spiderx', model),
                                                tf('dummy_link', weld.model_to_dummy)]),
                             weld, cfg, 0, 0)
    assert obs.pose6()[2] > c61.APPROVED_LIMITS['body_min_height_m']
    assert fb.ATTACHMENT_DISPLACED in obs.codes


def test_attachment_monitor_debounce(cfg, weld):
    mon = fb.AttachmentMonitor(cfg)
    bad = fb.BodyObservation(0, 0, (fb.ATTACHMENT_DISPLACED,), attachment=(0.01, 0, 0, 0))
    ok = fb.BodyObservation(0, 0, (), attachment=(0.001, 0, 0, 0))
    assert mon.update(bad) is None and mon.update(ok) is None and mon.update(bad) is None
    assert mon.update(bad) == fb.ATTACHMENT_DISPLACED
    assert mon.max_translation_m == 0.01 and mon.samples == 4
    assert mon.update(fb.BodyObservation(0, 0, (fb.POSE_MISSING_MODEL,))) is None


# ==================================================================== clocks and streams
def test_clock_monitor_progress_pause_reset(cfg):
    c = fb.ClockMonitor(5.0, 0.001)
    assert c.check(0.0) == fb.CLOCK_MISSING
    c.on_clock(0.0, 100.0)
    c.on_clock(1.0, 100.5)
    assert c.check(5.9) is None                       # last advance at wall 1.0
    c.on_clock(6.0, 100.5)                            # paused: same sim time
    assert c.check(6.1) == fb.CLOCK_STALLED           # 5.1 s of wall without an advance
    c.on_clock(6.2, 100.6)
    assert c.check(6.3) is None
    c.on_clock(6.4, float('nan'))                     # ignored
    c.on_clock(6.5, 100.6 - 0.0005)                   # within the reset tolerance
    assert c.check(6.6) is None
    c.on_clock(6.7, 3.0)                              # world reset
    assert c.check(6.8) == fb.CLOCK_RESET
    c.on_clock(7.0, 3.5)
    assert c.check(7.1) == fb.CLOCK_RESET             # latched


def test_joint_state_and_pose_freshness_are_wall_receipt_times():
    names = ['a', 'b']
    assert fb.check_joint_states(None, 1.0, names, 0.5) == fb.JOINT_STATES_MISSING
    assert fb.check_joint_states((0.0, {'a': 0, 'b': 0}), 0.6, names, 0.5) == \
        fb.JOINT_STATES_STALE
    assert fb.check_joint_states((0.5, {'a': 0}), 0.6, names, 0.5) == fb.JOINT_STATES_INCOMPLETE
    assert fb.check_joint_states((0.5, {'a': 0, 'b': float('nan')}), 0.6, names, 0.5) == \
        fb.JOINT_STATES_INCOMPLETE
    assert fb.check_joint_states((0.5, {'a': 0, 'b': 1}), 0.6, names, 0.5) is None
    smp = fb.PoseSample(10.0, 0.0, None)          # sim stamp 0 (bridge): only receipt counts
    assert fb.check_pose_freshness(None, 10.0, 1.0) == fb.POSE_NEVER_RECEIVED
    assert fb.check_pose_freshness(smp, 10.9, 1.0) is None
    assert fb.check_pose_freshness(smp, 11.1, 1.0) == fb.POSE_STALE


# ==================================================================== tracker and readiness
def test_tracker_events_and_invalid_samples(cfg, weld):
    tr = fb.FixedBasePoseTracker(cfg)
    assert tr.on_transforms(good(weld), 1.0, 2.0) == [
        ('fixed_base', 1.0, 2.0, (fb.DESCRIPTION_MISSING,), None)]
    assert tr.latest_body_pose() is None
    tr.on_description(fb.minimal_description(cfg.mount), 1.1)
    ev = tr.on_transforms(good(weld), 1.2, 2.2)
    assert [e[0] for e in ev] == ['body_pose', 'fixed_base'] and ev[1][3] == ()
    assert tr.latest_body_pose() == (1.2, 2.2, pytest.approx(cfg.mount.xyz_rpy()))
    assert tr.on_transforms(good(weld)[:2], 1.3, 2.3) == []        # missing body link
    assert tr.latest_usable.wall == 1.2 and tr.invalid == {fb.POSE_MISSING_BODY_LINK: 1}
    tr.on_description(fb.minimal_description(fixed_base=False), 1.4)
    assert tr.on_transforms(good(weld), 1.5, 2.5)[0][3] == (fb.DESCRIPTION_NOT_FIXED_BASE,)
    tr.on_description('<not xml', 1.6)
    assert tr.description_code is not None and tr.latest_body_pose() is None


def evidence(cfg, weld, now=10.0, **kw):
    c = fb.ClockMonitor(cfg.sim_stall_s, cfg.clock_reset_tol_s)
    c.on_clock(now - 0.1, 50.0)
    c.on_clock(now - 0.05, 50.1)
    e = fb.Evidence(description=fb.minimal_description(cfg.mount),
                    latest_usable_pose=fb.PoseSample(now - 0.05, 50.1,
                                                     fb.select_entries(good(weld))),
                    latest_joint_states=(now - 0.01, {'j1': 0.0, 'j2': 0.0}), clock=c,
                    controllers={'joint_state_broadcaster': 'active',
                                 'leg_trajectory_controller': 'active'},
                    joint_state_publishers=1, command_publishers=0, action_clients=0)
    for k, v in kw.items():
        setattr(e, k, v)
    return e


def test_assess_ready_with_complete_evidence(cfg, weld):
    ready, codes, rep = fb.assess(evidence(cfg, weld), cfg, 10.0, ['j1', 'j2'])
    assert ready and codes == []
    assert rep['checks']['body_pose_xyz_rpy'] == pytest.approx(cfg.mount.xyz_rpy())
    assert {k for k, v in rep['checks'].items() if isinstance(v, dict) and 'ok' in v} == {
        'robot_description', 'frame_body_link', 'spawn_identity', 'attachment',
        'body_pose_fresh', 'joint_states', 'sim_clock', 'controllers',
        'joint_state_publishers', 'command_publishers', 'action_clients'}


@pytest.mark.parametrize('kw, code', [
    ({'description': None}, fb.DESCRIPTION_MISSING),
    ({'description': BASE.format('')}, fb.DESCRIPTION_NOT_FIXED_BASE),
    ({'description': '<robot'}, fb.DESCRIPTION_INVALID),
    ({'latest_usable_pose': None}, fb.POSE_NEVER_RECEIVED),
    ({'latest_joint_states': None}, fb.JOINT_STATES_MISSING),
    ({'clock': None}, fb.CLOCK_MISSING),
    ({'controllers': None}, fb.CONTROLLERS_UNKNOWN),
    ({'controllers': {'joint_state_broadcaster': 'active',
                      'leg_trajectory_controller': 'inactive'}}, fb.CONTROLLERS_NOT_ACTIVE),
    ({'joint_state_publishers': 2}, fb.JOINT_STATE_PUBLISHERS),
    ({'command_publishers': 1}, fb.COMMAND_PUBLISHERS),
    # M6.1-A review: another FollowJointTrajectory client is a competing commander too, and an
    # unmeasured count (None) is not READY (behaviour change: evidence must now carry the count)
    ({'action_clients': 1}, fb.COMMAND_ACTION_CLIENTS),
    ({'action_clients': None}, fb.COMMAND_ACTION_CLIENTS),
])
def test_assess_names_each_failure(cfg, weld, kw, code):
    ready, codes, _ = fb.assess(evidence(cfg, weld, **kw), cfg, 10.0, ['j1', 'j2'])
    assert not ready and code in codes


def test_assess_stale_pose_and_stalled_clock_are_separate(cfg, weld):
    ready, codes, _ = fb.assess(evidence(cfg, weld), cfg, 16.0, ['j1', 'j2'])
    assert fb.POSE_STALE in codes and fb.CLOCK_STALLED in codes and fb.JOINT_STATES_STALE in codes


def test_assess_plant_frame_codes(cfg, weld):
    t = good(weld)
    t[1] = tf('spiderx', fb.Pose.from_xyz_rpy(z=0.075))
    ok, codes, _ = fb.assess_plant(fb.minimal_description(cfg.mount),
                                   fb.PoseSample(1, 1, fb.select_entries(t)), cfg)
    assert not ok and codes == [fb.FRAME_SPAWN_NOT_IDENTITY, fb.ATTACHMENT_DISPLACED]



@pytest.mark.parametrize('pubs, clients, codes', [
    (0, 0, []),
    (1, 0, [fb.COMMAND_PUBLISHERS]),
    (0, 2, [fb.COMMAND_ACTION_CLIENTS]),
    (None, 0, [fb.COMMAND_OWNER_UNKNOWN]),
    (0, None, [fb.COMMAND_OWNER_UNKNOWN]),
])
def test_command_owner_codes(pubs, clients, codes):
    assert fb.command_owner_codes(pubs, clients) == codes
