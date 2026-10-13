"""M6.1-A link-pose check (m61a_link_check): E8 body pose and provenance, E9 consistency.

Offline only (no Gazebo, no ROS graph). Synthetic streams follow the gz-sim 6.16 Physics rule:
model = X_WL * inv(X_ML), dummy_link = X_ML (never written), link = inv(model) * X_W_link. One
test writes and reads a real rosbag2 directory.
"""
import json
import math
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
import pytest

from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m61a_fixed_base as fb
from spiderx_controller import m61a_link_check as lc

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
WRAPPER = os.path.join(get_package_share_directory('spiderx_description'), 'urdf',
                       'spiderx_fixed_base.urdf.xacro')
# The 12 link entries the Cloud runs showed in pose/info (run_05L, ign topic -e).
GAZEBO_LINKS = ('b_s1_c_horn_1', 'b_s2_c_horn_1', 'b_s3_c_horn_1', 'b_s4_c_horn_1',
                'lf_c_horn_1', 'lf_foot_1', 'lr_c_horn_1', 'lr_foot_1',
                'rf_c_horn_1', 'rf_foot_1', 'rr_c_horn_1', 'rr_foot_1')
# The held joint positions of run_04 (/joint_states, constant to 1e-19 rad over 6 s).
HELD = {'lf_hip': -9.695957321523189e-05, 'lf_thigh_joint': -4.4193648471699876e-05,
        'lf_foot_joint': -8.66391202799433e-05, 'rf_hip': 9.695977715041874e-05,
        'rf_thigh_joint': 4.419928178681439e-05, 'rf_foot_joint': 8.66432539565463e-05,
        'lr_hip': 9.695957321522579e-05, 'lr_thigh_joint': -4.419364847169718e-05,
        'lr_foot_joint': 8.663912027994451e-05, 'rr_hip': -9.695977715041457e-05,
        'rr_thigh_joint': 4.4199281786814946e-05, 'rr_foot_joint': 8.664325395655142e-05}


@pytest.fixture(scope='module')
def urdf():
    return subprocess.run(['xacro', WRAPPER], check=True, capture_output=True, text=True).stdout


@pytest.fixture(scope='module')
def model(urdf):
    return lc.link_model(urdf)


@pytest.fixture(scope='module')
def cfg():
    return fb.load_config(CONFIG_DIR)


@pytest.fixture(scope='module')
def weld(urdf):
    return fb.parse_robot_description(urdf)


class Tr:
    """TransformStamped look-alike."""

    def __init__(self, name, pose):
        self.child_frame_id = name
        self.transform = type('T', (), {})()
        self.transform.translation = type('V', (), {})()
        self.transform.rotation = type('Q', (), {})()
        tr, q = self.transform.translation, self.transform.rotation
        tr.x, tr.y, tr.z = pose.p
        q.x, q.y, q.z, q.w = pose.q
        self.header = type('H', (), {'frame_id': ''})()


WELD = fb.Pose((0.0, 0.0, 0.125))


def physics_entries(model, q, body_world=WELD, x_ml=WELD, written=True, extra=()):
    """pose/info entries by the Physics rule. written=False: the SDF initial link values."""
    x_wm = body_world * x_ml.inverse()
    fk = lc.forward_links(model, q if written else {n: 0.0 for n in model.joints})
    out = [Tr('spiderx', x_wm), Tr('dummy_link', x_ml)]
    for link, rel in fk.items():
        if written:
            out.append(Tr(link, x_wm.inverse() * (body_world * rel)))
        else:
            out.append(Tr(link, x_ml * rel))
    return out + list(extra)


def stream(model, q_of_t, t0=10.0, dur=4.0, pose_hz=60.0, joint_hz=70.0, lag=0.0,
           entries=None, body_of_t=None):
    """read_bag-shaped data: poses at pose_hz (content from time t - lag), clock at 1 kHz,
    joints at joint_hz. Receive time = sim time here (no latency)."""
    clock = [(t0 + i * 1e-3, t0 + i * 1e-3) for i in range(int(dur * 1000) + 1)]
    joints = [lc.JointSample(t0 + i / joint_hz, q_of_t(t0 + i / joint_hz))
              for i in range(int(dur * joint_hz) + 1)]
    poses = []
    for i in range(int(dur * pose_hz) + 1):
        t = t0 + i / pose_hz
        body = body_of_t(t) if body_of_t else WELD
        tr = entries(t) if entries else physics_entries(model, q_of_t(t - lag), body)
        poses.append((t + 2e-4, tr))
    return {'poses': poses, 'joints': joints, 'clock': clock, 'description': None}


def held(t):
    return dict(HELD)


def trot_like(t, t0=10.5, amp=0.08, period=4.0):
    """Joints at the held pose plus a smooth swing (|v| <= 0.13 rad/s) after t0."""
    q = dict(HELD)
    if t > t0:
        s = math.sin(math.pi * (t - t0) / period) ** 2
        for n in ('lf_foot_joint', 'rr_foot_joint', 'rf_foot_joint', 'lr_foot_joint'):
            q[n] += amp * s
        for n in ('lf_thigh_joint', 'rr_thigh_joint'):
            q[n] -= 0.5 * amp * s
    return q


def analyse(model, urdf, cfg, data, log=None):
    return lc.analyse(data, urdf, cfg, log)


# ---------------------------------------------------------------- frozen values
def test_frozen_parameters_are_pinned():
    assert lc.frozen_parameters() == {
        'report_tol': 1e-6,
        'provenance': {'match_position_m': 1e-5, 'match_rotation_rad': 1e-5,
                       'min_separation_rad': 5e-5, 'steady_window_s': 0.25,
                       'steady_q_rad': 1e-6, 'min_samples': 20},
        'e9': {'position_tol_m': 1e-3, 'rotation_tol_rad': 5e-3, 'align_window_s': 0.025,
               'align_step_s': 0.001, 'min_samples': 100, 'max_gap_s': 1.0,
               'min_foot_span_rad': 0.02,
               'foot_links': ['lf_foot_1', 'rf_foot_1', 'lr_foot_1', 'rr_foot_1']},
        'log_patterns': ['Internal error', 'does not have a world pose'],
    }


def test_tolerances_clear_the_reporting_threshold():
    worst_t = math.sqrt(3) * lc.REPORT_TOL
    assert lc.PROV_MATCH_POSITION_M >= 2.5 * worst_t
    assert lc.PROV_MIN_SEPARATION_RAD >= 5 * lc.PROV_MATCH_ROTATION_RAD
    assert lc.E9_MIN_FOOT_SPAN_RAD >= 4 * lc.E9_ROTATION_TOL_RAD


# ---------------------------------------------------------------- kinematics
def test_link_model_has_the_gazebo_links(model):
    assert model.root == 'dummy_link'
    assert model.links == GAZEBO_LINKS
    assert len(model.joints) == 12 and set(model.joints) == set(HELD)
    assert 'rr_holder_1' not in model.links and 'base_link' not in model.links


def test_link_model_refuses_a_tree_without_moving_links():
    with pytest.raises(lc.LinkCheckError):
        lc.link_model('<robot name="r"><link name="dummy_link"/></robot>')


def test_forward_links_equals_the_m3_validated_leg_fk(model):
    geoms = lk.load_all_geometries()
    worst = 0.0
    for k in range(20):
        q = {n: 0.25 * math.sin(1.7 * k + i) for i, n in enumerate(model.joints)}
        fk = lc.forward_links(model, q)
        for g in geoms.values():
            ref = lk.forward(g, [q[n] for n in g.joint_names])
            R = lk.matrix_from_quaternion(*fk[g.foot_link].q)
            worst = max(worst, math.dist(fk[g.foot_link].p, ref['foot_link_position']),
                        lk.rotation_angle_between(R, ref['foot_link_rotation']))
    assert worst < 1e-12


def test_angle_between_is_accurate_for_small_angles():
    for a in (1e-9, 1e-6, 1e-3, 1.0, 3.0):
        q = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))
        assert lc.angle_between((0.0, 0.0, 0.0, 1.0), q) == pytest.approx(a, rel=1e-6)
        neg = tuple(-c for c in q)                 # same rotation, opposite sign
        assert lc.angle_between((0.0, 0.0, 0.0, 1.0), neg) == pytest.approx(a, rel=1e-6)


def test_held_pose_discriminates_fk_q_from_fk_0(model):
    fq = lc.forward_links(model, HELD)
    f0 = lc.forward_links(model, {n: 0.0 for n in model.joints})
    sep = [lc.residual(fq[k], f0[k])[1] for k in model.links]
    assert min(sep) >= lc.PROV_MIN_SEPARATION_RAD


# ---------------------------------------------------------------- samples and inputs
def test_pose_sample_codes(model):
    names = {'spiderx', 'dummy_link', *model.links}
    good = physics_entries(model, HELD)
    assert lc.pose_sample(good, 1.0, names).codes == ()
    s = lc.pose_sample(good[:-1], 1.0, names)
    assert s.codes == (f'missing:{good[-1].child_frame_id}',)
    assert lc.pose_sample(good + [good[3]], 1.0, names).codes == (
        f'ambiguous:{good[3].child_frame_id}',)
    bad = physics_entries(model, HELD)
    bad[2].transform.translation.x = float('nan')
    assert lc.pose_sample(bad, 1.0, names).codes == (
        f'{fb.POSE_NONFINITE}:{bad[2].child_frame_id}',)
    bad = physics_entries(model, HELD)
    bad[2].transform.rotation.w = 2.0
    assert lc.pose_sample(bad, 1.0, names).codes == (
        f'{fb.POSE_BAD_QUATERNION}:{bad[2].child_frame_id}',)
    assert lc.pose_sample(good, None, names).codes == ('no_sim_time',)
    other = good + [Tr('link', fb.Pose()), Tr('link', fb.Pose())]   # other models' names
    assert lc.pose_sample(other, 1.0, names).codes == ()


def test_attach_sim_time_uses_the_latest_clock_received_before():
    clock = [(1.0, 10.0), (2.0, 10.5), (3.0, 11.0)]
    poses = [(0.5, 'a'), (1.0, 'b'), (2.5, 'c'), (9.0, 'd')]
    assert lc.attach_sim_time(poses, clock) == [(None, 'a'), (10.0, 'b'), (10.5, 'c'),
                                                (11.0, 'd')]


def test_joint_track_interpolates_without_extrapolating():
    tr = lc.JointTrack([lc.JointSample(1.0, {'a': 0.0}), lc.JointSample(2.0, {'a': 1.0}),
                        lc.JointSample(1.5, {'a': float('nan')})], ['a'])
    assert len(tr) == 2
    assert tr.at(1.25) == {'a': 0.25} and tr.at(2.0) == {'a': 1.0}
    assert tr.at(0.99) is None and tr.at(2.01) is None
    assert tr.span(1.0, 1.5) == pytest.approx(0.5)


def test_server_log_scan():
    log = ('[Wrn] [Model.hh:69] Skipping serialization\n'
           '[Err] [Physics.cc:2794] Internal error: parent model [8] does not have a world pose '
           'available for child entity[9]\n')
    hits = lc.scan_server_log(log)
    assert len(hits) == 1 and 'parent model' in hits[0]
    assert lc.scan_server_log('[GUI] [Err] [Plugin.cc:147] Failed to instantiate QML') == []


# ---------------------------------------------------------------- E8: body pose
def test_body_pose_is_the_physics_pose_whatever_x_ml_is(model, urdf, cfg):
    # Physics divides by X_ML and the composition multiplies by the same X_ML: a wrong X_ML
    # (here 0.5 m instead of 0.125 m) changes the model entry but not the composed body.
    data = stream(model, held, entries=lambda t: physics_entries(
        model, HELD, WELD, fb.Pose((0.0, 0.0, 0.5))))
    rep = analyse(model, urdf, cfg, data)
    b = rep['e8_body_pose']
    assert b['verdict'] == lc.PASS
    assert b['max_deviation']['translation_m'] < 1e-12
    assert b['model_entry_first_xyz_rpy'][2] == pytest.approx(-0.375)


def test_body_pose_fails_when_the_physics_body_moves(model, urdf, cfg):
    dropped = fb.Pose((0.0, 0.0, 0.125 - 0.002))
    data = stream(model, held, body_of_t=lambda t: WELD if t < 11.0 else dropped)
    b = analyse(model, urdf, cfg, data)['e8_body_pose']
    assert b['verdict'] == lc.FAIL
    assert b['max_deviation']['abs_dz_m'] == pytest.approx(0.002)
    assert b['model_entry_max_change']['translation_m'] == pytest.approx(0.002)
    tilted = fb.Pose.from_xyz_rpy(0.0, 0.0, 0.125, 0.004, 0.0, 0.0)
    data = stream(model, held, body_of_t=lambda t: tilted)
    b = analyse(model, urdf, cfg, data)['e8_body_pose']
    assert b['verdict'] == lc.FAIL
    assert b['max_deviation']['rotation_rad'] == pytest.approx(0.004)


def test_body_pose_limits_are_one_third_of_the_attachment_tolerance(model, urdf, cfg):
    b = analyse(model, urdf, cfg, stream(model, held))['e8_body_pose']
    assert b['limits'] == {'translation_m': pytest.approx(0.001),
                           'rotation_rad': pytest.approx(0.01 / 3),
                           'abs_dz_m': pytest.approx(0.001)}


# ---------------------------------------------------------------- E8: provenance
def test_provenance_passes_for_entries_written_by_physics(model, urdf, cfg):
    p = analyse(model, urdf, cfg, stream(model, held), log='')['e8_provenance']
    assert p['verdict'] == lc.PASS, p
    assert p['discriminating_samples'] == p['samples_used'] >= lc.PROV_MIN_SAMPLES
    assert p['min_residual_vs_fk_0_on_discriminating_links_rad'] > 5e-5


def test_provenance_tolerates_the_reporting_threshold(model, urdf, cfg):
    def entries(t):
        out = physics_entries(model, HELD)
        for tr in out[2:]:                         # every link stale by up to 1e-6 per component
            tr.transform.translation.x += 1e-6
            tr.transform.translation.z -= 1e-6
            r = tr.transform.rotation
            r.x, r.y = r.x + 1e-6, r.y - 1e-6
        return out
    p = analyse(model, urdf, cfg, stream(model, held, entries=entries))['e8_provenance']
    assert p['verdict'] == lc.PASS


def test_provenance_fails_for_sdf_initial_entries(model, urdf, cfg):
    data = stream(model, held, entries=lambda t: physics_entries(model, HELD, written=False))
    p = analyse(model, urdf, cfg, data)['e8_provenance']
    assert p['verdict'] == lc.FAIL
    assert p['reason'] == 'entries do not match FK at the measured joint positions'
    assert p['samples_matching_fk_0'] == p['samples_used']


def test_provenance_fails_for_one_wrong_link(model, urdf, cfg):
    def entries(t):
        out = physics_entries(model, HELD)
        r = out[5].transform.rotation             # one leg link turned by 2e-5 rad about z
        q = fb.quat_mul((r.x, r.y, r.z, r.w), (0.0, 0.0, math.sin(1e-5), math.cos(1e-5)))
        r.x, r.y, r.z, r.w = q
        return out
    p = analyse(model, urdf, cfg, stream(model, held, entries=entries))['e8_provenance']
    assert p['verdict'] == lc.FAIL and p['mismatch_samples'] == p['samples_used']


def test_provenance_cannot_discriminate_at_zero(model, urdf, cfg):
    zero = {n: 0.0 for n in model.joints}
    p = analyse(model, urdf, cfg, stream(model, lambda t: zero))['e8_provenance']
    assert p['verdict'] == lc.INCONCLUSIVE and 'too close to 0' in p['reason']
    tiny = {n: 1e-6 for n in model.joints}
    p = analyse(model, urdf, cfg, stream(model, lambda t: tiny))['e8_provenance']
    assert p['verdict'] == lc.INCONCLUSIVE


def test_provenance_fails_on_a_contradicting_server_log_line(model, urdf, cfg):
    log = 'Internal error: parent model [8] does not have a world pose available\n'
    p = analyse(model, urdf, cfg, stream(model, held), log=log)['e8_provenance']
    assert p['verdict'] == lc.FAIL and p['server_log_hits']
    assert analyse(model, urdf, cfg, stream(model, held))['e8_provenance'][
        'server_log_checked'] is False


def test_provenance_uses_only_steady_complete_samples(model, urdf, cfg):
    data = stream(model, lambda t: trot_like(t, t0=11.0), dur=2.0)
    p = analyse(model, urdf, cfg, data)['e8_provenance']
    assert p['skipped']['not_steady'] > 0
    assert p['verdict'] == lc.PASS              # the 0.75 s before the swing are steady
    short = stream(model, held, dur=0.2)
    p = analyse(model, urdf, cfg, short)['e8_provenance']
    assert p['verdict'] == lc.INCONCLUSIVE and p['samples_used'] < lc.PROV_MIN_SAMPLES


# ---------------------------------------------------------------- E9
def test_e9_passes_for_a_live_stream_in_motion(model, urdf, cfg):
    e = analyse(model, urdf, cfg, stream(model, trot_like, dur=4.0))['e9_link_consistency']
    assert e['verdict'] == lc.PASS, e
    assert e['worst']['score'] < 0.05
    assert min(e['foot_link_max_turn_from_first_rad'].values()) >= 0.02
    assert e['stale_stream_counterfactual_worst_score'] > 1.0


def test_e9_tolerates_a_small_delivery_lag(model, urdf, cfg):
    e = analyse(model, urdf, cfg, stream(model, trot_like, lag=0.01))['e9_link_consistency']
    assert e['verdict'] == lc.PASS


def test_e9_fails_for_a_stale_or_late_stream(model, urdf, cfg):
    first = physics_entries(model, trot_like(10.0))
    e = analyse(model, urdf, cfg, stream(model, trot_like, entries=lambda t: first))[
        'e9_link_consistency']
    assert e['verdict'] == lc.FAIL and e['unexplained_samples'] > 0
    e = analyse(model, urdf, cfg, stream(model, trot_like, lag=0.15))['e9_link_consistency']
    assert e['verdict'] == lc.FAIL


def test_e9_without_motion_is_not_satisfied(model, urdf, cfg):
    e = analyse(model, urdf, cfg, stream(model, held))['e9_link_consistency']
    assert e['verdict'] == lc.NO_MOTION
    assert e['unexplained_samples'] == 0


def test_e9_is_blind_to_the_global_body_pose(model, urdf, cfg):
    # The body (and with it the model entry) displaced 5 cm and tilted 0.1 rad: the leg entries
    # are relative to the body, so E9 still passes while the E8 body check fails.
    moved = fb.Pose.from_xyz_rpy(0.01, 0.0, 0.075, 0.1, 0.0, 0.0)
    rep = analyse(model, urdf, cfg, stream(model, trot_like, body_of_t=lambda t: moved))
    assert rep['e9_link_consistency']['verdict'] == lc.PASS
    assert rep['e8_body_pose']['verdict'] == lc.FAIL
    assert 'not an independent measurement' in rep['e9_link_consistency']['claim_scope']


def test_e9_fails_on_a_gap_and_needs_enough_samples(model, urdf, cfg):
    data = stream(model, trot_like, dur=4.0)
    data['poses'] = [p for p in data['poses'] if not 11.0 < p[0] < 12.2]
    e = analyse(model, urdf, cfg, data)['e9_link_consistency']
    assert e['verdict'] == lc.FAIL and e['reason'] == 'pose gap'
    e = analyse(model, urdf, cfg, stream(model, trot_like, dur=1.0))['e9_link_consistency']
    assert e['verdict'] == lc.INCONCLUSIVE


# ---------------------------------------------------------------- bag and CLI
def _write_bag(path, model, urdf, n=40, description=True):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import TransformStamped
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, typ in (('/spiderx/sim/world_poses', 'tf2_msgs/msg/TFMessage'),
                      ('/joint_states', 'sensor_msgs/msg/JointState'),
                      ('/clock', 'rosgraph_msgs/msg/Clock'),
                      ('/robot_description', 'std_msgs/msg/String')):
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))
    if description:
        w.write('/robot_description', serialize_message(String(data=urdf)), 1)
    for i in range(n):
        t_ns = 1_000_000_000 + i * 20_000_000
        sim = 5.0 + i * 0.02
        c = Clock()
        c.clock.sec, c.clock.nanosec = int(sim), int(round((sim - int(sim)) * 1e9))
        w.write('/clock', serialize_message(c), t_ns)
        js = JointState(name=list(HELD), position=list(HELD.values()))
        js.header.stamp = c.clock
        w.write('/joint_states', serialize_message(js), t_ns + 1000)
        msg = TFMessage()
        for tr in physics_entries(model, HELD, extra=[Tr('ground_plane', fb.Pose())]):
            ts = TransformStamped()
            ts.child_frame_id = tr.child_frame_id
            v, r = tr.transform.translation, tr.transform.rotation
            ts.transform.translation.x, ts.transform.translation.y = v.x, v.y
            ts.transform.translation.z = v.z
            ts.transform.rotation.x, ts.transform.rotation.y = r.x, r.y
            ts.transform.rotation.z, ts.transform.rotation.w = r.z, r.w
            msg.transforms.append(ts)
        w.write('/spiderx/sim/world_poses', serialize_message(msg), t_ns + 2000)
    del w


def test_cli_reads_a_bag_and_writes_the_report(tmp_path, model, urdf):
    bag = str(tmp_path / 'bag')
    _write_bag(bag, model, urdf)
    log = tmp_path / 'launch.log'
    log.write_text('[Wrn] nothing relevant\n')
    out = tmp_path / 'out'
    assert lc.main(['--bag', bag, '--out', str(out), '--server-log', str(log),
                    '--config-dir', CONFIG_DIR]) == 0
    rep = json.loads((out / 'link_check.json').read_text())
    assert rep['schema'] == lc.SCHEMA and rep['analysis_sha256'] == lc.source_sha256()
    assert rep['inputs']['pose_messages'] == 40 and rep['inputs']['joint_samples'] == 40
    assert rep['e8_body_pose']['verdict'] == lc.PASS
    assert rep['e8_provenance']['verdict'] == lc.PASS
    assert rep['e8_provenance']['server_log_checked'] is True
    assert rep['e9_link_consistency']['verdict'] == lc.INCONCLUSIVE   # 40 samples, no motion
    assert 'E8 provenance' in (out / 'link_check.txt').read_text()


def test_cli_refuses_a_bag_without_a_description(tmp_path, model, urdf):
    bag = str(tmp_path / 'bag')
    _write_bag(bag, model, urdf, n=3, description=False)
    assert lc.main(['--bag', bag, '--out', str(tmp_path / 'o'), '--config-dir', CONFIG_DIR]) == 2
    f = tmp_path / 'd.urdf'
    f.write_text(urdf)
    assert lc.main(['--bag', bag, '--out', str(tmp_path / 'o'), '--urdf', str(f),
                    '--config-dir', CONFIG_DIR]) == 0


def test_module_is_read_only():
    src = open(lc.__file__).read()
    for word in ('create_publisher', 'create_client', 'ActionClient', 'send_goal', 'call_async',
                 'SequentialWriter'):
        assert word not in src
