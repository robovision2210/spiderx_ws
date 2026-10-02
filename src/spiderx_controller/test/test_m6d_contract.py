"""colcon test: M6.0-D Batch A - goal contract, fingerprint, classification, readiness freshness,
typed confirmation and protected-file pins. Pure Python: no rclpy.init, no node, no ROS graph.
"""
import copy
import hashlib
import math
import os

from control_msgs.action import FollowJointTrajectory
import pytest

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_preflight as lp
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC_CONFIG = os.path.join(PKG, 'config')


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


@pytest.fixture
def traj(sources):
    return m6t.build_trajectory(sources)


@pytest.fixture
def live(traj, sources):
    return gf.build_live_goal(traj, sources)


# ---------------------------------------------------------------- limits
def test_live_dispatch_is_hard_disabled():
    assert lc.LIVE_DISPATCH_ENABLED is False
    assert 'HARD-DISABLED' in lc.LIVE_DISPATCH_DISABLED_MESSAGE


def test_approved_limits_are_exact():
    assert lc.GOAL_VELOCITY_TOLERANCE_RAD_S == 0.05
    assert lc.GOAL_POSITION_TOLERANCE_RAD == lc.PATH_POSITION_TOLERANCE_RAD == 0.05
    assert lc.CLIENT_TRACKING_ABORT_RAD == 0.05
    assert lc.GOAL_TIME_TOLERANCE_S == 1.0
    assert (lc.GOAL_RESPONSE_TIMEOUT_S, lc.CANCEL_RESPONSE_TIMEOUT_S,
            lc.FINAL_STATUS_AFTER_CANCEL_S, lc.RESULT_WATCHDOG_S) == (10.0, 5.0, 5.0, 120.0)
    assert lc.READINESS_MAX_AGE_S == 10.0
    assert (lc.JOINT_STATES_STALE_S, lc.SIM_STALL_S, lc.SAMPLE_GAP_S,
            lc.CONTROLLER_CHECK_PERIOD_S) == (0.5, 5.0, 0.25, 1.0)
    assert lc.APPROVED_POINT_TIMES_S == (3.0, 6.0, 9.0)
    assert lc.APPROVED_GOAL_COUNT == 1


# ---------------------------------------------------------------- goal contract
def test_live_goal_has_every_approved_field(live, traj, sources):
    goal, report, spec, fp = live
    assert report.ok and isinstance(goal, FollowJointTrajectory.Goal)
    t = goal.trajectory
    assert (t.header.stamp.sec, t.header.stamp.nanosec) == (0, 0)
    assert list(t.joint_names) == list(sources.joint_names) and len(t.joint_names) == 12
    assert [(p.time_from_start.sec, p.time_from_start.nanosec) for p in t.points] == \
        [(3, 0), (6, 0), (9, 0)]
    for p, src in zip(t.points, traj['points']):
        assert list(p.positions) == src['positions']
        assert list(p.velocities) == [0.0] * 12
        assert list(p.accelerations) == [] and list(p.effort) == []
    for tol in goal.path_tolerance:
        assert (tol.position, tol.velocity, tol.acceleration) == (0.05, 0.0, 0.0)
    for tol in goal.goal_tolerance:
        assert (tol.position, tol.velocity, tol.acceleration) == (0.05, 0.05, 0.0)
    assert [x.name for x in goal.goal_tolerance] == list(t.joint_names)
    assert (goal.goal_time_tolerance.sec, goal.goal_time_tolerance.nanosec) == (1, 0)
    assert len(fp) == 64


def test_explicit_velocity_tolerance_replaces_controller_default(live):
    goal, _, spec, _ = live
    # 0 would mean "unspecified" -> the installed controller's stopped_velocity_tolerance 0.01
    assert all(t.velocity == lc.GOAL_VELOCITY_TOLERANCE_RAD_S != 0.0 for t in goal.goal_tolerance)
    assert all(t['velocity'] == 0.05 for t in spec['goal_tolerance'])


def test_m60_mock_goal_builder_is_unchanged(traj, sources):
    goal, _ = ac.build_goal(traj, sources)
    assert all(t.velocity == 0.0 for t in goal.goal_tolerance)       # composition, not edit


def test_fingerprint_is_deterministic(traj, sources):
    a = gf.build_live_goal(traj, sources)[3]
    b = gf.build_live_goal(m6t.build_trajectory(sources), sources)[3]
    assert a == b
    assert gf.canonical(gf.approved_spec(traj, sources)) == \
        gf.canonical(gf.approved_spec(traj, sources))


def _mutate(goal, what):
    g = copy.deepcopy(goal)
    t = g.trajectory
    if what == 'time':
        t.points[1].time_from_start.nanosec = 1
    elif what == 'position':
        t.points[1].positions[2] = t.points[1].positions[2] + 1e-9
    elif what == 'velocity':
        t.points[2].velocities[0] = 0.001
    elif what == 'joint_order':
        t.joint_names[0], t.joint_names[1] = t.joint_names[1], t.joint_names[0]
    elif what == 'joint_name':
        t.joint_names[0] = 'lf_hip_x'
    elif what == 'path_tolerance':
        g.path_tolerance[3].position = 0.06
    elif what == 'goal_tolerance_position':
        g.goal_tolerance[3].position = 0.04
    elif what == 'goal_velocity_tolerance':
        g.goal_tolerance[3].velocity = 0.01
    elif what == 'goal_velocity_unspecified':
        for tol in g.goal_tolerance:
            tol.velocity = 0.0
    elif what == 'goal_time_tolerance':
        g.goal_time_tolerance.sec = 2
    elif what == 'header_stamp':
        t.header.stamp.sec = 5
    elif what == 'added_point':
        t.points.append(copy.deepcopy(t.points[-1]))
    elif what == 'missing_point':
        t.points.pop()
    elif what == 'acceleration':
        t.points[0].accelerations = [0.0] * 12
    elif what == 'tolerance_dropped':
        g.path_tolerance.pop()
    elif what == 'neutral_return':      # a "return to neutral" goal: one neutral point
        del t.points[1:]
    return g


MUTATIONS = ['time', 'position', 'velocity', 'joint_order', 'joint_name', 'path_tolerance',
             'goal_tolerance_position', 'goal_velocity_tolerance', 'goal_velocity_unspecified',
             'goal_time_tolerance', 'header_stamp', 'added_point', 'missing_point',
             'acceleration', 'tolerance_dropped', 'neutral_return']


@pytest.mark.parametrize('what', MUTATIONS)
def test_every_altered_goal_field_is_rejected(what, live, traj):
    goal, _, _, fp = live
    assert gf.verify_goal(goal, gf.binding(traj), fp) == fp
    with pytest.raises(gf.FingerprintError) as e:
        gf.verify_goal(_mutate(goal, what), gf.binding(traj), fp)
    assert e.value.code == 'goal_fingerprint_mismatch'


@pytest.mark.parametrize('key, value', [
    ('trajectory_id', '0' * 16),
    ('mode', 'cyclic'),
    ('goal_count', 2),
    ('inputs_sha256', {'spiderx_poses.yaml': 'f' * 64}),
])
def test_altered_binding_is_rejected(key, value, live, traj):
    goal, _, _, fp = live
    bind = gf.binding(traj)
    bind[key] = value
    with pytest.raises(gf.FingerprintError):
        gf.verify_goal(goal, bind, fp)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_non_finite_values_are_rejected(bad, live, traj):
    goal, _, _, fp = live
    g = copy.deepcopy(goal)
    g.trajectory.points[1].positions[0] = bad
    with pytest.raises(gf.FingerprintError) as e:
        gf.verify_goal(g, gf.binding(traj), fp)
    assert e.value.code == 'non_finite_value'


def test_non_approved_trajectory_cannot_be_specified(traj, sources):
    t = copy.deepcopy(traj)
    t['points'][2]['time_from_start_s'] = 9.5
    t['trajectory_id'] = m6t.compute_trajectory_id(t)
    assert m6t.preflight(t, sources).ok                    # a valid trajectory, but not approved
    with pytest.raises(gf.FingerprintError) as e:
        gf.build_live_goal(t, sources)
    assert e.value.code == 'not_approved_content'


def test_refused_trajectory_cannot_be_specified(traj, sources):
    t = copy.deepcopy(traj)
    t['points'][1]['positions'][2] = 0.2
    t['trajectory_id'] = m6t.compute_trajectory_id(t)
    with pytest.raises(gf.FingerprintError) as e:
        gf.approved_spec(t, sources)
    assert e.value.code == 'preflight_refused'


# ---------------------------------------------------------------- classification
NAMES, NEUTRAL = lp.contract_from_config(SRC_CONFIG)
GOOD_INTERFACE = {'action_file_sha256': 'x',
                  'goal_fields': ['goal_time_tolerance', 'goal_tolerance', 'path_tolerance',
                                  'trajectory'],
                  'tolerance_fields': ['acceleration', 'name', 'position', 'velocity'],
                  'error_codes': dict(lp.REQUIRED_ERROR_CODES)}


def obs(**kw):
    o = lp.Observations(
        action_servers=[('/leg_trajectory_controller', [lp.env.ACTION_TYPE])],
        controllers=[('joint_state_broadcaster', 'x', 'active'),
                     ('leg_trajectory_controller', lp.JTC_TYPE, 'active')],
        joint_state_publishers=[('/joint_state_broadcaster', lp.JOINT_STATE_TYPE)],
        joint_state_messages=[(1.0 + 0.01 * i, list(NAMES), [0.0] * 12) for i in range(4)],
        versions=dict(lp.REFERENCE_VERSIONS), interface=dict(GOOD_INTERFACE))
    for k, v in kw.items():
        setattr(o, k, v)
    return o


def report(**kw):
    return lp.evaluate(obs(**kw), NAMES, NEUTRAL)


def test_classification_compatible():
    assert rd.classify(report()) == (rd.COMPATIBLE, [])


def test_classification_warning_for_version_difference_only():
    label, reasons = rd.classify(report(versions=dict(lp.REFERENCE_VERSIONS,
                                                      joint_trajectory_controller='2.54.0')))
    assert label == rd.WARNING and 'joint_trajectory_controller' in reasons[0]


def test_owner_pc_versions_classify_as_warning():
    local = dict(lp.REFERENCE_VERSIONS, joint_trajectory_controller='2.54.0',
                 control_msgs='4.9.0', controller_manager='2.54.2',
                 controller_manager_msgs='2.54.2', hardware_interface='2.54.2',
                 gz_ros2_control='0.7.21', rclpy='3.3.19', action_msgs='1.2.2')
    label, reasons = rd.classify(report(versions=local))
    assert label == rd.WARNING and len(reasons) == 8


@pytest.mark.parametrize('kw', [
    dict(action_servers=[]),
    dict(action_servers=[('/a', ['x/Other'])]),
    dict(action_servers=[('/a', [lp.env.ACTION_TYPE]), ('/b', [lp.env.ACTION_TYPE])]),
    dict(controllers=[('joint_state_broadcaster', 'x', 'active'),
                      ('leg_trajectory_controller', 'other/Type', 'active')]),
    dict(joint_state_messages=[(1.0 + i, NAMES[:11] + ['x'], [0.0] * 12) for i in range(3)]),
    dict(joint_state_publishers=[('/jsb', 'std_msgs/msg/String')]),
    dict(interface={**GOOD_INTERFACE, 'goal_fields': ['trajectory']}),
])
def test_contract_failures_are_incompatible_even_with_matching_versions(kw):
    assert rd.classify(report(**kw))[0] == rd.INCOMPATIBLE


def test_interface_only_report_is_incompatible():
    o = obs()
    o.graph_probed = False
    assert rd.classify(lp.evaluate(o, NAMES, NEUTRAL))[0] == rd.INCOMPATIBLE


def test_missing_report_is_incompatible():
    assert rd.classify(None)[0] == rd.INCOMPATIBLE


# ---------------------------------------------------------------- readiness freshness
def test_fresh_ready_warning_permits_dispatch():
    r = rd.assess(report(versions=dict(lp.REFERENCE_VERSIONS, rclpy='3.3.19')), 100.0)
    assert r.label == rd.WARNING
    assert rd.dispatch_permitted(r, 110.0) == (True, None)          # exactly 10 s old


@pytest.mark.parametrize('now, code', [(110.000001, 'readiness_stale'), (99.0, 'readiness_stale')])
def test_stale_or_future_readiness_is_refused(now, code):
    assert rd.dispatch_permitted(rd.assess(report(), 100.0), now) == (False, code)


def test_incompatible_readiness_is_refused():
    r = rd.assess(report(action_servers=[]), 100.0)
    assert rd.dispatch_permitted(r, 100.0) == (False, 'stack_incompatible')


def test_not_ready_readiness_is_refused():
    r = rd.assess(report(joint_state_messages=[(1.0 + i, NAMES, [0.0] * 11 + [0.2])
                                               for i in range(3)]), 100.0)
    assert r.label == rd.COMPATIBLE and not r.ready
    assert rd.dispatch_permitted(r, 100.0) == (False, 'readiness_not_ready')


def test_missing_readiness_is_refused():
    assert rd.dispatch_permitted(None, 1.0) == (False, 'readiness_missing')


# ---------------------------------------------------------------- typed confirmation
def _reader(value):
    def read():
        if isinstance(value, BaseException):
            raise value
        return value
    return read


@pytest.mark.parametrize('value, ok', [
    (lc.CONFIRMATION_WORD, True),
    (lc.CONFIRMATION_WORD + '\n', True),
    (lc.CONFIRMATION_WORD + '\r\n', True),
    ('', False), ('\n', False), ('   \n', False), (' ' + lc.CONFIRMATION_WORD, False),
    (lc.CONFIRMATION_WORD + ' ', False), (lc.CONFIRMATION_WORD.lower(), False),
    ('yes', False), ('y', False), (lc.CONFIRMATION_WORD + '\n\n', False),
    (EOFError(), False), (KeyboardInterrupt(), False), (OSError('closed'), False),
    (None, False),
])
def test_confirmation_parser(value, ok):
    assert lc.parse_confirmation(_reader(value)) is ok


# ---------------------------------------------------------------- protected files unchanged
PINNED_SHA256 = {
    'spiderx_controller/trajectory_client.py':
        'b5bf9a96508d115d60db76d5ea5b7d0b163a832f1c7bb0f06a00cccadfbc8d58',
    'spiderx_controller/joint_safety.py':
        '08eaefccb4b65a91c94cb90ab2faa65133f677f95a11e73da9558f7cf8c75ceb',
    'spiderx_controller/m4_pose_validation.py':
        '0d8624c83ae1ba2decc7d7aac597a11e3b3cf53282495d315677349d78476297',
    'spiderx_controller/kinematics_validation.py':
        '98e02e8ed51aee50465448a3d8b33f60ea6c9c963a5001bf9438cfc98d12ee29',
    'scripts/test_one_joint.py':
        'fa4a898d9ae6e6de24fa8dc0146b62b7b9ea2f5fc84e9bc9c550834b2b627619',
    'scripts/test_neutral_pose.py':
        '76a6d92414e5401a86247573af081bed10b74b25dfe75897435b58c298c151a5',
    'scripts/run_posture_hold_test.py':
        'cd7926028c90b3cf8c4ebcdf4c5a52ebc0ed50be2ee9449513b5e7e80b34f69d',
    'scripts/validate_leg_kinematics':
        'a5bcde9f50bb7425eba27a025dc700676b72b60a60cccaa66a071d7b8a174439',
    'scripts/m4_pose_validation':
        'f3aa22b53f65dbd3211292f2b26d6d9fb09a598eda941e54e8da6476635e72dd',
    'spiderx_controller/m6_action_client.py':
        '53ea435c3aaa9056e2770c2cde6209cba10f468ed96917366f0f6784ed75a6b9',
    'spiderx_controller/m6_trajectory.py':
        'a1bea1f781e04fd4af7a29bfdfebdb6161b8be983712e698c852f7df2efb9c83',
    'spiderx_controller/m6_envelope.py':
        '095bc40f802782d30103bce82d1433651d40e8fd977e0724a760bf3ed159218c',
    'spiderx_controller/m6_live_preflight.py':
        '3c32c04925409ed31c2a3613ecbde0b4dbbd431b62d9d0637d162d201dbb4611',
    'spiderx_controller/m6_offline_preflight.py':
        '2e558fe3edfd45fd1d531975471703625d5dbf9f0f73cc0cd17dbf85ba0c6fd0',
    'config/spiderx_ros2_controllers.yaml':
        'c449f24f2757d83fcf466dbbbaf2ebe0d4e0ef51cdfde91351aadfeba6496e01',
    'launch/controller.launch.py':
        'c0a5b78a22600d14cd261c05f9c210f0c2b43b9510f52b7dae896a2ebc5b0077',
}


@pytest.mark.parametrize('rel', sorted(PINNED_SHA256))
def test_existing_tools_and_controller_config_are_unchanged(rel):
    with open(os.path.join(PKG, rel), 'rb') as f:
        assert hashlib.sha256(f.read()).hexdigest() == PINNED_SHA256[rel], rel


def test_new_modules_import_no_ros_client():
    import subprocess
    import sys
    code = ('import sys, spiderx_controller.m6_live_contract, spiderx_controller.m6_live_readiness,'
            ' spiderx_controller.m6_goal_fingerprint;'
            'bad=[m for m in sys.modules if m.split(".")[0] == "rclpy"];'
            'print(bad); sys.exit(1 if bad else 0)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr


def test_no_nan_in_spec_by_construction(traj, sources):
    spec = gf.approved_spec(traj, sources)
    assert all(math.isfinite(v) for v in gf._walk_numbers(spec))
