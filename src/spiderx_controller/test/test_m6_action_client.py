"""colcon test: M6.0 action-client safety behaviour against a deterministic test double.

No live action server, no ROS graph: the session talks only to m6_mock_action.MockActionServer.
Proves: only a preflighted goal is ever constructed or dispatched; at most one goal per session;
no retry; no automatic return-to-neutral; rejection, abort and timeout become structured failed
outcomes; cancel means "cancel only, hold position"; the goal carries 0.05 rad path/goal
tolerances. Mutation tests show that removing a gate, adding a retry or adding an automatic
return goal is detected.
"""
import copy
import math
import os
import re

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
import m6_mock_action as mock
import pytest
import yaml

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


@pytest.fixture
def traj(sources):
    return m6t.build_trajectory(sources)


def restamp(t):
    t['trajectory_id'] = m6t.compute_trajectory_id(t)
    return t


def mutated(traj, path, value, stamp=True):
    t = copy.deepcopy(traj)
    node = t
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value
    return restamp(t) if stamp else t


def invalid_trajectories(traj):
    """Malformed, incomplete, NaN, non-monotonic, out-of-limit, stale, tampered, cyclic."""
    t_incomplete = copy.deepcopy(traj)
    t_incomplete['points'][1]['positions'].pop()
    return {
        'malformed': restamp({**copy.deepcopy(traj), 'repeat': 3}),
        'incomplete': restamp(t_incomplete),
        'nan': mutated(traj, ['points', 1, 'positions', 2], float('nan')),
        'non_monotonic': mutated(traj, ['points', 2, 'time_from_start_s'], 5.0),
        'out_of_limit': mutated(traj, ['points', 1, 'positions', 2], 0.6),
        'beyond_cap': mutated(traj, ['points', 1, 'positions', 2], 0.1224),
        'stale': mutated(traj, ['provenance', 'inputs_sha256', 'spiderx_legs.yaml'], 'f' * 64),
        'tampered_id': mutated(traj, ['points', 2, 'time_from_start_s'], 9.5, stamp=False),
        'cyclic': mutated(traj, ['mode'], 'cyclic'),
        'not_a_mapping': [1, 2, 3],
    }


def run(sources, trajectory, **server):
    srv = mock.MockActionServer(**server)
    session = ac.PlaybackSession(srv, sources, result_timeout_s=5.0)
    return session, srv, session.run(trajectory)


# ---------------------------------------------------------------- safety-property checkers
# Each returns a list of violations. The real tests assert []; the mutation tests assert that a
# deliberately broken client produces violations, proving the checkers can fail.
def dispatch_violations(sources, traj):
    out = []
    for name, bad in invalid_trajectories(traj).items():
        srv = mock.MockActionServer()
        try:
            outcome = ac.PlaybackSession(srv, sources, result_timeout_s=5.0).run(bad)
        except Exception as e:  # noqa: BLE001 - a crash is a violation, sent goals still count
            outcome = None
            out.append(f'{name}: session raised {type(e).__name__}')
        if srv.sent or any(c[0] == 'send_goal' for c in srv.calls):
            out.append(f'{name}: dispatched')
        if outcome is not None and (outcome.result != 'refused' or outcome.goals_sent != 0):
            out.append(f'{name}: outcome {outcome.result}, goals_sent {outcome.goals_sent}')
    return out


def build_goal_violations(sources, traj):
    out = []
    for name, bad in invalid_trajectories(traj).items():
        try:
            ac.build_goal(bad, sources)
            out.append(f'{name}: goal constructed')
        except ac.PreflightRefused:
            pass
        except Exception as e:  # noqa: BLE001 - an unclean refusal is a violation too
            out.append(f'{name}: unexpected {type(e).__name__}')
    return out


SCENARIOS = {
    'rejected': dict(accept=False),
    'aborted_invalid_goal': dict(result=(ac.STATUS_ABORTED, -1, 'invalid')),
    'aborted_path_tolerance': dict(result=(ac.STATUS_ABORTED, -4, 'path')),
    'aborted_goal_tolerance': dict(result=(ac.STATUS_ABORTED, -5, 'goal')),
    'canceled_status': dict(result=(ac.STATUS_CANCELED, 0, '')),
    'timeout': dict(result=None),
    'no_response': dict(respond=False),
    'interrupt': dict(wait_raises=KeyboardInterrupt()),
    'adapter_error': dict(wait_raises=RuntimeError('transport lost')),
    'tracking_failure': dict(samples='offset'),
    'server_unavailable': dict(ready=False),
}


def scenario_server_args(name, traj):
    args = dict(SCENARIOS[name])
    if args.get('samples') == 'offset':
        args['samples'] = mock.perfect_samples(traj, offset=0.08)
    elif 'samples' not in args:
        args['samples'] = mock.perfect_samples(traj)
    return args


def single_goal_violations(sources, traj, session_cls=ac.PlaybackSession):
    """No retry, no automatic return goal, no later goal, in every failure scenario."""
    out = []
    expected, _ = ac.build_goal(traj, sources)
    for name in SCENARIOS:
        srv = mock.MockActionServer(**scenario_server_args(name, traj))
        session = session_cls(srv, sources, result_timeout_s=5.0)
        session.run(traj)
        if len(srv.sent) > 1:
            out.append(f'{name}: {len(srv.sent)} goals sent')
        for g in srv.sent:
            if g != expected:
                out.append(f'{name}: a goal other than the preflighted one was sent')
        try:
            session.run(traj)
            out.append(f'{name}: session accepted a second run')
        except ac.SecondGoalForbidden:
            pass
        if len(srv.sent) > 1:
            out.append(f'{name}: a later goal followed')
    return out


# ---------------------------------------------------------------- goal construction
def test_valid_preflighted_goal_is_constructed_correctly(traj, sources):
    goal, report = ac.build_goal(traj, sources)
    assert report.ok
    assert isinstance(goal, FollowJointTrajectory.Goal)
    assert list(goal.trajectory.joint_names) == list(sources.joint_names)
    assert goal.trajectory.header.stamp.sec == 0 and goal.trajectory.header.stamp.nanosec == 0
    assert len(goal.trajectory.points) == 3
    for p, src in zip(goal.trajectory.points, traj['points']):
        assert list(p.positions) == src['positions']
        assert list(p.velocities) == [0.0] * 12
        assert p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 == \
            src['time_from_start_s']
        assert list(p.accelerations) == [] and list(p.effort) == []
    assert [(d.time_from_start.sec, d.time_from_start.nanosec)
            for d in goal.trajectory.points] == [(3, 0), (6, 0), (9, 0)]


def test_goal_carries_explicit_0_05_rad_tolerances(traj, sources):
    goal, _ = ac.build_goal(traj, sources)
    for tols in (goal.path_tolerance, goal.goal_tolerance):
        assert [t.name for t in tols] == list(sources.joint_names)
        for t in tols:
            assert isinstance(t, JointTolerance)
            assert t.position == env.TRACKING_TOLERANCE_RAD == 0.05
            assert t.velocity == 0.0 and t.acceleration == 0.0     # 0 = unspecified, legal
    assert goal.goal_time_tolerance == Duration(sec=1, nanosec=0)
    assert env.GOAL_TIME_TOLERANCE_S == env.SETTLE_S == 1.0
    assert list(goal.component_path_tolerance) == [] and list(goal.component_goal_tolerance) == []
    assert list(goal.multi_dof_trajectory.joint_names) == []


def test_tolerances_are_legal_for_the_installed_controller(traj, sources):
    # The installed joint_trajectory_controller silently falls back to its unchecked defaults if a
    # tolerance names an unknown joint or uses a negative value other than -1 (tolerances.hpp).
    with open(os.path.join(SRC_CONFIG, 'spiderx_ros2_controllers.yaml')) as f:
        joints = yaml.safe_load(f)['leg_trajectory_controller']['ros__parameters']['joints']
    goal, _ = ac.build_goal(traj, sources)
    for t in list(goal.path_tolerance) + list(goal.goal_tolerance):
        assert t.name in joints
        assert t.position > 0 and t.velocity >= 0 and t.acceleration >= 0
    assert goal.goal_time_tolerance.sec + goal.goal_time_tolerance.nanosec * 1e-9 > 0


def test_controller_yaml_is_not_the_tolerance_source():
    with open(os.path.join(SRC_CONFIG, 'spiderx_ros2_controllers.yaml')) as f:
        params = yaml.safe_load(f)['leg_trajectory_controller']['ros__parameters']
    assert 'constraints' not in params      # tolerances live only in the goal message (D3)


def test_tracking_tolerance_and_cap_are_not_swapped(traj, sources):
    goal, _ = ac.build_goal(traj, sources)
    assert all(t.position != env.M6_0_D_MAX_DISPLACEMENT_RAD for t in goal.goal_tolerance)


def test_duration_conversion():
    assert ac.seconds_to_duration_fields(3.0) == (3, 0)
    assert ac.seconds_to_duration_fields(0.02) == (0, 20_000_000)
    assert ac.seconds_to_duration_fields(9.5) == (9, 500_000_000)


@pytest.mark.parametrize('name', ['malformed', 'incomplete', 'nan', 'non_monotonic',
                                  'out_of_limit', 'beyond_cap', 'stale', 'tampered_id',
                                  'cyclic', 'not_a_mapping'])
def test_build_goal_refuses_every_invalid_input(name, traj, sources):
    with pytest.raises(ac.PreflightRefused) as e:
        ac.build_goal(invalid_trajectories(traj)[name], sources)
    assert e.value.report.codes


def test_no_invalid_input_ever_reaches_goal_construction(traj, sources):
    assert build_goal_violations(sources, traj) == []


# ---------------------------------------------------------------- dispatch gate
def test_no_invalid_input_ever_reaches_dispatch(traj, sources):
    assert dispatch_violations(sources, traj) == []


@pytest.mark.parametrize('name', ['nan', 'non_monotonic', 'out_of_limit', 'stale',
                                  'incomplete', 'malformed'])
def test_refused_input_never_calls_the_server_at_all(name, traj, sources):
    _, srv, outcome = run(sources, invalid_trajectories(traj)[name])
    assert srv.calls == [] and srv.sent == [] and srv.cancels == []
    assert outcome.result == 'refused' and outcome.state == 'refused'
    assert outcome.channels == {'action': 'unavailable', 'tracking': 'unavailable',
                                'goal_tolerances': 'unavailable'}
    assert outcome.hold_position is False
    assert outcome.preflight['ok'] is False


def test_sources_with_errors_refuse_everything(traj):
    bad = m6t.Sources(config_dir='x', errors=(('neutral_source_missing', 'missing'),))
    _, srv, outcome = run(bad, traj)
    assert srv.calls == [] and outcome.result == 'refused'
    assert 'neutral_source_missing' in outcome.preflight['failure_codes']


def test_unavailable_server_sends_nothing(traj, sources):
    _, srv, outcome = run(sources, traj, ready=False)
    assert srv.sent == [] and outcome.result == 'refused' and outcome.goals_sent == 0


# ---------------------------------------------------------------- outcomes
def test_successful_playback_passes_all_three_channels(traj, sources):
    session, srv, outcome = run(sources, traj, samples=mock.perfect_samples(traj))
    assert outcome.result == 'succeeded' and outcome.passed
    assert outcome.channels == {'action': 'passed', 'tracking': 'passed',
                                'goal_tolerances': 'passed'}
    assert len(srv.sent) == 1 and srv.cancels == []
    assert outcome.goals_sent == 1 and outcome.hold_position is False
    assert outcome.tracking['max_error_rad'] < 1e-12
    assert session.state == 'succeeded'


def test_action_success_alone_is_not_tracking_evidence(traj, sources):
    _, _, outcome = run(sources, traj, samples=[])
    assert outcome.channels['action'] == 'passed'
    assert outcome.channels['tracking'] == 'unavailable'
    assert outcome.result == 'failed' and not outcome.passed


def test_tracking_error_above_0_05_rad_fails_the_tracking_channel(traj, sources):
    _, srv, outcome = run(sources, traj, samples=mock.perfect_samples(traj, offset=0.051))
    assert outcome.channels == {'action': 'passed', 'tracking': 'failed',
                                'goal_tolerances': 'passed'}
    assert outcome.result == 'failed' and outcome.hold_position is True
    assert len(srv.sent) == 1


def test_tracking_error_below_0_05_rad_passes(traj, sources):
    _, _, outcome = run(sources, traj, samples=mock.perfect_samples(traj, offset=0.049))
    assert outcome.channels['tracking'] == 'passed'


@pytest.mark.parametrize('code, name', [(-1, 'INVALID_GOAL'), (-2, 'INVALID_JOINTS'),
                                        (-3, 'OLD_HEADER_TIMESTAMP')])
def test_aborted_results_are_structured_failures(code, name, traj, sources):
    _, srv, outcome = run(sources, traj, result=(ac.STATUS_ABORTED, code, 'x'),
                          samples=mock.perfect_samples(traj))
    d = outcome.to_dict()
    assert d['result'] == 'failed' and d['passed'] is False
    assert d['channels']['action'] == 'failed'
    assert d['channels']['goal_tolerances'] == 'unavailable'
    assert d['error_code'] == code and d['error_name'] == name
    assert len(srv.sent) == 1 and d['automatic_return_goals'] == 0 and d['retries'] == 0


@pytest.mark.parametrize('code', [-4, -5])
def test_tolerance_violations_fail_the_tolerance_channel(code, traj, sources):
    _, _, outcome = run(sources, traj, result=(ac.STATUS_ABORTED, code, 'tol'),
                        samples=mock.perfect_samples(traj))
    assert outcome.channels['action'] == 'failed'
    assert outcome.channels['goal_tolerances'] == 'failed'


def test_rejected_goal_is_a_structured_failure_without_retry(traj, sources):
    session, srv, outcome = run(sources, traj, accept=False)
    assert outcome.result == 'rejected' and outcome.channels['action'] == 'failed'
    assert len(srv.sent) == 1 and srv.cancels == []
    assert session.state == 'rejected'
    assert not any(c[0] == 'wait_result' for c in srv.calls)


def test_timeout_cancels_only_and_holds(traj, sources):
    session, srv, outcome = run(sources, traj, result=None)
    assert outcome.result == 'timed_out'
    assert outcome.channels['action'] == 'timed_out'
    assert outcome.channels['goal_tolerances'] == 'timed_out'
    assert len(srv.cancels) == 1 and len(srv.sent) == 1
    assert outcome.hold_position is True and session.state == 'timed_out'
    assert any('no return-to-neutral' in e for e in outcome.events)


def test_unanswered_goal_request_is_timed_out_without_further_goals(traj, sources):
    _, srv, outcome = run(sources, traj, respond=False)
    assert outcome.result == 'timed_out' and len(srv.sent) == 1 and srv.cancels == []


def test_interrupt_is_cancel_only_hold_position(traj, sources):
    session, srv, outcome = run(sources, traj, wait_raises=KeyboardInterrupt())
    assert outcome.result == 'canceled' and session.state == 'canceled_holding'
    assert len(srv.cancels) == 1 and len(srv.sent) == 1
    assert outcome.hold_position is True


def test_adapter_error_is_cancel_only_hold_position(traj, sources):
    session, srv, outcome = run(sources, traj, wait_raises=RuntimeError('transport lost'))
    assert outcome.result == 'error' and session.state == 'error_holding'
    assert 'transport lost' in outcome.reason
    assert len(srv.cancels) == 1 and len(srv.sent) == 1


def test_send_error_holds_without_further_goals(traj, sources):
    _, srv, outcome = run(sources, traj, send_raises=RuntimeError('send failed'))
    assert outcome.result == 'error' and len(srv.sent) == 1 and srv.cancels == []


def test_external_cancel_is_cancel_only(traj, sources):
    srv = mock.MockActionServer(samples=mock.perfect_samples(traj))
    session = ac.PlaybackSession(srv, sources, result_timeout_s=5.0)
    assert session.cancel() == 'idle' and srv.cancels == []        # nothing to cancel yet
    session.state, session._handle = 'executing', mock.MockHandle(True, 1)
    assert session.cancel() == 'canceled_holding'
    assert len(srv.cancels) == 1 and srv.sent == []


def test_every_failure_scenario_sends_at_most_the_one_preflighted_goal(traj, sources):
    assert single_goal_violations(sources, traj) == []


def test_session_is_single_use_even_after_success(traj, sources):
    session, srv, _ = run(sources, traj, samples=mock.perfect_samples(traj))
    with pytest.raises(ac.SecondGoalForbidden):
        session.run(traj)
    with pytest.raises(ac.SecondGoalForbidden):
        session._dispatch(ac.build_goal(traj, sources)[0])
    assert len(srv.sent) == 1


def test_refused_session_cannot_be_reused_for_a_valid_goal(traj, sources):
    srv = mock.MockActionServer(samples=mock.perfect_samples(traj))
    session = ac.PlaybackSession(srv, sources)
    assert session.run(invalid_trajectories(traj)['nan']).result == 'refused'
    with pytest.raises(ac.SecondGoalForbidden):
        session.run(traj)
    assert srv.sent == []


def test_default_watchdog_matches_existing_rule(traj, sources):
    session = ac.PlaybackSession(mock.MockActionServer(), sources)
    assert session.result_timeout_for(traj) == 10.0 * 9.0 + 30.0


def test_outcome_dict_is_complete(traj, sources):
    _, _, outcome = run(sources, traj, samples=mock.perfect_samples(traj))
    d = outcome.to_dict()
    assert d['schema'] == ac.OUTCOME_SCHEMA
    assert set(d['channels']) == {'action', 'tracking', 'goal_tolerances'}
    assert all(v in ac.CHANNEL_VALUES for v in d['channels'].values())
    assert d['non_claims'] == list(env.NON_CLAIMS)


# ---------------------------------------------------------------- tracking evaluation
def test_reference_is_cubic_between_zero_velocity_waypoints(traj):
    q1 = traj['points'][1]['positions']
    assert ac.reference_positions(traj, 2.9) is None
    assert ac.reference_positions(traj, 3.0) == traj['points'][0]['positions']
    mid = ac.reference_positions(traj, 4.5)
    assert all(abs(m - 0.5 * q) < 1e-15 for m, q in zip(mid, q1))
    assert ac.reference_positions(traj, 6.0) == q1
    assert ac.reference_positions(traj, 50.0) == traj['points'][2]['positions']


def test_tracking_requires_coverage_of_every_segment_and_the_end(traj):
    samples = [s for s in mock.perfect_samples(traj) if not 6.0 <= s[0] < 9.0]
    state, detail = ac.evaluate_tracking(traj, samples)
    assert state == 'failed' and detail['uncovered_intervals'] == [1]


def test_tracking_with_incomplete_or_nan_samples_fails(traj):
    samples = mock.perfect_samples(traj)
    t, obs = samples[20]
    samples[20] = (t, {k: v for k, v in obs.items() if k != 'rr_hip'})
    assert ac.evaluate_tracking(traj, samples)[0] == 'failed'
    samples = mock.perfect_samples(traj)
    samples[20][1]['lf_hip'] = math.nan
    assert ac.evaluate_tracking(traj, samples)[0] == 'failed'


def test_tracking_before_first_waypoint_is_not_judged(traj):
    samples = [(0.5, {n: 0.04 for n in traj['joint_names']})] + mock.perfect_samples(traj)[13:]
    assert ac.evaluate_tracking(traj, samples)[0] == 'passed'


# ---------------------------------------------------------------- installed interface contract
def test_action_endpoint_and_type_match_the_controller():
    with open(os.path.join(SRC_CONFIG, 'spiderx_ros2_controllers.yaml')) as f:
        cfg = yaml.safe_load(f)
    ctrl = cfg['controller_manager']['ros__parameters'][env.CONTROLLER_NAME]
    assert ctrl['type'] == 'joint_trajectory_controller/JointTrajectoryController'
    assert env.ACTION_NAME == f'/{env.CONTROLLER_NAME}/follow_joint_trajectory'
    assert env.ACTION_TYPE == 'control_msgs/action/FollowJointTrajectory'
    assert set(env.REQUIRED_CONTROLLERS) <= set(cfg['controller_manager']['ros__parameters'])


def test_installed_action_definition_has_the_used_fields():
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('control_msgs'), 'action',
                        'FollowJointTrajectory.action')
    with open(path) as f:
        text = f.read()
    for fld in ('trajectory_msgs/JointTrajectory trajectory', 'JointTolerance[] path_tolerance',
                'JointTolerance[] goal_tolerance',
                'builtin_interfaces/Duration goal_time_tolerance',
                'int32 error_code', 'string error_string'):
        assert re.search(r'\b' + re.escape(fld.split()[-1]) + r'\b', text), fld
    fields = FollowJointTrajectory.Goal.get_fields_and_field_types()
    assert {'trajectory', 'path_tolerance', 'goal_tolerance', 'goal_time_tolerance'} <= set(fields)


def test_error_codes_match_the_installed_result():
    r = FollowJointTrajectory.Result
    assert {getattr(r, name): name for name in ac.ERROR_CODES.values()} == ac.ERROR_CODES


def test_goal_status_codes_match_the_installed_message():
    from action_msgs.msg import GoalStatus
    assert (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED) \
        == (ac.STATUS_SUCCEEDED, ac.STATUS_CANCELED, ac.STATUS_ABORTED)


def test_installed_controller_honours_goal_tolerances():
    """Where the installed headers exist, the goal-tolerance path must be present (D3)."""
    prefix = os.environ.get('CONDA_PREFIX') or ''
    candidates = [os.path.join(p, 'include', 'joint_trajectory_controller',
                               'joint_trajectory_controller', 'tolerances.hpp')
                  for p in {prefix, '/opt/ros/humble', '/opt/mm/root/envs/humble'} if p]
    found = [c for c in candidates if os.path.isfile(c)]
    if not found:
        pytest.skip('joint_trajectory_controller headers not installed here')
    with open(found[0]) as f:
        text = f.read()
    assert 'get_segment_tolerances' in text and 'goal.goal_tolerance' in text
    assert 'goal.path_tolerance' in text and 'goal.goal_time_tolerance' in text


# ---------------------------------------------------------------- mutation tests
def test_mutation_removed_dispatch_gate_is_detected(traj, sources, monkeypatch):
    """Remove BOTH gates (session gate + goal-construction preflight): invalid input reaches the
    server, and the checker reports it."""
    monkeypatch.setattr(ac, '_dispatch_allowed', lambda report, t: True)

    def unchecked_build_goal(trajectory, srcs):
        from control_msgs.action import FollowJointTrajectory as F
        return F.Goal(), m6t.preflight(traj, srcs)
    monkeypatch.setattr(ac, 'build_goal', unchecked_build_goal)
    assert dispatch_violations(sources, traj) != []


def test_mutation_removed_session_gate_is_detected_by_spy(traj, sources, monkeypatch):
    """Removing only the session gate: the goal builder is then reached with invalid input."""
    calls = []
    real = ac.build_goal

    def spy(trajectory, srcs):
        calls.append(trajectory)
        return real(trajectory, srcs)
    monkeypatch.setattr(ac, 'build_goal', spy)
    run(sources, invalid_trajectories(traj)['nan'])
    assert calls == []                                  # real gate: never reached
    monkeypatch.setattr(ac, '_dispatch_allowed', lambda report, t: True)
    run(sources, invalid_trajectories(traj)['nan'])
    assert len(calls) == 1                              # mutant: reached -> detected


def test_mutation_removed_goal_construction_preflight_is_detected(traj, sources, monkeypatch):
    real_preflight = m6t.preflight

    def always_pass(trajectory, srcs):
        report = real_preflight(traj, srcs)
        return report
    monkeypatch.setattr(m6t, 'preflight', always_pass)
    assert build_goal_violations(sources, traj) != []


class RetryingSession(ac.PlaybackSession):
    """MUTANT: retries a rejected goal."""

    def _handle_rejection(self, report):
        goal, _ = ac.build_goal(self._trajectory, self.sources)
        self.adapter.send_goal(goal)
        return super()._handle_rejection(report)


class AutoReturnSession(ac.PlaybackSession):
    """MUTANT: sends a return-to-neutral goal after a timeout."""

    def _handle_timeout(self, report):
        neutral = FollowJointTrajectory.Goal()
        neutral.trajectory.joint_names = list(self.sources.joint_names)
        self.adapter.send_goal(neutral)
        return super()._handle_timeout(report)


class AutoReturnOnCancelSession(ac.PlaybackSession):
    """MUTANT: re-sends the trajectory's neutral waypoint after an interrupt."""

    def _cancel_and_hold(self, why):
        super()._cancel_and_hold(why)
        goal, _ = ac.build_goal(self._trajectory, self.sources)
        del goal.trajectory.points[1:]
        self.adapter.send_goal(goal)


@pytest.mark.parametrize('mutant', [RetryingSession, AutoReturnSession,
                                    AutoReturnOnCancelSession])
def test_mutation_retry_or_auto_return_is_detected(mutant, traj, sources):
    assert single_goal_violations(sources, traj, mutant) != []


def test_unmutated_client_has_no_retry_or_return_path():
    import inspect
    src = inspect.getsource(ac)
    assert src.count('self.adapter.send_goal(') == 1     # the single call inside _dispatch
    assert inspect.getsource(ac.PlaybackSession._dispatch).count('send_goal(') == 1
    session_src = inspect.getsource(ac.PlaybackSession)
    for word in ('NEUTRAL_LABEL', 'poses', 'build_trajectory', 'while ', 'for attempt'):
        assert word not in session_src
