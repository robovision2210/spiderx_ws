"""colcon test: M6.1 protected trot gait replay (offline, mock and isolated-domain only).

No Gazebo, launch, controller or bridge. The committed M6.1 gate stays False; the gated --live
wiring is exercised only with the gate set inside a test (monkeypatch) against the in-memory M6.1
mock transport and fake graph observations, exactly as the M6.0-D live-enabling tests do. One
isolated-domain test (explicit domain 150-199, ROS_LOCALHOST_ONLY=1, no other node) checks the
read-only body-pose subscription of the rclpy transport against an in-test publisher.

Covers: the trajectory loads; the content hash verifies (and any edit is refused); the waypoints
re-derive from the approved parameters; the M6.1 envelope and limits; the goal fingerprint; every
gait gate triggers correctly (mock); evidence is recorded under log/m61_run/... (mock and gated
live wiring) and never overwritten.
"""
import dataclasses
import json
import math
import os
import re
import subprocess
import sys
import time

import m61_gate
import m6d_gate
import pytest

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_gait_replay as gr
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_mock as m6mock
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_live_preflight as lpf
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller import m61_evidence as evd
from spiderx_controller import m61_gates as gates
from spiderx_controller import m61_goal as goal61
from spiderx_controller import m61_limits
from spiderx_controller import m61_live_contract as c61
from spiderx_controller import m61_mock
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller import m61a_fixed_base as fb
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller.config_check import load_urdf

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC_CONFIG = os.path.join(PKG, 'config')
CONTENT_SHA256 = '94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64'
DESIGN_NOTE_SHA256 = 'd03e80a0e733ae756ed95433514aea5490ac627ffb6b18f5b47d2816174e876c'
TRAJECTORY_ID = '241760e7dfd5ef12'
FINGERPRINT = '9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a'
WORD = c61.CONFIRMATION_WORD + '\n'


@pytest.fixture(scope='module')
def plan():
    p = tc.load_plan(SRC_CONFIG, load_urdf())
    assert p.errors == (), p.errors
    return p


@pytest.fixture(scope='module')
def traj(plan):
    return tc.build_trajectory(plan)


def word():
    return lambda: WORD


def mutated(traj, fn):
    t = json.loads(json.dumps(traj))
    fn(t)
    return t


def codes(traj, plan):
    return set(tc.preflight(traj, plan).codes)


# ==================================================================== gate and contract
# Both build modes keep meaningful, passing tests. test/m61_gate.py is the ONE test-side pin of
# the committed gate (False on this branch; the enabling patch flips the gate and the pin and
# nothing else). The committed value is checked against the pin; the disabled behaviour is
# tested with the gate set False explicitly (fixture `disabled`), the enabled wiring with the
# gate set True explicitly and in-memory fakes only (fixture `enabled`). No test reaches a
# simulator or a controller in either mode.
@pytest.fixture
def disabled(monkeypatch):
    monkeypatch.setattr(c61, 'M61_LIVE_DISPATCH_ENABLED', False)      # tests only


def test_m61_gate_matches_the_single_test_expectation():
    assert isinstance(m61_gate.EXPECTED_M61_LIVE_DISPATCH_ENABLED, bool)
    assert c61.M61_LIVE_DISPATCH_ENABLED is m61_gate.EXPECTED_M61_LIVE_DISPATCH_ENABLED
    # the CLI's stated state follows the committed gate (computed at import)
    assert ('HARD-DISABLED' in gr.LIVE_STATE) is (not c61.M61_LIVE_DISPATCH_ENABLED)
    assert ('ENABLED for exactly one goal' in gr.LIVE_STATE) is c61.M61_LIVE_DISPATCH_ENABLED
    # the M6.0-D gate is separate and untouched by M6.1
    assert lc.LIVE_DISPATCH_ENABLED is m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED


def _package_sources():
    d = os.path.join(PKG, 'spiderx_controller')
    for name in sorted(os.listdir(d)):
        if name.endswith('.py'):
            with open(os.path.join(d, name)) as f:
                yield name, f.read()


def test_m61_gate_is_a_single_literal_equal_to_the_pin():
    hits = [(n, line) for n, src in _package_sources() for line in src.splitlines()
            if re.match(r'\s*(\w+\.)?M61_LIVE_DISPATCH_ENABLED\s*=', line)]
    assert hits == [('m61_live_contract.py',
                     f'M61_LIVE_DISPATCH_ENABLED = '
                     f'{m61_gate.EXPECTED_M61_LIVE_DISPATCH_ENABLED}')]


M61_GATE_FILES = ('m61_live_contract.py', 'm6_gait_replay.py', 'm61_goal.py', 'm61_gates.py',
                  'm61_trot_cycle.py', 'm61_limits.py', 'm61_live_adapter.py', 'm61_mock.py')
ENABLERS = ('os.environ', 'getenv', 'DISPATCH_ENABLED or', "add_argument('--enable",
            "add_argument('--yes'", "add_argument('--force'", 'DISPATCH_ENABLED = True')


def _enabler_hits(srcs, expected):
    """(file, what) for every way the M6.1 sources could enable dispatch other than THE gate
    line. That line must appear exactly once, in m61_live_contract.py, with the pinned value;
    only that exact line is exempt from the scan, so the scan means the same in both modes."""
    gate = f'M61_LIVE_DISPATCH_ENABLED = {expected}'
    hits = []
    for name in M61_GATE_FILES:
        lines = srcs[name].splitlines()
        if name == 'm61_live_contract.py':
            n = lines.count(gate)
            if n != 1:
                hits.append((name, f'{gate!r} found {n} times'))
            lines = [line for line in lines if line != gate]
        text = '\n'.join(lines)
        hits += [(name, w) for w in ENABLERS if w in text]
    return hits


def test_no_environment_or_flag_can_enable_m61_dispatch():
    assert _enabler_hits(dict(_package_sources()),
                         m61_gate.EXPECTED_M61_LIVE_DISPATCH_ENABLED) == []


@pytest.mark.parametrize('expected', [False, True])
def test_the_enabler_scan_has_teeth_in_both_build_modes(expected):
    gate = f'M61_LIVE_DISPATCH_ENABLED = {expected}'
    clean = dict({n: 'X = 1\n' for n in M61_GATE_FILES}, **{'m61_live_contract.py': gate})
    assert _enabler_hits(clean, expected) == []
    # the gate line with the other value, a missing or a duplicated gate line
    flipped = f'M61_LIVE_DISPATCH_ENABLED = {not expected}'
    for text in (flipped, 'X = 1', f'{gate}\n{gate}'):
        assert _enabler_hits(dict(clean, **{'m61_live_contract.py': text}), expected)
    # an indented or second assignment is never the exempt line
    for text in (f'{gate}\n    M61_LIVE_DISPATCH_ENABLED = True',
                 f'{gate}\nc61.M61_LIVE_DISPATCH_ENABLED = True'):
        assert ('m61_live_contract.py', 'DISPATCH_ENABLED = True') in _enabler_hits(
            dict(clean, **{'m61_live_contract.py': text}), expected)
    for name in M61_GATE_FILES[1:]:
        for w in ENABLERS:
            assert (name, w) in _enabler_hits(dict(clean, **{name: f'y = {w}'}), expected)


def test_only_the_m61_adapter_imports_rclpy():
    srcs = dict(_package_sources())
    for name in ('m61_live_contract.py', 'm6_gait_replay.py', 'm61_goal.py', 'm61_gates.py',
                 'm61_trot_cycle.py', 'm61_limits.py', 'm61_mock.py', 'm61_evidence.py'):
        assert 'import rclpy' not in srcs[name] and 'from rclpy' not in srcs[name], name


def test_live_cli_refused_exit_3_before_anything(disabled, tmp_path, monkeypatch, capsys):
    def never(*a, **k):
        raise AssertionError('must not be reached')
    monkeypatch.setattr(tc, 'load_plan', never)
    monkeypatch.setattr(gr, '_live_main', never)
    rc = gr.main(['m6_gait_replay', '--live', '--domain-id', '7', '--out', str(tmp_path)],
                 reader=never)
    assert rc == gr.EXIT_DISABLED == 3
    assert 'REFUSED' in capsys.readouterr().out and os.listdir(tmp_path) == []


def test_live_cli_with_gate_false_imports_no_ros_client():
    # the gate is set False inside the child, so this tests the disabled path in either build
    # mode and can never reach a ROS graph, whatever is running on domain 0
    code = ('import sys; from spiderx_controller import m6_gait_replay as m;'
            'm.c61.M61_LIVE_DISPATCH_ENABLED = False;'
            'rc = m.main(["m6_gait_replay", "--live", "--domain-id", "0"]);'
            'bad=[x for x in sys.modules if x.split(".")[0] == "rclpy"'
            ' or x.endswith("m61_live_adapter") or x.endswith("m6_live_adapter")'
            ' or x.endswith("m61a_live")];'
            'print(rc, bad); sys.exit(0 if rc == 3 and not bad else 1)')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                         stdin=subprocess.DEVNULL)
    assert out.returncode == 0, out.stdout + out.stderr
    assert 'REFUSED' in out.stdout and 'HARD-DISABLED' in out.stdout


def test_gate_false_blocks_the_live_wiring_too(disabled, plan, tmp_path):
    def never(*a, **k):
        raise AssertionError('must not be reached')
    with pytest.raises(PermissionError):
        gr._run_live(plan, never, never, never, lpb.InterruptLatch(), 0)
    args = gr.parse_args(['--live', '--domain-id', '1', '--out', str(tmp_path)])
    assert gr._live_main(args, plan, never) == gr.EXIT_DISABLED
    assert os.listdir(tmp_path) == []


@pytest.mark.parametrize('value, ok', [
    (c61.CONFIRMATION_WORD, True), (c61.CONFIRMATION_WORD + '\n', True),
    (c61.CONFIRMATION_WORD + '\r\n', True), ('', False), (' ', False),
    (lc.CONFIRMATION_WORD + '\n', False),                       # the M6.0-D word is refused
    (c61.CONFIRMATION_WORD.lower(), False), (' ' + c61.CONFIRMATION_WORD, False),
    (c61.CONFIRMATION_WORD + ' ', False), ('yes', False), (EOFError(), False),
    (KeyboardInterrupt(), False), (OSError('closed'), False), (None, False),
])
def test_confirmation_word(value, ok):
    def read():
        if isinstance(value, BaseException):
            raise value
        return value
    assert c61.CONFIRMATION_WORD == 'SEND-ONE-TROT-CYCLE'
    assert c61.parse_confirmation(read) is ok


def test_help_has_no_bypass(capsys):
    with pytest.raises(SystemExit):
        gr.parse_args(['--help'])
    out = capsys.readouterr().out
    for w in ('--yes', '--force', '--enable', '--trajectory', '--repeat', '--retry'):
        assert w not in out


# ==================================================================== limits (m61_limits.yaml)
def test_limits_file_is_the_approved_one(plan):
    lim = plan.limits
    assert lim.as_dict() == c61.APPROVED_LIMITS
    assert c61.limits_problems(lim) == []
    assert lim.MAX_TRAJECTORY_POINTS == 9
    assert lim.MIN_SEGMENT_DURATION == 0.5
    assert lim.MAX_JOINT_DISPLACEMENT_RAD == 0.1223 == env.M6_0_D_MAX_DISPLACEMENT_RAD
    assert lim.body_min_height_m == 0.045 and lim.base_constraint == 'fixed'
    assert lim.contact_sensing == 'not_measured'


def test_limits_agree_with_the_reused_m6d_monitor(plan):
    lim = plan.limits
    assert lim.sim_stall_s == lc.SIM_STALL_S
    assert lim.joint_state_gap_s == lc.SAMPLE_GAP_S
    assert lim.joint_states_stale_s == lc.JOINT_STATES_STALE_S
    assert lim.goal_time_tolerance_s == lc.GOAL_TIME_TOLERANCE_S
    assert lim.client_tracking_abort_rad == lc.CLIENT_TRACKING_ABORT_RAD


def test_m6d_envelope_is_not_widened():
    # M6.1 has its own limits; the M6.0-D crouch envelope keeps its values
    assert (env.MAX_POINTS, env.MIN_SEGMENT_S, env.M6_0_D_MAX_DISPLACEMENT_RAD) == (5, 3.0, 0.1223)


def _limits_data():
    from spiderx_controller.posture_config import load_yaml_strict
    return load_yaml_strict(os.path.join(SRC_CONFIG, m61_limits.FILE_NAME))


@pytest.mark.parametrize('edit, fragment', [
    (lambda d: d.update(extra=1), 'unknown'),
    (lambda d: d.pop('body_min_height_m'), 'missing'),
    (lambda d: d.update(max_trajectory_points=True), 'number'),
    (lambda d: d.update(max_trajectory_points=9.5), 'integer'),
    (lambda d: d.update(body_min_height_m=float('nan')), 'finite'),
    (lambda d: d.update(body_min_height_m=-0.045), '> 0'),
    (lambda d: d.update(schema='x'), 'schema'),
    (lambda d: d.update(contact_sensing='measured'), 'contact'),
    (lambda d: d.update(base_constraint='hanging'), 'base_constraint'),
    (lambda d: d.update(body_pose_required='yes'), 'true or false'),
    (lambda d: d.update(joint_limit_fraction=1.2), '< 1'),
])
def test_limits_parser_refuses_malformed(edit, fragment):
    d = _limits_data()
    edit(d)
    with pytest.raises(m61_limits.LimitsError, match=re.escape(fragment)):
        m61_limits.parse(d)


def test_limits_loader_refuses_duplicate_keys(tmp_path):
    src = open(os.path.join(SRC_CONFIG, m61_limits.FILE_NAME)).read()
    (tmp_path / m61_limits.FILE_NAME).write_text(src + 'lead_in_s: 3.0\n')
    with pytest.raises(m61_limits.LimitsError, match='duplicate'):
        m61_limits.load(str(tmp_path))


def test_an_edited_limit_is_refused(plan, traj):
    for k, v in (('max_joint_displacement_rad', 0.2), ('body_min_height_m', 0.03),
                 ('sim_stall_s', 9.0), ('max_trajectory_points', 20)):
        edited = dataclasses.replace(plan, limits=dataclasses.replace(plan.limits, **{k: v}))
        assert 'limits_not_approved' in codes(traj, edited), k
        with pytest.raises(gf.FingerprintError):
            goal61.approved_spec(traj, edited)
    edited = dataclasses.replace(plan, limits=dataclasses.replace(plan.limits, sim_stall_s=9.0))
    assert 'limits_inconsistent_with_m6d_monitor' in codes(traj, edited)


# ==================================================================== trajectory file and hash
def test_trajectory_file_loads_with_its_metadata(plan):
    cyc = tc.load_file(SRC_CONFIG)
    assert cyc['gait_name'] == 'trot' and cyc['num_cycles'] == 1
    assert cyc['cycle_duration_s'] == 4.0 and cyc['num_waypoints'] == 9 == len(cyc['waypoints'])
    assert cyc['step_length_m'] == 0.02 and cyc['lift_height_m'] == 0.006
    assert cyc['joint_names'] == list(plan.sources.joint_names)
    assert [w['time_from_start_s'] for w in cyc['waypoints']] == \
        [3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0]
    assert cyc['content_sha256'] == CONTENT_SHA256 == c61.APPROVED_CONTENT_SHA256
    # the hash quoted in the request identifies the design-note 2.5 cm / 10 mm table only
    assert cyc['design_note_reference_sha256'] == DESIGN_NOTE_SHA256 != CONTENT_SHA256


def test_content_hash_verifies(traj):
    params = tc.GaitParams.from_gait(traj['gait']).design_params()
    assert tc.content_sha256(params, traj['joint_names'], traj['points']) == CONTENT_SHA256
    assert traj['provenance']['content_sha256'] == CONTENT_SHA256


def test_any_content_edit_changes_the_hash_and_is_refused(traj, plan):
    t = mutated(traj, lambda t: t['points'][4]['positions'].__setitem__(4, -0.069097))
    got = codes(t, plan)
    assert {'content_hash_mismatch', 'content_not_approved', 'rederivation_mismatch'} <= got


def test_a_stored_hash_edit_is_refused(traj, plan):
    t = mutated(traj, lambda t: t['provenance'].__setitem__('content_sha256', '0' * 64))
    assert 'content_hash_mismatch' in codes(t, plan)


def test_waypoints_rederive_from_the_approved_parameters(traj, plan):
    regen = tc.generate_points(tc.GaitParams.from_gait(traj['gait']), plan.geoms,
                               list(plan.sources.joint_names))
    assert len(regen) == 9
    for r, p in zip(regen, traj['points']):
        assert r['time_from_start_s'] == p['time_from_start_s']
        for k in ('positions', 'velocities'):
            assert max(abs(a - b) for a, b in zip(r[k], p[k])) <= plan.limits.source_match_rad


def test_the_unapproved_design_note_variant_exceeds_the_cap(plan, traj):
    # 2.5 cm / 10 mm (design-note table): the reason it was not approved
    gait = dict(traj['gait'], step_length_m=0.025, lift_height_m=0.010)
    pts = tc.generate_points(tc.GaitParams.from_gait(gait), plan.geoms,
                             list(plan.sources.joint_names))
    per = tc.spline_extremes({'joint_names': traj['joint_names'], 'points': pts}, plan.neutral,
                             plan.sources.limits, plan.sources.soft_margin_rad, 50)
    worst = max(r['max_displacement_rad'] for r in per.values())
    assert worst > 0.1223 + 0.02
    assert tc.content_sha256(tc.GaitParams.from_gait(gait).design_params(),
                             traj['joint_names'], pts) != CONTENT_SHA256


def test_trajectory_file_parser_refuses_malformed():
    from spiderx_controller.posture_config import load_yaml_strict
    base = load_yaml_strict(os.path.join(SRC_CONFIG, tc.FILE_NAME))
    edits = [
        lambda d: d.update(extra=1),
        lambda d: d.pop('waypoints'),
        lambda d: d.update(schema='x'),
        lambda d: d.update(content_sha256='ABC'),
        lambda d: d.update(swing_pair_first=['front_left', 'front_right']),
        lambda d: d['waypoints'][0]['positions'].pop(),
        lambda d: d['waypoints'][0]['positions'].__setitem__(0, 0),        # int, not float
        lambda d: d['waypoints'][1].update(note='x'),
        lambda d: d.update(num_waypoints=True),
    ]
    for edit in edits:
        d = json.loads(json.dumps(base))
        edit(d)
        with pytest.raises(tc.TrotCycleError):
            tc.parse_file(d)


# ==================================================================== preflight and envelope
def test_preflight_passes_with_margins(traj, plan):
    r = tc.preflight(traj, plan)
    assert r.ok, r.failures
    assert all(v == 'passed' for v in r.checks.values()), r.checks
    s = r.summary
    assert s['point_count'] == 9 and s['duration_s'] == 7.0 and s['cycle_duration_s'] == 4.0
    assert s['content_sha256'] == CONTENT_SHA256
    assert 0.105 < s['max_waypoint_displacement_rad'] <= s['max_spline_displacement_rad'] < 0.1223
    assert 0.12 < s['max_spline_speed_rad_s'] < 0.25
    assert 0.1 < s['max_limit_fraction'] < 0.8
    assert s['min_soft_limit_margin_rad'] > 0.3
    assert s['rederivation_max_diff'] <= plan.limits.source_match_rad
    json.dumps(r.to_dict(), allow_nan=False)


def test_identity_is_pinned(traj, plan):
    assert traj['trajectory_id'] == TRAJECTORY_ID
    goal, report, spec, fp = goal61.build_live_goal(traj, plan)
    assert fp == FINGERPRINT == gf.fingerprint(goal61.approved_spec(traj, plan))
    assert spec['binding']['trajectory_id'] == TRAJECTORY_ID
    assert spec['binding']['mode'] == tc.MODE and spec['binding']['goal_count'] == 1


MUTATIONS = {
    'extra_point': (lambda t: t['points'].append(dict(t['points'][-1], time_from_start_s=7.5)),
                    {'point_count_mismatch', 'too_many_points'}),
    'segment_too_short': (lambda t: t['points'][3].update(time_from_start_s=4.95),
                          {'segment_too_short'}),
    'times_not_increasing': (lambda t: t['points'][3].update(time_from_start_s=3.9),
                             {'times_not_increasing'}),
    'first_time': (lambda t: [p.update(time_from_start_s=p['time_from_start_s'] - 1.0)
                              for p in t['points']], {'first_time_not_lead_in'}),
    'over_cap': (lambda t: t['points'][5]['positions'].__setitem__(4, -0.13),
                 {'displacement_exceeds_cap'}),
    'end_velocity': (lambda t: t['points'][-1]['velocities'].__setitem__(1, 0.01),
                     {'endpoint_velocity_nonzero'}),
    'end_not_neutral': (lambda t: t['points'][-1]['positions'].__setitem__(1, 0.01),
                        {'end_not_neutral'}),
    'start_not_neutral': (lambda t: t['points'][0]['positions'].__setitem__(2, 0.01),
                          {'start_not_neutral'}),
    'nan': (lambda t: t['points'][2]['positions'].__setitem__(1, float('nan')),
            {'non_finite_value'}),
    'joint_order': (lambda t: t['joint_names'].reverse(), {'joint_names_not_canonical'}),
    'gait_edit': (lambda t: t['gait'].update(lift_height_m=0.01), {'gait_not_approved'}),
    'gait_cycles': (lambda t: t['gait'].update(num_cycles=2), {'gait_not_approved'}),
    'trajectory_id': (lambda t: t.update(trajectory_id='0' * 16), {'trajectory_id_mismatch'}),
    'speed': (lambda t: t['points'][4]['velocities'].__setitem__(4, 0.6),
              {'joint_speed_exceeds_limit'}),
    'duration': (lambda t: [p.update(time_from_start_s=p['time_from_start_s'] * 2)
                            for p in t['points']], {'first_time_not_lead_in'}),
    'schema': (lambda t: t.update(mode='walk'), {'schema_invalid'}),
}


@pytest.mark.parametrize('name', sorted(MUTATIONS))
def test_every_mutation_is_refused_and_gets_no_goal(name, traj, plan):
    fn, expected = MUTATIONS[name]
    t = mutated(traj, fn)
    if name != 'trajectory_id':
        t['trajectory_id'] = m6t.compute_trajectory_id(t)
    got = codes(t, plan)
    assert expected <= got, (name, got)
    with pytest.raises(gf.FingerprintError):
        goal61.approved_spec(t, plan)
    with pytest.raises(gf.FingerprintError):
        goal61.build_goal_message(t, plan)


def test_unusable_sources_refuse_everything(plan, traj):
    bad = dataclasses.replace(plan, errors=(('sources_unavailable', 'x'),))
    with pytest.raises(tc.TrajectoryBuildError):
        tc.build_trajectory(bad)
    assert tc.preflight(traj, bad).codes == ['sources_unavailable']


def test_missing_limits_file_is_a_plan_error(tmp_path):
    import shutil
    for name in os.listdir(SRC_CONFIG):
        if name != m61_limits.FILE_NAME:
            shutil.copy(os.path.join(SRC_CONFIG, name), tmp_path / name)
    p = tc.load_plan(str(tmp_path), load_urdf())
    assert p.limits is None and any(m61_limits.FILE_NAME in m for _, m in p.errors)
    with pytest.raises(tc.TrajectoryBuildError):
        tc.build_trajectory(p)


# ==================================================================== spline reference
def test_reference_spline_matches_the_waypoints(traj):
    pts = traj['points']
    assert tc.reference_state(traj, 2.999) is None
    for p in pts[:-1]:
        q, qd = tc.reference_state(traj, p['time_from_start_s'])
        assert max(abs(a - b) for a, b in zip(q, p['positions'])) < 1e-12
        assert max(abs(a - b) for a, b in zip(qd, p['velocities'])) < 1e-9
    q, qd = tc.reference_state(traj, 99.0)
    assert q == pts[-1]['positions'] and qd == [0.0] * 12
    # continuity just before / after each interior waypoint
    for p in pts[1:-1]:
        a = tc.reference_positions(traj, p['time_from_start_s'] - 1e-7)
        b = tc.reference_positions(traj, p['time_from_start_s'] + 1e-7)
        assert max(abs(x - y) for x, y in zip(a, b)) < 1e-6


def test_tracking_against_the_spline(traj):
    samples = [(t / 50, dict(zip(traj['joint_names'], tc.reference_positions(traj, t / 50))))
               for t in range(150, 410)]
    verdict, d = tc.evaluate_tracking(traj, samples, 0.05)
    assert verdict == ac.PASSED and d['max_error_rad'] < 1e-12
    bad = [(t, {k: v + (0.06 if t > 5 else 0) for k, v in o.items()}) for t, o in samples]
    assert tc.evaluate_tracking(traj, bad, 0.05)[0] == ac.FAILED
    assert tc.evaluate_tracking(traj, [], 0.05)[0] == ac.UNAVAILABLE


# ==================================================================== goal message
def test_goal_message_contents(traj, plan):
    goal, report, spec, fp = goal61.build_live_goal(traj, plan)
    g = goal.trajectory
    assert (g.header.stamp.sec, g.header.stamp.nanosec) == (0, 0)
    assert list(g.joint_names) == list(plan.sources.joint_names)
    assert len(g.points) == 9
    assert [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in g.points] == \
        pytest.approx([3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0])
    assert all(len(p.velocities) == 12 and len(p.accelerations) == 0 for p in g.points)
    assert {t.position for t in goal.path_tolerance} == {0.05}
    assert {(t.position, t.velocity) for t in goal.goal_tolerance} == {(0.05, 0.05)}
    assert (goal.goal_time_tolerance.sec, goal.goal_time_tolerance.nanosec) == (1, 0)
    assert gf.verify_goal(goal, gf.binding(traj), fp) == fp


def test_any_other_goal_is_refused_by_the_fingerprint(traj, plan, monkeypatch):
    goal, _, _, fp = goal61.build_live_goal(traj, plan)
    from trajectory_msgs.msg import JointTrajectoryPoint
    import copy
    second = copy.deepcopy(goal)
    second.trajectory.points.append(JointTrajectoryPoint(
        positions=[0.0] * 12, velocities=[0.0] * 12))                  # e.g. a return point
    with pytest.raises(gf.FingerprintError):
        gf.verify_goal(second, gf.binding(traj), fp)
    # the M6.0-D crouch goal does not pass the M6.1 fingerprint (and vice versa)
    sources = plan.sources
    m6d_traj = m6t.build_trajectory(sources)
    m6d_goal, _, _, m6d_fp = gf.build_live_goal(m6d_traj, sources)
    assert m6d_fp != fp
    with pytest.raises(gf.FingerprintError):
        gf.verify_goal(m6d_goal, gf.binding(m6d_traj), fp)
    with pytest.raises(gf.FingerprintError):
        gf.verify_goal(goal, gf.binding(traj), m6d_fp)


# ==================================================================== gates (pure)
@pytest.fixture
def monitor(plan):
    return gates.GateMonitor(plan.limits, plan.sources.limits)


def pose(z=0.0545, roll=0.0, pitch=0.0, x=0.0, yaw=0.0):
    return (x, 0.0, z, roll, pitch, yaw)


def test_g1_body_height_needs_two_consecutive_low_samples(monitor):
    assert monitor.on_body_pose(1.0, 1.0, pose(z=0.040), True) is None
    assert monitor.on_body_pose(1.1, 1.1, pose(), True) is None              # streak reset
    assert monitor.on_body_pose(1.2, 1.2, pose(z=0.044), True) is None
    assert monitor.on_body_pose(1.3, 1.3, pose(z=0.044), True) == gates.BODY_TOO_LOW
    assert monitor.on_body_pose(1.4, 1.4, pose(z=0.044), True) is None       # recorded once
    (trip,) = monitor.trips
    assert trip['gate'] == gates.G1 and trip['value'] == 0.044 and trip['threshold'] == 0.045
    assert monitor.on_body_pose(1.5, 1.5, pose(z=0.0451), True) is None


def test_g2_tilt_roll_or_pitch(monitor):
    assert monitor.on_body_pose(1.0, 1.0, pose(pitch=-0.27), True) is None
    assert monitor.on_body_pose(1.1, 1.1, pose(pitch=-0.27), True) == gates.BODY_TILT
    m2 = gates.GateMonitor(monitor.lim, monitor.joint_limits)
    for _ in range(5):
        assert m2.on_body_pose(1.0, 1.0, pose(roll=0.25), True) is None      # below 0.26


def test_g3_joint_near_limit(monitor, plan):
    names = list(plan.sources.joint_names)
    lo, hi = plan.sources.limits['rf_foot_joint']
    q = [0.0] * 12
    i = names.index('rf_foot_joint')
    q[i] = 0.79 * hi
    assert monitor.on_joint_state(1.0, 1.0, names, q, True) is None
    q[i] = 0.81 * lo                                                         # negative side
    assert monitor.on_joint_state(1.1, 1.1, names, q, True) == gates.JOINT_NEAR_LIMIT
    (trip,) = monitor.trips
    assert trip['gate'] == gates.G3 and 'rf_foot_joint' in trip['detail']
    m2 = gates.GateMonitor(monitor.lim, monitor.joint_limits)
    q[i] = float('nan')
    assert m2.on_joint_state(1.0, 1.0, names, q, True) is None               # tracking's job


def test_gates_only_trip_while_gating(monitor, plan):
    names = list(plan.sources.joint_names)
    for k in range(4):
        assert monitor.on_body_pose(k, k, pose(z=0.0, roll=1.0), False) is None
    assert monitor.on_joint_state(5, 5, names, [0.7] * 12, False) is None
    assert monitor.trips == [] and monitor.z_min == 0.0                      # still observed


def test_pose_freshness(plan):
    m = gates.GateMonitor(plan.limits, plan.sources.limits)
    assert m.check_pose_stale(100.0) is None                                 # not armed yet
    m.arm(10.0)
    assert m.check_pose_stale(10.9) is None
    m.on_body_pose(10.5, 1.0, pose(), True)
    assert m.check_pose_stale(11.4) is None
    m.on_body_pose(11.45, 1.0, (0.0, 0.0, float('nan'), 0.0, 0.0, 0.0), True)   # invalid
    assert m.check_pose_stale(11.6) == gates.BODY_POSE_STALE
    not_required = dataclasses.replace(plan.limits, body_pose_required=False)
    m2 = gates.GateMonitor(not_required, plan.sources.limits)
    m2.arm(0.0)
    assert m2.check_pose_stale(100.0) is None


@pytest.mark.parametrize('latest, now, code', [
    (None, 10.0, 'body_pose_missing'),
    ((5.0, 1.0, pose()), 10.0, 'body_pose_stale'),
    ((9.9, 1.0, (0.0, 0.0, float('inf'), 0.0, 0.0, 0.0)), 10.0, 'body_pose_invalid'),
    ((9.9, 1.0, pose(z=0.03)), 10.0, 'body_too_low'),
    ((9.9, 1.0, pose(pitch=0.4)), 10.0, 'body_tilted'),
    ((9.9, 1.0, pose()), 10.0, None),
])
def test_pose_readiness(plan, latest, now, code):
    ok, got, detail = gates.pose_readiness(latest, now, plan.limits)
    assert ok is (code is None) and got == code
    json.dumps(detail, allow_nan=True)


def test_g7_drift_is_report_only(monitor):
    monitor.on_body_pose(1.0, 1.0, pose(), True)
    assert monitor.on_body_pose(2.0, 2.0, pose(x=0.05, yaw=0.2), True) is None
    s = monitor.summary()
    assert s[gates.G7]['report_only'] and len(s[gates.G7]['flags']) == 2
    assert s[gates.G4]['active'] is False and 'Not measured' in s[gates.G4]['note']
    json.dumps(s, allow_nan=False)


# ==================================================================== session (mock)
# M6.1-A: run_mock (the CLI --mock) now plays the WELDED plant through m61a_fixed_base, because
# M6.1 is only valid on the fixed base. Two physical consequences changed this table:
#  * a body that drops (body_too_low) or tilts (body_tilt) on a weld has left the weld pose, so
#    G8 (attachment) trips too - after G1/G2, whose reason still ends the run (one cancel);
#  * 'body_drift_report_only' (a 30 mm slide, G7 report only) cannot happen on an intact weld:
#    it is now 'attachment_drift' (G8 trips, one cancel). G7 report-only behaviour is still tested
#    on the free-base mock (test_g7_drift_report_only_on_the_free_base_mock).
EXPECTED = {   # scenario: (state, reason, goals, cancels, gates tripped)
    'success': (lpb.SUCCEEDED, None, 1, 0, None),
    'attachment_drift': (gr.GATE_TRIPPED, 'attachment_lost', 1, 1, (gates.G8,)),
    'not_fixed_base': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
    'spawn_offset': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
    'description_mismatch': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
    'mount_not_approved': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
    'interrupt': (lpb.CANCEL_CONFIRMED, 'operator_interrupt', 1, 1, None),
    'tracking_error': (lpb.TRACKING_FAILED, 'tracking_error', 1, 1, None),
    'stale_joint_states': (lpb.READINESS_LOST, 'joint_states_stale', 1, 1, None),
    'controller_lost': (lpb.HELD_ERROR, 'controller_lost', 1, 1, None),
    'rejected': (lpb.REJECTED, 'goal_rejected', 1, 0, None),
    'body_too_low': (gr.GATE_TRIPPED, 'body_too_low', 1, 1, (gates.G1, gates.G8)),
    'body_tilt': (gr.GATE_TRIPPED, 'body_tilt', 1, 1, (gates.G2, gates.G8)),
    'joint_near_limit': (gr.GATE_TRIPPED, 'joint_near_limit', 1, 1, (gates.G3,)),
    'sim_stall': (lpb.READINESS_LOST, 'sim_time_stalled', 1, 1, (gates.G5,)),
    'joint_state_gap': (lpb.TRACKING_FAILED, 'sample_gap', 1, 1, (gates.G6,)),
    'body_pose_stale': (gr.GATE_TRIPPED, 'body_pose_stale', 1, 1, (gates.POSE_FRESHNESS,)),
    'no_body_pose': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
    'competing_publisher': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
    'competing_client': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
    'competing_client_late': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
    'sim_paused': (lpb.REFUSED, 'readiness_not_ready', 0, 0, None),
}


def test_every_mock_scenario_is_covered():
    assert set(EXPECTED) == set(m61_mock.SCENARIOS)


@pytest.mark.parametrize('scenario', sorted(EXPECTED))
def test_mock_scenario_outcomes(scenario, plan):
    out, session, _ = gr.run_mock(plan, scenario, word())
    state, reason, goals, cancels, gate = EXPECTED[scenario]
    assert (out['state'], out['reason']) == (state, reason), out['events']
    assert (out['goals_sent'], out['cancels_sent']) == (goals, cancels)
    assert out['mock_server_goals_received'] == goals
    assert out['mock_server_cancels_received'] == cancels
    assert out['retries'] == 0 and out['automatic_return_goals'] == 0
    assert out['terminal'] and out['passed'] is (state == lpb.SUCCEEDED)
    assert out['errors'] == []
    if goals:
        assert out['trajectory_id'] == TRAJECTORY_ID and out['goal_fingerprint'] == FINGERPRINT
    if gate is not None:
        assert out['gates'][gate[0]]['tripped'], out['gates']
        if gate[0] not in (gates.G5, gates.G6):      # re-used M6.0-D monitor: no gate trip record
            assert out['gates']['trips_in_order'][0]['gate'] == gate[0]
    tripped = [g for g in gates.GATE_IDS if g != gates.G7 and out['gates']
               and out['gates'][g].get('tripped')]
    assert tripped == (list(gate) if gate else [])
    json.dumps(out, allow_nan=False)


FIXED_BASE_REFUSAL_CODES = {
    'not_fixed_base': 'robot_description_not_fixed_base',
    'spawn_offset': 'frame_spawn_not_identity',
    'description_mismatch': 'frame_body_link_inconsistent',
    'mount_not_approved': 'mount_not_approved',
    'sim_paused': 'body_pose_sim_time_not_advancing',
}


@pytest.mark.parametrize('scenario', sorted(FIXED_BASE_REFUSAL_CODES))
def test_fixed_base_readiness_refusals_send_nothing(scenario, plan):
    out, _, _ = gr.run_mock(plan, scenario, word())
    codes = out['readiness'][0]['result']['failure_codes']
    assert FIXED_BASE_REFUSAL_CODES[scenario] in codes, codes
    assert out['goals_sent'] == 0 and out['mock_server_goals_received'] == 0


def test_fixed_base_success_records_the_plant_apart_from_the_trajectory(plan):
    out, _, _ = gr.run_mock(plan, 'success', word())
    f = out['fixed_base']
    assert f['plant']['ok'] and f['plant']['codes'] == []
    assert f['mount_xyz_rpy'] == pytest.approx([0, 0, 0.125, 0, 0, 0])
    assert re.fullmatch(r'[0-9a-f]{64}', f['config_sha256'])
    assert re.fullmatch(r'[0-9a-f]{64}', f['robot_description_sha256'])
    assert out['trajectory_id'] == TRAJECTORY_ID and out['goal_fingerprint'] == FINGERPRINT
    g8 = out['gates'][gates.G8]
    assert g8['active'] and not g8['tripped'] and g8['worst_observed']['translation_m'] < 1e-9
    assert out['body']['frame'] == gates.FIXED_BASE_FRAME


def test_g8_integrity_codes_match_the_fixed_base_module():
    from spiderx_controller import m61a_fixed_base as fb
    assert set(gates.INTEGRITY_CODES) == set(fb.INTEGRITY_CODES)
    assert gates.REASON_GATE[gates.ATTACHMENT_LOST] == gates.G8


def test_g7_drift_report_only_on_the_free_base_mock(plan):
    s, t = _session(plan, **m61_mock.FREE_BASE_SCENARIOS['body_drift_report_only'])
    out = s.run()
    assert out['state'] == lpb.SUCCEEDED and out['cancels_sent'] == 0
    assert len(out['gates'][gates.G7]['flags']) == 1 and not out['gates'][gates.G8]['active']


def test_success_outcome_fields(plan):
    out, session, traj = gr.run_mock(plan, 'success', word())
    assert out['schema'] == gr.OUTCOME_SCHEMA and out['milestone'] == 'M6.1'
    assert out['channels'] == {'action': 'passed', 'tracking': 'passed',
                               'goal_tolerances': 'passed'}
    assert out['tracking']['max_inflight_error_rad'] < 0.02
    assert out['tracking']['detail']['uncovered_intervals'] == []
    assert out['base_constraint'] == 'fixed' and out['contact']['status'] == 'not_measured'
    # the outcome records the committed gate truthfully (False here; True on an enabling branch)
    assert out['m61_live_dispatch_enabled'] is m61_gate.EXPECTED_M61_LIVE_DISPATCH_ENABLED
    assert out['limits']['confirmation_word'] == 'SEND-ONE-TROT-CYCLE'
    assert 'approved_point_times_s' not in out['limits']                 # no M6.0-D content
    assert out['m61_limits'] == c61.APPROVED_LIMITS
    assert out['gait']['step_length_m'] == 0.02 and out['gait']['num_cycles'] == 1
    assert out['trajectory_content_sha256'] == CONTENT_SHA256
    # M6.1-A: the mock body is the welded body at the provisional mount (was 0.0545 m free base)
    assert out['preflight']['ok'] and out['body']['z_min_m'] == pytest.approx(0.125)
    assert out['gates'][gates.G1]['margin'] == pytest.approx(0.125 - 0.045)
    assert out['joint_extremes_observed']['rf_thigh_joint']['min_rad'] < -0.1
    assert [r['when'] for r in out['readiness']] == ['before_confirmation',
                                                     'after_confirmation']
    assert 'walking' in ' '.join(out['non_claims'])


@pytest.mark.parametrize('text', ['', 'yes\n', lc.CONFIRMATION_WORD + '\n',
                                  c61.CONFIRMATION_WORD.lower() + '\n'])
def test_wrong_confirmation_sends_nothing(plan, text):
    out, _, _ = gr.run_mock(plan, 'success', lambda: text)
    assert out['state'] == lpb.REFUSED and out['reason'] == 'confirmation_refused'
    assert out['goals_sent'] == 0 and out['mock_server_goals_received'] == 0


def _session(plan, **kw):
    traj = tc.build_trajectory(plan)
    fp = gf.fingerprint(goal61.approved_spec(traj, plan))
    latch = lpb.InterruptLatch()
    t = m61_mock.M61FakeTransport(traj, fp, latch=latch, **kw)
    report = m6mock.mock_readiness_report(plan.sources.joint_names, plan.neutral)
    from spiderx_controller import m6_live_readiness as rd
    provide = gr.with_body_pose(lambda: rd.assess(report, t.wall_now()), t.latest_body_pose,
                                t.wall_now, plan.limits)
    return gr.M61Session(t, plan, provide, word(), latch=latch), t


def test_a_session_sends_at_most_one_goal(plan):
    s, t = _session(plan)
    assert s.run()['state'] == lpb.SUCCEEDED
    with pytest.raises(ac.SecondGoalForbidden):
        s.run()
    assert len(t.sent) == 1


def test_gate_trip_after_the_result_fails_without_a_cancel(plan):
    s, t = _session(plan, body_low_at=7.3)            # result at 7.15 s, settle 1.0 s
    out = s.run()
    assert out['state'] == gr.GATE_TRIPPED and out['reason'] == 'body_too_low'
    assert out['cancels_sent'] == 0 and t.cancels == 0 and len(t.sent) == 1
    assert out['channels']['tracking'] == ac.PASSED


def test_a_gate_trip_never_sends_a_second_goal_or_return(plan):
    s, t = _session(plan, tilt_at=4.0, body_low_at=4.06)
    out = s.run()
    assert out['state'] == gr.GATE_TRIPPED and out['reason'] == 'body_tilt'
    assert len(t.sent) == 1 and t.cancels == 1
    assert [c[0] for c in t.calls].count('send_goal') == 1
    # the later G1 trip is recorded too (robot holds after the cancel), but no second cancel
    assert [x['gate'] for x in out['gates']['trips_in_order']] == [gates.G2, gates.G1]


def test_low_body_at_readiness_sends_nothing(plan):
    s, t = _session(plan, body_z=0.03)
    out = s.run()
    assert out['state'] == lpb.REFUSED and t.sent == []
    assert 'body_too_low' in out['readiness'][0]['result']['failure_codes']


def test_preflight_failure_sends_nothing(plan):
    bad = dataclasses.replace(plan, limits=dataclasses.replace(plan.limits, lead_in_s=2.0))
    traj = tc.build_trajectory(plan)
    t = m61_mock.M61FakeTransport(traj, FINGERPRINT)
    s = gr.M61Session(t, bad, lambda: None, word())
    out = s.run()
    assert out['state'] == lpb.REFUSED and out['reason'] == 'preflight_refused'
    assert t.sent == [] and t.calls == []


# ==================================================================== evidence (mock CLI)
def cli(plan, tmp_path, *extra, reader=None):
    return gr.main(['m6_gait_replay', *extra, '--out', str(tmp_path)], plan=plan,
                   reader=reader or word())


def test_mock_cli_writes_the_evidence_directory(plan, tmp_path):
    assert cli(plan, tmp_path, '--mock', '--scenario', 'success') == gr.EXIT_OK
    (run,) = os.listdir(tmp_path / 'mock')
    assert re.fullmatch(r'\d{8}T\d{6}Z_success', run)
    d = tmp_path / 'mock' / run
    assert sorted(os.listdir(d)) == sorted(evd.SUPPORT_FILES + (evd.MOCK_OUTCOME,))
    out = json.loads((d / evd.MOCK_OUTCOME).read_text())
    assert out['state'] == lpb.SUCCEEDED and out['run_dir'] == str(d)
    assert set(out['evidence_files'].values()) == {'written'}
    js = (d / 'joint_states.csv').read_text().splitlines()
    assert js[0].split(',') == ['time_s'] + list(plan.sources.joint_names)
    assert len(js) - 1 == out['evidence_rows']['joint_states'] > 300
    assert all(len(row.split(',')) == 13 for row in js[1:])
    bp = (d / 'body_pose.csv').read_text().splitlines()
    assert bp[0] == 'time_s,x,y,z,roll,pitch,yaw'
    assert len(bp) - 1 == out['evidence_rows']['body_pose'] > 100
    assert float(bp[1].split(',')[3]) == pytest.approx(0.125)        # M6.1-A welded body
    cov = (d / 'commanded_vs_observed.csv').read_text().splitlines()
    assert cov[0].startswith('time_s,ref_time_s,lf_hip_cmd,lf_hip_obs,lf_hip_err')
    assert cov[0].endswith('max_abs_err_rad') and len(cov) > 200
    assert max(float(r.split(',')[-1]) for r in cov[1:]) < 0.05
    traj = json.loads((d / 'trajectory.json').read_text())
    assert traj['trajectory_id'] == TRAJECTORY_ID and len(traj['points']) == 9
    assert FINGERPRINT in (d / 'goal_fingerprint.txt').read_text()
    assert CONTENT_SHA256 in (d / 'goal_fingerprint.txt').read_text()
    assert json.loads((d / 'readiness_before.json').read_text())['when'] == 'before_confirmation'
    assert json.loads((d / 'readiness_after.json').read_text())['when'] == 'after_confirmation'
    g = json.loads((d / 'gates.json').read_text())
    assert set(gates.GATE_IDS) <= set(g['gates']) and g['contact']['status'] == 'not_measured'
    assert (f'm61_live_dispatch_enabled: {m61_gate.EXPECTED_M61_LIVE_DISPATCH_ENABLED}'
            in (d / 'git_state.txt').read_text())
    assert 'ROS_DISTRO' in (d / 'environment.txt').read_text()


def test_mock_cli_gate_trip_is_recorded(plan, tmp_path):
    assert cli(plan, tmp_path, '--mock', '--scenario', 'joint_near_limit') == gr.EXIT_FAILED
    (run,) = os.listdir(tmp_path / 'mock')
    g = json.loads((tmp_path / 'mock' / run / 'gates.json').read_text())
    assert g['gates'][gates.G3]['tripped'] and g['cancel_reason'] == 'joint_near_limit'


def test_mock_cli_refusal_still_records_evidence(plan, tmp_path):
    assert cli(plan, tmp_path, '--mock', '--scenario', 'no_body_pose') == gr.EXIT_REFUSED
    (run,) = os.listdir(tmp_path / 'mock')
    d = tmp_path / 'mock' / run
    assert (d / 'body_pose.csv').read_text() == 'time_s,x,y,z,roll,pitch,yaw\n'
    assert json.loads((d / evd.MOCK_OUTCOME).read_text())['goals_sent'] == 0


@pytest.mark.parametrize('gate', [False, True])
def test_evidence_records_the_gate_as_it_is_in_both_modes(gate, plan, tmp_path, monkeypatch):
    """Offline only (dry run, gate state helper): the recorded gate follows the gate itself."""
    monkeypatch.setattr(c61, 'M61_LIVE_DISPATCH_ENABLED', gate)        # tests only
    assert gr._gate_state()['m61_live_dispatch_enabled'] is gate
    assert c61.as_dict()['m61_live_dispatch_enabled'] is gate
    assert cli(plan, tmp_path, '--dry-run') == gr.EXIT_OK
    data = json.loads((tmp_path / 'dry_run' / TRAJECTORY_ID / 'dry_run_report.json').read_text())
    assert data['m61_live_dispatch_enabled'] is gate and data['goals_sent'] == 0


def test_no_write_writes_nothing(plan, tmp_path):
    assert cli(plan, tmp_path, '--mock', '--no-write') == gr.EXIT_OK
    assert cli(plan, tmp_path, '--dry-run', '--no-write') == gr.EXIT_OK
    assert os.listdir(tmp_path) == []


def test_dry_run_report_is_never_overwritten(plan, tmp_path):
    assert cli(plan, tmp_path, '--dry-run') == gr.EXIT_OK
    p = tmp_path / 'dry_run' / TRAJECTORY_ID / 'dry_run_report.json'
    data = json.loads(p.read_text())
    assert data['goal_fingerprint'] == FINGERPRINT and data['goals_sent'] == 0
    assert data['m61_live_dispatch_enabled'] is m61_gate.EXPECTED_M61_LIVE_DISPATCH_ENABLED
    before = p.read_bytes()
    assert cli(plan, tmp_path, '--dry-run') == gr.EXIT_REFUSED
    assert p.read_bytes() == before


def test_unknown_scenario_and_stray_domain_id_are_refused(plan, tmp_path):
    assert cli(plan, tmp_path, '--mock', '--scenario', 'walk') == gr.EXIT_REFUSED
    assert cli(plan, tmp_path, '--mock', '--domain-id', '3') == gr.EXIT_REFUSED
    assert os.listdir(tmp_path) == []


def test_evidence_primitives_never_overwrite(tmp_path):
    d = evd.reserve_run_dir(str(tmp_path), '20261003T000000Z')
    with pytest.raises(FileExistsError):
        evd.reserve_run_dir(str(tmp_path), '20261003T000000Z')
    p = os.path.join(d, 'x.txt')
    evd.write_exclusive(p, 'a')
    with pytest.raises(FileExistsError):
        evd.write_exclusive(p, 'b')
    assert open(p).read() == 'a'
    assert evd.utc_stamp().endswith('Z') and len(evd.utc_stamp()) == 16


def test_csv_formats():
    assert evd.joint_states_csv(['a', 'b'], [(1.5, {'a': 0.1, 'b': -0.2}), (None, {'a': 1})]) \
        == 'time_s,a,b\n1.5,0.1,-0.2\n,1.0,\n'
    assert evd.body_pose_csv([(2.0, (1, 2, 3, 0.1, 0.2, 0.3))]) == \
        'time_s,x,y,z,roll,pitch,yaw\n2.0,1.0,2.0,3.0,0.1,0.2,0.3\n'
    assert evd.body_pose_csv([]) == 'time_s,x,y,z,roll,pitch,yaw\n'


# ==================================================================== gated live wiring (test-only)
class Graph:
    """Read-only observations of a healthy stack (no ROS)."""

    def __init__(self, names, neutral):
        self.action_servers = [('/leg_trajectory_controller', [lpf.env.ACTION_TYPE])]
        self.controllers = [('joint_state_broadcaster', 'x', 'active'),
                            ('leg_trajectory_controller', lpf.JTC_TYPE, 'active')]
        self.joint_state_publishers = [('/joint_state_broadcaster', lpf.JOINT_STATE_TYPE)]
        self.joint_state_messages = [(1.0 + 0.01 * i, list(names), list(neutral))
                                     for i in range(4)]
        self.probe_errors = []


class Factories:
    def __init__(self, plan, **kw):
        self.plan, self.kw, self.made = plan, kw, []

    def transport(self, fp, domain_id, fixed_base):     # M6.1-A: the live path is fixed base
        t = m61_mock.M61FakeTransport(tc.build_trajectory(self.plan), fp, fixed_base=fixed_base,
                                      **self.kw)
        self.made.append(t)
        return t

    def collect(self, transport):
        return lambda: Graph(self.plan.sources.joint_names, self.plan.neutral)


@pytest.fixture
def enabled(monkeypatch):
    monkeypatch.setattr(c61, 'M61_LIVE_DISPATCH_ENABLED', True)       # tests only


def run_enabled(plan, tmp_path, f, utc, reader=None, *extra):
    args = gr.parse_args(['--live', '--domain-id', '42', '--out', str(tmp_path), *extra])
    return gr._live_main(args, plan, reader or word(), transport_factory=f.transport,
                         collect_factory=f.collect, utc_stamp=utc)


def test_enabled_live_wiring_records_log_m61_run_utc(enabled, plan, tmp_path):
    f = Factories(plan)
    rc = run_enabled(plan, tmp_path, f, '20261003T120000Z')
    assert rc == gr.EXIT_OK
    d = tmp_path / '20261003T120000Z'
    assert sorted(os.listdir(d)) == sorted(evd.SUPPORT_FILES + (evd.LIVE_OUTCOME,))
    out = json.loads((d / evd.LIVE_OUTCOME).read_text())
    assert out['evidence']['phase'] == 'final' and out['mode'] == 'live'
    assert out['state'] == lpb.SUCCEEDED and out['domain_id'] == 42
    assert out['goals_sent'] == 1 and len(f.made) == 1 and len(f.made[0].sent) == 1
    assert out['transport']['closed'] and set(out['evidence_files'].values()) == {'written'}
    assert out['goal_fingerprint'] == FINGERPRINT
    # a second run with the same stamp is refused: evidence is never overwritten
    before = (d / evd.LIVE_OUTCOME).read_bytes()
    assert run_enabled(plan, tmp_path, Factories(plan), '20261003T120000Z') == gr.EXIT_REFUSED
    assert (d / evd.LIVE_OUTCOME).read_bytes() == before


def test_enabled_live_gate_trip_and_refusals(enabled, plan, tmp_path):
    f = Factories(plan, body_low_at=5.0)
    assert run_enabled(plan, tmp_path, f, '20261003T120001Z') == gr.EXIT_FAILED
    out = json.loads((tmp_path / '20261003T120001Z' / evd.LIVE_OUTCOME).read_text())
    assert out['state'] == gr.GATE_TRIPPED and f.made[0].cancels == 1
    assert len(f.made[0].sent) == 1 and out['automatic_return_goals'] == 0
    f = Factories(plan, no_pose=True)
    assert run_enabled(plan, tmp_path, f, '20261003T120002Z') == gr.EXIT_REFUSED
    assert f.made[0].sent == []
    f = Factories(plan)
    assert run_enabled(plan, tmp_path, f, '20261003T120003Z',
                       lambda: lc.CONFIRMATION_WORD + '\n') == gr.EXIT_REFUSED
    assert f.made[0].sent == []


def test_enabled_live_refuses_a_paused_world_before_any_goal(enabled, plan, tmp_path):
    """Fakes only: /clock and pose/info keep arriving from a paused world, so the body pose is
    fresh by receipt; readiness still refuses because simulation time does not advance."""
    f = Factories(plan, **m61_mock.SCENARIOS['sim_paused'])
    assert run_enabled(plan, tmp_path, f, '20261003T120004Z') == gr.EXIT_REFUSED
    out = json.loads((tmp_path / '20261003T120004Z' / evd.LIVE_OUTCOME).read_text())
    assert f.made[0].sent == [] and out['goals_sent'] == 0
    res = out['readiness'][0]['result']
    assert res['failure_codes'] == [fb.POSE_SIM_NOT_ADVANCING], res


def test_enabled_live_still_needs_domain_and_evidence(enabled, plan, tmp_path):
    f = Factories(plan)
    args = gr.parse_args(['--live', '--out', str(tmp_path)])
    assert gr._live_main(args, plan, word(), transport_factory=f.transport,
                         collect_factory=f.collect) == gr.EXIT_REFUSED
    args = gr.parse_args(['--live', '--domain-id', '1', '--no-write', '--out', str(tmp_path)])
    assert gr._live_main(args, plan, word(), transport_factory=f.transport,
                         collect_factory=f.collect) == gr.EXIT_REFUSED
    assert f.made == [] and os.listdir(tmp_path) == []


# ==================================================================== isolated-domain: pose sub
DOMAIN = 150 + (os.getpid() + 25) % 50


@pytest.fixture
def isolated_env(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')


def test_rclpy_transport_reads_the_body_pose_read_only(isolated_env):
    """Isolated domain only: an in-test publisher stands in for the Gazebo pose bridge."""
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from tf2_msgs.msg import TFMessage
    from spiderx_controller import m61_live_adapter as la
    assert 150 <= DOMAIN < 200 and DOMAIN != int(os.environ.get('ROS_DOMAIN_ID', '0') or 0)
    t = la.M61RclpyLiveTransport('x' * 64, DOMAIN, node_name='m61_pose_test_client',
                                 use_sim_time=False).open()
    ctx = node = None
    try:
        t.poll(1.5)
        nodes = t.node.get_node_names_and_namespaces()
        if nodes != [('m61_pose_test_client', '/')]:
            pytest.skip(f'isolation not established on ROS domain {DOMAIN}: {nodes}')
        assert t.node.count_publishers('/spiderx/sim/world_poses') == 0
        ctx = Context()
        rclpy.init(context=ctx, domain_id=DOMAIN, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('m61_fake_pose_bridge', context=ctx)
        pub = node.create_publisher(TFMessage, '/spiderx/sim/world_poses', 10)
        model, other = TransformStamped(), TransformStamped()
        model.child_frame_id, other.child_frame_id = 'spiderx', 'ground_plane'
        model.transform.translation.x, model.transform.translation.z = 0.1, 0.0545
        model.transform.rotation.x = math.sin(0.05)            # roll 0.1 rad
        model.transform.rotation.w = math.cos(0.05)
        got = []
        end = time.monotonic() + 15.0
        while time.monotonic() < end and not got:
            pub.publish(TFMessage(transforms=[other]))          # no model: ignored
            pub.publish(TFMessage(transforms=[other, model]))
            got = [e for e in t.poll(0.1) if e[0] == 'body_pose']
        assert got, 'no body_pose event received'
        _, wall, stamp, p = got[0]
        assert p[0] == pytest.approx(0.1) and p[2] == pytest.approx(0.0545)
        assert p[3] == pytest.approx(0.1) and abs(p[4]) < 1e-9
        assert t.latest_body_pose()[2] == p
        assert t.pose_messages_without_model >= 1
        assert not t._goal_sent and not t._cancel_sent             # read-only
    finally:
        if node is not None:
            node.destroy_node()
        if ctx is not None:
            rclpy.try_shutdown(context=ctx)
        t.close()
    assert t.node is None and t.pose_sub is None


# ============================================================ command owner (M6.1-A review)
@pytest.mark.parametrize('scenario, code', [
    ('competing_publisher', fb.COMMAND_PUBLISHERS),
    ('competing_client', fb.COMMAND_ACTION_CLIENTS),
    ('competing_client_late', fb.COMMAND_ACTION_CLIENTS),
])
def test_a_visible_competing_commander_refuses_before_any_goal(plan, scenario, code):
    out, session, _ = gr.run_mock(plan, scenario, word())
    assert out['goals_sent'] == 0 and out['mock_server_goals_received'] == 0
    assert code in json.dumps(out)


def test_a_late_competing_client_is_caught_by_the_pre_dispatch_recheck(plan):
    """Readiness #1 sees no other commander; the client appears before the re-observation that
    immediately precedes dispatch, which refuses. A client appearing after that re-check is not
    seen by readiness at all (point-in-time graph observation); the controller would then
    preempt our goal, which ends not SUCCEEDED."""
    out, session, _ = gr.run_mock(plan, 'competing_client_late', word())
    assert session.transport.owner_snapshots == 2
    assert out['state'] == lpb.REFUSED and out['goals_sent'] == 0


def test_sim_progress_wrapper_needs_advancing_sim_time():
    cfg = gr.load_fixed_base_config(SRC_CONFIG)
    base = lambda: rd.ReadinessResult(True, 'compatible', (), (), 0.0)      # noqa: E731
    running = {'recent_usable_samples': tuple((9.0 + 0.02 * i, 100.0 + 0.014 * i)
                                              for i in range(51))}           # RTF 0.7
    paused = {'recent_usable_samples': tuple((9.0 + 0.02 * i, 100.0) for i in range(51))}
    res = gr.with_sim_progress(base, lambda: running, lambda: 10.0, cfg)()
    assert res.ready and res.report['m61a_sim_progress']['ok']
    assert res.report['m61a_sim_progress']['sim_advance_s'] == pytest.approx(0.7)
    for snap in (paused, {}, {'recent_usable_samples': ()}):
        res = gr.with_sim_progress(base, lambda: snap, lambda: 10.0, cfg)()
        assert not res.ready and res.failure_codes == (fb.POSE_SIM_NOT_ADVANCING,)
    # the same samples one window later are not evidence of progress NOW
    res = gr.with_sim_progress(base, lambda: running, lambda: 11.5, cfg)()
    assert not res.ready
    # a result that is already NOT READY keeps its codes and gains this one
    bad = lambda: rd.ReadinessResult(False, 'compatible', ('x',), ('joint_states_stale',), 0.0)  # noqa: E731,E501
    res = gr.with_sim_progress(bad, lambda: paused, lambda: 10.0, cfg)()
    assert res.failure_codes == ('joint_states_stale', fb.POSE_SIM_NOT_ADVANCING)


def test_command_owner_wrapper_unknown_counts_are_not_ready():
    base = lambda: rd.ReadinessResult(True, 'compatible', (), (), 0.0)      # noqa: E731
    res = gr.with_command_owner(base, lambda: {'command_publishers': None,
                                               'foreign_action_clients': 0})()
    assert not res.ready and fb.COMMAND_OWNER_UNKNOWN in res.failure_codes
    res = gr.with_command_owner(base, lambda: {'command_publishers': 0,
                                               'foreign_action_clients': 0})()
    assert res.ready and res.report['m61a_command_owner']['ok']
