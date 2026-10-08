"""M5.5 locomotion: gate, configuration, phase-goal library, shadow mode, the state machine (with
the in-memory dispatching fake) and the keyboard logic.

Pure Python (no ROS graph). The dispatching fake is used only with the M5.5 gate set True INSIDE
the test (monkeypatch); the committed literal stays False (test/m55_gate.py).
"""

import copy
import math
import os
import re

import m55_fixtures as fx
import m55_gate
import m61_gate
import m6d_gate
import pytest
import yaml

from spiderx_controller import m55_command as cmd
from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_crawl as cr
from spiderx_controller import m55_fake as fk
from spiderx_controller import m55_locomotion as loc
from spiderx_controller import m55_teleop as tp
from spiderx_controller import m61_live_contract as c61
from spiderx_controller import m6_live_contract as lc

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


@pytest.fixture(scope='module')
def cfg():
    return fx.config()


@pytest.fixture(scope='module')
def lib():
    return fx.synthetic_library()


@pytest.fixture
def gate_on(monkeypatch):
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', True)      # tests only


def dispatch_world(cfg, lib, scenario='ok', rtf=1.0, graph_period_s=None, **kw):
    """The dispatching fake follows SIM time, like the real controller."""
    kw.setdefault('use_sim', True)
    plant = fk.FakeLocomotionTransport(lib, lib.boundary_positions(0, 0), scenario=scenario, **kw)
    s = loc.LocomotionSession(cfg, lib, plant)
    return fk.FakeWorld(s, plant, rtf=rtf, graph_period_s=graph_period_s), s, plant


def shadow_world(cfg, lib):
    tr = loc.ShadowTransport(lib.fingerprints)
    s = loc.LocomotionSession(cfg, lib, tr)
    return fk.FakeWorld(s, fk.StaticPlant(lib.boundary_positions(0, 0))), s, tr


def arm(world):
    world.run(1.0)
    ok, msg = world.request('arm')
    assert ok, msg
    return msg


def fwd(lib, level=0, sign=1):
    return (sign * lib.speeds_m_s[level], 0.0, 0.0)


def assert_chained(goals):
    for a, b in zip(goals, goals[1:]):
        assert b.start == a.end, (a.summary(), b.summary())


# ==================================================================== gate and contract
def _package_sources():
    d = os.path.join(PKG, 'spiderx_controller')
    for name in sorted(os.listdir(d)):
        if name.endswith('.py'):
            with open(os.path.join(d, name)) as f:
                yield name, f.read()


def test_m55_gate_is_false_pinned_and_separate():
    assert c55.M55_LOCOMOTION_DISPATCH_ENABLED is \
        m55_gate.EXPECTED_M55_LOCOMOTION_DISPATCH_ENABLED is False
    assert lc.LIVE_DISPATCH_ENABLED is m6d_gate.EXPECTED_LIVE_DISPATCH_ENABLED is False
    assert c61.M61_LIVE_DISPATCH_ENABLED is m61_gate.EXPECTED_M61_LIVE_DISPATCH_ENABLED is False


def test_m55_gate_is_a_single_false_literal():
    hits = [(n, line) for n, src in _package_sources() for line in src.splitlines()
            if re.match(r'\s*(\w+\.)?M55_LOCOMOTION_DISPATCH_ENABLED\s*=', line)]
    assert hits == [('m55_contract.py', 'M55_LOCOMOTION_DISPATCH_ENABLED = False')]


M55_MODULES = ('m55_contract.py', 'm55_command.py', 'm55_crawl.py', 'm55_locomotion.py',
               'm55_fake.py', 'm55_rclpy.py', 'm55_teleop.py', 'm55_feasibility.py')


def test_no_environment_flag_or_parameter_can_enable_m55_dispatch():
    srcs = dict(_package_sources())
    for name in M55_MODULES:
        for w in ('os.environ', 'getenv', 'DISPATCH_ENABLED or', "add_argument('--enable",
                  "add_argument('--yes'", "add_argument('--force'", "add_argument('--live'",
                  'DISPATCH_ENABLED = True', 'declare_parameter'):
            assert w not in srcs[name], (name, w)
    # the gate is read, never assigned, outside the contract
    for name in M55_MODULES[1:]:
        for line in srcs[name].splitlines():
            assert not re.match(r'\s*(\w+\.)?M55_LOCOMOTION_DISPATCH_ENABLED\s*=[^=]', line), name
        assert 'setattr(c55' not in srcs[name], name


def test_only_the_ros_modules_import_rclpy_and_only_inside_functions():
    srcs = dict(_package_sources())
    for name in ('m55_contract.py', 'm55_command.py', 'm55_crawl.py', 'm55_locomotion.py',
                 'm55_fake.py', 'm55_feasibility.py'):
        assert 'import rclpy' not in srcs[name] and 'from rclpy' not in srcs[name], name
    for name in ('m55_rclpy.py', 'm55_teleop.py'):
        top = [ln for ln in srcs[name].splitlines() if ln.startswith(('import ', 'from '))]
        assert not [ln for ln in top if 'rclpy' in ln or '_msgs' in ln or 'std_srvs' in ln], top


def test_only_the_gated_transport_creates_an_action_client():
    srcs = dict(_package_sources())
    users = [n for n in M55_MODULES if 'ActionClient(' in srcs[n]]
    assert users == ['m55_rclpy.py']
    src = srcs['m55_rclpy.py']
    cls = src[src.index('class RclpyPhaseTransport'):src.index('# ====', src.index(
        'class RclpyPhaseTransport'))]
    assert cls.index('if not c55.M55_LOCOMOTION_DISPATCH_ENABLED') < cls.index('ActionClient(')
    assert 'create_publisher(' not in cls
    # the node publishes only its status; nothing towards the controller
    assert src.count('create_publisher(') == 1 and 'STATUS_TOPIC' in src


def test_session_refuses_a_dispatching_transport_while_the_gate_is_false(cfg, lib):
    plant = fk.FakeLocomotionTransport(lib, lib.boundary_positions(0, 0))
    with pytest.raises(loc.DispatchDisabled):
        loc.LocomotionSession(cfg, lib, plant)
    assert plant.sent == 0


def test_dispatch_rechecks_the_gate_before_every_send(cfg, lib, monkeypatch):
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', True)
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', False)
    w.cmd = fwd(lib)
    w.run(1.0)
    assert s.state == loc.FAULTED and s.faults == [loc.DISPATCH_DISABLED] and plant.sent == 0


def test_contract_names_and_non_claims():
    assert c55.MILESTONE == 'M5.5'
    assert c55.SERVICES == ('arm', 'disarm', 'stop', 'estop', 'reset', 'home')
    assert c55.STATUS_TOPIC.startswith(c55.SERVICE_PREFIX)
    text = ' '.join(c55.NON_CLAIMS)
    for w in ('nothing here has moved a robot', 'not a proof of balance', 'not odometry',
              'not an actuator rating'):
        assert w in text


# ==================================================================== configuration
def test_config_loads_provisional_with_hash(cfg):
    assert cfg.status == 'provisional' and len(cfg.sha256) == 64
    assert cfg.command.topic == '/cmd_vel' and cfg.dispatch.action.endswith(
        'follow_joint_trajectory')
    assert cfg.gait.stride_levels_m == (0.02, 0.04, 0.06)
    assert cfg.dispatch.tracking_tolerance_rad == 0.05            # = M6.0-D / M6.1
    assert cfg.monitor.max_tilt_rad == 0.26                       # = M6.1 G2 tilt limit
    assert cfg.teleop.hold_s > 0.66                               # > X11 default repeat delay


def _raw():
    with open(os.path.join(fx.CONFIG_DIR, loc.CONFIG_FILE)) as f:
        return yaml.safe_load(f)


def _edit(path, value):
    def fn(d):
        *head, last = path.split('.')
        for k in head:
            d = d[k]
        if value is KeyError:
            del d[last]
        else:
            d[last] = value
    return fn


@pytest.mark.parametrize('edit, fragment', [
    (_edit('schema', 'x'), 'schema'),
    (_edit('status', 'accepted'), 'provisional'),
    (_edit('extra', 1), 'config keys'),
    (_edit('command.lease_s', KeyError), 'command keys'),
    (_edit('command.bogus', 1), 'command keys'),
    (_edit('command.lease_s', -1), 'command.lease_s'),
    (_edit('command.lease_s', float('nan')), 'command.lease_s'),
    (_edit('command.lease_s', True), 'command.lease_s'),
    (_edit('command.topic', ''), 'command.topic'),
    (_edit('gait.stride_levels_m', []), 'stride_levels_m'),
    (_edit('gait.stride_levels_m', [0.04, 0.02]), 'strictly increasing'),
    (_edit('gait.speed_use', 1.0), 'speed_use'),
    (_edit('gait.min_margin_m', 0.03), 'min_margin_m'),
    (_edit('gait.check_dt_s', 0.03), 'divide'),
    (_edit('monitor.body_pose_required', 'yes'), 'true or false'),
    (_edit('dispatch.phase_lead_s', 0.05), 'lead-in'),
    (_edit('dispatch.tracking_tolerance_rad', 0.01), 'tracking_tolerance_rad'),
    (_edit('command.lease_s', 0.15), 'heartbeats'),
    (_edit('command.max_rate_hz', 10.0), 'max_rate_hz'),
    (_edit('teleop.hold_s', 0.05), 'hold_s'),
    (_edit('monitor.graph_stale_s', 0.5), 'graph_stale_s'),
])
def test_config_parser_refuses(edit, fragment):
    d = _raw()
    edit(d)
    with pytest.raises(loc.LocomotionError, match=re.escape(fragment)):
        loc.parse_config(d)


def test_config_round_trips_to_dict(cfg):
    d = cfg.to_dict()
    assert d['gait']['stride_levels_m'] == [0.02, 0.04, 0.06] and d['sha256'] == cfg.sha256


# ==================================================================== the phase library
def test_library_has_every_phase_goal_in_both_directions(lib):
    P = lib.n_phases
    assert P == 9 and len(lib.goals) == 3 * P * 2 == len(lib.fingerprints)
    for (level, k, d), g in lib.goals.items():
        assert g.points[0][0] == pytest.approx(lib.lead_s)
        assert all(v == 0.0 for v in g.points[0][2]) and all(v == 0.0 for v in g.points[-1][2])
        assert all(b[0] > a[0] for a, b in zip(g.points, g.points[1:]))
        assert (g.b_from, g.b_to) == ((k, k + 1) if d > 0 else (k + 1, k))
        assert loc.goal_fingerprint(g.joint_names, g.points) == g.fingerprint


def test_reverse_phase_is_the_forward_phase_in_reverse_time(lib):
    for level in range(3):
        for k in range(lib.n_phases):
            f, r = lib.goals[(level, k, 1)], lib.goals[(level, k, -1)]
            T = f.duration_s + lib.lead_s
            assert [p[1] for p in r.points] == [p[1] for p in reversed(f.points)]
            assert [p[0] for p in r.points] == pytest.approx([T - p[0] for p in
                                                             reversed(f.points)])
            for pr, pf in zip(r.points, reversed(f.points)):
                assert pr[2] == pytest.approx([-v for v in pf[2]])
            assert r.body_delta == pytest.approx(tuple(-x for x in f.body_delta))


def test_goal_selection_wraps_at_the_cycle_boundary(lib):
    P = lib.n_phases
    assert lib.goal(0, 0, 1).phase == 0 and lib.goal(0, P - 1, 1).b_to == P
    assert lib.goal(0, 0, -1).phase == P - 1 and lib.goal(0, 0, -1).b_from == P
    assert lib.goal(1, 4, -1).phase == 3


def test_neutral_is_shared_by_every_level_and_mid_boundaries_differ(lib):
    n = [lib.boundary_positions(lv, 0) for lv in range(3)]
    assert n[0] == n[1] == n[2]
    assert lib.boundary_positions(0, 4) != lib.boundary_positions(2, 4)


def test_posture_candidates(lib):
    assert lib.candidates(lib.boundary_positions(0, 0), 0.02) == [(None, 0, 0.0)]
    q = dict(lib.boundary_positions(1, 5))
    q['lf_hip'] += 0.015
    (level, b, err), = lib.candidates(q, 0.02)
    assert (level, b) == (1, 5) and err == pytest.approx(0.015)
    q['lf_hip'] += 0.01
    assert lib.candidates(q, 0.02) == []
    del q['lf_hip']
    assert lib.candidates(q, 0.02) == []
    wide = lib.candidates(lib.boundary_positions(1, 5), 10.0)       # every boundary, sorted
    assert wide[0][:2] == (1, 5) and len(wide) == 1 + 3 * 8


@pytest.mark.parametrize('mutate', [
    lambda g: g.__class__(**{**g.__dict__, 'points': g.points[:-1]}),
    lambda g: g.__class__(**{**g.__dict__, 'fingerprint': '0' * 64}),
    lambda g: g.__class__(**{**g.__dict__, 'points': ((g.points[0][0],
                                                       (9.0,) + g.points[0][1][1:],
                                                       g.points[0][2]),) + g.points[1:]}),
])
def test_tampered_goals_are_refused(lib, mutate):
    g = lib.goals[(0, 1, 1)]
    with pytest.raises(loc.GoalRefused):
        loc.verify_goal(mutate(g), lib.fingerprints)


def test_a_goal_from_another_library_is_refused(lib):
    other = fx.synthetic_library(strides=(0.03,))
    g = other.goals[(0, 0, 1)]
    loc.verify_goal(g, other.fingerprints)
    with pytest.raises(loc.GoalRefused):
        loc.verify_goal(g, lib.fingerprints)
    with pytest.raises(loc.GoalRefused):
        loc.verify_goal(g.to_dict(), lib.fingerprints)


def test_library_refuses_inconsistent_templates():
    t = fx.synthetic_template(0.02)
    bad = copy.deepcopy(t)
    bad.report['failures'] = ['static_margin']
    with pytest.raises(loc.LocomotionError, match='validation failures'):
        loc.PhaseLibrary([bad], 0.2)
    with pytest.raises(loc.LocomotionError, match='FORWARD'):
        loc.PhaseLibrary([t.reversed()], 0.2)
    moving = copy.deepcopy(t)
    moving.points[moving.boundaries[3]]['velocities'][0] = 0.1
    with pytest.raises(loc.LocomotionError, match='rest points'):
        loc.PhaseLibrary([moving], 0.2)
    shifted = copy.deepcopy(t)
    shifted.points[-1]['positions'][0] = 0.01
    with pytest.raises(loc.LocomotionError, match='neutral'):
        loc.PhaseLibrary([shifted], 0.2)
    with pytest.raises(loc.LocomotionError, match='at least one'):
        loc.PhaseLibrary([], 0.2)
    with pytest.raises(loc.LocomotionError, match='strictly increasing'):
        loc.PhaseLibrary([fx.synthetic_template(0.02), fx.synthetic_template(0.02)], 0.2)


def test_speed_levels_include_the_lead_in(lib):
    for lv, t in enumerate(lib.templates):
        assert lib.speeds_m_s[lv] == pytest.approx(
            t.params.stride_m / (t.cycle_s + lib.n_phases * lib.lead_s))


def test_session_refuses_a_deadband_above_the_slowest_level(cfg, lib):
    slow = loc.PhaseLibrary([fx.synthetic_template(0.0002)], 10.0)
    with pytest.raises(loc.LocomotionError, match='deadband'):
        loc.LocomotionSession(cfg, slow, loc.ShadowTransport(slow.fingerprints))


# ==================================================================== shadow mode
def test_shadow_mode_runs_the_state_machine_and_sends_nothing(cfg, lib):
    w, s, tr = shadow_world(cfg, lib)
    st = s.status(w.wall)
    assert st['mode'] == 'shadow' and st['dispatch_gate'] is False and st['state'] == 'DISARMED'
    arm(w)
    w.cmd = fwd(lib, 1)
    w.run(14.0)
    assert s.state == loc.WALKING and tr.sent >= 18 and s.cycles >= 2
    recs = list(tr.records)
    assert [r['seq'] for r in recs] == list(range(1, tr.sent + 1))
    assert all(r['level'] == 1 for r in recs)
    st = s.status(w.wall)
    assert st['continuity_reference'].startswith('planned') and st['goals_recorded'] == tr.sent
    w.cmd = (0.0, 0.0, 0.0)
    w.run(2.0)
    assert s.state == loc.READY and s.goal is None


def test_shadow_mode_still_needs_live_joint_states(cfg, lib):
    w, s, tr = shadow_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(2.0)
    w.joint_states = False
    w.run(1.0)
    assert s.state == loc.FAULTED and s.faults[0] == loc.JOINT_STATES_STALE


def test_shadow_transport_one_goal_at_a_time(lib):
    tr = loc.ShadowTransport(lib.fingerprints)
    tr.send(lib.goals[(0, 0, 1)], 0.0)
    with pytest.raises(loc.LocomotionError, match='one goal at a time'):
        tr.send(lib.goals[(0, 1, 1)], 0.1)
    tr.cancel(0.2)
    assert [e[0] for e in tr.poll(0.3)] == ['goal_response', 'cancel_response', 'result']


# ==================================================================== walking (dispatch fake)
def test_forward_walk_cycles_and_controlled_stop(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    assert s.state == loc.DISARMED
    arm(w)
    assert s.state == loc.READY and s.b == 0
    w.cmd = fwd(lib, 0)
    w.run(15.0)
    path = [e['state'] for e in s.log if e['event'] == 'transition']
    assert path[:3] == [loc.READY, loc.STARTING, loc.WALKING]
    assert s.cycles >= 2 and plant.sent >= 18
    assert all(g.direction == 1 and g.level == 0 for g in plant.goals)
    assert_chained(plant.goals)
    w.cmd = (0.0, 0.0, 0.0)
    w.run_until(lambda: s.state == loc.READY, 3.0)
    assert s.state == loc.READY and s.goal is None and loc.STOPPING in w.states
    sent = plant.sent
    done = s.phases_completed
    assert s.planned_xy[1] == pytest.approx(0.02 * done / 9 if s.b == 0 else
                                            0.02 * (done // 9) + 0.02 * s.b / 9)
    w.run(2.0)
    assert plant.sent == sent                                # holding: nothing more sent


def test_reverse_walk_moves_backwards(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib, 0, -1)
    w.run(8.0)
    assert all(g.direction == -1 for g in plant.goals) and s.cycles <= -1
    assert s.planned_xy[1] < 0
    assert plant.goals[0].phase == lib.n_phases - 1
    assert_chained(plant.goals)


def test_reversal_takes_effect_at_the_next_phase_boundary(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: plant.sent == 4, 10.0)
    w.cmd = fwd(lib, 0, -1)
    w.run_until(lambda: plant.sent == 8, 10.0)
    dirs = [g.direction for g in plant.goals]
    assert dirs[:4] == [1, 1, 1, 1] and set(dirs[4:]) == {-1}
    assert_chained(plant.goals)                              # continuous at rest boundaries
    assert plant.goals[4].phase == plant.goals[3].phase      # the same phase played back


def test_speed_level_changes_only_at_a_cycle_boundary(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib, 0)
    w.run_until(lambda: plant.sent == 3, 10.0)
    w.cmd = fwd(lib, 2)
    w.run_until(lambda: plant.sent == 12, 20.0)
    levels = [g.level for g in plant.goals]
    assert levels[:9] == [0] * 9 and levels[9:] == [2, 2, 2]
    assert plant.goals[9].phase == 0
    assert_chained(plant.goals)


def test_lease_expiry_stops_then_disarms_and_never_rearms(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(2.0)
    w.cmd = None                                             # teleop gone: no heartbeats
    w.run(3.0)
    assert s.state == loc.DISARMED and loc.STOPPING in w.states
    assert any(e['event'] == 'transition' and e['why'] == loc.LEASE_EXPIRED for e in s.log)
    sent = plant.sent
    w.cmd = fwd(lib)                                         # the teleop reconnects
    w.run(3.0)
    assert s.state == loc.DISARMED and plant.sent == sent


def test_ready_without_any_command_disarms_after_the_lease(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.run(1.0)
    assert s.state == loc.DISARMED and plant.sent == 0


def test_unsupported_command_while_walking_stops(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(1.5)
    w.cmd = None
    w.tick()
    assert s.on_command(w.wall, (lib.speeds_m_s[0], 0, 0), (0, 0, 0.3)) == cmd.UNSUPPORTED_YAW
    w.cmd = (0.0, 0.0, 0.0)
    w._next_cmd = w.wall + 0.1
    w.tick()
    assert s.state == loc.STOPPING                           # stopped at once, phase completes
    w.run(2.0)
    assert s.state == loc.READY
    w.cmd = (lib.speeds_m_s[0], 0.01, 0.0)                   # only sideways commands now
    w.run(2.0)
    assert s.state == loc.DISARMED and loc.STOPPING not in w.states[-1:]
    assert s.lease.rejections[cmd.UNSUPPORTED_LATERAL] >= 5  # never valid: the lease expired


def test_disarm_while_walking_stops_first(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(1.5)
    ok, msg = w.request('disarm')
    assert ok and 'stopping' in msg and s.state == loc.STOPPING and s.goal is not None
    w.run(2.0)
    assert s.state == loc.DISARMED and s.goal is None


def test_home_walks_the_fewest_phases_to_the_cycle_boundary(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    assert w.request('home') == (True, 'already at a cycle boundary')
    w.cmd = fwd(lib)
    w.run_until(lambda: plant.sent == 7, 10.0)
    w.cmd = (0.0, 0.0, 0.0)
    w.run_until(lambda: s.state == loc.READY, 3.0)
    assert s.b == 7
    ok, msg = w.request('home')
    assert ok and 'forward' in msg
    w.run_until(lambda: s.state == loc.READY, 5.0)
    assert s.b == 0 and [g.direction for g in plant.goals[7:]] == [1, 1]
    # from b = 2 home goes backwards
    w.cmd = fwd(lib)
    w.run_until(lambda: plant.sent == 11, 10.0)
    w.cmd = (0.0, 0.0, 0.0)
    w.run_until(lambda: s.state == loc.READY, 3.0)
    assert s.b == 2 and w.request('home')[0]
    w.run_until(lambda: s.state == loc.READY, 5.0)
    assert s.b == 0 and [g.direction for g in plant.goals[11:]] == [-1, -1]
    assert_chained(plant.goals)


def test_home_ignores_motion_keys_but_obeys_stop(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: plant.sent == 6, 10.0)
    w.cmd = (0.0, 0.0, 0.0)
    w.run_until(lambda: s.state == loc.READY, 3.0)
    w.request('home')
    w.cmd = fwd(lib, 0, -1)                                  # ignored during homing
    w.run(0.3)
    assert s.homing == 1
    assert w.request('stop')[0] and s.state == loc.STOPPING and s.homing == 0
    w.run(2.0)
    assert s.state == loc.READY and s.b == 7                 # reverse still held: stop holds
    assert s.status(w.wall)['stop_latched'] is True
    w.cmd = (0.0, 0.0, 0.0)
    w.run(0.3)
    assert not s.stop_latched
    w.cmd = fwd(lib, 0, -1)
    w.run(0.3)
    assert s.state in (loc.STARTING, loc.WALKING) and plant.goals[-1].direction == -1


# ==================================================================== arm refusals
def _refusal(world, name='arm'):
    ok, msg = world.request(name)
    assert not ok
    return msg.split(':')[0]


def test_arm_refusals(cfg, lib):
    w, s, tr = shadow_world(cfg, lib)
    w.joint_states = False
    w.run(1.0)
    assert _refusal(w) == loc.NO_JOINT_STATES
    w.joint_states = True
    w.run(0.2)
    w.graph = dict(w.graph, foreign_action_clients=1)
    w.run(0.2)
    assert _refusal(w) == loc.OWNER_CONFLICT
    w.graph = dict(w.graph, foreign_action_clients=0, command_topic_publishers=1)
    w.run(0.2)
    assert _refusal(w) == loc.TOPIC_PUBLISHER
    w.graph = dict(w.graph, command_topic_publishers=0, joint_state_publishers=2)
    w.run(0.2)
    assert _refusal(w) == loc.JOINT_STATE_PUBLISHERS
    w.graph = dict(w.graph, joint_state_publishers=1)
    w.tilt = 0.3
    w.run(0.2)
    assert _refusal(w) == loc.BODY_TILT_EXCEEDED
    w.tilt = 0.0
    w.pose_codes = ('pose_missing_model',)
    w.run(0.2)
    assert _refusal(w) == loc.BODY_POSE_INVALID
    w.pose_codes = ()
    w.cmd = fwd(lib)
    w.run(0.3)
    assert _refusal(w) == loc.MOVING_COMMAND
    w.cmd = (0.0, 0.0, 0.0)
    w.run(0.3)
    assert s.state == loc.DISARMED and s.faults == [] and tr.sent == 0
    assert w.request('arm')[0] and s.state == loc.READY
    assert _refusal(w) == loc.NOT_DISARMED


def test_arm_refuses_an_unknown_posture(cfg, lib):
    w, s, tr = shadow_world(cfg, lib)
    w.plant.q['rr_foot_joint'] += 0.03
    w.run(1.0)
    assert _refusal(w) == loc.POSTURE


def test_fresh_process_refuses_a_mid_cycle_posture(cfg, lib):
    """No rest record from this process: the posture could be partway through an adjacent swing
    and the level is unknown, so a mid-cycle boundary is refused, never guessed."""
    w, s, tr = shadow_world(cfg, lib)
    w.plant.q = dict(lib.boundary_positions(2, 4))
    w.run(1.0)
    assert _refusal(w) == loc.POSTURE_UNVERIFIED and tr.sent == 0


def test_arm_refuses_stalled_or_reset_sim_time(cfg, lib):
    w, s, tr = shadow_world(cfg, lib)
    w.sim_advancing = False
    w.run(6.0)
    assert _refusal(w) == loc.SIM_TIME_STALLED
    w.sim_advancing = True
    w.sim -= 20.0
    w.run(0.5)
    assert _refusal(w) == loc.SIM_TIME_RESET
    w.run(cfg.monitor.sim_stall_s)
    assert w.request('arm')[0]


def test_arm_needs_a_fresh_graph_and_body_pose(cfg, lib):
    w, s, tr = shadow_world(cfg, lib)
    w.graph = None
    w.run(0.5)
    assert _refusal(w) == loc.NO_GRAPH
    w2, s2, _ = shadow_world(cfg, lib)
    w2.pose = False
    w2.run(0.5)
    assert _refusal(w2) == loc.NO_BODY_POSE


def test_dispatch_arm_needs_the_action_server(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    plant.server_ready = False
    w.run(1.0)
    assert _refusal(w) == loc.SERVER_UNAVAILABLE
    plant.server_ready = True
    w.graph = dict(w.graph, action_server_present=False)
    w.run(0.2)
    assert _refusal(w) == loc.SERVER_MISSING


# ==================================================================== faults and recovery
def test_estop_cancels_once_holds_and_needs_reset_then_arm(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(1.0)
    w.run_until(lambda: s.goal is not None and s.goal.accepted and
                w.wall - s.goal.accept_wall > lib.lead_s + 0.2, 2.0)
    sent = plant.sent
    ok, msg = w.request('estop')
    assert ok and s.state == loc.FAULTED and s.faults == [loc.ESTOP]
    w.request('estop')                                       # a second press: no second cancel
    w.run(2.0)
    assert plant.cancels == 1 and plant.sent == sent and s.goal is None
    assert _refusal(w, 'arm') == loc.RESET_REQUIRED
    assert _refusal(w, 'disarm') == loc.RESET_REQUIRED
    assert _refusal(w, 'home') == loc.NOT_READY
    assert w.request('reset')[0] and s.state == loc.DISARMED and s.faults == []
    w.run(2.0)
    assert s.state == loc.DISARMED and plant.sent == sent    # commands alone never re-arm
    w.cmd = (0.0, 0.0, 0.0)
    w.run(0.3)
    assert _refusal(w) == loc.POSTURE                        # stopped mid-phase: no boundary


def test_estop_while_disarmed_latches(cfg, lib):
    w, s, tr = shadow_world(cfg, lib)
    w.run(0.5)
    assert w.request('estop')[0] and s.state == loc.FAULTED
    assert _refusal(w) == loc.RESET_REQUIRED
    assert w.request('reset')[0] and w.request('arm')[0]


def test_reset_refused_while_the_cancelled_goal_is_unresolved(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib, scenario='cancel_ignored', fail_phase=99)
    arm(w)
    w.cmd = fwd(lib)
    w.run(1.5)
    w.request('estop')
    w.run(2.0)
    assert plant.cancels == 1 and s.goal is not None and s.goal.cancel_response == 0
    assert _refusal(w, 'reset') == loc.GOAL_UNRESOLVED
    assert _refusal(w, 'reset') == loc.GOAL_UNRESOLVED and plant.cancels == 1


@pytest.mark.parametrize('scenario, kw, fault, resolved', [
    ('reject', {}, loc.GOAL_REJECTED, True),
    ('abort', {}, loc.GOAL_ABORTED, True),
    ('tracking_drift', {'drift_rad_s': 0.3}, loc.TRACKING, True),
    ('no_result', {}, loc.RESULT_WATCHDOG, True),
    ('no_response', {}, loc.RESPONSE_TIMEOUT, False),
    ('send_raises', {}, loc.TRANSPORT_ERROR, True),
])
def test_transport_failures_fault_and_hold(cfg, lib, gate_on, scenario, kw, fault, resolved):
    w, s, plant = dispatch_world(cfg, lib, scenario=scenario, fail_phase=3, **kw)
    arm(w)
    w.cmd = fwd(lib)
    w.run(8.0)
    assert s.state == loc.FAULTED and s.faults[0] == fault
    assert plant.sent <= 3 and plant.cancels <= 1
    assert (s.goal is None) is resolved
    assert w.request('reset')[0] is resolved


def test_small_tracking_drift_is_caught_by_the_next_continuity_check(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib, scenario='tracking_drift', fail_phase=2,
                                 drift_rad_s=0.05)
    arm(w)
    w.cmd = fwd(lib)
    w.run(5.0)
    assert s.faults == [loc.CONTINUITY] and plant.sent == 2
    ev = [e for e in s.log if e['event'] == 'fault'][0]
    assert ev['reference'] == 'measured'
    assert ev['error_rad'] > cfg.dispatch.continuity_tolerance_rad


@pytest.mark.parametrize('disturb, fault', [
    (lambda w: setattr(w, 'joint_states', False), loc.JOINT_STATES_STALE),
    (lambda w: setattr(w, 'sim_advancing', False), loc.SIM_TIME_STALLED),
    (lambda w: setattr(w, 'sim', w.sim - 30.0), loc.SIM_TIME_RESET),
    (lambda w: setattr(w, 'tilt', 0.4), loc.BODY_TILT_EXCEEDED),
    (lambda w: setattr(w, 'pose', False), loc.BODY_POSE_STALE),
    (lambda w: setattr(w, 'pose_codes', ('pose_nonfinite',)), loc.BODY_POSE_INVALID),
    (lambda w: setattr(w, 'graph', None), loc.GRAPH_STALE),
    (lambda w: setattr(w, 'graph', dict(w.graph, foreign_action_clients=1)), loc.OWNER_CONFLICT),
    (lambda w: setattr(w, 'graph', dict(w.graph, command_topic_publishers=1)),
     loc.TOPIC_PUBLISHER),
    (lambda w: setattr(w, 'graph', dict(w.graph, joint_state_publishers=0)),
     loc.JOINT_STATE_PUBLISHERS),
    (lambda w: setattr(w, 'graph', dict(w.graph, action_server_present=False)),
     loc.SERVER_MISSING),
])
def test_monitors_fault_while_walking(cfg, lib, gate_on, disturb, fault):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(1.5)
    disturb(w)
    w.run(cfg.monitor.sim_stall_s + 1.0)
    assert s.state == loc.FAULTED and s.faults[0] == fault
    assert plant.cancels <= 1


def test_invalid_joint_state_while_armed_faults(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    s.on_joint_state(w.wall, w.sim, ['lf_hip'], [0.0])
    assert s.state == loc.FAULTED and s.faults == [loc.JOINT_STATES_INVALID]
    w2, s2, _ = dispatch_world(cfg, lib)
    arm(w2)
    q = w2.plant.joint_positions()
    s2.on_joint_state(w2.wall, w2.sim, list(q), [math.nan] * len(q))
    assert s2.faults == [loc.JOINT_STATES_INVALID]


def test_requests_in_every_state_never_raise(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    seen = set()
    script = ['stop', 'home', 'disarm', 'reset', 'bogus', 'arm', 'home', 'stop']
    w.run(1.0)
    for name in script:
        ok, msg = w.request(name)
        assert isinstance(ok, bool) and isinstance(msg, str)
        seen.add(s.state)
    assert _refusal(w, 'bogus') == loc.UNKNOWN_REQUEST


def test_status_is_json_serialisable_in_every_state(cfg, lib, gate_on):
    import json
    w, s, plant = dispatch_world(cfg, lib)
    w.cmd = (0.0, 0.0, 0.0)
    states = set()
    for step in (lambda: None, lambda: arm(w), lambda: setattr(w, 'cmd', fwd(lib)),
                 lambda: w.request('stop'), lambda: w.request('estop')):
        step()
        w.run(0.8)
        states.add(s.state)
        st = json.loads(json.dumps(s.status(w.wall)))
        assert st['schema'] == loc.STATUS_SCHEMA and st['state'] == s.state
    assert {loc.DISARMED, loc.READY, loc.FAULTED} <= states


# ==================================================================== keyboard logic
def test_keymap_covers_the_documented_controls():
    km = tp.KEYMAP
    assert (km['a'], km['z'], km[' '], km['x'], km['\x1b'], km['r'], km['h']) == \
        ('arm', 'disarm', 'stop', 'estop', 'estop', 'reset', 'home')
    assert (km['w'], km['s'], km['+'], km['='], km['-'], km['j'], km['l'], km['q']) == \
        (tp.FORWARD, tp.REVERSE, tp.FASTER, tp.FASTER, tp.SLOWER, tp.TURN, tp.TURN, tp.QUIT)


def test_decoder_lone_escape_is_estop_and_sequences_are_ignored():
    d = tp.KeyDecoder()
    assert d.decode('\x1b') == [('estop', None)]
    out = d.decode('\x1b[A\x1bOBw')
    assert [a for a, _ in out] == ['ignored', 'ignored', tp.FORWARD] and d.ignored == 2
    assert [a for a, _ in d.decode('W S\x03')] == [tp.FORWARD, 'stop', tp.REVERSE, tp.QUIT]
    assert d.decode('9k') == []


def test_teleop_hold_window_and_levels():
    st = tp.TeleopState(0.8)
    assert st.key(tp.FORWARD, 0.0)[1].startswith('no locomotion status')
    assert st.twist(0.1) == (0.0, 0.0, 0.0)                  # levels unknown: zero
    st.on_status({'levels_m_s': [0.001, 0.002, 0.003]})
    st.key(tp.FORWARD, 1.0)
    assert st.twist(1.5) == (0.001, 0.0, 0.0)
    assert st.twist(1.8) == (0.001, 0.0, 0.0)
    assert st.twist(1.81) == (0.0, 0.0, 0.0) and st.motion == 0   # released by timeout
    st.key(tp.FASTER, 2.0)
    st.key(tp.FASTER, 2.0)
    st.key(tp.FASTER, 2.0)
    assert st.level == 2
    st.key(tp.REVERSE, 3.0)
    assert st.twist(3.1) == (-0.003, 0.0, 0.0)
    for k in range(10):
        st.key(tp.SLOWER, 3.2)
    assert st.level == 0
    assert st.key(tp.STOP, 3.3) == ('stop', None) and st.twist(3.31) == (0.0, 0.0, 0.0)
    st.key(tp.FORWARD, 4.0)
    assert st.key(tp.TURN, 4.1) == (None, tp.TURN_MESSAGE)
    assert st.twist(4.2) == (0.001, 0.0, 0.0)                # turn never adds yaw
    assert st.key(tp.ESTOP, 4.3) == ('estop', None) and st.motion == 0
    assert st.key(tp.QUIT, 4.4) == (None, None) and st.motion == 0


def test_teleop_ignores_malformed_status_levels():
    st = tp.TeleopState(0.8)
    for bad in ({}, {'levels_m_s': []}, {'levels_m_s': [0.001, float('nan')]},
                {'levels_m_s': 'fast'}, {'levels_m_s': [-0.001]}):
        st.on_status(bad)
        assert st.levels is None
    st.on_status({'levels_m_s': [0.001, 0.002, 0.003]})
    st.level = 2
    st.on_status({'levels_m_s': [0.001]})
    assert st.level == 0


def test_status_line():
    assert 'waiting' in tp.status_line(None)
    line = tp.status_line({'state': 'FAULTED', 'mode': 'shadow', 'levels_m_s': [0.001],
                           'level': 0, 'boundary': 3, 'intent': {'direction': 0},
                           'faults': ['estop'], 'last_refusal': {'reason': 'x'}})
    assert 'FAULTED' in line and 'SHADOW' in line and 'FAULTS estop' in line
    assert '1.00 mm/s' in line


def test_crawl_template_phase_metadata_reverses_consistently():
    t = fx.synthetic_template(0.02)
    r = t.reversed()
    n = len(t.points) - 1
    assert r.boundaries == [n - i for i in reversed(t.boundaries)]
    assert r.phase_info == list(reversed(t.phase_info))
    assert r.boundary_body[0] == (0.0, 0.0) and r.boundary_body[-1] == pytest.approx((0, -0.02))
    assert isinstance(r, cr.CrawlTemplate) and r.direction == -1


def test_stop_request_holds_until_a_zero_command(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(1.5)
    ok, msg = w.request('stop')
    assert ok and 'held until a zero command' in msg and s.state == loc.STOPPING
    w.run(3.0)                                               # forward still being sent
    assert s.state == loc.READY and s.goal is None
    sent = plant.sent
    w.run(0.4)
    assert plant.sent == sent and s.state == loc.READY       # not disarmed: heartbeats valid
    w.cmd = (0.0, 0.0, 0.0)
    w.run(0.3)
    w.cmd = fwd(lib)
    w.run(0.5)
    assert plant.sent == sent + 1


def test_next_phase_waits_for_a_joint_state_received_after_the_result(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: s.phases_completed == 1, 5.0)
    assert s.awaiting_js and s.goal is None and plant.sent == 1     # sample predates the result
    w.tick()
    assert not s.awaiting_js and plant.sent == 2
    ev = [e for e in s.log if e['event'] == 'goal_sent'][-1]
    assert ev['continuity_error_rad'] < 1e-9


def test_joint_states_lost_while_waiting_at_rest_faults(cfg, lib, gate_on):
    w, s, plant = dispatch_world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: s.phases_completed == 1, 5.0)
    w.joint_states = False
    w.run(1.0)
    assert s.faults == [loc.JOINT_STATES_STALE] and plant.sent == 1 and plant.cancels == 0
