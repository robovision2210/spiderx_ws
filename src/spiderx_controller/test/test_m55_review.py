"""M5.5 focused-review regression tests: sim-time versus wall-time timing, post-result feedback
freshness, clock anomalies, the arm posture policy, homing, graph snapshots and the measured stop
timeline (teleop hold window + heartbeat + lease + tick).

Pure Python (no ROS graph). The dispatching fake follows SIM time like the real controller and is
used only with the M5.5 gate set True inside the test.
"""

import m55_fixtures as fx
import pytest

from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_fake as fk
from spiderx_controller import m55_locomotion as loc
from spiderx_controller import m55_teleop as tp


@pytest.fixture(scope='module')
def cfg():
    return fx.config()


@pytest.fixture(scope='module')
def lib():
    return fx.synthetic_library()


@pytest.fixture
def gate_on(monkeypatch):
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', True)      # tests only


def world(cfg, lib, scenario='ok', rtf=1.0, graph_period_s=None, start=(0, 0), **kw):
    plant = fk.FakeLocomotionTransport(lib, lib.boundary_positions(*start), scenario=scenario,
                                       use_sim=True, **kw)
    s = loc.LocomotionSession(cfg, lib, plant)
    return fk.FakeWorld(s, plant, rtf=rtf, graph_period_s=graph_period_s), s, plant


def arm(w):
    w.cmd = (0.0, 0.0, 0.0)
    w.run(1.0)
    ok, msg = w.request('arm')
    assert ok, msg


def fwd(lib, level=0, sign=1):
    return (sign * lib.speeds_m_s[level], 0.0, 0.0)


def refusal(w, name='arm'):
    ok, msg = w.request(name)
    assert not ok, msg
    return msg.split(':')[0]


def sent_walls(s):
    return [e['wall'] for e in s.log if e['event'] == 'goal_sent']


# ==================================================================== sim time vs wall time
def test_low_real_time_factor_does_not_trip_the_watchdog(cfg, lib, gate_on):
    """Defect fixed: the result watchdog compared WALL time with the SIM-time phase duration. At
    RTF 0.2 every phase takes 5x its duration in wall time; the old wall limit (duration + 2 s)
    would have faulted each one."""
    w, s, plant = world(cfg, lib, rtf=0.2)
    arm(w)
    w.cmd = fwd(lib)
    w.run(25.0)
    assert s.faults == [] and s.phases_completed >= 3
    d = cfg.dispatch
    accepted = [e['wall'] for e in s.log if e['event'] == 'goal_response']
    done = [e['wall'] for e in s.log if e['event'] == 'phase_done']
    old_limit = plant.goals[0].duration_s + d.goal_time_tolerance_s + d.result_margin_s
    assert done[0] - accepted[0] > old_limit         # the old wall watchdog would have fired


def test_watchdog_measures_sim_time(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib, scenario='no_result', fail_phase=1)
    arm(w)
    w.cmd = fwd(lib)
    w.run(5.0)
    ev = [e for e in s.log if e['event'] == 'fault'][0]
    assert ev['reason'] == loc.RESULT_WATCHDOG and ev['basis'] == 'sim'
    assert ev['sim_elapsed_s'] > ev['limit_s'] and plant.cancels == 1


def test_wall_backstop_below_the_minimum_real_time_factor(cfg, lib, gate_on):
    """RTF 0.02 is below min_real_time_factor 0.1: a 0.6 s phase needs 30 s of wall time, past
    the backstop (limit / 0.1 = 26 s): treated as not progressing (documented assumption)."""
    w, s, plant = world(cfg, lib, rtf=0.02)
    arm(w)
    w.cmd = fwd(lib)
    w.run(28.0)
    ev = [e for e in s.log if e['event'] == 'fault'][0]
    assert ev['reason'] == loc.RESULT_WATCHDOG and ev['basis'] == 'wall_backstop'


def test_paused_simulation_freezes_the_phase_and_faults_on_the_stall(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: s.goal is not None and s.goal.accepted, 2.0)
    w.run(0.2)
    q_before = plant.joint_positions()
    w.sim_advancing = False                          # Gazebo paused
    w.run(cfg.monitor.sim_stall_s + 0.5)
    assert s.faults == [loc.SIM_TIME_STALLED] and plant.cancels == 1
    assert plant.joint_positions() == q_before       # nothing progressed while paused


def test_shadow_phases_complete_in_sim_time(cfg, lib):
    tr = loc.ShadowTransport(lib.fingerprints)
    s = loc.LocomotionSession(cfg, lib, tr)
    w = fk.FakeWorld(s, fk.StaticPlant(lib.boundary_positions(0, 0)), rtf=0.25)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: s.phases_completed == 1, 10.0)
    sent, done = sent_walls(s)[0], [e['wall'] for e in s.log if e['event'] == 'phase_done'][0]
    assert done - sent >= tr.records[0]['duration_s'] / 0.25 - 0.1
    w.sim_advancing = False
    n = s.phases_completed
    w.run(3.0)
    assert s.phases_completed == n                    # no sim time, no completion


# ==================================================================== post-result freshness
def _to_awaiting(cfg, lib):
    w, s, plant = world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: s.phases_completed == 1, 5.0)
    assert s.awaiting_js and s.goal is None and plant.sent == 1
    return w, s, plant


def _inject(s, wall, stamp, q):
    s.on_joint_state(wall, stamp, list(q), list(q.values()))
    s.step(wall)


def test_duplicate_sample_after_the_result_is_not_fresh(cfg, lib, gate_on):
    w, s, plant = _to_awaiting(cfg, lib)
    q, newest = plant.joint_positions(), s.js[1]
    _inject(s, w.wall + 0.01, newest, q)              # received later, measured at the same time
    assert s.awaiting_js and plant.sent == 1
    # a newer measurement taken at or after the planned end (acceptance stamp + duration; this
    # can trail the true end by the response latency, which only lengthens the wait)
    _inject(s, w.wall + 0.02, max(newest, s.rest_required_stamp) + 0.01, q)
    assert not s.awaiting_js and plant.sent == 2


def test_queued_older_sample_after_the_result_never_counts(cfg, lib, gate_on):
    w, s, plant = _to_awaiting(cfg, lib)
    q, newest = plant.joint_positions(), s.js[1]
    _inject(s, w.wall + 0.01, newest - 0.0005, q)     # reordered within 1 ms: discarded
    assert s.out_of_order == 1 and s.awaiting_js and plant.sent == 1
    _inject(s, w.wall + 0.02, newest - 0.1, q)        # 100 ms old, delivered late
    assert s.faults == [loc.SIM_TIME_RESET] and plant.sent == 1 and plant.cancels == 0


def test_sample_measured_before_the_planned_end_is_not_fresh(cfg, lib, gate_on):
    w, s, plant = _to_awaiting(cfg, lib)
    q, newest = plant.joint_positions(), s.js[1]
    s.rest_required_stamp = newest + 0.5              # e.g. the result arrived early
    _inject(s, w.wall + 0.01, newest + 0.1, q)
    assert s.awaiting_js and plant.sent == 1
    _inject(s, w.wall + 0.02, newest + 0.6, q)
    assert plant.sent == 2


def test_the_wait_for_a_fresh_sample_is_bounded(cfg, lib, gate_on):
    w, s, plant = _to_awaiting(cfg, lib)
    w.sim_advancing = False                           # receipts continue, stamps frozen
    w.run(cfg.dispatch.post_result_wait_s + 0.2)
    assert s.faults == [loc.POST_RESULT_TIMEOUT] and plant.cancels == 0
    assert s.rest == (0, 1)                           # the phase did complete
    w.sim_advancing = True
    w.run(cfg.monitor.sim_stall_s)
    assert w.request('reset')[0]
    w.cmd = (0.0, 0.0, 0.0)
    w.run(0.3)
    ok, msg = w.request('arm')
    assert ok and (s.level, s.b) == (0, 1), msg


# ==================================================================== arm posture policy
@pytest.mark.parametrize('cands, rest, shadow, expect', [
    ([], None, False, loc.POSTURE),
    ([(None, 0, 0.001)], None, False, (0, 0)),
    ([(None, 0, 0.001)], (2, 5), False, (2, 0)),
    ([(None, 0, 0.001)], None, True, (0, 0)),
    ([(None, 0, 0.01), (1, 3, 0.015)], (1, 3), False, loc.POSTURE_AMBIGUOUS),
    ([(1, 3, 0.001)], None, False, loc.POSTURE_UNVERIFIED),
    ([(1, 3, 0.001)], (1, 3), True, loc.POSTURE_UNVERIFIED),
    ([(1, 3, 0.001)], (1, 3), False, (1, 3)),
    ([(1, 3, 0.001)], (2, 3), False, loc.POSTURE_NOT_LAST_REST),
    ([(0, 3, 0.004), (1, 3, 0.012)], (1, 3), False, (1, 3)),   # record resolves close levels
    ([(0, 3, 0.004), (1, 3, 0.012)], None, False, loc.POSTURE_UNVERIFIED),
    ([(1, 3, 0.001)], (1, 0), False, loc.POSTURE_NOT_LAST_REST),
])
def test_resolve_arm_posture(cands, rest, shadow, expect):
    level, b, _ = loc.resolve_arm_posture(cands, rest, shadow)
    if isinstance(expect, tuple):
        assert (level, b) == expect
    else:
        assert level is None and b == expect


def _walk_to(w, s, plant, lib, n_goals, level=0):
    w.cmd = fwd(lib, level)
    w.run_until(lambda: plant.sent == n_goals, 20.0)
    w.cmd = (0.0, 0.0, 0.0)
    w.run_until(lambda: s.state == loc.READY, 5.0)


def test_rearm_at_the_recorded_rest_keeps_its_level(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib)
    arm(w)
    _walk_to(w, s, plant, lib, 3)
    assert s.rest == (0, 3)
    assert w.request('disarm')[0] and s.state == loc.DISARMED
    w.run(0.5)
    ok, msg = w.request('arm')
    assert ok and (s.level, s.b) == (0, 3), msg
    w.cmd = fwd(lib, 2)                                # a different level requested mid-cycle
    w.run_until(lambda: plant.sent == 9, 20.0)
    assert [g.level for g in plant.goals[3:9]] == [0] * 6 and plant.goals[8].b_to == 9
    w.run_until(lambda: plant.sent == 10, 5.0)
    assert plant.goals[9].level == 2                   # from the cycle boundary on


def test_rearm_refused_when_the_robot_is_not_at_its_recorded_rest(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib)
    arm(w)
    _walk_to(w, s, plant, lib, 3)
    w.request('disarm')
    plant.q = [lib.boundary_positions(0, 5)[n] for n in plant.names]   # moved externally
    w.run(0.5)
    assert refusal(w) == loc.POSTURE_NOT_LAST_REST


def test_an_interrupted_goal_leaves_the_rest_unknown(cfg, lib, gate_on):
    """Emergency stop during the lead-in of the 4th goal: the robot is still within tolerance of
    boundary 3, but the goal was interrupted, so the posture could be partway into the phase."""
    w, s, plant = world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: plant.sent == 4 and s.goal is not None and s.goal.accepted, 20.0)
    w.run(0.05)
    assert w.request('estop')[0] and s.rest is None
    w.run(1.0)
    assert w.request('reset')[0]
    w.cmd = (0.0, 0.0, 0.0)
    w.run(0.3)
    cands = lib.candidates(plant.joint_positions(), cfg.dispatch.continuity_tolerance_rad)
    assert (0, 3) in [c[:2] for c in cands]            # near boundary 3 ...
    assert refusal(w) == loc.POSTURE_UNVERIFIED         # ... and still refused


def test_an_interrupted_first_goal_at_neutral_can_rearm(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: s.goal is not None and s.goal.accepted, 2.0)
    w.request('estop')
    w.run(1.0)
    w.request('reset')
    w.cmd = (0.0, 0.0, 0.0)
    w.run(0.3)
    ok, msg = w.request('arm')
    assert ok and s.b == 0, msg


def test_a_rejected_goal_keeps_the_rest_where_it_was(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib, scenario='reject', fail_phase=3)
    arm(w)
    w.cmd = fwd(lib)
    w.run(5.0)
    assert s.faults == [loc.GOAL_REJECTED] and s.rest == (0, 2)
    w.request('reset')
    w.cmd = (0.0, 0.0, 0.0)
    w.run(0.3)
    ok, msg = w.request('arm')
    assert ok and s.b == 2, msg


def test_a_goal_that_completes_despite_the_cancel_updates_the_rest(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib, scenario='cancel_late')
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: plant.sent == 2 and s.goal is not None and s.goal.accepted, 10.0)
    w.request('estop')
    w.run(1.5)
    assert s.goal is None and s.rest == (0, 2) and s.b == 2
    assert w.request('reset')[0]


# ==================================================================== graph snapshots
def test_arm_needs_a_snapshot_younger_than_one_graph_period(cfg, lib):
    tr = loc.ShadowTransport(lib.fingerprints)
    s = loc.LocomotionSession(cfg, lib, tr)
    w = fk.FakeWorld(s, fk.StaticPlant(lib.boundary_positions(0, 0)), graph_period_s=2.5)
    w.cmd = (0.0, 0.0, 0.0)
    w.run(1.6)                                         # last snapshot 1.5 s old (< stale 3 s)
    assert refusal(w) == loc.GRAPH_STALE


def test_a_late_competing_commander_is_seen_at_the_next_snapshot(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib, graph_period_s=1.0)
    arm(w)
    w.cmd = fwd(lib)
    w.run(1.5)
    t0 = w.wall
    w.graph = dict(w.graph, foreign_action_clients=1)
    w.run_until(lambda: s.state == loc.FAULTED, 2.0)
    assert s.faults == [loc.OWNER_CONFLICT]
    assert w.wall - t0 <= cfg.monitor.graph_period_s + w.dt + 1e-9


def test_a_commander_between_snapshots_is_not_seen(cfg, lib, gate_on):
    """The documented limitation: graph checks are snapshots, not a lock."""
    w, s, plant = world(cfg, lib, graph_period_s=1.0)
    arm(w)
    w.cmd = fwd(lib)
    w.run_until(lambda: abs(w._next_graph - w.wall - 0.6) < 1e-6, 2.0)
    clean = dict(w.graph)
    w.graph = dict(clean, foreign_action_clients=1)
    w.run(0.3)                                         # appears and leaves between snapshots
    w.graph = clean
    w.run(2.0)
    assert s.faults == []


# ==================================================================== homing
def test_home_goes_through_the_dispatch_gate(cfg, lib, monkeypatch):
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', True)
    w, s, plant = world(cfg, lib)
    arm(w)
    _walk_to(w, s, plant, lib, 3)
    sent = plant.sent
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', False)
    ok, msg = w.request('home')
    assert not ok and s.faults == [loc.DISPATCH_DISABLED] and plant.sent == sent


@pytest.mark.parametrize('b', range(1, 9))
def test_home_takes_the_fewest_phases(cfg, lib, gate_on, b):
    w, s, plant = world(cfg, lib)
    arm(w)
    _walk_to(w, s, plant, lib, b)
    assert s.b == b and w.request('home')[0]
    w.run_until(lambda: s.state == loc.READY and s.b == 0, 20.0)
    homing = plant.goals[b:]
    assert len(homing) == min(b, 9 - b) <= 4
    assert len({g.direction for g in homing}) == 1


def test_reset_and_rearm_after_a_fault_during_homing_never_move(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib)
    arm(w)
    _walk_to(w, s, plant, lib, 6)
    w.request('home')
    w.run(0.3)
    w.request('estop')
    w.run(1.0)
    sent = plant.sent
    assert s.homing == 0 and w.request('reset')[0]
    w.cmd = (0.0, 0.0, 0.0)
    w.run(2.0)
    assert plant.sent == sent and s.homing == 0
    w.cmd = None                                       # teleop gone, then back
    w.run(1.0)
    w.cmd = (0.0, 0.0, 0.0)
    w.run(1.0)
    assert plant.sent == sent and s.state == loc.DISARMED


@pytest.mark.parametrize('state_setup', ['disarmed', 'faulted', 'walking'])
def test_home_only_from_ready(cfg, lib, gate_on, state_setup):
    w, s, plant = world(cfg, lib)
    w.run(1.0)
    if state_setup == 'faulted':
        w.request('estop')
    elif state_setup == 'walking':
        arm(w)
        w.cmd = fwd(lib)
        w.run(0.5)
    assert refusal(w, 'home') == loc.NOT_READY


# ==================================================================== measured stop timeline
def _teleop_world(cfg, lib, rtf=1.0):
    w, s, plant = world(cfg, lib, rtf=rtf)
    st = tp.TeleopState(cfg.teleop.hold_s)
    st.on_status({'levels_m_s': list(lib.speeds_m_s)})
    return w, s, plant, st


def _drive(w, st, until, presses=(), dt_key=1.0 / 30):
    """Run the world to wall time `until`, pressing `w` at each (start, end) interval at the
    terminal repeat rate after the first press, feeding the teleop's twist as the command."""
    while w.wall < until - 1e-9:
        for start, end, delay in presses:
            t = w.wall
            first = t - start < w.dt
            repeat = t - start >= delay and ((t - start - delay) % dt_key) < w.dt
            if start <= t < end and (first or repeat):
                st.key(tp.FORWARD, t)
        w.cmd = st.twist(w.wall)
        w.tick()


def test_key_release_stops_within_hold_plus_heartbeat_plus_tick(cfg, lib, gate_on):
    w, s, plant, st = _teleop_world(cfg, lib)
    arm(w)
    t0 = w.wall
    release = t0 + 3.0
    _drive(w, st, release + 5.0, presses=[(t0, release, 0.5)])
    stops = [e['wall'] for e in s.log if e['event'] == 'transition' and
             e['why'] == 'intent_stop']                 # STOPPING, or READY at a boundary
    assert stops, 'no stop'
    last_key = st.motion_wall                          # the last auto-repeat before release
    assert release - 0.05 <= last_key < release
    bound = cfg.teleop.hold_s + 1.0 / cfg.teleop.heartbeat_hz + w.dt
    assert stops[0] - last_key <= bound + 1e-6
    assert stops[0] - last_key >= cfg.teleop.hold_s - 1e-6
    assert max(sent_walls(s)) <= last_key + bound + 1e-6     # nothing sent after detection
    assert s.state == loc.READY


def test_a_repeat_delay_longer_than_the_hold_window_stutters_and_warns(cfg, lib, gate_on):
    st = tp.TeleopState(cfg.teleop.hold_s)
    st.on_status({'levels_m_s': [0.001]})
    st.key(tp.FORWARD, 0.0)
    assert st.twist(0.79) == (0.001, 0.0, 0.0) and st.twist(0.9) == (0.0, 0.0, 0.0)
    service, note = st.key(tp.FORWARD, 1.0)            # first auto-repeat after 1.0 s
    assert 'hold window' in note and st.repeat_gaps == [1.0]
    _, note = st.key(tp.FORWARD, 1.033)
    assert note is None and st.twist(1.05) == (0.001, 0.0, 0.0)


def test_a_held_key_with_a_short_repeat_delay_is_continuous(cfg, lib, gate_on):
    w, s, plant, st = _teleop_world(cfg, lib)
    arm(w)
    t0 = w.wall
    gaps = []
    while w.wall < t0 + 4.0:
        if w.wall - t0 < w.dt or (w.wall - t0 >= 0.5):
            st.key(tp.FORWARD, w.wall)
        gaps.append(st.twist(w.wall)[0] == 0.0)
        w.cmd = st.twist(w.wall)
        w.tick()
    assert not any(gaps[1:])
    assert [e['why'] for e in s.log if e['event'] == 'transition' and
            e['why'] == 'intent_stop'] == []


def test_teleop_loss_stops_within_lease_plus_tick(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(2.0)
    last_cmd = s.lease.last_valid_wall
    w.cmd = None                                       # process killed / network partition
    w.run_until(lambda: s.state == loc.STOPPING, 2.0)
    assert w.wall - last_cmd <= cfg.command.lease_s + w.dt + 1e-6
    assert max(sent_walls(s)) <= last_cmd + cfg.command.lease_s + w.dt + 1e-6
    w.run(3.0)
    assert s.state == loc.DISARMED


def test_space_stops_at_once(cfg, lib, gate_on):
    w, s, plant = world(cfg, lib)
    arm(w)
    w.cmd = fwd(lib)
    w.run(1.5)
    t = w.wall
    assert w.request('stop')[0] and s.state == loc.STOPPING
    w.run(3.0)
    assert max(sent_walls(s)) <= t and s.state == loc.READY


def test_a_tap_commits_at_most_the_phases_started_inside_the_hold_window(cfg, lib, gate_on):
    w, s, plant, st = _teleop_world(cfg, lib)
    arm(w)
    t0 = w.wall
    st.key(tp.FORWARD, t0)                             # one press, no repeat
    _drive(w, st, t0 + 5.0)
    bound = cfg.teleop.hold_s + 1.0 / cfg.teleop.heartbeat_hz + w.dt
    assert 1 <= plant.sent and max(sent_walls(s)) <= t0 + bound + 1e-6
    assert s.state == loc.READY
