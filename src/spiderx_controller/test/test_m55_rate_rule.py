"""colcon test: the M5.5 windowed receive-rate rule (m55_command.CommandLease). Fakes only.

The Cloud shadow session of 2026-10-13 (RTF 0.20) showed the old per-pair rule (two messages
< 20 ms apart -> reject, intent STOP) stopping a held walk: the teleop published at 10 Hz, but
heartbeats delayed by up to 197 ms reached the node bunched. The windowed rule rejects only a
sustained rate above max_rate_hz over rate_window_s, or more than max_burst messages within
burst_window_s. Covers: bunched-but-normal heartbeats pass (incl. the recorded gap pattern); a
sustained flood and a stuck-key flood are rejected; a burst above the hard cap is rejected;
window boundaries; rejected messages keep counting; config refusals; the locomotion session in
both gate modes (shadow transport with the gate False, dispatching fake with the gate True).
"""
import pytest

import m55_fixtures as fx

from spiderx_controller import m55_command as cmd
from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_fake as fk
from spiderx_controller import m55_locomotion as loc

LEVELS = (0.001, 0.002, 0.003)
FWD = (0.002, 0.0, 0.0)
ZERO = (0.0, 0.0, 0.0)
# heartbeat gaps (ms) seen by the recorder around the spurious rejection, 2026-10-13
RECORDED_GAPS_MS = (100.2, 100.3, 99.9, 100.1, 100.7, 141.3, 118.4, 197.4, 48.9, 100.6,
                    100.9, 100.2, 100.6)


@pytest.fixture
def le():
    return cmd.CommandLease(cmd.CommandMapper(LEVELS, 0.0002, 1e-6), 0.5, 20.0, 1.0, 0.1, 4)


@pytest.fixture(scope='module')
def cfg():
    return fx.config()


@pytest.fixture(scope='module')
def lib():
    return fx.synthetic_library()


def feed(le, walls, linear=FWD):
    return [le.on_command(w, linear, ZERO) for w in walls]


def times_from_gaps(start, gaps_ms):
    out, t = [start], start
    for g in gaps_ms:
        t = round(t + g / 1000.0, 9)
        out.append(t)
    return out


# ------------------------------------------------------------------ the committed parameters
def test_committed_parameters(cfg):
    c = cfg.command
    assert (c.max_rate_hz, c.rate_window_s, c.burst_window_s, c.max_burst) == (20.0, 1.0, 0.1, 4)
    assert c.max_rate_hz == 2 * cfg.teleop.heartbeat_hz                 # 2x the 10 Hz heartbeat
    assert c.burst_window_s == pytest.approx(1.0 / cfg.teleop.heartbeat_hz)


# ------------------------------------------------------------------ normal traffic passes
def test_steady_heartbeats_pass(le):
    assert set(feed(le, [10.0 + 0.1 * k for k in range(100)])) == {None}
    assert le.rejections == {} and le.accepted == 100


def test_recorded_jitter_pattern_passes(le):
    walls = times_from_gaps(10.0, RECORDED_GAPS_MS * 5)
    assert set(feed(le, walls)) == {None}


@pytest.mark.parametrize('bunch', [2, 3, 4])
def test_bunched_heartbeats_after_a_delivery_stall_pass(le, bunch):
    """A stall of (bunch - 1) heartbeat periods delivers that backlog within 1 ms."""
    walls = [10.0 + 0.1 * k for k in range(10)]
    stall_end = walls[-1] + 0.1 * bunch
    walls += [stall_end + 0.0003 * i for i in range(bunch)]
    walls += [stall_end + 0.1 * k for k in range(1, 20)]
    assert set(feed(le, walls)) == {None}
    assert le.current(walls[-1])[0].moving


def test_clean_quit_zero_right_after_a_heartbeat_passes(le):
    feed(le, [10.0 + 0.1 * k for k in range(5)])
    assert le.on_command(10.4003, ZERO, ZERO) is None                 # 0.3 ms later, as recorded


# ------------------------------------------------------------------ floods are rejected
def test_stuck_key_flood_at_30_hz_is_rejected_within_a_second(le):
    walls = [10.0 + k / 30.0 for k in range(60)]
    res = feed(le, walls)
    first = res.index(cmd.RATE_EXCEEDED)
    assert first == 20 and walls[first] - walls[0] < 0.7                 # the 21st in the window
    assert all(r == cmd.RATE_EXCEEDED for r in res[first:])             # stays rejected
    assert not le.current(walls[-1])[0].moving


def test_genuine_flood_is_rejected_by_the_burst_cap(le):
    walls = [10.0 + 0.01 * k for k in range(20)]                         # 100 Hz
    res = feed(le, walls)
    assert res[:4] == [None] * 4 and res[4] == cmd.RATE_EXCEEDED          # within 40 ms
    assert set(res[4:]) == {cmd.RATE_EXCEEDED}


def test_burst_above_the_hard_cap_is_rejected_even_at_a_low_average(le):
    walls = [10.0, 10.5, 11.0] + [11.5 + 0.001 * i for i in range(5)]
    res = feed(le, walls)
    assert res[:7] == [None] * 7 and res[7] == cmd.RATE_EXCEEDED
    assert not le.current(11.51)[0].moving


def test_after_a_flood_the_windows_drain_and_valid_commands_move_again(le):
    feed(le, [10.0 + 0.01 * k for k in range(30)])                       # rejected flood
    assert le.on_command(10.35, FWD, ZERO) == cmd.RATE_EXCEEDED          # still > 20 in 1 s
    assert le.on_command(11.40, FWD, ZERO) is None                       # windows drained
    assert le.current(11.41)[0].moving


def test_rejected_messages_do_not_renew_the_lease(le):
    le.on_command(10.0, FWD, ZERO)
    res = feed(le, [10.3 + 0.001 * i for i in range(6)])
    assert res == [None] * 4 + [cmd.RATE_EXCEEDED] * 2
    i, why = le.current(10.31)
    assert not i.moving and why is None
    assert le.last_valid_wall == pytest.approx(10.303)                   # the 4th, not the 6th
    assert le.current(10.8035)[1] == 'lease_expired'                     # 10.305 would hold it


# ------------------------------------------------------------------ boundaries
def test_sustained_cap_boundary(le):
    walls = [10.0 + 0.045 * k for k in range(20)]                       # 20 within 0.855 s
    assert set(feed(le, walls)) == {None}
    assert le.on_command(10.9, FWD, ZERO) == cmd.RATE_EXCEEDED          # the 21st in (9.9, 10.9]


def test_sustained_window_edge_is_excluded(le):
    walls = [10.0 + 0.05 * k for k in range(20)]                        # 10.00 .. 10.95
    assert set(feed(le, walls)) == {None}
    assert le.on_command(11.0, FWD, ZERO) is None                        # 10.00 left the window


def test_burst_cap_boundary_and_edge(le):
    assert set(feed(le, [10.0, 10.02, 10.04, 10.06])) == {None}          # exactly 4: pass
    assert le.on_command(10.1, FWD, ZERO) is None                        # 10.00 left (t - 0.1)
    assert le.on_command(10.11, FWD, ZERO) == cmd.RATE_EXCEEDED          # 5 in (10.01, 10.11]


# ------------------------------------------------------------------ configuration
@pytest.mark.parametrize('key, value, fragment', [
    ('max_burst', 1, 'max_burst'), ('max_burst', 2.5, 'max_burst'),
    ('max_burst', 30, 'max_burst'), ('burst_window_s', 2.0, 'burst_window_s'),
    ('rate_window_s', 0.0, 'rate_window_s'), ('max_rate_hz', 10.0, 'max_rate_hz'),
])
def test_config_refuses_unsafe_rate_parameters(key, value, fragment):
    import copy
    import yaml
    with open(f'{fx.CONFIG_DIR}/m55_locomotion.yaml') as f:
        d = yaml.safe_load(f)
    d = copy.deepcopy(d)
    d['command'][key] = value
    with pytest.raises(loc.LocomotionError, match=fragment):
        loc.parse_config(d)


# ------------------------------------------------------------------ the session, both gate modes
def _walk_with_schedule(world, s, lib, walls):
    """Arm, then deliver forward commands at the given receipt walls between ticks."""
    world.run(1.0)
    ok, msg = world.request('arm')
    assert ok, msg
    world.cmd = None
    t0 = world.wall
    pending = [t0 + w for w in walls]
    states = set()
    while pending:
        while pending and pending[0] <= world.wall + world.dt + 1e-9:
            s.on_command(pending.pop(0), (lib.speeds_m_s[0], 0.0, 0.0), (0.0, 0.0, 0.0))
        world.tick()
        states.add(s.state)
    return states


def _schedules():
    steady = [0.1 * k for k in range(1, 40)]
    bunched = times_from_gaps(0.1, RECORDED_GAPS_MS * 3)
    stall = bunched[-1] + 0.4                                            # < lease_s 0.5
    bunched += [stall + 0.0003 * i for i in range(4)] + [stall + 0.1 * k for k in range(1, 20)]
    return steady, bunched


def _session_case(cfg, lib, gate):
    if gate:
        plant = fk.FakeLocomotionTransport(lib, lib.boundary_positions(0, 0), use_sim=True)
        s = loc.LocomotionSession(cfg, lib, plant)
        world = fk.FakeWorld(s, plant)
    else:
        s = loc.LocomotionSession(cfg, lib, loc.ShadowTransport(lib.fingerprints))
        world = fk.FakeWorld(s, fk.StaticPlant(lib.boundary_positions(0, 0)))
    return world, s


@pytest.mark.parametrize('gate', [False, True], ids=['gate_false_shadow', 'gate_true_fake'])
def test_session_keeps_walking_under_bunched_heartbeats(cfg, lib, monkeypatch, gate):
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', gate)    # tests only
    world, s = _session_case(cfg, lib, gate)
    _, bunched = _schedules()
    states = _walk_with_schedule(world, s, lib, bunched)
    assert s.lease.rejections == {}
    assert loc.WALKING in states and loc.STOPPING not in states and s.state == loc.WALKING


@pytest.mark.parametrize('gate', [False, True], ids=['gate_false_shadow', 'gate_true_fake'])
def test_session_stops_on_a_stuck_key_flood(cfg, lib, monkeypatch, gate):
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', gate)    # tests only
    world, s = _session_case(cfg, lib, gate)
    steady, _ = _schedules()
    flood = steady + [steady[-1] + 0.1 + k / 30.0 for k in range(60)]
    states = _walk_with_schedule(world, s, lib, flood)
    assert s.lease.rejections.get(cmd.RATE_EXCEEDED, 0) >= 1
    assert loc.WALKING in states and loc.STOPPING in states


@pytest.mark.parametrize('gate', [False, True], ids=['gate_false_shadow', 'gate_true_fake'])
def test_session_stops_on_a_burst_above_the_hard_cap(cfg, lib, monkeypatch, gate):
    """Five messages within 4 ms at a 10 Hz average: only the burst cap catches this."""
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', gate)    # tests only
    world, s = _session_case(cfg, lib, gate)
    steady, _ = _schedules()
    burst = steady + [steady[-1] + 0.1 + 0.001 * i for i in range(5)]
    states = _walk_with_schedule(world, s, lib, burst)
    assert s.lease.rejections == {cmd.RATE_EXCEEDED: 1}
    assert loc.WALKING in states and loc.STOPPING in states
