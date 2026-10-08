"""M5.5 operator command: REP-103 -> base_link axes, validation, speed levels and the lease.

Pure Python (no ROS graph)."""

import math

import pytest

from spiderx_controller import m55_command as cmd

LEVELS = (0.001, 0.002, 0.003)


@pytest.fixture
def mapper():
    return cmd.CommandMapper(LEVELS, 0.0002, 1e-6)


def lease(mapper, lease_s=0.5, rate=50.0):
    return cmd.CommandLease(mapper, lease_s, rate)


# ------------------------------------------------------------------ axes and signs
def test_rep103_forward_is_base_plus_y_and_left_is_base_minus_x():
    assert cmd.rep103_to_base(1.0, 0.0) == (0.0, 1.0)       # forward -> base +y (robot front)
    assert cmd.rep103_to_base(0.0, 1.0) == (-1.0, 0.0)      # left -> base -x (+x is robot right)
    assert cmd.rep103_to_base(-1.0, 0.0) == (0.0, -1.0)
    for v in ((0.3, -0.7), (-1.2, 2.5), (0.0, 0.0)):
        assert cmd.base_to_rep103(*cmd.rep103_to_base(*v)) == pytest.approx(v)


def test_forward_and_reverse_map_to_directions(mapper):
    i, code = mapper.map((0.002, 0, 0), (0, 0, 0))
    assert code is None and i.direction == cmd.FORWARD and i.level == 1 and i.moving
    i, code = mapper.map((-0.003, 0, 0), (0, 0, 0))
    assert code is None and i.direction == cmd.REVERSE and i.level == 2


@pytest.mark.parametrize('vx, level', [(0.001, 0), (0.0015, 0), (0.002, 1), (0.0029, 1),
                                       (0.003, 2), (0.5, 2)])
def test_speed_maps_to_the_fastest_level_not_above_the_request(mapper, vx, level):
    i, _ = mapper.map((vx, 0, 0), (0, 0, 0))
    assert i.level == level and i.granted_m_s == LEVELS[level] <= vx
    assert i.requested_m_s == vx


@pytest.mark.parametrize('vx', [0.0, 0.0001, -0.00019, 0.0009, -0.0009])
def test_below_deadband_or_slowest_level_is_a_stop_never_faster(mapper, vx):
    i, code = mapper.map((vx, 0, 0), (0, 0, 0))
    assert code is None and not i.moving and i.granted_m_s == 0.0


# ------------------------------------------------------------------ rejected input
@pytest.mark.parametrize('lin, ang, code', [
    ((0.001, 0.001, 0), (0, 0, 0), cmd.UNSUPPORTED_LATERAL),
    ((0.0, -0.01, 0), (0, 0, 0), cmd.UNSUPPORTED_LATERAL),
    ((0.001, 0, 0), (0, 0, 0.1), cmd.UNSUPPORTED_YAW),
    ((0.0, 0, 0), (0, 0, -0.001), cmd.UNSUPPORTED_YAW),
    ((0.001, 0, 0.01), (0, 0, 0), cmd.UNSUPPORTED_OTHER),
    ((0.001, 0, 0), (0.01, 0, 0), cmd.UNSUPPORTED_OTHER),
    ((0.001, 0, 0), (0, 0.01, 0), cmd.UNSUPPORTED_OTHER),
    ((math.nan, 0, 0), (0, 0, 0), cmd.NONFINITE),
    ((math.inf, 0, 0), (0, 0, 0), cmd.NONFINITE),
    ((0.001, 0, 0), (0, 0, -math.inf), cmd.NONFINITE),
    ((True, 0, 0), (0, 0, 0), cmd.NONFINITE),
    (('0.001', 0, 0), (0, 0, 0), cmd.NONFINITE),
    ((0.001, 0), (0, 0, 0), cmd.NONFINITE),
])
def test_unsupported_or_malformed_commands_are_rejected(mapper, lin, ang, code):
    assert mapper.map(lin, ang) == (None, code)


def test_tiny_numerical_noise_on_unsupported_axes_is_tolerated(mapper):
    i, code = mapper.map((0.002, 1e-9, 0), (0, 0, -1e-9))
    assert code is None and i.level == 1


@pytest.mark.parametrize('levels', [(), (0.002, 0.001), (0.001, 0.001), (0.0, 0.001),
                                    (math.nan,), (-0.001,)])
def test_mapper_refuses_bad_levels(levels):
    with pytest.raises(ValueError):
        cmd.CommandMapper(levels, 0.0002, 1e-6)


# ------------------------------------------------------------------ lease
def test_lease_starts_empty_and_needs_a_command(mapper):
    le = lease(mapper)
    assert le.current(10.0) == (cmd.STOP, 'no_command_yet')


def test_valid_command_renews_and_expires_after_lease_s(mapper):
    le = lease(mapper)
    assert le.on_command(10.0, (0.002, 0, 0), (0, 0, 0)) is None
    i, why = le.current(10.4)
    assert why is None and i.level == 1
    assert le.current(10.5)[1] is None                   # boundary: exactly lease_s
    assert le.current(10.51) == (cmd.STOP, 'lease_expired')


def test_heartbeats_keep_the_lease_without_a_release_event(mapper):
    le = lease(mapper)
    for k in range(50):                                  # 5 s of 10 Hz heartbeats
        assert le.on_command(10.0 + 0.1 * k, (0.002, 0, 0), (0, 0, 0)) is None
        assert le.current(10.0 + 0.1 * k + 0.05)[1] is None
    assert le.current(14.9 + 0.6)[1] == 'lease_expired'  # heartbeats stopped


def test_rejected_command_stops_and_does_not_renew(mapper):
    le = lease(mapper)
    le.on_command(10.0, (0.003, 0, 0), (0, 0, 0))
    assert le.on_command(10.1, (0.003, 0, 0), (0, 0, 0.2)) == cmd.UNSUPPORTED_YAW
    i, why = le.current(10.2)
    assert not i.moving and why is None                  # stopped at once, lease from 10.0
    assert le.current(10.55)[1] == 'lease_expired'
    assert le.rejections == {cmd.UNSUPPORTED_YAW: 1} and le.accepted == 1


def test_rate_limit_rejects_bursts_and_stops(mapper):
    le = lease(mapper, rate=50.0)
    assert le.on_command(10.0, (0.002, 0, 0), (0, 0, 0)) is None
    assert le.on_command(10.005, (0.002, 0, 0), (0, 0, 0)) == cmd.RATE_EXCEEDED
    assert not le.current(10.006)[0].moving
    assert le.on_command(10.03, (0.002, 0, 0), (0, 0, 0)) is None   # spacing restored
    assert le.current(10.031)[0].moving


def test_clear_forgets_intent_and_lease(mapper):
    le = lease(mapper)
    le.on_command(10.0, (0.002, 0, 0), (0, 0, 0))
    le.clear()
    assert le.current(10.01) == (cmd.STOP, 'no_command_yet')
