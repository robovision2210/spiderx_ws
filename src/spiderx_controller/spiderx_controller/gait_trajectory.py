"""M4.5 OFFLINE foot-trajectory generator (body frame, time-parameterised, analytic velocities).

Every foot target is the leg's CAD-neutral derived tip (from the URDF, via leg_kinematics) plus an
OFFSET computed here. The offset depends only on the GaitSpec and the leg's phase (gait_phase.py).
Frame base_link: +x right, +y front, +z up; the body moves along +y at constant speed v.

Stance (foot planted on flat ground; the body moves over it):
    the foot moves backwards relative to the body at speed v, from +L/2 to -L/2 about the leg's
    stance centre, at the stance height (z offset = stance_height_offset_m).

Swing (cycloid in the WORLD frame, then converted to the body frame):
    world advance  x_w(s) = S (s - sin(2 pi s) / (2 pi))       S = stride length = L / beta
    lift           z(s)   = h (1 - cos(2 pi s)) / 2              h = step height
    body frame     y(s)   = -L/2 + x_w(s) - v * (s * tau)        tau = swing duration = (1 - beta) T
The cycloid starts and ends with zero WORLD velocity, so the foot velocity is continuous at
lift-off and touch-down (stance foot = zero world velocity). The vertical ACCELERATION is not
continuous there; gait_metrics reports it.

Pure functions: no IK, no ROS, no robot command.
"""

from dataclasses import dataclass
import math

from spiderx_controller import gait_phase as gp
from spiderx_controller import leg_kinematics as lk


@dataclass(frozen=True)
class FootSample:
    leg: str
    phase: str              # gait_phase.SWING or STANCE
    progress: float         # s in [0, 1)
    offset: tuple           # (x, y, z) from the neutral tip, base_link, m
    velocity: tuple         # (vx, vy, vz) in the BODY frame, m/s (analytic)
    target: tuple           # neutral tip + offset, base_link, m (None if no tips given)


def _swing_duration(spec):
    return (1.0 - spec.duty_factor) * spec.cycle_period_s


def foot_offset_and_velocity(spec, leg, u):
    """(phase info, offset (x, y, z), body-frame velocity (vx, vy, vz)) of one foot at time u."""
    ph = gp.leg_phase(spec, leg, u)
    L, h, v = spec.step_length_m, spec.step_height_m, spec.body_speed_m_s
    cx, cy = spec.stance_center_offset_m
    z0 = spec.stance_height_offset_m
    s = ph.progress
    if ph.phase == gp.STANCE:
        y, dz, vy, vz = L / 2.0 - L * s, 0.0, -v, 0.0
    else:
        S, tau = spec.stride_length_m, _swing_duration(spec)
        two_pi_s = 2.0 * math.pi * s
        world = S * (s - math.sin(two_pi_s) / (2.0 * math.pi))
        y = -L / 2.0 + world - v * (s * tau)
        dz = h * (1.0 - math.cos(two_pi_s)) / 2.0
        vy = (S / tau) * (1.0 - math.cos(two_pi_s)) - v
        vz = (h / tau) * math.pi * math.sin(two_pi_s)
    return ph, (cx, cy + y, z0 + dz), (0.0, vy, vz)


def foot_sample(spec, leg, u, tip0=None):
    """One FootSample. tip0: the leg's CAD-neutral tip (LegGeometry.tip0) to form the target."""
    leg = lk.resolve_leg(leg, allow_all_legs=True)
    ph, off, vel = foot_offset_and_velocity(spec, leg, u)
    target = tuple(t + o for t, o in zip(tip0, off)) if tip0 is not None else None
    return FootSample(leg, ph.phase, ph.progress, off, vel, target)


def sample_cycle(spec, n, tips=None):
    """Deterministic samples of one cycle.

    tips: optional {leg: neutral tip (x, y, z)} for all four legs (from leg_kinematics geometry).
    Returns a list of dicts {k, u, t, support: tuple of stance legs, feet: {leg: FootSample}}.
    """
    out = []
    for k, u in enumerate(gp.sample_us(n)):
        feet = {leg: foot_sample(spec, leg, u, tips[leg] if tips else None) for leg in lk.ALL_LEGS}
        out.append({'k': k, 'u': u, 't': u * spec.cycle_period_s,
                    'support': tuple(leg for leg in lk.ALL_LEGS if feet[leg].phase == gp.STANCE),
                    'feet': feet})
    return out


def world_velocity(spec, body_velocity):
    """Foot velocity in the world frame: body-frame velocity + the body's (0, v, 0)."""
    vx, vy, vz = body_velocity
    return (vx, vy + spec.body_speed_m_s, vz)


__all__ = ['FootSample', 'foot_offset_and_velocity', 'foot_sample', 'sample_cycle',
           'world_velocity']
