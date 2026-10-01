"""M4.5 OFFLINE phase engine: which legs swing or stand at any normalised cycle time.

A gait cycle has period T. Normalised time u = t / T lies in [0, 1). Leg i, with phase offset
phi_i and duty factor beta (fraction of the cycle in stance), has the local phase

    theta_i(u) = (u - phi_i) mod 1          (in [0, 1))

and is in SWING while theta_i < 1 - beta (lift-off at theta = 0), in STANCE otherwise (touch-down
at theta = 1 - beta). `progress` s in [0, 1) says how far through the current swing or stance the
leg is; the trajectory module turns (phase, s) into a foot position.

Pure functions of a GaitSpec (gait_config.py): no geometry, no ROS, no robot command.
"""

from dataclasses import dataclass

from spiderx_controller import leg_kinematics as lk

SWING, STANCE = 'swing', 'stance'
# Floating-point round-off tolerance at the lift-off / touch-down boundaries. Far below any sample
# spacing (1/n, n <= 4000) and below the 1e-9 offsets used to probe transitions, so it only moves
# values that lie on a boundary mathematically onto that boundary.
PHASE_SNAP = 1e-12


@dataclass(frozen=True)
class LegPhase:
    phase: str          # SWING or STANCE
    progress: float     # s in [0, 1) through the current swing or stance
    theta: float        # local phase (u - phi) mod 1


def local_phase(spec, leg, u):
    """theta = (u - phase_offset) mod 1, in [0, 1)."""
    return (u - spec.phase_offset(leg)) % 1.0


def leg_phase(spec, leg, u):
    """SWING/STANCE of one leg at normalised time u, with the progress s through that phase."""
    theta = local_phase(spec, leg, u)
    swing_fraction = 1.0 - spec.duty_factor
    # Snap round-off onto the boundaries so that the half-open definition decides, not the last bit
    # of (u - phi) mod 1 or of 1 - beta: lift-off (theta = 0, also reached as 1 - tiny) is swing,
    # touch-down (theta = 1 - beta) is stance. Found by the M5 Stage 0/1 gate: for wave
    # (beta 0.85) 1 - 0.85 = 0.15000000000000002 while (0.15 - 0) mod 1 = 0.15, which labelled the
    # rear-left touch-down sample as swing at every sample count.
    if theta >= 1.0 - PHASE_SNAP:
        theta = 0.0
    if abs(theta - swing_fraction) <= PHASE_SNAP:
        theta = swing_fraction
    if theta < swing_fraction:
        return LegPhase(SWING, theta / swing_fraction, theta)
    return LegPhase(STANCE, (theta - swing_fraction) / spec.duty_factor, theta)


def support_set(spec, u):
    """The legs in stance at u (ALL_LEGS order)."""
    return tuple(leg for leg in lk.ALL_LEGS if leg_phase(spec, leg, u).phase == STANCE)


def sample_us(n):
    """n uniform, deterministic samples u_k = k / n of one cycle (u = 1 is u = 0 again)."""
    if isinstance(n, bool) or not isinstance(n, int) or n < 1:
        raise ValueError(f'number of samples must be a positive integer, got {n!r}')
    return [k / n for k in range(n)]


def support_summary(spec, n):
    """How many feet support the body, over n samples of one cycle.

    Returns {'min_support': int, 'max_support': int,
             'fraction_by_count': {count: fraction of samples}, 'swing_fraction': {leg: fraction}}.
    """
    us = sample_us(n)
    counts = [len(support_set(spec, u)) for u in us]
    fraction = {c: counts.count(c) / n for c in sorted(set(counts))}
    swing = {leg: sum(leg_phase(spec, leg, u).phase == SWING for u in us) / n
             for leg in lk.ALL_LEGS}
    return {'min_support': min(counts), 'max_support': max(counts),
            'fraction_by_count': fraction, 'swing_fraction': swing}


__all__ = ['SWING', 'STANCE', 'PHASE_SNAP', 'LegPhase', 'local_phase', 'leg_phase', 'support_set', 'sample_us',
           'support_summary']
