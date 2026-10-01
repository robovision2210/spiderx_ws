"""M4.5 OFFLINE sampled IK of a gait cycle, reusing the M3/M4 kinematics unchanged.

For every sample u_k of one cycle and every leg, the foot target from gait_trajectory is solved
with leg_kinematics.inverse (analytic, position-only, never clamped, URDF limits minus the M1
margin). The previous sample's solution is passed as the IK `reference`, so the solver stays on
one solution branch; sample 0 uses CAD neutral (all zeros), exactly as M3/M4 do.

Outputs (CycleKinematics) are plain per-leg series, and the kinematic checks are:
    ik_feasible          every leg solved at every sample (reason per failing sample)
    fk_residual          FK(IK(target)) reproduces the target within the M3 tolerance
    singularity_margin   min hip/knee singularity margin of the selected solutions
    joint_limit_margin   min distance of any joint to its URDF limit minus the M1 margin
    joint_continuity     max |q_k - q_k-1| (including the wrap from the last sample to the first)
Joint speeds (finite differences, used by gait_metrics) are also computed here.

Offline only: no ROS runtime, no Gazebo, never commands the robot.
"""

from dataclasses import dataclass
import math

from spiderx_controller import gait_trajectory as gt
from spiderx_controller import leg_kinematics as lk
from spiderx_controller.gait_results import Check


@dataclass
class LegSeries:
    leg: str
    joint_names: tuple
    q: list                     # per sample: (hip, thigh, foot) or None if IK failed
    reason: list                # per sample: leg_kinematics.inverse reason ('ok', ...)
    fk_residual_m: list         # per sample (None if IK failed)
    singularity_margin_rad: list
    limit_margin_rad: list      # per sample: min over the 3 joints of the distance to the soft limit


@dataclass
class CycleKinematics:
    gait: str
    n: int
    dt_s: float
    samples: list               # gait_trajectory.sample_cycle output (targets included)
    legs: dict                  # {leg: LegSeries}, ALL_LEGS order

    @property
    def all_solved(self):
        return all(q is not None for s in self.legs.values() for q in s.q)


def _limit_margin(geom, q, margin):
    """Smallest distance (rad) of any joint to [lower + margin, upper - margin]; >= 0 if inside."""
    return min(min(v - (lo + margin), (hi - margin) - v) for v, (lo, hi) in zip(q, geom.limits))


def solve_cycle(spec, geoms, n, margin=lk.DEFAULT_MARGIN_RAD):
    """IK of every leg at every one of n samples. geoms: {leg: LegGeometry} (all four legs)."""
    tips = {leg: geoms[leg].tip0 for leg in lk.ALL_LEGS}
    samples = gt.sample_cycle(spec, n, tips)
    legs = {}
    for leg in lk.ALL_LEGS:
        geom = geoms[leg]
        series = LegSeries(leg, geom.joint_names, [], [], [], [], [])
        reference = None                                   # CAD neutral for sample 0
        for sample in samples:
            target = sample['feet'][leg].target
            r = lk.inverse(geom, target, reference=reference, margin=margin)
            series.reason.append(r['reason'])
            if not r['ok']:
                for lst in (series.q, series.fk_residual_m, series.singularity_margin_rad,
                            series.limit_margin_rad):
                    lst.append(None)
                continue                                   # keep the last good reference
            q = tuple(r['solution'])
            cand = r['candidates'][r['selected_index']]
            series.q.append(q)
            series.fk_residual_m.append(math.dist(lk.forward(geom, q)['tip_position'], target))
            series.singularity_margin_rad.append(min(cand['hip_singularity_margin_rad'],
                                                     cand['knee_singularity_margin_rad']))
            series.limit_margin_rad.append(_limit_margin(geom, q, margin))
            reference = q
        legs[leg] = series
    return CycleKinematics(spec.name, n, spec.cycle_period_s / n, samples, legs)


# ---------------------------------------------------------------- pure series helpers
def joint_steps(q_series):
    """|q_k - q_(k-1)| per joint, cyclic (sample 0 compared with the last). None-safe."""
    n = len(q_series)
    out = []
    for k in range(n):
        a, b = q_series[k - 1], q_series[k]
        out.append(None if a is None or b is None else tuple(abs(y - x) for x, y in zip(a, b)))
    return out


def joint_speeds(q_series, dt):
    """Central-difference joint speeds (rad/s) per sample, cyclic. None where a neighbour failed."""
    n = len(q_series)
    out = []
    for k in range(n):
        a, b = q_series[k - 1], q_series[(k + 1) % n]
        out.append(None if a is None or b is None
                   else tuple(abs(y - x) / (2.0 * dt) for x, y in zip(a, b)))
    return out


# ---------------------------------------------------------------- checks
def _where(kin, k):
    return f'sample {k} (u={kin.samples[k]["u"]:.3f}, t={kin.samples[k]["t"]:.2f} s)'


def check_ik_feasible(kin):
    c = Check('ik_feasible', True, None, None, 'samples', 'exact',
              'every leg has a joint-safe IK solution at every sample (URDF limits minus margin)')
    failed = 0
    for leg, s in kin.legs.items():
        for k, reason in enumerate(s.reason):
            if reason != 'ok':
                failed += 1
                c.add_failure(f'{leg} {_where(kin, k)}: {reason}')
    c.passed, c.value, c.threshold = failed == 0, failed, 0
    return c


def check_fk_residual(kin, tol):
    vals = [v for s in kin.legs.values() for v in s.fk_residual_m if v is not None]
    worst = max(vals) if vals else None
    c = Check('fk_residual', worst is not None and worst <= tol, worst, tol, 'm', 'exact',
              'FK(IK(target)) reproduces every solved target')
    for leg, s in kin.legs.items():
        for k, v in enumerate(s.fk_residual_m):
            if v is not None and v > tol:
                c.add_failure(f'{leg} {_where(kin, k)}: residual {v:.2e} m')
    if worst is None:
        c.add_failure('no sample was solved')
    return c


def check_min_series(kin, attr, name, threshold, unit, description):
    """Generic 'every solved sample has value >= threshold' check over a LegSeries attribute."""
    vals = [v for s in kin.legs.values() for v in getattr(s, attr) if v is not None]
    worst = min(vals) if vals else None
    c = Check(name, worst is not None and worst >= threshold, worst, threshold, unit, 'exact',
              description)
    for leg, s in kin.legs.items():
        for k, v in enumerate(getattr(s, attr)):
            if v is not None and v < threshold:
                c.add_failure(f'{leg} {_where(kin, k)}: {v:.4f} {unit}')
    if worst is None:
        c.add_failure('no sample was solved')
    return c


def check_joint_continuity(kin, max_step):
    worst, c = 0.0, Check('joint_continuity', True, None, max_step, 'rad', 'sampled',
                          'largest joint change between consecutive samples (cyclic); a large '
                          'jump means an IK branch flip or an unsampled excursion')
    any_value = False
    for leg, s in kin.legs.items():
        for k, step in enumerate(joint_steps(s.q)):
            if step is None:
                continue
            any_value = True
            m = max(step)
            worst = max(worst, m)
            if m > max_step:
                j = step.index(m)
                c.add_failure(f'{leg} {_where(kin, k)}: {s.joint_names[j]} jumps {m:.4f} rad')
    c.value = worst if any_value else None
    c.passed = any_value and c.failure_count == 0
    return c


def kinematic_checks(kin, cfg):
    """The Batch C checks of one gait, in report order."""
    a = cfg.analysis
    return [
        check_ik_feasible(kin),
        check_fk_residual(kin, cfg.ik_position_tol_m),
        check_min_series(kin, 'singularity_margin_rad', 'singularity_margin',
                         a.min_singularity_margin_rad, 'rad',
                         'min hip/knee singularity margin of every selected IK solution'),
        check_min_series(kin, 'limit_margin_rad', 'joint_limit_margin', 0.0, 'rad',
                         'min distance of any joint to its URDF limit minus the M1 margin '
                         '(>= 0 by construction for solved samples; reported as a workspace margin)'),
        check_joint_continuity(kin, a.max_joint_step_rad),
    ]


__all__ = ['LegSeries', 'CycleKinematics', 'solve_cycle', 'joint_steps', 'joint_speeds',
           'kinematic_checks', 'check_ik_feasible', 'check_fk_residual', 'check_min_series',
           'check_joint_continuity']
