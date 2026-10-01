"""M4.5 OFFLINE gait configurations: strict schema, loader and normalised (immutable) result.

config/m4_5_gaits.yaml is the single source of every gait parameter. This module only checks and
normalises the YAML; it holds no robot geometry (that always comes from the URDF through
leg_kinematics) and computes no trajectory. Data flow of the M4.5 framework:

    m4_5_gaits.yaml --gait_config--> GaitConfig --gait_phase/gait_trajectory--> foot targets(t)
        --gait_kinematics--> sampled IK --gait_metrics--> verdict + metrics --gait_report--> files

Conventions (docs/M4_5_PLAN.md section 5.1):
* frame base_link (NOT REP-103: +x right, +y front, +z up); the body travels along +y;
* phase: normalised cycle time u in [0, 1); leg i swings while (u - phase_offset_i) mod 1 < 1 - beta,
  beta = duty_factor (fraction of the cycle in stance);
* step_length_m = L = foot travel RELATIVE TO THE BODY during stance (the stance stroke). Derived:
  body speed v = L / (beta T), stride length S = v T = L / beta;
* give exactly one of body_speed_m_s or cycle_period_s per gait (the other is derived);
* stance_height_offset_m: +z offset of every foot from its CAD-neutral tip (M4 convention: +z
  LOWERS the body).

OFFLINE analysis only: nothing here imports a ROS runtime, starts Gazebo or commands a robot.
"""

from dataclasses import dataclass
import os
import re

from spiderx_controller import leg_kinematics as lk
from spiderx_controller.kinematics_targets import _num, TargetConfigError
from spiderx_controller.posture_config import load_yaml_strict, PostureConfigError

CONFIG_FILE = 'm4_5_gaits.yaml'
NAME_RE = re.compile(r'^[a-z][a-z0-9_]*$')      # gait names become file and directory names
SWING_PROFILES = ('cycloid',)
ANALYSIS_KEYS = ('samples_per_cycle', 'min_static_margin_m', 'min_singularity_margin_rad',
                 'max_joint_step_rad', 'joint_speed_reference_rad_s', 'velocity_jump_tol_m_s')
GAIT_KEYS = ('description', 'duty_factor', 'phase_offsets', 'swing_order', 'step_length_m',
             'step_height_m', 'stance_height_offset_m', 'stance_center_offset_m',
             'body_speed_m_s', 'cycle_period_s', 'swing_profile', 'requires_static_stability')
DEFAULTABLE_KEYS = ('step_length_m', 'step_height_m', 'stance_height_offset_m',
                    'stance_center_offset_m', 'body_speed_m_s', 'cycle_period_s', 'swing_profile')
PHASE_TOL = 1e-12


class GaitConfigError(TargetConfigError):
    """The M4.5 gait configuration is invalid; it is not evaluated."""


@dataclass(frozen=True)
class AnalysisSettings:
    """How densely and against which thresholds every gait is evaluated (all from the YAML)."""

    samples_per_cycle: int
    min_static_margin_m: float
    min_singularity_margin_rad: float
    max_joint_step_rad: float
    joint_speed_reference_rad_s: float      # = spiderx_legs.yaml SIMULATION_PLACEHOLDER (checked)
    velocity_jump_tol_m_s: float


@dataclass(frozen=True)
class GaitSpec:
    """One normalised gait. Legs are always the four of leg_kinematics.ALL_LEGS, in that order."""

    name: str
    description: str
    duty_factor: float
    phase_offsets: tuple            # (offset per leg, ALL_LEGS order), each in [0, 1)
    swing_order: tuple              # groups of legs that swing together, in phase order
    step_length_m: float            # stance stroke L (relative to the body)
    step_height_m: float            # swing lift h
    stance_height_offset_m: float   # +z of every foot from its neutral tip (+z lowers the body)
    stance_center_offset_m: tuple   # (x, y) of every leg's stance centre from its neutral tip
    cycle_period_s: float           # T
    body_speed_m_s: float           # v = L / (beta T)
    swing_profile: str
    requires_static_stability: bool

    @property
    def stride_length_m(self):
        """Body travel per cycle, S = v T = L / beta."""
        return self.body_speed_m_s * self.cycle_period_s

    def phase_offset(self, leg):
        return self.phase_offsets[lk.ALL_LEGS.index(lk.resolve_leg(leg, allow_all_legs=True))]


@dataclass(frozen=True)
class GaitConfig:
    config_version: int
    date: str
    frame: str
    margin_rad: float
    ik_position_tol_m: float
    analysis: AnalysisSettings
    gaits: tuple                    # GaitSpec, in YAML order

    def gait(self, name):
        for g in self.gaits:
            if g.name == name:
                return g
        raise GaitConfigError(f'unknown gait "{name}"; available: {[g.name for g in self.gaits]}')


def default_config_dir():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('spiderx_controller'), 'config')


# ---------------------------------------------------------------- small strict helpers
def _positive(value, what, allow_zero=False):
    v = _num(value, what)
    if v < 0 or (v == 0 and not allow_zero):
        raise GaitConfigError(f'{what} must be {">= 0" if allow_zero else "> 0"}, got {v}')
    return v


def _int_at_least(value, what, lowest):
    if isinstance(value, bool) or not isinstance(value, int) or value < lowest:
        raise GaitConfigError(f'{what} must be an integer >= {lowest}, got {value!r}')
    return value


def _leg(key, what):
    try:
        return lk.resolve_leg(key, allow_all_legs=True)
    except lk.KinematicsError as e:
        raise GaitConfigError(f'{what}: {e}') from e


def _exact_keys(mapping, allowed, what, required=()):
    if not isinstance(mapping, dict):
        raise GaitConfigError(f'{what} must be a mapping')
    unknown = sorted(set(mapping) - set(allowed))
    missing = sorted(set(required) - set(mapping))
    if unknown or missing:
        raise GaitConfigError(f'{what}: unknown keys {unknown}, missing keys {missing}')


# ---------------------------------------------------------------- per-gait validation
def _phase_offsets(value, what):
    if not isinstance(value, dict):
        raise GaitConfigError(f'{what} must map each of the four legs to a phase in [0, 1)')
    out = {}
    for key, v in value.items():
        leg = _leg(key, what)
        if leg in out:
            raise GaitConfigError(f'{what}: leg {leg} given twice (aliases)')
        phase = _num(v, f'{what}.{leg}')
        if not 0.0 <= phase < 1.0:
            raise GaitConfigError(f'{what}.{leg} must be in [0, 1), got {phase}')
        out[leg] = phase
    missing = [leg for leg in lk.ALL_LEGS if leg not in out]
    if missing:
        raise GaitConfigError(f'{what}: missing legs {missing}')
    return tuple(out[leg] for leg in lk.ALL_LEGS)


def _swing_order(value, offsets, what):
    """Groups of legs in increasing phase-offset order; legs in one group share one offset.

    A group is a leg name or a list of leg names (e.g. a trot's diagonal pair). The declared order
    must equal the order implied by the phase offsets, so the two can never silently disagree.
    """
    if not isinstance(value, list) or not value:
        raise GaitConfigError(f'{what} must be a non-empty list of legs or leg groups')
    groups, seen = [], []
    for i, item in enumerate(value):
        members = item if isinstance(item, list) else [item]
        if not members:
            raise GaitConfigError(f'{what}[{i}] is an empty group')
        group = []
        for key in members:
            leg = _leg(key, f'{what}[{i}]')
            if leg in seen:
                raise GaitConfigError(f'{what}: leg {leg} listed twice')
            seen.append(leg)
            group.append(leg)
        groups.append(tuple(group))
    missing = [leg for leg in lk.ALL_LEGS if leg not in seen]
    if missing:
        raise GaitConfigError(f'{what}: missing legs {missing}')
    phase = {leg: offsets[lk.ALL_LEGS.index(leg)] for leg in lk.ALL_LEGS}
    group_phases = []
    for i, group in enumerate(groups):
        ps = {phase[leg] for leg in group}
        if max(ps) - min(ps) > PHASE_TOL:
            raise GaitConfigError(f'{what}[{i}] {list(group)}: legs of one group must share one '
                                  f'phase offset, got { {leg: phase[leg] for leg in group} }')
        group_phases.append(min(ps))
    for i in range(1, len(group_phases)):
        if group_phases[i] <= group_phases[i - 1] + PHASE_TOL:
            raise GaitConfigError(
                f'{what} disagrees with phase_offsets: group {list(groups[i])} (phase '
                f'{group_phases[i]}) must come strictly after {list(groups[i - 1])} (phase '
                f'{group_phases[i - 1]}); legs sharing a phase belong in one group')
    return tuple(groups)


def _gait(name, raw, defaults):
    what = f'gait {name}'
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise GaitConfigError(f'gait name {name!r} must match {NAME_RE.pattern} (it becomes a file '
                              f'name)')
    _exact_keys(raw, GAIT_KEYS, what, required=('duty_factor', 'phase_offsets', 'swing_order',
                                                'requires_static_stability'))
    p = {**defaults, **raw}
    for key in ('step_length_m', 'step_height_m', 'stance_height_offset_m',
                'stance_center_offset_m', 'swing_profile'):
        if key not in p:
            raise GaitConfigError(f'{what}: "{key}" missing (in the gait or in defaults)')
    beta = _num(p['duty_factor'], f'{what}.duty_factor')
    if not 0.0 < beta < 1.0:
        raise GaitConfigError(f'{what}.duty_factor must be in (0, 1), got {beta}')
    offsets = _phase_offsets(p['phase_offsets'], f'{what}.phase_offsets')
    order = _swing_order(p['swing_order'], offsets, f'{what}.swing_order')
    L = _positive(p['step_length_m'], f'{what}.step_length_m')
    h = _positive(p['step_height_m'], f'{what}.step_height_m')
    dz = _num(p['stance_height_offset_m'], f'{what}.stance_height_offset_m')
    c = p['stance_center_offset_m']
    if not isinstance(c, list) or len(c) != 2:
        raise GaitConfigError(f'{what}.stance_center_offset_m must be [x, y], got {c!r}')
    center = tuple(_num(v, f'{what}.stance_center_offset_m[{i}]') for i, v in enumerate(c))
    # exactly one timing source; a gait-level value replaces the default of the other kind
    speed_raw = raw.get('body_speed_m_s', None if 'cycle_period_s' in raw
                        else defaults.get('body_speed_m_s'))
    period_raw = raw.get('cycle_period_s', None if 'body_speed_m_s' in raw
                         else defaults.get('cycle_period_s'))
    if (speed_raw is None) == (period_raw is None):
        raise GaitConfigError(f'{what}: give exactly one of body_speed_m_s or cycle_period_s')
    if speed_raw is not None:
        v = _positive(speed_raw, f'{what}.body_speed_m_s')
        T = L / (beta * v)
    else:
        T = _positive(period_raw, f'{what}.cycle_period_s')
        v = L / (beta * T)
    if p['swing_profile'] not in SWING_PROFILES:
        raise GaitConfigError(f'{what}.swing_profile must be one of {list(SWING_PROFILES)}, got '
                              f'{p["swing_profile"]!r}')
    if not isinstance(p['requires_static_stability'], bool):
        raise GaitConfigError(f'{what}.requires_static_stability must be true or false')
    return GaitSpec(name=name, description=str(p.get('description', '')), duty_factor=beta,
                    phase_offsets=offsets, swing_order=order, step_length_m=L, step_height_m=h,
                    stance_height_offset_m=dz, stance_center_offset_m=center, cycle_period_s=T,
                    body_speed_m_s=v, swing_profile=p['swing_profile'],
                    requires_static_stability=p['requires_static_stability'])


# ---------------------------------------------------------------- whole file
def validate_config(cfg, legs_cfg):
    """Validate a parsed m4_5_gaits.yaml against spiderx_legs.yaml. Returns a GaitConfig."""
    if not isinstance(cfg, dict):
        raise GaitConfigError('the gait configuration must be a mapping')
    if cfg.get('simulation_only') is not True or cfg.get('offline_only') is not True:
        raise GaitConfigError('simulation_only: true and offline_only: true are both required '
                              '(M4.5 gaits are offline analysis only)')
    for key in ('config_version', 'date'):
        if key not in cfg:
            raise GaitConfigError(f'missing top-level "{key}"')
    version = cfg['config_version']
    if isinstance(version, bool) or not isinstance(version, int) or version < 1:
        raise GaitConfigError(f'config_version must be a positive integer, got {version!r}')
    if cfg.get('frame') != lk.FRAME:
        raise GaitConfigError(f'frame must be "{lk.FRAME}", got {cfg.get("frame")!r}')
    if cfg.get('units') != {'length': 'm', 'angle': 'rad', 'time': 's'}:
        raise GaitConfigError(f'units must be {{length: m, angle: rad, time: s}}, got '
                              f'{cfg.get("units")!r}')
    if cfg.get('direction_of_travel') != '+y':
        raise GaitConfigError('direction_of_travel must be "+y" (base_link front)')
    ref = cfg.get('reference') or {}
    if ref != {'pose': 'cad_neutral', 'point': 'derived_foot_tip'}:
        raise GaitConfigError('reference must be {pose: cad_neutral, point: derived_foot_tip}')
    margin = _num(cfg.get('validation_margin_rad'), 'validation_margin_rad')
    legs_margin = _num(legs_cfg.get('soft_limit_margin_rad'), 'spiderx_legs soft_limit_margin_rad')
    if abs(margin - legs_margin) > 1e-12:
        raise GaitConfigError(f'validation_margin_rad {margin} must equal the M1 margin '
                              f'{legs_margin} (spiderx_legs.yaml)')
    tol = _num(cfg.get('ik_position_tol_m'), 'ik_position_tol_m')
    if abs(tol - lk.IK_POSITION_TOL_M) > 1e-15:
        raise GaitConfigError(f'ik_position_tol_m must equal leg_kinematics.IK_POSITION_TOL_M '
                              f'({lk.IK_POSITION_TOL_M})')

    a = cfg.get('analysis')
    _exact_keys(a, ANALYSIS_KEYS, 'analysis', required=ANALYSIS_KEYS)
    placeholder = (legs_cfg.get('motion_constraints') or {}).get('max_joint_velocity_rad_s')
    speed_ref = _positive(a['joint_speed_reference_rad_s'], 'analysis.joint_speed_reference_rad_s')
    if placeholder is None or abs(speed_ref - _num(placeholder, 'max_joint_velocity_rad_s')) > 1e-12:
        raise GaitConfigError('analysis.joint_speed_reference_rad_s must equal spiderx_legs.yaml '
                              f'motion_constraints.max_joint_velocity_rad_s ({placeholder})')
    analysis = AnalysisSettings(
        samples_per_cycle=_int_at_least(a['samples_per_cycle'], 'analysis.samples_per_cycle', 8),
        min_static_margin_m=_positive(a['min_static_margin_m'], 'analysis.min_static_margin_m',
                                      allow_zero=True),
        min_singularity_margin_rad=_positive(a['min_singularity_margin_rad'],
                                             'analysis.min_singularity_margin_rad'),
        max_joint_step_rad=_positive(a['max_joint_step_rad'], 'analysis.max_joint_step_rad'),
        joint_speed_reference_rad_s=speed_ref,
        velocity_jump_tol_m_s=_positive(a['velocity_jump_tol_m_s'],
                                        'analysis.velocity_jump_tol_m_s'))

    defaults = cfg.get('defaults', {})
    _exact_keys(defaults, DEFAULTABLE_KEYS, 'defaults')
    if 'body_speed_m_s' in defaults and 'cycle_period_s' in defaults:
        raise GaitConfigError('defaults: give at most one of body_speed_m_s or cycle_period_s')
    gaits = cfg.get('gaits')
    if not isinstance(gaits, dict) or not gaits:
        raise GaitConfigError('"gaits" must be a non-empty mapping')
    specs = tuple(_gait(name, raw, defaults) for name, raw in gaits.items())
    return GaitConfig(config_version=version, date=str(cfg['date']), frame=lk.FRAME,
                      margin_rad=margin, ik_position_tol_m=tol, analysis=analysis, gaits=specs)


def load_gait_config(config_path=None, config_dir=None):
    """Read and validate the gait YAML (default: the installed config/m4_5_gaits.yaml).

    spiderx_legs.yaml always comes from config_dir (default: the installed config directory),
    never from next to a custom gait file. Raises GaitConfigError for any invalid input.
    """
    config_dir = config_dir or default_config_dir()
    config_path = config_path or os.path.join(config_dir, CONFIG_FILE)
    try:
        raw = load_yaml_strict(config_path)
        legs = load_yaml_strict(os.path.join(config_dir, 'spiderx_legs.yaml'))
    except PostureConfigError as e:            # unreadable, malformed or duplicate-key YAML
        raise GaitConfigError(str(e)) from e
    try:
        return validate_config(raw, legs)
    except GaitConfigError:
        raise
    except TargetConfigError as e:             # the reused M3 number helper raises the M3 class
        raise GaitConfigError(str(e)) from e


def describe(spec):
    """One-line human summary (used by the CLI and the reports)."""
    return (f'{spec.name}: beta={spec.duty_factor:g}, T={spec.cycle_period_s:.3f} s, '
            f'L={spec.step_length_m * 1000:.1f} mm, h={spec.step_height_m * 1000:.1f} mm, '
            f'v={spec.body_speed_m_s * 1000:.2f} mm/s, stride={spec.stride_length_m * 1000:.1f} mm, '
            f'swing order {[list(g) for g in spec.swing_order]}')


__all__ = ['GaitConfigError', 'GaitConfig', 'GaitSpec', 'AnalysisSettings', 'load_gait_config',
           'validate_config', 'describe', 'CONFIG_FILE']
