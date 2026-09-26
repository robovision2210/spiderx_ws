"""Load, validate and evaluate the M3 SIMULATION-ONLY single-leg kinematics target config.

config/m3_kinematics_targets.yaml is the single source of targets, tolerances and safety bounds.
Targets are offsets from the CAD-neutral foot tip computed by leg_kinematics (never hard-coded).
Pure Python (no ROS graph), so it is unit-tested.
"""

import math
import os

from spiderx_controller import leg_kinematics as lk
from spiderx_controller.posture_config import PostureConfigError, load_yaml_strict

CONFIG_FILE = 'm3_kinematics_targets.yaml'
TOLERANCE_KEYS = ('ik_position_tol_m', 'fk_vs_tf_position_m', 'fk_vs_tf_orientation_rad',
                  'fk_vs_gazebo_position_m', 'fk_vs_gazebo_orientation_rad',
                  'target_reached_position_m')
SAFETY_KEYS = ('min_singularity_margin_rad', 'min_lift_m', 'max_joint_change_rad',
               'max_offset_m')
MOTION_KEYS = ('min_duration_s', 'speed_factor', 'settle_s')
EXPECT = {'unreachable': ('unreachable_hip', 'unreachable_knee', 'singular'),
          'joint_limits': ('joint_limits',)}


class TargetConfigError(PostureConfigError):
    """The M3 target configuration is invalid; nothing may be commanded."""


def _num(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise TargetConfigError(f'{what} must be a finite number, got {value!r}')
    return float(value)


def _vec3(value, what):
    if not isinstance(value, list) or len(value) != 3:
        raise TargetConfigError(f'{what} must be a list of 3 numbers, got {value!r}')
    return tuple(_num(v, f'{what}[{i}]') for i, v in enumerate(value))


def default_config_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('spiderx_controller'), 'config', CONFIG_FILE)


def validate_config(cfg, legs_margin, geom_leg='front_left'):
    """Structural validation of a parsed config. Returns a normalised dict."""
    if cfg.get('simulation_only') is not True:
        raise TargetConfigError('simulation_only: true is required')
    for key in ('config_version', 'date'):
        if key not in cfg:
            raise TargetConfigError(f'missing top-level "{key}"')
    try:
        leg = lk.resolve_leg(cfg.get('leg'))
    except lk.KinematicsError as e:
        raise TargetConfigError(str(e)) from e
    if leg != geom_leg:
        raise TargetConfigError(f'config leg {leg} differs from geometry leg {geom_leg}')
    try:
        lk.check_frame(cfg.get('frame'))
    except lk.KinematicsError as e:
        raise TargetConfigError(str(e)) from e
    ref = cfg.get('reference') or {}
    if ref.get('pose') != 'cad_neutral' or ref.get('point') != 'derived_foot_tip':
        raise TargetConfigError('reference must be pose cad_neutral, point derived_foot_tip')
    margin = _num(cfg.get('validation_margin_rad'), 'validation_margin_rad')
    if abs(margin - legs_margin) > 1e-12:
        raise TargetConfigError(f'validation_margin_rad {margin} must equal the M1 margin '
                                f'{legs_margin}')
    out = {'config_version': cfg['config_version'], 'date': str(cfg['date']), 'leg': leg,
           'frame': lk.FRAME, 'margin': margin}
    for section, keys in (('tolerances', TOLERANCE_KEYS), ('safety', SAFETY_KEYS),
                          ('motion', MOTION_KEYS)):
        block = cfg.get(section)
        if not isinstance(block, dict):
            raise TargetConfigError(f'"{section}" mapping is required')
        missing, unknown = set(keys) - set(block), set(block) - set(keys)
        if missing or unknown:
            raise TargetConfigError(f'{section}: missing {sorted(missing)}, unknown '
                                    f'{sorted(unknown)}')
        out[section] = {k: _num(block[k], f'{section}.{k}') for k in keys}
        for k, v in out[section].items():
            if v <= 0:
                raise TargetConfigError(f'{section}.{k} must be > 0, got {v}')
    if abs(out['tolerances']['ik_position_tol_m'] - lk.IK_POSITION_TOL_M) > 1e-15:
        raise TargetConfigError('tolerances.ik_position_tol_m must equal '
                                f'leg_kinematics.IK_POSITION_TOL_M ({lk.IK_POSITION_TOL_M})')
    if out['motion']['min_duration_s'] < 3.0:
        raise TargetConfigError('motion.min_duration_s must be >= 3.0 s')

    safe = cfg.get('safe_targets')
    if not isinstance(safe, dict) or not safe:
        raise TargetConfigError('"safe_targets" must be a non-empty mapping')
    out['safe_targets'] = {}
    for name, t in safe.items():
        if not isinstance(t, dict) or t.get('negative_test_only'):
            raise TargetConfigError(f'safe target {name} must be a mapping and not negative-only')
        if not isinstance(t.get('rationale'), str) or not t['rationale'].strip():
            raise TargetConfigError(f'safe target {name} needs a rationale')
        off = _vec3(t.get('offset_m'), f'safe_targets.{name}.offset_m')
        if off[2] < out['safety']['min_lift_m']:
            raise TargetConfigError(f'safe target {name} lifts {off[2]} m < min_lift_m '
                                    f'{out["safety"]["min_lift_m"]} (ground contact risk)')
        if max(abs(c) for c in off) > out['safety']['max_offset_m']:
            raise TargetConfigError(f'safe target {name} offset exceeds max_offset_m')
        out['safe_targets'][name] = {'offset_m': off, 'rationale': t['rationale']}

    neg = cfg.get('negative_tests')
    if not isinstance(neg, dict) or not neg:
        raise TargetConfigError('"negative_tests" must be a non-empty mapping')
    out['negative_tests'] = {}
    for name, t in neg.items():
        if not isinstance(t, dict) or t.get('negative_test_only') is not True:
            raise TargetConfigError(f'negative test {name} must set negative_test_only: true')
        if t.get('expect') not in EXPECT:
            raise TargetConfigError(f'negative test {name}: expect must be one of {sorted(EXPECT)}')
        has_off, has_q = 'offset_m' in t, 'from_joint_config_rad' in t
        if has_off == has_q:
            raise TargetConfigError(f'negative test {name}: give exactly one of offset_m or '
                                    'from_joint_config_rad')
        entry = {'expect': t['expect'], 'rationale': t.get('rationale', '')}
        if has_off:
            entry['offset_m'] = _vec3(t['offset_m'], f'negative_tests.{name}.offset_m')
        else:
            entry['from_joint_config_rad'] = _vec3(t['from_joint_config_rad'],
                                                   f'negative_tests.{name}.from_joint_config_rad')
        out['negative_tests'][name] = entry
    return out


def target_point(geom, entry):
    """Absolute base_link target of a (validated) safe or negative entry."""
    if 'offset_m' in entry:
        return tuple(r + o for r, o in zip(geom.tip0, entry['offset_m']))
    return lk.forward(geom, entry['from_joint_config_rad'])['tip_position']


def evaluate(geom, cfg):
    """Solve every target and apply the safety bounds. Returns (errors, plan).

    plan: {'reference_tip_m', 'safe': {name: {...ik...}}, 'negative': {name: {...}}}.
    """
    errors, plan = [], {'reference_tip_m': list(geom.tip0), 'safe': {}, 'negative': {}}
    s = cfg['safety']
    for name, entry in cfg['safe_targets'].items():
        target = target_point(geom, entry)
        r = lk.inverse(geom, target, margin=cfg['margin'])
        rec = {'offset_m': list(entry['offset_m']), 'target_m': list(target), 'ik': r}
        plan['safe'][name] = rec
        if not r['ok']:
            errors.append(f'safe target {name}: IK failed ({r["reason"]}: {r.get("message")})')
            continue
        c = r['candidates'][r['selected_index']]
        if min(c['hip_singularity_margin_rad'], c['knee_singularity_margin_rad']) < \
                s['min_singularity_margin_rad']:
            errors.append(f'safe target {name}: too close to a singularity')
        dq = max(abs(q) for q in r['solution'])     # CAD neutral is all zeros
        rec['max_joint_change_rad'] = dq
        if dq > s['max_joint_change_rad']:
            errors.append(f'safe target {name}: joint change {dq:.3f} rad > '
                          f'{s["max_joint_change_rad"]}')
    for name, entry in cfg['negative_tests'].items():
        target = target_point(geom, entry)
        r = lk.inverse(geom, target, margin=cfg['margin'])
        plan['negative'][name] = {'target_m': list(target), 'expect': entry['expect'], 'ik': r}
        if r['ok']:
            errors.append(f'negative test {name}: IK unexpectedly succeeded')
        elif r['reason'] not in EXPECT[entry['expect']]:
            errors.append(f'negative test {name}: rejected as {r["reason"]}, expected '
                          f'{entry["expect"]}')
    return errors, plan


def load_and_evaluate(config_path=None, config_dir=None, urdf_root=None):
    """Installed (or given) config + URDF geometry. Returns (cfg, geom, errors, plan).

    Raises TargetConfigError for an invalid config (nothing may be commanded).
    """
    # spiderx_legs.yaml always comes from config_dir (default: the INSTALLED config directory),
    # never from next to a custom --config file.
    config_dir = config_dir or os.path.dirname(default_config_path())
    config_path = config_path or os.path.join(config_dir, CONFIG_FILE)
    try:
        raw = load_yaml_strict(config_path)
        legs = load_yaml_strict(os.path.join(config_dir, 'spiderx_legs.yaml'))
    except PostureConfigError as e:          # malformed / duplicate-key YAML
        raise TargetConfigError(str(e)) from e
    try:
        leg = lk.resolve_leg(raw.get('leg'))
    except lk.KinematicsError as e:
        raise TargetConfigError(str(e)) from e
    try:
        geom = lk.load_geometry(leg, urdf_root=urdf_root, legs_cfg=legs)
    except lk.KinematicsError as e:
        raise TargetConfigError(f'geometry: {e}') from e
    cfg = validate_config(raw, float(legs['soft_limit_margin_rad']), geom.leg)
    errors, plan = evaluate(geom, cfg)
    return cfg, geom, errors, plan
