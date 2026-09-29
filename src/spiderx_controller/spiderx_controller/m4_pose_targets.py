"""Load, validate and evaluate the M4 SIMULATION-ONLY four-leg static foot-target poses.

config/m4_pose_targets.yaml is the single source of the poses. For each pose this module:
  1. resolves all four legs (M4 opt-in all-leg geometry from the expanded URDF);
  2. turns per-leg offsets into absolute base_link foot-tip targets (offset + CAD-neutral tip);
  3. solves IK for all four legs (leg_kinematics.inverse_all - analytic, never clamped);
  4. applies the safety bounds and the M1 12-joint limit rule (joint_safety.check_pose);
  5. checks only NECESSARY geometric support conditions (see below).
A pose yields a 12-joint command ONLY if every leg and every check passes (atomic, no partials).

Support/contact: the YAML declares the INTENDED support legs. This module checks that the support
targets are coplanar (so a level body is geometrically possible), that there are >= 3 of them and
that any other foot is lifted above that plane. It does NOT prove ground contact, support, balance
or centre-of-mass stability; the body height it reports is a labelled geometric expectation that
M4 Phase 4 must verify in Gazebo.

Pure Python: no ROS graph, no Gazebo, never commands the robot.
"""

import math
import os

from spiderx_controller import leg_kinematics as lk
from spiderx_controller.joint_safety import UnsafeCommandError, check_pose, load_limits
from spiderx_controller.kinematics_targets import TargetConfigError, _num, _vec3
from spiderx_controller.posture_config import PostureConfigError, load_yaml_strict

CONFIG_FILE = 'm4_pose_targets.yaml'
SAFETY_KEYS = ('max_offset_m', 'min_singularity_margin_rad', 'max_joint_change_rad',
               'min_lift_above_support_m', 'support_plane_tol_m')
EXPECT_REASONS = {'unreachable': ('unreachable_hip', 'unreachable_knee', 'singular'),
                  'joint_limits': ('joint_limits',)}
GEOMETRIC_NOTE = ('geometric expectation only (= -support-plane z, assuming the support feet rest '
                  'on flat ground and the body stays level); NOT a measured or verified height')


class PoseTargetError(TargetConfigError):
    """The M4 pose configuration is invalid; nothing may be commanded."""


def default_config_dir():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('spiderx_controller'), 'config')


def _legs_mapping(value, what):
    """{canonical leg: value} for EXACTLY the four legs (aliases accepted, duplicates refused)."""
    if not isinstance(value, dict):
        raise PoseTargetError(f'{what} must be a mapping of the four legs {list(lk.ALL_LEGS)}')
    out = {}
    for key, v in value.items():
        try:
            leg = lk.resolve_leg(key, allow_all_legs=True)
        except lk.KinematicsError as e:
            raise PoseTargetError(f'{what}: {e}') from e
        if leg in out:
            raise PoseTargetError(f'{what}: leg {leg} given twice (aliases)')
        out[leg] = v
    missing = [leg for leg in lk.ALL_LEGS if leg not in out]
    if missing:
        raise PoseTargetError(f'{what}: missing legs {missing} (every pose must give all four)')
    return {leg: out[leg] for leg in lk.ALL_LEGS}


def _support_legs(value, what):
    if not isinstance(value, list) or not value:
        raise PoseTargetError(f'{what} must be a non-empty list of legs')
    legs = []
    for key in value:
        try:
            leg = lk.resolve_leg(key, allow_all_legs=True)
        except lk.KinematicsError as e:
            raise PoseTargetError(f'{what}: {e}') from e
        if leg in legs:
            raise PoseTargetError(f'{what}: leg {leg} listed twice')
        legs.append(leg)
    if len(legs) < 3:
        raise PoseTargetError(f'{what}: a static pose needs >= 3 support legs, got {legs}')
    return legs


def validate_config(cfg, legs_margin):
    """Structural validation of a parsed config. Returns a normalised dict (raises PoseTargetError)."""
    if cfg.get('simulation_only') is not True:
        raise PoseTargetError('simulation_only: true is required (M4 poses are simulation-only)')
    for key in ('config_version', 'date'):
        if key not in cfg:
            raise PoseTargetError(f'missing top-level "{key}"')
    if cfg.get('frame') != lk.FRAME:
        raise PoseTargetError(f'frame must be "{lk.FRAME}" (SpiderX body frame), '
                              f'got {cfg.get("frame")!r}')
    if cfg.get('units') != {'length': 'm', 'angle': 'rad'}:
        raise PoseTargetError(f'units must be {{length: m, angle: rad}}, got {cfg.get("units")!r}')
    ref = cfg.get('reference') or {}
    if ref.get('pose') != 'cad_neutral' or ref.get('point') != 'derived_foot_tip':
        raise PoseTargetError('reference must be pose cad_neutral, point derived_foot_tip')
    margin = _num(cfg.get('validation_margin_rad'), 'validation_margin_rad')
    if abs(margin - legs_margin) > 1e-12:
        raise PoseTargetError(f'validation_margin_rad {margin} must equal the M1 margin '
                              f'{legs_margin}')
    tol = _num(cfg.get('ik_position_tol_m'), 'ik_position_tol_m')
    if abs(tol - lk.IK_POSITION_TOL_M) > 1e-15:
        raise PoseTargetError(f'ik_position_tol_m must equal leg_kinematics.IK_POSITION_TOL_M '
                              f'({lk.IK_POSITION_TOL_M})')
    safety = cfg.get('safety')
    if not isinstance(safety, dict):
        raise PoseTargetError('"safety" mapping is required')
    missing, unknown = set(SAFETY_KEYS) - set(safety), set(safety) - set(SAFETY_KEYS)
    if missing or unknown:
        raise PoseTargetError(f'safety: missing {sorted(missing)}, unknown {sorted(unknown)}')
    s = {k: _num(safety[k], f'safety.{k}') for k in SAFETY_KEYS}
    for k, v in s.items():
        if v <= 0:
            raise PoseTargetError(f'safety.{k} must be > 0, got {v}')
    out = {'config_version': cfg['config_version'], 'date': str(cfg['date']), 'frame': lk.FRAME,
           'margin': margin, 'ik_position_tol_m': tol, 'safety': s, 'poses': {}, 'negative': {}}

    poses = cfg.get('poses')
    if not isinstance(poses, dict) or not poses:
        raise PoseTargetError('"poses" must be a non-empty mapping')
    for name, p in poses.items():
        what = f'pose {name}'
        if not isinstance(p, dict):
            raise PoseTargetError(f'{what} must be a mapping')
        if p.get('negative_test_only'):
            raise PoseTargetError(f'{what}: negative_test_only entries belong in negative_tests')
        offsets = {leg: _vec3(v, f'{what} offsets_m.{leg}')
                   for leg, v in _legs_mapping(p.get('offsets_m'), f'{what} offsets_m').items()}
        for leg, off in offsets.items():
            if max(abs(c) for c in off) > s['max_offset_m']:
                raise PoseTargetError(f'{what}: {leg} offset {list(off)} exceeds max_offset_m '
                                      f'{s["max_offset_m"]}')
        out['poses'][name] = {'offsets_m': offsets, 'description': str(p.get('description', '')),
                              'support_legs': _support_legs(p.get('support_legs'),
                                                            f'{what} support_legs')}

    neg = cfg.get('negative_tests', {})
    if not isinstance(neg, dict):
        raise PoseTargetError('"negative_tests" must be a mapping')
    for name, p in neg.items():
        what = f'negative test {name}'
        if not isinstance(p, dict) or p.get('negative_test_only') is not True:
            raise PoseTargetError(f'{what} must set negative_test_only: true')
        exp = p.get('expect') or {}
        if exp.get('reason') not in EXPECT_REASONS:
            raise PoseTargetError(f'{what}: expect.reason must be one of {sorted(EXPECT_REASONS)}')
        try:
            exp_leg = lk.resolve_leg(exp.get('leg'), allow_all_legs=True)
        except lk.KinematicsError as e:
            raise PoseTargetError(f'{what}: expect.leg: {e}') from e
        legs = {}
        for leg, v in _legs_mapping(p.get('offsets_m'), f'{what} offsets_m').items():
            if isinstance(v, dict):
                if set(v) != {'from_joint_config_rad'}:
                    raise PoseTargetError(f'{what} {leg}: mapping must be '
                                          '{from_joint_config_rad: [hip, thigh, foot]}')
                legs[leg] = {'from_joint_config_rad': _vec3(v['from_joint_config_rad'],
                                                            f'{what} {leg}.from_joint_config_rad')}
            else:
                legs[leg] = {'offset_m': _vec3(v, f'{what} offsets_m.{leg}')}
        out['negative'][name] = {'expect_leg': exp_leg, 'expect_reason': exp['reason'],
                                 'legs': legs}
    return out


def _target(geom, entry):
    if 'from_joint_config_rad' in entry:
        return tuple(lk.forward(geom, entry['from_joint_config_rad'])['tip_position'])
    return tuple(t + o for t, o in zip(geom.tip0, entry['offset_m']))


def evaluate_pose(geoms, cfg, name, targets, support_legs, limits):
    """Evaluate ONE pose atomically. Returns a record; 'command_rad' is None unless 'ok'."""
    s = cfg['safety']
    rec = {'name': name, 'ok': False, 'reason': None, 'failing_legs': {}, 'failures': [],
           'frame': lk.FRAME, 'joint_names': lk.all_joint_names(geoms),
           'targets_m': {leg: list(t) for leg, t in targets.items()},
           'support_legs': list(support_legs), 'command_rad': None}
    ik = lk.inverse_all(geoms, targets, margin=cfg['margin'])
    rec['ik'] = {leg: {k: r.get(k) for k in ('ok', 'reason', 'message', 'solution')}
                 for leg, r in ik['legs'].items()}
    if not ik['ok']:
        rec['failing_legs'] = {leg: r['reason'] for leg, r in ik['legs'].items() if not r['ok']}
        rec['reason'] = 'ik_failed'
        rec['failures'] = [f'{leg}: {r["reason"]} ({r.get("message")})'
                           for leg, r in ik['legs'].items() if not r['ok']]
        if ik['reason'] == 'invalid_input':
            rec['failures'].append(ik.get('message', 'invalid input'))
        return rec
    q12 = ik['solution']
    rec['fk_residual_m'], rec['singularity_margin_rad'] = {}, {}
    fk = lk.forward_all(geoms, q12)
    for leg in lk.ALL_LEGS:
        res = math.dist(fk[leg]['tip_position'], targets[leg])
        rec['fk_residual_m'][leg] = res
        c = ik['legs'][leg]['candidates'][ik['legs'][leg]['selected_index']]
        margin = min(c['hip_singularity_margin_rad'], c['knee_singularity_margin_rad'])
        rec['singularity_margin_rad'][leg] = margin
        if res > cfg['ik_position_tol_m']:
            rec['failing_legs'][leg] = 'fk_residual'
            rec['failures'].append(f'{leg}: FK residual {res:.2e} m > {cfg["ik_position_tol_m"]}')
        if margin < s['min_singularity_margin_rad']:
            rec['failing_legs'][leg] = 'singularity'
            rec['failures'].append(f'{leg}: singularity margin {margin:.3f} rad < '
                                   f'{s["min_singularity_margin_rad"]}')
    rec['max_joint_change_rad'] = max(abs(v) for v in q12)        # CAD neutral = all zeros
    if rec['max_joint_change_rad'] > s['max_joint_change_rad']:
        worst = max(zip(rec['joint_names'], q12), key=lambda nv: abs(nv[1]))
        rec['failures'].append(f'joint change {worst[0]}={worst[1]:.3f} rad > '
                               f'{s["max_joint_change_rad"]}')
    try:                                             # the M1 rule, on all 12 values
        check_pose(dict(zip(rec['joint_names'], q12)), limits, cfg['margin'])
    except UnsafeCommandError as e:
        rec['failures'].append(f'M1 check_pose: {e}')
    # necessary geometric support conditions (NOT proof of contact/support/balance)
    zs = [targets[leg][2] for leg in support_legs]
    plane = sum(zs) / len(zs)
    rec['support_plane_z_m'] = plane
    if max(zs) - min(zs) > s['support_plane_tol_m']:
        rec['failures'].append(f'support feet not coplanar: z = '
                               f'{ {leg: round(targets[leg][2], 6) for leg in support_legs} }')
    for leg in lk.ALL_LEGS:
        if leg not in support_legs and targets[leg][2] - plane < s['min_lift_above_support_m']:
            rec['failing_legs'][leg] = 'not_lifted'
            rec['failures'].append(f'{leg}: non-support foot only '
                                   f'{(targets[leg][2] - plane) * 1000:.1f} mm above the support '
                                   f'plane (< {s["min_lift_above_support_m"] * 1000:.1f} mm)')
    rec['geometric_expected_body_height_m'] = {'value': -plane, 'note': GEOMETRIC_NOTE}
    if rec['failures']:
        rec['reason'] = 'safety_check_failed'
        return rec
    rec.update(ok=True, reason='ok', command_rad=list(q12))
    return rec


def evaluate(geoms, cfg, limits):
    """Evaluate every pose and every negative test. Returns (errors, plan)."""
    errors, plan = [], {'reference_tips_m': {leg: list(geoms[leg].tip0) for leg in lk.ALL_LEGS},
                        'poses': {}, 'negative': {}}
    for name, p in cfg['poses'].items():
        targets = {leg: _target(geoms[leg], {'offset_m': off})
                   for leg, off in p['offsets_m'].items()}
        rec = evaluate_pose(geoms, cfg, name, targets, p['support_legs'], limits)
        plan['poses'][name] = rec
        if not rec['ok']:
            errors.append(f'pose {name} refused ({rec["reason"]}): ' + '; '.join(rec['failures']))
    for name, n in cfg['negative'].items():
        targets = {leg: _target(geoms[leg], e) for leg, e in n['legs'].items()}
        rec = evaluate_pose(geoms, cfg, name, targets, list(lk.ALL_LEGS), limits)
        rec['expect'] = {'leg': n['expect_leg'], 'reason': n['expect_reason']}
        plan['negative'][name] = rec
        got = rec['failing_legs'].get(n['expect_leg'])
        if rec['ok']:
            errors.append(f'negative test {name}: pose was unexpectedly accepted')
        elif got not in EXPECT_REASONS[n['expect_reason']]:
            errors.append(f'negative test {name}: expected {n["expect_leg"]} '
                          f'{n["expect_reason"]}, got failing legs {rec["failing_legs"]}')
    return errors, plan


def load_and_evaluate(config_path=None, config_dir=None, urdf_root=None):
    """Installed (or given) config + all-leg URDF geometry. Returns (cfg, geoms, errors, plan).

    Raises PoseTargetError for an invalid config. spiderx_legs.yaml always comes from config_dir
    (default: the installed config directory), never from next to a custom config file.
    """
    config_dir = config_dir or default_config_dir()
    config_path = config_path or os.path.join(config_dir, CONFIG_FILE)
    try:
        raw = load_yaml_strict(config_path)
        legs = load_yaml_strict(os.path.join(config_dir, 'spiderx_legs.yaml'))
    except PostureConfigError as e:          # malformed / duplicate-key YAML
        raise PoseTargetError(str(e)) from e
    try:
        cfg = validate_config(raw, float(legs['soft_limit_margin_rad']))
    except PoseTargetError:
        raise
    except TargetConfigError as e:           # reused M3 number/vector helpers raise the M3 class
        raise PoseTargetError(str(e)) from e
    try:
        geoms = lk.load_all_geometries(urdf_root=urdf_root, legs_cfg=legs)
    except lk.KinematicsError as e:
        raise PoseTargetError(f'geometry: {e}') from e
    limits, _ = load_limits(config_dir)
    errors, plan = evaluate(geoms, cfg, limits)
    return cfg, geoms, errors, plan


def pose_command(plan, name):
    """The validated 12-joint command (all_joint_names order) for a pose, or raise PoseTargetError."""
    rec = plan['poses'].get(name)
    if rec is None:
        raise PoseTargetError(f'unknown pose "{name}"; available: {sorted(plan["poses"])}')
    if not rec['ok']:
        raise PoseTargetError(f'pose {name} is not safe to command: {rec["failures"]}')
    return list(rec['command_rad'])
