"""Load and strictly validate the M2 SIMULATION-ONLY posture configuration.

config/m2_simulation_postures.yaml is the single source of truth for the M2 posture-hold test.
Validation reuses the M1 safety checker (joint_safety.check_pose) and additionally requires the
targets to equal the existing CAD neutral pose, so the posture cannot be tuned silently.
Pure Python (no ROS graph), so it is unit-tested.
"""

import math
import os

import yaml

from spiderx_controller.joint_safety import UnsafeCommandError, check_pose, load_limits

CONFIG_FILE = 'm2_simulation_postures.yaml'
DEFAULT_POSTURE = 'cad_neutral_simulation_hold'
THRESHOLD_KEYS = {
    'max_joint_error_rad': float,
    'rms_joint_error_rad': float,
    'min_body_height_m': float,
    'max_body_height_range_m': float,
    'max_abs_roll_rad': float,
    'max_abs_pitch_rad': float,
    'min_pose_samples': int,
    'min_joint_state_samples': int,
}
EXACT_TOL = 1e-9


class PostureConfigError(UnsafeCommandError):
    """The M2 posture configuration is invalid; nothing may be sent."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """SafeLoader that rejects duplicate mapping keys (plain yaml silently keeps the last one)."""


def _construct_mapping(loader, node, deep=False):
    keys = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in keys:
            raise PostureConfigError(f'duplicate key "{key}" (line {key_node.start_mark.line + 1})')
        keys.add(key)
    return yaml.SafeLoader.construct_mapping(loader, node, deep)


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


def default_config_dir():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('spiderx_controller'), 'config')


def load_yaml_strict(path):
    """Parse YAML, rejecting duplicate keys and malformed files with PostureConfigError."""
    try:
        with open(path) as f:
            data = yaml.load(f, Loader=_UniqueKeyLoader)  # noqa: S506 (SafeLoader subclass)
    except OSError as e:
        raise PostureConfigError(f'cannot read {path}: {e}') from e
    except yaml.YAMLError as e:
        raise PostureConfigError(f'malformed YAML in {path}: {e}') from e
    if not isinstance(data, dict):
        raise PostureConfigError(f'{path}: top level must be a mapping')
    return data


def _number(value, what, kind=float):
    # bool is a subclass of int: reject it explicitly, and reject strings such as "0.0".
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PostureConfigError(f'{what} must be a number, got {value!r}')
    if kind is int and not float(value).is_integer():
        raise PostureConfigError(f'{what} must be an integer, got {value!r}')
    if not math.isfinite(float(value)):
        raise PostureConfigError(f'{what} must be finite, got {value!r}')
    return kind(value)


def validate_posture(cfg, name, limits, cad_neutral, legs_margin, max_joint_velocity):
    """Validate one posture of an already-parsed config. Returns a normalised dict.

    limits: {joint: (lower, upper)} from the URDF (spiderx_legs.yaml).
    cad_neutral: {joint: value} from spiderx_poses.yaml.
    """
    if cfg.get('simulation_only') is not True:
        raise PostureConfigError('top-level simulation_only: true is required')
    for key in ('config_version', 'date'):
        if key not in cfg:
            raise PostureConfigError(f'missing top-level "{key}"')
    postures = cfg.get('postures')
    if not isinstance(postures, dict) or not postures:
        raise PostureConfigError('"postures" must be a non-empty mapping')
    if name not in postures:
        raise PostureConfigError(f'unknown posture "{name}"; available: {sorted(postures)}')
    p = postures[name]
    if not isinstance(p, dict):
        raise PostureConfigError(f'posture "{name}" must be a mapping')
    if p.get('simulation_only') is not True:
        raise PostureConfigError(f'posture "{name}": simulation_only: true is required')
    if p.get('source_pose') != 'cad_neutral':
        raise PostureConfigError(f'posture "{name}": source_pose must be cad_neutral '
                                 '(owner decision: CAD neutral pose only)')

    targets = p.get('targets_rad')
    if not isinstance(targets, dict):
        raise PostureConfigError(f'posture "{name}": targets_rad must be a mapping of 12 joints')
    targets = {j: _number(v, f'target {j}') for j, v in targets.items()}

    margin = _number(p.get('validation_margin_rad'), 'validation_margin_rad')
    if abs(margin - legs_margin) > EXACT_TOL:
        raise PostureConfigError(f'validation_margin_rad {margin} must equal the M1 safety '
                                 f'margin soft_limit_margin_rad {legs_margin}')
    # M1 safety validator: exactly the 12 joints, every target inside URDF limits -/+ margin.
    try:
        check_pose(targets, limits, margin)
    except UnsafeCommandError as e:
        raise PostureConfigError(str(e)) from e
    for j, v in targets.items():
        if abs(v - float(cad_neutral[j])) > EXACT_TOL:
            raise PostureConfigError(f'{j} target {v} differs from cad_neutral '
                                     f'{cad_neutral[j]}: M2 must not alter the posture')

    duration = _number(p.get('command_duration_s'), 'command_duration_s')
    hold = _number(p.get('hold_duration_s'), 'hold_duration_s')
    if duration < 3.0:
        raise PostureConfigError('command_duration_s must be >= 3.0 s (slow 12-joint move)')
    if hold <= 0:
        raise PostureConfigError('hold_duration_s must be > 0')

    th = p.get('thresholds')
    if not isinstance(th, dict):
        raise PostureConfigError(f'posture "{name}": thresholds mapping is required')
    missing = set(THRESHOLD_KEYS) - set(th)
    unknown = set(th) - set(THRESHOLD_KEYS)
    if missing or unknown:
        raise PostureConfigError(f'thresholds: missing {sorted(missing)}, unknown {sorted(unknown)}')
    thresholds = {k: _number(th[k], f'thresholds.{k}', kind) for k, kind in THRESHOLD_KEYS.items()}
    for k, v in thresholds.items():
        if v <= 0:
            raise PostureConfigError(f'thresholds.{k} must be > 0, got {v}')
    if thresholds['rms_joint_error_rad'] > thresholds['max_joint_error_rad']:
        raise PostureConfigError('rms_joint_error_rad must not exceed max_joint_error_rad')
    if max_joint_velocity is None or max_joint_velocity <= 0:
        raise PostureConfigError('spiderx_legs.yaml max_joint_velocity_rad_s must be > 0')

    return {
        'name': name,
        'config_version': cfg['config_version'],
        'date': str(cfg['date']),
        'targets': targets,
        'margin': margin,
        'command_duration_s': duration,
        'hold_duration_s': hold,
        'thresholds': thresholds,
        'max_joint_velocity_rad_s': float(max_joint_velocity),
    }


def load_posture(name=DEFAULT_POSTURE, config_path=None, config_dir=None):
    """Load and validate a posture from the installed (or given) files.

    Returns (posture dict, joint order). Raises PostureConfigError on any problem.
    """
    config_dir = config_dir or default_config_dir()
    config_path = config_path or os.path.join(config_dir, CONFIG_FILE)
    cfg = load_yaml_strict(config_path)
    legs = load_yaml_strict(os.path.join(config_dir, 'spiderx_legs.yaml'))
    poses = load_yaml_strict(os.path.join(config_dir, 'spiderx_poses.yaml'))
    limits, order = load_limits(config_dir)
    posture = validate_posture(
        cfg, name, limits, poses['poses']['cad_neutral']['joints'],
        float(legs['soft_limit_margin_rad']),
        legs['motion_constraints']['max_joint_velocity_rad_s'])
    return posture, order


def peak_joint_speed(start, targets, duration_s):
    """Largest |target - start| / duration over the joints (rad/s)."""
    return max(abs(targets[j] - start[j]) for j in targets) / duration_s
