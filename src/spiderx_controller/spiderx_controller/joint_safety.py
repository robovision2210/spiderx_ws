"""Safety checks shared by the SpiderX M1 joint-position test tools.

The allowed joints and limits come from config/spiderx_legs.yaml. That file is validated against
the URDF by validate_controller_config, so these checks match the URDF.
Pure Python (no ROS graph), so it is unit-tested.
"""

import os

import yaml

DEFAULT_MARGIN_RAD = 0.05


class UnsafeCommandError(ValueError):
    """Raised when a requested joint command violates the safety rules."""


def load_limits(config_dir=None):
    """Return ({joint: (lower, upper)}, [joint order]) from spiderx_legs.yaml (URDF limits)."""
    if config_dir is None:
        from ament_index_python.packages import get_package_share_directory
        config_dir = os.path.join(get_package_share_directory('spiderx_controller'), 'config')
    with open(os.path.join(config_dir, 'spiderx_legs.yaml')) as f:
        cfg = yaml.safe_load(f)
    limits, order = {}, []
    for leg in cfg['legs'].values():
        for role in cfg['joint_order']:
            j = leg['joints'][role]
            limits[j['name']] = (float(j['lower']), float(j['upper']))
            order.append(j['name'])
    return limits, order


def load_pose(name, config_dir=None):
    """Return {joint: target} for a named pose in spiderx_poses.yaml."""
    if config_dir is None:
        from ament_index_python.packages import get_package_share_directory
        config_dir = os.path.join(get_package_share_directory('spiderx_controller'), 'config')
    with open(os.path.join(config_dir, 'spiderx_poses.yaml')) as f:
        poses = yaml.safe_load(f)['poses']
    if name not in poses:
        raise UnsafeCommandError(f'unknown pose "{name}"; available: {sorted(poses)}')
    return {k: float(v) for k, v in poses[name]['joints'].items()}


def check_target(joint, target, limits, margin=DEFAULT_MARGIN_RAD):
    """Raise UnsafeCommandError unless joint is valid and target is inside limits -/+ margin."""
    if joint not in limits:
        raise UnsafeCommandError(
            f'joint "{joint}" is not one of the 12 validated SpiderX joints: {sorted(limits)}')
    if margin < 0:
        raise UnsafeCommandError('margin must be >= 0')
    lower, upper = limits[joint]
    lo, hi = lower + margin, upper - margin
    if not lo <= target <= hi:
        raise UnsafeCommandError(
            f'{joint} target {target:.4f} rad is outside the allowed range '
            f'[{lo:.4f}, {hi:.4f}] (URDF limits [{lower:.4f}, {upper:.4f}] minus margin {margin})')


def check_pose(pose, limits, margin=DEFAULT_MARGIN_RAD):
    """Validate a full pose: exactly the 12 joints, every target inside limits -/+ margin."""
    missing = set(limits) - set(pose)
    extra = set(pose) - set(limits)
    if missing or extra:
        raise UnsafeCommandError(f'pose joints differ from the 12 SpiderX joints '
                                 f'(missing {sorted(missing)}, unknown {sorted(extra)})')
    for joint, target in pose.items():
        check_target(joint, target, limits, margin)
