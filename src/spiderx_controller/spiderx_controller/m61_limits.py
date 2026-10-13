"""M6.1 limits: strict loader for config/m61_limits.yaml. Pure Python; importing starts nothing.

The YAML file is the single source of the M6.1 numbers (owner decision: a separate limits file;
the M6.0-D limits in m6_envelope / m6_live_contract are neither read nor changed here). The loader
refuses a malformed file; whether the values are the APPROVED ones is decided separately by
m61_live_contract.limits_approved(), so an edited file can be loaded, reported and refused.

Owner names -> YAML keys: MAX_TRAJECTORY_POINTS -> max_trajectory_points, MIN_SEGMENT_DURATION ->
min_segment_duration_s, MAX_JOINT_DISPLACEMENT_RAD -> max_joint_displacement_rad.
"""

from dataclasses import asdict, dataclass
import math
import os

SCHEMA = 'spiderx.m61.limits/1'
FILE_NAME = 'm61_limits.yaml'

CONTACT_NOT_MEASURED = 'not_measured'
BASE_CONSTRAINTS = ('fixed', 'free')


class LimitsError(ValueError):
    """config/m61_limits.yaml is missing or malformed; nothing may be built or sent."""


@dataclass(frozen=True)
class Limits:
    # trajectory envelope
    max_trajectory_points: int
    min_segment_duration_s: float
    max_duration_s: float
    lead_in_s: float
    max_joint_displacement_rad: float
    displacement_epsilon_rad: float
    max_planned_joint_speed_rad_s: float
    source_match_rad: float
    spline_check_samples_per_segment: int
    # goal tolerances
    path_position_tolerance_rad: float
    goal_position_tolerance_rad: float
    goal_velocity_tolerance_rad_s: float
    goal_time_tolerance_s: float
    client_tracking_abort_rad: float
    # gait gates
    body_min_height_m: float
    body_max_tilt_rad: float
    body_gate_debounce_samples: int
    body_pose_required: bool
    body_pose_stale_s: float
    joint_limit_fraction: float
    sim_stall_s: float
    joint_state_gap_s: float
    joint_states_stale_s: float
    drift_lateral_report_m: float
    drift_yaw_report_rad: float
    contact_sensing: str
    base_constraint: str

    # the owner's names for the three headline limits
    @property
    def MAX_TRAJECTORY_POINTS(self):  # noqa: N802
        return self.max_trajectory_points

    @property
    def MIN_SEGMENT_DURATION(self):  # noqa: N802
        return self.min_segment_duration_s

    @property
    def MAX_JOINT_DISPLACEMENT_RAD(self):  # noqa: N802
        return self.max_joint_displacement_rad

    def displacement_within_cap(self, displacement_rad):
        """True iff |displacement| <= the cap plus the documented epsilon."""
        return abs(displacement_rad) <= (self.max_joint_displacement_rad
                                         + self.displacement_epsilon_rad)

    def as_dict(self):
        return asdict(self)


INT_KEYS = ('max_trajectory_points', 'spline_check_samples_per_segment',
            'body_gate_debounce_samples')
BOOL_KEYS = ('body_pose_required',)
STR_KEYS = ('contact_sensing', 'base_constraint')
FLOAT_KEYS = tuple(k for k in Limits.__dataclass_fields__
                   if k not in INT_KEYS + BOOL_KEYS + STR_KEYS)
HEADER_KEYS = ('schema', 'milestone', 'simulation_only')
KEYS = HEADER_KEYS + tuple(Limits.__dataclass_fields__)


def _positive(value, key, kind):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LimitsError(f'{key} must be a number, got {value!r}')
    if kind is int and not float(value).is_integer():
        raise LimitsError(f'{key} must be an integer, got {value!r}')
    if not math.isfinite(float(value)) or value <= 0:
        raise LimitsError(f'{key} must be finite and > 0, got {value!r}')
    return kind(value)


def parse(data):
    """A validated Limits from a parsed YAML mapping. Raises LimitsError."""
    if not isinstance(data, dict):
        raise LimitsError('top level must be a mapping')
    unknown = sorted(set(data) - set(KEYS))
    missing = sorted(set(KEYS) - set(data))
    if unknown or missing:
        raise LimitsError(f'keys differ from the schema: unknown {unknown}, missing {missing}')
    if data['schema'] != SCHEMA:
        raise LimitsError(f'schema must be {SCHEMA!r}, got {data["schema"]!r}')
    if data['milestone'] != 'M6.1' or data['simulation_only'] is not True:
        raise LimitsError('milestone must be M6.1 and simulation_only must be true')
    values = {}
    for k in INT_KEYS:
        values[k] = _positive(data[k], k, int)
    for k in FLOAT_KEYS:
        values[k] = _positive(data[k], k, float)
    for k in BOOL_KEYS:
        if not isinstance(data[k], bool):
            raise LimitsError(f'{k} must be true or false, got {data[k]!r}')
        values[k] = data[k]
    if data['contact_sensing'] != CONTACT_NOT_MEASURED:
        raise LimitsError(f'contact_sensing must be {CONTACT_NOT_MEASURED!r} (no contact sensor '
                          f'exists), got {data["contact_sensing"]!r}')
    if data['base_constraint'] not in BASE_CONSTRAINTS:
        raise LimitsError(f'base_constraint must be one of {BASE_CONSTRAINTS}, '
                          f'got {data["base_constraint"]!r}')
    values['contact_sensing'] = data['contact_sensing']
    values['base_constraint'] = data['base_constraint']
    if values['joint_limit_fraction'] >= 1.0:
        raise LimitsError('joint_limit_fraction must be < 1')
    return Limits(**values)


def path(config_dir=None):
    if config_dir is None:
        from spiderx_controller import m6_trajectory as m6t
        config_dir = m6t.default_config_dir()
    return os.path.join(config_dir, FILE_NAME)


def load(config_dir=None):
    """Limits from <config_dir>/m61_limits.yaml (strict: duplicate keys refused)."""
    from spiderx_controller.posture_config import PostureConfigError, load_yaml_strict
    try:
        data = load_yaml_strict(path(config_dir))
    except PostureConfigError as e:
        raise LimitsError(str(e)) from e
    return parse(data)


__all__ = ['SCHEMA', 'FILE_NAME', 'Limits', 'LimitsError', 'parse', 'load', 'path', 'KEYS']
