"""colcon test: M4.5 Batch A - gait configuration schema and loader (offline, no ROS graph)."""
import copy
import os

import pytest
import yaml

from spiderx_controller import gait_config as gc
from spiderx_controller import leg_kinematics as lk

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
SHIPPED_GAITS = ('wave', 'tripod_crawl', 'ripple', 'amble', 'pace', 'trot')


def _raw():
    with open(os.path.join(SRC_CONFIG, gc.CONFIG_FILE)) as f:
        return yaml.safe_load(f)


def _legs():
    with open(os.path.join(SRC_CONFIG, 'spiderx_legs.yaml')) as f:
        return yaml.safe_load(f)


def _refused(raw, match, legs=None):
    with pytest.raises(gc.GaitConfigError, match=match):
        gc.validate_config(raw, legs or _legs())


# ------------------------------------------------------------ the shipped configuration
def test_shipped_config_loads_all_gaits_in_yaml_order():
    cfg = gc.load_gait_config(config_dir=SRC_CONFIG)
    assert [g.name for g in cfg.gaits] == list(SHIPPED_GAITS)
    assert cfg.frame == lk.FRAME and cfg.margin_rad == 0.05
    assert cfg.ik_position_tol_m == lk.IK_POSITION_TOL_M
    assert cfg.analysis.samples_per_cycle == 200
    assert cfg.analysis.joint_speed_reference_rad_s == 0.5


@pytest.mark.parametrize('name', SHIPPED_GAITS)
def test_derived_timing(name):
    g = gc.load_gait_config(config_dir=SRC_CONFIG).gait(name)
    assert g.body_speed_m_s == pytest.approx(0.005)
    assert g.cycle_period_s == pytest.approx(g.step_length_m / (g.duty_factor * g.body_speed_m_s))
    assert g.stride_length_m == pytest.approx(g.step_length_m / g.duty_factor)
    assert len(g.phase_offsets) == 4 and all(0 <= p < 1 for p in g.phase_offsets)


def test_pairs_and_order_are_normalised():
    cfg = gc.load_gait_config(config_dir=SRC_CONFIG)
    assert cfg.gait('trot').swing_order == (('front_left', 'rear_right'),
                                            ('front_right', 'rear_left'))
    assert cfg.gait('wave').swing_order == (('rear_left',), ('front_left',), ('rear_right',),
                                            ('front_right',))
    assert cfg.gait('trot').phase_offset('lf') == 0.0          # aliases resolve
    assert 'trot: beta=0.5' in gc.describe(cfg.gait('trot'))


def test_loading_is_deterministic():
    assert gc.load_gait_config(config_dir=SRC_CONFIG) == gc.load_gait_config(config_dir=SRC_CONFIG)


def test_unknown_gait_is_named():
    with pytest.raises(gc.GaitConfigError, match='unknown gait "gallop"'):
        gc.load_gait_config(config_dir=SRC_CONFIG).gait('gallop')


# ------------------------------------------------------------ header refusals
@pytest.mark.parametrize('key,value,match', [
    ('simulation_only', False, 'offline analysis only'),
    ('offline_only', None, 'offline analysis only'),
    ('frame', 'odom', 'frame must be'),
    ('units', {'length': 'mm', 'angle': 'rad', 'time': 's'}, 'units must be'),
    ('direction_of_travel', '+x', 'direction_of_travel'),
    ('reference', {'pose': 'stand', 'point': 'derived_foot_tip'}, 'reference must be'),
    ('validation_margin_rad', 0.02, 'must equal the M1 margin'),
    ('ik_position_tol_m', 1e-4, 'IK_POSITION_TOL_M'),
    ('config_version', 0, 'config_version'),
])
def test_header_refusals(key, value, match):
    raw = _raw()
    raw[key] = value
    _refused(raw, match)


def test_missing_date_refused():
    raw = _raw()
    del raw['date']
    _refused(raw, 'missing top-level "date"')


def test_joint_speed_reference_must_match_spiderx_legs():
    raw = _raw()
    raw['analysis']['joint_speed_reference_rad_s'] = 1.0
    _refused(raw, 'max_joint_velocity_rad_s')


@pytest.mark.parametrize('change,match', [
    (lambda a: a.update(extra=1), 'unknown keys'),
    (lambda a: a.pop('min_static_margin_m'), 'missing keys'),
    (lambda a: a.update(samples_per_cycle=4), 'samples_per_cycle must be an integer >= 8'),
    (lambda a: a.update(samples_per_cycle=True), 'samples_per_cycle'),
    (lambda a: a.update(max_joint_step_rad=0), 'max_joint_step_rad must be > 0'),
    (lambda a: a.update(min_static_margin_m=-0.001), 'min_static_margin_m must be >= 0'),
])
def test_analysis_refusals(change, match):
    raw = _raw()
    change(raw['analysis'])
    _refused(raw, match)


# ------------------------------------------------------------ gait refusals
def _gait_refused(change, match, gait='wave'):
    raw = _raw()
    change(raw['gaits'][gait])
    _refused(raw, match)


@pytest.mark.parametrize('change,match', [
    (lambda g: g.update(colour='red'), 'unknown keys'),
    (lambda g: g.pop('requires_static_stability'), 'missing keys'),
    (lambda g: g.update(duty_factor=1.0), r'duty_factor must be in \(0, 1\)'),
    (lambda g: g.update(duty_factor=0.0), r'duty_factor must be in \(0, 1\)'),
    (lambda g: g['phase_offsets'].update(front_left=1.0), r'must be in \[0, 1\)'),
    (lambda g: g['phase_offsets'].pop('rear_right'), 'missing legs'),
    (lambda g: g['phase_offsets'].update(lf=0.25), 'given twice'),
    (lambda g: g.update(swing_order=['front_left', 'rear_left', 'rear_right', 'front_right']),
     'disagrees with phase_offsets'),
    (lambda g: g.update(swing_order=['rear_left', 'front_left', 'rear_right']), 'missing legs'),
    (lambda g: g.update(swing_order=['rear_left', 'rear_left', 'front_left', 'rear_right']),
     'listed twice'),
    (lambda g: g.update(swing_order=[['rear_left', 'front_left'], 'rear_right', 'front_right']),
     'must share one phase offset'),
    (lambda g: g.update(step_length_m=-0.01), 'step_length_m must be > 0'),
    (lambda g: g.update(step_height_m=0), 'step_height_m must be > 0'),
    (lambda g: g.update(stance_center_offset_m=[0.0]), r'stance_center_offset_m must be \[x, y\]'),
    (lambda g: g.update(swing_profile='bezier'), 'swing_profile must be one of'),
    (lambda g: g.update(requires_static_stability='yes'), 'must be true or false'),
    (lambda g: g.update(body_speed_m_s=0.005, cycle_period_s=10.0),
     'exactly one of body_speed_m_s or cycle_period_s'),
])
def test_gait_refusals(change, match):
    _gait_refused(change, match)


def test_pair_split_into_separate_groups_is_refused():
    """Legs sharing one phase offset must form one group (a trot pair is not two steps)."""
    _gait_refused(lambda g: g.update(swing_order=['front_left', 'rear_right',
                                                  ['front_right', 'rear_left']]),
                  'legs sharing a phase belong in one group', gait='trot')


def test_gait_name_must_be_a_safe_identifier():
    raw = _raw()
    raw['gaits']['Bad Name'] = raw['gaits'].pop('wave')
    _refused(raw, 'must match')


def test_no_timing_source_is_refused():
    raw = _raw()
    del raw['defaults']['body_speed_m_s']
    _refused(raw, 'exactly one of body_speed_m_s or cycle_period_s')


def test_gait_level_period_replaces_the_default_speed():
    raw = _raw()
    raw['gaits']['trot']['cycle_period_s'] = 8.0
    g = gc.validate_config(raw, _legs()).gait('trot')
    assert g.cycle_period_s == 8.0
    assert g.body_speed_m_s == pytest.approx(0.04 / (0.5 * 8.0))


def test_defaults_refusals():
    raw = _raw()
    raw['defaults']['duty_factor'] = 0.5
    _refused(raw, 'defaults: unknown keys')
    raw = _raw()
    raw['defaults']['cycle_period_s'] = 10.0
    _refused(raw, 'defaults: give at most one')


def test_missing_default_parameter_is_named():
    raw = _raw()
    del raw['defaults']['step_height_m']
    _refused(raw, '"step_height_m" missing')


def test_duplicate_yaml_key_is_refused(tmp_path):
    text = open(os.path.join(SRC_CONFIG, gc.CONFIG_FILE)).read()
    bad = tmp_path / gc.CONFIG_FILE
    bad.write_text(text + '\nframe: base_link\n')
    with pytest.raises(gc.GaitConfigError, match='duplicate key "frame"'):
        gc.load_gait_config(config_path=str(bad), config_dir=SRC_CONFIG)


def test_validation_does_not_modify_its_input():
    raw = _raw()
    before = copy.deepcopy(raw)
    gc.validate_config(raw, _legs())
    assert raw == before
