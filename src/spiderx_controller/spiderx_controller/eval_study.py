"""M5 OFFLINE evaluation study: strict study specification and the immutable experiment matrix.

config/m5_study.yaml declares the staged study of docs/M5_EVALUATION_PLAN.md (Stage 0/1 baseline
and resolution, Stage 2 kinematic block, Stage 3 support block, Stage 4 stance-centre translation
sensitivity), the Stage 0/1 gate, and the expected accounting. This module

* validates the specification strictly (schema, ranges, references to shipped gaits, grid
  alignment, duplicate levels) and refuses it with StudyError on any problem;
* expands it into Variants. Every variant is a YAML-equivalent M4.5 gait that passes the SAME
  validator as config/m4_5_gaits.yaml (gait_config.validate_config); the M4.5 YAML is only read;
* gives every variant a canonical, deterministic identity:
      config_id = first 12 hex of SHA-256 of the canonical PHYSICAL configuration
      eval_id   = first 12 hex of SHA-256 of (canonical configuration, samples per cycle)
* computes the accounting (planned, duplicates of earlier stages, new unique evaluations, distinct
  physical configurations) and refuses the study if it differs from `expected_counts`.

Nothing here evaluates kinematics (eval_runner does, with the unchanged M4.5 evaluator) or writes
files. Offline model analysis only: no ROS runtime, no Gazebo, no hardware.
"""

import copy
from dataclasses import dataclass, replace
import hashlib
import json
import math
import os

from spiderx_controller import gait_config as gc
from spiderx_controller import gait_phase as gp
from spiderx_controller import leg_kinematics as lk
from spiderx_controller.gait_report import jsonable
from spiderx_controller.posture_config import load_yaml_strict, PostureConfigError

SCHEMA = 'spiderx_m5_study/v1'
STUDY_FILE = 'm5_study.yaml'
STAGES = ('S01', 'S2', 'S3', 'S4')        # S01 = Stage 0 (reference resolution) + Stage 1
SPEED_STAGE = 'SPEED'                      # assumption checks, counted separately
ID_HEX = 12
GRID_TOL = 1e-9
POLICY_RULE = 'm45_policy: requires_static_stability = pattern guarantees >= 3 stance feet at every sample'

# allowed ranges (plan section 10.2: beta < 0.5 excluded)
BETA_RANGE = (0.5, 1.0)                    # [0.5, 1)
MAX_STEP_LENGTH_M = 0.30
MAX_STEP_HEIGHT_M = 0.10
MAX_TRANSLATION_M = 0.05
SAMPLES_RANGE = (8, 4000)

TOP_KEYS = ('schema', 'study_name', 'offline_only', 'simulation_only', 'source_config', 'fixed',
            'patterns', 'baseline', 'gate', 'screening', 'stages', 'expected_counts')
FIXED_KEYS = ('samples_per_cycle', 'body_speed_m_s', 'stance_height_offset_m',
              'stance_center_offset_m', 'swing_profile')
BASELINE_KEYS = ('gaits', 'negative_controls', 'reference_samples_per_cycle', 'resolutions',
                 'expected', 'tolerances')
EXPECTED_METRICS = ('min_static_margin_m', 'fraction_statically_stable', 'max_joint_speed_rad_s',
                    'lift_work_proxy_j_per_m', 'joint_travel_rad_per_m')
STAGE_KEYS = {
    'S2': ('title', 'pattern', 'duty_factors', 'step_lengths_m', 'step_heights_m'),
    'S3': ('title', 'patterns', 'duty_factors', 'step_lengths_m', 'step_height_m'),
    'S4': ('title', 'note', 'pattern', 'duty_factors', 'stance_center_y_offsets_m',
           'step_length_m', 'step_height_m'),
}
COUNT_STAGE_KEYS = ('planned', 'new_unique')


class StudyError(gc.GaitConfigError):
    """The M5 study specification is invalid; nothing is evaluated."""


# ---------------------------------------------------------------- canonical identity
def canonical_json(data):
    """Deterministic JSON text: floats rounded to 10 significant digits, sorted keys, no spaces."""
    return json.dumps(jsonable(data), sort_keys=True, separators=(',', ':'))


def short_hash(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()[:ID_HEX]


def physical_config(spec):
    """The physical gait configuration of a GaitSpec (no names, policies or derived values)."""
    return {
        'duty_factor': spec.duty_factor,
        'phase_offsets': {leg: spec.phase_offsets[i] for i, leg in enumerate(lk.ALL_LEGS)},
        'step_length_m': spec.step_length_m,
        'step_height_m': spec.step_height_m,
        'stance_height_offset_m': spec.stance_height_offset_m,
        'stance_center_offset_m': list(spec.stance_center_offset_m),
        'body_speed_m_s': spec.body_speed_m_s,
        'swing_profile': spec.swing_profile,
    }


def config_id_of(physical):
    return short_hash(canonical_json(physical))


def eval_id_of(physical, n):
    return short_hash(canonical_json({'configuration': physical, 'samples_per_cycle': n}))


# ---------------------------------------------------------------- data records
@dataclass(frozen=True)
class Context:
    """The shipped M4.5 inputs the study is built from (read only)."""

    gaits_raw: dict               # parsed m4_5_gaits.yaml
    legs_raw: dict                # parsed spiderx_legs.yaml
    gait_config: object           # validated GaitConfig of m4_5_gaits.yaml
    gaits_path: str
    legs_path: str


@dataclass(frozen=True)
class Study:
    name: str
    raw: dict                     # the validated specification (parsed YAML)
    spec_sha256: str              # SHA-256 of the canonical specification
    study_id: str                 # <name>-<first 12 hex of spec_sha256>


@dataclass(frozen=True)
class Variant:
    """One (configuration, samples per cycle) evaluation."""

    eval_id: str
    config_id: str
    role: str                     # 'stage' | 'speed_check'
    source_gait: str              # shipped gait the pattern (and, for baselines, all values) came from
    pattern: str                  # pattern name, or the gait name for Stage 0/1 baselines
    n: int
    physical: dict                # canonical physical configuration (config_id is its hash)
    effective: dict               # complete effective configuration (physical + derived + analysis)
    gait_config: object           # one-gait GaitConfig for gait_metrics.evaluate_gait, or None
    error: str                    # None, or the GaitConfigError text (invalid configuration)

    @property
    def spec(self):
        return self.gait_config.gaits[0] if self.gait_config is not None else None

    @property
    def valid(self):
        return self.error is None


@dataclass(frozen=True)
class PlanEntry:
    """One planned evaluation of one stage (the 293 study entries + 6 speed checks)."""

    stage: str
    label: str
    eval_id: str
    config_id: str
    n: int
    first_stage: str              # stage that first planned this eval_id
    first_label: str

    @property
    def duplicate(self):
        return self.first_label != self.label


@dataclass(frozen=True)
class Plan:
    study: Study
    entries: tuple                # study PlanEntry, stage order S01, S2, S3, S4
    speed_entries: tuple          # PlanEntry of the separately counted speed checks
    variants: dict                # eval_id -> Variant (unique, first-planned order)
    speed_base: dict              # speed-check eval_id -> baseline eval_id it is compared with
    accounting: dict

    def stage_entries(self, stage):
        return [e for e in self.entries if e.stage == stage]


# ---------------------------------------------------------------- loading
def default_study_path(config_dir=None):
    return os.path.join(config_dir or gc.default_config_dir(), STUDY_FILE)


def load_context(config_dir=None, gaits_path=None):
    """Read the shipped M4.5 gait file and spiderx_legs.yaml (validated with the M4.5 loader)."""
    config_dir = config_dir or gc.default_config_dir()
    gaits_path = gaits_path or os.path.join(config_dir, gc.CONFIG_FILE)
    legs_path = os.path.join(config_dir, 'spiderx_legs.yaml')
    cfg = gc.load_gait_config(config_path=gaits_path, config_dir=config_dir)
    try:
        gaits_raw = load_yaml_strict(gaits_path)
        legs_raw = load_yaml_strict(legs_path)
    except PostureConfigError as e:
        raise StudyError(str(e)) from e
    return Context(gaits_raw, legs_raw, cfg, gaits_path, legs_path)


def load_study(path=None, context=None, config_dir=None):
    """Read and validate the study file. Returns (Study, Context)."""
    context = context or load_context(config_dir=config_dir)
    path = path or default_study_path(config_dir)
    try:
        raw = load_yaml_strict(path)
    except PostureConfigError as e:
        raise StudyError(str(e)) from e
    return validate_study(raw, context), context


# ---------------------------------------------------------------- strict helpers
def _keys(mapping, allowed, what, required=None):
    if not isinstance(mapping, dict):
        raise StudyError(f'{what} must be a mapping')
    required = allowed if required is None else required
    unknown = sorted(set(mapping) - set(allowed))
    missing = sorted(set(required) - set(mapping))
    if unknown or missing:
        raise StudyError(f'{what}: unknown keys {unknown}, missing keys {missing}')


def _number(v, what):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise StudyError(f'{what} must be a finite number, got {v!r}')
    return float(v)


def _in_range(v, lo, hi, what, lo_open=False, hi_open=False):
    v = _number(v, what)
    if (v < lo or (lo_open and v == lo)) or (v > hi or (hi_open and v == hi)):
        raise StudyError(f'{what} = {v} outside {"(" if lo_open else "["}{lo}, {hi}'
                         f'{")" if hi_open else "]"}')
    return v


def _int(v, what, lo, hi):
    if isinstance(v, bool) or not isinstance(v, int) or not lo <= v <= hi:
        raise StudyError(f'{what} must be an integer in [{lo}, {hi}], got {v!r}')
    return v


def _levels(values, what, check):
    if not isinstance(values, list) or not values:
        raise StudyError(f'{what} must be a non-empty list')
    out = [check(v, f'{what}[{i}]') for i, v in enumerate(values)]
    if len(set(out)) != len(out):
        raise StudyError(f'{what} has duplicate levels {values}')
    return out


def _beta(v, what):
    return _in_range(v, BETA_RANGE[0], BETA_RANGE[1], what, hi_open=True)


def _length(v, what):
    return _in_range(v, 0.0, MAX_STEP_LENGTH_M, what, lo_open=True)


def _height(v, what):
    return _in_range(v, 0.0, MAX_STEP_HEIGHT_M, what, lo_open=True)


def _translation(v, what):
    return _in_range(v, -MAX_TRANSLATION_M, MAX_TRANSLATION_M, what)


def _samples(v, what):
    return _int(v, what, *SAMPLES_RANGE)


def _on_grid(x, n):
    return abs(x * n - round(x * n)) <= GRID_TOL


# ---------------------------------------------------------------- validation
def validate_study(raw, context):
    """Validate a parsed study specification against the shipped M4.5 inputs. Returns a Study."""
    _keys(raw, TOP_KEYS, 'study')
    if raw['schema'] != SCHEMA:
        raise StudyError(f'schema must be "{SCHEMA}", got {raw["schema"]!r}')
    if raw['offline_only'] is not True or raw['simulation_only'] is not True:
        raise StudyError('offline_only: true and simulation_only: true are both required')
    name = raw['study_name']
    if not isinstance(name, str) or not gc.NAME_RE.match(name):
        raise StudyError(f'study_name must match {gc.NAME_RE.pattern}, got {name!r}')
    if raw['source_config'] != gc.CONFIG_FILE:
        raise StudyError(f'source_config must be "{gc.CONFIG_FILE}"')
    shipped = {g.name: g for g in context.gait_config.gaits}
    defaults = context.gaits_raw.get('defaults', {})

    fixed = raw['fixed']
    _keys(fixed, FIXED_KEYS, 'fixed')
    _samples(fixed['samples_per_cycle'], 'fixed.samples_per_cycle')
    for key in ('body_speed_m_s', 'stance_height_offset_m', 'stance_center_offset_m',
                'swing_profile'):
        if canonical_json(fixed[key]) != canonical_json(defaults.get(key)):
            raise StudyError(f'fixed.{key} = {fixed[key]!r} must equal the M4.5 shipped default '
                             f'{defaults.get(key)!r} (the baseline must lie inside the grid)')

    patterns = raw['patterns']
    if not isinstance(patterns, dict) or not patterns:
        raise StudyError('patterns must be a non-empty mapping')
    for pname, p in patterns.items():
        if not isinstance(pname, str) or not gc.NAME_RE.match(pname):
            raise StudyError(f'pattern name {pname!r} must match {gc.NAME_RE.pattern}')
        _keys(p, ('source_gait',), f'patterns.{pname}')
        if p['source_gait'] not in shipped:
            raise StudyError(f'patterns.{pname}.source_gait "{p["source_gait"]}" is not a gait of '
                             f'{gc.CONFIG_FILE}: {sorted(shipped)}')

    b = raw['baseline']
    _keys(b, BASELINE_KEYS, 'baseline')
    gaits = b['gaits']
    if not isinstance(gaits, list) or not gaits or len(set(gaits)) != len(gaits):
        raise StudyError('baseline.gaits must be a non-empty list without duplicates')
    for g in gaits:
        if g not in shipped:
            raise StudyError(f'baseline gait "{g}" is not a gait of {gc.CONFIG_FILE}')
    nc = b['negative_controls']
    if not isinstance(nc, list) or not nc or any(g not in gaits for g in nc):
        raise StudyError('baseline.negative_controls must be a non-empty subset of baseline.gaits')
    resolutions = _levels(b['resolutions'], 'baseline.resolutions', _samples)
    ref_n = _samples(b['reference_samples_per_cycle'], 'baseline.reference_samples_per_cycle')
    if ref_n not in resolutions or ref_n != fixed['samples_per_cycle']:
        raise StudyError('baseline.reference_samples_per_cycle must be one of the resolutions and '
                         'equal fixed.samples_per_cycle')
    _keys(b['expected'], gaits, 'baseline.expected')
    for g in gaits:
        e = b['expected'][g]
        _keys(e, ('failed_checks',) + EXPECTED_METRICS, f'baseline.expected.{g}')
        if not isinstance(e['failed_checks'], list):
            raise StudyError(f'baseline.expected.{g}.failed_checks must be a list')
        for m in EXPECTED_METRICS:
            if e[m] is not None:
                _number(e[m], f'baseline.expected.{g}.{m}')
    for g in nc:
        if 'static_stability' not in b['expected'][g]['failed_checks']:
            raise StudyError(f'negative control "{g}" must be expected to fail static_stability')
    _keys(b['tolerances'], EXPECTED_METRICS, 'baseline.tolerances')
    for m, t in b['tolerances'].items():
        _in_range(t, 0.0, 1.0, f'baseline.tolerances.{m}', lo_open=True)

    gate = raw['gate']
    _keys(gate, ('speed_scale', 'pattern_separation'), 'gate')
    ss = gate['speed_scale']
    _keys(ss, ('factor', 'joint_series_abs_tol_rad', 'metric_rel_tol'), 'gate.speed_scale')
    _in_range(ss["factor"], 1.0, 100.0, "gate.speed_scale.factor", lo_open=True)
    _in_range(ss['joint_series_abs_tol_rad'], 0.0, 1e-3, 'gate.speed_scale.joint_series_abs_tol_rad',
              lo_open=True)
    _in_range(ss['metric_rel_tol'], 0.0, 1e-3, 'gate.speed_scale.metric_rel_tol', lo_open=True)
    ps = gate['pattern_separation']
    _keys(ps, ('equal_per_leg_metrics', 'metric_rel_tol', 'support_fraction_abs_tol'),
          'gate.pattern_separation')
    pairs = ps['equal_per_leg_metrics']
    if not isinstance(pairs, list) or not pairs:
        raise StudyError('gate.pattern_separation.equal_per_leg_metrics must be a non-empty list')
    for pair in pairs:
        if (not isinstance(pair, list) or len(pair) != 2 or pair[0] == pair[1]
                or any(g not in gaits for g in pair)):
            raise StudyError(f'gate.pattern_separation pair {pair!r} must name two different '
                             'baseline gaits')
        a, c = shipped[pair[0]], shipped[pair[1]]
        if (a.duty_factor, a.step_length_m, a.step_height_m) != (
                c.duty_factor, c.step_length_m, c.step_height_m) or \
                a.phase_offsets == c.phase_offsets:
            raise StudyError(f'gate.pattern_separation pair {pair}: the gaits must share beta, L '
                             'and h and differ in phase offsets')
    _in_range(ps['metric_rel_tol'], 0.0, 1e-3, 'gate.pattern_separation.metric_rel_tol',
              lo_open=True)
    _in_range(ps['support_fraction_abs_tol'], 0.0, 1e-3,
              'gate.pattern_separation.support_fraction_abs_tol', lo_open=True)

    scr = raw['screening']
    _keys(scr, ('joint_speed_reference_rad_s',), 'screening')
    ref = _number(scr['joint_speed_reference_rad_s'], 'screening.joint_speed_reference_rad_s')
    if abs(ref - context.gait_config.analysis.joint_speed_reference_rad_s) > 1e-12:
        raise StudyError('screening.joint_speed_reference_rad_s must equal the M4.5 analysis '
                         'reference (spiderx_legs.yaml SIMULATION_PLACEHOLDER)')

    stages = raw['stages']
    _keys(stages, ('S2', 'S3', 'S4'), 'stages')
    for sid, keys in STAGE_KEYS.items():
        _keys(stages[sid], keys, f'stages.{sid}')
    s2, s3, s4 = stages['S2'], stages['S3'], stages['S4']
    for sid, pat in (('S2', s2['pattern']), ('S4', s4['pattern'])):
        if pat not in patterns:
            raise StudyError(f'stages.{sid}.pattern "{pat}" is not a defined pattern')
    if not isinstance(s3['patterns'], list) or not s3['patterns'] or \
            len(set(s3['patterns'])) != len(s3['patterns']):
        raise StudyError('stages.S3.patterns must be a non-empty list without duplicates')
    for pat in s3['patterns']:
        if pat not in patterns:
            raise StudyError(f'stages.S3 pattern "{pat}" is not a defined pattern')
    for sid in ('S2', 'S3', 'S4'):
        _levels(stages[sid]['duty_factors'], f'stages.{sid}.duty_factors', _beta)
    _levels(s2['step_lengths_m'], 'stages.S2.step_lengths_m', _length)
    _levels(s2['step_heights_m'], 'stages.S2.step_heights_m', _height)
    _levels(s3['step_lengths_m'], 'stages.S3.step_lengths_m', _length)
    _height(s3['step_height_m'], 'stages.S3.step_height_m')
    _levels(s4['stance_center_y_offsets_m'], 'stages.S4.stance_center_y_offsets_m', _translation)
    _length(s4['step_length_m'], 'stages.S4.step_length_m')
    _height(s4['step_height_m'], 'stages.S4.step_height_m')
    if s4['title'] != 'stance-centre translation sensitivity':
        raise StudyError('stages.S4.title must be "stance-centre translation sensitivity"')

    ec = raw['expected_counts']
    _keys(ec, ('planned', 'new_unique', 'stages_0_3', 'all_stages', 'speed_checks'),
          'expected_counts')
    for key in COUNT_STAGE_KEYS:
        _keys(ec[key], STAGES, f'expected_counts.{key}')
        for sid in STAGES:
            _int(ec[key][sid], f'expected_counts.{key}.{sid}', 0, 100000)
    for key in ('stages_0_3', 'all_stages'):
        _keys(ec[key], ('planned', 'unique', 'distinct_configurations'), f'expected_counts.{key}')
        for k, v in ec[key].items():
            _int(v, f'expected_counts.{key}.{k}', 0, 100000)
    _int(ec['speed_checks'], 'expected_counts.speed_checks', 0, 100000)
    for sid in STAGES:
        if ec['new_unique'][sid] > ec['planned'][sid]:
            raise StudyError(f'expected_counts: new_unique.{sid} > planned.{sid} is impossible')
    text = canonical_json(raw)
    digest = hashlib.sha256(text.encode('utf-8')).hexdigest()
    return Study(name, copy.deepcopy(raw), digest, f'{name}-{digest[:ID_HEX]}')


# ---------------------------------------------------------------- variant construction
def _shipped_effective(context, gait):
    """The shipped gait's effective raw parameters (gait entry merged with the defaults)."""
    return {**context.gaits_raw.get('defaults', {}), **context.gaits_raw['gaits'][gait]}


def _pattern_raw(context, source_gait):
    spec = context.gait_config.gait(source_gait)
    return ({leg: spec.phase_offsets[i] for i, leg in enumerate(lk.ALL_LEGS)},
            [list(group) for group in spec.swing_order])


def make_variant(context, *, source_gait, pattern, n, role='stage', duty_factor, phase_offsets,
                 swing_order, step_length_m, step_height_m, stance_height_offset_m,
                 stance_center_offset_m, body_speed_m_s, swing_profile, description=''):
    """Build one Variant through the M4.5 validator. Invalid configurations are kept (error set)."""
    gait = {
        'description': description, 'duty_factor': duty_factor, 'phase_offsets': phase_offsets,
        'swing_order': swing_order, 'step_length_m': step_length_m,
        'step_height_m': step_height_m, 'stance_height_offset_m': stance_height_offset_m,
        'stance_center_offset_m': list(stance_center_offset_m), 'body_speed_m_s': body_speed_m_s,
        'swing_profile': swing_profile, 'requires_static_stability': False,
    }
    raw = copy.deepcopy(context.gaits_raw)
    raw['analysis']['samples_per_cycle'] = n
    raw['defaults'] = {}
    raw['gaits'] = {'m5_variant': gait}
    try:
        cfg = gc.validate_config(raw, context.legs_raw)
    except gc.TargetConfigError as e:      # GaitConfigError or the reused M3 number error
        physical = {k: gait[k] for k in ('duty_factor', 'phase_offsets', 'step_length_m',
                                         'step_height_m', 'stance_height_offset_m',
                                         'stance_center_offset_m', 'body_speed_m_s',
                                         'swing_profile')}
        physical = json.loads(json.dumps(physical, default=repr))
        eff = {**physical, 'swing_order': swing_order, 'samples_per_cycle': n,
               'source_gait': source_gait, 'pattern': pattern}
        return Variant(eval_id_of(physical, n), config_id_of(physical), role, source_gait,
                       pattern, n, physical, eff, None, str(e))
    spec = cfg.gaits[0]
    policy = gp.support_summary(spec, n)['min_support'] >= 3
    physical = physical_config(spec)
    cid = config_id_of(physical)
    spec = replace(spec, name=f'm5_{cid}', description=description,
                   requires_static_stability=policy)
    cfg = replace(cfg, gaits=(spec,))
    a = cfg.analysis
    effective = {
        **physical,
        'swing_order': [list(g) for g in spec.swing_order],
        'cycle_period_s': spec.cycle_period_s, 'stride_length_m': spec.stride_length_m,
        'requires_static_stability': policy, 'requires_static_stability_rule': POLICY_RULE,
        'samples_per_cycle': n, 'source_gait': source_gait, 'pattern': pattern,
        'frame': cfg.frame, 'validation_margin_rad': cfg.margin_rad,
        'ik_position_tol_m': cfg.ik_position_tol_m,
        'analysis': {'min_static_margin_m': a.min_static_margin_m,
                     'min_singularity_margin_rad': a.min_singularity_margin_rad,
                     'max_joint_step_rad': a.max_joint_step_rad,
                     'joint_speed_reference_rad_s': a.joint_speed_reference_rad_s,
                     'velocity_jump_tol_m_s': a.velocity_jump_tol_m_s},
    }
    return Variant(eval_id_of(physical, n), cid, role, source_gait, pattern, n, physical,
                   effective, cfg, None)


def baseline_variant(context, gait, n, speed_factor=1.0, role='stage'):
    eff = _shipped_effective(context, gait)
    offsets, order = _pattern_raw(context, gait)
    speed = eff['body_speed_m_s'] * speed_factor
    return make_variant(context, source_gait=gait, pattern=gait, n=n, role=role,
                        duty_factor=eff['duty_factor'], phase_offsets=offsets, swing_order=order,
                        step_length_m=eff['step_length_m'], step_height_m=eff['step_height_m'],
                        stance_height_offset_m=eff['stance_height_offset_m'],
                        stance_center_offset_m=eff['stance_center_offset_m'],
                        body_speed_m_s=speed, swing_profile=eff['swing_profile'],
                        description=f'baseline {gait}' + (f' at {speed_factor:g} v' if
                                                          speed_factor != 1.0 else ''))


def pattern_variant(study, context, pattern, *, beta, L, h, center_y=None):
    raw, fixed = study.raw, study.raw['fixed']
    source = raw['patterns'][pattern]['source_gait']
    offsets, order = _pattern_raw(context, source)
    cx, cy = fixed['stance_center_offset_m']
    return make_variant(context, source_gait=source, pattern=pattern,
                        n=fixed['samples_per_cycle'], duty_factor=beta, phase_offsets=offsets,
                        swing_order=order, step_length_m=L, step_height_m=h,
                        stance_height_offset_m=fixed['stance_height_offset_m'],
                        stance_center_offset_m=[cx, cy if center_y is None else center_y],
                        body_speed_m_s=fixed['body_speed_m_s'],
                        swing_profile=fixed['swing_profile'], description=f'{pattern}')


def _mm(x):
    return f'{x * 1000:g}'


def stage_variants(study, context):
    """[(stage, label, Variant)] in the fixed stage order S01, S2, S3, S4."""
    raw = study.raw
    out = []
    b = raw['baseline']
    for gait in b['gaits']:
        for n in b['resolutions']:
            out.append(('S01', f'S01-{gait}-n{n}', baseline_variant(context, gait, n)))
    s2 = raw['stages']['S2']
    for beta in s2['duty_factors']:
        for L in s2['step_lengths_m']:
            for h in s2['step_heights_m']:
                out.append(('S2', f'S2-{s2["pattern"]}-b{beta:g}-L{_mm(L)}-h{_mm(h)}',
                            pattern_variant(study, context, s2['pattern'], beta=beta, L=L, h=h)))
    s3 = raw['stages']['S3']
    for pat in s3['patterns']:
        for beta in s3['duty_factors']:
            for L in s3['step_lengths_m']:
                h = s3['step_height_m']
                out.append(('S3', f'S3-{pat}-b{beta:g}-L{_mm(L)}-h{_mm(h)}',
                            pattern_variant(study, context, pat, beta=beta, L=L, h=h)))
    s4 = raw['stages']['S4']
    for beta in s4['duty_factors']:
        for y in s4['stance_center_y_offsets_m']:
            out.append(('S4', f'S4-{s4["pattern"]}-b{beta:g}-y{_mm(y)}',
                        pattern_variant(study, context, s4['pattern'], beta=beta,
                                        L=s4['step_length_m'], h=s4['step_height_m'],
                                        center_y=y)))
    return out


def _check_grid(variant, label):
    spec = variant.spec
    if spec is None:
        return
    n = variant.n
    if not _on_grid(1.0 - spec.duty_factor, n) or not all(_on_grid(p, n)
                                                          for p in spec.phase_offsets):
        raise StudyError(f'{label}: phase transitions are not on the 1/{n} sample grid '
                         f'(beta {spec.duty_factor}, offsets {spec.phase_offsets}); support '
                         'fractions would be approximate - choose aligned levels')


# ---------------------------------------------------------------- the plan
def build_plan(study, context):
    """Expand the study into the immutable experiment matrix and verify the expected accounting."""
    raw = study.raw
    entries, variants, seen, labels = [], {}, {}, set()
    for stage, label, v in stage_variants(study, context):
        _check_grid(v, label)
        if label in labels:
            raise StudyError(f'duplicate variant label {label}')
        labels.add(label)
        if v.eval_id in seen and seen[v.eval_id][0] == stage:
            raise StudyError(f'{label} duplicates {seen[v.eval_id][1]} within stage {stage} '
                             '(duplicate levels)')
        if v.eval_id not in seen:
            seen[v.eval_id] = (stage, label)
            variants[v.eval_id] = v
        first_stage, first_label = seen[v.eval_id]
        entries.append(PlanEntry(stage, label, v.eval_id, v.config_id, v.n, first_stage,
                                 first_label))

    gate = raw['gate']['speed_scale']
    ref_n = raw['baseline']['reference_samples_per_cycle']
    speed_entries, speed_base = [], {}
    for gait in raw['baseline']['gaits']:
        v = baseline_variant(context, gait, ref_n, speed_factor=gate['factor'],
                             role='speed_check')
        base = baseline_variant(context, gait, ref_n)
        label = f'SPEED-{gait}-x{gate["factor"]:g}-n{ref_n}'
        if v.eval_id in variants or v.eval_id in speed_base:
            raise StudyError(f'{label} coincides with another evaluation; speed checks must be '
                             'separate configurations')
        _check_grid(v, label)
        speed_entries.append(PlanEntry(SPEED_STAGE, label, v.eval_id, v.config_id, v.n,
                                       SPEED_STAGE, label))
        speed_base[v.eval_id] = base.eval_id
        variants[v.eval_id] = v

    accounting = compute_accounting(entries, speed_entries, variants)
    check_accounting(accounting, raw['expected_counts'])
    return Plan(study, tuple(entries), tuple(speed_entries), variants, speed_base, accounting)


def compute_accounting(entries, speed_entries, variants):
    acc = {'planned': {}, 'duplicates_of_earlier': {}, 'new_unique': {}, 'new_configurations': {}}
    seen_configs = set()
    for sid in STAGES:
        es = [e for e in entries if e.stage == sid]
        new = [e for e in es if not e.duplicate]
        acc['planned'][sid] = len(es)
        acc['duplicates_of_earlier'][sid] = len(es) - len(new)
        acc['new_unique'][sid] = len(new)
        configs = {e.config_id for e in new} - seen_configs
        acc['new_configurations'][sid] = len(configs)
        seen_configs |= configs

    def totals(sids):
        es = [e for e in entries if e.stage in sids]
        return {'planned': len(es), 'unique': len({e.eval_id for e in es}),
                'distinct_configurations': len({e.config_id for e in es})}
    acc['stages_0_3'] = totals(('S01', 'S2', 'S3'))
    acc['all_stages'] = totals(STAGES)
    acc['speed_checks'] = len(speed_entries)
    acc['invalid_configurations'] = sum(1 for e in entries if not variants[e.eval_id].valid
                                        and not e.duplicate)
    acc['note'] = ('unique = distinct (configuration, samples per cycle) evaluations; '
                   'distinct_configurations ignores the sample count; the speed checks are '
                   'separate assumption-check evaluations, not part of the stage totals')
    return acc


def check_accounting(acc, expected):
    """Refuse the study if the generated matrix does not reproduce the declared accounting."""
    problems = []
    for key in COUNT_STAGE_KEYS:
        for sid in STAGES:
            if acc[key][sid] != expected[key][sid]:
                problems.append(f'{key}.{sid}: generated {acc[key][sid]}, expected '
                                f'{expected[key][sid]}')
    for key in ('stages_0_3', 'all_stages'):
        for k, v in expected[key].items():
            if acc[key][k] != v:
                problems.append(f'{key}.{k}: generated {acc[key][k]}, expected {v}')
    if acc['speed_checks'] != expected['speed_checks']:
        problems.append(f'speed_checks: generated {acc["speed_checks"]}, expected '
                        f'{expected["speed_checks"]}')
    if problems:
        raise StudyError('the generated experiment matrix does not match expected_counts: '
                         + '; '.join(problems))


__all__ = ['SCHEMA', 'STUDY_FILE', 'STAGES', 'SPEED_STAGE', 'StudyError', 'Context', 'Study',
           'Variant', 'PlanEntry', 'Plan', 'load_context', 'load_study', 'validate_study',
           'build_plan', 'make_variant', 'baseline_variant', 'pattern_variant', 'canonical_json',
           'physical_config', 'config_id_of', 'eval_id_of', 'compute_accounting',
           'check_accounting', 'default_study_path']
