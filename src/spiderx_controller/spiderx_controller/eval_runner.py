"""M5 OFFLINE study runner: evaluation records, the Stage 0/1 gate, and gate enforcement.

Every evaluation calls the unchanged M4.5 evaluator (gait_metrics.evaluate_gait) on one Variant of
the immutable experiment matrix (eval_study.build_plan). This module adds no kinematic or metric
mathematics of its own: it only classifies the M4.5 outputs into explicit result categories and
derives the continuous joint-speed indicator K = max joint speed / body speed.

Result categories (never filtered, always written):
    evaluation_category   valid_pass | kinematic_check_fail | ik_infeasible |
                          invalid_configuration | blocked_gate_failed
        valid_pass = valid configuration, sampled-IK feasible at every sample and every M4.5
        kinematic model check passed. It is NOT a static-support, stability, walking or hardware
        claim; static support and the joint-speed screen are separate columns.
    static_support_status pass | fail | not_applicable | not_evaluated
        static-support APPROXIMATION over the samples with >= 3 stance feet only;
        not_applicable = no sample has three stance feet (e.g. pace, trot) - never zero, never fail.
    joint_speed_screen    pass | fail | not_evaluated
        PROVISIONAL SECONDARY flag against the 0.5 rad/s SIMULATION_PLACEHOLDER. Not an actuator
        limit and not part of evaluation_category.

The Stage 0/1 gate (gate_checks) reproduces the M4.5 baseline and verifies the two assumptions that
justify the staged design (speed only rescales time; per-leg path metrics are pattern-independent
while support metrics are evaluated separately). Stages 2-4 run only if every gate check passes
(_gate_allows_later_stages is the single enforcement point); otherwise their evaluations are
recorded as blocked_gate_failed and the study status is 'blocked'.

Offline only: no ROS runtime, no Gazebo, never commands the robot.
"""

from collections import Counter
from dataclasses import dataclass, field

from spiderx_controller import eval_study as es
from spiderx_controller import gait_kinematics as gk
from spiderx_controller import gait_metrics as gm
from spiderx_controller import leg_kinematics as lk

LEG_SHORT = {'front_left': 'LF', 'front_right': 'RF', 'rear_left': 'LR', 'rear_right': 'RR'}
KINEMATIC_CHECKS = ('ik_feasible', 'fk_residual', 'singularity_margin', 'joint_limit_margin',
                    'joint_continuity', 'transition_velocity_jump')
CATEGORIES = ('valid_pass', 'kinematic_check_fail', 'ik_infeasible', 'invalid_configuration',
              'blocked_gate_failed')
STATIC_STATUSES = ('pass', 'fail', 'not_applicable', 'not_evaluated')
SCREEN_STATUSES = ('pass', 'fail', 'not_evaluated')
STUDY_STATUSES = ('completed', 'blocked')
# metrics that depend only on each leg's own path (pattern-independent on aligned grids)
PER_LEG_METRICS = ('max_joint_speed_rad_s', 'k_rad_per_m', 'min_singularity_margin_rad',
                   'min_joint_limit_margin_rad', 'max_joint_step_rad', 'foot_path_per_m',
                   'joint_travel_rad_per_m', 'lift_work_proxy_j_per_m', 'steps_per_m')
SUPPORT_METRICS = ('min_support_feet', 'max_support_feet', 'fraction_two_feet',
                   'fraction_three_feet', 'fraction_four_feet', 'support_ge3_fraction')
# speed-invariant (geometric) metrics checked by the speed-scale gate
GEOMETRIC_METRICS = ('min_static_margin_m', 'fraction_statically_stable',
                     'min_singularity_margin_rad', 'min_joint_limit_margin_rad',
                     'max_joint_step_rad', 'foot_path_per_m', 'joint_travel_rad_per_m',
                     'lift_work_proxy_j_per_m', 'steps_per_m') + SUPPORT_METRICS
M45_METRICS = ('duty_factor', 'cycle_period_s', 'body_speed_m_s', 'step_length_m',
               'stride_length_m', 'step_height_m', 'min_support_feet', 'max_support_feet',
               'fraction_two_feet', 'fraction_three_feet', 'fraction_four_feet',
               'min_static_margin_m', 'fraction_statically_stable', 'max_joint_speed_rad_s',
               'min_singularity_margin_rad', 'min_joint_limit_margin_rad', 'max_joint_step_rad',
               'max_transition_velocity_jump_m_s', 'max_transition_acceleration_jump_m_s2',
               'steps_per_m', 'foot_path_per_m', 'joint_travel_rad_per_m',
               'lift_work_proxy_j_per_m')


@dataclass(frozen=True)
class Inputs:
    geoms: dict
    mass: object


@dataclass
class EvalResult:
    record: dict                  # flat, deterministic record (CSV/JSON)
    q_series: dict = None         # {leg: [q tuple or None]} - in memory only (gate checks)
    checks: list = field(default_factory=list)       # M4.5 Check.as_dict() list
    ik_reasons: dict = field(default_factory=dict)   # {leg: {reason: count}}


@dataclass
class GateCheck:
    check_id: str
    scope: str
    passed: bool
    detail: str


@dataclass
class StudyResult:
    plan: object
    results: dict                 # eval_id -> EvalResult (unique stage evaluations + speed checks)
    gate: list                    # [GateCheck]
    status: str                   # 'completed' | 'blocked'
    evaluated: list               # eval_ids actually passed to the evaluator, in order

    @property
    def gate_passed(self):
        return all(g.passed for g in self.gate)


def load_inputs(config_dir=None):
    _, geoms, mass = gm.load_inputs(config_dir=config_dir)
    return Inputs(geoms, mass)


# ---------------------------------------------------------------- records
def _base_record(variant, entries):
    stages = sorted({e.stage for e in entries}, key=_stage_order)
    eff = variant.effective
    phases = eff.get('phase_offsets') or {}
    center = eff.get('stance_center_offset_m') or [None, None]
    return {
        'eval_id': variant.eval_id, 'config_id': variant.config_id, 'role': variant.role,
        'stages': '|'.join(stages), 'labels': '|'.join(e.label for e in entries),
        'source_gait': variant.source_gait, 'pattern': variant.pattern,
        'samples_per_cycle': variant.n,
        'duty_factor': eff.get('duty_factor'),
        **{f'phase_{LEG_SHORT[leg]}': phases.get(leg) for leg in lk.ALL_LEGS},
        'step_length_m': eff.get('step_length_m'), 'step_height_m': eff.get('step_height_m'),
        'stance_height_offset_m': eff.get('stance_height_offset_m'),
        'stance_center_x_m': center[0], 'stance_center_y_m': center[1],
        'body_speed_m_s': eff.get('body_speed_m_s'), 'cycle_period_s': eff.get('cycle_period_s'),
        'stride_length_m': eff.get('stride_length_m'), 'swing_profile': eff.get('swing_profile'),
        'requires_static_stability_m45_policy': eff.get('requires_static_stability'),
    }


EMPTY_OUTCOME = {
    'config_error': None, 'ik_status': 'not_evaluated', 'ik_failed_samples': None,
    'kinematic_checks_failed': None,
    'static_support_status': 'not_evaluated', 'support_ge3_fraction': None,
    'min_static_margin_m': None, 'fraction_margin_pos_given_ge3': None,
    'fraction_margin_ge_threshold_given_ge3': None, 'static_fail_samples_ge3': None,
    'binding_support': None, 'binding_u': None,
    'k_rad_per_m': None, 'max_joint_speed_rad_s': None, 'max_joint_speed_joint': None,
    'joint_speed_screen': 'not_evaluated', 'joint_speed_screen_breaches': None,
    'v_admissible_at_reference_m_s': None, 'joint_speed_reference_rad_s': None,
    'm45_verdict': None, 'm45_failed_checks': None, 'worst_fk_residual_m': None,
    **{f'm45_{k}': None for k in M45_METRICS},
}


def blocked_record(variant, entries):
    return {**_base_record(variant, entries), 'evaluation_status': 'blocked_gate_failed',
            'evaluation_category': 'blocked_gate_failed', **EMPTY_OUTCOME}


def invalid_record(variant, entries):
    return {**_base_record(variant, entries), 'evaluation_status': 'invalid_configuration',
            'evaluation_category': 'invalid_configuration',
            **{**EMPTY_OUTCOME, 'config_error': variant.error}}


def _support_outcome(ev, threshold, all_solved):
    counts = ev.series['support_count']
    margins = ev.series['static_margin_m']
    n = len(counts)
    ge3 = [k for k, c in enumerate(counts) if c >= 3]
    out = {'support_ge3_fraction': len(ge3) / n}
    if not all_solved:
        return {**out, 'static_support_status': 'not_evaluated'}
    if not ge3:
        return {**out, 'static_support_status': 'not_applicable'}
    ms = [margins[k] for k in ge3]
    worst = min(ms)
    k_min = ge3[ms.index(worst)]
    sample = ev.kinematics.samples[k_min]
    return {**out,
            'static_support_status': 'pass' if worst >= threshold else 'fail',
            'min_static_margin_m': worst,
            'fraction_margin_pos_given_ge3': sum(1 for m in ms if m > 0) / len(ge3),
            'fraction_margin_ge_threshold_given_ge3': sum(1 for m in ms if m >= threshold)
            / len(ge3),
            'static_fail_samples_ge3': sum(1 for m in ms if m < threshold),
            'binding_support': '|'.join(LEG_SHORT[leg] for leg in sample['support']),
            'binding_u': sample['u']}


def _joint_speed_outcome(ev, v, reference, all_solved):
    if not all_solved:
        return {}
    worst, where = None, None
    for leg, s in ev.kinematics.legs.items():
        for sp in gk.joint_speeds(s.q, ev.kinematics.dt_s):
            if sp is not None and (worst is None or max(sp) > worst):
                worst, where = max(sp), s.joint_names[sp.index(max(sp))]
    k = worst / v
    check = next(c for c in ev.checks if c.name == 'joint_speed')
    return {'k_rad_per_m': k, 'max_joint_speed_rad_s': worst, 'max_joint_speed_joint': where,
            'joint_speed_screen': 'pass' if worst <= reference else 'fail',
            'joint_speed_screen_breaches': check.failure_count,
            'v_admissible_at_reference_m_s': reference / k if k > 0 else None,
            'joint_speed_reference_rad_s': reference}


def record_from_evaluation(variant, entries, ev):
    checks = {c.name: c for c in ev.checks}
    all_solved = ev.kinematics.all_solved
    failed_kin = [k for k in KINEMATIC_CHECKS if not checks[k].passed]
    ik_ok = checks['ik_feasible'].passed
    category = ('valid_pass' if not failed_kin else
                'ik_infeasible' if not ik_ok else 'kinematic_check_fail')
    a = variant.gait_config.analysis
    rec = {**_base_record(variant, entries), 'evaluation_status': 'evaluated',
           'evaluation_category': category, **EMPTY_OUTCOME,
           'ik_status': 'feasible' if ik_ok else 'infeasible',
           'ik_failed_samples': checks['ik_feasible'].value,
           'kinematic_checks_failed': '|'.join(failed_kin),
           'worst_fk_residual_m': checks['fk_residual'].value,
           'm45_verdict': 'PASS' if ev.passed else 'FAIL',
           'm45_failed_checks': '|'.join(ev.failed_checks),
           **{f'm45_{k}': ev.metrics.get(k) for k in M45_METRICS}}
    rec.update(_support_outcome(ev, a.min_static_margin_m, all_solved))
    rec.update(_joint_speed_outcome(ev, variant.spec.body_speed_m_s,
                                    a.joint_speed_reference_rad_s, all_solved))
    return rec


def evaluate_variant(variant, entries, inputs):
    """Evaluate one Variant with the unchanged M4.5 evaluator. Returns an EvalResult."""
    if not variant.valid:
        return EvalResult(invalid_record(variant, entries))
    ev = gm.evaluate_gait(variant.spec, variant.gait_config, inputs.geoms, inputs.mass,
                          n=variant.n)
    reasons = {leg: dict(sorted(Counter(r for r in s.reason if r != 'ok').items()))
               for leg, s in ev.kinematics.legs.items()}
    return EvalResult(record_from_evaluation(variant, entries, ev),
                      {leg: list(s.q) for leg, s in ev.kinematics.legs.items()},
                      [c.as_dict() for c in ev.checks], reasons)


def _stage_order(stage):
    return (es.STAGES + (es.SPEED_STAGE,)).index(stage)


# ---------------------------------------------------------------- gate
def _rel_equal(a, b, tol):
    if a is None or b is None:
        return a is None and b is None
    return abs(a - b) <= tol * max(abs(a), abs(b), 1e-300) or a == b


def _baseline(plan, results):
    """{gait: {n: record}} of the Stage 0/1 evaluations."""
    out = {}
    for e in plan.stage_entries('S01'):
        rec = results[e.eval_id].record
        out.setdefault(rec['source_gait'], {})[e.n] = rec
    return out


def gate_checks(plan, results):
    """The Stage 0/1 gate. Every check is reported; the gate passes only if all pass."""
    raw = plan.study.raw
    b, g = raw['baseline'], raw['gate']
    ref_n = b['reference_samples_per_cycle']
    base = _baseline(plan, results)
    checks = []

    bad = [f'{gait} n={n}: {r["evaluation_category"]}' for gait, by_n in base.items()
           for n, r in sorted(by_n.items()) if r['ik_status'] != 'feasible'
           or r['evaluation_status'] != 'evaluated']
    checks.append(GateCheck('baseline_valid_and_ik_feasible', 'every baseline gait, every n',
                            not bad, '; '.join(bad) or 'all baseline evaluations valid and '
                            'sampled-IK feasible'))

    problems = []
    for gait in b['gaits']:
        rec, exp = base[gait][ref_n], b['expected'][gait]
        got_failed = rec['m45_failed_checks'].split('|') if rec['m45_failed_checks'] else []
        if got_failed != exp['failed_checks']:
            problems.append(f'{gait}: failed checks {got_failed} != {exp["failed_checks"]}')
        for m in es.EXPECTED_METRICS:
            got, want, tol = rec[f'm45_{m}'], exp[m], b['tolerances'][m]
            if want is None or got is None:
                if want is not got and not (want is None and got is None):
                    problems.append(f'{gait}: {m} {got} != {want}')
            elif abs(got - want) > tol:
                problems.append(f'{gait}: {m} {got:.6g} differs from {want} by more than {tol}')
    checks.append(GateCheck(f'baseline_reproduction_n{ref_n}', f'six M4.5 gaits at n={ref_n}',
                            not problems, '; '.join(problems) or
                            'verdicts, failed checks and pinned metrics reproduced'))

    problems = []
    for gait in b['negative_controls']:
        for n, rec in sorted(base[gait].items()):
            failed = (rec['m45_failed_checks'] or '').split('|')
            if rec['static_support_status'] != 'fail' or 'static_stability' not in failed:
                problems.append(f'{gait} n={n}: static support {rec["static_support_status"]}, '
                                f'M4.5 failed checks {failed}')
    checks.append(GateCheck('negative_controls_reproduced', 'negative controls, every n',
                            not problems, '; '.join(problems) or
                            f'{b["negative_controls"]} fail the static-support approximation at '
                            'every resolution'))

    problems = []
    for gait, by_n in base.items():
        for n, rec in sorted(by_n.items()):
            if rec['support_ge3_fraction'] == 0 and (
                    rec['static_support_status'] != 'not_applicable'
                    or rec['min_static_margin_m'] is not None):
                problems.append(f'{gait} n={n}: {rec["static_support_status"]}, '
                                f'margin {rec["min_static_margin_m"]}')
    na = sorted({gait for gait, by_n in base.items()
                 if all(r['static_support_status'] == 'not_applicable' for r in by_n.values())})
    checks.append(GateCheck('static_support_na_preserved', 'baselines without 3-foot support',
                            not problems and bool(na), '; '.join(problems) or
                            f'N/A (not zero, not fail) for {na} at every n'))

    shipped = plan.shipped_flags
    problems = [f'{gait}: {rec["requires_static_stability_m45_policy"]} != {shipped[gait]}'
                for gait, by_n in base.items() for rec in [by_n[ref_n]]
                if rec['requires_static_stability_m45_policy'] != shipped[gait]]
    checks.append(GateCheck('m45_policy_matches_shipped_flags', 'six baseline gaits',
                            not problems, '; '.join(problems) or
                            'derived requires_static_stability equals the shipped flags'))

    ss = g['speed_scale']
    problems, ratios = [], []
    for e in plan.speed_entries:
        fast = results[e.eval_id]
        slow = results[plan.speed_base[e.eval_id]]
        gait = fast.record['source_gait']
        if fast.record['ik_status'] != 'feasible' or slow.record['ik_status'] != 'feasible':
            problems.append(f'{gait}: not IK-feasible')
            continue
        qd = _max_q_diff(slow.q_series, fast.q_series)
        if qd is None or qd > ss['joint_series_abs_tol_rad']:
            problems.append(f'{gait}: joint series differ by {qd}')
        for m in GEOMETRIC_METRICS:
            a, c = _value(slow.record, m), _value(fast.record, m)
            if not _rel_equal(a, c, ss['metric_rel_tol']):
                problems.append(f'{gait}: {m} {a} != {c}')
        ratio = fast.record['max_joint_speed_rad_s'] / slow.record['max_joint_speed_rad_s']
        ratios.append(ratio)
        if not _rel_equal(ratio, ss['factor'], ss['metric_rel_tol']):
            problems.append(f'{gait}: joint-speed ratio {ratio:.12g} != {ss["factor"]}')
        if not _rel_equal(fast.record['k_rad_per_m'], slow.record['k_rad_per_m'],
                          ss['metric_rel_tol']):
            problems.append(f'{gait}: K not speed-invariant')
    checks.append(GateCheck('speed_scale_invariance', f'six baselines at {ss["factor"]:g} v',
                            not problems and len(ratios) == len(plan.speed_entries),
                            '; '.join(problems) or
                            f'joint series identical; geometric metrics unchanged; joint-speed '
                            f'ratio {min(ratios):.10g}..{max(ratios):.10g}; K unchanged'))

    ps = g['pattern_separation']
    problems = []
    for a, c in ps['equal_per_leg_metrics']:
        for n in b['resolutions']:
            ra, rc = base[a][n], base[c][n]
            for m in PER_LEG_METRICS:
                if not _rel_equal(_value(ra, m), _value(rc, m), ps['metric_rel_tol']):
                    problems.append(f'{a} vs {c} n={n}: {m} {_value(ra, m)} != '
                                    f'{_value(rc, m)}')
    for gait, by_n in base.items():
        recs = [by_n[n] for n in sorted(by_n)]
        for m in SUPPORT_METRICS:
            vals = [_value(r, m) for r in recs]
            if any(abs(v - vals[0]) > ps['support_fraction_abs_tol'] for v in vals):
                problems.append(f'{gait}: {m} varies with n: {vals}')
    checks.append(GateCheck('pattern_separation', 'per-leg vs support metrics',
                            not problems, '; '.join(problems) or
                            f'per-leg path metrics equal for {ps["equal_per_leg_metrics"]} at '
                            'every n; support metrics resolution-invariant on aligned grids'))
    return checks


def _value(rec, metric):
    """A record value: the M5 columns K, max joint speed and support_ge3_fraction; otherwise the
    verbatim M4.5 metric."""
    if metric in ('k_rad_per_m', 'max_joint_speed_rad_s', 'support_ge3_fraction'):
        return rec[metric]
    return rec[f'm45_{metric}']


def _max_q_diff(a, b):
    if not a or not b:
        return None
    worst = 0.0
    for leg in lk.ALL_LEGS:
        for qa, qb in zip(a[leg], b[leg]):
            if qa is None or qb is None:
                return None
            worst = max(worst, max(abs(x - y) for x, y in zip(qa, qb)))
    return worst


# ---------------------------------------------------------------- orchestration
def _gate_allows_later_stages(gate_passed):
    """THE gate enforcement point: Stages 2-4 are evaluated only after a passed Stage 0/1 gate."""
    return gate_passed


def run_study(plan, inputs, evaluate=evaluate_variant, progress=None):
    """Run the staged study. Stage 0/1 and the speed checks first, then the gate, then (only if it
    passed) Stages 2-4. Returns a StudyResult; nothing is written here."""
    entries_by_id = {}
    for e in plan.entries + plan.speed_entries:
        entries_by_id.setdefault(e.eval_id, []).append(e)
    first = [e for e in plan.entries if not e.duplicate]
    gate_ids = [e.eval_id for e in first if e.stage == 'S01'] + \
               [e.eval_id for e in plan.speed_entries]
    later_ids = [e.eval_id for e in first if e.stage != 'S01']
    results, evaluated = {}, []
    total = len(gate_ids) + len(later_ids)

    def run(eid):
        evaluated.append(eid)
        if progress:
            progress(len(evaluated), total, entries_by_id[eid][0].label)
        results[eid] = evaluate(plan.variants[eid], entries_by_id[eid], inputs)

    for eid in gate_ids:
        run(eid)
    gate = gate_checks(plan, results)
    passed = all(c.passed for c in gate)
    if _gate_allows_later_stages(passed):
        for eid in later_ids:
            run(eid)
    else:
        for eid in later_ids:
            results[eid] = EvalResult(blocked_record(plan.variants[eid], entries_by_id[eid]))
    for r in results.values():            # q series are gate evidence only; not kept or written
        r.q_series = None
    s01_ids = [e.eval_id for e in first if e.stage == 'S01']
    speed_ids = [e.eval_id for e in plan.speed_entries]
    ordered = {eid: results[eid] for eid in s01_ids + later_ids + speed_ids}
    return StudyResult(plan, ordered, gate, 'completed' if passed else 'blocked', evaluated)


def verify_result(result):
    """Structural invariants of a StudyResult. Returns a list of violations (empty = consistent).

    * one record per unique planned evaluation plus one per speed check - nothing filtered;
    * every category and status is one of the declared values;
    * a blocked study evaluated nothing after the gate and marked every later-stage record
      blocked_gate_failed; a completed study has no blocked records.
    """
    plan, out = result.plan, []
    expected = [e.eval_id for e in plan.entries if not e.duplicate] + \
               [e.eval_id for e in plan.speed_entries]
    if sorted(result.results) != sorted(expected):
        missing = sorted(set(expected) - set(result.results))
        extra = sorted(set(result.results) - set(expected))
        out.append(f'records do not match the plan: missing {missing}, unexpected {extra}')
    for eid, r in result.results.items():
        rec = r.record
        if rec['evaluation_category'] not in CATEGORIES:
            out.append(f'{eid}: unknown category {rec["evaluation_category"]}')
        if rec['static_support_status'] not in STATIC_STATUSES:
            out.append(f'{eid}: unknown static-support status')
        if rec['joint_speed_screen'] not in SCREEN_STATUSES:
            out.append(f'{eid}: unknown screen status')
        if rec['static_support_status'] == 'not_applicable' and rec['min_static_margin_m'] \
                is not None:
            out.append(f'{eid}: N/A static support carries a margin value')
    later = {e.eval_id for e in plan.entries if not e.duplicate and e.stage != 'S01'}
    blocked = {eid for eid, r in result.results.items()
               if r.record['evaluation_category'] == 'blocked_gate_failed'}
    if result.status == 'blocked':
        ran_late = [eid for eid in result.evaluated if eid in later]
        if ran_late:
            out.append(f'gate failed but {len(ran_late)} later-stage evaluations ran')
        if blocked != later:
            out.append(f'gate failed but {len(later - blocked)} later-stage records are not '
                       'blocked_gate_failed')
    elif blocked:
        out.append(f'study completed but {len(blocked)} records are blocked')
    if result.status not in STUDY_STATUSES:
        out.append(f'unknown study status {result.status}')
    return out


def category_counts(result):
    counts = {}
    for key, values in (('evaluation_category', CATEGORIES),
                        ('static_support_status', STATIC_STATUSES),
                        ('joint_speed_screen', SCREEN_STATUSES)):
        c = Counter(r.record[key] for r in result.results.values()
                    if r.record['role'] == 'stage')
        counts[key] = {v: c.get(v, 0) for v in values}
    return counts


__all__ = ['Inputs', 'EvalResult', 'GateCheck', 'StudyResult', 'CATEGORIES', 'STATIC_STATUSES',
           'SCREEN_STATUSES', 'load_inputs', 'evaluate_variant', 'gate_checks', 'run_study',
           'verify_result', 'category_counts', 'blocked_record', 'invalid_record']
