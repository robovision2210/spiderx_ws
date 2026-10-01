"""M5 OFFLINE study derived tables and summary (machine-readable; no figures, no ranking).

Written to <study dir>/derived/ by write_derived (an extra writer of eval_records.write_study, so
every derived file is hashed in the manifest):

    accounting.csv            planned / duplicates / new unique / new configurations per stage
    gate.csv                  the Stage 0/1 gate checks
    baseline_reproduction.csv six M4.5 gaits x resolutions vs the pinned M4.5 results
    negative_controls.csv     wave / tripod_crawl static-support failures at every resolution
    resolution.csv            metric x gait x n, relative change vs the finest n, 200->400 band
    stage2_kinematic.csv      kinematic block, one row per planned Stage 2 entry
    feasibility_boundary.csv  (beta, h) -> largest IK-feasible tested L
    stage3_support.csv        support block, one row per planned Stage 3 entry
    pattern_invariance.csv    per-leg metrics across patterns at equal (beta, L, h)
    stage4_translation.csv    stance-centre translation sensitivity (mechanism check)
    metric_labels.csv         honesty label and claim limits of every reported metric
    hypotheses.json           H1-H3 with their pre-registered criteria (plan section 4)
    summary.json, summary.md  counts, gate, accounting, hypotheses, scope - deterministic

Rows are NEVER filtered: failing, infeasible, invalid, blocked and N/A outcomes appear in every
table they belong to. Margin cells of not-applicable static support read "N/A" (never 0).
Offline model analysis only; no "best gait" is produced.
"""

import os

from spiderx_controller import eval_records as rec
from spiderx_controller import eval_runner as er
from spiderx_controller import eval_study as es
from spiderx_controller import gait_report as gr

NA = 'N/A'
RESOLUTION_METRICS = ('k_rad_per_m', 'max_joint_speed_rad_s', 'max_joint_step_rad',
                      'joint_travel_rad_per_m', 'lift_work_proxy_j_per_m', 'min_static_margin_m',
                      'support_ge3_fraction', 'fraction_three_feet', 'fraction_four_feet',
                      'min_singularity_margin_rad', 'min_joint_limit_margin_rad')
METRIC_LABELS = (
    ('evaluation_category', 'offline model metric',
     'valid_pass = valid config, sampled-IK feasible at every sample, every M4.5 kinematic check '
     'passed', 'not a stability, walking, dynamic or hardware claim'),
    ('ik_status', 'offline model metric (sampled)', 'IK solved at every u_k, n samples',
     'reachability between samples; real reach'),
    ('static_support_status', 'static-support approximation',
     'min COM-to-support-edge margin over samples with >= 3 stance feet vs 5 mm; N/A without '
     '3-contact support', 'dynamic stability, balance, terrain, slip, tipping'),
    ('min_static_margin_m', 'static-support approximation', 'quasi-static, URDF CAD masses, '
     'point feet, flat ground, level body', 'dynamic stability'),
    ('k_rad_per_m', 'offline model metric (derived)',
     'peak joint-rate demand per metre of travel = max joint speed / v; speed only rescales time '
     'in this clock-driven model', 'measured motor or servo capability'),
    ('joint_speed_screen', 'provisional secondary screening flag',
     'max joint speed <= 0.5 rad/s SIMULATION_PLACEHOLDER', 'actuator limit; validity criterion'),
    ('v_admissible_at_reference_m_s', 'offline model metric (derived)', '0.5 rad/s / K',
     'achievable robot speed'),
    ('min_singularity_margin_rad', 'offline model metric (sampled)', 'analytic IK reach-boundary '
     'distance', 'manipulability, force capacity'),
    ('min_joint_limit_margin_rad', 'offline model metric (sampled)', 'distance to URDF limits '
     'minus 0.05 rad', 'joint safety on hardware'),
    ('max_joint_step_rad', 'offline model metric (sampled, n-dependent)', 'branch-flip detector',
     'motion smoothness'),
    ('foot_path_per_m', 'heuristic motion proxy', 'world-frame swing arc per metre',
     'energy, power, efficiency'),
    ('joint_travel_rad_per_m', 'heuristic motion proxy', 'summed |dq| per metre',
     'energy, actuator work, wear'),
    ('lift_work_proxy_j_per_m', 'heuristic motion proxy', 'sum(m_leg g dz+) per metre (CAD '
     'masses)', 'energy consumption, electrical power, actuator work, efficiency, torque cost, '
     'battery or runtime'),
)


def _f(v):
    return v if isinstance(v, str) else gr.fmt(v)


def _margin_cell(r, key='min_static_margin_m'):
    if r['static_support_status'] == 'not_applicable':
        return NA
    return _f(r[key])


def _records_by_entry(result, stage):
    return [(e, result.results[e.eval_id].record) for e in result.plan.stage_entries(stage)]


def _rel(a, b):
    if a is None or b is None or b == 0:
        return None
    return abs(a - b) / abs(b)


def _value(r, metric):
    if metric in r:
        return r[metric]
    return r[f'm45_{metric}']


# ---------------------------------------------------------------- tables
def accounting_rows(plan):
    a = plan.accounting
    rows = [['stage', 'planned', 'duplicates_of_earlier', 'new_unique', 'new_configurations']]
    for sid in es.STAGES:
        rows.append([sid, a['planned'][sid], a['duplicates_of_earlier'][sid],
                     a['new_unique'][sid], a['new_configurations'][sid]])
    for key, title in (('stages_0_3', 'Stages 0-3'), ('all_stages', 'Stages 0-4')):
        t = a[key]
        rows.append([title, t['planned'], t['planned'] - t['unique'], t['unique'],
                     t['distinct_configurations']])
    rows.append(['speed checks (separate)', a['speed_checks'], 0, a['speed_checks'],
                 a['speed_checks']])
    return [[str(c) for c in r] for r in rows]


def gate_rows(result):
    return [['check_id', 'scope', 'passed', 'detail']] + [
        [g.check_id, g.scope, _f(g.passed), g.detail] for g in result.gate]


def baseline_rows(result):
    b = result.plan.study.raw['baseline']
    ref = b['reference_samples_per_cycle']
    rows = [['gait', 'samples_per_cycle', 'negative_control', 'evaluation_category',
             'm45_verdict', 'm45_failed_checks', 'expected_failed_checks',
             'static_support_status', 'min_static_margin_m', 'expected_min_static_margin_m',
             'm45_fraction_statically_stable', 'k_rad_per_m', 'max_joint_speed_rad_s',
             'joint_speed_screen', 'lift_work_proxy_j_per_m', 'joint_travel_rad_per_m',
             'reproduced_at_reference']]
    for e, r in _records_by_entry(result, 'S01'):
        gait, exp = r['source_gait'], b['expected'][r['source_gait']]
        at_ref = e.n == ref
        reproduced = ''
        if at_ref and r['evaluation_status'] == 'evaluated':
            ok = (r['m45_failed_checks'].split('|') if r['m45_failed_checks'] else []) == \
                exp['failed_checks']
            for m in es.EXPECTED_METRICS:
                got, want = r[f'm45_{m}'], exp[m]
                ok &= (got is None and want is None) or (
                    got is not None and want is not None and abs(got - want) <= b['tolerances'][m])
            reproduced = _f(ok)
        rows.append([gait, str(e.n), _f(gait in b['negative_controls']),
                     r['evaluation_category'], _f(r['m45_verdict']), _f(r['m45_failed_checks']),
                     '|'.join(exp['failed_checks']) if at_ref else '',
                     r['static_support_status'], _margin_cell(r),
                     (NA if exp['min_static_margin_m'] is None else _f(exp['min_static_margin_m']))
                     if at_ref else '',
                     _f(r['m45_fraction_statically_stable']), _f(r['k_rad_per_m']),
                     _f(r['max_joint_speed_rad_s']), r['joint_speed_screen'],
                     _f(r['m45_lift_work_proxy_j_per_m']), _f(r['m45_joint_travel_rad_per_m']),
                     reproduced])
    return rows


def negative_control_rows(result):
    nc = result.plan.study.raw['baseline']['negative_controls']
    rows = [['gait', 'samples_per_cycle', 'static_support_status', 'min_static_margin_m',
             'static_fail_samples_ge3', 'binding_support', 'binding_u', 'm45_failed_checks',
             'reproduced_static_support_fail']]
    for e, r in _records_by_entry(result, 'S01'):
        if r['source_gait'] in nc:
            rows.append([r['source_gait'], str(e.n), r['static_support_status'], _margin_cell(r),
                         _f(r['static_fail_samples_ge3']), _f(r['binding_support']),
                         _f(r['binding_u']), _f(r['m45_failed_checks']),
                         _f(r['static_support_status'] == 'fail')])
    return rows


def resolution_rows(result):
    b = result.plan.study.raw['baseline']
    ns = sorted(b['resolutions'])
    by = {}
    for e, r in _records_by_entry(result, 'S01'):
        by.setdefault(r['source_gait'], {})[e.n] = r
    rows = [['gait', 'metric'] + [f'n{n}' for n in ns] +
            ['max_rel_change_vs_finest', 'rel_change_200_to_400']]
    for gait in b['gaits']:
        for m in RESOLUTION_METRICS:
            vals = [_value(by[gait][n], m) for n in ns]
            finest = vals[-1]
            changes = [c for c in (_rel(v, finest) for v in vals[:-1]) if c is not None]
            band = _rel(_value(by[gait][400], m), _value(by[gait][200], m)) \
                if 200 in by[gait] and 400 in by[gait] else None
            cells = [NA if (v is None and m == 'min_static_margin_m' and
                            by[gait][n]['static_support_status'] == 'not_applicable') else _f(v)
                     for v, n in zip(vals, ns)]
            rows.append([gait, m] + cells + [_f(max(changes) if changes else None), _f(band)])
    return rows


def _band(result, metric='k_rad_per_m'):
    """Stage 1 discretisation band of a metric: max relative change 200 -> 400 over baselines."""
    by = {}
    for e, r in _records_by_entry(result, 'S01'):
        by.setdefault(r['source_gait'], {})[e.n] = r
    bands = [_rel(_value(d[400], metric), _value(d[200], metric))
             for d in by.values() if 200 in d and 400 in d]
    bands = [x for x in bands if x is not None]
    return max(bands) if bands else None


def stage2_rows(result):
    rows = [['label', 'duplicate_of', 'duty_factor', 'step_length_m', 'step_height_m',
             'evaluation_category', 'ik_status', 'kinematic_checks_failed', 'k_rad_per_m',
             'v_admissible_at_reference_m_s', 'joint_speed_screen', 'min_singularity_margin_rad',
             'min_joint_limit_margin_rad', 'max_joint_step_rad', 'foot_path_per_m',
             'joint_travel_rad_per_m', 'lift_work_proxy_j_per_m']]
    for e, r in _records_by_entry(result, 'S2'):
        rows.append([e.label, e.first_label if e.duplicate else '', _f(r['duty_factor']),
                     _f(r['step_length_m']), _f(r['step_height_m']), r['evaluation_category'],
                     r['ik_status'], _f(r['kinematic_checks_failed']), _f(r['k_rad_per_m']),
                     _f(r['v_admissible_at_reference_m_s']), r['joint_speed_screen'],
                     _f(r['m45_min_singularity_margin_rad'] if r['ik_status'] == 'feasible'
                        else None),
                     _f(r['m45_min_joint_limit_margin_rad'] if r['ik_status'] == 'feasible'
                        else None),
                     _f(r['m45_max_joint_step_rad'] if r['ik_status'] == 'feasible' else None),
                     _f(r['m45_foot_path_per_m'] if r['evaluation_status'] == 'evaluated'
                        else None),
                     _f(r['m45_joint_travel_rad_per_m']), _f(r['m45_lift_work_proxy_j_per_m'])])
    return rows


def feasibility_boundary(result):
    """{(beta, h): dict} from Stage 2: largest IK-feasible tested L and the first failure."""
    s2 = result.plan.study.raw['stages']['S2']
    Ls = sorted(s2['step_lengths_m'])
    cells = {}
    for _, r in _records_by_entry(result, 'S2'):
        cells.setdefault((r['duty_factor'], r['step_height_m']), {})[r['step_length_m']] = r
    out = {}
    for key in sorted(cells):
        by_L = cells[key]
        status = [by_L[L]['ik_status'] for L in Ls]
        if any(s == 'not_evaluated' for s in status):
            out[key] = {'evaluable': False}
            continue
        feasible = [L for L in Ls if by_L[L]['ik_status'] == 'feasible']
        first_bad = next((L for L in Ls if by_L[L]['ik_status'] != 'feasible'), None)
        idx = max((Ls.index(L) for L in feasible), default=-1)
        out[key] = {'evaluable': True, 'L_max': Ls[idx] if idx >= 0 else None, 'L_index': idx,
                    'contiguous': feasible == Ls[:len(feasible)],
                    'first_infeasible_L': first_bad,
                    'first_infeasible_category': by_L[first_bad]['evaluation_category']
                    if first_bad is not None else None,
                    'reached_largest_tested_L': idx == len(Ls) - 1}
    return out


def feasibility_rows(result):
    rows = [['duty_factor', 'step_height_m', 'evaluable', 'largest_feasible_step_length_m',
             'feasible_levels_contiguous', 'first_infeasible_step_length_m',
             'first_infeasible_category', 'reached_largest_tested_L']]
    for (beta, h), c in feasibility_boundary(result).items():
        rows.append([_f(beta), _f(h), _f(c['evaluable'])] + (
            [_f(c['L_max']), _f(c['contiguous']), _f(c['first_infeasible_L']),
             _f(c['first_infeasible_category']), _f(c['reached_largest_tested_L'])]
            if c['evaluable'] else ['', '', '', '', '']))
    return rows


def stage3_rows(result):
    rows = [['label', 'duplicate_of', 'pattern', 'duty_factor', 'step_length_m',
             'evaluation_category', 'requires_static_stability_m45_policy',
             'min_support_feet', 'support_ge3_fraction', 'static_support_status',
             'min_static_margin_m', 'fraction_margin_pos_given_ge3',
             'fraction_margin_ge_threshold_given_ge3', 'binding_support']]
    for e, r in _records_by_entry(result, 'S3'):
        pattern = e.label.split('-')[1]
        rows.append([e.label, e.first_label if e.duplicate else '', pattern,
                     _f(r['duty_factor']), _f(r['step_length_m']), r['evaluation_category'],
                     _f(r['requires_static_stability_m45_policy']), _f(r['m45_min_support_feet']),
                     _f(r['support_ge3_fraction']), r['static_support_status'], _margin_cell(r),
                     _margin_cell(r, 'fraction_margin_pos_given_ge3'),
                     _margin_cell(r, 'fraction_margin_ge_threshold_given_ge3'),
                     NA if r['static_support_status'] == 'not_applicable'
                     else _f(r['binding_support'])])
    return rows


def pattern_invariance_rows(result):
    groups = {}
    for e, r in _records_by_entry(result, 'S3'):
        groups.setdefault((r['duty_factor'], r['step_length_m'], r['step_height_m']),
                          []).append((e.label.split('-')[1], r))
    rows = [['duty_factor', 'step_length_m', 'patterns', 'evaluable', 'max_rel_diff',
             'per_leg_metrics_equal']]
    for key in sorted(groups):
        items = groups[key]
        ok = all(r['evaluation_status'] == 'evaluated' and r['ik_status'] == 'feasible'
                 for _, r in items)
        worst = None
        if ok:
            worst = 0.0
            for m in er.PER_LEG_METRICS:
                vals = [_value(r, m) for _, r in items]
                ref = vals[0]
                for v in vals[1:]:
                    d = _rel(v, ref) if ref else abs(v - ref)
                    worst = max(worst, d or 0.0)
        rows.append([_f(key[0]), _f(key[1]), '|'.join(p for p, _ in items), _f(ok), _f(worst),
                     _f(worst is not None and worst <= 1e-9) if ok else ''])
    return rows


def stage4_rows(result):
    s4 = result.plan.study.raw['stages']['S4']
    recs = _records_by_entry(result, 'S4')
    zero = {r['duty_factor']: r for _, r in recs if r['stance_center_y_m'] == 0.0}
    rows = [['label', 'duplicate_of', 'title', 'duty_factor', 'stance_center_y_offset_m',
             'evaluation_category', 'static_support_status', 'min_static_margin_m',
             'margin_change_vs_zero_offset_m', 'fraction_margin_ge_threshold_given_ge3',
             'binding_support', 'interpretation']]
    for e, r in recs:
        z = zero.get(r['duty_factor'])
        delta = (r['min_static_margin_m'] - z['min_static_margin_m']
                 if z and r['min_static_margin_m'] is not None and
                 z['min_static_margin_m'] is not None else None)
        rows.append([e.label, e.first_label if e.duplicate else '', s4['title'],
                     _f(r['duty_factor']), _f(r['stance_center_y_m']), r['evaluation_category'],
                     r['static_support_status'], _margin_cell(r), _f(delta),
                     _margin_cell(r, 'fraction_margin_ge_threshold_given_ge3'),
                     _f(r['binding_support']),
                     'constant translation; mechanism check only - not body sway, not a remedy'])
    return rows


def metric_label_rows():
    return [['metric', 'label', 'definition', 'cannot_support']] + [list(r) for r in METRIC_LABELS]


# ---------------------------------------------------------------- hypotheses (plan section 4)
def hypotheses(result):
    out = {'note': 'pre-registered in docs/M5_EVALUATION_PLAN.md section 4; evaluated '
                   'mechanically; model predictions only'}
    if result.status != 'completed':
        for h in ('H1', 'H2', 'H3'):
            out[h] = {'outcome': 'not_evaluable', 'reason': 'study blocked by the Stage 0/1 gate'}
        return out
    out['H1'] = _h1(result)
    out['H2'] = _h2(result)
    out['H3'] = _h3(result)
    return out


def _h1(result):
    seen, rows = set(), []
    for stage in ('S2', 'S3'):
        for e, r in _records_by_entry(result, stage):
            if (e.eval_id in seen or r['pattern'] != 'lateral_sequence'
                    or r['duty_factor'] not in (0.75, 0.85) or r['stance_center_y_m'] != 0.0):
                continue
            seen.add(e.eval_id)
            rows.append(r)
    evaluable = [r for r in rows if r['static_support_status'] in ('pass', 'fail')]
    exceptions = [f'{r["labels"].split("|")[0]}: {r["static_support_status"]}, binding '
                  f'{r["binding_support"]}' for r in evaluable
                  if r['static_support_status'] != 'fail' or r['binding_support'] != 'LF|RF|RR']
    outcome = ('not_evaluable' if not evaluable else 'falsified' if exceptions else 'supported')
    return {'statement': 'LS, zero stance-centre offset, beta in {0.75, 0.85}: min >= 3-contact '
                         'static margin < 5 mm for every tested L, binding triangle LF|RF|RR',
            'outcome': outcome, 'evaluable_configurations': len(evaluable),
            'not_evaluable_configurations': len(rows) - len(evaluable),
            'exceptions': exceptions}


def _pairs(values):
    return list(zip(values, values[1:]))


def _h2(result):
    band = _band(result) or 0.0
    s2 = result.plan.study.raw['stages']['S2']
    betas, Ls, hs = (sorted(s2['duty_factors']), sorted(s2['step_lengths_m']),
                     sorted(s2['step_heights_m']))
    k = {(r['duty_factor'], r['step_length_m'], r['step_height_m']): r['k_rad_per_m']
         for _, r in _records_by_entry(result, 'S2') if r['ik_status'] == 'feasible'}
    reversals, within, cells, pairs = [], [], 0, 0
    for L in Ls:
        for h in hs:
            vals = [k.get((b, L, h)) for b in betas]
            if any(v is None for v in vals):
                continue
            cells += 1
            for (b0, v0), (b1, v1) in _pairs(list(zip(betas, vals))):
                pairs += 1
                if v1 > v0:
                    continue
                (reversals if v1 < v0 * (1 - band) else within).append(
                    f'L={L} h={h}: K(beta {b1})={v1:.6g} <= K(beta {b0})={v0:.6g}')
    h_reversals, h_cells = [], 0
    for b in betas:
        for L in Ls:
            vals = [(h, k.get((b, L, h))) for h in hs if (b, L, h) in k]
            if len(vals) < 2:
                continue
            h_cells += 1
            for (h0, v0), (h1, v1) in _pairs(vals):
                pairs += 1
                if v1 < v0 * (1 - band):
                    h_reversals.append(f'beta={b} L={L}: K(h {h1})={v1:.6g} < K(h {h0})={v0:.6g}')
    outcome = ('not_evaluable' if not pairs else
               'falsified' if reversals or h_reversals else
               'resolution_limited' if within else 'supported')
    return {'statement': 'K increases strictly with beta at fixed (L, h) and does not decrease '
                         'with h at fixed (beta, L)',
            'outcome': outcome, 'discretisation_band_rel': band,
            'beta_cells_evaluated': cells, 'h_cells_evaluated': h_cells, 'pairs_compared': pairs,
            'beta_reversals': reversals, 'beta_within_band': within,
            'h_reversals': h_reversals}


def _h3(result):
    fb = feasibility_boundary(result)
    s2 = result.plan.study.raw['stages']['S2']
    betas, hs = sorted(s2['duty_factors']), sorted(s2['step_heights_m'])
    idx = {k: c['L_index'] for k, c in fb.items() if c['evaluable']}
    viol_far, viol_near, pairs = [], [], 0
    for b in betas:                                   # non-increasing in h
        for (h0, h1) in _pairs(hs):
            if (b, h0) in idx and (b, h1) in idx:
                pairs += 1
            if (b, h0) in idx and (b, h1) in idx and idx[(b, h1)] > idx[(b, h0)]:
                (viol_far if idx[(b, h1)] - idx[(b, h0)] > 1 else viol_near).append(
                    f'beta={b}: L_max rises from h={h0} to h={h1}')
    for h in hs:                                      # non-decreasing in beta
        for (b0, b1) in _pairs(betas):
            if (b0, h) in idx and (b1, h) in idx:
                pairs += 1
            if (b0, h) in idx and (b1, h) in idx and idx[(b1, h)] < idx[(b0, h)]:
                (viol_far if idx[(b0, h)] - idx[(b1, h)] > 1 else viol_near).append(
                    f'h={h}: L_max falls from beta={b0} to beta={b1}')
    reached = sum(1 for c in fb.values() if c.get('reached_largest_tested_L'))
    outcome = ('not_evaluable' if not pairs else 'falsified' if viol_far else
               'resolution_limited' if viol_near else 'supported')
    return {'statement': 'largest IK-feasible tested L is non-increasing in h and non-decreasing '
                         'in beta (violations of more than one L level falsify)',
            'outcome': outcome, 'cells': len(idx), 'pairs_compared': pairs,
            'cells_where_largest_tested_L_is_feasible': reached,
            'violations_more_than_one_level': viol_far, 'violations_one_level': viol_near}


# ---------------------------------------------------------------- summary
def summary(result):
    plan = result.plan
    return {'study_id': plan.study.study_id, 'status': result.status,
            'gate_passed': result.gate_passed,
            'failed_gate_checks': [g.check_id for g in result.gate if not g.passed],
            'accounting': plan.accounting,
            'category_counts_stage_records': er.category_counts(result),
            'hypotheses': {h: v['outcome'] for h, v in hypotheses(result).items()
                           if h.startswith('H')},
            'negative_controls': plan.study.raw['baseline']['negative_controls'],
            'scope': rec.SCOPE,
            'not_produced': 'no figures, no optimisation, no recommended or best gait'}


def summary_markdown(result):
    s = summary(result)
    acc = s['accounting']
    lines = [f'# M5 offline evaluation study - {s["study_id"]}', '',
             '> ' + rec.SCOPE, '',
             f'**Study status:** {s["status"]} (Stage 0/1 gate '
             f'{"passed" if s["gate_passed"] else "FAILED: " + ", ".join(s["failed_gate_checks"])}).',
             '', '## Accounting', '',
             '| Stage | Planned | Duplicates of earlier | New unique |', '|---|---|---|---|']
    for sid in es.STAGES:
        lines.append(f'| {sid} | {acc["planned"][sid]} | {acc["duplicates_of_earlier"][sid]} | '
                     f'{acc["new_unique"][sid]} |')
    for key, title in (('stages_0_3', 'Stages 0-3'), ('all_stages', 'Stages 0-4')):
        t = acc[key]
        lines.append(f'| {title} | {t["planned"]} | {t["planned"] - t["unique"]} | '
                     f'{t["unique"]} ({t["distinct_configurations"]} distinct configurations) |')
    lines += [f'| Speed checks (separate) | {acc["speed_checks"]} | 0 | {acc["speed_checks"]} |',
              '', '## Stage 0/1 gate', '', '| Check | Result | Detail |', '|---|---|---|']
    for g in result.gate:
        lines.append(f'| {g.check_id} | {"PASS" if g.passed else "FAIL"} | {g.detail} |')
    lines += ['', '## Result categories (stage records)', '']
    for key, counts in s['category_counts_stage_records'].items():
        lines.append(f'- **{key}:** ' + ', '.join(f'{k} {v}' for k, v in counts.items()))
    lines += ['', '## Pre-registered hypotheses (model predictions only)', '']
    for h, outcome in s['hypotheses'].items():
        lines.append(f'- **{h}:** {outcome}')
    lines += ['', '## Negative controls', '',
              f'{", ".join(s["negative_controls"])} are kept in every baseline table; see '
              '`negative_controls.csv`.', '',
              'Static support for configurations without three-contact support is **N/A** '
              '(not zero, not a failure). The 0.5 rad/s joint-speed screen is a provisional '
              'secondary flag, not an actuator limit. No best gait is recommended.', '']
    return '\n'.join(lines)


# ---------------------------------------------------------------- writer
DERIVED = {
    'derived/accounting.csv': lambda r: accounting_rows(r.plan),
    'derived/gate.csv': gate_rows,
    'derived/baseline_reproduction.csv': baseline_rows,
    'derived/negative_controls.csv': negative_control_rows,
    'derived/resolution.csv': resolution_rows,
    'derived/stage2_kinematic.csv': stage2_rows,
    'derived/feasibility_boundary.csv': feasibility_rows,
    'derived/stage3_support.csv': stage3_rows,
    'derived/pattern_invariance.csv': pattern_invariance_rows,
    'derived/stage4_translation.csv': stage4_rows,
    'derived/metric_labels.csv': lambda r: metric_label_rows(),
}


def write_derived(result, d):
    """Extra writer for eval_records.write_study: every derived table and the summary."""
    os.makedirs(os.path.join(d, 'derived'), exist_ok=True)
    for rel, rows in DERIVED.items():
        gr.write_csv(os.path.join(d, rel), rows(result))
    gr.write_json(os.path.join(d, 'derived/hypotheses.json'), hypotheses(result))
    gr.write_json(os.path.join(d, 'derived/summary.json'), summary(result))
    with open(os.path.join(d, 'derived/summary.md'), 'w') as f:
        f.write(summary_markdown(result))


__all__ = ['write_derived', 'hypotheses', 'summary', 'summary_markdown', 'feasibility_boundary',
           'DERIVED', 'NA']
