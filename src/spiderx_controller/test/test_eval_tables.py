"""colcon test: M5 Batch D - derived tables, hypotheses and the deterministic summary."""
import csv
import json
import os

import pytest

import m5_fixtures as fx
from spiderx_controller import eval_records as rec
from spiderx_controller import eval_runner as er
from spiderx_controller import eval_study as es
from spiderx_controller import eval_tables as et

ARGV = ['m5', '--allow-dirty']


@pytest.fixture(scope='module')
def context():
    return es.load_context(config_dir=fx.SRC_CONFIG)


@pytest.fixture(scope='module')
def evaluator():
    return fx.CachingEvaluator(er.load_inputs(config_dir=fx.SRC_CONFIG))


@pytest.fixture(scope='module')
def provenance(context):
    return rec.collect_provenance(fx.STUDY_YAML, context, allow_dirty=True)


@pytest.fixture(scope='module')
def mini(context, evaluator):
    return er.run_study(fx.plan_of(fx.mini_raw(), context), None, evaluate=evaluator)


@pytest.fixture(scope='module')
def written(mini, provenance, tmp_path_factory):
    out = tmp_path_factory.mktemp('derived')
    a = rec.write_study(mini, str(out / 'a'), provenance, ARGV, [et.write_derived])
    b = rec.write_study(mini, str(out / 'b'), provenance, ARGV, [et.write_derived])
    return a, b


def _csv(d, rel):
    with open(os.path.join(d, rel), newline='') as f:
        return list(csv.DictReader(f))


# ------------------------------------------------------------ files, hashing, determinism
def test_every_derived_file_is_written_hashed_and_deterministic(written):
    (a, ma), (b, mb) = written
    derived = [p for p in rec.compared_files(a) if p.startswith('derived/')]
    assert len(derived) == len(et.DERIVED) + 3          # + hypotheses, summary.json, summary.md
    assert set(derived) <= set(ma['files_sha256'])
    for rel in rec.compared_files(a):
        assert open(os.path.join(a, rel), 'rb').read() == open(os.path.join(b, rel), 'rb').read()
    assert ma['files_sha256'] == mb['files_sha256']


def test_accounting_table_reports_pairs_and_configurations(written):
    (a, _), _ = written
    rows = {r['stage']: r for r in _csv(a, 'derived/accounting.csv')}
    assert rows['S3'] == {'stage': 'S3', 'planned': '2', 'duplicates_of_earlier': '2',
                          'new_unique': '0', 'new_configurations': '0'}
    assert rows['Stages 0-4']['new_unique'] == '11'
    assert rows['Stages 0-4']['new_configurations'] == '7'
    assert rows['speed checks (separate)']['planned'] == '4'


# ------------------------------------------------------------ baseline, negative controls, N/A
def test_baseline_reproduction_and_na_cells(written):
    (a, _), _ = written
    rows = _csv(a, 'derived/baseline_reproduction.csv')
    assert len(rows) == 8                                # 4 gaits x 2 resolutions, none dropped
    at_ref = [r for r in rows if r['samples_per_cycle'] == '200']
    assert all(r['reproduced_at_reference'] == 'true' for r in at_ref)
    for r in rows:
        if r['gait'] in ('pace', 'trot'):
            assert r['min_static_margin_m'] == 'N/A' and r['static_support_status'] == \
                'not_applicable'
            assert r['min_static_margin_m'] != '0'
    trot40 = next(r for r in rows if r['gait'] == 'trot' and r['samples_per_cycle'] == '40')
    assert trot40['evaluation_category'] == 'kinematic_check_fail'   # kept, not filtered


def test_negative_control_table(written):
    (a, _), _ = written
    rows = _csv(a, 'derived/negative_controls.csv')
    assert [(r['gait'], r['samples_per_cycle']) for r in rows] == [('tripod_crawl', '40'),
                                                                  ('tripod_crawl', '200')]
    assert all(r['reproduced_static_support_fail'] == 'true' and
               r['binding_support'] == 'LF|RF|RR' for r in rows)


def test_resolution_table_keeps_sample_count_sensitivity_explicit(written):
    (a, _), _ = written
    rows = _csv(a, 'derived/resolution.csv')
    k = next(r for r in rows if r['gait'] == 'trot' and r['metric'] == 'k_rad_per_m')
    assert float(k['n40']) != float(k['n200']) and k['max_rel_change_vs_finest'] != ''
    pace = next(r for r in rows if r['gait'] == 'pace' and r['metric'] == 'min_static_margin_m')
    assert pace['n40'] == pace['n200'] == 'N/A'


# ------------------------------------------------------------ stage tables
def test_stage2_and_feasibility_boundary(written):
    (a, _), _ = written
    rows = _csv(a, 'derived/stage2_kinematic.csv')
    assert [r['evaluation_category'] for r in rows] == ['valid_pass', 'ik_infeasible']
    fb = _csv(a, 'derived/feasibility_boundary.csv')
    assert fb == [{'duty_factor': '0.5', 'step_height_m': '0.015', 'evaluable': 'true',
                   'largest_feasible_step_length_m': '0.04', 'feasible_levels_contiguous': 'true',
                   'first_infeasible_step_length_m': '0.3',
                   'first_infeasible_category': 'ik_infeasible',
                   'reached_largest_tested_L': 'false'}]


def test_stage3_support_and_pattern_invariance(written):
    (a, _), _ = written
    rows = {r['pattern']: r for r in _csv(a, 'derived/stage3_support.csv')}
    assert rows['diagonal_pairs']['static_support_status'] == 'not_applicable'
    assert rows['diagonal_pairs']['min_static_margin_m'] == 'N/A'
    assert rows['diagonal_pairs']['duplicate_of'] == 'S01-trot-n200'
    inv = _csv(a, 'derived/pattern_invariance.csv')
    assert inv[0]['patterns'] == 'lateral_sequence|diagonal_pairs'
    assert inv[0]['per_leg_metrics_equal'] == 'true'


def test_stage4_is_labelled_a_mechanism_check_not_sway(written):
    (a, _), _ = written
    rows = _csv(a, 'derived/stage4_translation.csv')
    assert len(rows) == 2
    assert all(r['title'] == 'stance-centre translation sensitivity' for r in rows)
    assert all('not body sway' in r['interpretation'] for r in rows)
    zero = next(r for r in rows if r['stance_center_y_offset_m'] == '0')
    assert float(zero['margin_change_vs_zero_offset_m']) == 0.0
    assert zero['duplicate_of'] == 'S01-tripod_crawl-n200'


def test_metric_labels_state_the_claim_limits(written):
    (a, _), _ = written
    rows = {r['metric']: r for r in _csv(a, 'derived/metric_labels.csv')}
    assert rows['lift_work_proxy_j_per_m']['label'] == 'heuristic motion proxy'
    assert 'energy consumption' in rows['lift_work_proxy_j_per_m']['cannot_support']
    assert rows['joint_speed_screen']['label'] == 'provisional secondary screening flag'
    assert 'actuator limit' in rows['joint_speed_screen']['cannot_support']
    assert rows['static_support_status']['label'] == 'static-support approximation'


def test_summary_is_deterministic_and_makes_no_recommendation(written, mini):
    (a, _), _ = written
    md = open(os.path.join(a, 'derived/summary.md')).read()
    assert md == et.summary_markdown(mini)
    assert 'No best gait is recommended' in md and '**N/A**' in md
    assert 'provisional' in md
    s = json.load(open(os.path.join(a, 'derived/summary.json')))
    assert s['status'] == 'completed' and s['not_produced'].startswith('no figures')


# ------------------------------------------------------------ hypotheses (synthetic results)
def _synthetic(context, k_of, feasible, static_of):
    """A 'completed' result over the SHIPPED plan with fabricated records (no evaluation)."""
    study, _ = es.load_study(fx.STUDY_YAML, context=context)
    plan = es.build_plan(study, context)
    results = {}
    for e in plan.entries + plan.speed_entries:
        if e.eval_id in results:
            continue
        v = plan.variants[e.eval_id]
        r = er.blocked_record(v, [e])
        beta, L, h = r['duty_factor'], r['step_length_m'], r['step_height_m']
        ok = feasible(beta, L, h)
        status, binding = static_of(beta, L, h) if ok else ('not_evaluated', None)
        r.update(evaluation_status='evaluated',
                 evaluation_category='valid_pass' if ok else 'ik_infeasible',
                 ik_status='feasible' if ok else 'infeasible',
                 k_rad_per_m=k_of(beta, L, h) if ok else None,
                 static_support_status=status, binding_support=binding)
        results[e.eval_id] = er.EvalResult(r)
    return er.StudyResult(plan, results, [], 'completed', list(results))


def _ok_static(beta, L, h):
    return ('fail', 'LF|RF|RR') if beta >= 0.75 else ('fail', 'LF|LR|RR')


def test_hypotheses_supported_on_a_consistent_synthetic_result(context):
    res = _synthetic(context, k_of=lambda b, L, h: 100 * b + 1000 * h,
                     feasible=lambda b, L, h: L <= 0.06 + 0.1 * (b - 0.5) - h,
                     static_of=_ok_static)
    hyp = et.hypotheses(res)
    assert hyp['H1']['outcome'] == 'supported' and hyp['H1']['evaluable_configurations'] > 0
    assert hyp['H2']['outcome'] == 'supported' and hyp['H2']['pairs_compared'] > 0
    assert hyp['H3']['outcome'] == 'supported'


def test_hypotheses_are_falsified_by_counter_examples(context):
    res = _synthetic(context, k_of=lambda b, L, h: 100 * (1 - b),          # decreasing in beta
                     feasible=lambda b, L, h: L <= (0.12 if b < 0.6 else 0.02),  # drops > 1 level
                     static_of=lambda b, L, h: ('pass', 'LF|RF|RR') if L == 0.02 else _ok_static(
                         b, L, h))
    hyp = et.hypotheses(res)
    assert hyp['H1']['outcome'] == 'falsified' and hyp['H1']['exceptions']
    assert hyp['H2']['outcome'] == 'falsified' and hyp['H2']['beta_reversals']
    assert hyp['H3']['outcome'] == 'falsified'


def test_single_level_hypotheses_are_not_evaluable(written):
    (a, _), _ = written
    hyp = json.load(open(os.path.join(a, 'derived/hypotheses.json')))
    assert hyp['H1']['outcome'] == 'not_evaluable'      # mini study has no beta 0.75/0.85 LS rows
    assert hyp['H2']['outcome'] == 'not_evaluable'      # one beta, one h: nothing to compare
    assert hyp['H3']['outcome'] == 'not_evaluable'


# ------------------------------------------------------------ blocked study
def test_blocked_study_tables_keep_blocked_rows(context, evaluator, provenance, tmp_path):
    raw = fx.mini_raw()
    raw['baseline']['expected']['tripod_crawl']['failed_checks'] = ['static_stability',
                                                                    'joint_speed']
    result = er.run_study(fx.plan_of(raw, context), None, evaluate=evaluator)
    d, _ = rec.write_study(result, str(tmp_path), provenance, ARGV, [et.write_derived])
    rows = _csv(d, 'derived/stage2_kinematic.csv')
    assert [r['evaluation_category'] for r in rows] == ['blocked_gate_failed'] * 2
    hyp = json.load(open(os.path.join(d, 'derived/hypotheses.json')))
    assert all(hyp[h]['outcome'] == 'not_evaluable' for h in ('H1', 'H2', 'H3'))
    s = json.load(open(os.path.join(d, 'derived/summary.json')))
    assert s['status'] == 'blocked' and s['failed_gate_checks'] == ['baseline_reproduction_n200']
