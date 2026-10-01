"""colcon test: M5 Batch B - runner, result categories, the Stage 0/1 gate and its enforcement."""
import copy
import os

import pytest

import m5_fixtures as fx
from spiderx_controller import eval_runner as er
from spiderx_controller import eval_study as es
from spiderx_controller import m5_offline_evaluation as cli


@pytest.fixture(scope='module')
def context():
    return es.load_context(config_dir=fx.SRC_CONFIG)


@pytest.fixture(scope='module')
def evaluator():
    return fx.CachingEvaluator(er.load_inputs(config_dir=fx.SRC_CONFIG))


@pytest.fixture(scope='module')
def mini(context, evaluator):
    """A real mini study (gate expected to pass), evaluated once."""
    plan = fx.plan_of(fx.mini_raw(), context)
    return er.run_study(plan, None, evaluate=evaluator)


def _rec(result, label):
    return next(r.record for r in result.results.values() if label in r.record['labels'].split('|'))


def _blocked_plan(context, edit):
    raw = fx.mini_raw()
    edit(raw)
    return fx.plan_of(raw, context)


# ------------------------------------------------------------ gate passes on the real mini study
def test_mini_study_completes_with_every_gate_check_passed(mini):
    assert mini.status == 'completed' and mini.gate_passed
    assert [g.check_id for g in mini.gate] == [
        'baseline_valid_and_ik_feasible', 'baseline_reproduction_n200',
        'negative_controls_reproduced', 'static_support_na_preserved',
        'm45_policy_matches_shipped_flags', 'speed_scale_invariance', 'pattern_separation']
    assert er.verify_result(mini) == []


def test_one_record_per_unique_evaluation_plus_speed_checks(mini):
    acc = mini.plan.accounting
    assert len(mini.results) == acc['all_stages']['unique'] + acc['speed_checks'] == 11 + 4
    shared = _rec(mini, 'S01-trot-n200')
    assert shared['stages'] == 'S01|S3'                 # one evaluation, both stage memberships
    assert 'S3-diagonal_pairs-b0.5-L40-h15' in shared['labels']


def test_speed_scale_gate_evidence(mini):
    g = next(c for c in mini.gate if c.check_id == 'speed_scale_invariance')
    assert g.passed and 'ratio 2..2' in g.detail
    slow, fast = _rec(mini, 'S01-tripod_crawl-n200'), _rec(mini, 'SPEED-tripod_crawl-x2-n200')
    assert fast['max_joint_speed_rad_s'] == pytest.approx(2 * slow['max_joint_speed_rad_s'])
    assert fast['k_rad_per_m'] == pytest.approx(slow['k_rad_per_m'], rel=1e-12)


# ------------------------------------------------------------ categories (never filtered)
def test_ik_infeasible_point_is_kept_with_reasons(mini):
    rec = _rec(mini, 'S2-lateral_sequence-b0.5-L300-h15')
    assert rec['evaluation_category'] == 'ik_infeasible' and rec['ik_status'] == 'infeasible'
    assert rec['ik_failed_samples'] > 0
    assert rec['static_support_status'] == 'not_evaluated'
    assert rec['k_rad_per_m'] is None and rec['joint_speed_screen'] == 'not_evaluated'
    reasons = next(r.ik_reasons for r in mini.results.values() if r.record is rec)
    assert any(r.startswith('unreachable') or r == 'joint_limits'
               for leg in reasons.values() for r in leg)


def test_pace_and_trot_static_support_is_not_applicable_never_zero(mini):
    for gait in ('pace', 'trot'):
        for n in (40, 200):
            rec = _rec(mini, f'S01-{gait}-n{n}')
            assert rec['static_support_status'] == 'not_applicable'
            assert rec['min_static_margin_m'] is None
            assert rec['fraction_margin_pos_given_ge3'] is None
            assert rec['support_ge3_fraction'] == 0.0
            assert rec['m45_fraction_statically_stable'] == 0.0   # legacy M4.5 field, verbatim


def test_negative_control_fails_static_support_at_every_resolution(mini):
    for n in (40, 200):
        rec = _rec(mini, f'S01-tripod_crawl-n{n}')
        assert rec['static_support_status'] == 'fail'
        assert rec['binding_support'] == 'LF|RF|RR'
        assert rec['min_static_margin_m'] == pytest.approx(-0.004073, abs=2e-6)


def test_k_is_continuous_and_the_screen_is_separate_from_the_category(mini):
    fast = _rec(mini, 'SPEED-tripod_crawl-x2-n200')
    assert fast['joint_speed_screen'] == 'fail'               # 0.61 rad/s > 0.5 placeholder
    assert fast['evaluation_category'] == 'valid_pass'        # the screen never sets the category
    assert fast['k_rad_per_m'] == pytest.approx(fast['max_joint_speed_rad_s'] /
                                                fast['body_speed_m_s'])
    assert fast['v_admissible_at_reference_m_s'] == pytest.approx(0.5 / fast['k_rad_per_m'])
    slow = _rec(mini, 'S01-tripod_crawl-n200')
    assert slow['joint_speed_screen'] == 'pass' and slow['m45_verdict'] == 'FAIL'


def test_resolution_dependent_kinematic_check_is_its_own_category(mini):
    rec = _rec(mini, 'S01-trot-n40')                          # joint_continuity at n=40 (M4.5 doc)
    assert rec['evaluation_category'] == 'kinematic_check_fail'
    assert rec['kinematic_checks_failed'] == 'joint_continuity'
    assert _rec(mini, 'S01-trot-n200')['evaluation_category'] == 'valid_pass'


def test_invalid_configuration_is_a_record_not_an_exception(context):
    v = es.make_variant(context, source_gait='trot', pattern='diagonal_pairs', n=200,
                        duty_factor=0.5, phase_offsets={'front_left': 0.0, 'front_right': 0.5,
                                                        'rear_left': 0.5, 'rear_right': 0.0},
                        swing_order=[['front_left', 'rear_right'], ['front_right', 'rear_left']],
                        step_length_m=0.04, step_height_m=-0.01, stance_height_offset_m=0.0,
                        stance_center_offset_m=[0.0, 0.0], body_speed_m_s=0.005,
                        swing_profile='cycloid')
    r = er.evaluate_variant(v, [], None)
    assert r.record['evaluation_category'] == 'invalid_configuration'
    assert 'step_height_m' in r.record['config_error']
    assert r.record['static_support_status'] == 'not_evaluated'


# ------------------------------------------------------------ forced gate failures block Stages 2-4
def test_forced_gate_failure_blocks_later_stages(context, evaluator):
    plan = _blocked_plan(context, lambda r: r['baseline']['expected']['tripod_crawl'].update(
        failed_checks=['static_stability', 'joint_speed']))
    evaluator.calls.clear()
    result = er.run_study(plan, None, evaluate=evaluator)
    assert result.status == 'blocked' and not result.gate_passed
    failed = [g.check_id for g in result.gate if not g.passed]
    assert failed == ['baseline_reproduction_n200']
    later = {e.eval_id for e in plan.entries if not e.duplicate and e.stage != 'S01'}
    assert not later & set(evaluator.calls)                   # never evaluated
    for eid in later:
        rec = result.results[eid].record
        assert rec['evaluation_category'] == rec['evaluation_status'] == 'blocked_gate_failed'
    assert er.verify_result(result) == []                     # evidence preserved, consistent


def test_perturbed_speed_evidence_fails_the_speed_gate(context, evaluator):
    plan = fx.plan_of(fx.mini_raw(), context)

    def perturbed(variant, entries, inputs=None):
        r = evaluator(variant, entries)
        if variant.role == 'speed_check' and r.q_series:
            leg = 'front_left'
            r.q_series[leg][3] = tuple(x + 1e-6 for x in r.q_series[leg][3])
        return r
    result = er.run_study(plan, None, evaluate=perturbed)
    g = next(c for c in result.gate if c.check_id == 'speed_scale_invariance')
    assert not g.passed and 'joint series differ' in g.detail
    assert result.status == 'blocked'


def test_real_wave_phase_boundary_artifact_fails_the_support_invariance_check(context, evaluator):
    """FINDING (M5 Batch B): in M4.5's gait_phase.leg_phase, wave's rear-left touch-down sample
    (u = 0.15) has theta = 0.15 < 1 - 0.85 = 0.15000000000000002 and is labelled swing. Wave's
    three-foot fraction is therefore 0.6 + 1/n, not 0.6, so the pre-registered resolution-
    invariance check of the support fractions (plan section 6.3) fails and the gate blocks."""
    raw = fx.mini_raw(gaits=('wave', 'pace', 'trot'), negative=('wave',))
    result = er.run_study(fx.plan_of(raw, context), None, evaluate=evaluator)
    g = next(c for c in result.gate if c.check_id == 'pattern_separation')
    assert not g.passed and 'wave: fraction_three_feet varies with n: [0.625, 0.605]' in g.detail
    assert [c.check_id for c in result.gate if not c.passed] == ['pattern_separation']
    assert result.status == 'blocked'


# ------------------------------------------------------------ mutation tests
def test_mutation_removing_gate_enforcement_is_detected(context, evaluator, monkeypatch):
    plan = _blocked_plan(context, lambda r: r['baseline']['expected']['tripod_crawl'].update(
        failed_checks=['static_stability', 'joint_speed']))
    monkeypatch.setattr(er, '_gate_allows_later_stages', lambda passed: True)
    evaluator.calls.clear()
    result = er.run_study(plan, None, evaluate=evaluator)
    later = {e.eval_id for e in plan.entries if not e.duplicate and e.stage != 'S01'}
    assert later & set(evaluator.calls)                       # the mutant evaluated them...
    violations = er.verify_result(result)
    assert any('later-stage evaluations ran' in v for v in violations)   # ...and it is caught


@pytest.mark.parametrize('mutate,match', [
    (lambda res: res.results.pop(next(iter(res.results))), 'missing'),
    (lambda res: next(iter(res.results.values())).record.update(evaluation_category='filtered'),
     'unknown category'),
    (lambda res: next(r for r in res.results.values()
                      if r.record['static_support_status'] == 'not_applicable')
     .record.update(min_static_margin_m=0.0), 'N/A static support carries a margin'),
    (lambda res: setattr(res, 'status', 'blocked'), 'later-stage'),
])
def test_mutation_filtering_or_rewriting_records_is_detected(mini, mutate, match):
    result = copy.copy(mini)
    result.results = {k: er.EvalResult(dict(v.record), None, v.checks, v.ik_reasons)
                      for k, v in mini.results.items()}
    mutate(result)
    assert any(match in v for v in er.verify_result(result))


# ------------------------------------------------------------ CLI
def test_cli_check_only_reports_the_exact_accounting(capsys):
    assert cli.main(['m5', '--check-only', '--config-dir', fx.SRC_CONFIG,
                     '--study', fx.STUDY_YAML]) == cli.EXIT_OK
    out = capsys.readouterr().out
    assert 'Stages 0-3: 285 planned, 264 unique (configuration, n) pairs, 240 distinct' in out
    assert 'Stages 0-4: 293 planned, 270 unique (configuration, n) pairs, 246 distinct' in out
    assert 'counted separately): 6' in out and 'nothing written' in out


def test_cli_dry_run_lists_every_planned_evaluation_and_writes_nothing(capsys, tmp_path,
                                                                       monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert cli.main(['m5', '--dry-run', '--config-dir', fx.SRC_CONFIG,
                     '--study', fx.STUDY_YAML]) == cli.EXIT_OK
    lines = capsys.readouterr().out.splitlines()
    plan_lines = [ln for ln in lines if ' eval ' in ln and ' config ' in ln]
    assert len(plan_lines) == 293 + 6
    assert sum('  = ' in ln for ln in plan_lines) == 23     # duplicates point to the first entry
    assert os.listdir(tmp_path) == []
    cli.main(['m5', '--dry-run', '--config-dir', fx.SRC_CONFIG, '--study', fx.STUDY_YAML])
    again = [ln for ln in capsys.readouterr().out.splitlines() if ' eval ' in ln]
    assert again == plan_lines                                # deterministic plan output


def test_cli_invalid_study_exits_2(tmp_path, capsys):
    raw = fx.shipped_raw()
    raw['expected_counts']['planned']['S2'] = 1
    path = fx.write_yaml(tmp_path / 'bad.yaml', raw)
    assert cli.main(['m5', '--study', path, '--config-dir', fx.SRC_CONFIG]) == cli.EXIT_INVALID
    assert 'REFUSED' in capsys.readouterr().out


def test_cli_forced_gate_failure_exits_3(tmp_path, capsys, evaluator):
    raw = fx.mini_raw()
    raw['baseline']['expected']['tripod_crawl']['failed_checks'] = ['static_stability',
                                                                    'joint_speed']
    path = fx.write_yaml(tmp_path / 'blocked.yaml', raw)
    assert cli.main(['m5', '--study', path, '--config-dir', fx.SRC_CONFIG],
                    evaluate=evaluator) == cli.EXIT_BLOCKED
    out = capsys.readouterr().out
    assert 'BLOCKED' in out and '"blocked_gate_failed": 3' in out


def test_cli_completed_mini_study_exits_0(tmp_path, capsys, evaluator):
    path = fx.write_yaml(tmp_path / 'mini.yaml', fx.mini_raw())
    assert cli.main(['m5', '--study', path, '--config-dir', fx.SRC_CONFIG],
                    evaluate=evaluator) == cli.EXIT_OK
    assert 'Study completed' in capsys.readouterr().out
