"""colcon test: M5 Batch C - deterministic raw records, manifest and provenance."""
import csv
import json
import os
import re
import subprocess

import pytest

import m5_fixtures as fx
from spiderx_controller import eval_records as rec
from spiderx_controller import eval_runner as er
from spiderx_controller import eval_study as es
from spiderx_controller import m5_offline_evaluation as cli

ARGV = ['m5', '--study', 'mini.yaml', '--allow-dirty']


@pytest.fixture(scope='module')
def context():
    return es.load_context(config_dir=fx.SRC_CONFIG)


@pytest.fixture(scope='module')
def evaluator():
    return fx.CachingEvaluator(er.load_inputs(config_dir=fx.SRC_CONFIG))


@pytest.fixture(scope='module')
def provenance(context, tmp_path_factory):
    path = fx.write_yaml(tmp_path_factory.mktemp('spec') / 'mini.yaml', fx.mini_raw())
    return rec.collect_provenance(path, context, allow_dirty=True)


@pytest.fixture(scope='module')
def mini(context, evaluator):
    return er.run_study(fx.plan_of(fx.mini_raw(), context), None, evaluate=evaluator)


@pytest.fixture(scope='module')
def written(mini, provenance, tmp_path_factory):
    out = tmp_path_factory.mktemp('runs')
    a, ma = rec.write_study(mini, str(out / 'a'), provenance, ARGV + ['--out', str(out / 'a')])
    b, mb = rec.write_study(mini, str(out / 'b'), provenance, ARGV + ['--out', str(out / 'b')])
    return a, ma, b, mb


def _csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def _read(path):
    with open(path, 'rb') as f:
        return f.read()


# ------------------------------------------------------------ determinism
def test_two_writes_are_byte_identical_except_the_environment_file(written):
    a, ma, b, mb = written
    files = rec.compared_files(a)
    assert files == rec.compared_files(b) and len(files) == 7
    for rel in files + ['manifest.json']:
        assert _read(os.path.join(a, rel)) == _read(os.path.join(b, rel)), rel
    assert ma == mb
    assert 'environment.json' in os.listdir(a)                  # written, but not compared
    assert ma['excluded_from_comparison'] == ['environment.json']


def test_manifest_hashes_every_compared_file(written):
    a, ma, _, _ = written
    assert set(ma['files_sha256']) == set(rec.compared_files(a))
    for relpath, digest in ma['files_sha256'].items():
        assert rec.sha256_file(os.path.join(a, relpath)) == digest


def test_no_timestamps_in_compared_artifacts(written):
    a, _, _, _ = written
    stamp = re.compile(r'\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}')
    for relpath in rec.compared_files(a) + ['manifest.json']:
        assert not stamp.search(_read(os.path.join(a, relpath)).decode()), relpath


# ------------------------------------------------------------ provenance and accounting
def test_manifest_carries_provenance_accounting_and_scope(written, mini):
    _, m, _, _ = written
    assert m['schema'] == es.SCHEMA and m['study_id'] == mini.plan.study.study_id
    assert m['status'] == 'completed' and m['gate_passed'] is True
    acc = m['accounting']
    assert acc['all_stages'] == {'planned': 14, 'unique': 11, 'distinct_configurations': 7}
    assert acc['speed_checks'] == 4
    assert m['record_counts'] == {'evaluation_records': 15, 'plan_entries': 18, 'evaluated': 15}
    p = m['provenance']
    assert p['git_commit'] is None or re.fullmatch('[0-9a-f]{40}', p['git_commit'])
    assert p['git_dirty_allowed'] is True and p['m45_config_version'] == 1
    assert set(p['inputs_sha256']) == {'study_spec', 'm4_5_gaits.yaml', 'spiderx_legs.yaml',
                                       'urdf_expanded_xml'}
    assert {'eval_study', 'eval_runner', 'gait_metrics', 'leg_kinematics'} <= \
        set(p['source_modules_sha256'])
    assert m['cli_args'][-2:] == ['--out', '<out>']
    assert 'Not walking' in m['scope'] and 'not hardware readiness' in m['scope']
    assert 'heuristic motion proxy' in m['labels']['lift_work_proxy_j_per_m']
    assert 'static-support approximation' in m['labels']['static_support_status']


def test_counts_cover_every_category_explicitly(written):
    _, m, _, _ = written
    c = m['category_counts_stage_records']
    assert set(c['evaluation_category']) == set(er.CATEGORIES)
    assert set(c['static_support_status']) == set(er.STATIC_STATUSES)
    assert c['evaluation_category']['ik_infeasible'] == 1


# ------------------------------------------------------------ raw tables keep every outcome
def test_raw_tables_have_one_row_per_unique_evaluation_and_plan_entry(written):
    a, _, _, _ = written
    rows = _csv(os.path.join(a, 'raw/evaluations.csv'))
    assert len(rows) == 15 and len({r['eval_id'] for r in rows}) == 15
    plan = _csv(os.path.join(a, 'raw/plan.csv'))
    assert len(plan) == 18 and sum(r['duplicate'] == 'true' for r in plan) == 3
    infeasible = [r for r in rows if r['evaluation_category'] == 'ik_infeasible']
    assert len(infeasible) == 1 and infeasible[0]['k_rad_per_m'] == ''
    reasons = _csv(os.path.join(a, 'raw/ik_reasons.csv'))
    assert {r['eval_id'] for r in reasons} == {infeasible[0]['eval_id']}


def test_na_is_explicit_in_csv_and_never_zero(written):
    a, _, _, _ = written
    rows = {r['labels'].split('|')[0]: r for r in _csv(os.path.join(a, 'raw/evaluations.csv'))}
    for gait in ('pace', 'trot'):
        r = rows[f'S01-{gait}-n200']
        assert r['static_support_status'] == 'not_applicable'
        assert r['min_static_margin_m'] == '' and r['fraction_margin_pos_given_ge3'] == ''
    checks = _csv(os.path.join(a, 'raw/checks.csv'))
    pace_id = rows['S01-pace-n200']['eval_id']
    stab = next(c for c in checks if c['eval_id'] == pace_id and c['check'] == 'static_stability')
    assert stab['status'] == 'n/a' and stab['applicable'] == 'false'


def test_json_records_carry_the_complete_effective_configuration(written):
    a, _, _, _ = written
    recs = json.load(open(os.path.join(a, 'raw/evaluations.json')))
    assert len(recs) == 15
    eff = recs[0]['effective_configuration']
    for key in ('duty_factor', 'phase_offsets', 'swing_order', 'step_length_m', 'step_height_m',
                'stance_center_offset_m', 'body_speed_m_s', 'cycle_period_s',
                'samples_per_cycle', 'source_gait', 'requires_static_stability', 'analysis',
                'validation_margin_rad', 'ik_position_tol_m'):
        assert key in eff
    configs = json.load(open(os.path.join(a, 'raw/configs.json')))
    assert all(es.config_id_of(phys) == cid for cid, phys in configs.items())


# ------------------------------------------------------------ blocked studies keep their evidence
def test_blocked_study_writes_all_evidence(context, evaluator, provenance, tmp_path):
    raw = fx.mini_raw()
    raw['baseline']['expected']['tripod_crawl']['failed_checks'] = ['static_stability',
                                                                    'joint_speed']
    result = er.run_study(fx.plan_of(raw, context), None, evaluate=evaluator)
    d, m = rec.write_study(result, str(tmp_path), provenance, ARGV)
    assert m['status'] == 'blocked' and m['failed_gate_checks'] == ['baseline_reproduction_n200']
    gate = json.load(open(os.path.join(d, 'raw/gate.json')))
    assert gate['blocked_evaluations'] == 3 and gate['gate_passed'] is False
    rows = _csv(os.path.join(d, 'raw/evaluations.csv'))
    assert sum(r['evaluation_category'] == 'blocked_gate_failed' for r in rows) == 3
    assert len(rows) == 15                                      # nothing dropped


# ------------------------------------------------------------ refusals and mutations
def test_existing_study_directory_is_never_overwritten(written, mini, provenance):
    a, _, _, _ = written
    with pytest.raises(es.StudyError, match='never overwritten'):
        rec.write_study(mini, os.path.dirname(a), provenance, ARGV)


def test_mutation_filtered_record_is_refused_before_writing(mini, provenance, tmp_path):
    import copy
    result = copy.copy(mini)
    result.results = dict(mini.results)
    result.results.pop(next(eid for eid, r in mini.results.items()
                            if r.record['evaluation_category'] == 'ik_infeasible'))
    with pytest.raises(es.StudyError, match='inconsistent result'):
        rec.write_study(result, str(tmp_path), provenance, ARGV)


def test_dirty_tree_is_refused_unless_allowed(context, tmp_path):
    repo = tmp_path / 'repo'
    repo.mkdir()
    git = ['git', '-c', 'user.name=t', '-c', 'user.email=t@t', '-C', str(repo)]
    subprocess.run(git + ['init', '-q'], check=True)
    (repo / 'a.txt').write_text('a')
    subprocess.run(git + ['add', 'a.txt'], check=True)
    subprocess.run(git + ['commit', '-q', '-m', 'a'], check=True)
    commit, dirty = rec.git_state(str(repo))
    assert re.fullmatch('[0-9a-f]{40}', commit) and dirty is False
    (repo / 'a.txt').write_text('changed')
    assert rec.git_state(str(repo))[1] is True
    with pytest.raises(rec.DirtyTreeError, match='--allow-dirty'):
        rec.collect_provenance(fx.STUDY_YAML, context, cwd=str(repo))
    p = rec.collect_provenance(fx.STUDY_YAML, context, allow_dirty=True, cwd=str(repo))
    assert p['git_dirty'] is True and p['git_commit'] == commit


def test_normalised_argv_hides_only_the_output_directory():
    assert rec.normalised_argv(['x', '--out', '/a/b', '--allow-dirty']) == \
        ['x', '--out', '<out>', '--allow-dirty']
    assert rec.normalised_argv(['x', '--out=/a']) == ['x', '--out=<out>']


def test_cli_writes_the_study_and_refuses_a_second_run_into_it(tmp_path, evaluator, capsys):
    path = fx.write_yaml(tmp_path / 'mini.yaml', fx.mini_raw())
    argv = ['m5', '--study', path, '--config-dir', fx.SRC_CONFIG, '--allow-dirty',
            '--out', str(tmp_path / 'out')]
    assert cli.main(argv, evaluate=evaluator) == cli.EXIT_OK
    (study_dir,) = os.listdir(tmp_path / 'out')
    assert study_dir.startswith('m5_offline_evaluation-')
    evaluator.calls.clear()
    assert cli.main(argv, evaluate=evaluator) == cli.EXIT_INVALID      # refused up front
    assert evaluator.calls == [] and 'never overwritten' in capsys.readouterr().out
