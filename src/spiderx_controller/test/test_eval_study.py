"""colcon test: M5 Batch A - study specification, immutable experiment matrix, accounting, IDs."""
import copy
import os

import pytest
import yaml

from spiderx_controller import eval_study as es
from spiderx_controller import gait_config as gc

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
STUDY_YAML = os.path.join(SRC_CONFIG, es.STUDY_FILE)


@pytest.fixture(scope='module')
def context():
    return es.load_context(config_dir=SRC_CONFIG)


@pytest.fixture(scope='module')
def raw():
    with open(STUDY_YAML) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def plan(context):
    study, _ = es.load_study(STUDY_YAML, context=context)
    return es.build_plan(study, context)


def _validate(raw, context, edit):
    r = copy.deepcopy(raw)
    edit(r)
    return es.validate_study(r, context)


def _plan(raw, context, edit):
    return es.build_plan(_validate(raw, context, edit), context)


# ------------------------------------------------------------ the shipped study and its accounting
def test_shipped_study_accounting_matches_the_plan(plan):
    acc = plan.accounting
    assert acc['planned'] == {'S01': 30, 'S2': 210, 'S3': 45, 'S4': 8}
    assert acc['new_unique'] == {'S01': 30, 'S2': 206, 'S3': 28, 'S4': 6}
    assert acc['duplicates_of_earlier'] == {'S01': 0, 'S2': 4, 'S3': 17, 'S4': 2}
    assert acc['stages_0_3'] == {'planned': 285, 'unique': 264, 'distinct_configurations': 240}
    assert acc['all_stages'] == {'planned': 293, 'unique': 270, 'distinct_configurations': 246}
    assert acc['speed_checks'] == 6 and acc['invalid_configurations'] == 0
    assert len(plan.entries) == 293 and len(plan.speed_entries) == 6
    assert len(plan.variants) == 270 + 6                # unique study evaluations + speed checks


def test_overlaps_are_exactly_the_documented_ones(plan):
    dup = {e.label: e.first_label for e in plan.entries if e.duplicate}
    assert dup['S2-lateral_sequence-b0.85-L40-h15'] == 'S01-wave-n200'
    assert dup['S2-lateral_sequence-b0.55-L40-h15'] == 'S01-amble-n200'
    assert dup['S3-diagonal_pairs-b0.5-L40-h15'] == 'S01-trot-n200'
    assert dup['S3-lateral_pairs-b0.5-L40-h15'] == 'S01-pace-n200'
    assert dup['S4-lateral_sequence-b0.75-y0'] == 'S01-tripod_crawl-n200'
    assert dup['S4-lateral_sequence-b0.85-y0'] == 'S01-wave-n200'
    s3_ls = [e for e in plan.stage_entries('S3') if e.label.startswith('S3-lateral_sequence')]
    assert len(s3_ls) == 15 and all(e.duplicate for e in s3_ls)
    assert len(dup) == 4 + 17 + 2


def test_speed_checks_are_separate_configurations(plan):
    stage_ids = {e.eval_id for e in plan.entries}
    for e in plan.speed_entries:
        assert e.eval_id not in stage_ids and e.stage == es.SPEED_STAGE
        v, base = plan.variants[e.eval_id], plan.variants[plan.speed_base[e.eval_id]]
        assert v.role == 'speed_check'
        assert v.spec.body_speed_m_s == pytest.approx(2 * base.spec.body_speed_m_s)
        assert v.spec.cycle_period_s == pytest.approx(base.spec.cycle_period_s / 2)
        assert {**v.physical, 'body_speed_m_s': 0} == {**base.physical, 'body_speed_m_s': 0}


def test_baseline_variants_equal_the_shipped_gaits(plan, context):
    for e in plan.stage_entries('S01'):
        v = plan.variants[e.eval_id]
        shipped = context.gait_config.gait(v.source_gait)
        for f in ('duty_factor', 'phase_offsets', 'swing_order', 'step_length_m', 'step_height_m',
                  'stance_height_offset_m', 'stance_center_offset_m', 'cycle_period_s',
                  'body_speed_m_s', 'swing_profile', 'requires_static_stability'):
            assert getattr(v.spec, f) == getattr(shipped, f), (v.source_gait, f)


def test_m45_policy_reproduces_the_shipped_flags(plan):
    flags = {plan.variants[e.eval_id].source_gait: plan.variants[e.eval_id].spec.requires_static_stability
             for e in plan.stage_entries('S01')}
    assert flags == {'wave': True, 'tripod_crawl': True, 'ripple': False, 'amble': False,
                     'pace': False, 'trot': False}
    s3 = {e.label: plan.variants[e.eval_id].spec.requires_static_stability
          for e in plan.stage_entries('S3')}
    assert s3['S3-diagonal_pairs-b0.85-L40-h15'] is False       # pairs never guarantee 3 feet
    assert s3['S3-lateral_sequence-b0.75-L20-h15'] is True


def test_stage4_is_a_constant_stance_centre_translation(plan, raw):
    assert raw['stages']['S4']['title'] == 'stance-centre translation sensitivity'
    ys = []
    for e in plan.stage_entries('S4'):
        v = plan.variants[e.eval_id]
        ys.append(v.spec.stance_center_offset_m)
        assert v.spec.step_length_m == 0.04 and v.spec.step_height_m == 0.015
    assert sorted(set(ys)) == [(0.0, -0.01), (0.0, -0.005), (0.0, 0.0), (0.0, 0.005)]


def test_every_variant_passed_the_m45_validator_and_is_grid_aligned(plan):
    for v in plan.variants.values():
        assert v.valid and v.gait_config is not None
        assert v.effective['samples_per_cycle'] == v.n
        assert v.effective['requires_static_stability_rule'].startswith('m45_policy')
        n = v.n
        assert abs((1 - v.spec.duty_factor) * n - round((1 - v.spec.duty_factor) * n)) < 1e-9


# ------------------------------------------------------------ canonical serialisation and IDs
def test_ids_are_deterministic_and_content_addressed(context, plan):
    again = es.build_plan(plan.study, context)
    assert [e.eval_id for e in again.entries] == [e.eval_id for e in plan.entries]
    v = plan.variants[plan.entries[0].eval_id]
    assert v.config_id == es.config_id_of(v.physical)
    assert v.eval_id == es.eval_id_of(v.physical, v.n)
    assert len(v.eval_id) == 12 and int(v.eval_id, 16) >= 0


def test_canonical_json_ignores_key_order_and_float_noise():
    a = es.canonical_json({'b': 0.1 + 0.2, 'a': [1, 2]})
    b = es.canonical_json({'a': [1, 2], 'b': 0.3})
    assert a == b == '{"a":[1,2],"b":0.3}'


def test_same_physics_different_n_share_the_config_id(plan):
    wave = [plan.variants[e.eval_id] for e in plan.stage_entries('S01')
            if plan.variants[e.eval_id].source_gait == 'wave']
    assert len({v.config_id for v in wave}) == 1 and len({v.eval_id for v in wave}) == 5


def test_study_id_is_a_hash_of_the_specification(raw, context):
    a = es.validate_study(raw, context)
    b = _validate(raw, context, lambda r: r['stages']['S3'].update(step_height_m=0.016))
    assert a.study_id.startswith('m5_offline_evaluation-') and a.study_id != b.study_id


def test_invalid_gait_configuration_is_kept_not_dropped(context):
    v = es.make_variant(context, source_gait='trot', pattern='diagonal_pairs', n=200,
                        duty_factor=0.5, phase_offsets={'front_left': 0.0, 'front_right': 0.5,
                                                        'rear_left': 0.5, 'rear_right': 0.0},
                        swing_order=[['front_left', 'rear_right'], ['front_right', 'rear_left']],
                        step_length_m=0.04, step_height_m=0.0, stance_height_offset_m=0.0,
                        stance_center_offset_m=[0.0, 0.0], body_speed_m_s=0.005,
                        swing_profile='cycloid')
    assert not v.valid and 'step_height_m' in v.error and v.gait_config is None
    assert len(v.eval_id) == 12                         # still identified and reportable


# ------------------------------------------------------------ refusals
@pytest.mark.parametrize('edit,match', [
    (lambda r: r.update(schema='spiderx_m5_study/v0'), 'schema'),
    (lambda r: r.update(offline_only=False), 'offline_only'),
    (lambda r: r.update(extra=1), 'unknown keys'),
    (lambda r: r['baseline']['gaits'].append('gallop'), 'not a gait'),
    (lambda r: r['patterns'].update(bad={'source_gait': 'gallop'}), 'not a gait'),
    (lambda r: r['stages']['S2'].update(pattern='zigzag'), 'not a defined pattern'),
    (lambda r: r['stages']['S3']['patterns'].append('zigzag'), 'not a defined pattern'),
    (lambda r: r['stages']['S2']['duty_factors'].append(0.4), 'outside'),
    (lambda r: r['stages']['S2']['duty_factors'].append(1.0), 'outside'),
    (lambda r: r['stages']['S2']['step_lengths_m'].append(-0.01), 'outside'),
    (lambda r: r['stages']['S2']['step_heights_m'].append(0.5), 'outside'),
    (lambda r: r['stages']['S4']['stance_center_y_offsets_m'].append(0.2), 'outside'),
    (lambda r: r['stages']['S2']['duty_factors'].append(0.85), 'duplicate levels'),
    (lambda r: r['baseline']['resolutions'].append(3), 'integer'),
    (lambda r: r['baseline'].update(negative_controls=['wave', 'nope']), 'subset'),
    (lambda r: r['baseline']['expected']['wave'].update(failed_checks=['joint_speed']),
     'must be expected to fail static_stability'),
    (lambda r: r['fixed'].update(body_speed_m_s=0.006), 'shipped default'),
    (lambda r: r['screening'].update(joint_speed_reference_rad_s=1.0), 'SIMULATION_PLACEHOLDER'),
    (lambda r: r['gate']['pattern_separation'].update(equal_per_leg_metrics=[['wave', 'trot']]),
     'share beta'),
    (lambda r: r['stages']['S4'].update(title='body sway'), 'stance-centre translation'),
    (lambda r: r['expected_counts']['new_unique'].update(S4=9), 'impossible'),
])
def test_invalid_specifications_are_refused(raw, context, edit, match):
    with pytest.raises(es.StudyError, match=match):
        _validate(raw, context, edit)


@pytest.mark.parametrize('edit,match', [
    (lambda r: r['expected_counts']['planned'].update(S2=211), 'planned.S2'),
    (lambda r: r['expected_counts']['stages_0_3'].update(unique=265), 'stages_0_3.unique'),
    (lambda r: r['expected_counts'].update(speed_checks=5), 'speed_checks'),
    (lambda r: r['stages']['S2']['step_heights_m'].append(0.04), 'planned.S2'),
    (lambda r: r['stages']['S2']['duty_factors'].append(0.623), 'grid'),
])
def test_mismatched_accounting_or_grid_is_refused(raw, context, edit, match):
    with pytest.raises(es.StudyError, match=match):
        _plan(raw, context, edit)


def test_study_error_is_a_gait_config_error():
    assert issubclass(es.StudyError, gc.GaitConfigError)


def test_duplicate_yaml_keys_are_refused(tmp_path, context):
    text = open(STUDY_YAML).read().replace('study_name: m5_offline_evaluation',
                                           'study_name: m5_offline_evaluation\nstudy_name: x')
    p = tmp_path / 'dup.yaml'
    p.write_text(text)
    with pytest.raises(es.StudyError, match='[Dd]uplicate'):
        es.load_study(str(p), context=context)


def test_the_shipped_m45_yaml_is_not_modified(context, plan):
    before = open(os.path.join(SRC_CONFIG, gc.CONFIG_FILE)).read()
    es.build_plan(plan.study, context)
    assert open(os.path.join(SRC_CONFIG, gc.CONFIG_FILE)).read() == before
    assert context.gaits_raw['analysis']['samples_per_cycle'] == 200    # deep-copied per variant
