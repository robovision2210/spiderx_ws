"""colcon test: M5 Batch E - regression pins of the SHIPPED study's baseline records (n = 200).

These pin how the M5 runner classifies the six M4.5 baseline gaits of config/m5_study.yaml, so that
a change to the categories, the static-support applicability (N/A), the negative controls or K is
noticed and has to be explained in docs/M5_TEST_RESULTS.md. They are REPORTED offline model results,
not targets. Static support is an approximation; K is a model-derived joint-rate-per-distance
indicator; the 0.5 rad/s screen is a provisional secondary flag.
"""
import os

import pytest

from spiderx_controller import eval_runner as er
from spiderx_controller import eval_study as es

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')

# gait: (static_support_status, min margin m | None, failing >=3-contact samples | None,
#        binding support | None, support_ge3_fraction, K rad/m, screen, m45 verdict)
EXPECTED = {
    'wave': ('fail', -0.002582, 60, 'LF|RF|RR', 1.0, 108.10, 'fail', 'FAIL'),
    'tripod_crawl': ('fail', -0.004073, 108, 'LF|RF|RR', 1.0, 60.95, 'pass', 'FAIL'),
    'ripple': ('fail', -0.002098, 60, 'LF|RF|RR', 0.6, 40.58, 'pass', 'PASS'),
    'amble': ('fail', 0.000842, 20, 'LF|RF|RR', 0.2, 29.29, 'pass', 'PASS'),
    'pace': ('not_applicable', None, None, None, 0.0, 25.37, 'pass', 'PASS'),
    'trot': ('not_applicable', None, None, None, 0.0, 25.37, 'pass', 'PASS'),
}
# wave support fractions after the M4.5 phase-boundary fix (ece1e23): exactly 0.6 / 0.4
SUPPORT = {'wave': (0.6, 0.4), 'tripod_crawl': (1.0, 0.0), 'ripple': (0.6, 0.0),
           'amble': (0.2, 0.0), 'pace': (0.0, 0.0), 'trot': (0.0, 0.0)}


@pytest.fixture(scope='module')
def plan():
    context = es.load_context(config_dir=SRC_CONFIG)
    study, _ = es.load_study(os.path.join(SRC_CONFIG, es.STUDY_FILE), context=context)
    return es.build_plan(study, context)


@pytest.fixture(scope='module')
def baseline(plan):
    inputs = er.load_inputs(config_dir=SRC_CONFIG)
    out = {}
    for e in plan.stage_entries('S01'):
        if e.n == 200:
            r = er.evaluate_variant(plan.variants[e.eval_id], [e], inputs).record
            out[r['source_gait']] = r
    return out


def test_shipped_study_identity_and_accounting(plan):
    assert plan.study.name == 'm5_offline_evaluation'
    acc = plan.accounting
    assert (acc['stages_0_3']['planned'], acc['stages_0_3']['unique']) == (285, 264)
    assert (acc['all_stages']['planned'], acc['all_stages']['unique']) == (293, 270)
    assert acc['speed_checks'] == 6
    assert plan.study.raw['baseline']['negative_controls'] == ['wave', 'tripod_crawl']


@pytest.mark.parametrize('gait', list(EXPECTED))
def test_baseline_record_pins(baseline, gait):
    r = baseline[gait]
    status, margin, fails, binding, ge3, k, screen, verdict = EXPECTED[gait]
    assert r['evaluation_category'] == 'valid_pass' and r['ik_status'] == 'feasible'
    assert r['static_support_status'] == status
    if margin is None:
        assert r['min_static_margin_m'] is None and r['static_fail_samples_ge3'] is None
        assert r['fraction_margin_pos_given_ge3'] is None
    else:
        assert r['min_static_margin_m'] == pytest.approx(margin, abs=2e-6)
        assert r['static_fail_samples_ge3'] == fails
    assert r['binding_support'] == binding
    assert r['support_ge3_fraction'] == ge3
    assert r['k_rad_per_m'] == pytest.approx(k, abs=0.01)
    assert r['joint_speed_screen'] == screen and r['m45_verdict'] == verdict
    assert (r['m45_fraction_three_feet'], r['m45_fraction_four_feet']) == SUPPORT[gait]


def test_negative_controls_fail_static_support_and_stay_in_the_records(baseline, plan):
    for gait in plan.study.raw['baseline']['negative_controls']:
        r = baseline[gait]
        assert r['static_support_status'] == 'fail'
        assert 'static_stability' in r['m45_failed_checks'].split('|')
        assert r['requires_static_stability_m45_policy'] is True


def test_static_support_is_only_required_for_the_crawl_gaits(baseline):
    """ripple/amble show 'fail' over their 3-contact samples, but static support is NOT required
    for them (m45_policy) and their M4.5 verdict is PASS - two separate, explicit facts."""
    for gait in ('ripple', 'amble'):
        assert baseline[gait]['requires_static_stability_m45_policy'] is False
        assert baseline[gait]['m45_verdict'] == 'PASS'


def test_pace_and_trot_are_na_not_zero_not_fail(baseline):
    for gait in ('pace', 'trot'):
        r = baseline[gait]
        assert r['static_support_status'] == 'not_applicable'
        assert r['min_static_margin_m'] is None and r['binding_support'] is None
        assert r['m45_fraction_statically_stable'] == 0.0      # legacy M4.5 field only


def test_screen_is_secondary_and_k_is_continuous(baseline):
    wave = baseline['wave']
    assert wave['joint_speed_screen'] == 'fail' and wave['evaluation_category'] == 'valid_pass'
    assert wave['joint_speed_screen_breaches'] == 16
    assert wave['v_admissible_at_reference_m_s'] == pytest.approx(0.5 / wave['k_rad_per_m'])
    assert wave['k_rad_per_m'] == pytest.approx(wave['max_joint_speed_rad_s'] / 0.005)
