"""Shared helpers for the M5 tests: small real studies and a caching evaluator.

The shipped study (293 + 6 evaluations, ~11 min) is far too large for unit tests, so the tests use
MINI studies derived from config/m5_study.yaml with fewer levels and their own exact accounting.
Every evaluation is still the real, unchanged M4.5 evaluator; CachingEvaluator only remembers
results by eval_id so that later tests (forced gate failures, mutations) can re-run instantly.
"""
import copy
import os

import yaml

from spiderx_controller import eval_runner as er
from spiderx_controller import eval_study as es

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
STUDY_YAML = os.path.join(SRC_CONFIG, es.STUDY_FILE)


def shipped_raw():
    with open(STUDY_YAML) as f:
        return yaml.safe_load(f)


def mini_raw(gaits=('tripod_crawl', 'amble', 'pace', 'trot'), negative=('tripod_crawl',)):
    """A small study: 2 resolutions, an IK-infeasible Stage 2 point, Stage 3 and Stage 4 points.

    Accounting (with the default gaits): S01 8, S2 2 new, S3 2 planned / 0 new (duplicates of
    Stage 2 and of the trot baseline), S4 2 planned / 1 new (y = 0 is the tripod_crawl baseline).
    """
    r = copy.deepcopy(shipped_raw())
    b = r['baseline']
    b['gaits'] = list(gaits)
    b['negative_controls'] = list(negative)
    b['resolutions'] = [40, 200]
    b['expected'] = {g: b['expected'][g] for g in gaits}
    r['gate']['pattern_separation']['equal_per_leg_metrics'] = [['pace', 'trot']]
    r['stages']['S2'].update(duty_factors=[0.5], step_lengths_m=[0.04, 0.30],
                             step_heights_m=[0.015])
    r['stages']['S3'].update(patterns=['lateral_sequence', 'diagonal_pairs'], duty_factors=[0.5],
                             step_lengths_m=[0.04])
    r['stages']['S4'].update(duty_factors=[0.75], stance_center_y_offsets_m=[-0.005, 0.0])
    s01 = 2 * len(gaits)
    s4_dup = 1 if 'tripod_crawl' in gaits else 0
    s3_dup = 2 if 'trot' in gaits else 1
    configs = len(gaits) + 2
    r['expected_counts'] = {
        'planned': {'S01': s01, 'S2': 2, 'S3': 2, 'S4': 2},
        'new_unique': {'S01': s01, 'S2': 2, 'S3': 2 - s3_dup, 'S4': 2 - s4_dup},
        'stages_0_3': {'planned': s01 + 4, 'unique': s01 + 4 - s3_dup,
                       'distinct_configurations': configs + (2 - s3_dup)},
        'all_stages': {'planned': s01 + 6, 'unique': s01 + 6 - s3_dup - s4_dup,
                       'distinct_configurations': configs + (2 - s3_dup) + (2 - s4_dup)},
        'speed_checks': len(gaits),
    }
    return r


def plan_of(raw, context):
    return es.build_plan(es.validate_study(raw, context), context)


def write_yaml(path, raw):
    with open(path, 'w') as f:
        yaml.safe_dump(raw, f, sort_keys=False)
    return str(path)


class CachingEvaluator:
    """The real evaluator with a cache by eval_id, and a log of what was actually requested."""

    def __init__(self, inputs):
        self.inputs, self.cache, self.calls = inputs, {}, []

    def __call__(self, variant, entries, inputs=None):
        self.calls.append(variant.eval_id)
        if variant.eval_id not in self.cache:
            r = er.evaluate_variant(variant, entries, self.inputs)
            self.cache[variant.eval_id] = (r.record, r.q_series, r.checks, r.ik_reasons)
        rec, q, checks, reasons = self.cache[variant.eval_id]
        order = (es.STAGES + (es.SPEED_STAGE,)).index
        rec = {**rec, 'stages': '|'.join(sorted({e.stage for e in entries}, key=order)),
               'labels': '|'.join(e.label for e in entries)}
        return er.EvalResult(rec, copy.deepcopy(q), copy.deepcopy(checks), dict(reasons))
