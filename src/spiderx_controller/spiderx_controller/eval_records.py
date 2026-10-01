"""M5 OFFLINE study records: deterministic raw CSV/JSON files and a provenance-rich manifest.

Layout (under the git-ignored log/ by default; one directory per study_id):

    <out>/<study_id>/manifest.json          study, status, gate, accounting (pairs AND physical
                                             configurations), category counts, provenance (git
                                             commit + dirty flag, SHA-256 of every input and
                                             source module), CLI arguments, SHA-256 of every
                                             compared file. No timestamps.
    <out>/<study_id>/environment.json       Python / platform versions. EXCLUDED from the
                                             deterministic comparison (differs between machines).
    <out>/<study_id>/raw/plan.csv           every planned evaluation (293 stage entries + speed
                                             checks), with duplicate -> first-entry mapping
    <out>/<study_id>/raw/evaluations.csv    one row per unique evaluation (incl. invalid,
                                             IK-infeasible, blocked and negative-control rows)
    <out>/<study_id>/raw/evaluations.json   the same records + complete effective configuration,
                                             every M4.5 check, IK failure reasons
    <out>/<study_id>/raw/configs.json       config_id -> canonical physical configuration
    <out>/<study_id>/raw/checks.csv         long format: one row per (evaluation, M4.5 check)
    <out>/<study_id>/raw/ik_reasons.csv     eval_id, leg, reason, count (non-'ok' reasons only)
    <out>/<study_id>/raw/gate.json          the structured Stage 0/1 gate report

Numbers use 10 significant digits and JSON keys are sorted (gait_report helpers), so two runs of
the same study at the same commit produce byte-identical files (environment.json excepted).
Nothing is ever filtered: every planned unique evaluation has exactly one record.
"""

import hashlib
import json
import os
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET

from spiderx_controller import eval_runner as er
from spiderx_controller import eval_study as es
from spiderx_controller import gait_report as gr
from spiderx_controller import leg_kinematics as lk

DEFAULT_OUT = os.path.join('log', 'm5_offline_evaluation')
EXCLUDED_FROM_COMPARISON = ('environment.json',)
SOURCE_MODULES = ('eval_study', 'eval_runner', 'eval_records', 'eval_tables',
                  'm5_offline_evaluation', 'gait_config', 'gait_phase', 'gait_trajectory',
                  'gait_kinematics', 'gait_metrics', 'gait_results', 'gait_report',
                  'leg_kinematics', 'posture_config', 'kinematics_targets', 'config_check')
SCOPE = ('OFFLINE model analysis of gait configuration classes on the SpiderX URDF kinematic '
         'model with the unchanged M4.5 evaluator. Not walking, not dynamic stability, not '
         'energy or power, not navigation, not real-time execution, not actuator capability, '
         'not hardware readiness.')
LABELS = {
    'evaluation_category': 'offline model metric (kinematic checks only)',
    'static_support_status': 'static-support approximation (>= 3 stance feet, quasi-static, '
                             'URDF CAD masses, point feet, level body)',
    'k_rad_per_m': 'offline model metric: peak joint-rate demand per metre of travel; speed '
                   'rescales time only in this clock-driven model; not a motor capability',
    'joint_speed_screen': 'provisional secondary screening flag vs the 0.5 rad/s simulation '
                          'placeholder; not an actuator limit, not a validity criterion',
    'foot_path_per_m': 'heuristic motion proxy (not energy)',
    'joint_travel_rad_per_m': 'heuristic motion proxy (not energy)',
    'lift_work_proxy_j_per_m': 'heuristic motion proxy (not energy, power or efficiency)',
}


class DirtyTreeError(es.StudyError):
    """The git working tree is dirty and --allow-dirty was not given."""


# ---------------------------------------------------------------- provenance
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 16), b''):
            h.update(chunk)
    return h.hexdigest()


def _git(args, cwd):
    try:
        out = subprocess.run(['git'] + args, cwd=cwd, capture_output=True, text=True,
                             timeout=30, check=True)
        return out.stdout
    except (OSError, subprocess.SubprocessError):
        return None


def git_state(cwd=None):
    """(commit, dirty) of the repository containing cwd; (None, None) outside a repository."""
    cwd = cwd or os.getcwd()
    commit = _git(['rev-parse', 'HEAD'], cwd)
    if commit is None:
        return None, None
    status = _git(['status', '--porcelain'], cwd)
    return commit.strip(), bool(status and status.strip())


def collect_provenance(study_path, context, allow_dirty=False, cwd=None, urdf_root=None):
    """Provenance of a run. Raises DirtyTreeError for a dirty tree unless allow_dirty."""
    commit, dirty = git_state(cwd)
    if dirty and not allow_dirty:
        raise DirtyTreeError('the git working tree has uncommitted changes; commit them or pass '
                             '--allow-dirty (recorded in the manifest)')
    if urdf_root is None:
        from spiderx_controller.config_check import load_urdf
        urdf_root = load_urdf()
    import spiderx_controller
    pkg = os.path.dirname(os.path.abspath(spiderx_controller.__file__))
    sources = {m: sha256_file(os.path.join(pkg, f'{m}.py')) for m in SOURCE_MODULES
               if os.path.exists(os.path.join(pkg, f'{m}.py'))}
    return {
        'git_commit': commit, 'git_dirty': dirty, 'git_dirty_allowed': bool(allow_dirty),
        'inputs_sha256': {
            'study_spec': sha256_file(study_path),
            'm4_5_gaits.yaml': sha256_file(context.gaits_path),
            'spiderx_legs.yaml': sha256_file(context.legs_path),
            'urdf_expanded_xml': hashlib.sha256(ET.tostring(urdf_root)).hexdigest(),
        },
        'source_modules_sha256': sources,
        'm45_config_version': context.gait_config.config_version,
    }


def environment():
    return {'python': sys.version.split()[0], 'implementation': platform.python_implementation(),
            'platform': platform.platform(), 'note': 'excluded from deterministic comparison'}


def normalised_argv(argv):
    """CLI arguments with the --out value replaced, so runs into different directories compare."""
    out, skip = [], False
    for a in argv:
        if skip:
            out.append('<out>')
            skip = False
        elif a == '--out':
            out.append(a)
            skip = True
        elif a.startswith('--out='):
            out.append('--out=<out>')
        else:
            out.append(a)
    return out


# ---------------------------------------------------------------- rows
def raw_columns(result):
    cols = None
    for r in result.results.values():
        keys = list(r.record)
        if cols is None:
            cols = keys
        elif keys != cols:
            raise ValueError('records do not share one schema')
    return cols


def evaluation_rows(result):
    cols = raw_columns(result)
    return [cols] + [[_cell(r.record[c]) for c in cols] for r in result.results.values()]


def _cell(v):
    return v if isinstance(v, str) else gr.fmt(v)


def plan_rows(plan):
    rows = [['stage', 'label', 'eval_id', 'config_id', 'samples_per_cycle', 'duplicate',
             'first_stage', 'first_label']]
    for e in plan.entries + plan.speed_entries:
        rows.append([e.stage, e.label, e.eval_id, e.config_id, str(e.n),
                     gr.fmt(e.duplicate), e.first_stage, e.first_label])
    return rows


def check_rows(result):
    rows = [['eval_id', 'check', 'status', 'passed', 'applicable', 'value', 'threshold', 'unit',
             'label', 'failure_count']]
    for eid, r in result.results.items():
        for c in r.checks:
            status = 'n/a' if not c['applicable'] else ('pass' if c['passed'] else 'fail')
            rows.append([eid, c['name'], status, gr.fmt(c['passed']), gr.fmt(c['applicable']),
                         gr.fmt(c['value']), gr.fmt(c['threshold']), c['unit'], c['label'],
                         str(c['failure_count'])])
    return rows


def ik_reason_rows(result):
    rows = [['eval_id', 'leg', 'reason', 'count']]
    for eid, r in result.results.items():
        for leg in lk.ALL_LEGS:
            for reason, count in sorted((r.ik_reasons or {}).get(leg, {}).items()):
                rows.append([eid, leg, reason, str(count)])
    return rows


def evaluation_json(result):
    out = []
    for eid, r in result.results.items():
        v = result.plan.variants[eid]
        out.append({'record': r.record, 'effective_configuration': v.effective,
                    'checks': r.checks, 'ik_reasons': r.ik_reasons})
    return out


def gate_report(result):
    return {'status': result.status, 'gate_passed': result.gate_passed,
            'failed_checks': [g.check_id for g in result.gate if not g.passed],
            'checks': [{'check_id': g.check_id, 'scope': g.scope, 'passed': g.passed,
                        'detail': g.detail} for g in result.gate],
            'blocked_evaluations': sum(1 for r in result.results.values()
                                       if r.record['evaluation_category'] ==
                                       'blocked_gate_failed'),
            'rule': 'Stages 2-4 are evaluated only if every Stage 0/1 gate check passes'}


# ---------------------------------------------------------------- writing
def study_dir(out_root, study):
    return os.path.join(out_root, study.study_id)


def _write_json(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    gr.write_json(path, data)


def _write_csv(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    gr.write_csv(path, rows)


def write_raw(result, d):
    """Write every raw file. Returns the relative paths written (sorted)."""
    plan = result.plan
    files = {
        'raw/plan.csv': lambda p: _write_csv(p, plan_rows(plan)),
        'raw/evaluations.csv': lambda p: _write_csv(p, evaluation_rows(result)),
        'raw/evaluations.json': lambda p: _write_json(p, evaluation_json(result)),
        'raw/configs.json': lambda p: _write_json(p, {v.config_id: v.physical
                                                      for v in plan.variants.values()}),
        'raw/checks.csv': lambda p: _write_csv(p, check_rows(result)),
        'raw/ik_reasons.csv': lambda p: _write_csv(p, ik_reason_rows(result)),
        'raw/gate.json': lambda p: _write_json(p, gate_report(result)),
    }
    for rel, write in files.items():
        write(os.path.join(d, rel))
    return sorted(files)


def compared_files(d):
    """Every file under d except the manifest itself and the excluded environment file."""
    out = []
    for root, _, names in os.walk(d):
        for name in names:
            rel = os.path.relpath(os.path.join(root, name), d).replace(os.sep, '/')
            if rel not in EXCLUDED_FROM_COMPARISON and rel != 'manifest.json':
                out.append(rel)
    return sorted(out)


def write_manifest(result, d, provenance, argv, extra=None):
    plan, study = result.plan, result.plan.study
    manifest = {
        'schema': es.SCHEMA, 'study_id': study.study_id, 'study_name': study.name,
        'spec_sha256': study.spec_sha256, 'status': result.status,
        'gate_passed': result.gate_passed,
        'failed_gate_checks': [g.check_id for g in result.gate if not g.passed],
        'accounting': plan.accounting,
        'record_counts': {'evaluation_records': len(result.results),
                          'plan_entries': len(plan.entries) + len(plan.speed_entries),
                          'evaluated': len(result.evaluated)},
        'category_counts_stage_records': er.category_counts(result),
        'provenance': provenance,
        'cli_args': normalised_argv(argv),
        'scope': SCOPE, 'labels': LABELS,
        'excluded_from_comparison': list(EXCLUDED_FROM_COMPARISON),
        'files_sha256': {rel: sha256_file(os.path.join(d, rel)) for rel in compared_files(d)},
        **(extra or {}),
    }
    _write_json(os.path.join(d, 'manifest.json'), manifest)
    return manifest


def write_study(result, out_root, provenance, argv, extra_writers=()):
    """Write the raw records (+ extra writers, e.g. derived tables) and the manifest.

    Refuses to write into an existing study directory (never overwrites earlier evidence)."""
    d = study_dir(out_root, result.plan.study)
    if os.path.exists(d):
        raise es.StudyError(f'{d} already exists; choose another --out (earlier results are '
                            'never overwritten)')
    os.makedirs(d)
    violations = er.verify_result(result)
    if violations:
        raise es.StudyError('refusing to write an inconsistent result: ' + '; '.join(violations))
    write_raw(result, d)
    for writer in extra_writers:
        writer(result, d)
    _write_json(os.path.join(d, 'environment.json'), environment())
    manifest = write_manifest(result, d, provenance, argv)
    return d, manifest


def load_json(path):
    with open(path) as f:
        return json.load(f)


__all__ = ['DEFAULT_OUT', 'DirtyTreeError', 'collect_provenance', 'git_state', 'write_study',
           'write_raw', 'write_manifest', 'compared_files', 'study_dir', 'normalised_argv',
           'evaluation_rows', 'plan_rows', 'check_rows', 'ik_reason_rows', 'gate_report',
           'sha256_file', 'LABELS', 'SCOPE']
