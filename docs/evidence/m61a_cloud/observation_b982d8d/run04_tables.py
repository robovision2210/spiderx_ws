#!/usr/bin/env python3
"""run_04 requirement table (evidence helper; read-only on the run directory).

Reads run_04/analysis.json (the frozen run_analysis.py output), the observer files and
run_04/criterion6_v2.json (crit6_v2.py, frozen at f44c27c), and writes run_04_requirements.md /
.json next to this script. The frozen analysis labels its supplementary /scan rows "6"; they are
reported here as 6a/6b (supplementary), separate from criterion 6 version 1 (the criterion of
record) and the version-2 proposal.
    python3 run04_tables.py   (from this directory)
"""
import glob
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RUN = os.path.join(HERE, 'run_04')


def main():
    an = json.load(open(os.path.join(RUN, 'analysis.json')))
    c6 = json.load(open(os.path.join(RUN, 'criterion6_v2.json')))
    obs = {}
    for p in sorted(glob.glob(os.path.join(RUN, 'observer', '*', 'observation.json'))):
        d = json.load(open(p))
        obs[d['mode']] = (d, os.path.relpath(p, HERE))
    rows = []
    for r in an['rows']:
        rid, crit = r['id'], r['criterion']
        if rid == '6':
            rid = '6b (supplementary)' if 'per-surface' in crit else '6a (supplementary)'
        rows.append({'id': rid, 'criterion': crit, 'result': 'PASS' if r['result'] else 'FAIL',
                     'value': r['value'], 'threshold': r.get('threshold')})
    v1, v2 = c6['v1'], c6['v2']
    rows.append({'id': '6 (v1, of record)', 'criterion': 'every individual range within +-8 mm of '
                 'the welded pose (single scans and 40-scan means; seam and grazing beams '
                 'excluded)', 'result': v1['verdict'],
                 'value': {'single_scan_within': v1['single_scan_ranges']['within_fraction'],
                           'mean_within': v1['per_beam_40_scan_means']['within_fraction'],
                           'mean_max_abs_m': v1['per_beam_40_scan_means']['max_abs_m'],
                           'minimax_any_planar_pose_m':
                               v1['minimax_over_planar_pose']['max_abs_residual_m']},
                 'threshold': '100 % within +-0.008 m'})
    rows.append({'id': '6 (v2 proposal)', 'criterion': 'fitted body planar pose vs pose/info '
                 '(frozen spec f44c27c)', 'result': v2['verdict'],
                 'value': {'dxy_m': v2['dxy_m'], 'dyaw_rad': v2['dyaw_rad'],
                           'sigma_used_m_m_rad': v2['sigma_used_m_m_rad'],
                           'prerequisites_ok': v2['prerequisites_ok']},
                 'threshold': '|dxy| <= 1.946 mm, |dyaw| <= 6.103 mrad; sigma <= 0.641 mm / '
                              '2.369 mrad'})
    rows.append({'id': '6 (v2 counterfactual)', 'criterion': 'measured error pattern on a '
                 'displaced body: offsets >= G8 FAIL, every offset recovered within 3 sigma',
                 'result': 'PASS' if c6['counterfactual_all_as_expected'] else 'FAIL',
                 'value': [(c['offset_m_m_rad'], c['verdict'], c['recovery_within_3_sigma'])
                           for c in c6['counterfactual']], 'threshold': 'all as expected'})
    pre = obs.get('preflight', ({}, None))[0]
    chk = pre.get('checks', {})
    rows.append({'id': 'R (new readiness)', 'criterion': 'preflight: /clock advanced and sim time '
                 'advanced while the body pose arrived', 'result':
                 'PASS' if chk.get('sim_clock', {}).get('ok') and
                 chk.get('body_pose_sim_progress', {}).get('ok') else 'FAIL',
                 'value': chk.get('body_pose_sim_progress'), 'threshold': '>= 0.1 s sim in 1.0 s'})
    out = {'run': 'run_04', 'rows': rows, 'observer_files': {k: v[1] for k, v in obs.items()},
           'criterion6_v2': 'run_04/criterion6_v2.json'}
    json.dump(out, open(os.path.join(HERE, 'run_04_requirements.json'), 'w'), indent=1)
    with open(os.path.join(HERE, 'run_04_requirements.md'), 'w') as f:
        f.write('| ID | Criterion | Result | Value |\n|---|---|---|---|\n')
        for r in rows:
            v = json.dumps(r['value'])
            f.write(f"| {r['id']} | {r['criterion']} | **{r['result']}** | "
                    f"`{v[:160]}{'…' if len(v) > 160 else ''}` |\n")
    for r in rows:
        print(f"{r['result']:5s} {r['id']:22s} {r['criterion'][:70]}")


if __name__ == '__main__':
    main()
