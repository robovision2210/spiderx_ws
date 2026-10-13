#!/usr/bin/env python3
"""Self-test of crit6_v2.py on SYNTHETIC captures only (no recorded run is judged here).

Synthetic scans are ray-cast from a known body pose with the frozen ray caster, plus the sensor
model: gaussian noise 10 mm per scan and an independent per-beam systematic error of 5 mm RMS.
Checks: (a) no discrepancy -> v2 PASS, v1 FAIL, counterfactuals as expected; (b) the body
displaced 3 mm while pose/info still reports the weld -> v2 FAIL; (c) 30 scans -> INCONCLUSIVE.
    python3 selftest_crit6_v2.py WORLD_SDF TEMPLATE_CAPTURE   (exit 0 = all checks pass)
The template capture provides only the message layout (angles, limits, pose entries); its ranges
are replaced.
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import crit6_v2 as c6  # noqa: E402


def synth(spec, world, template, body, scans, seed):
    sc = c6.load_ray_caster(os.path.normpath(os.path.join(HERE, spec['ray_caster']['path'])))
    cap = json.load(open(template))
    s0 = cap['scans'][0]
    scene = c6.Scene(sc, world, spec, s0)
    n = len(s0['ranges'])
    exact = scene.predict(body, range(n))
    rng = np.random.default_rng(seed)
    sys_err = rng.normal(0.0, 0.005, n)
    out = dict(cap, scans=[])
    for k in range(scans):
        r = exact + sys_err + rng.normal(0.0, 0.010, n)
        r = [float(v) if np.isfinite(v) and s0['range_min'] < v < s0['range_max'] else None
             for v in r]
        out['scans'].append(dict(s0, ranges=r))
    return out


def run(spec_path, world, cap):
    with tempfile.TemporaryDirectory() as d:
        cp, rp = os.path.join(d, 'capture.json'), os.path.join(d, 'result.json')
        json.dump(cap, open(cp, 'w'))
        subprocess.run([sys.executable, os.path.join(HERE, 'crit6_v2.py'), spec_path, cp, world,
                        '--out', rp], check=True, stdout=subprocess.DEVNULL)
        return json.load(open(rp))


def main():
    world, template = sys.argv[1], sys.argv[2]
    spec_path = os.path.join(HERE, 'crit6_v2_spec.json')
    spec = json.load(open(spec_path))
    ok = True
    a = run(spec_path, world, synth(spec, world, template, (0.0, 0.0, 0.0), 40, 1))
    checks = [('a: v2 PASS', a['v2']['verdict'] == 'PASS'),
              ('a: v1 FAIL (noise alone)', a['v1']['verdict'] == 'FAIL'),
              ('a: counterfactuals as expected', a['counterfactual_all_as_expected'])]
    b = run(spec_path, world, synth(spec, world, template, (0.003, 0.0, 0.0), 40, 2))
    checks.append(('b: body displaced 3 mm, pose/info at the weld -> FAIL',
                   b['v2']['verdict'] == 'FAIL'))
    c = run(spec_path, world, synth(spec, world, template, (0.0, 0.0, 0.0), 30, 3))
    checks.append(('c: 30 scans -> INCONCLUSIVE', c['v2']['verdict'] == 'INCONCLUSIVE'))
    for name, good in checks:
        print(('PASS ' if good else 'FAIL ') + name)
        ok = ok and good
    print(f"a: |dxy| {1e3 * a['v2']['dxy_m']:.3f} mm, sigma used "
          f"{[round(1e3 * s, 3) for s in a['v2']['sigma_used_m_m_rad']]}; b: |dxy| "
          f"{1e3 * b['v2']['dxy_m']:.3f} mm")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
