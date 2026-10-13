#!/usr/bin/env python3
"""Criterion 6, version 2 (PROPOSAL, frozen before its validation run): /scan pose cross-check.

Evaluates ONE capture (tools/capture.py output: 40 scans + the pose/info samples of the same
window) against the frozen definition in crit6_v2_spec.json, and reports version 1 alongside:

  v1 (original M6.1-A section 10 criterion 6, reported unchanged): every individual range agrees
     with the welded pose within the M0 +-8 mm - per single-scan range and per 40-scan mean; plus
     the smallest max |residual| any planar pose can reach (minimax), which shows whether ANY pose
     could satisfy v1;
  v2 (proposal): the body planar pose (x, y, yaw) fitted to the 40-scan per-beam means of the
     frozen beam set, compared with the body pose that pose/info reports in the same window.
     PASS needs the prerequisites (data, resolving power) and |dxy| <= tau_xy, |dyaw| <= tau_yaw;
     a missing prerequisite is INCONCLUSIVE, never PASS;
  counterfactual: the measured per-beam error pattern re-applied to a body displaced by each
     frozen offset; the same fit and rule must recover the offset and FAIL every offset at or
     above the G8 tolerance.
    python3 crit6_v2.py SPEC CAPTURE WORLD_SDF --out result.json
Needs numpy. Uses the ray caster of observation_7f4c30f/tools/scan_check.py (frozen, imported by
path from the spec). Nothing here touches ROS or the simulator.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os

import numpy as np


def load_ray_caster(path):
    spec = importlib.util.spec_from_file_location('scan_check_frozen', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def quat_rot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def reported_body(capture, spec):
    """The body pose pose/info reports: T_world_model * T_model_dummy * T_dummy_body (identity),
    per captured sample; returns the planar (x, y, yaw) mean and the full 6-DOF spread."""
    rows = []
    for p in capture['poses']:
        t = {e['child']: e for e in p['transforms']}
        m, d = t.get(spec['model_name']), t.get(spec['body_link'])
        if m is None or d is None:
            continue
        pos = tuple(a + b for a, b in zip(m['xyz'], quat_rot(m['q'], d['xyz'])))
        q = quat_mul(m['q'], d['q'])
        x, y, z, w = q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        rows.append((pos[0], pos[1], pos[2], roll, pitch, quat_yaw(q)))
    a = np.array(rows)
    return {'samples': len(rows), 'mean_xyz_rpy': a.mean(0).tolist(),
            'range_xyz_rpy': (a.max(0) - a.min(0)).tolist()}


class Scene:
    def __init__(self, sc, world, spec, scan0):
        self.sc = sc
        self.lb = spec['lidar_in_body_xyz_yaw']
        self.z_plane = spec['body_z_for_scan_plane_m'] + self.lb[2]
        self.shapes = sc.world_shapes(world, self.z_plane)
        n = len(scan0['ranges'])
        self.ang = np.array([scan0['angle_min'] + i * scan0['angle_increment']
                             for i in range(n)])

    def lidar(self, p):
        xb, yb, yb_yaw = p
        lx, ly, _, lyaw = self.lb
        return (xb + math.cos(yb_yaw) * lx - math.sin(yb_yaw) * ly,
                yb + math.sin(yb_yaw) * lx + math.cos(yb_yaw) * ly, yb_yaw + lyaw)

    def predict(self, p, idx):
        x, y, yaw = self.lidar(p)
        return np.array([self.sc.ray(self.shapes, x, y, yaw + self.ang[i])[0] for i in idx])

    def surfaces(self, p, idx):
        x, y, yaw = self.lidar(p)
        return [self.sc.ray(self.shapes, x, y, yaw + self.ang[i])[1] for i in idx]

    def jacobian(self, p, idx, h=1e-5):
        base = self.predict(p, idx)
        J = np.zeros((len(idx), 3))
        for k in range(3):
            d = np.zeros(3)
            d[k] = h
            J[:, k] = (self.predict(np.asarray(p) + d, idx) - base) / h
        return J


def fit(scene, mean, idx, p0, f):
    """Robust Gauss-Newton of the body planar pose on a FIXED beam set (no re-selection)."""
    p = np.array(p0, float)
    it = 0
    for it in range(1, f['max_iterations'] + 1):
        r0 = mean[idx] - scene.predict(p, idx)
        w = np.where(np.abs(r0) <= f['huber_m'], 1.0, f['huber_m'] / np.maximum(np.abs(r0), 1e-12))
        J = scene.jacobian(p, idx)
        sw = np.sqrt(w)
        step, *_ = np.linalg.lstsq(J * sw[:, None], r0 * sw, rcond=None)
        p = p + np.clip(step, -f['step_clip'], f['step_clip'])
        if np.max(np.abs(step)) < f['converged_step']:
            break
    r = mean[idx] - scene.predict(p, idx)
    J = scene.jacobian(p, idx)
    s = 1.4826 * float(np.median(np.abs(r - np.median(r))))
    sig = np.sqrt(np.diag(np.linalg.inv(J.T @ J))) * s
    return p, sig, r, s, it


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def decide(d, sig, prereq_ok, spec):
    t = spec['thresholds']
    dxy = math.hypot(d[0], d[1])
    ok_xy, ok_yaw = dxy <= t['tau_xy_m'], abs(d[2]) <= t['tau_yaw_rad']
    if not prereq_ok:
        verdict = 'INCONCLUSIVE'
    else:
        verdict = 'PASS' if (ok_xy and ok_yaw) else 'FAIL'
    return {'dxy_m': dxy, 'dyaw_rad': d[2], 'within_tau_xy': ok_xy, 'within_tau_yaw': ok_yaw,
            'verdict': verdict}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('spec')
    ap.add_argument('capture')
    ap.add_argument('world')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    spec = json.load(open(a.spec))
    here = os.path.dirname(os.path.abspath(a.spec))
    sc_path = os.path.normpath(os.path.join(here, spec['ray_caster']['path']))
    if sha256(sc_path) != spec['ray_caster']['sha256']:
        raise SystemExit(f'ray caster {sc_path} differs from the frozen SHA-256')
    sc = load_ray_caster(sc_path)
    cap = json.load(open(a.capture))
    scans = cap['scans']
    s0 = scans[0]
    R = np.array([[np.nan if r is None else r for r in s['ranges']] for s in scans])
    valid = np.isfinite(R) & (R > s0['range_min']) & (R < s0['range_max'])
    Rv = np.where(valid, R, np.nan)
    with np.errstate(invalid='ignore', divide='ignore'):
        mean = np.nanmean(Rv, 0)
    scene = Scene(sc, a.world, spec, s0)
    n = len(s0['ranges'])
    excl = set(spec['beam_exclusions']['seam_indices']) | \
        set(spec['beam_exclusions']['edge_grazing_indices'])
    frozen = [i for i in range(n) if i not in excl]
    idx = np.array([i for i in frozen if np.isfinite(mean[i])])
    weld = spec['weld_body_xy_yaw']
    who_all = scene.surfaces(weld, frozen)
    who = dict(zip(frozen, who_all))
    rep = reported_body(cap, spec)
    ref = [rep['mean_xyz_rpy'][0], rep['mean_xyz_rpy'][1], rep['mean_xyz_rpy'][5]]

    # ---------------- v1, reported unchanged (individual ranges vs the welded pose, +-8 mm)
    exp_weld = scene.predict(weld, frozen)
    single = (Rv[:, frozen] - exp_weld[None, :])
    single = single[np.isfinite(single)]
    res_mean_weld = mean[frozen] - exp_weld
    res_mean_weld = res_mean_weld[np.isfinite(res_mean_weld)]
    tol = spec['v1']['tolerance_m']
    v1 = {'wording': spec['v1']['wording'],
          'single_scan_ranges': {'count': int(single.size),
                                 'within_fraction': float(np.mean(np.abs(single) <= tol)),
                                 'max_abs_m': float(np.max(np.abs(single)))},
          'per_beam_40_scan_means': {'count': int(res_mean_weld.size),
                                     'within_fraction': float(np.mean(np.abs(res_mean_weld) <= tol)),
                                     'max_abs_m': float(np.max(np.abs(res_mean_weld)))}}
    # minimax: the smallest max |residual| of the 40-scan means any planar pose reaches
    J0 = scene.jacobian(weld, idx)
    r0 = mean[idx] - scene.predict(weld, idx)
    best = (float(np.max(np.abs(r0))), np.zeros(3))
    g = np.linspace(-1, 1, 41)
    for scale in (0.01, 0.002, 0.0004):
        c = best[1]
        for gx in g:
            for gy in g:
                P = np.stack([np.full(g.size, c[0] + gx * scale), np.full(g.size, c[1] + gy * scale),
                              c[2] + g * scale * 2], 1)
                m = np.max(np.abs(r0[:, None] - J0 @ P.T), 0)
                k = int(np.argmin(m))
                if m[k] < best[0]:
                    best = (float(m[k]), P[k].copy())
    pm = np.asarray(weld) + best[1]
    exact = float(np.max(np.abs(mean[idx] - scene.predict(pm, idx))))
    v1['minimax_over_planar_pose'] = {'max_abs_residual_m': exact,
                                      'at_body_offset_m_m_rad': best[1].tolist()}
    v1['verdict'] = 'PASS' if v1['per_beam_40_scan_means']['within_fraction'] == 1.0 and \
        v1['single_scan_ranges']['within_fraction'] == 1.0 else 'FAIL'

    # ---------------- v2
    f = spec['fit']
    p, sig_r, r, s, iters = fit(scene, mean, idx, ref, f)
    names = sorted({who[i] for i in idx})
    jk = []
    for nm in names:
        keep = np.array([i for i in idx if who[i] != nm])
        jk.append(fit(scene, mean, keep, p, f)[0])
    jk = np.array(jk)
    ns = len(names)
    sig_j = np.sqrt((ns - 1) / ns * np.sum((jk - jk.mean(0)) ** 2, 0))
    sig = np.maximum(sig_r, sig_j)
    pre = spec['prerequisites']
    walls = {w: sum(1 for i in idx if who[i] == w) for w in pre['walls']}
    checks = {
        'scans_averaged': (len(scans) >= pre['min_scans'], len(scans)),
        'frozen_beams_finite': (len(idx) >= pre['min_finite_fraction'] * len(frozen),
                                f'{len(idx)}/{len(frozen)}'),
        'beams_per_wall': (all(v >= pre['min_beams_per_wall'] for v in walls.values()), walls),
        'pose_info_constant_in_window': (
            max(rep['range_xyz_rpy'][:2]) <= pre['max_reported_planar_spread_m'] and
            rep['range_xyz_rpy'][5] <= pre['max_reported_yaw_spread_rad'], rep['range_xyz_rpy']),
        'resolving_power_xy': (max(sig[0], sig[1]) <= spec['thresholds']['sigma_max_xy_m'],
                               max(sig[0], sig[1])),
        'resolving_power_yaw': (sig[2] <= spec['thresholds']['sigma_max_yaw_rad'], sig[2]),
    }
    prereq_ok = all(v[0] for v in checks.values())
    d = [p[0] - ref[0], p[1] - ref[1], wrap(p[2] - ref[2])]
    dw = [p[0] - weld[0], p[1] - weld[1], wrap(p[2] - weld[2])]
    v2 = {'version': spec['version'], 'reference': 'pose/info composed body (same window)',
          'reported_body': rep, 'fit_body_xy_yaw': p.tolist(), 'iterations': iters,
          'beams_used': int(len(idx)), 'beam_robust_sigma_m': s,
          'sigma_robust_m_m_rad': sig_r.tolist(), 'sigma_jackknife_m_m_rad': sig_j.tolist(),
          'jackknife_groups': names, 'sigma_used_m_m_rad': sig.tolist(),
          'prerequisites': {k: {'ok': bool(v[0]), 'value': v[1]} for k, v in checks.items()},
          'prerequisites_ok': bool(prereq_ok),
          'fit_minus_reported_m_m_rad': d, 'fit_minus_weld_m_m_rad': dw}
    v2.update(decide(d, sig, prereq_ok, spec))

    # ---------------- counterfactual: the measured error pattern on a displaced body
    e = mean[idx] - scene.predict(p, idx)
    cf = []
    for off in spec['counterfactual_offsets_m_m_rad']:
        q = p + np.asarray(off, float)
        synth = np.full(n, np.nan)
        synth[idx] = scene.predict(q, idx) + e
        pc, sc_sig, _, _, _ = fit(scene, synth, idx, ref, f)
        dc = [pc[0] - ref[0], pc[1] - ref[1], wrap(pc[2] - ref[2])]
        rec = decide(dc, sig, prereq_ok, spec)
        err = (pc - p) - np.asarray(off)
        at_or_above = (math.hypot(off[0], off[1]) >= spec['objective']['delta_xy_m'] - 1e-12 or
                       abs(off[2]) >= spec['objective']['delta_yaw_rad'] - 1e-12)
        rec.update(offset_m_m_rad=off, recovered_minus_fit_m_m_rad=(pc - p).tolist(),
                   recovery_error_m_m_rad=err.tolist(),
                   recovery_within_3_sigma=bool(np.all(np.abs(err) <= 3 * sig)),
                   expected_verdict='FAIL' if at_or_above else None)
        cf.append(rec)
    as_expected = all(c['recovery_within_3_sigma'] and
                      (c['expected_verdict'] is None or c['verdict'] == c['expected_verdict'])
                      for c in cf)
    out = {'schema': 'spiderx.m61a.criterion6_v2_result/1', 'spec_sha256': sha256(a.spec),
           'capture_sha256': sha256(a.capture), 'world_sha256': sha256(a.world),
           'tool_sha256': sha256(os.path.abspath(__file__)), 'v1': v1, 'v2': v2,
           'counterfactual': cf, 'counterfactual_all_as_expected': as_expected}
    json.dump(out, open(a.out, 'w'), indent=1)
    print(f"v1 (original, reported): {v1['verdict']}: single-scan ranges within +-8 mm "
          f"{100 * v1['single_scan_ranges']['within_fraction']:.1f} %, 40-scan means "
          f"{100 * v1['per_beam_40_scan_means']['within_fraction']:.1f} %, best any planar pose "
          f"can reach: max |r| {1e3 * exact:.1f} mm")
    print(f"v2 ({spec['version']}): {v2['verdict']}: |dxy| {1e3 * v2['dxy_m']:.2f} mm "
          f"(tau {1e3 * spec['thresholds']['tau_xy_m']:.2f}), |dyaw| "
          f"{1e3 * abs(v2['dyaw_rad']):.3f} mrad (tau {1e3 * spec['thresholds']['tau_yaw_rad']:.1f}); "
          f"sigma used {1e3 * max(sig[0], sig[1]):.3f} mm / {1e3 * sig[2]:.3f} mrad; "
          f"prerequisites {'met' if prereq_ok else 'NOT met'}")
    for c in cf:
        print(f"  counterfactual {c['offset_m_m_rad']}: {c['verdict']} (expected "
              f"{c['expected_verdict'] or 'any; recovery only'}), recovery error "
              f"{[round(1e3 * v, 3) for v in c['recovery_error_m_m_rad']]} (mm, mm, mrad), "
              f"within 3 sigma: {c['recovery_within_3_sigma']}")
    print(f"counterfactual all as expected: {as_expected}")


if __name__ == '__main__':
    main()
