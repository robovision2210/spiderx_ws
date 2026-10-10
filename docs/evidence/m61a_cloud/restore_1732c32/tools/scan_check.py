#!/usr/bin/env python3
"""Supplementary /scan cross-check for the M6.1-A launch-only observation (evidence helper).

Expected ranges: 2-D ray casting in the horizontal scan plane against the world's collision
geometry (spiderx_fortress.sdf: four walls, box_a (yaw 0.4), box_b, pillar), from the lidar pose
implied by the WELD: T_world_lidar = T_world_base (mount x, y, z, yaw) * T_base_lidar (URDF
lidar_joint 0.051, -0.045, 0.141, yaw +pi/2). Scan angle 0 = lidar +x = robot front.

Measured: the per-beam MEAN over N scans (sensor noise: gaussian sigma 0.01 m per beam).
Reports residual statistics per surface and a least-squares fit of the lidar's planar pose
(x, y, yaw). A horizontal scan of vertical walls cannot observe z, roll or pitch (beyond a
small tilt-induced lengthening), so it can confirm the planar pose only.
    python3 scan_check.py capture.json world.sdf --mount 0 0 0.125 0 --out scan_check.json
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

LIDAR_IN_BASE = (0.051, -0.045, 0.141, math.pi / 2)


def world_shapes(sdf_path, z_plane):
    w = ET.parse(sdf_path).getroot().find('world')
    shapes = []
    for m in w.findall('model'):
        mp = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
        for link in m.findall('link'):
            lp = [float(v) for v in (link.findtext('pose') or '0 0 0 0 0 0').split()]
            for c in link.findall('collision'):
                cp = [float(v) for v in (c.findtext('pose') or '0 0 0 0 0 0').split()]
                g = c.find('geometry')
                # compose model * link * collision (planar: this world has no roll or pitch)
                x, y, z, yaw = mp[0], mp[1], mp[2], mp[5]
                for q in (lp, cp):
                    x, y = (x + math.cos(yaw) * q[0] - math.sin(yaw) * q[1],
                            y + math.sin(yaw) * q[0] + math.cos(yaw) * q[1])
                    z, yaw = z + q[2], yaw + q[5]
                cx, cy, cz = x, y, z
                name = f"{m.get('name')}/{link.get('name')}"
                if g.find('box') is not None:
                    sx, sy, sz = [float(v) for v in g.find('box').findtext('size').split()]
                    if cz - sz / 2 <= z_plane <= cz + sz / 2:
                        shapes.append(('box', name, cx, cy, yaw, sx / 2, sy / 2))
                elif g.find('cylinder') is not None:
                    r = float(g.find('cylinder').findtext('radius'))
                    ln = float(g.find('cylinder').findtext('length'))
                    if cz - ln / 2 <= z_plane <= cz + ln / 2:
                        shapes.append(('cyl', name, cx, cy, r))
    return shapes


def ray(shapes, ox, oy, ang):
    dx, dy = math.cos(ang), math.sin(ang)
    best, who = math.inf, None
    for s in shapes:
        if s[0] == 'box':
            _, name, cx, cy, yaw, hx, hy = s
            c, sn = math.cos(-yaw), math.sin(-yaw)
            px, py = c * (ox - cx) - sn * (oy - cy), sn * (ox - cx) + c * (oy - cy)
            ux, uy = c * dx - sn * dy, sn * dx + c * dy
            tmin, tmax = -math.inf, math.inf
            ok = True
            for p, u, h in ((px, ux, hx), (py, uy, hy)):
                if abs(u) < 1e-12:
                    if abs(p) > h:
                        ok = False
                else:
                    t1, t2 = (-h - p) / u, (h - p) / u
                    tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if ok and tmax >= max(tmin, 0) and tmin > 0 and tmin < best:
                best, who = tmin, name
        else:
            _, name, cx, cy, r = s
            fx, fy = ox - cx, oy - cy
            b = fx * dx + fy * dy
            cc = fx * fx + fy * fy - r * r
            disc = b * b - cc
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if 0 < t < best:
                    best, who = t, name
    return best, who


def lidar_pose(mount):
    x, y, z, yaw = mount
    lx, ly, lz, lyaw = LIDAR_IN_BASE
    return (x + math.cos(yaw) * lx - math.sin(yaw) * ly,
            y + math.sin(yaw) * lx + math.cos(yaw) * ly, z + lz, yaw + lyaw)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('capture')
    p.add_argument('world')
    p.add_argument('--mount', type=float, nargs=4, default=[0.0, 0.0, 0.125, 0.0])
    p.add_argument('--out', required=True)
    p.add_argument('--exclude-seams', action='store_true',
                   help='drop beams within 1 deg of the gpu_lidar cube-map seams (lidar-frame '
                        '+-45, +-135 deg), where Gazebo renders a neighbouring direction')
    a = p.parse_args()
    cap = json.load(open(a.capture))
    scans = cap['scans']
    s0 = scans[0]
    n = len(s0['ranges'])
    angles = [s0['angle_min'] + i * s0['angle_increment'] for i in range(n)]
    seam = [a.exclude_seams and min(abs(math.degrees(math.atan2(math.sin(t - c), math.cos(t - c))))
                                    for c in (math.pi / 4, -math.pi / 4, 3 * math.pi / 4,
                                              -3 * math.pi / 4)) <= 1.0 for t in angles]
    R = np.array([[np.nan if r is None else r for r in s['ranges']] for s in scans])
    valid = np.isfinite(R) & (R > s0['range_min']) & (R < s0['range_max'])
    with np.errstate(invalid='ignore'):
        mean = np.where(valid.sum(0) > 0, np.nansum(np.where(valid, R, 0), 0) /
                        np.maximum(valid.sum(0), 1), np.nan)
    lx, ly, lz, lyaw = lidar_pose(a.mount)
    shapes = world_shapes(a.world, lz)

    def predict(x, y, yaw):
        return [ray(shapes, x, y, yaw + th) for th in angles]

    pred = predict(lx, ly, lyaw)
    exp = np.array([d for d, _ in pred])
    who = [w for _, w in pred]
    usable = np.isfinite(mean) & np.isfinite(exp) & (exp > s0['range_min'] + 0.05) & \
        (exp < s0['range_max'] - 0.05)
    # exclude beams grazing an edge: predicted range changes > 5 cm within +-0.5 deg
    for i in range(n):
        if seam[i]:
            usable[i] = False
        if usable[i]:
            dm = ray(shapes, lx, ly, lyaw + angles[i] - math.radians(0.5))[0]
            dp = ray(shapes, lx, ly, lyaw + angles[i] + math.radians(0.5))[0]
            if abs(dm - exp[i]) > 0.05 or abs(dp - exp[i]) > 0.05:
                usable[i] = False
    res = mean - exp
    per = {}
    for i in np.where(usable)[0]:
        per.setdefault(who[i], []).append(float(res[i]))
    surfaces = {k: {'beams': len(v), 'median_residual_m': float(np.median(v)),
                    'mean_residual_m': float(np.mean(v)), 'max_abs_residual_m':
                    float(np.max(np.abs(v)))} for k, v in sorted(per.items())}
    # least-squares fit of the lidar planar pose: Gauss-Newton with a numeric Jacobian; at every
    # iterate the beams are re-selected (finite, not grazing an edge at that pose) and weighted
    # with a Huber weight (2 cm), so a beam that switches surfaces cannot trap the fit
    def select(v):
        keep = []
        for i in range(n):
            if not np.isfinite(mean[i]) or seam[i]:
                continue
            d0 = ray(shapes, v[0], v[1], v[2] + angles[i])[0]
            dm = ray(shapes, v[0], v[1], v[2] + angles[i] - math.radians(0.5))[0]
            dp = ray(shapes, v[0], v[1], v[2] + angles[i] + math.radians(0.5))[0]
            if np.isfinite(d0) and abs(dm - d0) <= 0.05 and abs(dp - d0) <= 0.05 and                     s0['range_min'] + 0.05 < d0 < s0['range_max'] - 0.05:
                keep.append(i)
        return np.array(keep)

    def resid(v, sel):
        return np.array([mean[i] - ray(shapes, v[0], v[1], v[2] + angles[i])[0] for i in sel])
    x = np.array([lx, ly, lyaw])
    for _ in range(40):
        idx = select(x)
        r0 = resid(x, idx)
        w = np.where(np.abs(r0) <= 0.02, 1.0, 0.02 / np.maximum(np.abs(r0), 1e-12))
        J = np.zeros((len(idx), 3))
        for k in range(3):
            dv = np.zeros(3)
            dv[k] = 1e-4
            J[:, k] = -(resid(x + dv, idx) - r0) / 1e-4
        sw = np.sqrt(w)
        step, *_ = np.linalg.lstsq(J * sw[:, None], r0 * sw, rcond=None)   # r(x+s) ~ r0 - J s
        x = x + np.clip(step, -0.05, 0.05)
        if np.max(np.abs(step)) < 1e-7:
            break
    idx = select(x)
    rf = resid(x, idx)
    dof = max(1, len(idx) - 3)
    JtJ_inv = np.linalg.inv(J.T @ J)
    cov = JtJ_inv * float(rf @ rf) / dof
    robust_scale = 1.4826 * float(np.median(np.abs(rf - np.median(rf))))   # MAD -> sigma
    cov_robust = JtJ_inv * robust_scale ** 2
    out = {
        'scans_averaged': len(scans), 'beams': n, 'beams_used': int(usable.sum()),
        'frame_id': s0['frame_id'], 'angle_min': s0['angle_min'],
        'angle_increment': s0['angle_increment'],
        'mount_assumed_world_base_xyz_yaw': a.mount,
        'lidar_expected_world_xyz_yaw': [lx, ly, lz, lyaw],
        'scan_plane_z_m': lz, 'shapes_in_plane': [s[1] for s in shapes],
        'residual_mean_m': float(np.mean(res[usable])),
        'residual_median_m': float(np.median(res[usable])),
        'residual_abs_p95_m': float(np.percentile(np.abs(res[usable]), 95)),
        'residual_abs_max_m': float(np.max(np.abs(res[usable]))),
        'per_beam_expected_noise_m': 0.01 / math.sqrt(len(scans)),
        'surfaces': surfaces,
        'fit_lidar_world_xy_yaw': [float(v) for v in x],
        'fit_minus_expected_m_m_rad': [float(x[0] - lx), float(x[1] - ly),
                                       float(math.atan2(math.sin(x[2] - lyaw),
                                                        math.cos(x[2] - lyaw)))],
        'fit_1sigma_m_m_rad': [float(math.sqrt(cov[k, k])) for k in range(3)],
        'fit_1sigma_robust_m_m_rad': [float(math.sqrt(cov_robust[k, k])) for k in range(3)],
        'fit_residual_robust_sigma_m': robust_scale,
        'seam_beams_excluded': int(sum(seam)),
        'fit_residual_rms_m': float(math.sqrt(float(rf @ rf) / len(idx))),
        'fit_beams_used': int(len(idx)),
    }
    json.dump(out, open(a.out, 'w'), indent=1)
    d = out['fit_minus_expected_m_m_rad']
    print(f"scans {len(scans)} beams used {out['beams_used']}/{n}; residual median "
          f"{out['residual_median_m']*1000:+.1f} mm, |res| p95 "
          f"{out['residual_abs_p95_m']*1000:.1f} mm; fit - expected: dx {d[0]*1000:+.1f} mm "
          f"dy {d[1]*1000:+.1f} mm dyaw {d[2]*1000:+.2f} mrad "
          f"(1 sigma {out['fit_1sigma_m_m_rad'][0]*1000:.1f}/"
          f"{out['fit_1sigma_m_m_rad'][1]*1000:.1f} mm, "
          f"{out['fit_1sigma_m_m_rad'][2]*1000:.2f} mrad)")
    for k, v in surfaces.items():
        print(f"  {k:24s} beams {v['beams']:3d} median {v['median_residual_m']*1000:+6.1f} mm")


if __name__ == '__main__':
    main()
