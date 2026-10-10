#!/usr/bin/env python3
"""List the /scan beams whose mean range differs from the weld-implied expectation by > 2 cm, and
recompute the robust (median/MAD) residual spread without them (evidence helper; read-only).
    python3 scan_outliers.py capture.json world.sdf OUT_JSON"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scan_check as sc  # noqa: E402

cap = json.load(open(sys.argv[1]))
scans = cap['scans']
s0 = scans[0]
n = len(s0['ranges'])
angles = [s0['angle_min'] + i * s0['angle_increment'] for i in range(n)]
R = np.array([[np.nan if r is None else r for r in s['ranges']] for s in scans])
valid = np.isfinite(R) & (R > s0['range_min']) & (R < s0['range_max'])
cnt = valid.sum(0)
mean = np.where(cnt > 0, np.nansum(np.where(valid, R, 0), 0) / np.maximum(cnt, 1), np.nan)
std = np.array([np.std(R[valid[:, i], i]) if cnt[i] > 1 else np.nan for i in range(n)])
lx, ly, lz, lyaw = sc.lidar_pose([0.0, 0.0, 0.125, 0.0])
shapes = sc.world_shapes(sys.argv[2], lz)
rows, res_ok = [], []
for i in range(n):
    exp, who = sc.ray(shapes, lx, ly, lyaw + angles[i])
    dm = sc.ray(shapes, lx, ly, lyaw + angles[i] - math.radians(0.5))[0]
    dp = sc.ray(shapes, lx, ly, lyaw + angles[i] + math.radians(0.5))[0]
    grazing = abs(dm - exp) > 0.05 or abs(dp - exp) > 0.05
    r = mean[i] - exp if np.isfinite(mean[i]) and np.isfinite(exp) else float('nan')
    if np.isfinite(r) and abs(r) > 0.02:
        rows.append({'beam': i, 'angle_lidar_deg': round(math.degrees(angles[i]), 1),
                     'bearing_world_deg': round(math.degrees(lyaw + angles[i]), 1),
                     'expected_m': round(float(exp), 4), 'measured_mean_m': round(float(mean[i]), 4),
                     'measured_std_m': round(float(std[i]), 4), 'valid_scans': int(cnt[i]),
                     'surface': who, 'edge_grazing': grazing})
    elif np.isfinite(r) and not grazing:
        res_ok.append(float(r))
res_ok = np.array(res_ok)
out = {'outliers_gt_2cm': rows, 'n_outliers': len(rows),
       'inliers': {'beams': int(res_ok.size), 'median_m': float(np.median(res_ok)),
                   'mad_m': float(np.median(np.abs(res_ok - np.median(res_ok)))),
                   'abs_p95_m': float(np.percentile(np.abs(res_ok), 95)),
                   'abs_max_m': float(np.max(np.abs(res_ok)))}}
json.dump(out, open(sys.argv[3], 'w'), indent=1)
print(json.dumps(out['inliers']), 'outliers', len(rows))
for r in rows:
    print(r)
