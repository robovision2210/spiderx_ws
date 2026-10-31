#!/usr/bin/env python3
"""Level-0 forward planned joint margins (S3 prerequisite P6), offline: no ROS graph, no motion.

Inputs, all unchanged: the M5.5 phase library exactly as the node builds it (load_config +
build_library), the URDF joint limits (joint_safety.load_limits, i.e. spiderx_legs.yaml, checked
here against the expanded URDF), and the 9 level-0 forward phase goals that S3 sends (one cycle
leaves boundaries 0..8; the estop exercise repeats the goal leaving boundary 0).

Two bases, both reported:
- waypoints: the dispatched trajectory points;
- interpolated: the trajectory controller's cubic Hermite segments between those points
  (positions and velocities), sampled every 1 ms of trajectory time.
The lead-in (measured state -> first point over phase_lead_s) is planned from the measured state
at dispatch, not from the library, so it is excluded; continuity at dispatch is <= 0.02 rad.

Usage: level0_joint_margins.py [--out <json>]"""
import argparse
import json
import math
import sys

STEP_S = 0.001


def hermite(p0, v0, p1, v1, dt, s):
    """Cubic Hermite position at fraction s of a segment of duration dt."""
    s2, s3 = s * s, s * s * s
    return ((2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * dt * v0 +
            (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * dt * v1)


def goal_samples(goal, step_s=STEP_S):
    """[(kind, t, {joint: q})]: every waypoint, then the interpolation between waypoints."""
    out = []
    pts = goal.points
    for t, q, _ in pts:
        out.append(('waypoint', t, dict(zip(goal.joint_names, q))))
    for (t0, q0, v0), (t1, q1, v1) in zip(pts, pts[1:]):
        dt = t1 - t0
        n = max(1, int(math.ceil(dt / step_s)))
        for k in range(1, n):
            s = k / n
            out.append(('interpolated', t0 + s * dt,
                        {j: hermite(a, va, b, vb, dt, s)
                         for j, a, va, b, vb in zip(goal.joint_names, q0, v0, q1, v1)}))
    return out


def margins(goals, limits):
    """Per joint and basis: the minimum distance to the lower and upper URDF limit."""
    res = {}
    for b, goal in goals:
        for kind, t, q in goal_samples(goal):
            for j, x in q.items():
                lo, hi = limits[j]
                for side, d in (('lower', x - lo), ('upper', hi - x)):
                    for basis in ('waypoints', 'interpolated'):
                        if basis == 'waypoints' and kind != 'waypoint':
                            continue
                        r = res.setdefault(j, {}).setdefault(basis, {})
                        if side not in r or d < r[side]['margin_rad']:
                            r[side] = {'margin_rad': d, 'goal_leaving_boundary': b,
                                       't_s': round(t, 4), 'q_rad': x}
    for j, bases in res.items():
        for basis, r in bases.items():
            r['min_rad'] = min(r['lower']['margin_rad'], r['upper']['margin_rad'])
            r['nearest'] = 'lower' if r['lower']['margin_rad'] <= r['upper']['margin_rad'] \
                else 'upper'
    return res


def urdf_limits(root):
    out = {}
    for j in root.iter('joint'):
        lim = j.find('limit')
        if lim is not None and lim.get('lower') is not None:
            out[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
    return out


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    a.add_argument('--out')
    o = a.parse_args(argv)
    from spiderx_controller import config_check as cc
    from spiderx_controller import joint_safety as js
    from spiderx_controller import m55_locomotion as loc
    limits, order = js.load_limits()
    urdf = urdf_limits(cc.load_urdf())
    mismatch = {j: (limits[j], urdf.get(j)) for j in order if urdf.get(j) is None
                or any(abs(x - y) > 1e-9 for x, y in zip(limits[j], urdf[j]))}
    if mismatch:
        print(f'URDF and spiderx_legs.yaml limits differ: {mismatch}', file=sys.stderr)
        return 1
    cfg = loc.load_config()
    lib = loc.build_library(cfg)
    goals = [(b, lib.goal(0, b, 1)) for b in range(lib.n_phases)]
    res = margins(goals, limits)
    overall = {basis: min(((res[j][basis]['min_rad'], j) for j in order)) for basis in
               ('waypoints', 'interpolated')}
    report = {
        'scope': 'level 0, forward, the 9 phase goals S3 sends (lead-in excluded)',
        'library_goals': len(lib.fingerprints), 'phase_lead_s': lib.lead_s,
        'limits_source': 'joint_safety.load_limits (spiderx_legs.yaml) == expanded URDF',
        'interpolation': f'cubic Hermite (positions + velocities), {STEP_S * 1000:.0f} ms',
        'joints': {j: res[j] for j in order},
        'overall_min': {k: {'margin_rad': v[0], 'joint': v[1]} for k, v in overall.items()},
        'stops': {'joint_limit_stop_rad': 0.02, 'tracking_abort_rad': 0.05},
    }
    if o.out:
        with open(o.out, 'w') as f:
            json.dump(report, f, indent=1, sort_keys=True)
    print(f"{'joint':16} {'lower':>10} {'upper':>10} {'min (interp)':>13} {'nearest':>8} "
          f"{'min (wayp.)':>12}")
    for j in order:
        r, w = res[j]['interpolated'], res[j]['waypoints']
        print(f"{j:16} {r['lower']['margin_rad']:10.4f} {r['upper']['margin_rad']:10.4f} "
              f"{r['min_rad']:13.4f} {r['nearest']:>8} {w['min_rad']:12.4f}")
    for k, v in report['overall_min'].items():
        print(f'overall minimum ({k}): {v["margin_rad"]:.4f} rad at {v["joint"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
