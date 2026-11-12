#!/usr/bin/env python3
"""Read-only pre-shadow settle check for the S3 shadow run (S3 design section 2 criteria, through the
S3 driver's own fresh_pose and Settle helpers, imported unchanged from scripts/s3/s3_driver.py).

Subscribes to /spiderx/locomotion/status only; sends nothing. PASS when every status reports SHADOW
mode, dispatch gate False and DISARMED, and >= 10 s of simulation time of FRESH samples (body pose
<= 1.0 s old, strictly later sim time) satisfy body z >= 0.045 m, z range <= 0.005 m, tilt <= 0.10
rad and graph 1/0/0, with the node's joint-state age <= 0.5 s and graph age <= 3.0 s in every
counted sample (a violation restarts the window). Records the window (standing height, range,
tilt), the body's drift within it, and the largest ages seen.
Usage: settle_check.py --out <json> [--timeout-s 900] [--domain-id 0]"""
import argparse
import json
import math
import sys
import time

sys.path.insert(0, '/home/user/spiderx_ws/scripts/s3')
import s3_driver as s3  # noqa: E402

JS_STALE_S = 0.5      # monitor.joint_states_stale_s
GRAPH_STALE_S = 3.0   # monitor.graph_stale_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout-s', type=float, default=900.0)
    ap.add_argument('--domain-id', type=int, default=0)
    a = ap.parse_args()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=a.domain_id)
    node = rclpy.create_node('s3_shadow_settle_check', context=ctx)
    q = []
    node.create_subscription(String, s3.s2.STATUS_TOPIC, lambda m: q.append(m.data), 10)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    settle, last, rows, viol = s3.Settle(), None, [], []
    n_status = n_fresh = 0
    modes, max_age = set(), {'joint_states_age_s': 0.0, 'body_pose_age_s': 0.0, 'graph_age_s': 0.0}
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.timeout_s and settle.done is None:
        ex.spin_once(timeout_sec=0.1)
        while q and settle.done is None:
            st = json.loads(q.pop(0))
            n_status += 1
            modes.add((st.get('mode'), st.get('dispatch_gate'), st.get('state')))
            mon = st.get('monitors') or {}
            fp = s3.fresh_pose(st, last)
            if fp is None:
                continue
            gt, sim = fp
            last = sim
            n_fresh += 1
            for k in max_age:
                if isinstance(mon.get(k), (int, float)):
                    max_age[k] = max(max_age[k], mon[k])
            js, ga = mon.get('joint_states_age_s'), mon.get('graph_age_s')
            ok_mode = (st.get('mode') == 'shadow' and st.get('dispatch_gate') is False
                       and st.get('state') == 'DISARMED')
            ok_age = (isinstance(js, (int, float)) and js <= JS_STALE_S
                      and isinstance(ga, (int, float)) and ga <= GRAPH_STALE_S)
            if not (ok_mode and ok_age):
                viol.append({'sim': sim, 'mode': st.get('mode'), 'gate': st.get('dispatch_gate'),
                             'state': st.get('state'), 'joint_states_age_s': js,
                             'graph_age_s': ga})
                settle.window.clear()
                continue
            rows.append({'sim': sim, 'xy': list(gt['xy']), 'yaw': gt['yaw'], 'z': gt['z'],
                         'tilt': gt['tilt']})
            settle.add(sim, gt['z'], gt['tilt'], mon.get('graph'))
    res = {'criteria': 'z >= 0.045 m, z range <= 0.005 m, tilt <= 0.10 rad, graph 1/0/0, '
                       'joint-state age <= 0.5 s, pose age <= 1.0 s, graph age <= 3.0 s, sim '
                       'strictly advancing, over >= 10 s of sim; SHADOW, gate False, DISARMED',
           'statuses': n_status, 'fresh_samples': n_fresh, 'violations': viol[:20],
           'violation_count': len(viol), 'modes_seen': sorted(map(list, modes), key=str),
           'max_ages_s': max_age, 'window': settle.done,
           'wall_s': round(time.monotonic() - t0, 1)}
    if settle.done is not None:
        w = [r for r in rows if settle.done['sim_from_s'] <= r['sim'] <= settle.done['sim_to_s']]
        fwd, lat, dyaw = s3.body_delta(w[0], w[-1])
        span = w[-1]['sim'] - w[0]['sim']
        res['drift_in_window'] = {
            'sim_span_s': span, 'forward_m': fwd, 'lateral_m': lat, 'yaw_rad': dyaw,
            'forward_per_level0_cycle_m': fwd / span * s3.CYCLE_S0 if span > 0 else None,
            'max_abs_xy_step_m': max(math.hypot(b['xy'][0] - a_['xy'][0], b['xy'][1] - a_['xy'][1])
                                     for a_, b in zip(w, w[1:])) if len(w) > 1 else None}
    ok = (settle.done is not None and modes == {('shadow', False, 'DISARMED')})
    res['verdict'] = 'PASS' if ok else 'FAIL'
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True)
    print(json.dumps(res, sort_keys=True))
    print('SETTLE VERDICT:', res['verdict'])
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown(context=ctx)


if __name__ == '__main__':
    main()
