#!/usr/bin/env python3
"""Read-only diagnosis of the readiness refusal `action_server_missing`.

Replicates the M6.0-D GraphProbe.action_servers() query (rclpy.action.
get_action_server_names_and_types_by_node over get_node_names_and_namespaces) from a FRESH node,
every 0.25 s for up to --seconds, and records when the action server first appears. The probe
itself queries once, after a fixed 2.0 s discovery spin. No action client, no publisher, no
goal. Each trial uses its own context and node."""
import argparse
import json
import time

import rclpy
from rclpy.action import get_action_server_names_and_types_by_node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

ACTION = '/leg_trajectory_controller/follow_joint_trajectory'

ap = argparse.ArgumentParser()
ap.add_argument('--trials', type=int, default=3)
ap.add_argument('--seconds', type=float, default=15.0)
ap.add_argument('--out', required=True)
a = ap.parse_args()
results = []
for k in range(a.trials):
    ctx = Context()
    rclpy.init(context=ctx, domain_id=0, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(f'm61_action_discovery_probe_{k}', context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    t0 = time.monotonic()
    nxt, first, log = 0.0, None, []
    while time.monotonic() - t0 < a.seconds:
        ex.spin_once(timeout_sec=0.05)
        t = time.monotonic() - t0
        if t < nxt:
            continue
        nxt += 0.25
        names = node.get_node_names_and_namespaces()
        hit = []
        for n, ns in names:
            try:
                acts = get_action_server_names_and_types_by_node(node, n, ns)
            except Exception as e:  # noqa: BLE001 - recorded
                acts = [('error', [f'{type(e).__name__}: {e}'])]
            hit += [(n, ns, list(ty)) for act, ty in acts if act == ACTION]
        log.append({'t_s': round(t, 3), 'nodes': len(names), 'server_seen': bool(hit)})
        if hit and first is None:
            first = {'t_s': round(t, 3), 'by': hit}
    seen_at_2s = any(e['server_seen'] for e in log if e['t_s'] <= 2.05)
    results.append({'trial': k, 'first_seen': first, 'seen_by_2_0_s': seen_at_2s, 'log': log})
    print(f'trial {k}: first seen {first["t_s"] if first else None} s; seen by 2.0 s: {seen_at_2s}; '
          f'nodes at 2.0 s: {[e["nodes"] for e in log if e["t_s"] <= 2.05][-1:]}')
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown(context=ctx)
    time.sleep(1.0)
with open(a.out, 'w') as f:
    json.dump(results, f, indent=1)
