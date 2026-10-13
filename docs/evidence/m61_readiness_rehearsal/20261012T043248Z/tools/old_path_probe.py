#!/usr/bin/env python3
"""READ-ONLY diagnostic: the readiness composition of the refused attempt (a1a7d60), replayed.

It opens the production M6.1-A transport as m61_readiness_rehearsal.RehearsalTransport (send and
cancel raise; nothing can leave), does what --live did at a1a7d60 before readiness #1 (the
offline preflight), then runs readiness #1 exactly as composed there: the M6.0-B observation
(GraphProbe: a fixed 2.0 s spin, then ONE per-node action-server query) and the four M6.1
layers, WITHOUT the bounded discovery added at 61f3ffc. Every action-server query is timed in
this process (diagnostic instrumentation only). Afterwards the same node runs the bounded
discovery (m61_discovery) to record when it first sees the server. No goal, no cancel, no
session, no publisher; no gate is read or changed.
"""
import argparse
import json
import time

from spiderx_controller import m6_envelope as env
from spiderx_controller import m6_gait_replay as gr
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_preflight as lpf
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m61_discovery as dsc
from spiderx_controller import m61_evidence as evd
from spiderx_controller import m61_goal as goal61
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller.m61_readiness_rehearsal import RehearsalTransport

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--domain-id', type=int, required=True)
a = ap.parse_args()
t_proc = time.monotonic()
plan = tc.load_plan()
traj = tc.build_trajectory(plan)
fp = gf.fingerprint(goal61.approved_spec(traj, plan))
cfg = gr.load_fixed_base_config(plan.config_dir)

queries = []
original = lpf.GraphProbe.action_servers


def timed(self):
    t = time.monotonic()
    try:
        r = original(self)
        queries.append({'monotonic': round(t, 6), 'servers': r, 'error': None})
        return r
    except Exception as e:  # noqa: BLE001
        queries.append({'monotonic': round(t, 6), 'servers': None,
                        'error': f'{type(e).__name__}: {e}'})
        raise


lpf.GraphProbe.action_servers = timed
t_open0 = time.monotonic()
transport = RehearsalTransport(fp, a.domain_id, cfg, node_name='m61_old_path_probe').open()
t_open = time.monotonic()
rec = {'schema': 'spiderx.m61.old_path_probe/1', 'composition': 'a1a7d60 (no discovery wait)',
       'domain_id': a.domain_id, 'process_start_monotonic': round(t_proc, 6),
       'transport_open_started_monotonic': round(t_open0, 6),
       'transport_opened_monotonic': round(t_open, 6), 'goals_sent': 0, 'cancels_sent': 0}
try:
    tc.preflight(traj, plan)                   # what the session did before readiness #1
    base = rd.make_readiness_provider(transport.graph_collector(), plan.sources.joint_names,
                                      plan.neutral, transport.wall_now)
    provide = gr.with_command_owner(gr.with_sim_progress(gr.with_fixed_base(
        gr.with_body_pose(base, transport.latest_body_pose, transport.wall_now, plan.limits),
        transport.fixed_base_snapshot, cfg), transport.fixed_base_snapshot, transport.wall_now,
        cfg), transport.command_owner_snapshot)
    t_req = time.monotonic()
    res = provide()
    t_done = time.monotonic()
    permitted, code = rd.dispatch_permitted(res, t_done)
    lpf.GraphProbe.action_servers = original
    rec.update({
        'readiness_requested_monotonic': round(t_req, 6),
        'readiness_completed_monotonic': round(t_done, 6),
        'result': dict(res.to_dict(t_done), report=evd.json_safe(res.report)),
        'permitted': permitted, 'code': code,
        'action_queries': evd.json_safe(queries),
        'first_query_node_age_s': (round(queries[0]['monotonic'] - t_open, 6)
                                   if queries else None),
        'action_server_missing': 'action_server_missing' in res.failure_codes})
    after = dsc.wait_for_action_server(transport.action_discovery_snapshot, transport.spin_for,
                                       transport.wall_now, env.ACTION_NAME,
                                       domain_id=a.domain_id)
    rec['discovery_after_readiness'] = after
    if after['outcome'] == dsc.FOUND:
        rec['server_first_seen_node_age_s'] = round(
            after['started_monotonic'] + after['found_after_s'] - t_open, 6)
finally:
    lpf.GraphProbe.action_servers = original
    rec['dispatch_calls'] = transport.dispatch_calls
    transport.close()
with open(a.out, 'x') as f:
    f.write(evd.dumps(evd.json_safe(rec)))
print(f"old path: permitted {rec.get('permitted')} ({rec.get('code')}); failure codes "
      f"{(rec.get('result') or {}).get('failure_codes')}; first query at node age "
      f"{rec.get('first_query_node_age_s')} s; server first seen at node age "
      f"{rec.get('server_first_seen_node_age_s')} s; dispatch calls {rec['dispatch_calls']}")
print(json.dumps({'action_queries': [{'servers': q['servers'], 'error': q['error']}
                                     for q in queries]}))
