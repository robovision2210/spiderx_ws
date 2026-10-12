#!/usr/bin/env python3
"""Summarize the rehearsal trials (read-only, offline): one row per trial, plus the discovery
timelines of the new path (when the host node, the status publisher and the per-node server
first appeared) and the pre-fix replays' single query."""
import glob
import json
import os
import sys

run = sys.argv[1]
rows = []
for d in sorted(glob.glob(os.path.join(run, 'trials', '*'))):
    name = os.path.basename(d)
    k, mode, delay = name.split('_')
    row = {'trial': int(k), 'mode': mode, 'delay_s': int(delay[1:])}
    obs = sorted(glob.glob(os.path.join(d, 'observer', '*', 'observation.json')))
    if obs:
        o = json.load(open(obs[-1]))
        row['observer'] = {'ready': o.get('ready'), 'start': o['graph']['start'],
                           'end': o['graph']['end']}
    meta = os.path.join(d, 'bag_metadata.yaml')
    row['command_topic_in_bag'] = ('joint_trajectory' in open(meta).read()) if os.path.exists(meta) else None
    if mode == 'new':
        (f,) = glob.glob(os.path.join(d, 'rehearsal', '*', 'rehearsal.json'))
        r = json.load(open(f))
        disc = r['discovery']
        first = {}
        for p in disc['polls']:
            for key, cond in (('host_node', p['host_node_visible']),
                              ('status_publisher', bool(p['status_publishers'])),
                              ('client_matched', bool(p['client_matched'])),
                              ('per_node_server', p['servers'] > 0)):
                if cond and key not in first:
                    first[key] = p['t_s']
        row.update(verdict=r['verdict'], stopped_at=r['stopped_at'],
                   discovery_outcome=disc['outcome'], found_after_s=disc['found_after_s'],
                   node_age_at_discovery_start_s=round(disc['started_monotonic']
                                                       - r['transport_opened_monotonic'], 6),
                   first_seen_in_polls_s=first, polls=len(disc['polls']),
                   failure_codes=[s['result']['failure_codes'] for s in r['steps']],
                   streams_ok=(r['streams_at_send'] or {}).get('ok'),
                   age_at_send_s=(r['age_check_at_send'] or {}).get('age_s'),
                   goals_sent=r['goals_sent'], dispatch_calls=r['dispatch_calls'],
                   gate=r['gates']['m61_live_dispatch_enabled'])
    else:
        r = json.load(open(os.path.join(d, 'old_path.json')))
        q = r['action_queries']
        row.update(permitted=r['permitted'], code=r['code'],
                   failure_codes=r['result']['failure_codes'],
                   first_query_node_age_s=r['first_query_node_age_s'],
                   first_query_servers=q[0]['servers'] if q else None,
                   checks={k: v for k, v in r['result']['report']['checks'].items()},
                   after_found_after_s=r['discovery_after_readiness']['found_after_s'],
                   goals_sent=r['goals_sent'], dispatch_calls=r['dispatch_calls'])
    rows.append(row)
json.dump(rows, open(os.path.join(run, 'trials_summary.json'), 'w'), indent=1)
print(f"{'#':>2} {'mode':4} {'d':>2}  result")
for r in rows:
    if r['mode'] == 'new':
        print(f"{r['trial']:2d} new  {r['delay_s']:2d}  {r['verdict']:9s} discovery {r['discovery_outcome']} "
              f"after {r['found_after_s']} s (node age at start {r['node_age_at_discovery_start_s']} s); "
              f"first seen {r['first_seen_in_polls_s']}; codes {r['failure_codes']}; streams {r['streams_ok']}; "
              f"age at send {r['age_at_send_s']}; goals {r['goals_sent']}; dispatch calls {r['dispatch_calls']}")
    else:
        print(f"{r['trial']:2d} old  {r['delay_s']:2d}  permitted {r['permitted']} ({r['code']}); codes "
              f"{r['failure_codes']}; single query at node age {r['first_query_node_age_s']} s saw "
              f"{r['first_query_servers']}; goals {r['goals_sent']}; dispatch calls {r['dispatch_calls']}")
    o = r.get('observer') or {}
    print(f"      observer ready {o.get('ready')} start {o.get('start', {}).get('command_publishers')}/"
          f"{o.get('start', {}).get('action_clients')} end {o.get('end', {}).get('command_publishers')}/"
          f"{o.get('end', {}).get('action_clients')} (cmd pubs/action clients); command topic in bag {r['command_topic_in_bag']}")
