#!/usr/bin/env python3
"""Analyse the S2 shadow rehearsal: goals planned, nothing sent, timing on the real graph.

Inputs: <run dir> (node/, driver/, captures/) and <bag dir>. The driver's key times, its status
receipt times and the node's event times are all time.monotonic() on the same machine.
Writes <run dir>/analysis.json and prints it."""
import json
import math
import os
import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

run, bag = sys.argv[1], sys.argv[2]


def jl(path):
    with open(path) as f:
        return [json.loads(ln) for ln in f if ln.strip()]


# ------------------------------------------------------------------ bag
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
counts = {t: 0 for t in types}
last_rx, max_gap = {}, {}
q0, max_dq, n_js = None, 0.0, 0
while r.has_next():
    topic, data, t = r.read_next()
    t *= 1e-9
    counts[topic] += 1
    if topic in last_rx:
        max_gap[topic] = max(max_gap.get(topic, 0.0), t - last_rx[topic])
    last_rx[topic] = t
    if topic == '/joint_states':
        m = deserialize_message(data, get_message(types[topic]))
        q = dict(zip(m.name, m.position))
        n_js += 1
        if q0 is None:
            q0 = q
        max_dq = max([max_dq] + [abs(q[k] - q0[k]) for k in q if k in q0])

# ------------------------------------------------------------------ node evidence + driver
ev = jl(os.path.join(run, 'node', 'events.jsonl'))
dl = jl(os.path.join(run, 'driver', 'driver.jsonl'))
with open(os.path.join(run, 'node', 'shadow_goals.json')) as f:
    shadow = json.load(f)
goals = shadow if isinstance(shadow, list) else shadow.get('goals', shadow.get('records', []))
with open(os.path.join(run, 'driver', 'driver_summary.json')) as f:
    summary = json.load(f)
statuses = [(d['mono'], d['status']) for d in dl if 'status' in d]
sim_of = [(m, s['monitors']['sim_time_s']) for m, s in statuses
          if s['monitors'].get('sim_time_s') is not None]


def last_status_before(mono):
    return max((x for x in statuses if x[0] <= mono), key=lambda x: x[0])


def first_status_after(mono, pred):
    return min((x for x in statuses if x[0] >= mono and pred(x[1])), key=lambda x: x[0])


def key_mono(why_part):
    return next(d['mono'] for d in dl if 'key' in d and why_part in d.get('why', ''))


def exercise(start_mono, end_pred, label):
    m0, s0 = last_status_before(start_mono)
    g = s0.get('goal') or {}
    m1, s1 = first_status_after(start_mono, end_pred)
    d_sim = s1['monitors']['sim_time_s'] - s0['monitors']['sim_time_s']
    dur = g.get('duration_s')
    return {'label': label, 'goal_in_flight': {k: g.get(k) for k in ('seq', 'b_from', 'b_to',
                                                                      'duration_s')},
            'sim_s': round(d_sim, 3), 'wall_s': round(m1 - start_mono, 3),
            'bound_sim_s': None if dur is None else round(dur + 1.0, 3),
            'pass_sim': dur is not None and d_sim <= dur + 1.0}


stop_ex = exercise(key_mono('stop exercise: space'),
                   lambda s: s['state'] == 'READY', 'check 1.7 (stop exercise, space)')
kill = next(d['mono'] for d in dl if d.get('event') == 'teleop_killed')
lease_ex = exercise(kill, lambda s: s['state'] == 'DISARMED',
                    'check 1.9 (lease exercise, teleop SIGKILL)')
lease_ex['wall_backstop_s'] = 120.0
stop_ex['wall_backstop_s'] = 60.0
lease_ex['pass_wall'] = lease_ex['wall_s'] <= 120.0
stop_ex['pass_wall'] = stop_ex['wall_s'] <= 60.0
lease_stopping = next((e['wall'] for e in ev if e.get('event') == 'transition'
                       and e['state'] == 'STOPPING' and e['wall'] >= kill), None)

mon = [s['monitors'] for _, s in statuses]


def mx(key):
    v = [m[key] for m in mon if m.get(key) is not None]
    return round(max(v), 3) if v else None


rtf = None
if len(sim_of) > 2:
    rtf = (sim_of[-1][1] - sim_of[0][1]) / (sim_of[-1][0] - sim_of[0][0])
gt = [s['ground_truth'] for _, s in statuses if s.get('ground_truth')]
arm = next((e for e in ev if e.get('request') == 'arm' and 'armed' in e.get('message', '')), {})
node_info = open(os.path.join(run, 'captures', 'after_node_info.txt')).read()


def section(text, head):
    lines, on = [], False
    for ln in text.splitlines():
        if ln.strip().endswith(':') and not ln.startswith('    '):
            on = ln.strip() == head
            continue
        if on and ln.strip():
            lines.append(ln.strip())
    return lines


assert '/spiderx_locomotion' in node_info and 'Action Clients:' in node_info
action_clients = section(node_info, 'Action Clients:')
publishers = section(node_info, 'Publishers:')
by_run = {}
for g in goals:
    by_run.setdefault((g.get('level'), g.get('direction')), 0)
    by_run[(g.get('level'), g.get('direction'))] += 1
final = json.load(open(os.path.join(run, 'node', 'final_status.json')))

out = {
    'driver': summary,
    'goals_planned_by_node': len(goals),
    'goals_by_level_direction': {f'L{k[0]} dir{k[1]:+d}': v for k, v in sorted(by_run.items())},
    'cancels': sum(1 for e in ev if e.get('event') == 'cancel_sent'),
    'command_rejections': final.get('command_rejections'),
    'commands_accepted': final.get('commands_accepted'),
    'faults_final': final.get('faults'),
    'arm': arm.get('message'),
    'nothing_sent': {
        'joint_trajectory_msgs': counts.get('/leg_trajectory_controller/joint_trajectory', 0),
        'action_feedback_msgs': counts.get(
            '/leg_trajectory_controller/follow_joint_trajectory/_action/feedback', 0),
        'action_status_msgs': counts.get(
            '/leg_trajectory_controller/follow_joint_trajectory/_action/status', 0),
        'node_action_clients': action_clients, 'node_publishers': publishers,
        'max_joint_motion_rad': max_dq, 'joint_state_samples': n_js},
    'bag_counts': counts,
    'timing': {
        'rtf_from_status': None if rtf is None else round(rtf, 3),
        'max_status_joint_states_age_s': mx('joint_states_age_s'),
        'max_status_body_pose_age_s': mx('body_pose_age_s'),
        'max_status_graph_age_s': mx('graph_age_s'),
        'max_rx_gap_joint_states_s': round(max_gap.get('/joint_states', math.nan), 3),
        'max_rx_gap_world_poses_s': round(max_gap.get('/spiderx/sim/world_poses', math.nan), 3),
        'max_rx_gap_clock_s': round(max_gap.get('/clock', math.nan), 3),
        'max_rx_gap_cmd_vel_s': round(max_gap.get('/cmd_vel', math.nan), 3),
        'lease_kill_to_stopping_wall_s': None if lease_stopping is None
        else round(lease_stopping - kill, 3)},
    'check_1_7': stop_ex, 'check_1_9': lease_ex,
    'weld': {'z_min': min((g['z'] for g in gt), default=None),
             'z_max': max((g['z'] for g in gt), default=None),
             'tilt_max': max((g['tilt'] for g in gt), default=None)},
}
with open(os.path.join(run, 'analysis.json'), 'w') as f:
    json.dump(out, f, indent=1, sort_keys=True)
print(json.dumps(out, indent=1, sort_keys=True))
