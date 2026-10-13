#!/usr/bin/env python3
"""READ-ONLY offline diagnosis of the TRACKING_FAILED cancel (no ROS graph, nothing sent).

Inputs: the run's m61_run/<UTC>/ evidence and the bag. It reports:
  1. the recorded /joint_states against the approved trajectory (m61_trot_cycle.reference_positions,
     the tool's own reference function), with the trajectory start at goal ACCEPTANCE (sim) and at
     the tool's feedback start estimate (the median of feedback stamp - desired.time_from_start);
  2. the controller's own /leg_trajectory_controller/controller_state: desired.time_from_start and
     its reported error, over the goal window.
Usage: tracking_reference_analysis.py <m61_run/UTC dir> <bag dir> <out.json>
"""
import csv
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from spiderx_controller import m61_trot_cycle as tc

run, bag, out = sys.argv[1:4]
traj = json.load(open(f'{run}/trajectory.json'))
outcome = json.load(open(f'{run}/live_outcome.json'))
acc = outcome['tracking']['accept_sim_s']
est = outcome['tracking']['feedback_start_estimate_s']
names = traj['joint_names']
js = [(float(r['time_s']), [float(r[n]) for n in names])
      for r in csv.DictReader(open(f'{run}/joint_states.csv')) if r['time_s']]


def errors(t0):
    worst, n, rows = (0.0, None, None), 0, []
    for st, q in js:
        ref = tc.reference_positions(traj, st - t0)
        if ref is None:
            continue
        n += 1
        e = max(abs(a - b) for a, b in zip(q, ref))
        rows.append((st, st - t0, e))
        if e > worst[0]:
            worst = (e, st, st - t0)
    return {'t0_sim_s': t0, 'tracked_samples': n, 'max_abs_err_rad': worst[0],
            'at_stamp': worst[1], 'at_ref_time_s': worst[2],
            'first_tracked_ref_time_s': rows[0][1] if rows else None,
            'last_tracked_ref_time_s': rows[-1][1] if rows else None}


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
topic = '/leg_trajectory_controller/controller_state'
cls = get_message(types[topic])
r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
cs = []
while r.has_next():
    _, data, _ = r.read_next()
    m = deserialize_message(data, cls)
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    d = m.reference if len(m.reference.positions) else m.desired
    tfs = d.time_from_start.sec + d.time_from_start.nanosec * 1e-9
    cs.append((st, tfs, max([abs(x) for x in m.error.positions] or [0.0])))
win = [c for c in cs if acc <= c[0] <= js[-1][0]]
res = {
    'schema': 'spiderx.m61.tracking_reference_analysis/1',
    'accept_sim_s': acc, 'feedback_start_estimate_s': est,
    'feedback_offset_s': outcome['tracking']['feedback_offset_s'],
    'tool_max_inflight_error_rad': outcome['tracking']['max_inflight_error_rad'],
    'client_abort_threshold_rad': outcome['m61_limits']['client_tracking_abort_rad'],
    'reference_from_acceptance': errors(acc),
    'reference_from_feedback_estimate': errors(est),
    'controller_state': {
        'type': types[topic], 'samples_in_goal_window': len(win),
        'desired_time_from_start_values': sorted({round(c[1], 6) for c in win}),
        'max_controller_reported_abs_error_rad': max(c[2] for c in win) if win else None,
        'median_stamp_in_window': sorted(c[0] for c in win)[len(win) // 2] if win else None},
}
json.dump(res, open(out, 'x'), indent=1)
print(json.dumps(res, indent=1))
