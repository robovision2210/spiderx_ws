#!/usr/bin/env python3
"""READ-ONLY: build the M6.1 tracking-replay test fixture from the recorded Cloud cycle.

No ROS graph, nothing sent. Inputs (all from docs/evidence/m61_cloud_one_cycle/20261012T054741Z):
  - run/m61_run/<UTC>/joint_states.csv   every /joint_states sample the tool's session processed
  - run/m61_run/<UTC>/live_outcome.json  acceptance (sim), the tool's estimate, the cancel
  - run/m61_run/<UTC>/trajectory.json    id and content hash of the approved trajectory
  - the extracted bag (run/bag.tar.xz)   /leg_trajectory_controller/controller_state
The action FEEDBACK itself was not recorded (action topics are hidden from ros2 bag record), so
the fixture's feedback is a PROXY: every controller_state message of the same controller in the
goal window (acceptance .. the last joint sample), with its header stamp and time_from_start 0.
In controller_state (2.48.0) the deprecated 'desired' point is empty and the populated
'reference' point also carries time_from_start 0 in every message; the fixture records both.
That the real action feedback carried 0 is shown by the tool's own record, not by the proxy:
its estimate median(stamp - time_from_start) was acceptance + 3.216 s, about the median feedback
stamp, where an informative time_from_start would have given acceptance - 0.015 s.
Usage: make_replay_fixture.py <m61_run/UTC dir> <bag dir> <bag.tar.xz> <out.json>
"""
import csv
import hashlib
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def sha(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


run, bag, bag_archive, out = sys.argv[1:5]
traj = json.load(open(f'{run}/trajectory.json'))
outcome = json.load(open(f'{run}/live_outcome.json'))
names = traj['joint_names']
rows = list(csv.DictReader(open(f'{run}/joint_states.csv')))
js = [[float(r['time_s'])] + [float(r[n]) for n in names] for r in rows if r['time_s']]
acc = outcome['tracking']['accept_sim_s']
last = js[-1][0]

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
topic = '/leg_trajectory_controller/controller_state'
cls = get_message(types[topic])
r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
fb, seen = [], set()
while r.has_next():
    _, data, _ = r.read_next()
    m = deserialize_message(data, cls)
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    tfs = m.desired.time_from_start.sec + m.desired.time_from_start.nanosec * 1e-9
    ref_tfs = m.reference.time_from_start.sec + m.reference.time_from_start.nanosec * 1e-9
    if acc <= st <= last:
        fb.append([st, tfs])
        seen.add(('desired', len(m.desired.positions), tfs))
        seen.add(('reference', len(m.reference.positions), ref_tfs))

fixture = {
    'schema': 'spiderx.m61.tracking_replay_fixture/1',
    'source': 'docs/evidence/m61_cloud_one_cycle/20261012T054741Z',
    'source_sha256': {
        'run/m61_run/20261012T055506Z/joint_states.csv': sha(f'{run}/joint_states.csv'),
        'run/m61_run/20261012T055506Z/live_outcome.json': sha(f'{run}/live_outcome.json'),
        'run/m61_run/20261012T055506Z/trajectory.json': sha(f'{run}/trajectory.json'),
        'run/bag.tar.xz': sha(bag_archive),
    },
    'trajectory_id': traj['trajectory_id'],
    'trajectory_content_sha256': traj['provenance']['content_sha256'],
    'recorded': {
        'state': outcome['state'], 'reason': outcome['reason'],
        'goals_sent': outcome['goals_sent'], 'cancels_sent': outcome['cancels_sent'],
        'accept_sim_s': acc,
        'feedback_start_estimate_s': outcome['tracking']['feedback_start_estimate_s'],
        'max_inflight_error_rad': outcome['tracking']['max_inflight_error_rad'],
        'last_joint_stamp_s': last,
    },
    'joint_names': names,
    'joint_states_columns': ['stamp_s'] + names,
    'joint_states': js,
    'feedback_proxy': {
        'source_topic': topic,
        'why': ('the action feedback was not recorded (hidden action topics); controller_state '
                'is published by the same controller update loop with the same header stamp '
                'clock. time_from_start is 0 in every message (the deprecated desired point is '
                'empty; the populated reference point also carries 0). The real feedback '
                'carried 0 by the tool record: its estimate was acceptance + 3.216 s, about '
                'the median feedback stamp'),
        'controller_state_points_seen': sorted([list(x) for x in seen]),
        'window_s': [acc, last],
        'columns': ['stamp_s', 'desired_time_from_start_s'],
        'samples': fb,
    },
}
with open(out, 'w') as f:
    json.dump(fixture, f, indent=None, separators=(',', ':'))
    f.write('\n')
print(f'joint_states {len(js)}  feedback_proxy {len(fb)}  '
      f'tfs values {sorted({round(t, 9) for _, t in fb})}  -> {out}')
