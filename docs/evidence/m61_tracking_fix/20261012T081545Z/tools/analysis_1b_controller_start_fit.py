#!/usr/bin/env python3
"""READ-ONLY (question 1b): when did the controller REALLY start the approved trajectory?

No ROS graph, nothing sent. Fits the start time t0 (1 ms grid, acceptance +-0.5 s, reference
time 3.0..6.2 s) of the approved M6.1 trajectory (m61_trot_cycle.reference_positions) to
  - the controller's own setpoint (/leg_trajectory_controller/controller_state 'reference'),
  - the recorded /joint_states the tool processed,
from the cycle 20261012T054741Z. Usage: <cycle evidence dir> <extracted bag dir>
"""
import csv
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from spiderx_controller import m61_trot_cycle as tc

E, bag = sys.argv[1:3]
M = E + '/run/m61_run/20261012T055506Z'
traj = json.load(open(M + '/trajectory.json'))
names = traj['joint_names']
out = json.load(open(M + '/live_outcome.json'))
acc = out['tracking']['accept_sim_s']
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
topic = '/leg_trajectory_controller/controller_state'
cls = get_message(types[topic])
r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
cs = []
while r.has_next():
    _, d, _ = r.read_next()
    m = deserialize_message(d, cls)
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    jn = list(m.joint_names)
    for fld in ('reference', 'desired'):
        p = getattr(m, fld)
        if len(p.positions):
            cs.append((st, fld, dict(zip(jn, p.positions))))
            break
print('controller_state setpoint field used:', sorted({c[1] for c in cs}))
js = [(float(x['time_s']), {n: float(x[n]) for n in names})
      for x in csv.DictReader(open(M + '/joint_states.csv')) if x['time_s']]


def fit(series, lo=3.0, hi=6.2):
    best = None
    for k in range(-500, 501):
        t0 = acc + k * 0.001
        errs = []
        for st, q in series:
            rt = st - t0
            if not (lo <= rt <= hi):
                continue
            ref = tc.reference_positions(traj, rt)
            if ref is None:
                continue
            errs.append(max(abs(q[n] - v) for n, v in zip(names, ref)))
        if len(errs) < 20:
            continue
        rms = (sum(e * e for e in errs) / len(errs)) ** 0.5
        if best is None or rms < best[1]:
            best = (t0, rms, max(errs), len(errs))
    return best


def at(series, t0):
    e = [max(abs(q[n] - v) for n, v in zip(names, tc.reference_positions(traj, st - t0)))
         for st, q in series if 3.0 <= st - t0 <= 6.2]
    return max(e), len(e)


setp = [(st, q) for st, _, q in cs if acc - 1 <= st <= 36.1]
b = fit(setp)
print(f'acceptance (sim) {acc:.3f} s')
print(f'controller setpoint vs approved trajectory: best start {b[0]:.3f} = acceptance '
      f'{b[0] - acc:+.3f} s; rms {b[1]:.2e} max {b[2]:.2e} rad over {b[3]} samples')
b2 = fit(js)
print(f'recorded joint_states vs approved trajectory: best start {b2[0]:.3f} = acceptance '
      f'{b2[0] - acc:+.3f} s; rms {b2[1]:.4f} max {b2[2]:.4f} rad over {b2[3]} samples')
print('setpoint max err at the acceptance anchor (rad, samples):', at(setp, acc))
print('joint_states max err at the acceptance anchor (rad, samples):', at(js, acc))
print('controller_state rate in the goal window: '
      f'{len([1 for st, _, _ in cs if acc <= st <= 36.03]) / (36.03 - acc):.2f} Hz')
