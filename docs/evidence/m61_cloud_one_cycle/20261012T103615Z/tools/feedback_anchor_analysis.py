#!/usr/bin/env python3
"""READ-ONLY offline analysis (plan F, third authorization): the recorded ACTION FEEDBACK vs the
acceptance-anchor assumption. No ROS graph, nothing sent.

Inputs: the run's m61_run/<UTC>/ evidence and the bag (which recorded
/leg_trajectory_controller/follow_joint_trajectory/_action/feedback). Reports:
  1. the feedback messages: count, stamps, desired.time_from_start values (0 or positive);
  2. the controller's real start relative to acceptance, by fitting the approved trajectory
     (m61_trot_cycle.reference_positions) to the feedback's desired positions, to
     controller_state.reference, and to the recorded /joint_states (1 ms grid, +-0.5 s,
     reference time 3.0-7.0 s);
  3. the recorded joints against the trajectory timed from acceptance, and from the unfiltered
     M6.0-D estimate the tool also recorded (what the pre-fix rule would have used);
  4. the controller's own reported error in the feedback.
Usage: feedback_anchor_analysis.py <m61_run/UTC dir> <bag dir> <out.json>
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
oc = json.load(open(f'{run}/live_outcome.json'))
names = traj['joint_names']
acc = oc['tracking']['accept_sim_s']
m6d_est = oc['tracking_reference']['unfiltered_m6d_estimate_s']
FB = '/leg_trajectory_controller/follow_joint_trajectory/_action/feedback'
CS = '/leg_trajectory_controller/controller_state'


def secs(t):
    return t.sec + t.nanosec * 1e-9


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=[FB, CS]))
cls = {FB: get_message(types[FB]), CS: get_message(types[CS])}
fb, cs = [], []
while r.has_next():
    topic, data, _ = r.read_next()
    m = deserialize_message(data, cls[topic])
    if topic == FB:
        f = m.feedback
        jn = list(f.joint_names)
        fb.append({'stamp': secs(f.header.stamp), 'tfs': secs(f.desired.time_from_start),
                   'desired': dict(zip(jn, f.desired.positions)),
                   'err': max((abs(e) for e in f.error.positions), default=None),
                   'goal_id': bytes(m.goal_id.uuid).hex()})
    else:
        jn = list(m.joint_names)
        cs.append((secs(m.header.stamp), dict(zip(jn, m.reference.positions))))
js = [(float(x['time_s']), {n: float(x[n]) for n in names})
      for x in csv.DictReader(open(f'{run}/joint_states.csv')) if x['time_s']]


def fit(series, lo=3.0, hi=7.0):
    best = None
    for k in range(-500, 501):
        t0 = acc + k * 0.001
        errs = []
        for st, q in series:
            rt = st - t0
            if not (lo <= rt <= hi) or any(n not in q for n in names):
                continue
            ref = tc.reference_positions(traj, rt)
            if ref is not None:
                errs.append(max(abs(q[n] - v) for n, v in zip(names, ref)))
        if len(errs) < 20:
            continue
        rms = (sum(e * e for e in errs) / len(errs)) ** 0.5
        if best is None or rms < best[1]:
            best = (t0, rms, max(errs), len(errs))
    if best is None:
        return None
    return {'best_start_sim_s': round(best[0], 6),
            'offset_from_acceptance_s': round(best[0] - acc, 6),
            'rms_rad': best[1], 'max_rad': best[2], 'samples': best[3]}


def at(series, t0):
    e = []
    for st, q in series:
        ref = tc.reference_positions(traj, st - t0)
        if ref is not None and all(n in q for n in names):
            e.append(max(abs(q[n] - v) for n, v in zip(names, ref)))
    return {'t0_sim_s': t0, 'tracked_samples': len(e), 'max_abs_err_rad': max(e) if e else None}


tfs = [f['tfs'] for f in fb]
res = {
    'schema': 'spiderx.m61.feedback_anchor_analysis/1',
    'accept_sim_s': acc,
    'feedback': {
        'topic': FB, 'messages': len(fb), 'goal_ids': sorted({f['goal_id'] for f in fb}),
        'stamp_first_s': fb[0]['stamp'] if fb else None,
        'stamp_last_s': fb[-1]['stamp'] if fb else None,
        'time_from_start_zero': sum(1 for t in tfs if t == 0.0),
        'time_from_start_positive': sum(1 for t in tfs if t > 0.0),
        'time_from_start_values': sorted(set(round(t, 9) for t in tfs))[:10],
        'max_controller_reported_error_rad': max((f['err'] for f in fb if f['err'] is not None),
                                                 default=None),
        'median_stamp_minus_acceptance_s': (sorted(f['stamp'] for f in fb)[len(fb) // 2] - acc
                                            if fb else None),
    },
    'controller_start_fit': {
        'feedback_desired': fit([(f['stamp'], f['desired']) for f in fb]),
        'controller_state_reference': fit(cs),
        'joint_states': fit(js),
    },
    'joints_vs_trajectory': {
        'from_acceptance': at(js, acc),
        'from_unfiltered_m6d_estimate': at(js, m6d_est),
    },
    'tool_recorded': {'max_inflight_error_rad': oc['tracking']['max_inflight_error_rad'],
                      'tracking_reference': oc['tracking_reference']},
}
json.dump(res, open(out, 'w'), indent=1)
f = res['feedback']
print(f"feedback messages {f['messages']} (goal ids {len(f['goal_ids'])}); time_from_start: "
      f"{f['time_from_start_zero']} zero, {f['time_from_start_positive']} positive")
for k, v in res['controller_start_fit'].items():
    print(f"start fit {k}: {v}")
print('joints from acceptance:', res['joints_vs_trajectory']['from_acceptance'])
print('joints from the unfiltered M6.0-D estimate:',
      res['joints_vs_trajectory']['from_unfiltered_m6d_estimate'])
print('controller-reported max error in feedback:', f['max_controller_reported_error_rad'])
