"""SUPPLEMENTARY, read-only (bag opened immutable): how far goal 1 progressed before the cancel. No verdict.
Reports, in bag receipt time and joint-state stamps (sim): the controller reference and actual joint positions
from just before goal 1 to the end of the hold; the largest joint displacement from the pre-goal pose; the
sim time between the goal's acceptance and its result. Usage: goal_progress_supplementary.py <run root>"""
import json, sqlite3, sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
R = sys.argv[1]; bag = R + '/bag/bag_0.db3'
con = sqlite3.connect(f'file:{bag}?mode=ro&immutable=1', uri=True)
topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
want = {'/joint_states', '/leg_trajectory_controller/controller_state', '/leg_trajectory_controller/follow_joint_trajectory/_action/feedback', '/spiderx/locomotion/status'}
types = {i: get_message(t) for i, (n, t) in topics.items() if n in want}
js, cs, fb, st = [], [], [], []
for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id'):
    n = topics[tid][0]
    if n not in want: continue
    m = deserialize_message(data, types[tid]); t = ts / 1e9
    if n == '/joint_states': js.append((t, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, dict(zip(m.name, m.position))))
    elif n.endswith('controller_state'): cs.append((t, list(m.joint_names), list(m.reference.positions), list(m.feedback.positions)))
    elif n.endswith('feedback'): fb.append((t, list(m.feedback.desired.positions), list(m.feedback.actual.positions), list(m.feedback.error.positions)))
    else:
        d = json.loads(m.data); g = d.get('goal') or {}
        st.append((t, d.get('state'), g.get('seq'), g.get('accepted'), (d.get('monitors') or {}).get('sim_time_s')))
con.close()
walk = [s for s in st if s[1] in ('STARTING', 'WALKING')]
first_goal_t = walk[0][0] if walk else None
faulted = [s for s in st if s[1] == 'FAULTED']
t_fault = faulted[0][0] if faulted else None
nogoal = [s for s in faulted if s[2] is None]
t_res = nogoal[0][0] if nogoal else None
pre = [c for c in cs if first_goal_t is not None and c[0] < first_goal_t]
ref0 = pre[-1][2] if pre else None; act0 = pre[-1][3] if pre else None
def maxdiff(a, b): return max(abs(x - y) for x, y in zip(a, b)) if a and b else None
win = [c for c in cs if first_goal_t is not None and c[0] >= first_goal_t]
ref_dev = max((maxdiff(c[2], ref0) for c in win), default=None)
act_dev = max((maxdiff(c[3], act0) for c in win), default=None)
pre_js = [j for j in js if first_goal_t is not None and j[0] < first_goal_t]
j0 = pre_js[-1] if pre_js else None
jdev = max((max(abs(v - j0[2][k]) for k, v in j[2].items() if k in j0[2]) for j in js if j0 and j[0] >= first_goal_t), default=None)
def sim_at(t):
    c = [j for j in js if j[0] <= t]; return c[-1][1] if c else None
out = {'status_first_STARTING_or_WALKING_receipt': first_goal_t, 'status_first_FAULTED_receipt': t_fault,
       'status_first_FAULTED_no_goal_receipt': t_res,
       'sim_at_first_goal_status': sim_at(first_goal_t) if first_goal_t else None, 'sim_at_first_FAULTED_status': sim_at(t_fault) if t_fault else None,
       'sim_at_first_FAULTED_no_goal_status': sim_at(t_res) if t_res else None,
       'controller_reference_max_change_from_pre_goal_rad': ref_dev, 'controller_actual_max_change_from_pre_goal_rad': act_dev,
       'joint_states_max_change_from_pre_goal_rad': jdev,
       'feedback_samples': [{'receipt': f[0], 'max_abs_desired_minus_pre_goal_ref_rad': maxdiff(f[1], ref0), 'max_abs_error_rad': max(abs(e) for e in f[3])} for f in fb],
       'note': 'supplementary only; the verdict is score/summary.json'}
print(json.dumps(out, indent=1, default=str))
