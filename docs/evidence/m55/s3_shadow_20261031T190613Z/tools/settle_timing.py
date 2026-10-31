#!/usr/bin/env python3
"""Offline, read-only: settle-bag timing. Pose samples get sim time from the nearest /joint_states
sample by receive time (the bridged pose headers carry no stamp). Also locates /joint_states receive
gaps > 0.3 s and the RTF per 10 s wall window, with UTC times. Usage: settle_timing.py <run root>"""
import bisect, datetime, glob, json, os, sqlite3, sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from spiderx_controller import m55_locomotion as loc
from spiderx_controller.m55_rclpy import body_pose_sample
R = sys.argv[1]
dbf = glob.glob(os.path.join(R, 'bag_settle', '*.db3'))[0]
con = sqlite3.connect(f'file:{dbf}?mode=ro&immutable=1', uri=True)
topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
types = {i: get_message(t) for i, (n, t) in topics.items()}
cfg = loc.load_config()
js, ps = [], []
for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id'):
    n = topics[tid][0]; m = deserialize_message(data, types[tid]); t = ts * 1e-9
    if n == '/joint_states':
        js.append((t, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    else:
        c, p = body_pose_sample(m.transforms, cfg)
        if p is not None:
            ps.append((t, p))
jt = [a for a, _ in js]
def sim(t):
    i = min(max(bisect.bisect_left(jt, t), 1), len(jt) - 1)
    a, b = js[i - 1], js[i]
    return a[1] if abs(t - a[0]) <= abs(t - b[0]) else b[1]
utc = lambda t: datetime.datetime.utcfromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]
zf = json.load(open(os.path.join(R, 'run', 'premotion', 'settle_check.json')))['window']['z_mean_m']
k = len(ps)
while k > 0 and abs(ps[k - 1][1]['z'] - zf) <= 0.0025 and ps[k - 1][1]['tilt'] <= 0.10:
    k -= 1
first = ps[0]
gaps = [(b[0] - a[0], a[0], b[0], a[1], b[1]) for a, b in zip(js, js[1:]) if b[0] - a[0] > 0.3]
win, i = [], 0
for j in range(len(js)):
    while js[j][0] - js[i][0] > 10.0:
        i += 1
    if js[j][0] - js[i][0] >= 9.5:
        win.append(((js[j][1] - js[i][1]) / (js[j][0] - js[i][0]), js[i][0], js[j][0]))
low = [w for w in win if w[0] < 0.1]
out = {'first_pose': {'utc': utc(first[0]), 'sim_s': sim(first[0]), 'z_m': first[1]['z']},
       'first_joint_state': {'utc': utc(js[0][0]), 'sim_s': js[0][1]},
       'min_z_m': min(p['z'] for _, p in ps), 'min_z_sim_s': sim(min(ps, key=lambda q: q[1]['z'])[0]),
       'settled_band_m': [zf - 0.0025, zf + 0.0025],
       'in_band_from': None if k >= len(ps) else {'utc': utc(ps[k][0]), 'sim_s': sim(ps[k][0])},
       'settle_bag_end': {'utc': utc(ps[-1][0]), 'sim_s': sim(ps[-1][0])},
       'joint_state_receive_gaps_over_0.3s': [{'gap_s': round(g, 3), 'from_utc': utc(a), 'to_utc': utc(b), 'sim_from_s': sa, 'sim_to_s': sb} for g, a, b, sa, sb in gaps],
       'rtf_10s_window_min': min(win)[0], 'rtf_10s_windows_below_0.1': {'count': len(low), 'first_utc': utc(low[0][1]) if low else None, 'last_utc': utc(low[-1][2]) if low else None}}
print(json.dumps(out, indent=1))
