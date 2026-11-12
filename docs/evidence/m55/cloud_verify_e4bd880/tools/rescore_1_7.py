"""Re-score check 1.7 (sim-time bound, wall backstop) from the shadow bag's status topic.

The status carries the state and monitors.sim_time_s; the bag stamps are recorder wall (epoch).
The space keypress is located by aligning the node's STOPPING->READY transition (events.jsonl,
node mono) with the first READY status after it."""
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

bag, events = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/spiderx/locomotion/status']))
rows = []
while r.has_next():
    _, data, t = r.read_next()
    d = json.loads(deserialize_message(data, String).data)
    rows.append((t * 1e-9, d.get('state'), d.get('monitors', {}).get('sim_time_s')))
ev = [json.loads(ln) for ln in open(events)]
space = next(e['wall'] for e in ev if e.get('request') == 'stop' and e['state'] == 'STOPPING')
ready = next(e['wall'] for e in ev if e.get('event') == 'transition' and e['state'] == 'READY'
             and e['wall'] > space)
# the status READY that follows the STOPPING run containing the space
trans = [(i, x) for i, x in enumerate(rows) if i and x[1] != rows[i - 1][1]]
for i, x in trans:
    print('%.3f %-9s sim %s' % (x[0], x[1], x[2]))
# epoch <-> node mono offset from the STOPPING -> READY transition after the space
st_ready = next(x for i, x in trans if x[1] == 'READY' and rows[i - 1][1] == 'STOPPING'
                and x[0] > rows[0][0] and x[2] > 12)
off = st_ready[0] - ready
space_epoch = space + off
before = max((x for x in rows if x[0] <= space_epoch), key=lambda x: x[0])
after = min((x for x in rows if x[0] >= space_epoch), key=lambda x: x[0])
f = (space_epoch - before[0]) / (after[0] - before[0])
sim_space = before[2] + f * (after[2] - before[2])
print('offset %.3f; status around the space: %.3f s and %.3f s apart' %
      (off, space_epoch - before[0], after[0] - space_epoch))
print('space sim %.3f (state %s); READY sim %.3f' % (sim_space, before[1], st_ready[2]))
d_sim = st_ready[2] - sim_space
d_wall = ready - space
goal = [e for e in ev if e.get('event') == 'goal_sent' and e.get('duration_s')]
dur = 2.8                                   # the in-flight goal at the space (b4 -> b5, level 0)
assert any(abs(g['duration_s'] - dur) < 1e-9 and g['b_from'] == 4 for g in goal)
d_sim_cons = st_ready[2] - before[2]        # from the last status BEFORE the space (conservative)
print('criterion: space->READY sim (last status before the space) %.2f <= %.1f + 1.0 -> %s; '
      'wall %.2f <= 60 -> %s' % (d_sim_cons, dur, 'PASS' if d_sim_cons <= dur + 1.0 else 'FAIL',
                                 d_wall, 'PASS' if d_wall <= 60 else 'FAIL'))
