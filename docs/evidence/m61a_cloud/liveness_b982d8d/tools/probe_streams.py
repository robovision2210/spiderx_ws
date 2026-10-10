#!/usr/bin/env python3
"""READ-ONLY stream probe (evidence helper): receipt of /clock, the bridged pose stream and
/joint_states over a fixed wall window, on an explicit domain. Subscriptions only: no publisher,
no service or action client. Writes one JSON summary.
    python3 probe_streams.py --domain-id 0 --seconds 5 --out probe.json
Per stream: messages, wall span, rate; /clock: first/last sim time and the advance; pose stream:
the receiving node's /clock at the first and last receipt (the bridge leaves the per-pose stamps
at 0); /joint_states: first/last header stamp. Pose entries: the distinct values of the model and
body-link entries and the names of every entry seen.
"""
import argparse
import json
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--domain-id', type=int, required=True)
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    ctx = Context()
    rclpy.init(context=ctx, domain_id=a.domain_id, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('m61a_stream_probe', context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    st = {'clock': [], 'pose': [], 'js': []}
    last_sim = [None]
    names, values = set(), {}

    def on_clock(m):
        s = m.clock.sec + m.clock.nanosec * 1e-9
        last_sim[0] = s
        st['clock'].append((time.monotonic(), s))

    def on_pose(m):
        st['pose'].append((time.monotonic(), last_sim[0], len(m.transforms)))
        for t in m.transforms:
            names.add(t.child_frame_id)
            if t.child_frame_id in ('spiderx', 'dummy_link'):
                tr, r = t.transform.translation, t.transform.rotation
                values.setdefault(t.child_frame_id, set()).add(
                    (tr.x, tr.y, tr.z, r.x, r.y, r.z, r.w))

    def on_js(m):
        st['js'].append((time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    subs = [node.create_subscription(Clock, '/clock', on_clock,
                                     QoSProfile(depth=10,
                                                reliability=ReliabilityPolicy.BEST_EFFORT)),
            node.create_subscription(TFMessage, '/spiderx/sim/world_poses', on_pose, 50),
            node.create_subscription(JointState, '/joint_states', on_js, 50)]
    end = time.monotonic() + 1.0                       # discovery, not recorded
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.05)
    for k in st:
        st[k].clear()
    t0 = time.monotonic()
    while time.monotonic() < t0 + a.seconds:
        ex.spin_once(timeout_sec=0.05)

    def span(rows):
        return (rows[-1][0] - rows[0][0]) if len(rows) > 1 else 0.0
    c, p, j = st['clock'], st['pose'], st['js']
    out = {'window_s': a.seconds, 'domain_id': a.domain_id,
           'own_publishers': [t for t, _ in node.get_publisher_names_and_types_by_node(
               'm61a_stream_probe', '/') if t not in ('/rosout', '/parameter_events')],
           'clock': {'messages': len(c), 'rate_hz': len(c) / a.seconds,
                     'sim_first': c[0][1] if c else None, 'sim_last': c[-1][1] if c else None,
                     'sim_advance_s': (c[-1][1] - c[0][1]) if c else None,
                     'wall_span_s': span(c)},
           'pose_stream': {'messages': len(p), 'rate_hz': len(p) / a.seconds,
                           'node_clock_at_first_receipt': p[0][1] if p else None,
                           'node_clock_at_last_receipt': p[-1][1] if p else None,
                           'transforms_per_message': sorted({x[2] for x in p}),
                           'entry_names': sorted(names),
                           'distinct_values': {k: [list(v) for v in sorted(vals)]
                                               for k, vals in values.items()}},
           'joint_states': {'messages': len(j), 'rate_hz': len(j) / a.seconds,
                            'stamp_first': j[0][1] if j else None,
                            'stamp_last': j[-1][1] if j else None}}
    for s in subs:
        node.destroy_subscription(s)
    ex.shutdown(timeout_sec=1.0)
    node.destroy_node()
    rclpy.try_shutdown(context=ctx)
    json.dump(out, open(a.out, 'w'), indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != 'pose_stream'}),
          json.dumps({k: v for k, v in out['pose_stream'].items()
                      if k not in ('entry_names', 'distinct_values')}))


if __name__ == '__main__':
    main()
