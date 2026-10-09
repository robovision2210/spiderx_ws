#!/usr/bin/env python3
"""READ-ONLY capture for the M6.1-A launch-only observation (evidence helper, not project code).

Subscribes (never publishes) on an explicit ROS domain to /scan, /joint_states,
/spiderx/sim/world_poses, /clock and /robot_description for a fixed WALL duration and writes JSON.
    python3 capture.py --domain-id 0 --seconds 8 --scans 30 --out capture.json
"""
import argparse
import json
import math
import time
import xml.etree.ElementTree as ET


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--domain-id', type=int, required=True)
    p.add_argument('--seconds', type=float, default=8.0)
    p.add_argument('--scans', type=int, default=30)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    ctx = Context()
    rclpy.init(context=ctx, domain_id=a.domain_id, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('m61a_phase1_readonly_capture', context=ctx)
    data = {'scans': [], 'joint_states': [], 'poses': [], 'clock': [], 'description': None,
            'domain_id': a.domain_id}

    def st(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def on_scan(m):
        if len(data['scans']) < a.scans:
            data['scans'].append({'wall': time.monotonic(), 'stamp': st(m.header),
                                  'frame_id': m.header.frame_id, 'angle_min': m.angle_min,
                                  'angle_max': m.angle_max, 'angle_increment': m.angle_increment,
                                  'range_min': m.range_min, 'range_max': m.range_max,
                                  'ranges': [r if math.isfinite(r) else None for r in m.ranges]})

    def on_js(m):
        data['joint_states'].append({'wall': time.monotonic(), 'stamp': st(m.header),
                                     'name': list(m.name), 'position': list(m.position),
                                     'velocity': list(m.velocity), 'effort': list(m.effort)})

    def on_pose(m):
        data['poses'].append({'wall': time.monotonic(), 'transforms': [
            {'child': t.child_frame_id, 'frame': t.header.frame_id, 'stamp': st(t.header),
             'xyz': [t.transform.translation.x, t.transform.translation.y,
                     t.transform.translation.z],
             'q': [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                   t.transform.rotation.w]}
            for t in m.transforms if t.child_frame_id in ('spiderx', 'dummy_link', 'base_link',
                                                          'lidar_link')]})

    def on_clock(m):
        data['clock'].append([time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9])

    def on_desc(m):
        data['description'] = m.data

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', on_js, 50)
    node.create_subscription(TFMessage, '/spiderx/sim/world_poses', on_pose, 50)
    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(String, '/robot_description', on_desc, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds or len(data['scans']) < a.scans:
        ex.spin_once(timeout_sec=0.05)
        if time.monotonic() - t0 > a.seconds * 6:
            break
    data['wall_span_s'] = time.monotonic() - t0
    data['own_publishers'] = [t for t, _ in node.get_publisher_names_and_types_by_node(
        node.get_name(), '/') if t not in ('/rosout', '/parameter_events')]
    desc = data.pop('description')
    weld = None
    if desc:
        root = ET.fromstring(desc)
        for j in root.findall('joint'):
            if j.get('name') == 'spiderx_fixed_base_weld':
                o = j.find('origin')
                weld = {'type': j.get('type'), 'parent': j.find('parent').get('link'),
                        'child': j.find('child').get('link'),
                        'xyz': o.get('xyz'), 'rpy': o.get('rpy')}
        data['description_root_link'] = [l.get('name') for l in root.findall('link')][:2]
        data['description_has_world_link'] = any(l.get('name') == 'world'
                                                 for l in root.findall('link'))
    data['weld_joint'] = weld
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown(context=ctx)
    with open(a.out, 'w') as f:
        json.dump(data, f)
    print(f"scans {len(data['scans'])} joint_states {len(data['joint_states'])} "
          f"poses {len(data['poses'])} clock {len(data['clock'])} weld {weld} "
          f"own_publishers {data['own_publishers']}")


if __name__ == '__main__':
    main()
