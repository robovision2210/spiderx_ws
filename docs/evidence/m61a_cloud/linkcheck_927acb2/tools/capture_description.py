#!/usr/bin/env python3
"""Read-only: receive /robot_description once (transient local, reliable) and write it to a file.
Subscriptions only; publishes nothing."""
import argparse
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--timeout', type=float, default=15.0)
a = ap.parse_args()
rclpy.init()
node = rclpy.create_node('m61a_linkcheck_description_reader')
got = []
node.create_subscription(String, '/robot_description', lambda m: got.append(m.data),
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.monotonic()
while not got and time.monotonic() - t0 < a.timeout:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
if not got:
    print('no /robot_description received')
    sys.exit(1)
with open(a.out, 'w') as f:
    f.write(got[0])
print(f'/robot_description: {len(got[0])} bytes')
