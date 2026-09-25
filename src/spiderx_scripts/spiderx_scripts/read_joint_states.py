"""Print SpiderX joint angles grouped by leg (hip / thigh / foot), in degrees.

    ros2 run spiderx_scripts read_joint_states

The leg grouping is read from spiderx_controller/config/spiderx_legs.yaml, so it stays consistent
with the URDF. In simulation /joint_states comes from Gazebo. The robot is passive today, so the
values show where gravity has settled the legs.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


def load_legs():
    path = os.path.join(get_package_share_directory('spiderx_controller'),
                        'config', 'spiderx_legs.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return cfg['joint_order'], cfg['legs']


class JointStateReader(Node):
    def __init__(self):
        super().__init__('spiderx_read_joint_states')
        self.order, self.legs = load_legs()
        self.create_subscription(JointState, 'joint_states', self.on_state, 10)
        self.last = None

    def on_state(self, msg):
        now = self.get_clock().now().nanoseconds
        if self.last is not None and now - self.last < 1e9:
            return  # print at most once per second
        self.last = now
        pos = dict(zip(msg.name, msg.position))
        lines = []
        for leg, cfg in self.legs.items():
            vals = []
            for role in self.order:
                name = cfg['joints'][role]['name']
                vals.append(f'{role} {math.degrees(pos[name]):7.2f}' if name in pos
                            else f'{role}    n/a')
            lines.append(f'{leg:<12} ' + '  '.join(vals))
        self.get_logger().info('joint angles [deg]\n' + '\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = JointStateReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
