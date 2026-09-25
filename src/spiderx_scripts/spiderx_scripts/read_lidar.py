"""Print SpiderX lidar distances in four sectors (front, left, rear, right).

    ros2 run spiderx_scripts read_lidar

Works in the Gazebo Fortress simulation and on hardware. /scan is in lidar_link, whose +x is the
robot's front, so sector angles are measured from the front: left = +90 deg, right = -90 deg.
Each sector reports the closest valid return within +/-20 deg of its centre.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SECTORS = {'front': 0.0, 'left': 90.0, 'rear': 180.0, 'right': -90.0}
HALF_WIDTH_DEG = 20.0


def sector_minimum(msg, centre_deg, half_width_deg=HALF_WIDTH_DEG):
    """Closest valid range (m) within +/-half_width of centre, or None if none is valid."""
    best = None
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
            continue
        angle = math.degrees(msg.angle_min + i * msg.angle_increment)
        diff = (angle - centre_deg + 180.0) % 360.0 - 180.0
        if abs(diff) <= half_width_deg and (best is None or r < best):
            best = r
    return best


class LidarReader(Node):
    def __init__(self):
        super().__init__('spiderx_read_lidar')
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        parts = []
        for name, centre in SECTORS.items():
            d = sector_minimum(msg, centre)
            parts.append(f'{name}: ' + (f'{d:.2f} m' if d is not None else '--'))
        self.get_logger().info(f'[{msg.header.frame_id}] ' + ' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
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
