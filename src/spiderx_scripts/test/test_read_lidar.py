"""Unit test for the sector logic of read_lidar (no ROS graph needed)."""
import math

from sensor_msgs.msg import LaserScan
from spiderx_scripts.read_lidar import sector_minimum


def _scan():
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_increment = math.radians(1.0)
    msg.range_min, msg.range_max = 0.15, 12.0
    msg.ranges = [5.0] * 360
    return msg


def test_front_and_wraparound():
    msg = _scan()
    msg.ranges[180] = 1.0          # angle 0 deg (front)
    msg.ranges[0] = 2.0            # angle -180 deg (rear)
    assert sector_minimum(msg, 0.0) == 1.0
    assert sector_minimum(msg, 180.0) == 2.0


def test_ignores_invalid():
    msg = _scan()
    msg.ranges = [float('inf')] * 360
    msg.ranges[270] = 0.05         # below range_min, must be ignored
    assert sector_minimum(msg, 90.0) is None
