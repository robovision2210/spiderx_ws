"""Real Slamtec RPLidar A1 driver for the physical SpiderX (hardware only).

    ros2 launch spiderx_firmware lidar.launch.py serial_port:=/dev/ttyUSB0

Publishes sensor_msgs/LaserScan on /scan in the URDF frame lidar_link, the same topic and frame
that the Gazebo Fortress simulation publishes. It opens a serial port, so no simulation launch
file may include it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial device of the RPLidar A1.')

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'),
                         'launch', 'rplidar_a1_launch.py')),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'frame_id': 'lidar_link',
        }.items(),
    )

    return LaunchDescription([serial_port_arg, rplidar])
