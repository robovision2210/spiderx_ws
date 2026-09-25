"""Real-robot lidar bring-up: robot_state_publisher + Slamtec RPLidar A1 driver.

    ros2 launch spiderx_bringup hardware_lidar.launch.py serial_port:=/dev/ttyUSB0

For the physical robot only. It opens the serial port, so it must never be
included in a simulation launch. Scans are published on /scan in the lidar_link
frame defined in the URDF.

Limitation: the servos have no ROS driver yet, so nothing publishes /joint_states
on hardware. robot_state_publisher therefore only provides the fixed transforms
(dummy_link -> base_link -> lidar_link, top_1, hip servo bodies). The leg frames
are unavailable until a servo driver exists.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('spiderx_description'), 'urdf', 'spiderx.urdf.xacro')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial device of the RPLidar A1.')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file, ' sim_backend:=none']), value_type=str),
            'use_sim_time': False,
        }],
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'),
                         'launch', 'rplidar_a1_launch.py')),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'frame_id': 'lidar_link',
        }.items(),
    )

    return LaunchDescription([serial_port_arg, robot_state_publisher, rplidar])
