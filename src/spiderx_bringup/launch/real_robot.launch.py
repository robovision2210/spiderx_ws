"""SpiderX real-robot entry point (hardware only; never used in simulation).

    ros2 launch spiderx_bringup real_robot.launch.py serial_port:=/dev/ttyUSB0

Starts today:
  * robot_state_publisher with the description expanded for sim_backend:=none
  * the RPLidar A1 driver (spiderx_firmware/launch/lidar.launch.py) publishing /scan in lidar_link

Not started, because it does not exist yet: the servo/actuator interface. Nothing publishes
/joint_states on hardware, so robot_state_publisher provides only the fixed transforms
(dummy_link -> base_link -> lidar_link, top_1, hip servo bodies). Leg frames are unavailable
until the actuator interface exists (docs/SPIDERX_HARDWARE_INTERFACE.md).
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

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('spiderx_firmware'),
                         'launch', 'lidar.launch.py')),
        launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items(),
    )

    return LaunchDescription([serial_port_arg, robot_state_publisher, lidar])
