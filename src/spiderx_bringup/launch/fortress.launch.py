"""SpiderX simulation entry point: Gazebo Fortress + ROS 2 bridge (+ optional RViz).

    ros2 launch spiderx_bringup fortress.launch.py
    ros2 launch spiderx_bringup fortress.launch.py rviz:=true
    ros2 launch spiderx_bringup fortress.launch.py headless:=true

All simulation arguments (world, headless, spawn_x/y/z/yaw, gz_verbosity, ...) are
declared in spiderx_description/launch/fortress.launch.py and are accepted here.

This launch never starts the hardware lidar driver, SLAM, AMCL or Nav2.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('spiderx_description')
    bringup_share = get_package_share_directory('spiderx_bringup')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='true: also open RViz with the robot model, TF and /scan.')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'fortress.launch.py')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'spiderx_fortress.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, simulation, rviz])
