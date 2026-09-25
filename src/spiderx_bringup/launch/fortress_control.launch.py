"""SpiderX M1: Gazebo Fortress with gz_ros2_control joint position control.

    ros2 launch spiderx_bringup fortress_control.launch.py
    ros2 launch spiderx_bringup fortress_control.launch.py rviz:=true
    ros2 launch spiderx_bringup fortress_control.launch.py enable_control:=false   # passive

Starts the verified Fortress world, model, lidar and bridge (spiderx_description/fortress.launch.py)
with enable_control:=true. That adds gz_ros2_control and the controller_manager inside Gazebo, and
this file then spawns joint_state_broadcaster and leg_trajectory_controller in order.

This is joint-position control only: no standing controller, gait, /cmd_vel or odometry.
It never starts hardware drivers and opens no serial device.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('spiderx_description')
    controller_share = get_package_share_directory('spiderx_controller')
    bringup_share = get_package_share_directory('spiderx_bringup')
    enable_control = LaunchConfiguration('enable_control')

    # Make sure Fortress finds the gz_ros2_control system plugin (libgz_ros2_control-system.so)
    # without relying on LD_LIBRARY_PATH, which gz_sim.launch.py uses as the default search path.
    gz_ros2_control_lib = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    plugin_paths = [
        AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', gz_ros2_control_lib),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_ros2_control_lib),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'fortress.launch.py')),
        launch_arguments={
            'enable_control': enable_control,
            'controllers_file': os.path.join(controller_share, 'config',
                                             'spiderx_ros2_controllers.yaml'),
        }.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')),
        condition=IfCondition(enable_control),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'spiderx_fortress.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(plugin_paths + [
        DeclareLaunchArgument(
            'enable_control', default_value='true',
            description='true: gz_ros2_control + controllers (M1). false: passive simulation.'),
        DeclareLaunchArgument('rviz', default_value='false', description='true: also open RViz.'),
        simulation,
        controllers,
        rviz,
    ])
