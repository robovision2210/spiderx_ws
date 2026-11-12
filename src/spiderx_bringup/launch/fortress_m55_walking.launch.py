"""SpiderX M5.5: FREE-BASE Gazebo Fortress bench for continuous walking with keyboard teleop.

    ros2 launch spiderx_bringup fortress_m55_walking.launch.py
    ros2 run spiderx_controller m55_teleop_keyboard            # second terminal (a TTY)

Includes the unchanged M2 posture-hold bench (fortress_posture_hold.launch.py: world, free-base
model, lidar, gz_ros2_control, joint_state_broadcaster + leg_trajectory_controller, and the
ground-truth bridge /world/spiderx_fortress/pose/info -> /spiderx/sim/world_poses) and adds the
M5.5 locomotion node (spiderx_locomotion).

CONTINUOUS LOCOMOTION DISPATCH IS HARD-DISABLED in this build
(m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED = False): the node runs in SHADOW mode. It
validates commands, runs the state machine and records the phase goals it WOULD send, but it
creates no action client and sends nothing; the robot stays in its posture hold. No launch
argument changes that.

The node designs and validates its crawl templates at start-up (~20 s per speed level) before
its services appear. Simulation development only; no hardware driver, no serial device.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('spiderx_bringup')

    bench = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'fortress_posture_hold.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    locomotion = Node(
        package='spiderx_controller',
        executable='m55_locomotion_node',
        name='spiderx_locomotion',
        output='screen',
        arguments=['--evidence-dir', LaunchConfiguration('evidence_dir')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='true: also open RViz.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: Gazebo server only (EGL rendering; untested).'),
        DeclareLaunchArgument('evidence_dir', default_value='',
                              description='directory for the node evidence files (empty: off)'),
        bench,
        locomotion,
    ])
