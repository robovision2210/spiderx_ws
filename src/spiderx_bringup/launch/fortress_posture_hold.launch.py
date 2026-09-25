"""SpiderX M2: SIMULATION-ONLY posture-hold test bench in Gazebo Fortress.

    ros2 launch spiderx_bringup fortress_posture_hold.launch.py
    ros2 run spiderx_controller run_posture_hold_test.py        # second terminal

Includes the unchanged M1 launch (fortress_control.launch.py: world, model, lidar, gz_ros2_control
and both controllers) and adds ONE extra one-way bridge for the test:

    Gazebo /world/spiderx_fortress/pose/info (gz.msgs.Pose_V, SceneBroadcaster ground truth)
      -> ROS /spiderx/sim/world_poses (tf2_msgs/msg/TFMessage)

It is remapped away from /tf on purpose: it is simulation ground truth for measuring body height
and tilt, not odometry, and it never enters the TF tree.

Simulation-only posture hold. Not balance, walking, IK or hardware validation.
It never starts hardware drivers and opens no serial device.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = 'spiderx_fortress'   # <world name> in spiderx_description/worlds/spiderx_fortress.sdf
GZ_POSE_TOPIC = f'/world/{WORLD_NAME}/pose/info'
ROS_POSE_TOPIC = '/spiderx/sim/world_poses'


def generate_launch_description():
    bringup_share = get_package_share_directory('spiderx_bringup')

    control_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'fortress_control.launch.py')),
        launch_arguments={
            'enable_control': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='spiderx_sim_ground_truth_bridge',
        output='screen',
        arguments=[f'{GZ_POSE_TOPIC}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[(GZ_POSE_TOPIC, ROS_POSE_TOPIC)],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='true: also open RViz.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: Gazebo server only (EGL rendering; untested).'),
        control_sim,
        ground_truth_bridge,
    ])
