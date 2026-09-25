"""EKF (robot_localization) for SpiderX. CONFIGURED BUT BLOCKED - not started by any other launch.

    ros2 launch spiderx_localization local_localization.launch.py

Its inputs (/spiderx/leg_odometry, /imu/data) do not exist yet; see config/ekf.yaml and STATUS.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(get_package_share_directory('spiderx_localization'),
                                       'config', 'ekf.yaml'),
            description='Full path to the robot_localization EKF parameter file'),
        LogInfo(msg='[spiderx_localization] BLOCKED: EKF inputs /spiderx/leg_odometry and '
                    '/imu/data do not exist yet (see STATUS.md).'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('ekf_config'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
