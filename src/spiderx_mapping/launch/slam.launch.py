"""SLAM Toolbox mapping for SpiderX. CONFIGURED BUT BLOCKED - not started by any other launch.

    ros2 launch spiderx_mapping slam.launch.py

Requires an odom -> dummy_link transform from real odometry, which SpiderX does not have yet
(no gait, no leg odometry). Without it SLAM Toolbox only waits for TF. See STATUS.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config = LaunchConfiguration('slam_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam_config',
            default_value=os.path.join(get_package_share_directory('spiderx_mapping'),
                                       'config', 'slam_toolbox.yaml'),
            description='Full path to the SLAM Toolbox parameter file'),
        LogInfo(msg='[spiderx_mapping] BLOCKED: SLAM needs odom -> dummy_link from real '
                    'odometry. SpiderX has no locomotion/odometry yet (see STATUS.md).'),
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver_server',
            output='screen',
            parameters=[{'save_map_timeout': 5.0}, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'node_names': ['map_saver_server']},
                        {'use_sim_time': use_sim_time},
                        {'autostart': True}],
        ),
    ])
