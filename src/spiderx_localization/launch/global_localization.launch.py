"""AMCL + map server for SpiderX. CONFIGURED BUT BLOCKED - not started by any other launch.

    ros2 launch spiderx_localization global_localization.launch.py map:=/path/to/map.yaml

There is no default map: SpiderX has no SLAM-built map yet. AMCL also needs odom -> dummy_link
from real odometry. See STATUS.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to a map .yaml (required)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'amcl_config',
            default_value=os.path.join(get_package_share_directory('spiderx_localization'),
                                       'config', 'amcl.yaml'),
            description='Full path to the AMCL parameter file'),
        LogInfo(msg='[spiderx_localization] BLOCKED: AMCL needs odom -> dummy_link from real '
                    'odometry, which SpiderX does not have yet (see STATUS.md).'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map')},
                        {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('amcl_config'), {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'node_names': ['map_server', 'amcl']},
                        {'use_sim_time': use_sim_time},
                        {'autostart': True}],
        ),
    ])
