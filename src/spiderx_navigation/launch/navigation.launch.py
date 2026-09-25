"""Nav2 servers for SpiderX. CONFIGURED BUT BLOCKED - not started by any other launch.

    ros2 launch spiderx_navigation navigation.launch.py

Nav2 publishes /cmd_vel, but nothing on SpiderX consumes it yet (no gait controller), and Nav2
needs odom -> dummy_link from real odometry. See STATUS.md and docs/SPIDERX_DEVELOPMENT_ROADMAP.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = [
    # (package, executable, node name, config file)
    ('nav2_controller', 'controller_server', 'controller_server', 'controller_server.yaml'),
    ('nav2_planner', 'planner_server', 'planner_server', 'planner_server.yaml'),
    ('nav2_smoother', 'smoother_server', 'smoother_server', 'smoother_server.yaml'),
    ('nav2_behaviors', 'behavior_server', 'behavior_server', 'behavior_server.yaml'),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', 'bt_navigator.yaml'),
    ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', 'waypoint_follower.yaml'),
]


def generate_launch_description():
    pkg = get_package_share_directory('spiderx_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bt_xml = os.path.join(pkg, 'behavior_tree',
                          'spiderx_navigate_w_replanning_and_recovery.xml')

    nodes = []
    for package, executable, name, config in SERVERS:
        params = [os.path.join(pkg, 'config', config), {'use_sim_time': use_sim_time}]
        if name == 'bt_navigator':
            params.append({'default_nav_to_pose_bt_xml': bt_xml,
                           'default_nav_through_poses_bt_xml': bt_xml})
        nodes.append(Node(package=package, executable=executable, name=name,
                          output='screen', parameters=params))

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'node_names': [s[2] for s in SERVERS]},
                    {'use_sim_time': use_sim_time},
                    {'autostart': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        LogInfo(msg='[spiderx_navigation] BLOCKED: no /cmd_vel consumer (gait controller) and no '
                    'odometry exist yet; Nav2 cannot move SpiderX (see STATUS.md).'),
    ] + nodes + [lifecycle])
