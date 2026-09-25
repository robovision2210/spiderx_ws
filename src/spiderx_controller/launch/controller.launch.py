"""Spawn the SpiderX ros2_control controllers in dependency order (M1).

    Included by spiderx_bringup/launch/fortress_control.launch.py; not meant to be run alone.

Order:
  1. joint_state_broadcaster. Its spawner waits until /controller_manager (created by the
     gz_ros2_control plugin inside Gazebo) is available.
  2. leg_trajectory_controller, started only after spawner 1 has exited (event handler, no sleeps).
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

CONTROLLER_MANAGER_TIMEOUT_S = '120'  # time allowed for Gazebo + the robot to come up


def _spawner(controller):
    return Node(
        package='controller_manager',
        executable='spawner',
        name=f'spawner_{controller}',
        output='screen',
        arguments=[controller,
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', CONTROLLER_MANAGER_TIMEOUT_S],
    )


def generate_launch_description():
    joint_state_broadcaster = _spawner('joint_state_broadcaster')
    leg_trajectory_controller = _spawner('leg_trajectory_controller')

    return LaunchDescription([
        joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[leg_trajectory_controller],
        )),
    ])
