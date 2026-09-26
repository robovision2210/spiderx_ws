"""Spawn the SpiderX ros2_control controllers in dependency order (M1).

    Included by spiderx_bringup/launch/fortress_control.launch.py; not meant to be run alone.

Order:
  1. joint_state_broadcaster. Its spawner waits until /controller_manager (created by the
     gz_ros2_control plugin inside Gazebo) is available.
  2. leg_trajectory_controller, started only after spawner 1 has exited with return code 0
     (event handler, no sleeps).
If a spawner exits with a non-zero code, an error is logged and nothing further is started, so
leg_trajectory_controller can never be active while joint_state_broadcaster is not (no
/joint_states). The launch is deliberately not shut down from here: a launch-initiated shutdown
leaves the Gazebo Fortress server and GUI running as orphans, whereas Ctrl-C stops them cleanly.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch_ros.actions import Node

CONTROLLER_MANAGER_TIMEOUT_S = '120'  # time allowed for Gazebo + the robot to come up
# Simulation-only tuning. An activation completes only on a controller_manager update, which
# gz_ros2_control drives from simulation time, but the spawner waits in wall time (default 5 s).
# With software rendering (real-time factor ~0.25) and a simulation that is still starting, 5 s
# was sometimes too short. The service-call timeout must be longer than the switch timeout:
# otherwise the spawner re-sends the pending switch request (after 10 s by default) and the
# controller_manager rejects the duplicate as "already active".
SWITCH_TIMEOUT_S = '60'
SERVICE_CALL_TIMEOUT_S = '75'


def _spawner(controller):
    return Node(
        package='controller_manager',
        executable='spawner',
        name=f'spawner_{controller}',
        output='screen',
        arguments=[controller,
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', CONTROLLER_MANAGER_TIMEOUT_S,
                   '--switch-timeout', SWITCH_TIMEOUT_S,
                   '--service-call-timeout', SERVICE_CALL_TIMEOUT_S],
    )


def _on_success(spawner, controller, then=()):
    """Run `then` when `spawner` exits with code 0; otherwise log an error and start nothing."""
    def on_exit(event, context):
        if context.is_shutdown:  # normal teardown (Ctrl-C): start nothing, report nothing
            return None
        if event.returncode == 0:
            return list(then)
        get_logger('spiderx_controller').error(
            f'spawner for {controller} failed (exit code {event.returncode})'
            + (', so the next controller was NOT started' if then else '')
            + '. The controllers are not usable: stop this launch (Ctrl-C) and start it again.'
            ' See the spawner output above.')
        return None

    return RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=on_exit))


def generate_launch_description():
    joint_state_broadcaster = _spawner('joint_state_broadcaster')
    leg_trajectory_controller = _spawner('leg_trajectory_controller')

    return LaunchDescription([
        joint_state_broadcaster,
        _on_success(joint_state_broadcaster, 'joint_state_broadcaster',
                    then=[leg_trajectory_controller]),
        _on_success(leg_trajectory_controller, 'leg_trajectory_controller'),
    ])
