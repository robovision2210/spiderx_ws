"""Spawn the SpiderX ros2_control controllers in dependency order (M1; start-up hardened in M4.1).

    Included by spiderx_bringup/launch/fortress_control.launch.py; not meant to be run alone.

Order:
  1. joint_state_broadcaster. Its spawner waits until /controller_manager (created by the
     gz_ros2_control plugin inside Gazebo) is available.
  2. leg_trajectory_controller, started only after spawner 1 has exited with code 0, i.e. after
     joint_state_broadcaster was configured and activated (event handler, no sleeps).

If a spawner fails (non-zero exit code or killed by a signal), one error is logged and no further
spawner is started, so leg_trajectory_controller never runs without joint_state_broadcaster
(/joint_states). During a launch shutdown (Ctrl+C) no further spawner is started either; this is
checked first, because a spawner interrupted by Ctrl+C also exits with code 0.
On failure the launch is deliberately not shut down, so that Ctrl+C stays the one shutdown path:
a launch-initiated shutdown was recorded to leave the Gazebo Fortress server and GUI running as
orphans (docs/M4_1_PLAN.md, section 5).
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch_ros.actions import Node

CONTROLLER_MANAGER_TIMEOUT_S = '120'  # time allowed for Gazebo + the robot to come up
# Simulation-only tuning (M4.1). The controller manager applies an activation in a step of its
# control loop, which gz_ros2_control runs from the Gazebo simulation step, while the spawner waits
# in wall time (default 5 s). On a slow start-up with software rendering 5 s was sometimes too
# short. The service-call timeout must stay longer than the switch timeout: otherwise the spawner
# re-sends the pending request after its default 10 s, and the duplicate fails as "already active".
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


def _failure_message(controller, next_controller, returncode):
    not_started = f' {next_controller} was not started;' if next_controller else ''
    return (f'{controller} startup failed;{not_started} press Ctrl+C and relaunch. '
            f'(spawner exit code {returncode}; see its output above)')


def _start_after_success(controller, next_spawner=None, next_controller=None):
    """OnProcessExit callback for the spawner of `controller`.

    Exit code 0 (the spawner configured and activated `controller`): start `next_spawner`.
    Any other exit code, including a signal: log one error and start nothing.
    During a launch shutdown: start nothing and log nothing.
    """
    def on_exit(event, context):
        if context.is_shutdown:  # checked first: a spawner interrupted by Ctrl+C exits 0 as well
            return None
        if event.returncode == 0:
            return [next_spawner] if next_spawner is not None else None
        get_logger('spiderx_controller').error(
            _failure_message(controller, next_controller, event.returncode))
        return None

    return on_exit


def generate_launch_description():
    joint_state_broadcaster = _spawner('joint_state_broadcaster')
    leg_trajectory_controller = _spawner('leg_trajectory_controller')

    return LaunchDescription([
        joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=_start_after_success('joint_state_broadcaster', leg_trajectory_controller,
                                         'leg_trajectory_controller'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=leg_trajectory_controller,
            on_exit=_start_after_success('leg_trajectory_controller'),
        )),
    ])
