"""SpiderX M6.1-A: FIXED-BASE (welded) simulation for observation and the protected M6.1 replay.

    ros2 launch spiderx_bringup fortress_m61a_fixed_base.launch.py
    ros2 launch spiderx_bringup fortress_m61a_fixed_base.launch.py rviz:=true
    ros2 launch spiderx_bringup fortress_m61a_fixed_base.launch.py headless:=true

SIMULATION ONLY. It starts exactly what fortress_control.launch.py + the posture-hold ground-truth
bridge start, with two deliberate differences:
  * the robot description is spiderx_description/urdf/spiderx_fixed_base.urdf.xacro (the unchanged
    model + ONE fixed joint world -> dummy_link at the provisional mounting transform);
  * the model is spawned at the IDENTITY pose (-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0), because the weld
    joint already carries the mounting transform (Approach B; a non-identity spawn would apply it
    twice, and the M6.1-A checks refuse that as frame_spawn_not_identity).

Nodes: Gazebo (unchanged world), robot_state_publisher, ros_gz_sim create, spiderx_gz_bridge
(fortress_bridge_control.yaml: /clock, /scan), the gz_ros2_control plugin path, the unchanged
controller.launch.py (joint_state_broadcaster, then leg_trajectory_controller: the usual
controllers, which HOLD the current joint positions and receive no goal from this launch), and the
ground-truth bridge /world/spiderx_fortress/pose/info -> /spiderx/sim/world_poses (not /tf).

It sends no trajectory, goal or command, and starts no gait, replay or teleoperation node. The
read-only observer is a separate command (ros2 run spiderx_controller m61a_observe_fixed_base).
There is no argument to change the mount or the spawn pose: the approved provisional mount lives
in the wrapper xacro and config/m61a_fixed_base.yaml (kept equal by a test).
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MODEL_NAME = 'spiderx'
WORLD_NAME = 'spiderx_fortress'
GZ_POSE_TOPIC = f'/world/{WORLD_NAME}/pose/info'
ROS_POSE_TOPIC = '/spiderx/sim/world_poses'
IDENTITY_SPAWN = ['-x', '0', '-y', '0', '-z', '0', '-R', '0', '-P', '0', '-Y', '0']


def generate_launch_description():
    description_share = get_package_share_directory('spiderx_description')
    controller_share = get_package_share_directory('spiderx_controller')
    bringup_share = get_package_share_directory('spiderx_bringup')
    wrapper = os.path.join(description_share, 'urdf', 'spiderx_fixed_base.urdf.xacro')
    world = os.path.join(description_share, 'worlds', 'spiderx_fortress.sdf')
    bridge_config = os.path.join(description_share, 'config', 'fortress_bridge_control.yaml')
    controllers_file = os.path.join(controller_share, 'config', 'spiderx_ros2_controllers.yaml')
    gz_ros2_control_lib = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    resource_root = os.path.dirname(description_share)
    headless = LaunchConfiguration('headless')
    gz_verbosity = LaunchConfiguration('gz_verbosity')

    args = [
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: Gazebo server only (EGL rendering).'),
        DeclareLaunchArgument('rviz', default_value='false', description='true: also open RViz.'),
        DeclareLaunchArgument('gz_verbosity', default_value='2',
                              description='Gazebo console verbosity (0-4).'),
    ]
    env = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_root),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_root),
        AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', gz_ros2_control_lib),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_ros2_control_lib),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', wrapper, ' sim_backend:=fortress enable_control:=true',
                 " controllers_file:='", controllers_file, "'"]),
        value_type=str)

    gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                 'gz_sim.launch.py')
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r -v ', gz_verbosity, ' ', world],
                          'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless))
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-s -r --headless-rendering -v ', gz_verbosity, ' ', world],
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless))

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', name='spawn_spiderx', output='screen',
        arguments=['-name', MODEL_NAME, '-topic', 'robot_description'] + IDENTITY_SPAWN)

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='spiderx_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}])

    ground_truth_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='spiderx_sim_ground_truth_bridge', output='screen',
        arguments=[f'{GZ_POSE_TOPIC}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[(GZ_POSE_TOPIC, ROS_POSE_TOPIC)],
        parameters=[{'use_sim_time': True}])

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'spiderx_fortress.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription(args + env + [
        gazebo_gui, gazebo_headless, robot_state_publisher, spawn_robot, bridge,
        ground_truth_bridge, controllers, rviz])
