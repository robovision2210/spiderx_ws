"""Spawn SpiderX in Gazebo Fortress (Ignition Gazebo 6) through ros_gz_sim.

Starts:
  * Gazebo Fortress with worlds/spiderx_fortress.sdf (ros_gz_sim gz_sim.launch.py)
  * robot_state_publisher with the xacro expanded for sim_backend:=fortress
  * ros_gz_sim `create`, which spawns that same robot_description
  * ros_gz_bridge for /clock, /scan and /joint_states (config/fortress_bridge.yaml)

The robot is passive: no joint is actuated. No Gazebo Classic package, plugin or
environment variable is used here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Lowest point of the robot (foot soles) is 0.0545 m below the root link origin in
# the CAD zero pose (measured from the meshes, docs/SPIDERX_URDF_AUDIT.md section 3).
# Spawning at 0.075 m leaves ~2 cm of clearance above the ground plane.
DEFAULT_SPAWN_Z = '0.075'


def generate_launch_description():
    pkg_share = get_package_share_directory('spiderx_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'spiderx.urdf.xacro')
    default_world = os.path.join(pkg_share, 'worlds', 'spiderx_fortress.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'fortress_bridge.yaml')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    gz_verbosity = LaunchConfiguration('gz_verbosity')

    declare_args = [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Absolute path to the Gazebo Fortress world (SDF).'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: run the Gazebo server only (no GUI) with headless '
                        'EGL rendering for the lidar.'),
        DeclareLaunchArgument(
            'gz_verbosity', default_value='2',
            description='Gazebo console verbosity (0-4).'),
        DeclareLaunchArgument(
            'robot_name', default_value='spiderx',
            description='Name of the spawned Gazebo model.'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Spawn x [m].'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Spawn y [m].'),
        DeclareLaunchArgument(
            'spawn_z', default_value=DEFAULT_SPAWN_Z,
            description='Spawn height of the root link [m]; must stay above 0.0545.'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Spawn yaw [rad].'),
    ]

    # Let Gazebo resolve model:// URIs under this package's share tree. Fortress reads
    # IGN_GAZEBO_RESOURCE_PATH; GZ_SIM_RESOURCE_PATH is the newer name read by later
    # Fortress releases and Garden+. Both are appended, never overwritten. The robot
    # meshes themselves use absolute file:// URIs produced by $(find ...) in the xacro.
    resource_root = os.path.dirname(pkg_share)
    set_resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_root),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_root),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_backend:=fortress']),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    # GUI + server, simulation running from the start (-r).
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': ['-r -v ', gz_verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )

    # Server only; the lidar still renders through EGL (--headless-rendering).
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': ['-s -r --headless-rendering -v ', gz_verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_spiderx',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='spiderx_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    return LaunchDescription(
        declare_args
        + set_resource_paths
        + [
            gazebo_gui,
            gazebo_headless,
            robot_state_publisher,
            spawn_robot,
            bridge,
        ]
    )
