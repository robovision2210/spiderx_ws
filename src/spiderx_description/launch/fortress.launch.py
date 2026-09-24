import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    spiderx_description = get_package_share_directory("spiderx_description")
    world_path = os.path.join(spiderx_description, "worlds", "spiderx_fortress.sdf")
    xacro_path = os.path.join(spiderx_description, "urdf", "spiderx.urdf.xacro")
    generated_sdf_path = "/tmp/spiderx_fortress_model.sdf"

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Fortress world",
    )

    model_search_path = str(Path(spiderx_description).parent.resolve())
    model_search_path += pathsep + spiderx_description

    set_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_search_path,
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_path,
                " enable_classic_gazebo:=false enable_fortress_lidar:=true",
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gazebo_fortress = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),
    )

    generate_sdf = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                f"xacro {xacro_path} "
                "enable_classic_gazebo:=false "
                "enable_fortress_lidar:=true "
                f"| gz sdf -p > {generated_sdf_path}"
            ),
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "spiderx", "-file", generated_sdf_path, "-z", "0.25"],
    )

    spawn_after_sdf = RegisterEventHandler(
        OnProcessExit(target_action=generate_sdf, on_exit=[spawn_robot])
    )

    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            world_arg,
            set_resource_path,
            robot_state_publisher,
            gazebo_fortress,
            generate_sdf,
            spawn_after_sdf,
            lidar_bridge,
        ]
    )
