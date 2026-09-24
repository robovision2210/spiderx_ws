import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    spiderx_description = get_package_share_directory("spiderx_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(spiderx_description, "urdf", "spiderx.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(spiderx_description, "worlds", "spiderx_world.world"),
        description="Absolute path to world file",
    )

    model_path = str(Path(spiderx_description).parent.resolve())
    model_path += pathsep + spiderx_description

    gazebo_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "spiderx"],
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        gazebo_model_path,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
    ])
