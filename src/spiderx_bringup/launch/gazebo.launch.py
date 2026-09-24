import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    spiderx_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("spiderx_description"),
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rplidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rplidar_ros"),
                "launch",
                "rplidar_a1_launch.py",
            )
        ),
        launch_arguments={
            "serial_port": "/dev/ttyUSB0",
            "frame_id": "lidar_link",
        }.items(),
    )

    return LaunchDescription([
        spiderx_description_launch,
        joint_state_publisher,
        rplidar_driver,
    ])
