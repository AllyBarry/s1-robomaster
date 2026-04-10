import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value=os.environ.get("ROBOT_ID", "0"),
        description="Robot AprilTag ID — used as topic namespace",
    )

    namespace = ["robot_", LaunchConfiguration("robot_id")]

    return LaunchDescription(
        [
            robot_id_arg,
            Node(
                package="robomaster_can_ros_bridge",
                executable="controlling",
                namespace=namespace,
                name="robomaster_controlling",
            ),
            Node(
                package="robomaster_can_ros_bridge",
                executable="sensing",
                namespace=namespace,
                name="robomaster_sensing",
            ),
        ]
    )
