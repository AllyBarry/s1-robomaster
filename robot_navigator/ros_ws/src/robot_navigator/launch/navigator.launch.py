from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_navigator")
    nav_yaml = os.path.join(pkg_share, "config", "navigator.yaml")

    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="0",
        description="ID of the robot this navigator instance controls",
    )
    align_heading_arg = DeclareLaunchArgument(
        "align_heading",
        default_value="false",
        description="If true, rotate to match the goal pose's yaw (else position-only)",
    )

    navigator = Node(
        package="robot_navigator",
        executable="navigator",
        name="robot_navigator",
        parameters=[
            nav_yaml,
            {
                "robot_id": LaunchConfiguration("robot_id"),
                "align_heading": LaunchConfiguration("align_heading"),
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_id_arg,
        align_heading_arg,
        navigator,
    ])
