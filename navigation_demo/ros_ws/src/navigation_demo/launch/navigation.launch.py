from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("navigation_demo")
    nav_yaml = os.path.join(pkg_share, "config", "navigation.yaml")
    field_yaml = os.path.join(pkg_share, "config", "field.yaml")

    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="4",
        description="AprilTag ID of the robot to control",
    )

    go_to_point = Node(
        package="navigation_demo",
        executable="go_to_point",
        name="go_to_point",
        parameters=[
            nav_yaml,
            {"robot_id": LaunchConfiguration("robot_id")},
        ],
        output="screen",
    )

    field_visualizer = Node(
        package="navigation_demo",
        executable="field_visualizer",
        name="field_visualizer",
        parameters=[field_yaml],
        output="screen",
    )

    return LaunchDescription([
        robot_id_arg,
        go_to_point,
        field_visualizer,
    ])
