from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_navigator")
    cfg = os.path.join(pkg_share, "config", "global_feedback.yaml")

    formation_arg = DeclareLaunchArgument(
        "formation",
        default_value="triangle",
        description="line | triangle | circle | custom",
    )
    spacing_arg = DeclareLaunchArgument(
        "formation_spacing",
        default_value="0.5",
        description="Line step or ring radius (m)",
    )
    assignment_arg = DeclareLaunchArgument(
        "assignment",
        default_value="ordered",
        description="ordered | nearest",
    )

    feedback = Node(
        package="robot_navigator",
        executable="global_feedback",
        name="global_feedback",
        parameters=[
            cfg,
            {
                "formation": LaunchConfiguration("formation"),
                "formation_spacing": LaunchConfiguration("formation_spacing"),
                "assignment": LaunchConfiguration("assignment"),
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        formation_arg,
        spacing_arg,
        assignment_arg,
        feedback,
    ])
