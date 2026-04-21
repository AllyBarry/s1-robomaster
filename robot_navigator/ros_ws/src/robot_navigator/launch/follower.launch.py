from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_navigator")
    cfg_yaml = os.path.join(pkg_share, "config", "follower.yaml")

    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="0",
        description="ID of the follower robot",
    )
    target_id_arg = DeclareLaunchArgument(
        "target_robot_id",
        default_value="1",
        description="ID of the robot to follow",
    )
    distance_arg = DeclareLaunchArgument(
        "follow_distance",
        default_value="0.5",
        description="Target separation (metres) from the leader",
    )

    follower = Node(
        package="robot_navigator",
        executable="follower",
        name="follower",
        parameters=[
            cfg_yaml,
            {
                "robot_id": LaunchConfiguration("robot_id"),
                "target_robot_id": LaunchConfiguration("target_robot_id"),
                "follow_distance": LaunchConfiguration("follow_distance"),
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_id_arg,
        target_id_arg,
        distance_arg,
        follower,
    ])
