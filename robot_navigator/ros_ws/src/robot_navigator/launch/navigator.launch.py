from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("robot_navigator")
    nav_yaml = os.path.join(pkg_share, "config", "navigator.yaml")

    robot_id = int(LaunchConfiguration("robot_id").perform(context))
    align_heading = LaunchConfiguration("align_heading").perform(context).lower() == "true"
    peer_csv = LaunchConfiguration("peer_robot_ids").perform(context).strip()
    peer_ids = [int(v) for v in peer_csv.split(",") if v.strip()]

    navigator = Node(
        package="robot_navigator",
        executable="navigator",
        name="robot_navigator",
        parameters=[
            nav_yaml,
            {
                "robot_id": robot_id,
                "align_heading": align_heading,
                "peer_robot_ids": peer_ids,
            },
        ],
        output="screen",
    )
    return [navigator]


def generate_launch_description():
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
    peer_ids_arg = DeclareLaunchArgument(
        "peer_robot_ids",
        default_value="",
        description="Comma-separated peer robot IDs for reactive collision avoidance",
    )

    return LaunchDescription([
        robot_id_arg,
        align_heading_arg,
        peer_ids_arg,
        OpaqueFunction(function=_launch_setup),
    ])
