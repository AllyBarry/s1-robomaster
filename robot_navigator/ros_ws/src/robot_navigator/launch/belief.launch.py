from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("robot_navigator")
    cfg = os.path.join(pkg_share, "config", "belief.yaml")

    robot_id = LaunchConfiguration("robot_id").perform(context)
    publish_waypoints = LaunchConfiguration("publish_waypoints").perform(context)
    peer_csv = LaunchConfiguration("peer_robot_ids").perform(context).strip()
    peer_ids = [int(v) for v in peer_csv.split(",") if v.strip()]

    belief = Node(
        package="robot_navigator",
        executable="belief",
        name="belief_node",
        parameters=[
            cfg,
            {
                "robot_id": int(robot_id),
                "publish_waypoints": publish_waypoints.lower() == "true",
                "peer_robot_ids": peer_ids,
            },
        ],
        output="screen",
    )
    return [belief]


def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="0",
        description="ID of the robot whose belief we're tracking",
    )
    publish_waypoints_arg = DeclareLaunchArgument(
        "publish_waypoints",
        default_value="false",
        description="If true, the belief node also publishes UCB-driven waypoints",
    )
    peer_ids_arg = DeclareLaunchArgument(
        "peer_robot_ids",
        default_value="",
        description="Comma-separated peer robot IDs for repulsive collision avoidance",
    )

    return LaunchDescription([
        robot_id_arg,
        publish_waypoints_arg,
        peer_ids_arg,
        OpaqueFunction(function=_launch_setup),
    ])
