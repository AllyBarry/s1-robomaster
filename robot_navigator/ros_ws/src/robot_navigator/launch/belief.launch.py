from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_navigator")
    cfg = os.path.join(pkg_share, "config", "belief.yaml")

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

    belief = Node(
        package="robot_navigator",
        executable="belief",
        name="belief_node",
        parameters=[
            cfg,
            {
                "robot_id": LaunchConfiguration("robot_id"),
                "publish_waypoints": LaunchConfiguration("publish_waypoints"),
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_id_arg,
        publish_waypoints_arg,
        belief,
    ])
