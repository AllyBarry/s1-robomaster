from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("robot_navigator")

    robot_ids_csv = LaunchConfiguration("robot_ids").perform(context).strip()
    robot_ids = [int(v) for v in robot_ids_csv.split(",") if v.strip()]
    if not robot_ids:
        raise RuntimeError("robot_ids must not be empty")

    formation = LaunchConfiguration("formation").perform(context)
    spacing = LaunchConfiguration("formation_spacing").perform(context)
    assignment = LaunchConfiguration("assignment").perform(context)
    publish_waypoints = LaunchConfiguration("publish_waypoints").perform(context)

    actions = []

    gf_launch = os.path.join(pkg_share, "launch", "global_feedback.launch.py")
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gf_launch),
        launch_arguments={
            "formation": formation,
            "formation_spacing": spacing,
            "assignment": assignment,
        }.items(),
    ))

    belief_launch = os.path.join(pkg_share, "launch", "belief.launch.py")
    for rid in robot_ids:
        peers = ",".join(str(p) for p in robot_ids if p != rid)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(belief_launch),
            launch_arguments={
                "robot_id": str(rid),
                "publish_waypoints": publish_waypoints,
                "peer_robot_ids": peers,
            }.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ids",
            default_value="0,1,2",
            description="Comma-separated robot IDs to spawn belief nodes for",
        ),
        DeclareLaunchArgument(
            "formation",
            default_value="line",
            description="line | triangle | circle | custom (forwarded to global_feedback)",
        ),
        DeclareLaunchArgument(
            "formation_spacing",
            default_value="0.4",
            description="Line neighbour spacing, or triangle/circle radius (m)",
        ),
        DeclareLaunchArgument(
            "assignment",
            default_value="ordered",
            description="ordered | nearest (forwarded to global_feedback)",
        ),
        DeclareLaunchArgument(
            "publish_waypoints",
            default_value="true",
            description="If true, each belief node also publishes UCB-driven waypoints",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
