from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    center_x = LaunchConfiguration("formation_center_x").perform(context)
    center_y = LaunchConfiguration("formation_center_y").perform(context)
    assignment = LaunchConfiguration("assignment").perform(context)
    publish_waypoints = LaunchConfiguration("publish_waypoints").perform(context)

    actions = []

    # global_feedback is spawned directly (not via IncludeLaunchDescription)
    # so we can override `robot_ids` alongside the formation args. Keeps the
    # bundle self-contained without teaching global_feedback.launch.py about
    # every parameter a scenario might want to override.
    gf_cfg = os.path.join(pkg_share, "config", "global_feedback.yaml")
    actions.append(Node(
        package="robot_navigator",
        executable="global_feedback",
        name="global_feedback",
        parameters=[
            gf_cfg,
            {
                "formation": formation,
                "formation_spacing": float(spacing),
                "formation_center_x": float(center_x),
                "formation_center_y": float(center_y),
                "assignment": assignment,
                "robot_ids": robot_ids,
            },
        ],
        output="screen",
    ))

    # belief.launch.py already accepts every per-robot knob we need, so
    # we compose it via IncludeLaunchDescription — no point duplicating
    # its parameter-handling logic here.
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
            description="Comma-separated robot IDs — applied to global_feedback and spawned as beliefs",
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
            "formation_center_x",
            default_value="1.5",
            description="Formation centre x (field is 3.0 m wide → 1.5 m = middle)",
        ),
        DeclareLaunchArgument(
            "formation_center_y",
            default_value="1.0",
            description="Formation centre y (field is 2.0 m tall → 1.0 m = middle)",
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
