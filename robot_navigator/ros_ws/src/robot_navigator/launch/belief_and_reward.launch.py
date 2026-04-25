from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
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
    collision_avoidance = LaunchConfiguration("collision_avoidance").perform(context).lower() == "true"
    scenario = LaunchConfiguration("scenario").perform(context)
    duration_sec = LaunchConfiguration("duration_sec").perform(context)
    log_dir = LaunchConfiguration("log_dir").perform(context)
    sample_hz = LaunchConfiguration("sample_hz").perform(context)
    record_video = LaunchConfiguration("record_video").perform(context).lower() == "true"
    video_fps = LaunchConfiguration("video_fps").perform(context)
    hold_enabled = LaunchConfiguration("hold_enabled").perform(context)
    rotation_rate = LaunchConfiguration("rotation_rate").perform(context)
    soft_hold_enabled = LaunchConfiguration("soft_hold_enabled").perform(context)

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
                "rotation_rate_rad_per_sec": float(rotation_rate),
            },
        ],
        output="screen",
    ))

    # belief.launch.py already accepts every per-robot knob we need, so
    # we compose it via IncludeLaunchDescription — no point duplicating
    # its parameter-handling logic here.
    belief_launch = os.path.join(pkg_share, "launch", "belief.launch.py")
    for rid in robot_ids:
        peers = ",".join(str(p) for p in robot_ids if p != rid) if collision_avoidance else ""
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(belief_launch),
            launch_arguments={
                "robot_id": str(rid),
                "publish_waypoints": publish_waypoints,
                "peer_robot_ids": peers,
                "hold_enabled": hold_enabled,
                "soft_hold_enabled": soft_hold_enabled,
            }.items(),
        ))

    # trajectory_logger records per-tick robot poses and global_reward into
    # a CSV plus a JSON sidecar with targets — consumed offline by
    # scripts/plot_experiment.py.
    # on_exit=Shutdown() — when trajectory_logger hits duration_sec and
    # calls os._exit(0), cascade a clean SIGINT to every sibling node so
    # video_recorder can finalize its mp4 moov atom before the container
    # is torn down (otherwise docker compose eventually SIGKILLs it and
    # the file is unplayable).
    actions.append(Node(
        package="robot_navigator",
        executable="trajectory_logger",
        name="trajectory_logger",
        parameters=[{
            "robot_ids": robot_ids,
            "scenario": scenario,
            "duration_sec": float(duration_sec),
            "log_dir": log_dir,
            "sample_hz": float(sample_hz),
        }],
        output="screen",
        on_exit=Shutdown(),
    ))

    # Optional: record the overhead camera with per-robot trail overlays
    # to {log_dir}/{scenario}.mp4. Relies on field_localizer publishing
    # /field/homography_inv so trails can be projected into pixel space.
    if record_video:
        actions.append(Node(
            package="robot_navigator",
            executable="video_recorder",
            name="video_recorder",
            parameters=[{
                "robot_ids": robot_ids,
                "scenario": scenario,
                "log_dir": log_dir,
                "fps": float(video_fps),
            }],
            output="screen",
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
            default_value="1.5",
            description="Formation centre y (field is 3.0 m tall → 1.5 m = middle)",
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
        DeclareLaunchArgument(
            "collision_avoidance",
            default_value="false",
            description="If true, wire peer_robot_ids into each belief so UCB waypoints repel peers",
        ),
        DeclareLaunchArgument(
            "scenario",
            default_value="run",
            description="Episode name — becomes the CSV/JSON filename stem in log_dir",
        ),
        DeclareLaunchArgument(
            "duration_sec",
            default_value="0.0",
            description="Auto-stop logger after this many seconds. 0 = run until killed.",
        ),
        DeclareLaunchArgument(
            "log_dir",
            default_value="/ros_ws/experiment_logs",
            description="Where the CSV + JSON sidecar are written (container path)",
        ),
        DeclareLaunchArgument(
            "sample_hz",
            default_value="10.0",
            description="Logger sampling rate (Hz)",
        ),
        DeclareLaunchArgument(
            "record_video",
            default_value="false",
            description="If true, record the overhead camera with trajectory overlays to {scenario}.mp4",
        ),
        DeclareLaunchArgument(
            "video_fps",
            default_value="5.0",
            description="Output video frame rate (match detection_overlay's publish_rate_hz for real-time playback)",
        ),
        DeclareLaunchArgument(
            "hold_enabled",
            default_value="false",
            description=(
                "Enable the 1-bit hold/probe coordination channel on every belief. "
                "OFF by default — the probe model is run via ./run_belief_and_reward_hold.sh."
            ),
        ),
        DeclareLaunchArgument(
            "rotation_rate",
            default_value="0.0",
            description=(
                "If non-zero, formation rotates around its centre at this "
                "rad/s rate. Used by experiment_rotation to test the "
                "BLR forgetting factor under a drifting reward field."
            ),
        ),
        DeclareLaunchArgument(
            "soft_hold_enabled",
            default_value="false",
            description=(
                "Enable the soft (smooth) hold/probe variant on every "
                "belief. Forwarded to belief.launch.py as a per-instance arg."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
