"""
Onboard vision pipeline: compressed camera input → rectify → apriltag, plus
the field_localizer / overlay / visualizer standalone Python nodes.

The camera itself runs in the separate `rpi_camera_streamer` container,
which owns /dev/video1 and publishes /webcam/image_raw (raw) plus
/webcam/image_raw/compressed (JPEG, via image_transport_plugins). This
container subscribes to the compressed topic — small messages don't hit
the DDS fragmentation failure mode that raw cross-container images did.
rectify decompresses on arrival and hands raw pixels intra-process to
apriltag, so the hot path (rectify → apriltag) is still zero-copy.

Network-exposed topics (what downstream consumers should use):
  /detections                    AprilTagDetectionArray
  /field/robot_{id}/pose         PoseStamped (via field_localizer)
  /target_markers                MarkerArray  (via field_visualizer)
  /detections/image/compressed   CompressedImage (throttled, for RViz)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("apriltag_detector")
    apriltag_yaml = os.path.join(pkg_share, "config", "apriltag.yaml")
    field_yaml = os.path.join(pkg_share, "config", "field.yaml")

    overlay_rate = LaunchConfiguration("overlay_rate_hz")

    # rectify subscribes to /webcam/image_raw using the "compressed" image
    # transport (so the wire format is JPEG), decompresses, and emits raw
    # image_rect intra-process to apriltag.
    image_container = ComposableNodeContainer(
        name="image_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
                parameters=[{"image_transport": "compressed"}],
                remappings=[
                    ("image", "/webcam/image_raw"),
                    ("camera_info", "/webcam/camera_info"),
                    ("image_rect", "/webcam/image_rect"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                parameters=[apriltag_yaml],
                remappings=[
                    ("image_rect", "/webcam/image_rect"),
                    ("camera_info", "/webcam/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    # Standalone Python nodes — they consume small topics (detections)
    # or already-throttled image streams, so no gain from composing them.
    field_localizer = Node(
        package="apriltag_detector",
        executable="field_localizer",
        name="field_localizer",
        parameters=[field_yaml],
        output="screen",
    )

    detection_overlay = Node(
        package="apriltag_detector",
        executable="detection_overlay",
        name="detection_overlay",
        parameters=[
            field_yaml,
            {"publish_rate_hz": overlay_rate},
        ],
        remappings=[
            ("image_raw", "/webcam/image_rect"),
        ],
        output="screen",
    )

    field_visualizer = Node(
        package="apriltag_detector",
        executable="field_visualizer",
        name="field_visualizer",
        parameters=[field_yaml],
        output="screen",
    )

    robot_boundary_visualizer = Node(
        package="apriltag_detector",
        executable="robot_boundary_visualizer",
        name="robot_boundary_visualizer",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "overlay_rate_hz",
            default_value="5.0",
            description="Max rate for the compressed detections/image debug stream",
        ),
        image_container,
        field_localizer,
        detection_overlay,
        field_visualizer,
        robot_boundary_visualizer,
    ])
