"""
Single-process onboard vision pipeline: camera → rectify → apriltag, plus
the field_localizer / overlay / visualizer standalone Python nodes.

Running the camera and image processing in one ComposableNodeContainer
keeps raw images in intra-process memory — they never cross DDS. This
avoids the fragmented-large-message failure mode we hit when the camera
and apriltag ran in separate containers (rectify silently saw zero
images despite image_raw flowing on the network).

Network-exposed topics (what downstream consumers should use):
  /detections                    AprilTagDetectionArray
  /field/robot_{id}/pose         PoseStamped (via field_localizer)
  /target_markers                MarkerArray  (via field_visualizer)
  /detections/image/compressed   CompressedImage (throttled, for RViz)

Raw / rectified image topics are NOT recommended for remote consumers:
even though they're advertised on DDS, subscribing across the network
re-introduces the fragmentation problem. Use `publish_raw:=true` only
for local calibration / bag recording.
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

    video_device = LaunchConfiguration("video_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_info_url = LaunchConfiguration("camera_info_url")
    overlay_rate = LaunchConfiguration("overlay_rate_hz")

    # Everything that touches raw pixels lives in one process so the
    # camera → rectify → apriltag hop is intra-process (zero-copy, no
    # DDS serialization). use_intra_process_comms=True is the switch
    # that actually makes that work.
    image_container = ComposableNodeContainer(
        name="image_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="v4l2_camera",
                plugin="v4l2_camera::V4L2Camera",
                name="webcam",
                parameters=[{
                    "video_device": video_device,
                    "image_size": [image_width, image_height],
                    "pixel_format": "YUYV",
                    "io_method": "read",
                    "camera_frame_id": "webcam_link",
                    "camera_info_url": camera_info_url,
                }],
                remappings=[
                    ("image_raw", "/webcam/image_raw"),
                    ("camera_info", "/webcam/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
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
            "video_device",
            default_value="/dev/video20",
            description="V4L2 video device path (defaults to camera_broker loopback; pass /dev/video1 to bypass)",
        ),
        DeclareLaunchArgument(
            "image_width",
            default_value="640",
            description="Image width (pixels)",
        ),
        DeclareLaunchArgument(
            "image_height",
            default_value="480",
            description="Image height (pixels)",
        ),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value="file:///root/.ros/camera_info/webcam.yaml",
            description="Calibration file path (file:// URL)",
        ),
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
