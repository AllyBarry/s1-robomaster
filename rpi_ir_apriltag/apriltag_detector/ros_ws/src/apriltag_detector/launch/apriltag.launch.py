from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("apriltag_detector")
    apriltag_yaml = os.path.join(pkg_share, "config", "apriltag.yaml")
    field_yaml = os.path.join(pkg_share, "config", "field.yaml")

    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/webcam/image_raw",
        description="Raw image topic to subscribe to",
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/webcam/camera_info",
        description="Camera info topic to subscribe to",
    )

    # Single 36h11 detector — picks up both corner and robot tags
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        parameters=[apriltag_yaml],
        remappings=[
            ("image_rect", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
        ],
        output="screen",
    )

    # Field localizer — splits corner vs robot tags by ID
    field_localizer = Node(
        package="apriltag_detector",
        executable="field_localizer",
        name="field_localizer",
        parameters=[field_yaml],
        output="screen",
    )

    overlay = Node(
        package="apriltag_detector",
        executable="detection_overlay",
        name="detection_overlay",
        parameters=[field_yaml],
        remappings=[
            ("image_raw", LaunchConfiguration("image_topic")),
        ],
        output="screen",
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        apriltag,
        field_localizer,
        overlay,
    ])
