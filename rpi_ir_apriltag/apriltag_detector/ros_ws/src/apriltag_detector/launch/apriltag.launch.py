from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("apriltag_detector")
    apriltag_robots_yaml = os.path.join(pkg_share, "config", "apriltag.yaml")
    apriltag_field_yaml = os.path.join(pkg_share, "config", "apriltag_field.yaml")
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

    # 36h11 detector — robots
    apriltag_robots = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_robots",
        parameters=[apriltag_robots_yaml],
        remappings=[
            ("image_rect", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
            ("detections", "/detections/robots"),
        ],
        output="screen",
    )

    # 16h5 detector — field corners
    apriltag_field = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_field",
        parameters=[apriltag_field_yaml],
        remappings=[
            ("image_rect", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
            ("detections", "/detections/field"),
        ],
        output="screen",
    )

    # Field localizer — homography from corners, publishes robot field coords
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
        remappings=[
            ("image_raw", LaunchConfiguration("image_topic")),
        ],
        output="screen",
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        apriltag_robots,
        apriltag_field,
        field_localizer,
        overlay,
    ])
