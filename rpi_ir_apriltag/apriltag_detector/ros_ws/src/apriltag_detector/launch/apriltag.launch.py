from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("apriltag_detector")
    apriltag_yaml = os.path.join(pkg_share, "config", "apriltag.yaml")

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

    # rectify = Node(
    #     package="image_proc",
    #     executable="rectify_node",
    #     name="rectify",
    #     remappings=[
    #         ("image", LaunchConfiguration("image_topic")),
    #         ("camera_info", LaunchConfiguration("camera_info_topic")),
    #         ("image_rect", "/camera/image_rect"),
    #     ],
    #     output="screen",
    # )

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
        # rectify,
        apriltag,
        overlay,
    ])
