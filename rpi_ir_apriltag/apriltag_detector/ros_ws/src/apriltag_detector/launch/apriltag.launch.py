from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("apriltag_detector")
    apriltag_yaml = os.path.join(pkg_share, "config", "apriltag.yaml")

    rectify_ir = Node(
        package="image_proc",
        executable="rectify_node",
        name="ir_rectify",
        remappings=[
            ("image", "/ir/image"),
            ("camera_info", "/ir/camera_info"),
            ("image_rect", "/ir/image_rect"),
        ],
        output="screen",
    )

    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        parameters=[apriltag_yaml],
        remappings=[
            ("image_rect", "/ir/image_rect"),
            ("camera_info", "/ir/camera_info"),
        ],
        output="screen",
    )

    return LaunchDescription([
        rectify_ir,
        apriltag,
    ])
