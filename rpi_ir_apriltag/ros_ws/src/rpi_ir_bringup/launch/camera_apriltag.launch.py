from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("rpi_ir_bringup")
    apriltag_yaml = os.path.join(pkg_share, "config", "apriltag.yaml")

    camera_container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="openni2_camera",
                plugin="openni2_wrapper::OpenNI2Driver",
                name="openni2_camera",
                parameters=[{
                "depth_registration": False,
                "color_depth_synchronization": False,
                "use_device_time": True,

                # Prefer IR-only operation
                "rgb_frame_id": "",
                "enable_color": False,
                "enable_ir": True,
                "enable_depth": False,
            }],
            ),
        ],
    )

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

    # apriltag = Node(
    #     package="apriltag_ros",
    #     executable="apriltag_node",
    #     name="apriltag",
    #     parameters=[apriltag_yaml],
    #     remappings=[
    #         ("image_rect", "/ir/image_rect"),
    #         ("camera_info", "/ir/camera_info"),
    #     ],
    #     output="screen",
    # )

    return LaunchDescription([
        camera_container,
        # rectify_ir,
        # apriltag,
    ])