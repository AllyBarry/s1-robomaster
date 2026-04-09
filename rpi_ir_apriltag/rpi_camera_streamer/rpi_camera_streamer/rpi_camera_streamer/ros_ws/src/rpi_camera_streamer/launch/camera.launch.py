from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("rpi_camera_streamer")
    calibration_file = os.path.join(pkg_share, "config", "cam_calibration.yaml")

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
                    "rgb_frame_id": "",
                    "enable_color": False,
                    "enable_ir": True,
                    "enable_depth": False,
                }],
            ),
        ],
    )

    camera_info_publisher = Node(
        package="rpi_camera_streamer",
        executable="camera_info_publisher",
        name="camera_info_publisher",
        parameters=[{"calibration_file": calibration_file}],
        output="screen",
    )

    return LaunchDescription([
        camera_container,
        camera_info_publisher,
    ])
