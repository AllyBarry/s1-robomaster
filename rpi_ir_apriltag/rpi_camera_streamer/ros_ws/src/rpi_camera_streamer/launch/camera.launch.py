from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
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

    return LaunchDescription([
        camera_container,
    ])
