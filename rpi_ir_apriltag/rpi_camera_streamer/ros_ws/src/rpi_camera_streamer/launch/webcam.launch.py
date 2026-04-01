from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value="/dev/video0",
        description="V4L2 video device path",
    )

    image_width_arg = DeclareLaunchArgument(
        "image_width", default_value="640", description="Image width"
    )

    image_height_arg = DeclareLaunchArgument(
        "image_height", default_value="480", description="Image height"
    )

    webcam_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="webcam",
        parameters=[{
            "video_device": LaunchConfiguration("video_device"),
            "image_size": [
                LaunchConfiguration("image_width"),
                LaunchConfiguration("image_height"),
            ],
            "pixel_format": "YUYV",
            "camera_frame_id": "webcam_link",
        }],
        remappings=[
            ("image_raw", "/webcam/image_raw"),
            ("camera_info", "/webcam/camera_info"),
        ],
        output="screen",
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        webcam_node,
    ])
