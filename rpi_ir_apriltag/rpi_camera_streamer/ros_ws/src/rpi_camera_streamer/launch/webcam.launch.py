from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    video_device = LaunchConfiguration("video_device").perform(context)
    image_width = int(LaunchConfiguration("image_width").perform(context))
    image_height = int(LaunchConfiguration("image_height").perform(context))

    webcam_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="webcam",
        parameters=[{
            "video_device": video_device,
            "image_size": [image_width, image_height],
            "pixel_format": "YUYV",
            "io_method": "read",
            "camera_frame_id": "webcam_link",
        }],
        remappings=[
            ("image_raw", "/webcam/image_raw"),
            ("camera_info", "/webcam/camera_info"),
        ],
        output="screen",
    )

    return [webcam_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "video_device",
            default_value="/dev/video0",
            description="V4L2 video device path",
        ),
        DeclareLaunchArgument(
            "image_width",
            default_value="640",
            description="Image width",
        ),
        DeclareLaunchArgument(
            "image_height",
            default_value="480",
            description="Image height",
        ),
        OpaqueFunction(function=launch_setup),
    ])
