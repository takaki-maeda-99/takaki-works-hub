import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    device_id = DeclareLaunchArgument(
        "device_id", default_value=TextSubstitution(text="/dev/video0")
    )
    config_file_path = DeclareLaunchArgument(
        "config_file_path",
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("ball_camera"), "config", "params.yaml"
            )
        ),
    )

    webcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[{"video_device": LaunchConfiguration("device_id")}],
    )

    detector = Node(
        package="ball_camera",
        executable="detector",
        parameters=[LaunchConfiguration("config_file_path")],
    )

    return LaunchDescription(
        [
            device_id,
            config_file_path,
            webcam,
            detector,
        ]
    )
