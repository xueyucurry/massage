from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share_dir = Path(get_package_share_directory("rm_massage"))
    frames_config = LaunchConfiguration("frames_config")
    safety_config = LaunchConfiguration("safety_config")
    marker_config = LaunchConfiguration("marker_config")
    lines_config = LaunchConfiguration("lines_config")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    return LaunchDescription([
        DeclareLaunchArgument("frames_config", default_value=str(share_dir / "config" / "frames.yaml")),
        DeclareLaunchArgument("safety_config", default_value=str(share_dir / "config" / "safety.yaml")),
        DeclareLaunchArgument("marker_config", default_value=str(share_dir / "config" / "body_markers.yaml")),
        DeclareLaunchArgument("lines_config", default_value=str(share_dir / "config" / "massage_lines.yaml")),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        Node(
            package="rm_massage",
            executable="aruco_body_pose",
            name="aruco_body_pose",
            output="screen",
            arguments=[
                "--frames", frames_config,
                "--markers", marker_config,
                "--image-topic", image_topic,
                "--camera-info-topic", camera_info_topic,
            ],
        ),
        Node(
            package="rm_massage",
            executable="preview_line_rviz",
            name="preview_line_rviz",
            output="screen",
            arguments=[
                "--frames", frames_config,
                "--safety", safety_config,
                "--lines", lines_config,
            ],
        ),
    ])
