from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    telemetry_host = LaunchConfiguration("telemetry_host")
    telemetry_port = LaunchConfiguration("telemetry_port")

    description_share = Path(get_package_share_directory("so101_description"))
    robot_description = (
        description_share / "urdf" / "so101_new_calib.urdf"
    ).read_text(encoding="utf-8")

    return LaunchDescription(
        [
            DeclareLaunchArgument("telemetry_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("telemetry_port", default_value="15001"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            Node(
                package="drone_handeye_calibration",
                executable="so101_telemetry_publisher",
                output="screen",
                parameters=[
                    {
                        "listen_host": telemetry_host,
                        "listen_port": telemetry_port,
                        "timeout_s": 0.5,
                        "poll_rate": 120.0,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": "follower/",
                    }
                ],
                remappings=[
                    ("joint_states", "/follower/joint_states"),
                ],
            ),
            Node(
                package="drone_handeye_calibration",
                executable="charuco_tf_publisher",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "marker_frame": "charuco_board",
                        "squares_x": 7,
                        "squares_y": 5,
                        "square_length_mm": 24.0,
                        "marker_length_mm": 19.0,
                        "min_corners": 18,
                        "max_reprojection_error_px": 1.5,
                    }
                ],
            ),
        ]
    )
