from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("drone_handeye_calibration")
    )
    easy_handeye_share = Path(get_package_share_directory("easy_handeye2"))

    telemetry_host = LaunchConfiguration("telemetry_host")
    telemetry_port = LaunchConfiguration("telemetry_port")

    capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "capture.launch.py")
        ),
        launch_arguments={
            "telemetry_host": telemetry_host,
            "telemetry_port": telemetry_port,
        }.items(),
    )
    solver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(easy_handeye_share / "launch" / "calibrate.launch.py")
        ),
        launch_arguments={
            "name": "d455_so101_eye_to_hand",
            "calibration_type": "eye_on_base",
            "robot_base_frame": "follower/base_link",
            "robot_effector_frame": "follower/gripper_frame_link",
            "tracking_base_frame": "camera_color_optical_frame",
            "tracking_marker_frame": "charuco_board",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("telemetry_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("telemetry_port", default_value="15001"),
            capture,
            solver,
        ]
    )
