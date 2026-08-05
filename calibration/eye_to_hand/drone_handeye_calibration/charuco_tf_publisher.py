#!/usr/bin/env python3
"""Publish the camera-to-ChArUco transform required by easy_handeye2."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from .charuco import (
    CameraModel,
    decode_color,
    detect_board,
    make_board,
    rotation_matrix_to_quaternion,
)


class CharucoTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("charuco_tf_publisher")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("marker_frame", "charuco_board")
        self.declare_parameter("squares_x", 7)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length_mm", 24.0)
        self.declare_parameter("marker_length_mm", 19.0)
        self.declare_parameter("min_corners", 18)
        self.declare_parameter("max_reprojection_error_px", 1.5)

        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.min_corners = int(self.get_parameter("min_corners").value)
        self.max_reprojection_error_px = float(
            self.get_parameter("max_reprojection_error_px").value
        )
        self.board, self.detector = make_board(
            int(self.get_parameter("squares_x").value),
            int(self.get_parameter("squares_y").value),
            float(self.get_parameter("square_length_mm").value) / 1000.0,
            float(self.get_parameter("marker_length_mm").value) / 1000.0,
        )
        self.camera: CameraModel | None = None
        self.accepted = 0
        self.rejected = 0
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            qos_profile_sensor_data,
        )

    def camera_info_callback(self, message: CameraInfo) -> None:
        self.camera = CameraModel(
            matrix=np.asarray(message.k, dtype=np.float64).reshape(3, 3),
            distortion=np.asarray(message.d, dtype=np.float64),
            width=message.width,
            height=message.height,
            frame_id=message.header.frame_id,
        )

    def image_callback(self, message: Image) -> None:
        if self.camera is None:
            return
        image = decode_color(message)
        if image.shape[:2] != (self.camera.height, self.camera.width):
            self.get_logger().error(
                "Image dimensions do not match CameraInfo",
                throttle_duration_sec=2.0,
            )
            return
        detection = detect_board(image, self.board, self.detector, self.camera)
        error = detection.reprojection_rms_px
        valid = (
            detection.rvec is not None
            and detection.tvec is not None
            and detection.corner_count >= self.min_corners
            and error is not None
            and error <= self.max_reprojection_error_px
        )
        if not valid:
            self.rejected += 1
            if self.rejected == 1 or self.rejected % 30 == 0:
                error_text = "none" if error is None else f"{error:.2f}"
                self.get_logger().warning(
                    f"Board pose rejected: corners={detection.corner_count}, "
                    f"reprojection={error_text}px"
                )
            return

        rotation, _ = cv2.Rodrigues(detection.rvec)
        quaternion = rotation_matrix_to_quaternion(rotation)
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.camera.frame_id
        transform.child_frame_id = self.marker_frame
        transform.transform.translation.x = float(detection.tvec[0])
        transform.transform.translation.y = float(detection.tvec[1])
        transform.transform.translation.z = float(detection.tvec[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.broadcaster.sendTransform(transform)
        self.accepted += 1
        if self.accepted == 1 or self.accepted % 30 == 0:
            self.get_logger().info(
                f"Publishing {self.camera.frame_id} -> {self.marker_frame}: "
                f"corners={detection.corner_count}, reprojection={error:.2f}px"
            )


def main() -> None:
    rclpy.init()
    node = CharucoTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
