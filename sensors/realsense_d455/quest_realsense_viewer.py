#!/usr/bin/env python3
"""Stream RealSense ROS2 color and aligned-depth frames to a Quest WebXR page."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PYBULLET_DIR = PROJECT_ROOT / "sim" / "pybullet"
if str(PYBULLET_DIR) not in sys.path:
    sys.path.insert(0, str(PYBULLET_DIR))

from quest_webxr_server import (  # noqa: E402
    REALSENSE_HTML,
    SensorFrameState,
    TeleopState,
    serve_https,
)


class RealSenseQuestBridge(Node):
    def __init__(
        self,
        frame_state: SensorFrameState,
        color_topic: str,
        depth_topic: str,
        jpeg_quality: int,
        min_depth_m: float,
        max_depth_m: float,
    ) -> None:
        super().__init__("quest_realsense_viewer")
        self.frame_state = frame_state
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.color_frames = 0
        self.depth_frames = 0
        self.last_report_time = time.monotonic()
        self.create_subscription(Image, color_topic, self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_timer(5.0, self.report_rates)
        self.get_logger().info(f"color: subscribing to {color_topic}")
        self.get_logger().info(f"depth: subscribing to {depth_topic}")

    def encode_jpeg(self, image: np.ndarray) -> bytes | None:
        ok, encoded = cv2.imencode(".jpg", image, self.jpeg_params)
        if not ok:
            self.get_logger().warning("JPEG encoding failed")
            return None
        return encoded.tobytes()

    def on_color(self, message: Image) -> None:
        try:
            image = self.color_to_bgr(message)
            jpeg = self.encode_jpeg(image)
        except Exception as exc:
            self.get_logger().warning(f"Color conversion failed: {exc}")
            return
        if jpeg is None:
            return
        height, width = image.shape[:2]
        self.frame_state.update_color(jpeg, width, height)
        self.color_frames += 1

    def on_depth(self, message: Image) -> None:
        try:
            depth_m = self.depth_in_meters(message)
        except Exception as exc:
            self.get_logger().warning(f"Depth conversion failed: {exc}")
            return

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        normalized = np.zeros(depth_m.shape, dtype=np.uint8)
        if np.any(valid):
            scaled = (depth_m[valid] - self.min_depth_m) / (self.max_depth_m - self.min_depth_m)
            normalized[valid] = np.clip(scaled * 255.0, 0.0, 255.0).astype(np.uint8)
        colorized = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
        colorized[~valid] = 0
        jpeg = self.encode_jpeg(colorized)
        if jpeg is None:
            return
        height, width = depth_m.shape[:2]
        self.frame_state.update_depth(jpeg, width, height)
        self.depth_frames += 1

    @staticmethod
    def packed_rows(message: Image, bytes_per_pixel: int) -> np.ndarray:
        required_step = message.width * bytes_per_pixel
        if message.step < required_step:
            raise ValueError(
                f"Image step {message.step} is smaller than packed row {required_step}"
            )
        raw = np.frombuffer(message.data, dtype=np.uint8)
        expected = message.height * message.step
        if raw.size < expected:
            raise ValueError(f"Image payload has {raw.size} bytes; expected at least {expected}")
        return raw[:expected].reshape(message.height, message.step)[:, :required_step]

    @classmethod
    def color_to_bgr(cls, message: Image) -> np.ndarray:
        encoding = message.encoding.lower()
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            raise ValueError(f"Unsupported color encoding: {message.encoding}")
        packed = cls.packed_rows(message, channels)
        image = packed.reshape(message.height, message.width, channels)
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding == "mono8":
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image

    @classmethod
    def depth_in_meters(cls, message: Image) -> np.ndarray:
        encoding = message.encoding.upper()
        if encoding in ("16UC1", "MONO16"):
            dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
            scale = 0.001
        elif encoding == "32FC1":
            dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
            scale = 1.0
        else:
            raise ValueError(f"Unsupported depth encoding: {message.encoding}")

        packed = cls.packed_rows(message, dtype.itemsize)
        depth = packed.copy().view(dtype).reshape(message.height, message.width)
        return depth.astype(np.float32) * scale

    def report_rates(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last_report_time
        if elapsed < 5.0:
            return
        self.get_logger().info(
            f"bridge rates: color={self.color_frames / elapsed:.1f} FPS "
            f"depth={self.depth_frames / elapsed:.1f} FPS"
        )
        self.color_frames = 0
        self.depth_frames = 0
        self.last_report_time = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--advertise-ip", default="", help="Windows Wi-Fi IP shown in the Quest URL.")
    parser.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--min-depth-m", type=float, default=0.2)
    parser.add_argument("--max-depth-m", type=float, default=2.5)
    parser.add_argument("--run-seconds", type=float, default=0.0, help="0 means run until Ctrl+C.")
    args = parser.parse_args()
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")
    if args.min_depth_m < 0 or args.max_depth_m <= args.min_depth_m:
        parser.error("--max-depth-m must be greater than --min-depth-m")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    frame_state = SensorFrameState()
    node = RealSenseQuestBridge(
        frame_state=frame_state,
        color_topic=args.color_topic,
        depth_topic=args.depth_topic,
        jpeg_quality=args.jpeg_quality,
        min_depth_m=args.min_depth_m,
        max_depth_m=args.max_depth_m,
    )
    server = serve_https(
        args.host,
        args.port,
        args.advertise_ip,
        TeleopState(),
        sensor_state=frame_state,
        root_html=REALSENSE_HTML,
    )
    started = time.monotonic()
    try:
        while args.run_seconds <= 0 or time.monotonic() - started < args.run_seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
