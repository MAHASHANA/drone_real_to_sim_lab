#!/usr/bin/env python3
"""Check ChArUco detection and pose stability on a live ROS2 color stream."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from drone_handeye_calibration.charuco import (
    CameraModel,
    Detection,
    decode_color,
    detect_board,
    make_board,
)


def rotation_difference_deg(first: np.ndarray, second: np.ndarray) -> float:
    first_matrix, _ = cv2.Rodrigues(first)
    second_matrix, _ = cv2.Rodrigues(second)
    relative = second_matrix @ first_matrix.T
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


class CharucoCheckNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("charuco_live_check")
        self.args = args
        self.board, self.detector = make_board(
            args.squares_x,
            args.squares_y,
            args.square_length_mm / 1000.0,
            args.marker_length_mm / 1000.0,
        )
        self.camera: CameraModel | None = None
        self.frame_count = 0
        self.detections: list[Detection] = []
        self.best: Detection | None = None
        self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            args.image_topic,
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
        if self.frame_count >= self.args.frames or self.camera is None:
            return
        image = decode_color(message)
        if (
            image.shape[1] != self.camera.width
            or image.shape[0] != self.camera.height
        ):
            raise RuntimeError(
                "Color image and CameraInfo dimensions differ: "
                f"image={image.shape[1]}x{image.shape[0]}, "
                f"info={self.camera.width}x{self.camera.height}"
            )
        detection = detect_board(image, self.board, self.detector, self.camera)
        self.frame_count += 1
        self.detections.append(detection)
        best_score = (
            detection.corner_count,
            -(
                detection.reprojection_rms_px
                if detection.reprojection_rms_px is not None
                else float("inf")
            ),
            detection.coverage,
        )
        if self.best is None:
            self.best = detection
        else:
            current_score = (
                self.best.corner_count,
                -(
                    self.best.reprojection_rms_px
                    if self.best.reprojection_rms_px is not None
                    else float("inf")
                ),
                self.best.coverage,
            )
            if best_score > current_score:
                self.best = detection
        if self.frame_count == 1 or self.frame_count % 10 == 0:
            print(
                f"frames={self.frame_count}/{self.args.frames} "
                f"markers={len(detection.marker_ids)} "
                f"corners={detection.corner_count}"
            )

    @property
    def complete(self) -> bool:
        return self.frame_count >= self.args.frames


def build_summary(node: CharucoCheckNode, args: argparse.Namespace) -> dict:
    detections = node.detections
    detected = [item for item in detections if item.corner_count > 0]
    posed = [item for item in detections if item.tvec is not None]
    corner_counts = [item.corner_count for item in detections]
    coverages = [item.coverage for item in posed]
    errors = [
        item.reprojection_rms_px
        for item in posed
        if item.reprojection_rms_px is not None
    ]
    translation_std_mm: list[float] | None = None
    rotation_jitter_deg: list[float] | None = None
    if posed:
        translations = np.asarray([item.tvec for item in posed])
        translation_std_mm = (translations.std(axis=0) * 1000.0).tolist()
        reference = posed[0].rvec
        rotation_jitter_deg = [
            rotation_difference_deg(reference, item.rvec) for item in posed
        ]

    detection_rate = len(detected) / len(detections) if detections else 0.0
    median_corners = float(np.median(corner_counts)) if corner_counts else 0.0
    median_coverage = float(np.median(coverages)) if coverages else 0.0
    median_error = float(np.median(errors)) if errors else None
    translation_jitter_norm_mm = (
        float(np.linalg.norm(translation_std_mm))
        if translation_std_mm is not None
        else None
    )
    median_rotation_jitter_deg = (
        float(np.median(rotation_jitter_deg))
        if rotation_jitter_deg
        else None
    )
    checks = {
        "detection_rate_at_least_80_percent": detection_rate >= 0.8,
        "median_corners_at_least_18_of_24": median_corners >= 18,
        "median_internal_corner_coverage_at_least_0_5_percent": (
            median_coverage >= 0.005
        ),
        "median_reprojection_rms_at_most_1_5_px": (
            median_error is not None and median_error <= 1.5
        ),
        "translation_jitter_at_most_3_mm": (
            translation_jitter_norm_mm is not None
            and translation_jitter_norm_mm <= 3.0
        ),
    }
    return {
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_mm": args.square_length_mm,
            "marker_length_mm": args.marker_length_mm,
            "dictionary": "DICT_6X6_250",
            "start_id": 0,
            "maximum_charuco_corners": (args.squares_x - 1)
            * (args.squares_y - 1),
        },
        "camera": {
            "frame_id": node.camera.frame_id if node.camera else None,
            "width": node.camera.width if node.camera else None,
            "height": node.camera.height if node.camera else None,
            "matrix": node.camera.matrix.tolist() if node.camera else None,
            "distortion": node.camera.distortion.tolist() if node.camera else None,
        },
        "frames_processed": len(detections),
        "frames_with_detection": len(detected),
        "frames_with_pose": len(posed),
        "detection_rate": detection_rate,
        "median_corners": median_corners,
        "maximum_corners": max(corner_counts, default=0),
        "median_coverage": median_coverage,
        "median_reprojection_rms_px": median_error,
        "translation_std_mm_xyz": translation_std_mm,
        "translation_jitter_norm_mm": translation_jitter_norm_mm,
        "median_rotation_jitter_deg": median_rotation_jitter_deg,
        "best_marker_ids": node.best.marker_ids if node.best else [],
        "best_tvec_camera_m": (
            node.best.tvec.tolist()
            if node.best is not None and node.best.tvec is not None
            else None
        ),
        "best_rvec_camera_rad": (
            node.best.rvec.tolist()
            if node.best is not None and node.best.rvec is not None
            else None
        ),
        "checks": checks,
        "result": "PASS" if all(checks.values()) else "CHECK_SETUP",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image-topic",
        default="/camera/camera/color/image_raw",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/camera/camera/color/camera_info",
    )
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-length-mm", type=float, default=24.0)
    parser.add_argument("--marker-length-mm", type=float, default=19.0)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.squares_x < 3 or args.squares_y < 3:
        parser.error("board must contain at least 3x3 squares")
    if not 0 < args.marker_length_mm < args.square_length_mm:
        parser.error("marker length must be positive and smaller than square length")
    if args.frames < 1 or args.timeout_sec <= 0:
        parser.error("frames and timeout must be positive")
    return args


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir or (
        Path("captures")
        / f"charuco_check_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    rclpy.init()
    node = CharucoCheckNode(args)
    start = time.monotonic()
    try:
        while not node.complete and time.monotonic() - start < args.timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
        summary = build_summary(node, args)
        output_dir.mkdir(parents=True, exist_ok=True)
        summary_path = output_dir / "summary.json"
        summary_path.write_text(
            json.dumps(summary, indent=2) + "\n",
            encoding="utf-8",
        )
        if node.best is not None:
            cv2.imwrite(str(output_dir / "annotated.png"), node.best.annotated)
        print(json.dumps(summary, indent=2))
        print(f"Saved check to {output_dir.resolve()}")
        if summary["result"] != "PASS":
            raise SystemExit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
