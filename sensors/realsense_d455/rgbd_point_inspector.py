#!/usr/bin/env python3
"""Inspect aligned RealSense RGB-D points in the color optical frame."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str


@dataclass(frozen=True)
class PlaneEstimate:
    normal: np.ndarray
    offset: float
    inlier_fraction: float
    rms_error_m: float


def deproject_pixels(
    u: np.ndarray,
    v: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: Intrinsics,
) -> np.ndarray:
    return np.column_stack(
        (
            (u - intrinsics.cx) * depth_m / intrinsics.fx,
            (v - intrinsics.cy) * depth_m / intrinsics.fy,
            depth_m,
        )
    )


def fit_plane_ransac(
    points: np.ndarray,
    threshold_m: float,
    iterations: int,
    seed: int = 7,
) -> PlaneEstimate | None:
    if points.shape[0] < 3:
        return None

    rng = np.random.default_rng(seed)
    best_inliers: np.ndarray | None = None
    best_count = 0

    for _ in range(iterations):
        sample = points[rng.choice(points.shape[0], size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        offset = -float(normal @ sample[0])
        inliers = np.abs(points @ normal + offset) <= threshold_m
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < 3:
        return None

    inlier_points = points[best_inliers]
    centroid = inlier_points.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    offset = -float(normal @ centroid)

    # Point toward the camera so points above the table have positive height.
    if offset < 0.0:
        normal = -normal
        offset = -offset

    residuals = inlier_points @ normal + offset
    return PlaneEstimate(
        normal=normal,
        offset=offset,
        inlier_fraction=best_count / points.shape[0],
        rms_error_m=float(np.sqrt(np.mean(residuals**2))),
    )


class RgbdPointInspector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("realsense_rgbd_point_inspector")
        self.window_name = "D455 aligned RGB-D point inspector"
        self.roi = tuple(args.roi)
        self.roi_reference_size = tuple(args.roi_reference_size)
        self.depth_window = args.depth_window
        self.min_depth_m = args.min_depth_m
        self.max_depth_m = args.max_depth_m
        self.auto_depth_contrast = args.depth_contrast == "auto"
        self.auto_depth_percentiles = tuple(args.auto_depth_percentiles)
        self.display_depth_range = (self.min_depth_m, self.max_depth_m)
        self.view = args.view
        self.min_height_m = args.min_height_m
        self.max_height_m = args.max_height_m
        self.plane_sample_step = args.plane_sample_step
        self.plane_threshold_m = args.plane_threshold_m
        self.plane_iterations = args.plane_iterations
        self.min_plane_inlier_fraction = args.min_plane_inlier_fraction
        self.plane_refresh_sec = args.plane_refresh_sec
        self.depth_valid_fraction = 0.0
        self.color: np.ndarray | None = None
        self.depth_m: np.ndarray | None = None
        self.intrinsics: Intrinsics | None = None
        self.plane: PlaneEstimate | None = None
        self.selected_pixel: tuple[int, int] | None = None
        self.selected_point: np.ndarray | None = None
        self.last_frame_time = 0.0
        self.last_plane_fit_time = 0.0
        self.depth_sequence = 0
        self.rendered_depth_sequence = -1

        self.create_subscription(Image, args.color_topic, self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse)
        self.get_logger().info(f"color: {args.color_topic}")
        self.get_logger().info(f"aligned depth: {args.depth_topic}")
        self.get_logger().info(f"intrinsics: {args.camera_info_topic}")

    @staticmethod
    def packed_rows(message: Image, bytes_per_pixel: int) -> np.ndarray:
        row_bytes = message.width * bytes_per_pixel
        if message.step < row_bytes:
            raise ValueError(f"Image step {message.step} is smaller than {row_bytes}")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        expected = message.height * message.step
        if raw.size < expected:
            raise ValueError(f"Image payload has {raw.size} bytes; expected {expected}")
        return raw[:expected].reshape(message.height, message.step)[:, :row_bytes]

    @classmethod
    def color_to_bgr(cls, message: Image) -> np.ndarray:
        encoding = message.encoding.lower()
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
        if channels is None:
            raise ValueError(f"Unsupported color encoding: {message.encoding}")
        image = cls.packed_rows(message, channels).reshape(
            message.height, message.width, channels
        )
        conversions = {
            "rgb8": cv2.COLOR_RGB2BGR,
            "rgba8": cv2.COLOR_RGBA2BGR,
            "bgra8": cv2.COLOR_BGRA2BGR,
        }
        conversion = conversions.get(encoding)
        return cv2.cvtColor(image, conversion) if conversion is not None else image.copy()

    @classmethod
    def depth_to_meters(cls, message: Image) -> np.ndarray:
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

    def on_color(self, message: Image) -> None:
        try:
            self.color = self.color_to_bgr(message)
            self.last_frame_time = time.monotonic()
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def on_depth(self, message: Image) -> None:
        try:
            self.depth_m = self.depth_to_meters(message)
            self.depth_sequence += 1
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def on_camera_info(self, message: CameraInfo) -> None:
        self.intrinsics = Intrinsics(
            width=message.width,
            height=message.height,
            fx=float(message.k[0]),
            fy=float(message.k[4]),
            cx=float(message.k[2]),
            cy=float(message.k[5]),
            frame_id=message.header.frame_id,
        )

    def depth_at(self, u: int, v: int) -> float | None:
        if self.depth_m is None:
            return None
        radius = self.depth_window // 2
        x0, x1 = max(0, u - radius), min(self.depth_m.shape[1], u + radius + 1)
        y0, y1 = max(0, v - radius), min(self.depth_m.shape[0], v + radius + 1)
        values = self.depth_m[y0:y1, x0:x1]
        valid = values[
            np.isfinite(values)
            & (values >= self.min_depth_m)
            & (values <= self.max_depth_m)
        ]
        return float(np.median(valid)) if valid.size else None

    def deproject(self, u: int, v: int) -> np.ndarray | None:
        intrinsics = self.intrinsics
        if intrinsics is None:
            return None
        depth = self.depth_at(u, v)
        if depth is None:
            return None
        return np.array(
            [
                (u - intrinsics.cx) * depth / intrinsics.fx,
                (v - intrinsics.cy) * depth / intrinsics.fy,
                depth,
            ],
            dtype=np.float64,
        )

    def selected_height(self) -> float | None:
        if self.selected_point is None or self.plane is None:
            return None
        return float(self.selected_point @ self.plane.normal + self.plane.offset)

    def toggle_view(self) -> None:
        self.view = "height" if self.view == "depth" else "depth"
        self.rendered_depth_sequence = -1

    def reset_plane(self) -> None:
        self.plane = None
        self.last_plane_fit_time = 0.0
        self.rendered_depth_sequence = -1

    def on_mouse(self, event: int, x: int, y: int, _flags: int, _data: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self.color is None or self.depth_m is None:
            return
        color_h, color_w = self.color.shape[:2]
        depth_h, depth_w = self.depth_m.shape
        x_min, y_min, x_max, y_max = self.clamped_roi(color_w, color_h)
        panel_width = x_max - x_min
        panel_x = x % panel_width
        color_u = panel_x + x_min
        color_v = y + y_min
        u = int(np.clip(round(color_u * depth_w / color_w), 0, depth_w - 1))
        v = int(np.clip(round(color_v * depth_h / color_h), 0, depth_h - 1))
        self.selected_pixel = (u, v)
        self.selected_point = self.deproject(u, v)
        if self.selected_point is None:
            self.get_logger().warning(f"No valid depth near pixel ({u}, {v})")
            return
        frame_id = self.intrinsics.frame_id if self.intrinsics else "camera frame"
        x_m, y_m, z_m = self.selected_point
        self.get_logger().info(
            f"pixel=({u}, {v}) {frame_id}: x={x_m:.4f} y={y_m:.4f} z={z_m:.4f} m"
        )

    def clamped_roi(self, width: int, height: int) -> tuple[int, int, int, int]:
        x_min, y_min, x_max, y_max = self.roi
        reference_width, reference_height = self.roi_reference_size
        scale_x = width / reference_width
        scale_y = height / reference_height
        x_min = round(x_min * scale_x)
        y_min = round(y_min * scale_y)
        x_max = round(x_max * scale_x)
        y_max = round(y_max * scale_y)
        x_min = int(np.clip(x_min, 0, width - 1))
        y_min = int(np.clip(y_min, 0, height - 1))
        x_max = int(np.clip(x_max, x_min + 1, width))
        y_max = int(np.clip(y_max, y_min + 1, height))
        return x_min, y_min, x_max, y_max

    def render(self) -> None:
        if (
            self.color is not None
            and self.depth_sequence == self.rendered_depth_sequence
        ):
            return
        self.rendered_depth_sequence = self.depth_sequence

        if self.color is None:
            x_min, y_min, x_max, y_max = self.roi
            panel = np.zeros((y_max - y_min, x_max - x_min, 3), dtype=np.uint8)
            cv2.putText(
                panel,
                "Waiting for D455 color frames...",
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            canvas = np.hstack((panel, panel.copy()))
        else:
            color_h, color_w = self.color.shape[:2]
            x_min, y_min, x_max, y_max = self.clamped_roi(color_w, color_h)
            color_panel = self.color[y_min:y_max, x_min:x_max].copy()
            depth_panel = self.render_geometry_panel(
                color_panel.shape[1], color_panel.shape[0]
            )
            canvas = np.hstack((color_panel, depth_panel))

        if self.selected_pixel is not None and self.depth_m is not None:
            full_color_h, full_color_w = self.color.shape[:2]
            depth_h, depth_w = self.depth_m.shape
            u, v = self.selected_pixel
            color_u = int(round(u * full_color_w / depth_w))
            color_v = int(round(v * full_color_h / depth_h))
            x = color_u - x_min
            y = color_v - y_min
            panel_width = x_max - x_min
            if 0 <= x < panel_width and 0 <= y < canvas.shape[0]:
                cv2.drawMarker(canvas, (x, y), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
                cv2.drawMarker(
                    canvas,
                    (x + panel_width, y),
                    (0, 255, 255),
                    cv2.MARKER_CROSS,
                    18,
                    2,
                )

        panel_width = canvas.shape[1] // 2
        cv2.putText(
            canvas,
            "RGB",
            (8, canvas.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if self.view == "height":
            geometry_label = (
                f"Height above table ({self.min_height_m * 1000:.0f} to "
                f"{self.max_height_m * 1000:.0f} mm)"
            )
        else:
            geometry_label = (
                f"Aligned depth ({self.display_depth_range[0]:.2f}-"
                f"{self.display_depth_range[1]:.2f} m, "
                f"valid {self.depth_valid_fraction:.0%})"
            )
        cv2.putText(
            canvas,
            geometry_label,
            (panel_width + 8, canvas.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.line(
            canvas,
            (panel_width, 0),
            (panel_width, canvas.shape[0] - 1),
            (255, 255, 255),
            1,
        )

        lines = ["m: depth/height | r: refit table | q/esc: quit"]
        lines.append(f"ROI: x=[{x_min},{x_max}) y=[{y_min},{y_max})")
        if self.intrinsics is not None:
            lines.append(f"frame: {self.intrinsics.frame_id}")
        if self.selected_point is not None:
            x_m, y_m, z_m = self.selected_point
            lines.append(f"camera XYZ: {x_m:+.4f}, {y_m:+.4f}, {z_m:+.4f} m")
        selected_height = self.selected_height()
        if selected_height is not None:
            lines.append(f"table height: {selected_height * 1000:+.1f} mm")
        if self.plane is not None:
            lines.append(
                f"plane: inliers {self.plane.inlier_fraction:.0%}, "
                f"RMS {self.plane.rms_error_m * 1000:.1f} mm"
            )
        elif self.view == "height":
            lines.append("plane: unavailable")
        if self.last_frame_time and time.monotonic() - self.last_frame_time > 1.0:
            lines.append("WARNING: color stream is stale")

        for index, line in enumerate(lines):
            y = 22 + 22 * index
            cv2.putText(
                canvas,
                line,
                (8, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                line,
                (8, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.imshow(self.window_name, canvas)

    def valid_depth_mask(self, depth: np.ndarray) -> np.ndarray:
        return (
            np.isfinite(depth)
            & (depth >= self.min_depth_m)
            & (depth <= self.max_depth_m)
        )

    def sampled_roi_points(
        self,
        depth_roi: np.ndarray,
        valid: np.ndarray,
        x_min: int,
        y_min: int,
    ) -> np.ndarray | None:
        if self.intrinsics is None:
            return None
        step = self.plane_sample_step
        sampled_depth = depth_roi[::step, ::step]
        sampled_valid = valid[::step, ::step]
        local_v, local_u = np.nonzero(sampled_valid)
        if local_u.size < 3:
            return None
        u = x_min + local_u * step
        v = y_min + local_v * step
        z = sampled_depth[local_v, local_u]
        return deproject_pixels(u, v, z, self.intrinsics)

    def update_plane(
        self,
        depth_roi: np.ndarray,
        valid: np.ndarray,
        x_min: int,
        y_min: int,
    ) -> None:
        now = time.monotonic()
        if self.plane is not None:
            if self.plane_refresh_sec <= 0.0:
                return
            if now - self.last_plane_fit_time < self.plane_refresh_sec:
                return

        self.last_plane_fit_time = now
        points = self.sampled_roi_points(depth_roi, valid, x_min, y_min)
        if points is None:
            return
        estimate = fit_plane_ransac(
            points,
            threshold_m=self.plane_threshold_m,
            iterations=self.plane_iterations,
        )
        if (
            estimate is not None
            and estimate.inlier_fraction >= self.min_plane_inlier_fraction
        ):
            self.plane = estimate
            nx, ny, nz = estimate.normal
            self.get_logger().info(
                f"table plane n=({nx:+.4f}, {ny:+.4f}, {nz:+.4f}) "
                f"d={estimate.offset:+.4f} m, "
                f"inliers={estimate.inlier_fraction:.1%}, "
                f"RMS={estimate.rms_error_m * 1000:.1f} mm"
            )

    def render_geometry_panel(self, width: int, height: int) -> np.ndarray:
        if self.depth_m is None:
            panel = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(
                panel,
                "Waiting for aligned depth...",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            return panel

        depth_h, depth_w = self.depth_m.shape
        x_min, y_min, x_max, y_max = self.clamped_roi(depth_w, depth_h)
        depth_roi = self.depth_m[y_min:y_max, x_min:x_max]
        valid = self.valid_depth_mask(depth_roi)
        self.depth_valid_fraction = float(np.count_nonzero(valid) / valid.size)
        if self.view == "height":
            self.update_plane(depth_roi, valid, x_min, y_min)
            panel = self.render_height_panel(depth_roi, valid, x_min, y_min)
        else:
            panel = self.render_raw_depth_panel(depth_roi, valid)
        if panel.shape[1] != width or panel.shape[0] != height:
            panel = cv2.resize(panel, (width, height), interpolation=cv2.INTER_NEAREST)
        return panel

    def render_raw_depth_panel(
        self,
        depth_roi: np.ndarray,
        valid: np.ndarray,
    ) -> np.ndarray:
        display_min = self.min_depth_m
        display_max = self.max_depth_m
        if self.auto_depth_contrast and np.any(valid):
            display_min, display_max = np.percentile(
                depth_roi[valid],
                self.auto_depth_percentiles,
            )
            if display_max - display_min < 0.05:
                center = (display_min + display_max) / 2.0
                display_min = center - 0.025
                display_max = center + 0.025
            display_min = max(self.min_depth_m, float(display_min))
            display_max = min(self.max_depth_m, float(display_max))
        self.display_depth_range = (display_min, display_max)

        normalized = np.zeros(depth_roi.shape, dtype=np.uint8)
        if np.any(valid):
            scale_range = max(display_max - display_min, 1e-6)
            scaled = (depth_roi[valid] - display_min) / scale_range
            normalized[valid] = np.clip(scaled * 255.0, 0.0, 255.0).astype(np.uint8)
        panel = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
        panel[~valid] = 0
        return panel

    def render_height_panel(
        self,
        depth_roi: np.ndarray,
        valid: np.ndarray,
        x_min: int,
        y_min: int,
    ) -> np.ndarray:
        panel = np.zeros((*depth_roi.shape, 3), dtype=np.uint8)
        if self.plane is None or self.intrinsics is None or not np.any(valid):
            return panel

        local_v, local_u = np.nonzero(valid)
        z = depth_roi[local_v, local_u]
        points = deproject_pixels(
            x_min + local_u,
            y_min + local_v,
            z,
            self.intrinsics,
        )
        heights = points @ self.plane.normal + self.plane.offset
        visible = (heights >= self.min_height_m) & (heights <= self.max_height_m)
        normalized = np.zeros(depth_roi.shape, dtype=np.uint8)
        height_range = self.max_height_m - self.min_height_m
        normalized_values = np.clip(
            (heights[visible] - self.min_height_m) / height_range,
            0.0,
            1.0,
        )
        normalized[local_v[visible], local_u[visible]] = (
            normalized_values * 255.0
        ).astype(np.uint8)
        panel = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        visible_mask = np.zeros(depth_roi.shape, dtype=bool)
        visible_mask[local_v[visible], local_u[visible]] = True
        panel[~visible_mask] = 0
        return panel

    def close(self) -> None:
        cv2.destroyWindow(self.window_name)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    parser.add_argument(
        "--depth-topic",
        default="/camera/camera/aligned_depth_to_color/image_raw",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/camera/camera/aligned_depth_to_color/camera_info",
    )
    parser.add_argument("--depth-window", type=int, default=5)
    parser.add_argument("--min-depth-m", type=float, default=0.15)
    parser.add_argument("--max-depth-m", type=float, default=3.0)
    parser.add_argument(
        "--view",
        choices=("depth", "height"),
        default="height",
        help="Initial right-panel visualization. Press m to toggle at runtime.",
    )
    parser.add_argument("--min-height-m", type=float, default=-0.01)
    parser.add_argument("--max-height-m", type=float, default=0.12)
    parser.add_argument("--plane-sample-step", type=int, default=6)
    parser.add_argument("--plane-threshold-m", type=float, default=0.008)
    parser.add_argument("--plane-iterations", type=int, default=80)
    parser.add_argument("--min-plane-inlier-fraction", type=float, default=0.25)
    parser.add_argument(
        "--plane-refresh-sec",
        type=float,
        default=0.0,
        help="Zero fits once; positive values periodically refit the table.",
    )
    parser.add_argument(
        "--depth-contrast",
        choices=("auto", "fixed"),
        default="auto",
        help="Auto uses ROI depth percentiles for display contrast only.",
    )
    parser.add_argument(
        "--auto-depth-percentiles",
        type=float,
        nargs=2,
        metavar=("LOW", "HIGH"),
        default=(2.0, 95.0),
    )
    parser.add_argument(
        "--roi",
        type=int,
        nargs=4,
        metavar=("X_MIN", "Y_MIN", "X_MAX", "Y_MAX"),
        default=(68, 66, 368, 238),
        help="Operating ROI in reference-image pixels; maximum coordinates are exclusive.",
    )
    parser.add_argument(
        "--roi-reference-size",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        default=(424, 240),
        help="Resolution in which --roi coordinates were measured.",
    )
    args = parser.parse_args()
    if args.depth_window < 1 or args.depth_window % 2 == 0:
        parser.error("--depth-window must be a positive odd number")
    if args.max_depth_m <= args.min_depth_m:
        parser.error("--max-depth-m must be greater than --min-depth-m")
    if args.max_height_m <= args.min_height_m:
        parser.error("--max-height-m must be greater than --min-height-m")
    if args.plane_sample_step < 1:
        parser.error("--plane-sample-step must be positive")
    if args.plane_threshold_m <= 0.0:
        parser.error("--plane-threshold-m must be positive")
    if args.plane_iterations < 1:
        parser.error("--plane-iterations must be positive")
    if not 0.0 < args.min_plane_inlier_fraction <= 1.0:
        parser.error("--min-plane-inlier-fraction must be in (0, 1]")
    if args.plane_refresh_sec < 0.0:
        parser.error("--plane-refresh-sec cannot be negative")
    low_percentile, high_percentile = args.auto_depth_percentiles
    if not 0 <= low_percentile < high_percentile <= 100:
        parser.error("Depth contrast percentiles must satisfy 0 <= LOW < HIGH <= 100")
    if args.roi[0] < 0 or args.roi[1] < 0:
        parser.error("ROI minimum coordinates cannot be negative")
    if args.roi[2] <= args.roi[0] or args.roi[3] <= args.roi[1]:
        parser.error("ROI maximum coordinates must exceed minimum coordinates")
    if args.roi_reference_size[0] <= 0 or args.roi_reference_size[1] <= 0:
        parser.error("ROI reference dimensions must be positive")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RgbdPointInspector(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.render()
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("m"):
                node.toggle_view()
            elif key == ord("r"):
                node.reset_plane()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
