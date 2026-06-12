#!/usr/bin/env python3
"""Live OpenCV viewer for LOTA TCP PLY point-cloud stream on port 9848."""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from lota_receiver import PLY_POINT_BYTES, recv_exact, save_binary_ply


PLY_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])


class OscPoseReader:
    def __init__(self, log_path: Path | None, source: str = "") -> None:
        self.log_path = log_path
        self.source = source
        self.position: np.ndarray | None = None
        self.rotation_xyzw: np.ndarray | None = None
        self.euler: list[float] | None = None
        self.mode: str | None = None
        self.fps: float | None = None
        self.last_time_unix_ms: int | None = None
        self.last_read_time = 0.0
        self.last_resolve_time = 0.0

    def refresh_auto_log(self) -> None:
        if self.source.lower() != "auto":
            return
        now = time.time()
        if now - self.last_resolve_time < 1.0:
            return
        self.last_resolve_time = now
        latest = find_latest_osc_log(Path(__file__).resolve().parent / "captures")
        if latest is not None and latest != self.log_path:
            print(f"Switched OSC pose log to {latest}")
            self.log_path = latest

    def update(self, min_interval: float = 0.10) -> None:
        self.refresh_auto_log()
        if self.log_path is None:
            return
        now = time.time()
        if now - self.last_read_time < min_interval:
            return
        self.last_read_time = now
        if not self.log_path.exists():
            return

        # The Windows logger appends JSONL. Reading the tail keeps this cheap
        # while still recovering the latest position/rotation triplet.
        with self.log_path.open("rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 256_000))
            raw = f.read().decode("utf-8", errors="ignore")

        for line in raw.splitlines():
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            address = record.get("address")
            values = record.get("values") or []
            if address == "/lota/camera/position" and len(values) >= 3:
                self.position = np.asarray(values[:3], dtype=np.float32)
                self.last_time_unix_ms = record.get("time_unix_ms", self.last_time_unix_ms)
            elif address == "/lota/camera/rotation" and len(values) >= 4:
                self.rotation_xyzw = np.asarray(values[:4], dtype=np.float32)
                self.last_time_unix_ms = record.get("time_unix_ms", self.last_time_unix_ms)
            elif address == "/lota/camera/euler" and len(values) >= 3:
                self.euler = values[:3]
                self.last_time_unix_ms = record.get("time_unix_ms", self.last_time_unix_ms)
            elif address == "/lota/fps" and values:
                self.fps = float(values[0])
            elif address == "/lota/mode" and values:
                self.mode = str(values[0])

    def has_pose(self) -> bool:
        return self.position is not None and self.rotation_xyzw is not None

    def file_age_s(self) -> float | None:
        if self.log_path is None or not self.log_path.exists():
            return None
        return max(0.0, time.time() - self.log_path.stat().st_mtime)


def find_latest_osc_log(captures_dir: Path) -> Path | None:
    candidates = sorted(
        captures_dir.glob("lota_osc_windows_*/osc_messages.jsonl"),
        key=lambda p: p.stat().st_mtime if p.exists() else 0,
        reverse=True,
    )
    return candidates[0] if candidates else None


def resolve_osc_log(value: str) -> Path | None:
    if value.lower() in {"", "none", "off", "false"}:
        return None
    if value.lower() == "auto":
        return find_latest_osc_log(Path(__file__).resolve().parent / "captures")
    return Path(value).expanduser()


def rotation_matrix_from_xyzw(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = quat.astype(np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def open_server(host: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    return server


def accept_connection(server: socket.socket) -> socket.socket:
    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Connected from {addr[0]}:{addr[1]}")
    return conn


def recv_cloud(conn: socket.socket, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    count_raw = recv_exact(conn, 4)
    point_count = struct.unpack("<I", count_raw)[0]
    if point_count > max_points:
        raise ValueError(f"Refusing huge point frame: {point_count} > {max_points}")
    raw = recv_exact(conn, point_count * PLY_POINT_BYTES)
    records = np.frombuffer(raw, dtype=PLY_DTYPE, count=point_count)
    return records["xyz"].copy(), records["rgb"].copy()


def make_capture_dir(base: str | None) -> Path | None:
    if not base:
        return None
    path = Path(base) / datetime.now().strftime("lota_ply_live_%Y%m%d_%H%M%S")
    path.mkdir(parents=True, exist_ok=True)
    print(f"Saving selected clouds to {path.resolve()}")
    return path


def robust_range(values: np.ndarray, pad_ratio: float = 0.08) -> tuple[float, float]:
    lo, hi = np.percentile(values, [1.0, 99.0])
    if abs(hi - lo) < 1e-6:
        lo -= 0.5
        hi += 0.5
    pad = (hi - lo) * pad_ratio
    return float(lo - pad), float(hi + pad)


def select_projection(points: np.ndarray, args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    if args.projection == "xy":
        return points[:, 0], points[:, 1], points[:, 2], "x/y"
    if args.projection == "xz":
        return points[:, 0], points[:, 2], points[:, 2], "x/z"
    if args.projection == "yz":
        return points[:, 1], points[:, 2], points[:, 2], "y/z"

    # Auto mode: use the two axes with the largest spread. Keep z as range filter.
    spreads = np.ptp(points, axis=0)
    axes = np.argsort(spreads)[-2:]
    axis_names = ["x", "y", "z"]
    a, b = int(axes[0]), int(axes[1])
    return points[:, a], points[:, b], points[:, 2], f"{axis_names[a]}/{axis_names[b]} auto"


def world_to_camera(points: np.ndarray, pose: OscPoseReader, args: argparse.Namespace) -> tuple[np.ndarray, str]:
    if args.points_frame == "camera":
        return points, "camera-local PLY"
    if args.points_frame == "world" and not pose.has_pose():
        return points, "world PLY, no OSC pose"
    if args.points_frame == "auto" and not pose.has_pose():
        return points, "camera-local PLY, no OSC pose"

    assert pose.position is not None
    assert pose.rotation_xyzw is not None
    local_to_world = rotation_matrix_from_xyzw(pose.rotation_xyzw)
    camera_points = (points.astype(np.float64) - pose.position.astype(np.float64)) @ local_to_world
    return camera_points.astype(np.float32), "world PLY + OSC pose"


def project_camera_view(points: np.ndarray, colors: np.ndarray, args: argparse.Namespace, pose: OscPoseReader) -> tuple[np.ndarray, dict]:
    image = np.zeros((args.height, args.width, 3), dtype=np.uint8)
    if len(points) == 0:
        return image, {"visible": 0, "projection": "camera"}

    camera_points, frame_name = world_to_camera(points, pose, args)
    x = camera_points[:, 0]
    y = camera_points[:, 1]
    z = camera_points[:, 2]
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    depth = -z if args.camera_forward == "neg-z" else z
    pos_z_count = int(np.count_nonzero(finite & (z > args.min_z) & (z < args.max_z)))
    neg_z_count = int(np.count_nonzero(finite & (-z > args.min_z) & (-z < args.max_z)))
    near = (depth > args.min_z) & (depth < args.max_z)
    mask = finite & near
    if not np.any(mask):
        stats = {
            "visible": 0,
            "projection": "camera",
            "frame_name": frame_name,
            "pose": pose.has_pose(),
            "osc_age_s": pose.file_age_s(),
            "pos_z_count": pos_z_count,
            "neg_z_count": neg_z_count,
            "x_range": robust_range(points[:, 0][np.isfinite(points[:, 0])]) if np.any(np.isfinite(points[:, 0])) else None,
            "y_range": robust_range(points[:, 1][np.isfinite(points[:, 1])]) if np.any(np.isfinite(points[:, 1])) else None,
            "z_range": robust_range(points[:, 2][np.isfinite(points[:, 2])]) if np.any(np.isfinite(points[:, 2])) else None,
        }
        return image, stats

    x = x[mask]
    y = y[mask]
    depth = depth[mask]
    c = colors[mask]
    nx = x / np.maximum(depth, 1e-6)
    ny = y / np.maximum(depth, 1e-6)

    if args.camera_fit:
        min_u, max_u = robust_range(nx)
        min_v, max_v = robust_range(ny)
        px = ((nx - min_u) / max(1e-6, max_u - min_u) * (args.width - 1)).astype(np.int32)
        py = ((max_v - ny) / max(1e-6, max_v - min_v) * (args.height - 1)).astype(np.int32)
        fit_range = (min_u, max_u, min_v, max_v)
    else:
        focal = 0.5 * args.width / math.tan(math.radians(args.camera_fov_deg) * 0.5)
        px = (nx * focal + args.width * 0.5).astype(np.int32)
        py = (args.height * 0.5 - ny * focal).astype(np.int32)
        fit_range = None
    inside = (px >= 0) & (px < args.width) & (py >= 0) & (py < args.height)
    if not np.any(inside):
        return image, {
            "visible": 0,
            "projection": "camera",
            "frame_name": frame_name,
            "pose": pose.has_pose(),
            "osc_age_s": pose.file_age_s(),
            "pos_z_count": pos_z_count,
            "neg_z_count": neg_z_count,
            "fit_range": fit_range,
            "x_range": robust_range(points[:, 0][np.isfinite(points[:, 0])]),
            "y_range": robust_range(points[:, 1][np.isfinite(points[:, 1])]),
            "z_range": robust_range(points[:, 2][np.isfinite(points[:, 2])]),
        }

    px = px[inside]
    py = py[inside]
    depth_inside = depth[inside]
    c = c[inside]

    # Draw far-to-near so nearer points win when multiple samples land on one pixel.
    order = np.argsort(depth_inside)[::-1]
    image[py[order], px[order]] = c[order][:, ::-1]
    if args.point_radius > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (args.point_radius, args.point_radius))
        image = cv2.dilate(image, kernel)

    return image, {
        "visible": int(np.count_nonzero(inside)),
        "projection": "camera",
        "frame_name": frame_name,
        "pose": pose.has_pose(),
        "osc_age_s": pose.file_age_s(),
        "pos_z_count": pos_z_count,
        "neg_z_count": neg_z_count,
        "fit_range": fit_range,
        "depth_range": robust_range(depth[np.isfinite(depth)]),
        "x_range": robust_range(points[:, 0][np.isfinite(points[:, 0])]),
        "y_range": robust_range(points[:, 1][np.isfinite(points[:, 1])]),
        "z_range": robust_range(points[:, 2][np.isfinite(points[:, 2])]),
    }


def project_axis_view(points: np.ndarray, colors: np.ndarray, args: argparse.Namespace) -> tuple[np.ndarray, dict]:
    image = np.zeros((args.height, args.width, 3), dtype=np.uint8)
    if len(points) == 0:
        return image, {"visible": 0}

    u, v, z_filter, projection_name = select_projection(points, args)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    near = (z_filter > args.min_z) & (z_filter < args.max_z)
    mask = finite & near
    if not np.any(mask):
        stats = {
            "visible": 0,
            "projection": projection_name,
            "x_range": robust_range(x[finite]) if np.any(finite) else None,
            "y_range": robust_range(y[finite]) if np.any(finite) else None,
            "z_range": robust_range(z[finite]) if np.any(finite) else None,
        }
        return image, stats

    u = u[mask]
    v = v[mask]
    c = colors[mask]

    if args.auto_fit:
        min_u, max_u = robust_range(u)
        min_v, max_v = robust_range(v)
    else:
        min_u, max_u = args.min_x, args.max_x
        min_v, max_v = args.min_y, args.max_y

    px = ((u - min_u) / max(1e-6, max_u - min_u) * (args.width - 1)).astype(np.int32)
    py = ((max_v - v) / max(1e-6, max_v - min_v) * (args.height - 1)).astype(np.int32)
    inside = (px >= 0) & (px < args.width) & (py >= 0) & (py < args.height)
    if not np.any(inside):
        return image, {
            "visible": 0,
            "projection": projection_name,
            "u_range": (min_u, max_u),
            "v_range": (min_v, max_v),
        }

    # LOTA docs describe RGB order; OpenCV display wants BGR.
    image[py[inside], px[inside]] = c[inside][:, ::-1]
    if args.point_radius > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (args.point_radius, args.point_radius))
        image = cv2.dilate(image, kernel)
    return image, {
        "visible": int(np.count_nonzero(inside)),
        "projection": projection_name,
        "frame_name": "fixed axes",
        "u_range": (min_u, max_u),
        "v_range": (min_v, max_v),
        "x_range": robust_range(x[finite]),
        "y_range": robust_range(y[finite]),
        "z_range": robust_range(z[finite]),
    }


def project_points(
    points: np.ndarray,
    colors: np.ndarray,
    args: argparse.Namespace,
    pose: OscPoseReader,
) -> tuple[np.ndarray, dict]:
    pose.update()
    args.osc_log_resolved = str(pose.log_path) if pose.log_path is not None else "none"
    if args.projection == "camera":
        return project_camera_view(points, colors, args, pose)
    return project_axis_view(points, colors, args)


def draw_overlay(
    image: np.ndarray,
    frame_idx: int,
    fps: float,
    point_count: int,
    args: argparse.Namespace,
    stats: dict,
) -> np.ndarray:
    out = image.copy()
    x_range = stats.get("x_range")
    y_range = stats.get("y_range")
    z_range = stats.get("z_range")
    depth_range = stats.get("depth_range")
    osc_age = stats.get("osc_age_s")
    pos_z_count = stats.get("pos_z_count")
    neg_z_count = stats.get("neg_z_count")
    osc_label = getattr(args, "osc_log_resolved", args.osc_log)
    if isinstance(osc_label, str) and len(osc_label) > 44:
        osc_label = "..." + osc_label[-41:]
    lines = [
        f"LOTA PLY cloud  {frame_idx}  {fps:5.1f} FPS  points {point_count}",
        f"view {stats.get('projection', args.projection)}  {stats.get('frame_name', '')}  visible {stats.get('visible', 0)}",
        f"pose {stats.get('pose', False)}  osc_age {osc_age:.1f}s  osc {osc_label}  mode {args.points_frame}  fwd {args.camera_forward}" if osc_age is not None else f"pose {stats.get('pose', False)}  osc none  mode {args.points_frame}  fwd {args.camera_forward}",
        f"x {x_range[0]:.2f}..{x_range[1]:.2f}  y {y_range[0]:.2f}..{y_range[1]:.2f}  z {z_range[0]:.2f}..{z_range[1]:.2f}" if x_range and y_range and z_range else "no finite points",
        f"depth {depth_range[0]:.2f}..{depth_range[1]:.2f}m  fit {args.camera_fit}  fov {args.camera_fov_deg:.0f}" if depth_range else "no projected depth",
        f"front +z {pos_z_count}  -z {neg_z_count}  q quit  c camera/world  f +/-z  a fit  v view  s save" if pos_z_count is not None else "q quit  c camera/world  f +/-z  a fit  v view  s save",
    ]
    y = 24
    for line in lines:
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
        y += 24
    return out


def save_cloud(
    capture_dir: Path,
    frame_idx: int,
    points: np.ndarray,
    colors: np.ndarray,
    view: np.ndarray,
    args: argparse.Namespace,
    pose: OscPoseReader,
    stats: dict,
) -> None:
    stem = f"cloud_{frame_idx:06d}"
    np.savez_compressed(capture_dir / f"{stem}.npz", points=points, colors=colors)
    save_binary_ply(points, colors, capture_dir / f"{stem}.ply")
    cv2.imwrite(str(capture_dir / f"{stem}_view.png"), view)
    (capture_dir / f"{stem}_meta.json").write_text(
        json.dumps(
            {
                "time": time.time(),
                "points": int(len(points)),
                "points_frame": args.points_frame,
                "camera_forward": args.camera_forward,
                "camera_fit": args.camera_fit,
                "camera_fov_deg": args.camera_fov_deg,
                "min_z": args.min_z,
                "max_z": args.max_z,
                "osc_log": getattr(args, "osc_log_resolved", args.osc_log),
                "osc_age_s": pose.file_age_s(),
                "osc_position_w": pose.position.tolist() if pose.position is not None else None,
                "osc_rotation_xyzw_w_c": pose.rotation_xyzw.tolist() if pose.rotation_xyzw is not None else None,
                "osc_euler": pose.euler,
                "projection_stats": {
                    key: value
                    for key, value in stats.items()
                    if key in {"visible", "pos_z_count", "neg_z_count", "depth_range", "x_range", "y_range", "z_range"}
                },
            },
            indent=2,
        )
        + "\n"
    )
    print(f"Saved {stem}")


def synthetic_cloud(t: float) -> tuple[np.ndarray, np.ndarray]:
    xs = np.linspace(-0.35, 0.35, 180)
    zs = np.linspace(0.35, 1.4, 140)
    xx, zz = np.meshgrid(xs, zs)
    yy = 0.08 * np.sin(8 * xx + t) + 0.02 * np.cos(9 * zz)
    points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()]).astype("<f4")
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = np.clip(255 * (xx.ravel() - xs.min()) / (xs.max() - xs.min()), 0, 255).astype(np.uint8)
    colors[:, 1] = np.clip(255 * (1.0 - (zz.ravel() - zs.min()) / (zs.max() - zs.min())), 0, 255).astype(np.uint8)
    colors[:, 2] = 180
    return points, colors


def show_loop(args: argparse.Namespace, server: socket.socket | None) -> None:
    capture_dir = make_capture_dir(args.save_dir)
    pose = OscPoseReader(resolve_osc_log(args.osc_log), args.osc_log)
    args.osc_log_resolved = str(pose.log_path) if pose.log_path is not None else "none"
    if pose.log_path is not None:
        print(f"Reading OSC pose from {pose.log_path}")
    elif args.projection == "camera":
        print("No OSC pose log found; treating PLY points as camera-local.")
    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window_name, args.width, args.height)
    frame_idx = 0
    last_t = time.time()
    fps = 0.0
    conn: socket.socket | None = None
    try:
        while args.max_frames <= 0 or frame_idx < args.max_frames:
            if args.demo:
                points, colors = synthetic_cloud(time.time())
                time.sleep(1.0 / max(1.0, args.demo_fps))
            else:
                assert server is not None
                if conn is None:
                    print("Waiting for LOTA PLY point-cloud connection...")
                    conn = accept_connection(server)
                try:
                    points, colors = recv_cloud(conn, args.max_points)
                except EOFError:
                    print("Peer disconnected before a complete point-cloud frame")
                    conn.close()
                    conn = None
                    continue

            frame_idx += 1
            now = time.time()
            dt = now - last_t
            last_t = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else 1.0 / dt

            view, stats = project_points(points, colors, args, pose)
            view = draw_overlay(view, frame_idx, fps, len(points), args, stats)
            cv2.imshow(args.window_name, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                args.points_frame = "camera" if args.points_frame != "camera" else "world"
                print(f"points_frame={args.points_frame}")
            if key == ord("f"):
                args.camera_forward = "pos-z" if args.camera_forward == "neg-z" else "neg-z"
                print(f"camera_forward={args.camera_forward}")
            if key == ord("a"):
                args.camera_fit = not args.camera_fit
                print(f"camera_fit={args.camera_fit}")
            if key == ord("v"):
                views = ["camera", "auto", "xy", "xz", "yz"]
                args.projection = views[(views.index(args.projection) + 1) % len(views)]
                print(f"projection={args.projection}")
            if key == ord("s"):
                if capture_dir is None:
                    capture_dir = make_capture_dir("captures")
                save_cloud(capture_dir, frame_idx, points, colors, view, args, pose, stats)
    finally:
        if conn is not None:
            conn.close()
        if server is not None:
            server.close()
        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9848)
    parser.add_argument("--width", type=int, default=900)
    parser.add_argument("--height", type=int, default=700)
    parser.add_argument("--min-x", type=float, default=-0.75)
    parser.add_argument("--max-x", type=float, default=0.75)
    parser.add_argument("--min-y", type=float, default=-0.75)
    parser.add_argument("--max-y", type=float, default=0.75)
    parser.add_argument("--min-z", type=float, default=0.20)
    parser.add_argument("--max-z", type=float, default=2.50)
    parser.add_argument("--projection", choices=["camera", "auto", "xy", "xz", "yz"], default="camera")
    parser.add_argument("--auto-fit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--osc-log", default="auto", help="OSC JSONL pose log path. Use auto, none, or an explicit path.")
    parser.add_argument("--points-frame", choices=["auto", "world", "camera"], default="camera")
    parser.add_argument("--camera-forward", choices=["neg-z", "pos-z"], default="neg-z")
    parser.add_argument("--camera-fov-deg", type=float, default=70.0)
    parser.add_argument("--camera-fit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--point-radius", type=int, default=2)
    parser.add_argument("--max-points", type=int, default=1_000_000)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--save-dir", default=None)
    parser.add_argument("--window-name", default="LOTA iPhone point cloud")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--demo-fps", type=float, default=30.0)
    args = parser.parse_args()

    if args.demo:
        show_loop(args, None)
        return

    server = open_server(args.host, args.port)
    print(f"Listening for LOTA PLY point cloud on {args.host}:{args.port}")
    show_loop(args, server)


if __name__ == "__main__":
    main()
