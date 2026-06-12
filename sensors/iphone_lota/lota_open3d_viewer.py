#!/usr/bin/env python3
"""Open3D viewer for LOTA PLY point clouds.

This shows the point cloud directly in 3D. It can either listen to the live
LOTA TCP PLY stream, or inspect a saved .npz cloud from lota_ply_live_viewer.py.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import time
from pathlib import Path

import numpy as np
import open3d as o3d

from lota_receiver import PLY_POINT_BYTES, recv_exact, save_binary_ply


PLY_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])


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


def load_npz(path: Path) -> tuple[np.ndarray, np.ndarray, dict]:
    data = np.load(path)
    meta_path = path.with_name(path.stem + "_meta.json")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    return data["points"].copy(), data["colors"].copy(), meta


def rotation_matrix_from_xyzw(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = quat.astype(np.float64)
    norm = np.linalg.norm([x, y, z, w])
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


def transform_saved_cloud_to_world(points: np.ndarray, meta: dict) -> np.ndarray:
    position = meta.get("osc_position_w")
    quat = meta.get("osc_rotation_xyzw_w_c")
    if position is None or quat is None:
        raise ValueError("Saved cloud metadata does not contain OSC world pose")
    camera_to_world = rotation_matrix_from_xyzw(np.asarray(quat, dtype=np.float64))
    return points.astype(np.float64) @ camera_to_world.T + np.asarray(position, dtype=np.float64)


def filter_points(
    points: np.ndarray,
    colors: np.ndarray,
    args: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray, dict]:
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    colors = colors[finite]
    if len(points) == 0:
        return points, colors, {"kept": 0}

    depth = -points[:, 2] if args.camera_forward == "neg-z" else points[:, 2]
    mask = (depth >= args.min_depth) & (depth <= args.max_depth)
    points = points[mask]
    colors = colors[mask]
    depth = depth[mask]

    if args.max_display_points > 0 and len(points) > args.max_display_points:
        idx = np.linspace(0, len(points) - 1, args.max_display_points).astype(np.int64)
        points = points[idx]
        colors = colors[idx]
        depth = depth[idx]

    stats = {
        "kept": int(len(points)),
        "depth_min": float(depth.min()) if len(depth) else None,
        "depth_max": float(depth.max()) if len(depth) else None,
    }
    return points, colors, stats


def update_cloud(
    pcd: o3d.geometry.PointCloud,
    points: np.ndarray,
    colors: np.ndarray,
    args: argparse.Namespace,
) -> None:
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)
    if args.voxel_size > 0 and len(points) > 0:
        down = pcd.voxel_down_sample(args.voxel_size)
        pcd.points = down.points
        pcd.colors = down.colors


def save_cloud(save_dir: Path, frame_idx: int, points: np.ndarray, colors: np.ndarray) -> None:
    save_dir.mkdir(parents=True, exist_ok=True)
    stem = f"cloud3d_{frame_idx:06d}"
    np.savez_compressed(save_dir / f"{stem}.npz", points=points, colors=colors)
    save_binary_ply(points.astype("<f4"), colors.astype("u1"), save_dir / f"{stem}.ply")
    print(f"Saved {save_dir / stem}")


def make_visualizer(args: argparse.Namespace) -> tuple[o3d.visualization.VisualizerWithKeyCallback, o3d.geometry.PointCloud]:
    vis = o3d.visualization.VisualizerWithKeyCallback()
    created = vis.create_window("LOTA Open3D point cloud", width=args.width, height=args.height)
    if not created:
        raise RuntimeError(
            "Open3D could not create an OpenGL window. In WSL this usually means "
            "the active display backend does not provide a working OpenGL context."
        )
    render = vis.get_render_option()
    if render is None:
        raise RuntimeError(
            "Open3D created a window object but did not initialize rendering. "
            "This usually follows a GLFW/GLEW failure in WSL."
        )
    render.background_color = np.asarray(args.background)
    render.point_size = args.point_size

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)
    if args.axes:
        vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=args.axis_size))
    return vis, pcd


def show_once(points: np.ndarray, colors: np.ndarray, args: argparse.Namespace) -> None:
    points, colors, stats = filter_points(points, colors, args)
    print(f"Displaying {stats['kept']} points, depth {stats['depth_min']}..{stats['depth_max']}")
    pcd = o3d.geometry.PointCloud()
    update_cloud(pcd, points, colors, args)
    geometries: list[o3d.geometry.Geometry] = [pcd]
    if args.axes:
        geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=args.axis_size))
    o3d.visualization.draw_geometries(
        geometries,
        window_name="LOTA Open3D saved cloud",
        width=args.width,
        height=args.height,
    )


def live_loop(args: argparse.Namespace) -> None:
    server = open_server(args.host, args.port)
    print(f"Listening for LOTA PLY point cloud on {args.host}:{args.port}")
    vis, pcd = make_visualizer(args)
    conn: socket.socket | None = None
    frame_idx = 0
    last_print = time.time()
    latest_points: np.ndarray | None = None
    latest_colors: np.ndarray | None = None
    save_dir = Path(args.save_dir) if args.save_dir else None
    should_quit = False

    def request_quit(_: o3d.visualization.Visualizer) -> bool:
        nonlocal should_quit
        should_quit = True
        return False

    def request_save(_: o3d.visualization.Visualizer) -> bool:
        if save_dir is not None and latest_points is not None and latest_colors is not None:
            save_cloud(save_dir, frame_idx, latest_points, latest_colors)
        return False

    vis.register_key_callback(ord("Q"), request_quit)
    vis.register_key_callback(256, request_quit)  # Escape on many Open3D builds.
    vis.register_key_callback(ord("S"), request_save)

    try:
        while not should_quit:
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
            points, colors, stats = filter_points(points, colors, args)
            latest_points = points
            latest_colors = colors
            update_cloud(pcd, points, colors, args)
            vis.update_geometry(pcd)
            if not vis.poll_events():
                break
            vis.update_renderer()

            now = time.time()
            if now - last_print >= args.print_every:
                last_print = now
                print(
                    f"frame={frame_idx} points={stats['kept']} "
                    f"depth={stats['depth_min']}..{stats['depth_max']} "
                    f"frame={args.display_frame} fwd={args.camera_forward}"
                )
    finally:
        if conn is not None:
            conn.close()
        server.close()
        vis.destroy_window()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9848)
    parser.add_argument("--npz", type=Path, default=None, help="View a saved .npz cloud instead of listening live.")
    parser.add_argument("--display-frame", choices=["camera", "world"], default="camera")
    parser.add_argument("--camera-forward", choices=["neg-z", "pos-z"], default="neg-z")
    parser.add_argument("--min-depth", type=float, default=0.05)
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument("--max-points", type=int, default=1_000_000)
    parser.add_argument("--max-display-points", type=int, default=250_000)
    parser.add_argument("--voxel-size", type=float, default=0.0)
    parser.add_argument("--point-size", type=float, default=2.0)
    parser.add_argument("--width", type=int, default=1200)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--axes", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--axis-size", type=float, default=0.15)
    parser.add_argument("--background", type=float, nargs=3, default=[0.02, 0.02, 0.02])
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--save-dir", default=None)
    args = parser.parse_args()

    if args.npz is not None:
        points, colors, meta = load_npz(args.npz)
        if args.display_frame == "world":
            points = transform_saved_cloud_to_world(points, meta)
        show_once(points, colors, args)
        return

    if args.display_frame == "world":
        raise SystemExit("--display-frame world is only supported for saved .npz clouds with pose metadata")
    try:
        live_loop(args)
    except RuntimeError as exc:
        raise SystemExit(
            f"{exc}\n\n"
            "Use lota_browser_3d_live_viewer.py for real-time 3D viewing in WSL, "
            "or run Open3D on a machine/session with working desktop OpenGL."
        ) from exc


if __name__ == "__main__":
    main()
