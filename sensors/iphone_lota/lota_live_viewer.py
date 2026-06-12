#!/usr/bin/env python3
"""Live OpenCV viewer for LOTA TCP depth streams.

Use this when LOTA is in Depth / Point Cloud / Blob Track mode with TCP output
enabled on port 9847. It displays the raw 256x192 Float32 depth map as a
colorized live window through WSLg/X11.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from lota_receiver import DEPTH_HEIGHT, DEPTH_WIDTH, depth_stats, read_lota_depth_frame


def colorize_depth(depth: np.ndarray, min_depth: float, max_depth: float) -> np.ndarray:
    valid = np.isfinite(depth) & (depth > 0.0)
    clipped = np.clip(depth, min_depth, max_depth)
    gray = 255.0 * (1.0 - (clipped - min_depth) / max(1e-6, max_depth - min_depth))
    image = np.zeros(depth.shape, dtype=np.uint8)
    image[valid] = gray[valid].astype(np.uint8)
    color = cv2.applyColorMap(image, cv2.COLORMAP_TURBO)
    color[~valid] = (0, 0, 0)
    return color


def draw_overlay(image: np.ndarray, frame_idx: int, fps: float, stats: dict, min_depth: float, max_depth: float) -> np.ndarray:
    out = image.copy()
    median = stats["median_m"]
    median_text = "none" if median is None else f"{median:.3f}m"
    lines = [
        f"LOTA depth  {frame_idx}  {fps:5.1f} FPS",
        f"valid {stats['valid_ratio']:.2f}  median {median_text}",
        f"range {min_depth:.2f}m - {max_depth:.2f}m   q/esc quit  s save",
    ]
    y = 22
    for line in lines:
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y += 22
    return out


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


def make_capture_dir(base: str | None) -> Path | None:
    if not base:
        return None
    path = Path(base) / datetime.now().strftime("lota_live_%Y%m%d_%H%M%S")
    path.mkdir(parents=True, exist_ok=True)
    print(f"Saving selected frames to {path.resolve()}")
    return path


def save_frame(capture_dir: Path, frame_idx: int, depth: np.ndarray, view: np.ndarray, stats: dict) -> None:
    stem = f"frame_{frame_idx:06d}"
    np.save(capture_dir / f"{stem}_depth.npy", depth)
    cv2.imwrite(str(capture_dir / f"{stem}_view.png"), view)
    (capture_dir / f"{stem}_meta.json").write_text(
        json.dumps({"time": time.time(), "stats": stats}, indent=2) + "\n"
    )
    print(f"Saved {stem}")


def synthetic_depth(t: float) -> np.ndarray:
    yy, xx = np.mgrid[0:DEPTH_HEIGHT, 0:DEPTH_WIDTH]
    cx = DEPTH_WIDTH * (0.5 + 0.25 * np.sin(t))
    cy = DEPTH_HEIGHT * (0.5 + 0.18 * np.cos(t * 0.7))
    depth = np.full((DEPTH_HEIGHT, DEPTH_WIDTH), 1.25, dtype=np.float32)
    blob = ((xx - cx) ** 2 / 42.0**2 + (yy - cy) ** 2 / 28.0**2) < 1.0
    plate = (xx > 70) & (xx < 188) & (yy > 88) & (yy < 115)
    depth[blob] = 0.55 + 0.08 * np.sin(t * 2.0)
    depth[plate] = 0.78
    return depth


def show_loop(args: argparse.Namespace, server: socket.socket | None) -> None:
    capture_dir = make_capture_dir(args.save_dir)
    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window_name, DEPTH_WIDTH * args.scale, DEPTH_HEIGHT * args.scale)

    frame_idx = 0
    last_t = time.time()
    fps = 0.0
    conn: socket.socket | None = None
    try:
        while args.max_frames <= 0 or frame_idx < args.max_frames:
            if args.demo:
                depth = synthetic_depth(time.time())
                time.sleep(1.0 / max(1.0, args.demo_fps))
            else:
                assert server is not None
                if conn is None:
                    print("Waiting for LOTA TCP depth connection...")
                    conn = accept_connection(server)
                try:
                    depth, _frame_header = read_lota_depth_frame(conn)
                except EOFError:
                    print("Peer disconnected before a complete depth frame")
                    conn.close()
                    conn = None
                    continue

            frame_idx += 1
            now = time.time()
            dt = now - last_t
            last_t = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else 1.0 / dt

            stats = depth_stats(depth)
            color = colorize_depth(depth, args.min_depth, args.max_depth)
            view = cv2.resize(color, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_NEAREST)
            view = draw_overlay(view, frame_idx, fps, stats, args.min_depth, args.max_depth)
            cv2.imshow(args.window_name, view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                if capture_dir is None:
                    capture_dir = make_capture_dir("captures")
                save_frame(capture_dir, frame_idx, depth, view, stats)
    finally:
        if conn is not None:
            conn.close()
        if server is not None:
            server.close()
        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0", help="Bind address. Use 0.0.0.0 for phone access.")
    parser.add_argument("--port", type=int, default=9847)
    parser.add_argument("--min-depth", type=float, default=0.20)
    parser.add_argument("--max-depth", type=float, default=2.50)
    parser.add_argument("--scale", type=int, default=4)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means run until q/esc/disconnect.")
    parser.add_argument("--save-dir", default=None, help="Directory for frames saved with 's'.")
    parser.add_argument("--window-name", default="LOTA iPhone depth")
    parser.add_argument("--demo", action="store_true", help="Show synthetic depth instead of opening a socket.")
    parser.add_argument("--demo-fps", type=float, default=30.0)
    args = parser.parse_args()

    if args.demo:
        show_loop(args, server=None)
        return

    server = open_server(args.host, args.port)
    print(f"Listening for LOTA TCP depth on {args.host}:{args.port}")
    show_loop(args, server=server)


if __name__ == "__main__":
    main()
