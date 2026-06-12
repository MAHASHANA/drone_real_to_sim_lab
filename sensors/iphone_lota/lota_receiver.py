#!/usr/bin/env python3
"""Receive LOTA depth maps or binary point clouds.

LOTA sends to the computer, so this script listens as a TCP server. In the LOTA
app, set Receiver IP to this machine's LAN IP, then enable the matching stream.

Depth TCP:
    Depth/Point Cloud/Blob modes, TCP/UDP Output on, Protocol TCP, port 9847.
    Payload is repeated 256x192 little-endian Float32 depth maps in meters.

PLY TCP:
    Point Cloud Stream / PLY Streaming on, port 9848.
    Payload per frame is UInt32 LE point_count, then point_count records:
    x Float32, y Float32, z Float32, r UInt8, g UInt8, b UInt8.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


DEPTH_WIDTH = 256
DEPTH_HEIGHT = 192
DEPTH_FRAME_BYTES = DEPTH_WIDTH * DEPTH_HEIGHT * 4
FRAME_HEADER_BYTES = 24
PLY_POINT_BYTES = 15


def recv_exact(conn: socket.socket, nbytes: int) -> bytes:
    chunks = []
    remaining = nbytes
    while remaining > 0:
        chunk = conn.recv(remaining)
        if not chunk:
            raise EOFError("connection closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_lota_depth_frame(conn: socket.socket) -> tuple[np.ndarray, dict]:
    """Read one LOTA TCP depth frame.

    Current LOTA TCP/UDP depth payloads include a 24-byte binary frame header:
    UInt64 timestamp/counter, UInt32 width, UInt32 height, UInt32 payload bytes,
    UInt32 frame type. Older/raw streams can be parsed by falling back to
    headerless 256x192 Float32 depth.
    """

    prefix = recv_exact(conn, FRAME_HEADER_BYTES)
    timestamp, width, height, payload_bytes, frame_type = struct.unpack("<QIIII", prefix)
    header = {
        "timestamp_or_counter": int(timestamp),
        "width": int(width),
        "height": int(height),
        "payload_bytes": int(payload_bytes),
        "frame_type": int(frame_type),
        "has_header": True,
    }
    valid_header = (
        0 < width <= 4096
        and 0 < height <= 4096
        and payload_bytes == width * height * 4
        and payload_bytes <= 128 * 1024 * 1024
    )
    if valid_header:
        raw = recv_exact(conn, payload_bytes)
        depth = np.frombuffer(raw, dtype="<f4").reshape((height, width)).copy()
        return depth, header

    raw_tail = recv_exact(conn, DEPTH_FRAME_BYTES - FRAME_HEADER_BYTES)
    raw = prefix + raw_tail
    depth = np.frombuffer(raw, dtype="<f4").reshape((DEPTH_HEIGHT, DEPTH_WIDTH)).copy()
    return depth, {"has_header": False, "width": DEPTH_WIDTH, "height": DEPTH_HEIGHT}


def depth_stats(depth: np.ndarray) -> dict:
    valid = np.isfinite(depth) & (depth > 0.0)
    if not np.any(valid):
        return {"valid_ratio": 0.0, "min_m": None, "max_m": None, "median_m": None}
    values = depth[valid]
    return {
        "valid_ratio": float(valid.mean()),
        "min_m": float(values.min()),
        "max_m": float(values.max()),
        "median_m": float(np.median(values)),
    }


def save_depth_preview(depth: np.ndarray, path: Path) -> None:
    valid = np.isfinite(depth) & (depth > 0.0)
    preview = np.zeros(depth.shape, dtype=np.uint8)
    if np.any(valid):
        clipped = np.clip(depth, 0.2, 4.0)
        norm = 255.0 * (1.0 - (clipped - 0.2) / (4.0 - 0.2))
        preview[valid] = norm[valid].astype(np.uint8)
    colored = cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)
    colored[~valid] = (0, 0, 0)
    cv2.imwrite(str(path), colored)


def save_binary_ply(points: np.ndarray, colors: np.ndarray, path: Path) -> None:
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")
    packed = np.empty(
        len(points),
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")],
    )
    packed["x"] = points[:, 0]
    packed["y"] = points[:, 1]
    packed["z"] = points[:, 2]
    packed["red"] = colors[:, 0]
    packed["green"] = colors[:, 1]
    packed["blue"] = colors[:, 2]
    with path.open("wb") as f:
        f.write(header)
        f.write(packed.tobytes())


def make_capture_dir(base: Path, mode: str) -> Path:
    session = datetime.now().strftime(f"lota_{mode}_%Y%m%d_%H%M%S")
    path = base / session
    path.mkdir(parents=True, exist_ok=True)
    return path


def listen(host: str, port: int) -> tuple[socket.socket, socket.socket, tuple[str, int]]:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    print(f"Listening on {host}:{port}")
    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Connected from {addr[0]}:{addr[1]}")
    return server, conn, addr


def receive_depth(args: argparse.Namespace) -> None:
    capture_dir = make_capture_dir(Path(args.out), "depth")
    meta_path = capture_dir / "session_meta.json"
    meta_path.write_text(
        json.dumps(
            {
                "mode": "depth",
                "width": DEPTH_WIDTH,
                "height": DEPTH_HEIGHT,
                "dtype": "float32_le_meters",
                "port": args.port,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"Writing depth frames to {capture_dir.resolve()}")

    server, conn, _ = listen(args.host, args.port)
    frame = 0
    try:
        while args.max_frames <= 0 or frame < args.max_frames:
            frame += 1
            depth, frame_header = read_lota_depth_frame(conn)
            stem = f"frame_{frame:06d}"
            np.save(capture_dir / f"{stem}_depth.npy", depth)
            if args.preview_every > 0 and frame % args.preview_every == 0:
                save_depth_preview(depth, capture_dir / f"{stem}_preview.png")
            stats = depth_stats(depth)
            (capture_dir / f"{stem}_meta.json").write_text(
                json.dumps({"time": time.time(), "stats": stats, "frame_header": frame_header}, indent=2) + "\n"
            )
            print(f"{stem}: median={stats['median_m']} valid={stats['valid_ratio']:.3f}")
    except EOFError:
        print("LOTA disconnected")
    finally:
        conn.close()
        server.close()


def receive_ply(args: argparse.Namespace) -> None:
    capture_dir = make_capture_dir(Path(args.out), "ply")
    meta_path = capture_dir / "session_meta.json"
    meta_path.write_text(
        json.dumps(
            {
                "mode": "ply",
                "point_record": "3 Float32 XYZ + 3 UInt8 RGB",
                "point_bytes": PLY_POINT_BYTES,
                "port": args.port,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"Writing point clouds to {capture_dir.resolve()}")

    server, conn, _ = listen(args.host, args.port)
    frame = 0
    dtype = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])
    try:
        while args.max_frames <= 0 or frame < args.max_frames:
            count_raw = recv_exact(conn, 4)
            point_count = struct.unpack("<I", count_raw)[0]
            if point_count > args.max_points:
                raise ValueError(f"Refusing huge point frame: {point_count} > {args.max_points}")
            raw = recv_exact(conn, point_count * PLY_POINT_BYTES)
            frame += 1
            records = np.frombuffer(raw, dtype=dtype, count=point_count)
            points = records["xyz"].copy()
            colors = records["rgb"].copy()
            stem = f"frame_{frame:06d}"
            np.savez_compressed(capture_dir / f"{stem}_points.npz", points=points, colors=colors)
            if args.save_ply_every > 0 and frame % args.save_ply_every == 0:
                save_binary_ply(points, colors, capture_dir / f"{stem}.ply")
            print(f"{stem}: points={point_count}")
    except EOFError:
        print("LOTA disconnected")
    finally:
        conn.close()
        server.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["depth", "ply"], default="depth")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--out", default="captures")
    parser.add_argument("--max-frames", type=int, default=0, help="0 means receive forever.")
    parser.add_argument("--preview-every", type=int, default=10, help="Depth mode preview PNG cadence. 0 disables.")
    parser.add_argument("--save-ply-every", type=int, default=10, help="PLY mode .ply save cadence. 0 disables.")
    parser.add_argument("--max-points", type=int, default=1_000_000)
    args = parser.parse_args()

    if args.port is None:
        args.port = 9847 if args.mode == "depth" else 9848

    if args.mode == "depth":
        receive_depth(args)
    else:
        receive_ply(args)


if __name__ == "__main__":
    main()
