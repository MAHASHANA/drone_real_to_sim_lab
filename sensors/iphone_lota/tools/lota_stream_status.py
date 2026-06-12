#!/usr/bin/env python3
"""Check which LOTA streams are currently arriving.

This is a diagnostic tool, not a recorder. Stop the live viewers/recorders
first, then run this script and start LOTA streaming. It listens for:

- TCP 9847: mode-dependent LOTA data stream, usually framed depth maps
- TCP 9848: PLY point-cloud stream
- UDP 9000: OSC camera pose, if UDP is forwarded into WSL
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SENSOR_ROOT = Path(__file__).resolve().parents[1]
if str(SENSOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SENSOR_ROOT))

from lota_osc_receiver import parse_osc_message
from lota_receiver import FRAME_HEADER_BYTES, PLY_POINT_BYTES, recv_exact


@dataclass
class StreamState:
    name: str
    ok: bool = False
    connections: int = 0
    frames: int = 0
    packets: int = 0
    last: dict[str, Any] = field(default_factory=dict)
    error: str | None = None


def tcp_server(host: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    server.settimeout(0.5)
    return server


def check_9847(host: str, port: int, deadline: float, state: StreamState) -> None:
    try:
        server = tcp_server(host, port)
    except OSError as exc:
        state.error = f"bind failed: {exc}"
        return

    try:
        while time.time() < deadline:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            state.connections += 1
            conn.settimeout(1.0)
            try:
                prefix = recv_exact(conn, FRAME_HEADER_BYTES)
                timestamp, width, height, payload_bytes, frame_type = struct.unpack("<QIIII", prefix)
                valid_header = (
                    0 < width <= 4096
                    and 0 < height <= 4096
                    and payload_bytes > 0
                    and payload_bytes <= 128 * 1024 * 1024
                )
                if valid_header:
                    # Drain one payload so we know it is a complete frame.
                    recv_exact(conn, payload_bytes)
                    state.ok = True
                    state.frames += 1
                    state.last = {
                        "remote": f"{addr[0]}:{addr[1]}",
                        "timestamp_or_counter": int(timestamp),
                        "width": int(width),
                        "height": int(height),
                        "payload_bytes": int(payload_bytes),
                        "frame_type": int(frame_type),
                        "looks_like_depth": payload_bytes == width * height * 4,
                    }
                else:
                    state.error = f"unknown 9847 header bytes={prefix.hex()}"
            except Exception as exc:
                state.error = str(exc)
            finally:
                conn.close()
    finally:
        server.close()


def check_9848(host: str, port: int, deadline: float, state: StreamState, max_points: int) -> None:
    try:
        server = tcp_server(host, port)
    except OSError as exc:
        state.error = f"bind failed: {exc}"
        return

    try:
        while time.time() < deadline:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            state.connections += 1
            conn.settimeout(1.0)
            try:
                count_raw = recv_exact(conn, 4)
                point_count = struct.unpack("<I", count_raw)[0]
                if 0 < point_count <= max_points:
                    recv_exact(conn, point_count * PLY_POINT_BYTES)
                    state.ok = True
                    state.frames += 1
                    state.last = {
                        "remote": f"{addr[0]}:{addr[1]}",
                        "point_count": int(point_count),
                        "payload_bytes": int(point_count * PLY_POINT_BYTES),
                    }
                else:
                    state.error = f"bad point count: {point_count}"
            except Exception as exc:
                state.error = str(exc)
            finally:
                conn.close()
    finally:
        server.close()


def check_osc(host: str, port: int, deadline: float, state: StreamState) -> None:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((host, port))
        sock.settimeout(0.5)
    except OSError as exc:
        state.error = f"bind failed: {exc}"
        return

    try:
        while time.time() < deadline:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            state.packets += 1
            try:
                messages = parse_osc_message(data)
                state.ok = True
                state.last = {
                    "remote": f"{addr[0]}:{addr[1]}",
                    "messages": messages,
                }
            except Exception as exc:
                state.error = str(exc)
    finally:
        sock.close()


def print_state(states: list[StreamState]) -> None:
    print("\nLOTA stream status")
    print("==================")
    for state in states:
        status = "OK" if state.ok else "NO DATA"
        print(f"{state.name}: {status}")
        if state.connections:
            print(f"  connections: {state.connections}")
        if state.frames:
            print(f"  frames: {state.frames}")
        if state.packets:
            print(f"  packets: {state.packets}")
        if state.last:
            print(f"  last: {state.last}")
        if state.error:
            print(f"  last error: {state.error}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--port-9847", type=int, default=9847)
    parser.add_argument("--port-ply", type=int, default=9848)
    parser.add_argument("--port-osc", type=int, default=9000)
    parser.add_argument("--max-points", type=int, default=1_000_000)
    args = parser.parse_args()

    deadline = time.time() + args.seconds
    states = [
        StreamState("TCP 9847 mode-dependent stream"),
        StreamState("TCP 9848 PLY point cloud"),
        StreamState("UDP 9000 OSC pose"),
    ]
    threads = [
        threading.Thread(target=check_9847, args=(args.host, args.port_9847, deadline, states[0]), daemon=True),
        threading.Thread(target=check_9848, args=(args.host, args.port_ply, deadline, states[1], args.max_points), daemon=True),
        threading.Thread(target=check_osc, args=(args.host, args.port_osc, deadline, states[2]), daemon=True),
    ]
    print(
        f"Listening for {args.seconds:.1f}s on TCP {args.port_9847}, "
        f"TCP {args.port_ply}, UDP {args.port_osc}. Start LOTA streaming now."
    )
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    print_state(states)


if __name__ == "__main__":
    main()
