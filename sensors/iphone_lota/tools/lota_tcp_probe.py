#!/usr/bin/env python3
"""Probe a LOTA TCP stream and print the first bytes received.

Use this when LOTA connects and immediately disconnects. It helps determine
whether the app is sending binary PLY, raw depth, OSC-like data, HTTP, or no
payload at all.
"""

from __future__ import annotations

import argparse
import socket
import struct
import time
from pathlib import Path


def hexdump(data: bytes, width: int = 16) -> str:
    lines = []
    for offset in range(0, len(data), width):
        chunk = data[offset : offset + width]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{offset:08x}  {hex_part:<{width * 3}}  {ascii_part}")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9848)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--max-bytes", type=int, default=4096)
    parser.add_argument("--out", default=None, help="Optional file to save raw bytes.")
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print(f"Waiting for TCP connection on {args.host}:{args.port}")
    conn, addr = server.accept()
    conn.settimeout(0.5)
    print(f"Connected from {addr[0]}:{addr[1]}")

    chunks = []
    deadline = time.time() + args.seconds
    try:
        while time.time() < deadline and sum(len(c) for c in chunks) < args.max_bytes:
            try:
                chunk = conn.recv(min(4096, args.max_bytes - sum(len(c) for c in chunks)))
            except socket.timeout:
                continue
            if not chunk:
                print("Peer closed connection")
                break
            chunks.append(chunk)
            print(f"received {len(chunk)} bytes, total {sum(len(c) for c in chunks)}")
    finally:
        conn.close()
        server.close()

    data = b"".join(chunks)
    print(f"\nTotal bytes: {len(data)}")
    if not data:
        print("No payload received. This was probably a connectivity test or LOTA opened TCP without streaming this data mode.")
        return

    if len(data) >= 4:
        first_u32 = struct.unpack("<I", data[:4])[0]
        print(f"First UInt32 little-endian: {first_u32}")
        if 0 < first_u32 < 1_000_000:
            expected_ply_frame = 4 + first_u32 * 15
            print(f"If this is LOTA PLY, first frame would be {expected_ply_frame} bytes.")
        if first_u32 == 0x3F800000:
            print("First 4 bytes look like Float32 1.0, which suggests raw Float32 depth data.")

    print("\nFirst bytes:")
    print(hexdump(data[:512]))

    if args.out:
        Path(args.out).write_bytes(data)
        print(f"\nSaved raw bytes to {args.out}")


if __name__ == "__main__":
    main()

