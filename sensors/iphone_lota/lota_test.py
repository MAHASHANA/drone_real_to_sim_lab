#!/usr/bin/env python3
"""Small parser tests for lota_receiver.py without opening network sockets."""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

from lota_receiver import (
    DEPTH_FRAME_BYTES,
    DEPTH_HEIGHT,
    DEPTH_WIDTH,
    PLY_POINT_BYTES,
    read_lota_depth_frame,
    save_binary_ply,
)


def test_depth_shape() -> None:
    depth = np.linspace(0.2, 2.0, DEPTH_WIDTH * DEPTH_HEIGHT, dtype="<f4")
    raw = depth.tobytes()
    assert len(raw) == DEPTH_FRAME_BYTES
    parsed = np.frombuffer(raw, dtype="<f4").reshape((DEPTH_HEIGHT, DEPTH_WIDTH))
    assert parsed.shape == (DEPTH_HEIGHT, DEPTH_WIDTH)
    assert float(parsed[0, 0]) == float(depth[0])


class FakeSocket:
    def __init__(self, data: bytes):
        self.data = bytearray(data)

    def recv(self, nbytes: int) -> bytes:
        if not self.data:
            return b""
        n = min(nbytes, len(self.data))
        out = bytes(self.data[:n])
        del self.data[:n]
        return out


def test_lota_header_depth() -> None:
    depth = np.linspace(0.2, 2.0, DEPTH_WIDTH * DEPTH_HEIGHT, dtype="<f4")
    header = struct.pack("<QIIII", 0x01B3EB78, DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FRAME_BYTES, 2)
    parsed, meta = read_lota_depth_frame(FakeSocket(header + depth.tobytes()))
    assert parsed.shape == (DEPTH_HEIGHT, DEPTH_WIDTH)
    assert meta["has_header"] is True
    assert meta["width"] == DEPTH_WIDTH
    assert meta["height"] == DEPTH_HEIGHT
    assert meta["payload_bytes"] == DEPTH_FRAME_BYTES


def test_ply_record_dtype(tmp_path: Path = Path("/tmp/lota_test.ply")) -> None:
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype="<f4")
    colors = np.array([[255, 0, 0], [0, 255, 0]], dtype="u1")
    header = struct.pack("<I", len(points))
    records = np.empty(len(points), dtype=[("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])
    records["xyz"] = points
    records["rgb"] = colors
    raw = header + records.tobytes()
    assert len(raw) == 4 + len(points) * PLY_POINT_BYTES
    save_binary_ply(points, colors, tmp_path)
    assert tmp_path.exists()


if __name__ == "__main__":
    test_depth_shape()
    test_lota_header_depth()
    test_ply_record_dtype()
    print("LOTA parser tests passed")
