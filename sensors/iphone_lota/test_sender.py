#!/usr/bin/env python3
"""Send a synthetic RGB-D frame to receiver.py for local validation."""

from __future__ import annotations

import argparse
import base64
import json
import urllib.request

import cv2
import numpy as np


def b64(data: bytes) -> str:
    return base64.b64encode(data).decode("ascii")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:5000/frame")
    args = parser.parse_args()

    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[:] = (28, 28, 28)
    cv2.rectangle(rgb, (180, 180), (460, 260), (30, 180, 240), -1)
    cv2.putText(rgb, "Mark4 depth test", (180, 320), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (240, 240, 240), 2)
    ok, jpeg = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise RuntimeError("JPEG encode failed")

    depth = np.full((192, 256), 0.85, dtype="<f4")
    depth[70:120, 80:180] = 0.55

    payload = {
        "timestamp": 0.0,
        "rgb_jpeg_base64": b64(jpeg.tobytes()),
        "depth_base64": b64(depth.tobytes()),
        "depth_width": depth.shape[1],
        "depth_height": depth.shape[0],
        "intrinsics": [[250.0, 0.0, 128.0], [0.0, 250.0, 96.0], [0.0, 0.0, 1.0]],
        "camera_transform": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    }
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(args.url, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as response:
        print(response.read().decode("utf-8"))


if __name__ == "__main__":
    main()

