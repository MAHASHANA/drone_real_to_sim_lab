#!/usr/bin/env python3
"""Receive RGB-D frames from the iPhone ARKit streamer.

The HTTP payload is JSON so the first version stays easy to inspect:

{
  "timestamp": ...,
  "rgb_jpeg_base64": "...",
  "depth_base64": "...",        # little-endian Float32 depth meters
  "depth_width": 256,
  "depth_height": 192,
  "intrinsics": [[fx,0,cx],[0,fy,cy],[0,0,1]],
  "camera_transform": [[...], ...]
}
"""

from __future__ import annotations

import argparse
import base64
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, jsonify, request


app = Flask(__name__)
CAPTURE_DIR: Path
FRAME_COUNT = 0


def decode_frame(payload: dict) -> tuple[np.ndarray, np.ndarray, dict]:
    rgb_bytes = base64.b64decode(payload["rgb_jpeg_base64"])
    rgb_array = np.frombuffer(rgb_bytes, dtype=np.uint8)
    rgb_bgr = cv2.imdecode(rgb_array, cv2.IMREAD_COLOR)
    if rgb_bgr is None:
        raise ValueError("Could not decode RGB JPEG")

    depth_width = int(payload["depth_width"])
    depth_height = int(payload["depth_height"])
    depth_bytes = base64.b64decode(payload["depth_base64"])
    depth = np.frombuffer(depth_bytes, dtype="<f4").reshape((depth_height, depth_width))

    meta = {
        "timestamp": payload.get("timestamp"),
        "rgb_width": int(rgb_bgr.shape[1]),
        "rgb_height": int(rgb_bgr.shape[0]),
        "depth_width": depth_width,
        "depth_height": depth_height,
        "intrinsics": payload.get("intrinsics"),
        "camera_transform": payload.get("camera_transform"),
        "camera_euler": payload.get("camera_euler"),
        "depth_confidence_width": payload.get("depth_confidence_width"),
        "depth_confidence_height": payload.get("depth_confidence_height"),
    }
    return rgb_bgr, depth, meta


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


@app.post("/frame")
def receive_frame():
    global FRAME_COUNT
    try:
        payload = request.get_json(force=True)
        rgb_bgr, depth, meta = decode_frame(payload)
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400

    FRAME_COUNT += 1
    stem = f"frame_{FRAME_COUNT:06d}"
    rgb_path = CAPTURE_DIR / f"{stem}_rgb.jpg"
    depth_path = CAPTURE_DIR / f"{stem}_depth.npy"
    meta_path = CAPTURE_DIR / f"{stem}_meta.json"

    meta["received_unix_s"] = time.time()
    meta["depth_stats"] = depth_stats(depth)

    cv2.imwrite(str(rgb_path), rgb_bgr)
    np.save(depth_path, depth)
    meta_path.write_text(json.dumps(meta, indent=2) + "\n")

    print(
        f"{stem}: rgb={meta['rgb_width']}x{meta['rgb_height']} "
        f"depth={meta['depth_width']}x{meta['depth_height']} "
        f"median_depth={meta['depth_stats']['median_m']}"
    )
    return jsonify({"ok": True, "frame": FRAME_COUNT})


@app.get("/health")
def health():
    return jsonify({"ok": True, "frames": FRAME_COUNT, "capture_dir": str(CAPTURE_DIR)})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--out", default="captures")
    args = parser.parse_args()

    global CAPTURE_DIR
    session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
    CAPTURE_DIR = Path(args.out) / session_name
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Writing captures to {CAPTURE_DIR.resolve()}")
    print(f"Listening on http://{args.host}:{args.port}/frame")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()

