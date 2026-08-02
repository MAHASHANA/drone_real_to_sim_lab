#!/usr/bin/env python3

"""Publish the latest D455 RGB frame using LeRobot's ZMQ camera protocol."""

from __future__ import annotations

import argparse
import base64
import json
import signal
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import zmq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="035322251087")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--camera-name", default="overhead")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("--jpeg-quality must be between 1 and 100")

    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps)

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(f"tcp://{args.bind}:{args.port}")

    profile = None
    frame_count = 0
    started_at = time.monotonic()
    try:
        profile = pipeline.start(config)
        print(
            f"D455 {args.serial}: {args.width}x{args.height}@{args.fps} RGB -> "
            f"tcp://{args.bind}:{args.port} as '{args.camera_name}'",
            flush=True,
        )

        while running:
            frames = pipeline.wait_for_frames(1000)
            color = frames.get_color_frame()
            if not color:
                continue

            image = np.asanyarray(color.get_data())
            encoded_ok, encoded = cv2.imencode(
                ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]
            )
            if not encoded_ok:
                continue

            message = {
                "timestamps": {args.camera_name: time.time()},
                "images": {args.camera_name: base64.b64encode(encoded).decode("ascii")},
            }
            try:
                socket.send_string(json.dumps(message), zmq.NOBLOCK)
            except zmq.Again:
                pass

            frame_count += 1
            if frame_count % (args.fps * 5) == 0:
                elapsed = max(time.monotonic() - started_at, 1e-6)
                print(f"published={frame_count} average_fps={frame_count / elapsed:.1f}", flush=True)
    finally:
        if profile is not None:
            pipeline.stop()
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
