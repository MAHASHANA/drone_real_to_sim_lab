#!/usr/bin/env python3
"""Asynchronous mirrored-scene rendering for the Quest wrist camera."""

from __future__ import annotations

import math
import multiprocessing as mp
import queue
import time
import traceback
from dataclasses import dataclass

import cv2
import numpy as np

from pybullet_utils import connect_pybullet
from robot_arm_pybullet import (
    PANDA_ARM_JOINTS,
    PANDA_EE_LINK,
    PANDA_FINGER_JOINTS,
    load_panda,
)
from wrist_rgbd_camera import WristCameraConfig, WristRgbdCamera


PANDA_RENDER_JOINTS = tuple(PANDA_ARM_JOINTS + PANDA_FINGER_JOINTS)
WORKCELL_BODY_NAMES = ("table", "blue_block", "orange_block", "green_block")


@dataclass(frozen=True)
class RenderSnapshot:
    sequence: int
    source_time: float
    joint_positions: tuple[float, ...]
    body_poses: tuple[
        tuple[
            tuple[float, float, float],
            tuple[float, float, float, float],
        ],
        ...,
    ]


def create_box(
    p,
    client_id: int,
    half_extents,
    position,
    color,
    mass: float = 0.0,
) -> int:
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        physicsClientId=client_id,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=color,
        physicsClientId=client_id,
    )
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position,
        physicsClientId=client_id,
    )


def create_workcell(p, client_id: int) -> list[int]:
    bodies = [
        create_box(
            p,
            client_id,
            half_extents=[0.46, 0.38, 0.025],
            position=[0.0, -0.36, -0.025],
            color=[0.22, 0.25, 0.27, 1.0],
        )
    ]
    block_specs = [
        ([-0.13, -0.40, 0.025], [0.05, 0.035, 0.025], [0.12, 0.55, 0.95, 1.0]),
        ([0.02, -0.36, 0.035], [0.04, 0.04, 0.035], [0.95, 0.35, 0.15, 1.0]),
        ([0.15, -0.46, 0.02], [0.055, 0.025, 0.02], [0.25, 0.82, 0.43, 1.0]),
    ]
    for position, half_extents, color in block_specs:
        bodies.append(
            create_box(
                p,
                client_id,
                half_extents=half_extents,
                position=position,
                color=color,
                mass=0.05,
            )
        )
    return bodies


def encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    ok, encoded = cv2.imencode(
        ".jpg",
        image,
        [cv2.IMWRITE_JPEG_QUALITY, quality],
    )
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return encoded.tobytes()


def colorize_depth(depth_m: np.ndarray, near_m: float, far_m: float) -> np.ndarray:
    valid = np.isfinite(depth_m) & (depth_m >= near_m) & (depth_m < far_m * 0.999)
    normalized = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.any(valid):
        scaled = (depth_m[valid] - near_m) / (far_m - near_m)
        normalized[valid] = np.clip(scaled * 255.0, 0.0, 255.0).astype(np.uint8)
    colorized = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
    colorized[~valid] = 0
    return colorized


def render_side_view(p, client_id: int, camera_config: WristCameraConfig) -> np.ndarray:
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[0.50, -0.39, 0.18],
        cameraTargetPosition=[0.0, -0.39, 0.15],
        cameraUpVector=[0.0, 0.0, 1.0],
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=38.0,
        aspect=camera_config.width / camera_config.height,
        nearVal=camera_config.near_m,
        farVal=camera_config.far_m,
    )
    image = p.getCameraImage(
        width=camera_config.width,
        height=camera_config.height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER,
        flags=p.ER_NO_SEGMENTATION_MASK,
        physicsClientId=client_id,
    )
    rgba = np.asarray(image[2], dtype=np.uint8).reshape(
        camera_config.height,
        camera_config.width,
        4,
    )
    return rgba[:, :, :3].copy()


def _put_latest(target_queue, value) -> None:
    try:
        target_queue.put_nowait(value)
        return
    except queue.Full:
        pass
    try:
        target_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        target_queue.put_nowait(value)
    except queue.Full:
        pass


def _drain_latest(source_queue, latest):
    while True:
        try:
            latest = source_queue.get_nowait()
        except queue.Empty:
            return latest


def _apply_snapshot(p, client_id: int, panda_id: int, bodies: list[int], snapshot: RenderSnapshot) -> None:
    for joint_index, position in zip(PANDA_RENDER_JOINTS, snapshot.joint_positions):
        p.resetJointState(
            panda_id,
            joint_index,
            position,
            physicsClientId=client_id,
        )
    for body_id, (position, orientation) in zip(bodies, snapshot.body_poses):
        p.resetBasePositionAndOrientation(
            body_id,
            position,
            orientation,
            physicsClientId=client_id,
        )


def _render_process(
    input_queue,
    output_queue,
    stop_event,
    camera_config: WristCameraConfig,
    camera_fps: float,
    jpeg_quality: int,
) -> None:
    p = None
    client_id = -1
    try:
        p, client_id = connect_pybullet(gui=False)
        p.loadURDF("plane.urdf", physicsClientId=client_id)
        bodies = create_workcell(p, client_id)
        panda_id = load_panda(
            p,
            client_id,
            (0.0, -0.78, 0.0),
            math.radians(90.0),
        )
        camera = WristRgbdCamera(
            p,
            client_id,
            panda_id,
            PANDA_EE_LINK,
            camera_config,
        )
        period = 1.0 / camera_fps
        next_frame_time = time.monotonic()
        latest_snapshot = None

        while not stop_event.is_set():
            timeout = max(0.0, min(0.05, next_frame_time - time.monotonic()))
            try:
                latest_snapshot = input_queue.get(timeout=timeout)
            except queue.Empty:
                pass
            latest_snapshot = _drain_latest(input_queue, latest_snapshot)
            now = time.monotonic()
            if latest_snapshot is None or now < next_frame_time:
                continue

            _apply_snapshot(p, client_id, panda_id, bodies, latest_snapshot)
            render_started = time.monotonic()
            rgb, depth_m = camera.render()
            side_rgb = render_side_view(p, client_id, camera_config)
            color_jpeg = encode_jpeg(
                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                jpeg_quality,
            )
            depth_jpeg = encode_jpeg(
                colorize_depth(depth_m, camera_config.near_m, camera_config.far_m),
                jpeg_quality,
            )
            side_jpeg = encode_jpeg(
                cv2.cvtColor(side_rgb, cv2.COLOR_RGB2BGR),
                jpeg_quality,
            )
            completed = time.monotonic()
            _put_latest(
                output_queue,
                {
                    "kind": "frame",
                    "sequence": latest_snapshot.sequence,
                    "source_time": latest_snapshot.source_time,
                    "completed_time": completed,
                    "render_ms": (completed - render_started) * 1000.0,
                    "color_jpeg": color_jpeg,
                    "depth_jpeg": depth_jpeg,
                    "side_jpeg": side_jpeg,
                    "width": camera_config.width,
                    "height": camera_config.height,
                },
            )
            next_frame_time = max(next_frame_time + period, completed)
    except BaseException:  # Report child-process failures to the parent process.
        _put_latest(
            output_queue,
            {
                "kind": "error",
                "traceback": traceback.format_exc(),
            },
        )
    finally:
        if p is not None and client_id >= 0:
            p.disconnect(physicsClientId=client_id)


class AsyncWristRenderer:
    def __init__(
        self,
        camera_config: WristCameraConfig,
        camera_fps: float,
        jpeg_quality: int,
    ) -> None:
        context = mp.get_context("spawn")
        self._input_queue = context.Queue(maxsize=2)
        self._output_queue = context.Queue(maxsize=2)
        self._stop_event = context.Event()
        self._process = context.Process(
            target=_render_process,
            args=(
                self._input_queue,
                self._output_queue,
                self._stop_event,
                camera_config,
                camera_fps,
                jpeg_quality,
            ),
            name="pybullet-wrist-renderer",
        )

    def start(self) -> None:
        self._process.start()

    def publish(self, snapshot: RenderSnapshot) -> None:
        _put_latest(self._input_queue, snapshot)

    def receive(self) -> list[dict]:
        frames = []
        while True:
            try:
                frames.append(self._output_queue.get_nowait())
            except queue.Empty:
                return frames

    def close(self) -> None:
        self._stop_event.set()
        self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        self._input_queue.close()
        self._output_queue.close()
