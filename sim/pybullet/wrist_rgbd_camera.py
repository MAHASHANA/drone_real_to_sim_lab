"""Eye-in-hand RGB-D camera rendering for a PyBullet robot link."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class WristCameraConfig:
    width: int = 320
    height: int = 180
    vertical_fov_deg: float = 60.0
    near_m: float = 0.03
    far_m: float = 2.0
    link_to_camera_xyz: tuple[float, float, float] = (0.0, 0.0, -0.08)
    link_to_camera_quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass
class WristRgbdCamera:
    p: object
    client_id: int
    robot_id: int
    link_index: int
    config: WristCameraConfig = field(default_factory=WristCameraConfig)
    show_frustum: bool = False

    def __post_init__(self) -> None:
        if self.config.width <= 0 or self.config.height <= 0:
            raise ValueError("Camera width and height must be positive")
        if not 0.0 < self.config.vertical_fov_deg < 180.0:
            raise ValueError("Camera vertical field of view must be between 0 and 180 degrees")
        if self.config.near_m <= 0 or self.config.far_m <= self.config.near_m:
            raise ValueError("Camera far plane must be greater than its positive near plane")
        self._projection = self.p.computeProjectionMatrixFOV(
            fov=self.config.vertical_fov_deg,
            aspect=self.config.width / self.config.height,
            nearVal=self.config.near_m,
            farVal=self.config.far_m,
        )
        self._frustum_line_ids: list[int] = []

    def world_pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        link_state = self.p.getLinkState(
            self.robot_id,
            self.link_index,
            computeForwardKinematics=True,
            physicsClientId=self.client_id,
        )
        position, orientation = self.p.multiplyTransforms(
            link_state[4],
            link_state[5],
            self.config.link_to_camera_xyz,
            self.config.link_to_camera_quat,
            physicsClientId=self.client_id,
        )
        return tuple(position), tuple(orientation)

    @staticmethod
    def optical_axes(
        rotation_matrix: tuple[float, ...] | list[float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        # OpenCV optical frame: +x right, +y down, +z forward.
        forward = (rotation_matrix[2], rotation_matrix[5], rotation_matrix[8])
        up = (-rotation_matrix[1], -rotation_matrix[4], -rotation_matrix[7])
        return forward, up

    def render(self) -> tuple[np.ndarray, np.ndarray]:
        position, orientation = self.world_pose()
        rotation = self.p.getMatrixFromQuaternion(orientation)
        forward, up = self.optical_axes(rotation)
        target = [position[i] + forward[i] for i in range(3)]
        view = self.p.computeViewMatrix(position, target, up)
        image = self.p.getCameraImage(
            width=self.config.width,
            height=self.config.height,
            viewMatrix=view,
            projectionMatrix=self._projection,
            renderer=self.p.ER_TINY_RENDERER,
            physicsClientId=self.client_id,
        )
        rgba = np.asarray(image[2], dtype=np.uint8).reshape(
            self.config.height,
            self.config.width,
            4,
        )
        depth_buffer = np.asarray(image[3], dtype=np.float32).reshape(
            self.config.height,
            self.config.width,
        )
        near = self.config.near_m
        far = self.config.far_m
        depth_m = (far * near) / (far - (far - near) * depth_buffer)
        if self.show_frustum:
            self.update_debug_frustum()
        return rgba[:, :, :3].copy(), depth_m

    def update_debug_frustum(self, length_m: float = 0.18) -> None:
        position, orientation = self.world_pose()
        half_height = length_m * math.tan(math.radians(self.config.vertical_fov_deg) / 2.0)
        half_width = half_height * self.config.width / self.config.height
        local_corners = [
            (-half_width, -half_height, length_m),
            (half_width, -half_height, length_m),
            (half_width, half_height, length_m),
            (-half_width, half_height, length_m),
        ]
        corners = [
            self.p.multiplyTransforms(
                position,
                orientation,
                corner,
                (0.0, 0.0, 0.0, 1.0),
                physicsClientId=self.client_id,
            )[0]
            for corner in local_corners
        ]
        segments = [(position, corner) for corner in corners]
        segments.extend((corners[i], corners[(i + 1) % 4]) for i in range(4))
        new_ids = []
        for index, (start, end) in enumerate(segments):
            replace_id = self._frustum_line_ids[index] if index < len(self._frustum_line_ids) else -1
            line_id = self.p.addUserDebugLine(
                start,
                end,
                [0.1, 0.9, 0.95],
                lineWidth=1.5,
                lifeTime=0,
                replaceItemUniqueId=replace_id,
                physicsClientId=self.client_id,
            )
            new_ids.append(line_id)
        self._frustum_line_ids = new_ids
