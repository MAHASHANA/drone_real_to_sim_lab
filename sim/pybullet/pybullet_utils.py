"""Shared PyBullet helpers for the Mark4 frame demos."""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = Path(__file__).resolve().parent
URDF_PATH = PROJECT_ROOT / "generated" / "7in_fpv_frame.urdf"
MANIFEST_PATH = PROJECT_ROOT / "generated" / "part_manifest.json"


def ensure_generated() -> None:
    if URDF_PATH.exists() and MANIFEST_PATH.exists():
        return
    subprocess.run([sys.executable, str(SIM_ROOT / "generate_frame_urdf.py")], check=True)


def load_manifest() -> list[dict]:
    ensure_generated()
    return json.loads(MANIFEST_PATH.read_text())["parts"]


def import_pybullet():
    try:
        import pybullet as p
    except ImportError as exc:
        raise SystemExit(
            "PyBullet is not installed. Install it with:\n"
            "  python3 -m pip install --user pybullet\n"
        ) from exc
    return p


def connect_pybullet(gui: bool):
    p = import_pybullet()
    try:
        import pybullet_data
    except ImportError:
        pybullet_data = None

    mode = p.GUI if gui else p.DIRECT
    client_id = p.connect(mode)
    if pybullet_data is not None:
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0.0, 0.0, -9.81, physicsClientId=client_id)
    p.setTimeStep(1.0 / 240.0, physicsClientId=client_id)
    p.setPhysicsEngineParameter(numSolverIterations=80, physicsClientId=client_id)
    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.55,
            cameraYaw=-45,
            cameraPitch=-35,
            cameraTargetPosition=[0.0, 0.0, 0.03],
            physicsClientId=client_id,
        )
    return p, client_id


def table_pose(index: int, total: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    cols = 5
    spacing = 0.09
    row = index // cols
    col = index % cols
    width = min(cols, total) - 1
    x = (col - width / 2.0) * spacing
    y = -0.34 - row * spacing
    z = 0.035
    yaw = (index % 8) * math.pi / 8.0
    return (x, y, z), (0.0, 0.0, yaw)


def target_pose(part: dict, z_offset: float = 0.04) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    xyz = part["pose_xyz_m"]
    rpy = part["pose_rpy_rad"]
    return (xyz[0], xyz[1], xyz[2] + z_offset), tuple(rpy)


def create_part_body(p, client_id: int, part: dict, xyz, rpy, fixed: bool = False) -> int:
    rgba = [0.02, 0.025, 0.028, 1.0]
    if part["name"].startswith("stack_standoff"):
        rgba = [0.55, 0.57, 0.60, 1.0]
    elif part["name"].startswith("motor_mount"):
        rgba = [0.05, 0.05, 0.05, 1.0]
    elif part["name"] == "top_plate":
        rgba = [0.08, 0.10, 0.11, 1.0]

    if part["shape"] == "box":
        half_extents = [value / 2.0 for value in part["size_m"]]
        collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            physicsClientId=client_id,
        )
        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=rgba,
            physicsClientId=client_id,
        )
    elif part["shape"] == "cylinder":
        collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=part["radius_m"],
            height=part["height_m"],
            physicsClientId=client_id,
        )
        visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=part["radius_m"],
            length=part["height_m"],
            rgbaColor=rgba,
            physicsClientId=client_id,
        )
    else:
        raise ValueError(f"Unsupported part shape: {part['shape']}")

    mass = 0.0 if fixed else float(part["mass_kg"])
    quat = p.getQuaternionFromEuler(rpy)
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=xyz,
        baseOrientation=quat,
        physicsClientId=client_id,
    )


def add_debug_axes(p, client_id: int, length: float = 0.18) -> None:
    p.addUserDebugLine([0, 0, 0.002], [length, 0, 0.002], [1, 0, 0], physicsClientId=client_id)
    p.addUserDebugLine([0, 0, 0.002], [0, length, 0.002], [0, 1, 0], physicsClientId=client_id)
    p.addUserDebugLine([0, 0, 0.002], [0, 0, length], [0, 0.4, 1], physicsClientId=client_id)
