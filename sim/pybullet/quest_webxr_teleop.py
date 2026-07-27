#!/usr/bin/env python3
"""Quest controller teleop control loop for the PyBullet Panda arm.

The WebXR HTTPS/WebSocket server lives in quest_webxr_server.py. This file only
maps the latest Quest right-controller state into a PyBullet IK target.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

from pybullet_utils import add_debug_axes, connect_pybullet
from quest_webxr_server import MotionRecorder, TeleopState, serve_https
from robot_arm_pybullet import (
    PANDA_EE_LINK,
    command_joints,
    configure_robot_camera,
    load_panda,
    set_gripper,
    solve_ik,
)


def distance(a, b) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(a: list[list[float]]) -> list[list[float]]:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def quat_to_matrix(q: tuple[float, float, float, float]) -> list[list[float]]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-9:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def matrix_to_quat(m: list[list[float]]) -> tuple[float, float, float, float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def map_quest_relative_orientation(
    current_q: tuple[float, float, float, float],
    origin_q: tuple[float, float, float, float],
) -> list[list[float]]:
    # Same basis mapping as quest_to_robot(): robot [x,y,z] = [quest x, -quest z, quest y].
    quest_to_robot_basis = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    current_r = quat_to_matrix(current_q)
    origin_r = quat_to_matrix(origin_q)
    relative_quest = matmul(current_r, transpose(origin_r))
    return matmul(matmul(quest_to_robot_basis, relative_quest), transpose(quest_to_robot_basis))


def create_touch_demo_scene(p, client_id: int) -> tuple[int, tuple[float, float, float], float]:
    center = (0.0, -0.42, 0.16)
    radius = 0.045
    collision = p.createCollisionShape(
        p.GEOM_SPHERE,
        radius=radius,
        physicsClientId=client_id,
    )
    visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=[0.1, 0.55, 1.0, 0.9],
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=center,
        physicsClientId=client_id,
    )
    p.addUserDebugText(
        "haptic target",
        [center[0], center[1], center[2] + 0.075],
        textColorRGB=[0.2, 0.7, 1.0],
        textSize=1.1,
        physicsClientId=client_id,
    )
    return body_id, center, radius


def create_orientation_demo_object(p, client_id: int) -> int:
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[0.065, 0.014, 0.014],
        physicsClientId=client_id,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.065, 0.014, 0.014],
        rgbaColor=[1.0, 0.36, 0.12, 0.95],
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0.0, -0.42, 0.28],
        physicsClientId=client_id,
    )
    return body_id


def update_orientation_demo_object(p, client_id: int, body_id: int, ee_pos, object_orn) -> None:
    obj_pos, obj_orn = p.multiplyTransforms(
        ee_pos,
        object_orn,
        [0.0, 0.0, 0.08],
        [0.0, 0.0, 0.0, 1.0],
        physicsClientId=client_id,
    )
    p.resetBasePositionAndOrientation(body_id, obj_pos, obj_orn, physicsClientId=client_id)


def quest_to_robot(
    quest_position: tuple[float, float, float],
    origin: tuple[float, float, float],
    home: tuple[float, float, float],
    scale: float,
) -> tuple[float, float, float]:
    rel = tuple(quest_position[i] - origin[i] for i in range(3))
    x = home[0] + rel[0] * scale
    y = home[1] - rel[2] * scale
    z = home[2] + rel[1] * scale
    return (
        max(-0.35, min(0.35, x)),
        max(-0.72, min(0.02, y)),
        max(0.05, min(0.65, z)),
    )


def pybullet_loop(args: argparse.Namespace, state: TeleopState) -> None:
    p, client_id = connect_pybullet(gui=args.gui)
    p.loadURDF("plane.urdf", physicsClientId=client_id)
    add_debug_axes(p, client_id)
    panda_id = load_panda(p, client_id, (0.0, -0.78, 0.0), math.radians(90.0))
    if args.gui:
        configure_robot_camera(p, client_id, arms=1)
    touch_body = None
    touch_center = None
    touch_radius = 0.0
    if args.touch_demo:
        touch_body, touch_center, touch_radius = create_touch_demo_scene(p, client_id)
        print(f"Touch demo target at {tuple(round(v, 3) for v in touch_center)}")
    orientation_body = None
    if args.orientation_demo_object:
        orientation_body = create_orientation_demo_object(p, client_id)
        print("Orientation demo object follows end-effector position and controller wrist rotation")

    home_orn = p.getQuaternionFromEuler([math.pi, 0.0, math.pi / 2.0])
    home_orn_matrix = quat_to_matrix(tuple(home_orn))
    home = (args.home_x, args.home_y, args.home_z)
    origin: tuple[float, float, float] | None = None
    origin_orientation: tuple[float, float, float, float] | None = None
    rest_pose = None
    last_print = 0.0
    last_haptic = 0.0
    started = time.time()

    try:
        while args.run_seconds <= 0 or time.time() - started < args.run_seconds:
            snap = state.snapshot()
            quest_position = snap["right_position"]
            quest_orientation = snap["right_orientation"]
            if quest_position is not None:
                if origin is None:
                    origin = quest_position
                    print(f"Quest right-controller origin set to {tuple(round(v, 3) for v in origin)}")
                if args.orientation_mode == "controller" and quest_orientation is not None and origin_orientation is None:
                    origin_orientation = quest_orientation
                    print(
                        "Quest right-controller neutral orientation set to "
                        f"{tuple(round(v, 3) for v in origin_orientation)}"
                    )
                target = quest_to_robot(quest_position, origin, home, args.scale)
                desired_orn = home_orn
                if args.orientation_mode == "controller" and quest_orientation is not None and origin_orientation is not None:
                    rel_robot = map_quest_relative_orientation(quest_orientation, origin_orientation)
                    if args.orientation_order == "tool":
                        ee_matrix = matmul(home_orn_matrix, rel_robot)
                    else:
                        ee_matrix = matmul(rel_robot, home_orn_matrix)
                    desired_orn = matrix_to_quat(ee_matrix)
                arm_orn = desired_orn if args.orientation_target == "ik" else home_orn
                rest_pose = solve_ik(p, client_id, panda_id, target, arm_orn, rest_pose=rest_pose)
                command_joints(p, client_id, panda_id, rest_pose, max_velocity=args.max_velocity)
                gripper_closed = max(snap["grip_value"], snap["trigger_value"]) > 0.25
                set_gripper(p, client_id, panda_id, open_width=0.0 if gripper_closed else 0.04)
                now = time.time()
                ee_state = p.getLinkState(panda_id, PANDA_EE_LINK, physicsClientId=client_id)
                ee = ee_state[4]
                if orientation_body is not None:
                    object_orn = desired_orn if args.orientation_target == "object" else ee_state[5]
                    update_orientation_demo_object(p, client_id, orientation_body, ee, object_orn)
                if touch_body is not None and touch_center is not None and now - last_haptic >= args.haptic_interval:
                    dist = distance(ee, touch_center)
                    contact_margin = max(0.0, dist - touch_radius)
                    if contact_margin <= args.haptic_range:
                        closeness = 1.0 - min(1.0, contact_margin / max(args.haptic_range, 1e-6))
                        intensity = 0.2 + 0.55 * closeness
                        reason = "near"
                        if gripper_closed and contact_margin <= 0.035:
                            intensity = 0.9
                            reason = "grip-near-target"
                        state.request_haptic(intensity, args.haptic_duration_ms, reason)
                        last_haptic = now
                if now - last_print >= 1.0:
                    last_print = now
                    touch_text = ""
                    if touch_center is not None:
                        touch_text = f" touch_dist={distance(ee, touch_center):.3f}"
                    print(
                        "teleop",
                        f"packets={snap['packets']}",
                        f"target={tuple(round(v, 3) for v in target)}",
                        f"ee={tuple(round(v, 3) for v in ee)}",
                        f"orn_mode={args.orientation_mode}",
                        f"orn_target={args.orientation_target}",
                        f"grip={gripper_closed}",
                        touch_text,
                    )
            p.stepSimulation(physicsClientId=client_id)
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect(physicsClientId=client_id)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--advertise-ip", default="", help="LAN IP shown for the Quest URL. Default auto-detects.")
    parser.add_argument("--gui", action="store_true", help="Open PyBullet GUI.")
    parser.add_argument("--scale", type=float, default=0.65, help="Quest controller motion to robot workspace scale.")
    parser.add_argument("--home-x", type=float, default=0.0)
    parser.add_argument("--home-y", type=float, default=-0.42)
    parser.add_argument("--home-z", type=float, default=0.30)
    parser.add_argument("--max-velocity", type=float, default=1.0)
    parser.add_argument("--run-seconds", type=float, default=0.0, help="0 means run until Ctrl+C.")
    parser.add_argument("--orientation-mode", choices=["fixed", "controller"], default="controller")
    parser.add_argument("--orientation-target", choices=["object", "ik"], default="object")
    parser.add_argument("--orientation-order", choices=["tool", "world"], default="tool")
    parser.add_argument("--orientation-demo-object", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--touch-demo", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--haptic-range", type=float, default=0.07)
    parser.add_argument("--haptic-duration-ms", type=int, default=35)
    parser.add_argument("--haptic-interval", type=float, default=0.12)
    parser.add_argument("--record", action="store_true", help="Record Quest controller samples to captures/quest_tracking_*.")
    parser.add_argument("--out", default="captures")
    parser.add_argument("--motion-threshold-m", type=float, default=0.004)
    parser.add_argument("--idle-split-s", type=float, default=0.35)
    args = parser.parse_args()

    recorder = (
        MotionRecorder(Path(args.out), args.motion_threshold_m, args.idle_split_s)
        if args.record
        else None
    )
    state = TeleopState(recorder=recorder)
    server = serve_https(args.host, args.port, args.advertise_ip, state)
    try:
        pybullet_loop(args, state)
    finally:
        server.shutdown()
        server.server_close()
        if recorder is not None:
            recorder.close()


if __name__ == "__main__":
    main()
