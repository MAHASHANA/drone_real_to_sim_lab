#!/usr/bin/env python3
"""PyBullet assembly scaffold for depth-segmented Mark4 frame parts.

This is the simulator-side target for the real-to-sim loop:

1. Each physical part is represented as an independent rigid body.
2. The manifest provides target poses for the assembled frame.
3. A future depth-camera tracker can call the same pose update methods used
   here by the demo animation.
"""

from __future__ import annotations

import argparse
import time

from pybullet_utils import (
    add_debug_axes,
    connect_pybullet,
    create_part_body,
    load_manifest,
    table_pose,
    target_pose,
)


def lerp(a: float, b: float, t: float) -> float:
    return a * (1.0 - t) + b * t


def interpolate_pose(start_xyz, target_xyz, start_rpy, target_rpy, t: float):
    xyz = tuple(lerp(start_xyz[i], target_xyz[i], t) for i in range(3))
    rpy = tuple(lerp(start_rpy[i], target_rpy[i], t) for i in range(3))
    return xyz, rpy


def create_fixed_constraints(p, client_id: int, bodies: dict[str, int], parts: list[dict]) -> list[int]:
    constraints = []
    base_id = bodies["bottom_plate"]
    base_xyz, _ = target_pose(parts[0], z_offset=0.04)
    for part in parts:
        name = part["name"]
        if name == "bottom_plate":
            continue
        xyz, rpy = target_pose(part, z_offset=0.04)
        relative_xyz = [xyz[i] - base_xyz[i] for i in range(3)]
        constraint_id = p.createConstraint(
            parentBodyUniqueId=base_id,
            parentLinkIndex=-1,
            childBodyUniqueId=bodies[name],
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=relative_xyz,
            parentFrameOrientation=p.getQuaternionFromEuler(rpy),
            childFramePosition=[0, 0, 0],
            childFrameOrientation=[0, 0, 0, 1],
            physicsClientId=client_id,
        )
        constraints.append(constraint_id)
    return constraints


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Open the PyBullet GUI.")
    parser.add_argument("--layout", choices=["loose", "assembled"], default="loose")
    parser.add_argument("--assemble-demo", action="store_true", help="Animate loose parts into target poses.")
    parser.add_argument("--steps", type=int, default=480)
    parser.add_argument("--hold-steps", type=int, default=240)
    args = parser.parse_args()

    parts = load_manifest()
    p, client_id = connect_pybullet(gui=args.gui)
    p.loadURDF("plane.urdf", physicsClientId=client_id)
    add_debug_axes(p, client_id)

    bodies: dict[str, int] = {}
    starts: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {}
    targets: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {}

    for i, part in enumerate(parts):
        if args.layout == "assembled" and not args.assemble_demo:
            xyz, rpy = target_pose(part)
        else:
            xyz, rpy = table_pose(i, len(parts))
        body_id = create_part_body(p, client_id, part, xyz, rpy, fixed=True)
        bodies[part["name"]] = body_id
        starts[part["name"]] = (xyz, rpy)
        targets[part["name"]] = target_pose(part)

    print(f"Loaded {len(parts)} Mark4 parts as independent PyBullet bodies.")
    print(f"Layout: {args.layout}")

    if args.assemble_demo:
        print("Animating loose parts toward target assembled poses.")
        for step in range(args.steps):
            t = min(1.0, step / max(1, args.steps - 1))
            t = t * t * (3.0 - 2.0 * t)
            for part in parts:
                name = part["name"]
                xyz, rpy = interpolate_pose(*starts[name], *targets[name], t)
                p.resetBasePositionAndOrientation(
                    bodies[name],
                    xyz,
                    p.getQuaternionFromEuler(rpy),
                    physicsClientId=client_id,
                )
            p.stepSimulation(physicsClientId=client_id)
            if args.gui:
                time.sleep(1.0 / 240.0)
    else:
        for _ in range(args.steps):
            p.stepSimulation(physicsClientId=client_id)
            if args.gui:
                time.sleep(1.0 / 240.0)

    constraints = create_fixed_constraints(p, client_id, bodies, parts)
    print(f"Created {len(constraints)} fixed constraints to represent assembled attachments.")

    for _ in range(args.hold_steps):
        p.stepSimulation(physicsClientId=client_id)
        if args.gui:
            time.sleep(1.0 / 240.0)

    p.disconnect(physicsClientId=client_id)


if __name__ == "__main__":
    main()
