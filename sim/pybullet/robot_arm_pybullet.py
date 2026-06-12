#!/usr/bin/env python3
"""Robot-arm manipulation scaffold for Mark4 frame assembly in PyBullet.

This demo uses one or two Franka Panda arms from pybullet_data. The arm moves
to a selected loose frame part, closes the gripper, creates a fixed grasp
constraint, and carries the part to its target assembly pose.
"""

from __future__ import annotations

import argparse
import math
import time

from pybullet_utils import (
    add_debug_axes,
    connect_pybullet,
    create_part_body,
    load_manifest,
    table_pose,
    target_pose,
)


PANDA_ARM_JOINTS = list(range(7))
PANDA_FINGER_JOINTS = [9, 10]
PANDA_EE_LINK = 11
PANDA_REST = [0.0, -0.55, 0.0, -2.35, 0.0, 1.85, 0.78]
PANDA_LOWER = [-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671]
PANDA_UPPER = [2.9671, 1.8326, 2.9671, 0.0, 2.9671, 3.8223, 2.9671]
PANDA_RANGE = [u - l for l, u in zip(PANDA_LOWER, PANDA_UPPER)]


def load_panda(p, client_id: int, base_xyz, base_yaw: float) -> int:
    panda_id = p.loadURDF(
        "franka_panda/panda.urdf",
        basePosition=base_xyz,
        baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, base_yaw]),
        useFixedBase=True,
        physicsClientId=client_id,
    )
    for joint, value in zip(PANDA_ARM_JOINTS, PANDA_REST):
        p.resetJointState(panda_id, joint, value, physicsClientId=client_id)
    set_gripper(p, client_id, panda_id, open_width=0.04)
    return panda_id


def set_gripper(p, client_id: int, panda_id: int, open_width: float) -> None:
    for joint in PANDA_FINGER_JOINTS:
        p.setJointMotorControl2(
            panda_id,
            joint,
            p.POSITION_CONTROL,
            targetPosition=open_width,
            force=30.0,
            physicsClientId=client_id,
        )


def command_ee_pose(p, client_id: int, panda_id: int, xyz, orn) -> None:
    joint_positions = p.calculateInverseKinematics(
        panda_id,
        PANDA_EE_LINK,
        xyz,
        orn,
        lowerLimits=PANDA_LOWER,
        upperLimits=PANDA_UPPER,
        jointRanges=PANDA_RANGE,
        restPoses=PANDA_REST,
        maxNumIterations=80,
        residualThreshold=1e-4,
        physicsClientId=client_id,
    )
    for joint, target in zip(PANDA_ARM_JOINTS, joint_positions[:7]):
        p.setJointMotorControl2(
            panda_id,
            joint,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=220.0,
            maxVelocity=1.2,
            physicsClientId=client_id,
        )


def step_for(p, client_id: int, steps: int, gui: bool) -> None:
    for _ in range(steps):
        p.stepSimulation(physicsClientId=client_id)
        if gui:
            time.sleep(1.0 / 240.0)


def create_grasp_constraint(p, client_id: int, panda_id: int, body_id: int) -> int:
    ee_state = p.getLinkState(panda_id, PANDA_EE_LINK, physicsClientId=client_id)
    body_pos, body_orn = p.getBasePositionAndOrientation(body_id, physicsClientId=client_id)

    inv_ee_pos, inv_ee_orn = p.invertTransform(ee_state[4], ee_state[5])
    local_body_pos, local_body_orn = p.multiplyTransforms(
        inv_ee_pos,
        inv_ee_orn,
        body_pos,
        body_orn,
    )

    return p.createConstraint(
        parentBodyUniqueId=panda_id,
        parentLinkIndex=PANDA_EE_LINK,
        childBodyUniqueId=body_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=local_body_pos,
        parentFrameOrientation=local_body_orn,
        childFramePosition=[0, 0, 0],
        childFrameOrientation=[0, 0, 0, 1],
        physicsClientId=client_id,
    )


def create_assembly_constraint(p, client_id: int, base_body: int, part_body: int, part: dict) -> int:
    base_xyz, _ = target_pose({"pose_xyz_m": [0, 0, 0], "pose_rpy_rad": [0, 0, 0]}, z_offset=0.04)
    xyz, rpy = target_pose(part, z_offset=0.04)
    relative_xyz = [xyz[i] - base_xyz[i] for i in range(3)]
    return p.createConstraint(
        parentBodyUniqueId=base_body,
        parentLinkIndex=-1,
        childBodyUniqueId=part_body,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=relative_xyz,
        parentFrameOrientation=p.getQuaternionFromEuler(rpy),
        childFramePosition=[0, 0, 0],
        childFrameOrientation=[0, 0, 0, 1],
        physicsClientId=client_id,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Open the PyBullet GUI.")
    parser.add_argument("--arms", type=int, choices=[1, 2], default=1)
    parser.add_argument(
        "--parts",
        nargs="+",
        default=["front_right_arm"],
        help="Part names to grab. With --arms 2, pass two names.",
    )
    parser.add_argument("--approach-steps", type=int, default=180)
    parser.add_argument("--carry-steps", type=int, default=240)
    parser.add_argument("--hold-steps", type=int, default=240)
    args = parser.parse_args()

    parts = load_manifest()
    part_by_name = {part["name"]: part for part in parts}
    selected_names = args.parts[: args.arms]
    if len(selected_names) < args.arms:
        selected_names.append("front_left_arm")
    missing = [name for name in selected_names if name not in part_by_name]
    if missing:
        raise SystemExit(f"Unknown part name(s): {missing}")

    p, client_id = connect_pybullet(gui=args.gui)
    p.loadURDF("plane.urdf", physicsClientId=client_id)
    add_debug_axes(p, client_id)

    panda_specs = [
        ((0.0, -0.78, 0.0), math.radians(90.0)),
        ((0.45, -0.38, 0.0), math.radians(150.0)),
    ]
    pandas = [load_panda(p, client_id, *panda_specs[i]) for i in range(args.arms)]

    bodies: dict[str, int] = {}
    targets = {}
    starts = {}
    for i, part in enumerate(parts):
        xyz, rpy = table_pose(i, len(parts))
        fixed = part["name"] not in selected_names
        body_id = create_part_body(p, client_id, part, xyz, rpy, fixed=fixed)
        bodies[part["name"]] = body_id
        starts[part["name"]] = (xyz, rpy)
        targets[part["name"]] = target_pose(part)

    # Put the bottom plate at its target and fix it as the assembly root.
    bottom_xyz, bottom_rpy = target_pose(part_by_name["bottom_plate"])
    p.resetBasePositionAndOrientation(
        bodies["bottom_plate"],
        bottom_xyz,
        p.getQuaternionFromEuler(bottom_rpy),
        physicsClientId=client_id,
    )

    grasp_constraints = []
    ee_down = p.getQuaternionFromEuler([math.pi, 0.0, math.pi / 2.0])
    for panda_id, name in zip(pandas, selected_names):
        part_xyz, _ = starts[name]
        above = (part_xyz[0], part_xyz[1], part_xyz[2] + 0.18)
        grasp = (part_xyz[0], part_xyz[1], part_xyz[2] + 0.055)
        command_ee_pose(p, client_id, panda_id, above, ee_down)
        step_for(p, client_id, args.approach_steps, args.gui)
        command_ee_pose(p, client_id, panda_id, grasp, ee_down)
        step_for(p, client_id, args.approach_steps, args.gui)
        set_gripper(p, client_id, panda_id, open_width=0.0)
        step_for(p, client_id, 60, args.gui)
        grasp_constraints.append(create_grasp_constraint(p, client_id, panda_id, bodies[name]))
        print(f"Arm grasped {name}")

    for panda_id, name in zip(pandas, selected_names):
        target_xyz, _ = targets[name]
        carry = (target_xyz[0], target_xyz[1], target_xyz[2] + 0.16)
        place = (target_xyz[0], target_xyz[1], target_xyz[2] + 0.055)
        command_ee_pose(p, client_id, panda_id, carry, ee_down)
        step_for(p, client_id, args.carry_steps, args.gui)
        command_ee_pose(p, client_id, panda_id, place, ee_down)
        step_for(p, client_id, args.carry_steps, args.gui)

    assembly_constraints = []
    for constraint_id, panda_id, name in zip(grasp_constraints, pandas, selected_names):
        p.removeConstraint(constraint_id, physicsClientId=client_id)
        set_gripper(p, client_id, panda_id, open_width=0.04)
        assembly_constraints.append(
            create_assembly_constraint(
                p,
                client_id,
                bodies["bottom_plate"],
                bodies[name],
                part_by_name[name],
            )
        )
        print(f"Attached {name} to bottom_plate target pose")

    step_for(p, client_id, args.hold_steps, args.gui)
    print(
        f"Loaded {args.arms} Panda arm(s), grasped {len(grasp_constraints)} part(s), "
        f"created {len(assembly_constraints)} assembly constraint(s)."
    )
    p.disconnect(physicsClientId=client_id)


if __name__ == "__main__":
    main()
