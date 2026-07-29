#!/usr/bin/env python3

"""Move the calibrated SO-101 follower slowly to a recorded vertical pose."""

from __future__ import annotations

import argparse
import math
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E115987-if00"
DEFAULT_ROBOT_ID = "my_awesome_follower_arm"

# Recorded from the physical follower while it was held near vertical.
VERTICAL_POSE = {
    "shoulder_pan": 4.264,
    "shoulder_lift": 5.934,
    "elbow_flex": -83.209,
    "wrist_flex": 6.022,
    "wrist_roll": -1.363,
    "gripper": 2.586,
}

BODY_MOTORS = tuple(name for name in VERTICAL_POSE if name != "gripper")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move an SO-101 follower to the recorded vertical pose using slow "
            "joint-space interpolation. The default is a read-only dry run."
        )
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID)
    parser.add_argument("--joint-speed-deg-s", type=float, default=8.0)
    parser.add_argument("--gripper-speed-percent-s", type=float, default=10.0)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--settle-seconds", type=float, default=0.75)
    parser.add_argument("--max-body-delta-deg", type=float, default=120.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Enable torque and execute the motion. Without this flag, only print the plan.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the typed MOVE confirmation. Only valid with --execute.",
    )
    args = parser.parse_args()

    positive_values = (
        args.joint_speed_deg_s,
        args.gripper_speed_percent_s,
        args.rate_hz,
        args.max_body_delta_deg,
    )
    if any(value <= 0 for value in positive_values):
        parser.error("speeds, rate, and maximum body delta must be positive")
    if args.settle_seconds < 0:
        parser.error("--settle-seconds cannot be negative")
    if args.yes and not args.execute:
        parser.error("--yes requires --execute")
    return args


def print_pose(label: str, pose: dict[str, float]) -> None:
    print(label)
    for motor in VERTICAL_POSE:
        unit = "%" if motor == "gripper" else "deg"
        print(f"  {motor:14s} {pose[motor]:9.3f} {unit}")


def plan_duration(
    current: dict[str, float],
    target: dict[str, float],
    joint_speed_deg_s: float,
    gripper_speed_percent_s: float,
) -> float:
    body_duration = max(
        abs(target[motor] - current[motor]) / joint_speed_deg_s for motor in BODY_MOTORS
    )
    gripper_duration = (
        abs(target["gripper"] - current["gripper"]) / gripper_speed_percent_s
    )
    return max(body_duration, gripper_duration, 1.0)


def validate_plan(
    current: dict[str, float],
    target: dict[str, float],
    max_body_delta_deg: float,
) -> None:
    excessive = {
        motor: target[motor] - current[motor]
        for motor in BODY_MOTORS
        if abs(target[motor] - current[motor]) > max_body_delta_deg
    }
    if excessive:
        details = ", ".join(f"{motor}={delta:.1f} deg" for motor, delta in excessive.items())
        raise RuntimeError(
            f"Refusing motion because body-joint delta exceeds "
            f"{max_body_delta_deg:.1f} deg: {details}"
        )


def main() -> None:
    args = parse_args()
    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
        )
    )
    bus = robot.bus
    connected = False

    try:
        bus.connect()
        connected = True
        bus.disable_torque()

        current = bus.sync_read("Present_Position")
        target = dict(VERTICAL_POSE)
        duration = plan_duration(
            current,
            target,
            args.joint_speed_deg_s,
            args.gripper_speed_percent_s,
        )
        deltas = {motor: target[motor] - current[motor] for motor in target}

        print_pose("Current pose:", current)
        print_pose("Vertical target:", target)
        print_pose("Required change:", deltas)
        print(f"Estimated motion time: {duration:.2f} s")

        try:
            validate_plan(current, target, args.max_body_delta_deg)
        except RuntimeError as exc:
            print(f"\nSafety check: {exc}")
            if args.execute:
                raise
            print(
                "Dry run only. Increase --max-body-delta-deg explicitly only "
                "after inspecting the full path."
            )
            return

        if not args.execute:
            print("\nDry run only. Add --execute after inspecting this plan.")
            return

        if not args.yes:
            confirmation = input(
                "\nClear the workspace and keep the power switch accessible. "
                "Type MOVE to continue: "
            )
            if confirmation.strip() != "MOVE":
                print("Motion cancelled.")
                return

        # Prevent a startup jump: write measured positions before enabling torque.
        bus.sync_write("Goal_Position", current)
        time.sleep(0.05)
        for motor in VERTICAL_POSE:
            bus.port_handler.clearPort()
            bus.enable_torque(motor, num_retry=5)
            print(f"Torque enabled: {motor}", flush=True)

        steps = max(1, math.ceil(duration * args.rate_hz))
        period = 1.0 / args.rate_hz
        print(f"Executing {steps} interpolated steps...")
        for step in range(1, steps + 1):
            alpha = step / steps
            goal = {
                motor: current[motor] + alpha * (target[motor] - current[motor])
                for motor in target
            }
            bus.sync_write("Goal_Position", goal)
            time.sleep(period)

        time.sleep(args.settle_seconds)
        final = bus.sync_read("Present_Position")
        print_pose("Measured final pose:", final)
        print_pose(
            "Final error:",
            {motor: target[motor] - final[motor] for motor in target},
        )
    finally:
        if connected:
            try:
                bus.disable_torque(num_retry=3)
            finally:
                bus.port_handler.closePort()
            print("Torque disabled.")


if __name__ == "__main__":
    main()
