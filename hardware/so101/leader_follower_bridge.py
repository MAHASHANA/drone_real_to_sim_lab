#!/usr/bin/env python3
"""Low-latency SO-101 leader/follower control with localhost telemetry."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
import uuid
from pathlib import Path
from typing import Mapping

from lerobot.processor import make_default_processors
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.robot_utils import precise_sleep


JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
DEFAULT_LEADER_PORT = (
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E113301-if00"
)
DEFAULT_FOLLOWER_PORT = (
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E115987-if00"
)


def positions(action: Mapping[str, float]) -> dict[str, float]:
    """Extract a complete calibrated joint vector from a LeRobot action."""
    result = {}
    for joint in JOINT_NAMES:
        key = f"{joint}.pos"
        if key not in action:
            raise KeyError(f"LeRobot action is missing {key!r}")
        value = float(action[key])
        if not math.isfinite(value):
            raise ValueError(f"LeRobot action contains non-finite {key}={value}")
        result[joint] = value
    return result


def make_packet(
    session_id: str,
    sequence: int,
    mode: str,
    leader: Mapping[str, float],
    follower: Mapping[str, float],
    command: Mapping[str, float] | None,
    loop_hz: float,
) -> bytes:
    payload = {
        "schema": "so101.telemetry.v1",
        "session_id": session_id,
        "sequence": sequence,
        "wall_time_ns": time.time_ns(),
        "monotonic_ns": time.monotonic_ns(),
        "mode": mode,
        "loop_hz": loop_hz,
        "units": {"body": "degree", "gripper": "percent"},
        "leader": dict(leader),
        "follower": dict(follower),
        "command": None if command is None else dict(command),
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--leader-port", default=DEFAULT_LEADER_PORT)
    result.add_argument("--follower-port", default=DEFAULT_FOLLOWER_PORT)
    result.add_argument("--leader-id", default="my_awesome_leader_arm")
    result.add_argument("--follower-id", default="my_awesome_follower_arm")
    result.add_argument("--fps", type=float, default=60.0)
    result.add_argument("--telemetry-host", default="127.0.0.1")
    result.add_argument("--telemetry-port", type=int, default=15001)
    result.add_argument(
        "--enable-motion",
        action="store_true",
        help="Required to enable follower torque and send commands.",
    )
    result.add_argument(
        "--yes",
        action="store_true",
        help="Skip the MOVE confirmation; intended only for a supervised launcher.",
    )
    return result


def validate_args(args: argparse.Namespace) -> None:
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.leader_port == args.follower_port:
        raise ValueError("Leader and follower ports must be different")
    for port in (args.leader_port, args.follower_port):
        if not Path(port).exists():
            raise FileNotFoundError(f"Serial port does not exist: {port}")


def run(args: argparse.Namespace) -> None:
    validate_args(args)
    if not args.enable_motion:
        raise SystemExit(
            "Motion is disabled. Re-run with --enable-motion after clearing the workspace."
        )

    if not args.yes:
        print(
            "Manually pose-match both arms, clamp both bases, clear the follower "
            "workspace, and keep power accessible."
        )
        confirmation = input("Type MOVE to start direct LeRobot mirroring: ")
        if confirmation != "MOVE":
            raise SystemExit("Cancelled; no arm connection was opened.")

    leader = SO101Leader(
        SO101LeaderConfig(port=args.leader_port, id=args.leader_id)
    )
    follower = SO101Follower(
        SO101FollowerConfig(
            port=args.follower_port,
            id=args.follower_id,
            max_relative_target=None,
            disable_torque_on_disconnect=True,
        )
    )
    telemetry = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.telemetry_host, args.telemetry_port)
    session_id = uuid.uuid4().hex
    sequence = 0
    next_report = time.monotonic()
    loop_hz = args.fps
    teleop_action_processor, robot_action_processor, _ = make_default_processors()

    try:
        leader.connect()
        follower.connect()
        print("Direct LeRobot mirroring is active.")
        while True:
            loop_start = time.perf_counter()
            raw_observation = follower.get_observation()
            raw_action = leader.get_action()
            teleop_action = teleop_action_processor((raw_action, raw_observation))
            robot_action = robot_action_processor((teleop_action, raw_observation))
            sent_action = follower.send_action(robot_action)

            follower_pose = positions(raw_observation)
            leader_pose = positions(raw_action)
            command = positions(sent_action)
            packet = make_packet(
                session_id,
                sequence,
                "mirroring",
                leader_pose,
                follower_pose,
                command,
                loop_hz,
            )
            telemetry.sendto(packet, destination)
            sequence += 1

            elapsed_before_sleep = time.perf_counter() - loop_start
            precise_sleep(max(1.0 / args.fps - elapsed_before_sleep, 0.0))
            loop_elapsed = time.perf_counter() - loop_start
            loop_hz = 1.0 / loop_elapsed if loop_elapsed > 0 else 0.0

            now = time.monotonic()
            if now >= next_report:
                print(f"mode=mirroring packets={sequence} loop={loop_hz:.1f} Hz")
                next_report = now + 1.0
    except KeyboardInterrupt:
        print("Stopping on Ctrl+C.")
    finally:
        telemetry.close()
        if leader.is_connected:
            try:
                leader.disconnect()
                print("Leader disconnected.")
            except Exception as error:
                print(f"WARNING: leader disconnect failed: {error}")
        if follower.is_connected:
            try:
                follower.disconnect()
                print("Follower disconnected; torque disabled.")
            except Exception as error:
                print(
                    f"WARNING: follower disconnect failed ({error}); "
                    "use the physical power switch"
                )


def main() -> None:
    run(parser().parse_args())


if __name__ == "__main__":
    main()
