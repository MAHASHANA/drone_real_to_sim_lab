#!/usr/bin/env python3
"""Teleoperate an SO-101 follower and record a blind pick demonstration."""

from __future__ import annotations

import argparse
import time
from datetime import datetime, timezone
from pathlib import Path

import lerobot
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

from demo_trajectory import (
    SCHEMA,
    assert_pose_near,
    positions,
    write_jsonl_record,
)
from gripper_telemetry import read_gripper_telemetry, set_runtime_torque_limit


def default_output() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("captures") / f"so101_blind_{timestamp}" / "demo.jsonl"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-id", default="my_awesome_leader_arm")
    parser.add_argument("--follower-id", default="my_awesome_follower_arm")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--max-relative-target", type=float, default=2.0)
    parser.add_argument("--gripper-torque-limit", type=int, default=200)
    parser.add_argument("--max-start-body-delta", type=float, default=12.0)
    parser.add_argument("--max-start-gripper-delta", type=float, default=20.0)
    args = parser.parse_args()
    if args.fps <= 0.0:
        parser.error("--fps must be positive")
    if args.duration_sec < 0.0:
        parser.error("--duration-sec cannot be negative")
    if args.max_relative_target <= 0.0:
        parser.error("--max-relative-target must be positive")
    if not 1 <= args.gripper_torque_limit <= 500:
        parser.error("--gripper-torque-limit must be in [1, 500]")
    return args


def main() -> None:
    args = parse_args()
    output = args.output or default_output()
    leader = SO101Leader(
        SO101LeaderConfig(port=args.leader_port, id=args.leader_id)
    )
    follower = SO101Follower(
        SO101FollowerConfig(
            port=args.follower_port,
            id=args.follower_id,
            max_relative_target=args.max_relative_target,
        )
    )
    leader_connected = False
    follower_connected = False

    try:
        leader.connect()
        leader_connected = True
        follower.connect()
        follower_connected = True
        torque_limit = set_runtime_torque_limit(
            follower.bus,
            args.gripper_torque_limit,
        )
        print(
            "Gripper runtime torque limit: "
            f"{torque_limit['applied_raw']}/"
            f"{torque_limit['configured_max_raw']} configured maximum"
        )

        initial_leader = positions(leader.get_action())
        initial_follower = positions(follower.get_observation())
        assert_pose_near(
            initial_leader,
            initial_follower,
            max_body_delta=args.max_start_body_delta,
            max_gripper_delta=args.max_start_gripper_delta,
            context="Initial leader/follower pose",
        )

        print("Leader and follower are aligned within configured limits.")
        print("Clear the workspace and keep the follower power switch accessible.")
        if input("Type RECORD to enable teleoperation and recording: ").strip() != "RECORD":
            print("Recording cancelled.")
            return

        output.parent.mkdir(parents=True, exist_ok=True)
        period = 1.0 / args.fps
        start = time.perf_counter()
        next_frame_time = start
        frame_index = 0

        with output.open("w", encoding="utf-8") as handle:
            write_jsonl_record(
                handle,
                {
                    "type": "metadata",
                    "schema": SCHEMA,
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                    "lerobot_version": getattr(lerobot, "__version__", "unknown"),
                    "leader_port": args.leader_port,
                    "follower_port": args.follower_port,
                    "leader_id": args.leader_id,
                    "follower_id": args.follower_id,
                    "requested_fps": args.fps,
                    "max_relative_target": args.max_relative_target,
                    "gripper_torque_limit": torque_limit,
                    "gripper_telemetry_units": {
                        "present_current_raw": "servo count",
                        "present_current_a_est": "A, 0.0065 A/count nominal",
                        "present_load_raw": "signed servo count",
                        "present_load_percent_est": "%, abs(raw)/10 nominal",
                        "present_voltage_raw": "servo count",
                        "present_voltage_v": "V, 0.1 V/count",
                        "present_temperature_c": "degC",
                        "position_error_pct": "calibrated gripper range percent",
                    },
                    "initial_leader": initial_leader,
                    "initial_follower": initial_follower,
                },
            )
            print(f"Recording to {output}. Press Ctrl+C after the complete pick.")
            while True:
                loop_start = time.perf_counter()
                elapsed = loop_start - start
                if args.duration_sec and elapsed >= args.duration_sec:
                    break

                leader_action = positions(leader.get_action())
                command_time = time.perf_counter() - start
                sent_action = positions(follower.send_action(leader_action))
                follower_observation = positions(follower.get_observation())
                gripper_telemetry = read_gripper_telemetry(
                    follower.bus,
                    sent_action,
                    follower_observation,
                )
                observation_time = time.perf_counter() - start
                write_jsonl_record(
                    handle,
                    {
                        "type": "frame",
                        "index": frame_index,
                        "t_s": elapsed,
                        "command_t_s": command_time,
                        "observation_t_s": observation_time,
                        "leader_action": leader_action,
                        "sent_action": sent_action,
                        "follower_observation": follower_observation,
                        "gripper_telemetry": gripper_telemetry,
                    },
                )
                if frame_index % 20 == 0:
                    handle.flush()
                    print(
                        f"\rframes={frame_index + 1} elapsed={elapsed:.1f}s",
                        end="",
                        flush=True,
                    )
                frame_index += 1
                next_frame_time += period
                time.sleep(max(0.0, next_frame_time - time.perf_counter()))

        print(f"\nSaved {frame_index} frames to {output}")
    except KeyboardInterrupt:
        print(f"\nRecording stopped. Partial trajectory remains at {output}")
    finally:
        if follower_connected:
            follower.disconnect()
            print("Follower disconnected; torque disabled.")
        if leader_connected:
            leader.disconnect()


if __name__ == "__main__":
    main()
