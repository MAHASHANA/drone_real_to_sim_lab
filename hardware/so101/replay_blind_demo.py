#!/usr/bin/env python3
"""Validate and optionally replay an SO-101 blind demonstration."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from demo_trajectory import (
    JOINT_KEYS,
    assert_pose_near,
    load_demo,
    positions,
    pose_delta,
    validate_frames,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--follower-port")
    parser.add_argument("--follower-id")
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--max-relative-target", type=float, default=3.0)
    parser.add_argument("--max-body-step", type=float, default=5.0)
    parser.add_argument("--max-gripper-step", type=float, default=10.0)
    parser.add_argument("--max-start-body-delta", type=float, default=10.0)
    parser.add_argument("--max-start-gripper-delta", type=float, default=15.0)
    parser.add_argument("--max-tracking-body-error", type=float, default=15.0)
    parser.add_argument("--max-tracking-gripper-error", type=float, default=25.0)
    args = parser.parse_args()
    if not 0.0 < args.speed <= 1.0:
        parser.error("--speed must be in (0, 1]")
    return args


def print_summary(summary: dict[str, object]) -> None:
    print(
        f"frames={summary['frame_count']} duration={summary['duration_s']:.2f}s "
        f"recorded_fps={summary['average_fps']:.1f}"
    )
    print("Maximum consecutive step:")
    max_step = summary["max_step"]
    for key in JOINT_KEYS:
        print(f"  {key:<20} {max_step[key]:7.3f}")


def main() -> None:
    args = parse_args()
    metadata, frames = load_demo(args.trajectory)
    summary = validate_frames(
        frames,
        max_body_step=args.max_body_step,
        max_gripper_step=args.max_gripper_step,
    )
    print_summary(summary)
    if not args.execute:
        print("Dry run complete. No robot was connected or commanded.")
        return

    follower_port = args.follower_port or metadata["follower_port"]
    follower_id = args.follower_id or metadata["follower_id"]
    follower = SO101Follower(
        SO101FollowerConfig(
            port=follower_port,
            id=follower_id,
            max_relative_target=args.max_relative_target,
        )
    )
    connected = False
    try:
        follower.connect()
        connected = True
        current = positions(follower.get_observation())
        first_action = positions(frames[0]["sent_action"])
        assert_pose_near(
            current,
            first_action,
            max_body_delta=args.max_start_body_delta,
            max_gripper_delta=args.max_start_gripper_delta,
            context="Follower versus recorded start pose",
        )

        print("Starting pose is within configured limits.")
        print("Use the identical object placement and clear the surrounding workspace.")
        if input("Type REPLAY to execute the blind trajectory: ").strip() != "REPLAY":
            print("Replay cancelled.")
            return

        start = time.perf_counter()
        for frame in frames:
            target_time = float(frame["t_s"]) / args.speed
            time.sleep(max(0.0, start + target_time - time.perf_counter()))
            target = positions(frame["sent_action"])
            sent = positions(follower.send_action(target))
            measured = positions(follower.get_observation())
            assert_pose_near(
                sent,
                measured,
                max_body_delta=args.max_tracking_body_error,
                max_gripper_delta=args.max_tracking_gripper_error,
                context=f"Tracking error at frame {frame['index']}",
            )

        final_error = pose_delta(frames[-1]["sent_action"], measured)
        print("Replay complete. Final absolute joint errors:")
        for key in JOINT_KEYS:
            print(f"  {key:<20} {final_error[key]:7.3f}")
    finally:
        if connected:
            follower.disconnect()
            print("Follower disconnected; torque disabled.")


if __name__ == "__main__":
    main()
