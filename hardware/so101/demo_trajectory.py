#!/usr/bin/env python3
"""Shared JSONL handling and safety validation for SO-101 blind demonstrations."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
JOINT_KEYS = tuple(f"{name}.pos" for name in JOINT_NAMES)
BODY_KEYS = JOINT_KEYS[:-1]
GRIPPER_KEY = JOINT_KEYS[-1]
SCHEMA = "so101_blind_demo/v1"


def positions(values: dict[str, Any]) -> dict[str, float]:
    missing = [key for key in JOINT_KEYS if key not in values]
    if missing:
        raise ValueError(f"Missing joint positions: {', '.join(missing)}")
    return {key: float(values[key]) for key in JOINT_KEYS}


def pose_delta(
    first: dict[str, Any],
    second: dict[str, Any],
) -> dict[str, float]:
    first_pos = positions(first)
    second_pos = positions(second)
    return {key: abs(second_pos[key] - first_pos[key]) for key in JOINT_KEYS}


def assert_pose_near(
    first: dict[str, Any],
    second: dict[str, Any],
    max_body_delta: float,
    max_gripper_delta: float,
    context: str,
) -> None:
    delta = pose_delta(first, second)
    violations = [
        f"{key}={value:.2f}"
        for key, value in delta.items()
        if (
            (key in BODY_KEYS and value > max_body_delta)
            or (key == GRIPPER_KEY and value > max_gripper_delta)
        )
    ]
    if violations:
        raise RuntimeError(
            f"{context} exceeds safety limits: {', '.join(violations)}"
        )


def write_jsonl_record(handle: Any, record: dict[str, Any]) -> None:
    handle.write(json.dumps(record, separators=(",", ":")) + "\n")


def load_demo(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata: dict[str, Any] | None = None
    frames: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            record = json.loads(line)
            record_type = record.get("type")
            if record_type == "metadata":
                if metadata is not None:
                    raise ValueError("Trajectory contains multiple metadata records")
                metadata = record
            elif record_type == "frame":
                frames.append(record)
            else:
                raise ValueError(
                    f"Unknown record type on line {line_number}: {record_type!r}"
                )
    if metadata is None:
        raise ValueError("Trajectory does not contain metadata")
    if metadata.get("schema") != SCHEMA:
        raise ValueError(f"Unsupported trajectory schema: {metadata.get('schema')!r}")
    if not frames:
        raise ValueError("Trajectory does not contain frames")
    return metadata, frames


def validate_frames(
    frames: list[dict[str, Any]],
    max_body_step: float,
    max_gripper_step: float,
) -> dict[str, Any]:
    previous_time = -1.0
    previous_action: dict[str, float] | None = None
    max_step = {key: 0.0 for key in JOINT_KEYS}
    joint_min = {key: float("inf") for key in JOINT_KEYS}
    joint_max = {key: float("-inf") for key in JOINT_KEYS}

    for expected_index, frame in enumerate(frames):
        if int(frame.get("index", -1)) != expected_index:
            raise ValueError(f"Unexpected frame index at position {expected_index}")
        frame_time = float(frame["t_s"])
        if frame_time < previous_time:
            raise ValueError(f"Frame time moved backward at index {expected_index}")
        action = positions(frame["sent_action"])
        for key, value in action.items():
            joint_min[key] = min(joint_min[key], value)
            joint_max[key] = max(joint_max[key], value)
        if previous_action is not None:
            delta = pose_delta(previous_action, action)
            for key, value in delta.items():
                max_step[key] = max(max_step[key], value)
        previous_action = action
        previous_time = frame_time

    violations = [
        f"{key}={value:.2f}"
        for key, value in max_step.items()
        if (
            (key in BODY_KEYS and value > max_body_step)
            or (key == GRIPPER_KEY and value > max_gripper_step)
        )
    ]
    if violations:
        raise RuntimeError(
            "Recorded trajectory has unsafe consecutive steps: "
            + ", ".join(violations)
        )

    duration = float(frames[-1]["t_s"])
    return {
        "frame_count": len(frames),
        "duration_s": duration,
        "average_fps": (len(frames) - 1) / duration if duration > 0.0 else 0.0,
        "max_step": max_step,
        "joint_min": joint_min,
        "joint_max": joint_max,
    }
