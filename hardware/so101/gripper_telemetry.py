#!/usr/bin/env python3
"""SO-101 gripper torque-limit and telemetry helpers."""

from __future__ import annotations

from typing import Any


GRIPPER = "gripper"
CURRENT_AMPS_PER_RAW = 0.0065
VOLTAGE_VOLTS_PER_RAW = 0.1


def set_runtime_torque_limit(bus: Any, requested_raw: int) -> dict[str, int]:
    """Set and verify the volatile gripper torque cap."""
    configured_max = int(
        bus.read("Max_Torque_Limit", GRIPPER, normalize=False, num_retry=2)
    )
    if not 1 <= requested_raw <= configured_max:
        raise ValueError(
            "Gripper torque limit must be between 1 and the configured maximum "
            f"({configured_max}); received {requested_raw}."
        )

    previous = int(bus.read("Torque_Limit", GRIPPER, normalize=False, num_retry=2))
    bus.write("Torque_Limit", GRIPPER, requested_raw, num_retry=2)
    applied = int(bus.read("Torque_Limit", GRIPPER, normalize=False, num_retry=2))
    if applied != requested_raw:
        raise RuntimeError(
            f"Gripper torque-limit readback mismatch: requested={requested_raw}, "
            f"applied={applied}"
        )
    return {
        "requested_raw": requested_raw,
        "applied_raw": applied,
        "previous_raw": previous,
        "configured_max_raw": configured_max,
    }


def read_gripper_telemetry(
    bus: Any,
    sent_action: dict[str, float],
    observation: dict[str, float],
) -> dict[str, float | int | bool]:
    """Read raw servo feedback and attach explicitly labeled estimates."""
    current_raw = int(
        bus.read("Present_Current", GRIPPER, normalize=False, num_retry=1)
    )
    load_raw = int(bus.read("Present_Load", GRIPPER, normalize=False, num_retry=1))
    voltage_raw = int(
        bus.read("Present_Voltage", GRIPPER, normalize=False, num_retry=1)
    )
    temperature_raw = int(
        bus.read("Present_Temperature", GRIPPER, normalize=False, num_retry=1)
    )
    moving_raw = int(bus.read("Moving", GRIPPER, normalize=False, num_retry=1))
    goal = float(sent_action["gripper.pos"])
    measured = float(observation["gripper.pos"])

    return {
        "present_current_raw": current_raw,
        "present_current_a_est": abs(current_raw) * CURRENT_AMPS_PER_RAW,
        "present_load_raw": load_raw,
        "present_load_percent_est": abs(load_raw) / 10.0,
        "present_voltage_raw": voltage_raw,
        "present_voltage_v": voltage_raw * VOLTAGE_VOLTS_PER_RAW,
        "present_temperature_c": temperature_raw,
        "moving": bool(moving_raw),
        "goal_position_pct": goal,
        "measured_position_pct": measured,
        "position_error_pct": goal - measured,
    }
