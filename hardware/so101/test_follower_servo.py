#!/usr/bin/env python3
"""Apply a controlled reversible movement to one SO-101 follower servo."""

from __future__ import annotations

import argparse
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


FOLLOWER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E115987-if00"
FOLLOWER_ID = "my_awesome_follower_arm"
TESTABLE_MOTORS = ("wrist_flex", "wrist_roll", "gripper")


def read_telemetry(bus, motor: str) -> dict[str, float | int]:
    return {
        "position": float(bus.read("Present_Position", motor, num_retry=2)),
        "current": int(bus.read("Present_Current", motor, normalize=False, num_retry=2)),
        "load": int(bus.read("Present_Load", motor, normalize=False, num_retry=2)),
        "voltage": int(bus.read("Present_Voltage", motor, normalize=False, num_retry=2)),
        "temperature": int(bus.read("Present_Temperature", motor, normalize=False, num_retry=2)),
        "status": int(bus.read("Status", motor, normalize=False, num_retry=2)),
        "torque": int(bus.read("Torque_Enable", motor, normalize=False, num_retry=2)),
    }


def show(label: str, values: dict[str, float | int], unit: str) -> None:
    print(
        f"{label:<10} pos={values['position']:8.3f}{unit} "
        f"current={values['current']:4d} load={values['load']:5d} "
        f"voltage={values['voltage'] / 10:.1f}V temp={values['temperature']}C "
        f"status={values['status']} torque={values['torque']}"
    )


def ramp_and_observe(
    bus,
    motor: str,
    start: float,
    target: float,
    rate: float,
    hold_s: float,
    max_error: float,
    unit: str,
) -> None:
    duration = abs(target - start) / rate
    print(f"Ramping {motor}: {start:.3f}{unit} -> {target:.3f}{unit} at {rate:.1f}{unit}/s")
    started = time.monotonic()
    excessive_error_since = None

    while True:
        elapsed = time.monotonic() - started
        fraction = 1.0 if duration == 0 else min(1.0, elapsed / duration)
        goal = start + fraction * (target - start)
        bus.write("Goal_Position", motor, goal, num_retry=2)
        values = read_telemetry(bus, motor)
        show("sample", values, unit)

        if values["status"] != 0:
            raise RuntimeError(f"{motor} reported status {values['status']}")
        if values["voltage"] < 50:
            raise RuntimeError(f"{motor} voltage dropped below 5.0 V")
        if values["temperature"] >= 60:
            raise RuntimeError(f"{motor} reached {values['temperature']} C")

        error = abs(float(values["position"]) - goal)
        if error > max_error:
            excessive_error_since = excessive_error_since or time.monotonic()
            if time.monotonic() - excessive_error_since > 0.75:
                raise RuntimeError(
                    f"{motor} tracking error stayed above {max_error:.1f}{unit}: "
                    f"goal={goal:.2f}, position={values['position']:.2f}"
                )
        else:
            excessive_error_since = None

        if fraction >= 1.0:
            break
        time.sleep(0.05)

    end = time.monotonic() + hold_s
    while time.monotonic() < end:
        values = read_telemetry(bus, motor)
        show("hold", values, unit)
        if abs(float(values["position"]) - target) <= max_error:
            break
        time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("motor", choices=TESTABLE_MOTORS)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--delta", type=float, help="Degrees, or percent for the gripper")
    target.add_argument("--target", type=float, help="Absolute target in degrees, or percent for gripper")
    parser.add_argument("--hold-s", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=15.0, help="Maximum command rate in units per second")
    parser.add_argument("--max-error", type=float, default=8.0, help="Persistent tracking-error abort limit")
    parser.add_argument("--port", default=FOLLOWER_PORT)
    args = parser.parse_args()

    arm = SO101Follower(SO101FollowerConfig(port=args.port, id=FOLLOWER_ID))
    bus = arm.bus
    unit = "%" if args.motor == "gripper" else "deg"
    bus.connect()

    start = float(bus.read("Present_Position", args.motor, num_retry=2))
    target = args.target if args.target is not None else start + args.delta
    if args.rate <= 0:
        bus.disconnect(False)
        raise ValueError("--rate must be positive")
    if args.motor == "gripper" and not 5.0 <= target <= 95.0:
        bus.disconnect(False)
        raise ValueError(f"Refusing gripper target {target:.2f}%; allowed diagnostic range is 5..95%")

    try:
        show("initial", read_telemetry(bus, args.motor), unit)

        # Prevent an old goal register from causing a jump when torque is enabled.
        bus.write("Goal_Position", args.motor, start, num_retry=2)
        bus.enable_torque(args.motor, num_retry=2)
        show("enabled", read_telemetry(bus, args.motor), unit)

        ramp_and_observe(
            bus, args.motor, start, target, args.rate, args.hold_s, args.max_error, unit
        )
        reached = float(bus.read("Present_Position", args.motor, num_retry=2))
        ramp_and_observe(
            bus, args.motor, reached, start, args.rate, args.hold_s, args.max_error, unit
        )
        show("final", read_telemetry(bus, args.motor), unit)
    finally:
        try:
            bus.disable_torque(args.motor, num_retry=5)
            print(f"Torque disabled on {args.motor}")
        except Exception as exc:
            print(f"FAILED TO DISABLE TORQUE: {exc}")
            print("Switch follower power off immediately.")
        bus.disconnect(False)


if __name__ == "__main__":
    main()
