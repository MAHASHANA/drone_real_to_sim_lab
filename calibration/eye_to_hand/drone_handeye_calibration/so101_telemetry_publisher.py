#!/usr/bin/env python3
"""Publish SO-101 localhost telemetry as ROS 2 joint states."""

from __future__ import annotations

import json
import math
import socket
import time
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def validate_packet(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("telemetry payload must be an object")
    if payload.get("schema") != "so101.telemetry.v1":
        raise ValueError("unsupported SO-101 telemetry schema")
    if not isinstance(payload.get("session_id"), str) or not payload["session_id"]:
        raise ValueError("telemetry is missing a session identifier")
    if not isinstance(payload.get("sequence"), int) or payload["sequence"] < 0:
        raise ValueError("telemetry has an invalid sequence")
    if not isinstance(payload.get("mode"), str):
        raise ValueError("telemetry has an invalid mode")
    for arm in ("leader", "follower"):
        values = payload.get(arm)
        if not isinstance(values, dict):
            raise ValueError(f"telemetry is missing {arm} positions")
        for joint in JOINT_NAMES:
            value = values.get(joint)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"invalid {arm} position for {joint}")
    return payload


def ros_positions(calibrated: dict[str, float]) -> list[float]:
    result = [math.radians(float(calibrated[joint])) for joint in JOINT_NAMES[:-1]]
    result.append(float(calibrated["gripper"]) / 100.0 * math.pi)
    return result


class SO101TelemetryPublisher(Node):
    def __init__(self) -> None:
        super().__init__("so101_telemetry_publisher")
        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 15001)
        self.declare_parameter("timeout_s", 0.5)
        self.declare_parameter("poll_rate", 120.0)

        host = str(self.get_parameter("listen_host").value)
        port = int(self.get_parameter("listen_port").value)
        poll_rate = float(self.get_parameter("poll_rate").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind((host, port))
        self.leader_publisher = self.create_publisher(
            JointState, "/leader/joint_states", 10
        )
        self.follower_publisher = self.create_publisher(
            JointState, "/follower/joint_states", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/so101/control_status", 10
        )
        self.last_packet_time: float | None = None
        self.last_session_id: str | None = None
        self.last_sequence: int | None = None
        self.stale_reported = False
        self.create_timer(1.0 / poll_rate, self.poll)
        self.get_logger().info(f"Listening for SO-101 telemetry on udp://{host}:{port}")

    def poll(self) -> None:
        latest = None
        while True:
            try:
                data, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                latest = validate_packet(json.loads(data.decode("utf-8")))
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                self.get_logger().warning(f"Rejected SO-101 telemetry: {error}")

        if latest is not None:
            self.publish_packet(latest)

        if (
            self.last_packet_time is not None
            and time.monotonic() - self.last_packet_time > self.timeout_s
            and not self.stale_reported
        ):
            self.get_logger().error("SO-101 control telemetry is stale")
            self.status_publisher.publish(String(data="stale"))
            self.stale_reported = True

    def publish_packet(self, packet: dict[str, Any]) -> None:
        session_id = str(packet["session_id"])
        sequence = int(packet["sequence"])
        if session_id != self.last_session_id:
            self.last_session_id = session_id
            self.last_sequence = None
        if self.last_sequence is not None and sequence <= self.last_sequence:
            return
        self.last_sequence = sequence
        self.last_packet_time = time.monotonic()
        self.stale_reported = False

        stamp = self.get_clock().now().to_msg()
        for arm, publisher in (
            ("leader", self.leader_publisher),
            ("follower", self.follower_publisher),
        ):
            message = JointState()
            message.header.stamp = stamp
            message.name = list(JOINT_NAMES)
            message.position = ros_positions(packet[arm])
            publisher.publish(message)

        self.status_publisher.publish(String(data=str(packet["mode"])))

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SO101TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
