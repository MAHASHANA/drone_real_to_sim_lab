import math
import unittest

from drone_handeye_calibration.so101_telemetry_publisher import (
    ros_positions,
    validate_packet,
)


class SO101TelemetryPublisherTest(unittest.TestCase):
    def setUp(self):
        self.joints = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 90.0,
            "elbow_flex": -90.0,
            "wrist_flex": 45.0,
            "wrist_roll": -45.0,
            "gripper": 50.0,
        }

    def test_converts_degrees_and_gripper_percent_to_ros_radians(self):
        result = ros_positions(self.joints)
        self.assertAlmostEqual(result[1], math.pi / 2)
        self.assertAlmostEqual(result[2], -math.pi / 2)
        self.assertAlmostEqual(result[5], math.pi / 2)

    def test_validates_complete_packet(self):
        packet = {
            "schema": "so101.telemetry.v1",
            "session_id": "test-session",
            "sequence": 1,
            "mode": "mirroring",
            "leader": self.joints,
            "follower": self.joints,
        }
        self.assertIs(validate_packet(packet), packet)

    def test_rejects_wrong_schema(self):
        with self.assertRaises(ValueError):
            validate_packet({"schema": "unknown"})


if __name__ == "__main__":
    unittest.main()
