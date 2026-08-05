import json
import unittest

from leader_follower_bridge import (
    make_packet,
    positions,
)


class LeaderFollowerBridgeTest(unittest.TestCase):
    def setUp(self):
        self.follower = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 10.0,
            "elbow_flex": 20.0,
            "wrist_flex": 30.0,
            "wrist_roll": 40.0,
            "gripper": 50.0,
        }
        self.leader = {name: value + 10.0 for name, value in self.follower.items()}

    def test_positions_rejects_incomplete_action(self):
        with self.assertRaises(KeyError):
            positions({"shoulder_pan.pos": 0.0})

    def test_packet_is_versioned_and_complete(self):
        packet = json.loads(
            make_packet(
                "test-session",
                7,
                "mirroring",
                self.leader,
                self.follower,
                self.leader,
                60.0,
            )
        )
        self.assertEqual(packet["schema"], "so101.telemetry.v1")
        self.assertEqual(packet["session_id"], "test-session")
        self.assertEqual(packet["sequence"], 7)
        self.assertEqual(packet["leader"], self.leader)
        self.assertEqual(packet["follower"], self.follower)


if __name__ == "__main__":
    unittest.main()
