#!/usr/bin/env python3

import json
import tempfile
import unittest
from pathlib import Path

from demo_trajectory import JOINT_KEYS, SCHEMA, load_demo, validate_frames


def pose(value: float) -> dict[str, float]:
    return {key: value for key in JOINT_KEYS}


class DemoTrajectoryTest(unittest.TestCase):
    def test_load_and_validate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "demo.jsonl"
            records = [
                {"type": "metadata", "schema": SCHEMA},
                {
                    "type": "frame",
                    "index": 0,
                    "t_s": 0.0,
                    "sent_action": pose(0.0),
                },
                {
                    "type": "frame",
                    "index": 1,
                    "t_s": 0.1,
                    "sent_action": pose(1.0),
                },
            ]
            path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )
            metadata, frames = load_demo(path)
            summary = validate_frames(frames, 2.0, 2.0)

        self.assertEqual(metadata["schema"], SCHEMA)
        self.assertEqual(summary["frame_count"], 2)
        self.assertAlmostEqual(summary["average_fps"], 10.0)

    def test_rejects_large_step(self) -> None:
        frames = [
            {
                "type": "frame",
                "index": 0,
                "t_s": 0.0,
                "sent_action": pose(0.0),
            },
            {
                "type": "frame",
                "index": 1,
                "t_s": 0.1,
                "sent_action": pose(20.0),
            },
        ]
        with self.assertRaises(RuntimeError):
            validate_frames(frames, 5.0, 10.0)


if __name__ == "__main__":
    unittest.main()
