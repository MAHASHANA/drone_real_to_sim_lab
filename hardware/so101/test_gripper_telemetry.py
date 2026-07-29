#!/usr/bin/env python3

import unittest

from gripper_telemetry import read_gripper_telemetry, set_runtime_torque_limit


class FakeBus:
    def __init__(self) -> None:
        self.values = {
            "Max_Torque_Limit": 500,
            "Torque_Limit": 500,
            "Present_Current": 100,
            "Present_Load": -200,
            "Present_Voltage": 74,
            "Present_Temperature": 31,
            "Moving": 1,
        }

    def read(
        self,
        data_name: str,
        motor: str,
        *,
        normalize: bool,
        num_retry: int,
    ) -> int:
        self.assert_gripper(motor)
        return self.values[data_name]

    def write(
        self,
        data_name: str,
        motor: str,
        value: int,
        *,
        num_retry: int,
    ) -> None:
        self.assert_gripper(motor)
        self.values[data_name] = value

    @staticmethod
    def assert_gripper(motor: str) -> None:
        if motor != "gripper":
            raise AssertionError(f"Unexpected motor: {motor}")


class GripperTelemetryTest(unittest.TestCase):
    def test_sets_and_verifies_runtime_limit(self) -> None:
        bus = FakeBus()
        result = set_runtime_torque_limit(bus, 200)
        self.assertEqual(result["previous_raw"], 500)
        self.assertEqual(result["applied_raw"], 200)
        self.assertEqual(bus.values["Torque_Limit"], 200)

    def test_rejects_limit_above_configured_maximum(self) -> None:
        with self.assertRaises(ValueError):
            set_runtime_torque_limit(FakeBus(), 501)

    def test_reads_gripper_telemetry(self) -> None:
        result = read_gripper_telemetry(
            FakeBus(),
            {"gripper.pos": 60.0},
            {"gripper.pos": 55.0},
        )
        self.assertAlmostEqual(result["present_current_a_est"], 0.65)
        self.assertAlmostEqual(result["present_load_percent_est"], 20.0)
        self.assertAlmostEqual(result["present_voltage_v"], 7.4)
        self.assertAlmostEqual(result["position_error_pct"], 5.0)
        self.assertTrue(result["moving"])


if __name__ == "__main__":
    unittest.main()
