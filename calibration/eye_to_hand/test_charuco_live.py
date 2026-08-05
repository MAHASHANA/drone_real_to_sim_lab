#!/usr/bin/env python3

import unittest

import cv2
import numpy as np

from drone_handeye_calibration.charuco import (
    detect_board,
    make_board,
    rotation_matrix_to_quaternion,
)


class CharucoDetectionTest(unittest.TestCase):
    def test_generated_board_detects_all_internal_corners(self) -> None:
        board, detector = make_board(7, 5, 0.024, 0.019)
        image = board.generateImage((1400, 1000), marginSize=60, borderBits=1)
        image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        result = detect_board(image_bgr, board, detector, camera=None)

        self.assertEqual(result.corner_count, 24)
        self.assertEqual(sorted(result.marker_ids), list(range(17)))
        self.assertGreater(result.coverage, 0.3)

    def test_rotation_matrix_to_xyzw_quaternion(self) -> None:
        rotation, _ = cv2.Rodrigues(
            np.asarray([0.0, 0.0, np.pi / 2.0], dtype=np.float64)
        )

        quaternion = rotation_matrix_to_quaternion(rotation)

        np.testing.assert_allclose(
            np.abs(quaternion),
            [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)],
            atol=1e-8,
        )

    def test_quaternion_is_normalized_for_half_turn(self) -> None:
        rotation = np.diag([1.0, -1.0, -1.0])

        quaternion = rotation_matrix_to_quaternion(rotation)

        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0)
        np.testing.assert_allclose(np.abs(quaternion), [1.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
