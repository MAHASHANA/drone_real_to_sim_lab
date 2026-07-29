#!/usr/bin/env python3

import unittest

import numpy as np

from rgbd_point_inspector import (
    Intrinsics,
    deproject_pixels,
    fit_plane_ransac,
)


class RgbdGeometryTest(unittest.TestCase):
    def test_deproject_pixels(self) -> None:
        intrinsics = Intrinsics(
            width=640,
            height=480,
            fx=500.0,
            fy=500.0,
            cx=320.0,
            cy=240.0,
            frame_id="camera",
        )
        points = deproject_pixels(
            np.array([320.0, 420.0]),
            np.array([240.0, 190.0]),
            np.array([1.0, 2.0]),
            intrinsics,
        )
        np.testing.assert_allclose(
            points,
            np.array([[0.0, 0.0, 1.0], [0.4, -0.2, 2.0]]),
        )

    def test_plane_fit_rejects_object_outliers(self) -> None:
        rng = np.random.default_rng(11)
        x, y = np.meshgrid(
            np.linspace(-0.35, 0.35, 35),
            np.linspace(-0.25, 0.25, 25),
        )
        z = 0.7 + 0.08 * x - 0.04 * y
        table = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
        table[:, 2] += rng.normal(0.0, 0.001, table.shape[0])

        objects = table[rng.choice(table.shape[0], 120, replace=False)].copy()
        objects[:, 2] -= rng.uniform(0.025, 0.09, objects.shape[0])
        estimate = fit_plane_ransac(
            np.vstack((table, objects)),
            threshold_m=0.006,
            iterations=100,
        )

        self.assertIsNotNone(estimate)
        assert estimate is not None
        self.assertGreater(estimate.inlier_fraction, 0.8)
        self.assertLess(estimate.rms_error_m, 0.002)

        table_height = table @ estimate.normal + estimate.offset
        object_height = objects @ estimate.normal + estimate.offset
        self.assertLess(abs(float(np.median(table_height))), 0.002)
        self.assertGreater(float(np.median(object_height)), 0.02)


if __name__ == "__main__":
    unittest.main()
