#!/usr/bin/env python3
"""Load the generated Mark4 frame URDF in PyBullet."""

from __future__ import annotations

import argparse
import time

from pybullet_utils import URDF_PATH, add_debug_axes, connect_pybullet, ensure_generated


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Open the PyBullet GUI.")
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--fixed", action="store_true", help="Keep the frame fixed in space.")
    args = parser.parse_args()

    ensure_generated()
    p, client_id = connect_pybullet(gui=args.gui)
    p.loadURDF("plane.urdf", physicsClientId=client_id)
    add_debug_axes(p, client_id)

    frame_id = p.loadURDF(
        str(URDF_PATH),
        basePosition=[0.0, 0.0, 0.08],
        useFixedBase=args.fixed,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
        physicsClientId=client_id,
    )

    print(f"Loaded frame URDF: {URDF_PATH}")
    for i in range(args.steps):
        p.stepSimulation(physicsClientId=client_id)
        if args.gui:
            time.sleep(1.0 / 240.0)
        if i % 60 == 0 or i == args.steps - 1:
            pos, orn = p.getBasePositionAndOrientation(frame_id, physicsClientId=client_id)
            print(f"{i:04d}: pos={tuple(round(v, 5) for v in pos)} orn={tuple(round(v, 5) for v in orn)}")

    p.disconnect(physicsClientId=client_id)


if __name__ == "__main__":
    main()

