#!/usr/bin/env python3
"""Quest teleoperation of a Panda with a simulated eye-in-hand RGB-D camera."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

from async_wrist_renderer import (
    PANDA_RENDER_JOINTS,
    AsyncWristRenderer,
    RenderSnapshot,
    create_workcell,
)
from pybullet_utils import add_debug_axes, connect_pybullet
from quest_webxr_server import (
    MotionRecorder,
    SIM_WRIST_HTML,
    SensorFrameState,
    TeleopState,
    serve_https,
)
from quest_webxr_teleop import (
    map_quest_relative_orientation,
    matmul,
    matrix_to_quat,
    quat_to_matrix,
    quest_to_robot,
)
from robot_arm_pybullet import (
    PANDA_EE_LINK,
    command_joints,
    configure_robot_camera,
    load_panda,
    set_gripper,
    solve_ik,
)
from wrist_rgbd_camera import WristCameraConfig, WristRgbdCamera


def demo_target(home, elapsed: float) -> tuple[float, float, float]:
    return (
        home[0] + 0.10 * math.sin(elapsed * 0.65),
        home[1] + 0.08 * math.sin(elapsed * 0.42),
        home[2] + 0.05 * math.sin(elapsed * 0.83),
    )


def run_simulation(
    args: argparse.Namespace,
    teleop_state: TeleopState,
    frame_state: SensorFrameState,
) -> None:
    p, client_id = connect_pybullet(gui=args.gui)
    p.loadURDF("plane.urdf", physicsClientId=client_id)
    add_debug_axes(p, client_id)
    workcell_bodies = create_workcell(p, client_id)
    panda_id = load_panda(p, client_id, (0.0, -0.78, 0.0), math.radians(90.0))
    if args.gui:
        configure_robot_camera(p, client_id, arms=1)

    camera_config = WristCameraConfig(
        width=args.camera_width,
        height=args.camera_height,
        vertical_fov_deg=args.camera_fov,
        near_m=args.camera_near,
        far_m=args.camera_far,
        link_to_camera_xyz=(args.camera_mount_x, args.camera_mount_y, args.camera_mount_z),
    )
    debug_camera = WristRgbdCamera(
        p,
        client_id,
        panda_id,
        PANDA_EE_LINK,
        camera_config,
        show_frustum=args.gui,
    )
    renderer = AsyncWristRenderer(
        camera_config,
        args.camera_fps,
        args.jpeg_quality,
    )
    renderer.start()

    home = (args.home_x, args.home_y, args.home_z)
    home_orientation = tuple(p.getQuaternionFromEuler([math.pi, 0.0, math.pi / 2.0]))
    home_rotation = quat_to_matrix(home_orientation)
    quest_origin = None
    quest_orientation_origin = None
    rest_pose = None
    started = time.monotonic()
    next_snapshot_time = started
    snapshot_period = 1.0 / max(30.0, args.camera_fps * 2.0)
    snapshot_sequence = 0
    camera_frames = 0
    control_steps = 0
    last_render_ms = 0.0
    last_frame_latency_ms = 0.0
    render_error = None
    last_report = started
    target = home

    print(
        "Wrist camera mount T_EE_C:",
        f"xyz={camera_config.link_to_camera_xyz}",
        f"quat={camera_config.link_to_camera_quat}",
    )
    try:
        while args.run_seconds <= 0 or time.monotonic() - started < args.run_seconds:
            now = time.monotonic()
            snapshot = teleop_state.snapshot()
            quest_position = snapshot["right_position"]
            desired_orientation = home_orientation

            if quest_position is not None:
                if quest_origin is None:
                    quest_origin = quest_position
                    print(f"Quest controller origin: {tuple(round(v, 3) for v in quest_origin)}")
                target = quest_to_robot(
                    quest_position,
                    quest_origin,
                    home,
                    args.scale,
                )
                quest_orientation = snapshot["right_orientation"]
                if (
                    args.orientation_mode == "controller"
                    and quest_orientation is not None
                    and quest_orientation_origin is None
                ):
                    quest_orientation_origin = quest_orientation
                if (
                    args.orientation_mode == "controller"
                    and quest_orientation is not None
                    and quest_orientation_origin is not None
                ):
                    relative_rotation = map_quest_relative_orientation(
                        quest_orientation,
                        quest_orientation_origin,
                    )
                    desired_orientation = matrix_to_quat(
                        matmul(home_rotation, relative_rotation)
                    )
            elif args.demo_motion:
                target = demo_target(home, now - started)

            rest_pose = solve_ik(
                p,
                client_id,
                panda_id,
                target,
                desired_orientation,
                rest_pose=rest_pose,
            )
            command_joints(
                p,
                client_id,
                panda_id,
                rest_pose,
                max_velocity=args.max_velocity,
            )
            gripper_closed = max(snapshot["grip_value"], snapshot["trigger_value"]) > 0.25
            set_gripper(
                p,
                client_id,
                panda_id,
                open_width=0.0 if gripper_closed else 0.04,
            )

            p.stepSimulation(physicsClientId=client_id)
            control_steps += 1

            if now >= next_snapshot_time:
                joint_states = p.getJointStates(
                    panda_id,
                    PANDA_RENDER_JOINTS,
                    physicsClientId=client_id,
                )
                body_poses = tuple(
                    p.getBasePositionAndOrientation(
                        body_id,
                        physicsClientId=client_id,
                    )
                    for body_id in workcell_bodies
                )
                snapshot_sequence += 1
                renderer.publish(
                    RenderSnapshot(
                        sequence=snapshot_sequence,
                        source_time=now,
                        joint_positions=tuple(state[0] for state in joint_states),
                        body_poses=body_poses,
                    )
                )
                next_snapshot_time = now + snapshot_period
                if args.gui:
                    debug_camera.update_debug_frustum()

            for rendered in renderer.receive():
                if rendered["kind"] == "error":
                    render_error = rendered["traceback"]
                    continue
                frame_state.update_color(
                    rendered["color_jpeg"],
                    rendered["width"],
                    rendered["height"],
                )
                frame_state.update_depth(
                    rendered["depth_jpeg"],
                    rendered["width"],
                    rendered["height"],
                )
                camera_frames += 1
                last_render_ms = rendered["render_ms"]
                last_frame_latency_ms = (
                    time.monotonic() - rendered["source_time"]
                ) * 1000.0

            if now - last_report >= 5.0:
                report_period = now - last_report
                ee_state = p.getLinkState(
                    panda_id,
                    PANDA_EE_LINK,
                    computeForwardKinematics=True,
                    physicsClientId=client_id,
                )
                print(
                    f"control={control_steps / report_period:.1f} Hz",
                    f"camera={camera_frames / report_period:.1f} FPS",
                    f"render={last_render_ms:.1f} ms",
                    f"frame_latency={last_frame_latency_ms:.1f} ms",
                    f"quest_packets={snapshot['packets']}",
                    f"target={tuple(round(v, 3) for v in target)}",
                    f"ee={tuple(round(v, 3) for v in ee_state[4])}",
                )
                camera_frames = 0
                control_steps = 0
                last_report = now
                if render_error is not None:
                    print("Camera render process failed:\n" + render_error)
                    render_error = None
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        renderer.close()
        p.disconnect(physicsClientId=client_id)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--advertise-ip", default="")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--run-seconds", type=float, default=0.0)
    parser.add_argument("--demo-motion", action="store_true")
    parser.add_argument("--scale", type=float, default=0.65)
    parser.add_argument("--home-x", type=float, default=0.0)
    parser.add_argument("--home-y", type=float, default=-0.42)
    parser.add_argument("--home-z", type=float, default=0.30)
    parser.add_argument("--max-velocity", type=float, default=1.0)
    parser.add_argument("--orientation-mode", choices=["fixed", "controller"], default="fixed")
    parser.add_argument("--camera-width", type=int, default=320)
    parser.add_argument("--camera-height", type=int, default=180)
    parser.add_argument("--camera-fps", type=float, default=10.0)
    parser.add_argument("--camera-fov", type=float, default=60.0)
    parser.add_argument("--camera-near", type=float, default=0.03)
    parser.add_argument("--camera-far", type=float, default=2.0)
    parser.add_argument("--camera-mount-x", type=float, default=0.0)
    parser.add_argument("--camera-mount-y", type=float, default=0.0)
    parser.add_argument("--camera-mount-z", type=float, default=-0.08)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--out", default="captures")
    parser.add_argument("--motion-threshold-m", type=float, default=0.004)
    parser.add_argument("--idle-split-s", type=float, default=0.35)
    args = parser.parse_args()
    if args.camera_fps <= 0:
        parser.error("--camera-fps must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")
    return args


def main() -> None:
    args = parse_args()
    recorder = (
        MotionRecorder(Path(args.out), args.motion_threshold_m, args.idle_split_s)
        if args.record
        else None
    )
    teleop_state = TeleopState(recorder=recorder)
    frame_state = SensorFrameState()
    server = serve_https(
        args.host,
        args.port,
        args.advertise_ip,
        teleop_state,
        sensor_state=frame_state,
        root_html=SIM_WRIST_HTML,
    )
    try:
        run_simulation(args, teleop_state, frame_state)
    finally:
        server.shutdown()
        server.server_close()
        if recorder is not None:
            recorder.close()


if __name__ == "__main__":
    main()
