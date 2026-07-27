#!/usr/bin/env python3
"""Quest teleoperation of a Panda with a simulated eye-in-hand RGB-D camera."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

from async_wrist_renderer import (
    PANDA_RENDER_JOINTS,
    WORKCELL_BODY_NAMES,
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
    create_grasp_constraint,
    load_panda,
    set_gripper,
    solve_ik,
)
from wrist_rgbd_camera import WristCameraConfig, WristRgbdCamera


PANDA_FINGER_LINKS = frozenset((9, 10))


@dataclass
class AssistedPickState:
    phase: str = "idle"
    body_id: int | None = None
    constraint_id: int | None = None
    target: tuple[float, float, float] | None = None
    pregrasp: tuple[float, float, float] | None = None
    grasp: tuple[float, float, float] | None = None
    phase_started: float = 0.0
    contact_started: float = 0.0


def apply_deadzone(value: float, deadzone: float) -> float:
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(min(1.0, scaled), value)


def clamp_workspace(target) -> tuple[float, float, float]:
    return (
        max(-0.35, min(0.35, target[0])),
        max(-0.72, min(0.02, target[1])),
        max(0.05, min(0.65, target[2])),
    )


def distance(a, b) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def move_toward(current, goal, max_distance: float) -> tuple[float, float, float]:
    separation = distance(current, goal)
    if separation <= max_distance or separation <= 1e-9:
        return tuple(float(v) for v in goal)
    scale = max_distance / separation
    return tuple(
        float(current[i]) + (float(goal[i]) - float(current[i])) * scale
        for i in range(3)
    )


def nearest_object(
    p,
    client_id: int,
    object_bodies: list[int],
    position,
    maximum_distance: float,
) -> tuple[int | None, float]:
    nearest_body = None
    nearest_distance = float("inf")
    for body_id in object_bodies:
        body_position, _ = p.getBasePositionAndOrientation(
            body_id,
            physicsClientId=client_id,
        )
        candidate_distance = distance(position, body_position)
        if candidate_distance < nearest_distance:
            nearest_body = body_id
            nearest_distance = candidate_distance
    if nearest_distance > maximum_distance:
        return None, nearest_distance
    return nearest_body, nearest_distance


def grasp_poses(
    p,
    client_id: int,
    body_id: int,
    approach_height: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    body_position, _ = p.getBasePositionAndOrientation(
        body_id,
        physicsClientId=client_id,
    )
    _, aabb_max = p.getAABB(body_id, physicsClientId=client_id)
    grasp = clamp_workspace(
        (
            body_position[0],
            body_position[1],
            aabb_max[2],
        )
    )
    pregrasp = clamp_workspace(
        (
            grasp[0],
            grasp[1],
            grasp[2] + approach_height,
        )
    )
    return pregrasp, grasp


def gripper_contact_state(
    p,
    client_id: int,
    panda_id: int,
    object_bodies: list[int],
    force_threshold: float,
) -> tuple[int | None, int | None]:
    touching_body = None
    two_finger_body = None
    for body_id in object_bodies:
        finger_links = {
            int(contact[3])
            for contact in p.getContactPoints(
                bodyA=panda_id,
                bodyB=body_id,
                physicsClientId=client_id,
            )
            if int(contact[3]) in PANDA_FINGER_LINKS
            and float(contact[9]) >= force_threshold
        }
        if finger_links and touching_body is None:
            touching_body = body_id
        if PANDA_FINGER_LINKS.issubset(finger_links):
            two_finger_body = body_id
            break
    return touching_body, two_finger_body


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
    orientation_anchor_rotation = home_rotation
    motion_anchor = home
    quest_origin = None
    quest_orientation_origin = None
    fine_offset = [0.0, 0.0, 0.0]
    thumbstick_was_pressed = False
    rest_pose = None
    started = time.monotonic()
    previous_control_time = started
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
    last_touch_body = None
    grasp_candidate_body = None
    grasp_candidate_since = 0.0
    confirmed_grasp_body = None
    last_haptic_time = 0.0
    body_names = dict(zip(workcell_bodies, WORKCELL_BODY_NAMES))
    assisted_pick = AssistedPickState()
    primary_was_pressed = False
    secondary_was_pressed = False

    print(
        "Wrist camera mount T_EE_C:",
        f"xyz={camera_config.link_to_camera_xyz}",
        f"quat={camera_config.link_to_camera_quat}",
    )
    try:
        while args.run_seconds <= 0 or time.monotonic() - started < args.run_seconds:
            now = time.monotonic()
            control_dt = min(0.05, max(0.0, now - previous_control_time))
            previous_control_time = now
            snapshot = teleop_state.snapshot()
            quest_position = snapshot["right_position"]
            desired_orientation = home_orientation
            packet_fresh = (
                snapshot["last_time"] > 0.0
                and time.time() - snapshot["last_time"] <= args.controller_timeout
            )
            primary_pressed = packet_fresh and snapshot["primary_pressed"]
            secondary_pressed = packet_fresh and snapshot["secondary_pressed"]
            primary_rising = primary_pressed and not primary_was_pressed
            secondary_rising = secondary_pressed and not secondary_was_pressed
            primary_was_pressed = primary_pressed
            secondary_was_pressed = secondary_pressed

            if quest_position is not None:
                if quest_origin is None:
                    quest_origin = quest_position
                    print(f"Quest controller origin: {tuple(round(v, 3) for v in quest_origin)}")
                base_target = quest_to_robot(
                    quest_position,
                    quest_origin,
                    motion_anchor,
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
                        matmul(orientation_anchor_rotation, relative_rotation)
                    )
                thumbstick_pressed = (
                    packet_fresh and snapshot["thumbstick_pressed"]
                )
                if thumbstick_pressed and not thumbstick_was_pressed:
                    motion_anchor = clamp_workspace(
                        tuple(base_target[i] + fine_offset[i] for i in range(3))
                    )
                    quest_origin = quest_position
                    fine_offset = [0.0, 0.0, 0.0]
                    base_target = motion_anchor
                    if quest_orientation is not None:
                        orientation_anchor_rotation = quat_to_matrix(desired_orientation)
                        quest_orientation_origin = quest_orientation
                    teleop_state.request_haptic(0.35, 35, "controller-recentered")
                    print(
                        "Quest controller recentered:",
                        tuple(round(v, 3) for v in motion_anchor),
                    )
                thumbstick_was_pressed = thumbstick_pressed

                stick_x = apply_deadzone(
                    snapshot["thumbstick_x"] if packet_fresh else 0.0,
                    args.joystick_deadzone,
                )
                stick_y = apply_deadzone(
                    snapshot["thumbstick_y"] if packet_fresh else 0.0,
                    args.joystick_deadzone,
                )
                desired_rotation = quat_to_matrix(desired_orientation)
                tool_approach_axis = [desired_rotation[i][2] for i in range(3)]
                for axis in range(3):
                    fine_offset[axis] += (
                        (1.0 if axis == 0 else 0.0) * stick_x
                        + tool_approach_axis[axis] * -stick_y
                    ) * args.joystick_speed * control_dt
                    fine_offset[axis] = max(
                        -args.joystick_max_offset,
                        min(args.joystick_max_offset, fine_offset[axis]),
                    )
                target = clamp_workspace(
                    tuple(base_target[i] + fine_offset[i] for i in range(3))
                )
            elif args.demo_motion:
                target = demo_target(home, now - started)
            else:
                thumbstick_was_pressed = False

            if secondary_rising and assisted_pick.phase != "idle":
                if assisted_pick.constraint_id is not None:
                    p.removeConstraint(
                        assisted_pick.constraint_id,
                        physicsClientId=client_id,
                    )
                released_name = body_names.get(assisted_pick.body_id, "selection")
                assisted_pick = AssistedPickState()
                confirmed_grasp_body = None
                grasp_candidate_body = None
                grasp_candidate_since = 0.0
                motion_anchor = target
                if quest_position is not None:
                    quest_origin = quest_position
                fine_offset = [0.0, 0.0, 0.0]
                teleop_state.request_haptic(0.4, 70, f"assist-released:{released_name}")
                print("Assisted pick released:", released_name)

            if primary_rising:
                if assisted_pick.phase != "idle":
                    teleop_state.request_haptic(0.18, 80, "assist-busy")
                else:
                    ee_position = p.getLinkState(
                        panda_id,
                        PANDA_EE_LINK,
                        computeForwardKinematics=True,
                        physicsClientId=client_id,
                    )[4]
                    selected_body, selection_distance = nearest_object(
                        p,
                        client_id,
                        workcell_bodies[1:],
                        ee_position,
                        args.assist_radius,
                    )
                    if selected_body is None:
                        teleop_state.request_haptic(0.18, 130, "assist-no-object-nearby")
                        print(
                            "Assisted pick rejected:",
                            f"nearest object is {selection_distance:.3f} m away",
                        )
                    else:
                        pregrasp, grasp = grasp_poses(
                            p,
                            client_id,
                            selected_body,
                            args.assist_approach_height,
                        )
                        assisted_pick = AssistedPickState(
                            phase="approach",
                            body_id=selected_body,
                            target=tuple(float(v) for v in ee_position),
                            pregrasp=pregrasp,
                            grasp=grasp,
                            phase_started=now,
                        )
                        target = assisted_pick.target
                        teleop_state.request_haptic(
                            0.42,
                            45,
                            f"assist-selected:{body_names.get(selected_body, selected_body)}",
                        )
                        print(
                            "Assisted pick selected:",
                            body_names.get(selected_body, selected_body),
                            f"distance={selection_distance:.3f} m",
                        )

            if assisted_pick.phase == "approach":
                assisted_pick.target = move_toward(
                    assisted_pick.target,
                    assisted_pick.pregrasp,
                    args.assist_speed * control_dt,
                )
                target = assisted_pick.target
                if distance(target, assisted_pick.pregrasp) <= args.assist_tolerance:
                    assisted_pick.phase = "descend"
                    assisted_pick.phase_started = now
            elif assisted_pick.phase == "descend":
                assisted_pick.target = move_toward(
                    assisted_pick.target,
                    assisted_pick.grasp,
                    args.assist_descend_speed * control_dt,
                )
                target = assisted_pick.target
                if distance(target, assisted_pick.grasp) <= args.assist_tolerance:
                    assisted_pick.phase = "close"
                    assisted_pick.phase_started = now
                    assisted_pick.contact_started = 0.0
            elif assisted_pick.phase == "close":
                target = assisted_pick.grasp
            elif assisted_pick.phase == "lift":
                assisted_pick.target = move_toward(
                    assisted_pick.target,
                    assisted_pick.pregrasp,
                    args.assist_speed * control_dt,
                )
                target = assisted_pick.target
                if distance(target, assisted_pick.pregrasp) <= args.assist_tolerance:
                    assisted_pick.phase = "held"
                    assisted_pick.phase_started = now
                    motion_anchor = target
                    if quest_position is not None:
                        quest_origin = quest_position
                    fine_offset = [0.0, 0.0, 0.0]
                    teleop_state.request_haptic(
                        0.62,
                        65,
                        f"assist-lift-complete:{body_names.get(assisted_pick.body_id, assisted_pick.body_id)}",
                    )
                    print(
                        "Assisted pick complete:",
                        body_names.get(assisted_pick.body_id, assisted_pick.body_id),
                    )

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
            manual_gripper_closed = (
                packet_fresh
                and max(snapshot["grip_value"], snapshot["trigger_value"]) > 0.25
            )
            if assisted_pick.phase in ("approach", "descend"):
                gripper_closed = False
            elif assisted_pick.phase in ("close", "lift", "held"):
                gripper_closed = True
            else:
                gripper_closed = manual_gripper_closed
            set_gripper(
                p,
                client_id,
                panda_id,
                open_width=0.0 if gripper_closed else 0.04,
            )

            p.stepSimulation(physicsClientId=client_id)
            control_steps += 1

            touching_body, two_finger_body = gripper_contact_state(
                p,
                client_id,
                panda_id,
                workcell_bodies[1:],
                args.contact_force_threshold,
            )
            if (
                touching_body is not None
                and touching_body != last_touch_body
                and now - last_haptic_time >= args.haptic_interval
            ):
                teleop_state.request_haptic(
                    0.28,
                    28,
                    f"object-contact:{body_names.get(touching_body, touching_body)}",
                )
                last_haptic_time = now
            last_touch_body = touching_body

            if gripper_closed and two_finger_body is not None:
                if grasp_candidate_body != two_finger_body:
                    grasp_candidate_body = two_finger_body
                    grasp_candidate_since = now
                elif (
                    confirmed_grasp_body != two_finger_body
                    and now - grasp_candidate_since >= args.grasp_confirm_time
                ):
                    confirmed_grasp_body = two_finger_body
                    teleop_state.request_haptic(
                        0.9,
                        90,
                        f"grasp-confirmed:{body_names.get(two_finger_body, two_finger_body)}",
                    )
                    last_haptic_time = now
                    print(
                        "Grasp confirmed:",
                        body_names.get(two_finger_body, two_finger_body),
                    )
            else:
                grasp_candidate_body = None
                grasp_candidate_since = 0.0
                if confirmed_grasp_body is not None:
                    teleop_state.request_haptic(
                        0.45,
                        120,
                        f"grasp-lost:{body_names.get(confirmed_grasp_body, confirmed_grasp_body)}",
                    )
                    print(
                        "Grasp lost:",
                        body_names.get(confirmed_grasp_body, confirmed_grasp_body),
                    )
                    confirmed_grasp_body = None

            if assisted_pick.phase == "close":
                if confirmed_grasp_body == assisted_pick.body_id:
                    assisted_pick.constraint_id = create_grasp_constraint(
                        p,
                        client_id,
                        panda_id,
                        assisted_pick.body_id,
                    )
                    assisted_pick.phase = "lift"
                    assisted_pick.phase_started = now
                    assisted_pick.target = assisted_pick.grasp
                elif now - assisted_pick.phase_started >= args.assist_close_timeout:
                    failed_name = body_names.get(
                        assisted_pick.body_id,
                        assisted_pick.body_id,
                    )
                    assisted_pick = AssistedPickState()
                    motion_anchor = target
                    if quest_position is not None:
                        quest_origin = quest_position
                    fine_offset = [0.0, 0.0, 0.0]
                    teleop_state.request_haptic(
                        0.2,
                        160,
                        f"assist-grasp-failed:{failed_name}",
                    )
                    print("Assisted pick failed to establish two-finger contact:", failed_name)

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
                    f"fine={tuple(round(v, 3) for v in fine_offset)}",
                    f"contact={body_names.get(touching_body, '-')}",
                    f"grasp={body_names.get(confirmed_grasp_body, '-')}",
                    f"assist={assisted_pick.phase}:{body_names.get(assisted_pick.body_id, '-')}",
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
    parser.add_argument("--joystick-speed", type=float, default=0.055)
    parser.add_argument("--joystick-deadzone", type=float, default=0.16)
    parser.add_argument("--joystick-max-offset", type=float, default=0.25)
    parser.add_argument("--controller-timeout", type=float, default=0.25)
    parser.add_argument("--contact-force-threshold", type=float, default=0.02)
    parser.add_argument("--grasp-confirm-time", type=float, default=0.08)
    parser.add_argument("--haptic-interval", type=float, default=0.12)
    parser.add_argument("--assist-radius", type=float, default=0.18)
    parser.add_argument("--assist-approach-height", type=float, default=0.12)
    parser.add_argument("--assist-speed", type=float, default=0.18)
    parser.add_argument("--assist-descend-speed", type=float, default=0.075)
    parser.add_argument("--assist-tolerance", type=float, default=0.004)
    parser.add_argument("--assist-close-timeout", type=float, default=1.5)
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
    if not 0.0 <= args.joystick_deadzone < 1.0:
        parser.error("--joystick-deadzone must be in [0, 1)")
    if args.joystick_speed <= 0.0 or args.joystick_max_offset <= 0.0:
        parser.error("joystick speed and maximum offset must be positive")
    if args.controller_timeout <= 0.0:
        parser.error("--controller-timeout must be positive")
    if (
        args.assist_radius <= 0.0
        or args.assist_approach_height <= 0.0
        or args.assist_speed <= 0.0
        or args.assist_descend_speed <= 0.0
        or args.assist_tolerance <= 0.0
        or args.assist_close_timeout <= 0.0
    ):
        parser.error("assisted-pick distances, speeds, tolerance, and timeout must be positive")
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
