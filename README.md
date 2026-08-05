# Drone Real-to-Sim Lab

A robotics workcell for teaching assembly tasks with an SO-101 leader/follower
pair, observing them with RGB-D cameras, and transferring the resulting data
between real hardware, ROS2, and simulation.

## Current Status

Working end to end:

- Low-latency SO-101 leader-to-follower control through LeRobot.
- ROS2 telemetry for leader and follower joint states without ROS owning the
  motor serial ports.
- Intel RealSense D455 RGB-D streaming under WSL2.
- D455-to-SO-101 eye-to-hand calibration using a ChArUco board and
  `easy_handeye2`.
- PyBullet drone-frame assembly and Quest 2 teleoperation experiments.

The current workcell calibration used 26 poses. A held-out 11-pose validation
reported 8.7 mm maximum translation divergence, which is sufficient for an
initial large-object pick-and-place test but not precision insertion or screw
handling.

The next milestone is to record one repeatable rigid-object task and train an
ACT imitation-learning baseline before evaluating a VLA.

## Architecture

```text
SO-101 leader ──> LeRobot control bridge ──> SO-101 follower
                         │
                         └── UDP telemetry ──> ROS2 joint states / TF

D455 RGB-D ─────────────────────────────────> ROS2 perception
ChArUco board + robot TF ───────────────────> eye-to-hand calibration

ROS2 demonstrations ──> LeRobot dataset ──> ACT policy ──> guarded rollout
```

The direct LeRobot bridge is the only process that commands the physical arm.
ROS2 handles cameras, transforms, recording, monitoring, and calibration.

## Repository

```text
hardware/so101/             SO-101 control and diagnostics
sensors/realsense_d455/     D455 ROS2 and WebXR tools
sensors/iphone_lota/        iPhone LiDAR/LOTA experiments
calibration/eye_to_hand/    ChArUco and easy_handeye2 pipeline
ros2/handeye_ws/            reproducible ROS2 workspace and patches
sim/pybullet/               drone assembly and Quest simulations
configs/                    Mark4 frame parameters
analysis/                   offline diagnostics
```

Generated builds, external ROS repositories, virtual environments, captures,
and machine-local calibration results are intentionally excluded from Git.

## Quick Start

### ROS2 workspace

The setup pins external dependencies and applies the ROS Humble compatibility
patches used by this workcell:

```bash
ros2/handeye_ws/setup.sh
ros2/handeye_ws/build.sh
source ros2/handeye_ws/env.sh
```

### SO-101 teleoperation

Clamp both arm bases, clear the follower workspace, pose-match the arms, and
keep the follower power switch accessible.

```bash
source .venv-lerobot/bin/activate
python hardware/so101/leader_follower_bridge.py --enable-motion
```

The bridge requires a typed confirmation and disables follower torque during a
normal shutdown. See [hardware/so101/README.md](hardware/so101/README.md) for
port mapping, calibration, and servo diagnostics.

### RealSense D455

After attaching the D455 to WSL:

```bash
source ros2/handeye_ws/env.sh
sensors/realsense_d455/launch_d455_ros2.sh --profile highres
```

See [sensors/realsense_d455/README.md](sensors/realsense_d455/README.md) for
USB/IP setup, RGB-D profiles, topics, and visualization.

### Eye-to-hand calibration

Do not run another teleoperation process at the same time. The integrated
launcher starts the D455, direct arm bridge, ROS telemetry, ChArUco tracking,
and Easy Handeye GUI:

```bash
calibration/eye_to_hand/start_calibration_stack.sh
```

The board is 7 x 5 squares, with 24 mm squares, 19 mm markers, and
`DICT_6X6_250`. Full instructions and frame definitions are in
[calibration/eye_to_hand/README.md](calibration/eye_to_hand/README.md).

### PyBullet

```bash
python3 sim/pybullet/generate_frame_urdf.py
python3 sim/pybullet/assembly_pybullet.py --layout loose --assemble-demo --gui
python3 sim/pybullet/robot_arm_pybullet.py --parts front_right_arm --gui
```

## Experimental Tools

- Quest 2 WebXR teleoperation and RGB-D panels are under `sim/pybullet/` and
  `sensors/realsense_d455/`.
- iPhone 14 Pro LiDAR, PLY, depth, and OSC receivers are under
  `sensors/iphone_lota/`.
- The generated Mark4 seven-inch frame is configured in
  `configs/frame_config.json`.

These tools support data collection and simulation experiments; they are not
safety-rated robot controllers.
