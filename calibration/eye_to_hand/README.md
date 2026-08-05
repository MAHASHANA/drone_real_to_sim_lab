# D455 to SO-101 Eye-to-Hand Calibration

This is an **eye-on-base** setup:

- The Intel RealSense D455 stays fixed above and to the side of the workcell.
- The SO-101 base stays fixed.
- The ChArUco board is attached rigidly to the wrist/end effector only while
  calibration samples are collected.
- The camera is not mounted on the arm.

The result is the rigid transform from the D455 color optical frame to the
SO-101 base frame:

```text
T_base_camera
```

It maps camera-frame points into the robot base:

```text
p_base = T_base_camera * p_camera
```

## Packages

The calibration solver is
[`easy_handeye2`](https://github.com/marcoesposito1988/easy_handeye2). It
samples transforms from ROS TF, runs OpenCV hand-eye algorithms, saves the
result, and publishes the calibrated transform.

The SO-101 joint-state bridge and URDF come from
[`so101_ros2`](https://github.com/nimiCurtis/so101_ros2). A pinned,
ROS-Humble-compatible LeRobot fork is installed in a separate Python 3.10
environment. The existing Python 3.12 `.venv-lerobot` environment and saved
motor calibration files are not modified.

Project code only supplies the workcell-specific ChArUco TF publisher and
launch configuration.

## One-Time Setup

From the repository root:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-launch-param-builder \
  ros-humble-usb-cam

ros2/handeye_ws/setup.sh
ros2/handeye_ws/build.sh
```

`build.sh` compiles the complete upstream hardware path: the Python bridge,
ROS 2 Control hardware interface, controllers, leader teleoperation component,
description, and bringup packages, plus the hand-eye calibration packages. It
runs `check_ros_dependencies.sh` first and stops with the install command when
a required binary package is missing.

The generic `image_tools/cam2image` OpenCV path is not used because it can fail
to open this camera through USB/IP even when V4L2 advertises valid formats.

The setup script pins all three external repositories to known commits. Local
copies, virtual environments, and ROS build outputs are ignored by Git.

For each new terminal:

```bash
source ros2/handeye_ws/env.sh
```

## ChArUco Target

The printed board is:

```text
7 x 5 squares
24 mm square length
19 mm marker length
DICT_6X6_250
start ID 0
```

Mount the print flat on a rigid backing. Attach the backing rigidly to the
wrist or gripper so the D455 can see it without gripper, cable, or fastener
occlusion. The exact board-to-gripper transform does not need to be measured,
but it must not change during calibration.

With the board stationary and the D455 ROS stream active, check visibility:

```bash
/bin/python3 calibration/eye_to_hand/check_charuco_live.py
```

Do not collect calibration samples until the checker consistently sees at
least 18 of the 24 internal corners with low pose jitter. The previous test
only saw a median of 12 corners because the board was small in the image and
partly obstructed.

## Start Camera

### Integrated calibration stack

After attaching both CH343 boards, the D455, and the wrist USB camera to WSL,
start the complete stack from the repository root:

```bash
calibration/eye_to_hand/start_calibration_stack.sh
```

The default launcher validates the stable arm ports and saved motor
calibrations, starts the fixed D455 in high-resolution color-only mode, and
then starts the direct LeRobot controller, ROS telemetry receiver, follower TF,
ChArUco tracking, and the `easy_handeye2` calibration GUI. Color-only mode
avoids unnecessary D455 depth bandwidth during calibration. It requires an
explicit `START` confirmation after the camera preflight and before connecting
to the follower. `Ctrl+C` stops all child processes. Logs are stored under
`captures/`.

The wrist camera, D455 depth stream, compressed image transports, and rosbag
recording are optional because none is an input to eye-to-hand calibration.
Enable all of them explicitly for a diagnostic recording run:

```bash
D455_PROFILE=balanced RECORD_SESSION=true \
calibration/eye_to_hand/start_calibration_stack.sh
```

Use the balanced profile for recording under WSL2/USB-IP; simultaneous
1280x720 RGB and depth has produced D455 stream watchdog failures.

Do not run `lerobot-teleoperate` at the same time. The direct LeRobot controller
in this launcher already owns both serial ports. The fixed D455 provides the hand-eye
observations; the wrist USB camera is published on
`/wrist_camera/image_raw` for inspection and is not used by this calibration.

The stable paths and optional wrist stream can be overridden when necessary:

```bash
LEADER_PORT=/dev/serial/by-id/<leader> \
FOLLOWER_PORT=/dev/serial/by-id/<follower> \
ENABLE_WRIST_CAMERA=true \
WRIST_VIDEO_DEVICE=/dev/video0 \
calibration/eye_to_hand/start_calibration_stack.sh
```


### Camera only

Attach the D455 to WSL and run:

```bash
sensors/realsense_d455/launch_d455_ros2.sh --profile highres
```

Confirm both topics have publishers:

```bash
ros2 topic info /camera/camera/color/image_raw
ros2 topic info /camera/camera/color/camera_info
```

## Validate TF Inputs

Only the LeRobot control bridge may open the arm serial ports. The ROS launch
receives localhost telemetry and never commands the follower. With the arm
controller running, start the TF inputs with:

```bash
source ros2/handeye_ws/env.sh
ros2 launch drone_handeye_calibration capture.launch.py
```

In another sourced terminal, verify both independent transforms:

```bash
ros2 run tf2_ros tf2_echo \
  follower/base_link follower/gripper_frame_link

ros2 run tf2_ros tf2_echo \
  camera_color_optical_frame charuco_board
```

The first transform must change with robot motion. The second must change with
the board motion and stop updating when fewer than 18 corners are visible.

## Run Calibration

Stop `capture.launch.py`, then start the complete solver launch:

```bash
source ros2/handeye_ws/env.sh
ros2 launch drone_handeye_calibration calibrate.launch.py
```

Collect at least 15-20 stationary poses. Vary wrist orientation substantially
about multiple axes while keeping the full board visible; translations alone
do not constrain hand-eye rotation. Avoid nearly identical or collinear poses.

The integrated launcher provides direct LeRobot leader-driven calibration
motion. Manually place the leader and follower in closely matching poses before
typing `START`; commands are mirrored immediately with the same default
processors and loop timing used by `lerobot-teleoperate`. Stop at each sample
pose before capturing it in `easy_handeye2`.

## Frames and Math

The TF inputs to `easy_handeye2` are:

```text
robot base:       follower/base_link
robot effector:   follower/gripper_frame_link
tracking base:    camera_color_optical_frame
tracking marker:  charuco_board
calibration type: eye_on_base
```

For each stationary sample, the robot contributes
`T_base_gripper`, and ChArUco PnP contributes `T_camera_board`.
`easy_handeye2` solves the fixed camera-to-base transform while eliminating the
unknown fixed board-to-gripper transform.

The D455 optical frame follows the ROS convention:

```text
+X right in the image
+Y down in the image
+Z forward from the camera
```

Recalibrate whenever the camera mount, robot base, board attachment, camera
intrinsics/profile, or workcell geometry changes.
