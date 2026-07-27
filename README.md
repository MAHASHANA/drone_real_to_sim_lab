# Drone Real-to-Sim Lab

Real-to-sim workspace for RGB-D and iPhone LiDAR sensing, Mark4 7-inch FPV
frame simulation, calibration, Quest teleoperation, and assembly experiments.

The current stack is intentionally simple:

- Intel RealSense D455 through ROS2 for aligned RGB-D and point clouds.
- iPhone 14 Pro + LOTA for depth, PLY point clouds, and OSC camera pose.
- PyBullet for the drone frame and robot-arm assembly scaffold.
- Meta Quest 2 + WebXR for controller tracking and simulated Panda teleoperation.
- Generated primitive URDF/manifest for fast iteration before CAD meshes.

## Layout

```text
configs/
  frame_config.json

sim/pybullet/
  generate_frame_urdf.py
  preview_pybullet.py
  assembly_pybullet.py
  robot_arm_pybullet.py
  pybullet_utils.py

sensors/iphone_lota/
  lota_receiver.py
  lota_live_viewer.py
  lota_ply_live_viewer.py
  lota_realtime_viewer.py
  lota_browser_3d_live_viewer.py
  lota_windows_osc_logger.ps1

sensors/realsense_d455/
  build_rsusb_backend.sh
  launch_d455_ros2.sh

calibration/
  future camera/PLY/world frame validation scripts

analysis/
  future offline capture and pose-analysis scripts

generated/
  7in_fpv_frame.urdf
  part_manifest.json
```

Runtime captures live under `sensors/iphone_lota/captures/` and are ignored by
Git.

## Generate Frame Assets

```bash
python3 sim/pybullet/generate_frame_urdf.py
```

This reads `configs/frame_config.json` and writes:

```text
generated/7in_fpv_frame.urdf
generated/part_manifest.json
```

## PyBullet

Frame URDF smoke test:

```bash
python3 sim/pybullet/preview_pybullet.py --steps 240
```

Separate-parts assembly demo:

```bash
python3 sim/pybullet/assembly_pybullet.py --layout loose --assemble-demo
```

One-arm pick/place demo:

```bash
python3 sim/pybullet/robot_arm_pybullet.py --parts front_right_arm
```

Two-arm pick/place demo:

```bash
python3 sim/pybullet/robot_arm_pybullet.py --arms 2 --parts front_right_arm front_left_arm
```

Add `--gui` to open the PyBullet viewer.

Quest/WebXR controller teleop:

```bash
python3 sim/pybullet/quest_webxr_teleop.py --gui --advertise-ip 10.0.0.6
```

The Quest/WebXR code is split into two layers:

- `sim/pybullet/quest_webxr_server.py`: HTTPS/WebXR page, WebSocket receiver,
  latest controller state.
- `sim/pybullet/quest_webxr_teleop.py`: PyBullet Panda IK control loop using
  the latest controller state.

Open the printed `https://<computer-ip>:8443/` URL in Meta Quest Browser,
accept the local certificate warning, and press `Start VR Teleop`. The right
controller drives the Panda end-effector target through IK. Trigger or grip
closes the simulated gripper. This is a demo/teleop bridge; it is not a
safety-rated robot controller.

Controller orientation is enabled by default. The first right-controller pose
after streaming starts is treated as neutral. By default, wrist tilt/twist
rotates only the orange held-object visualization while the Panda arm keeps a
stable downward gripper pose. This avoids full-arm IK reconfiguration when you
are just trying to inspect object orientation. For position-only testing, use
`--orientation-mode fixed`. To make wrist orientation drive the full Panda IK
target, use `--orientation-target ik`. If wrist rotation feels like it is
rotating around the wrong frame, try `--orientation-order world`.

By default the teleop scene includes a blue haptic target in PyBullet. Moving
the simulated gripper near it sends short vibration pulses to the right Quest
controller; squeezing trigger/grip near the target sends a stronger pulse.
Disable this with `--no-touch-demo`.

For live tracking diagnostics, open this on the laptop while the Quest page is
streaming:

```text
https://10.0.0.6:8443/debug
```

The debug page plots raw right-controller XYZ over time, top/front motion
traces, packet age, min/max position range, trigger, and grip values.

To record a motion-range session:

```bash
python3 sim/pybullet/quest_webxr_teleop.py --gui --advertise-ip 10.0.0.6 --record
```

Move the right controller through the usable area in front, sides, high/low,
and near/behind the headset. Samples are saved under
`captures/quest_tracking_*` with motion segment labels and a continuously
updated `summary.json`.

Summarize a recording after stopping:

```bash
python3 analysis/quest_tracking_summary.py captures/quest_tracking_YYYYMMDD_HHMMSS/samples.jsonl
```

When running inside WSL2 NAT, forward Windows Wi-Fi TCP `8443` to the current
WSL IP first:

```powershell
netsh interface portproxy add v4tov4 listenaddress=10.0.0.6 listenport=8443 connectaddress=172.31.204.166 connectport=8443
New-NetFirewallRule -DisplayName "Quest WebXR to WSL TCP 8443" -Direction Inbound -Action Allow -Protocol TCP -LocalAddress 10.0.0.6 -LocalPort 8443
```

### PyBullet Wrist-Camera Workcell

Run a single Panda with a simulated eye-in-hand RGB-D camera and control it
from Quest:

```bash
npm install
python3 sim/pybullet/quest_wrist_camera_teleop.py \
  --gui \
  --advertise-ip 10.0.0.6
```

Open `https://10.0.0.6:8443/` in Meta Quest Browser and select **Enter VR
Workcell**. The right controller drives the Panda end-effector through IK;
trigger or grip closes the gripper. The left-controller ray can select either
RGB-D panel, and holding left grip attaches that panel to the controller for
repositioning.

Position teleoperation keeps the end-effector orientation fixed by default.
Add `--orientation-mode controller` only when controller wrist rotation should
also become an IK target.

The right thumbstick provides fine adjustment on top of motion tracking:

- Left/right adds a bounded world-X offset.
- Forward/back adds a bounded tool-axis approach/retract offset.
- Pressing the thumbstick captures a new controller neutral pose without
  moving the robot target.
- Pressing right A near a workpiece starts an assisted pick: align above the
  nearest object, descend, close, verify two-finger contact, and lift.
- Pressing right A again releases a completed assisted pick. Right B also
  cancels an active pick or releases the held workpiece when Quest reports it.

The right controller gives a light haptic pulse when either finger contacts a
workpiece and a stronger pulse after both fingers maintain contact while the
gripper is commanded closed. This confirms simulated contact geometry, not
force-closure or a guaranteed stable physical grasp.

Assisted pick currently selects from PyBullet ground-truth object poses. It is
a shared-control prototype; replacing ground truth with calibrated RGB-D part
poses is required before this behavior represents a real perception pipeline.

The GUI client runs IK and physics independently from a spawned DIRECT-mode
render client. The render client mirrors the latest Panda joint and workpiece
poses, preventing RGB-D rendering from blocking the high-frequency control
loop.

The simulated optical camera is rigidly mounted to Panda link 11 with the
default transform:

```text
T_EE_C.xyz  = [0.00, 0.00, -0.08] m
T_EE_C.quat = [0.00, 0.00, 0.00, 1.00]
```

Its optical convention is `+x` right, `+y` down, `+z` forward. The camera pose
for every frame is `T_W_C = T_W_EE * T_EE_C`. Use scripted motion for a
headset-free transport test:

```bash
python3 sim/pybullet/quest_wrist_camera_teleop.py \
  --demo-motion \
  --run-seconds 15 \
  --port 9444 \
  --advertise-ip 127.0.0.1
```

## Intel RealSense D455 and ROS2

The verified WSL2 path uses the ROS2 Humble `realsense2_camera` package with a
local librealsense RSUSB backend. This bypasses the missing `uvcvideo` support
in the current WSL kernel without replacing the system ROS2 libraries.

After attaching the D455 to WSL with `usbipd-win`, launch the verified
low-bandwidth RGB-D configuration:

```bash
sensors/realsense_d455/launch_d455_ros2.sh
```

This publishes color, depth, aligned depth, camera calibration, extrinsics, and
a registered colored point cloud. See
[`sensors/realsense_d455/README.md`](sensors/realsense_d455/README.md) for
setup, build, topic, and rate-check details.

To visualize live color and aligned depth in Quest, leave the camera launch
running and start the RGB-D WebXR bridge in a second ROS2 terminal:

```bash
npm install
python3 sensors/realsense_d455/quest_realsense_viewer.py \
  --advertise-ip 10.0.0.6
```

Open the printed `https://10.0.0.6:8443/` URL in Meta Quest Browser. Accept the
local certificate warning, verify both feeds on the normal page, then select
**Enter VR Workcell** to show color and depth as world-space panels. Point at a
panel and hold controller grip to move it, use the thumbstick to adjust its
distance, and press A to reset both panels. The viewer keeps only the newest
JPEG-compressed frame so slow headset or network clients cannot build a
stale-frame queue. It does not yet render the registered 3D point cloud.

## iPhone LOTA

Run the low-latency browser viewer. Choose the `--lota-mode` that matches the
LOTA app mode.

Point Cloud mode receives Float32 LiDAR depth on TCP `9847` and PLY XYZ/RGB on
TCP `9848`. OSC camera pose is separate UDP data, usually on port `9000`, and
the realtime viewer listens for it in parallel by default:

```bash
cd sensors/iphone_lota
python3 lota_realtime_viewer.py \
  --host 0.0.0.0 \
  --lota-mode point-cloud \
  --depth-port 9847 \
  --ply-port 9848 \
  --osc-port 9000 \
  --http-host 0.0.0.0 \
  --http-port 8765 \
  --camera-forward neg-z
```

Depth Image mode uses only TCP `9847`:

```bash
python3 lota_realtime_viewer.py --lota-mode depth-image
```

Color mode sends H264 color video on TCP `9847`. This viewer detects the H264
stream and reports throughput, but H264 decode/display is not implemented yet:

```bash
python3 lota_realtime_viewer.py --lota-mode color
```

Neural Depth uses NDI, not TCP `9847/9848`, so it needs a separate NDI receiver.
Motion/IMU-style data should be treated as OSC/status data unless LOTA exposes a
separate binary stream for it.

Then open:

```text
http://localhost:8765/
```

The web page can open before the phone connects. In WSL, start this viewer
first, then restart/toggle LOTA streaming so the iPhone reconnects to the
active TCP listener.

For 2D/debug PLY viewing:

```bash
python3 lota_ply_live_viewer.py --host 0.0.0.0 --port 9848 --camera-forward neg-z
```

For depth maps:

```bash
python3 lota_live_viewer.py --host 0.0.0.0 --port 9847 --min-depth 0.2 --max-depth 2.5
```

For Windows-native OSC pose logging:

```powershell
powershell -ExecutionPolicy Bypass -File .\lota_windows_osc_logger.ps1 -ListenAddress 10.0.0.6 -Port 9000
```

## Current Coordinate Finding

LOTA PLY points appear to be camera/view-relative with camera forward along
negative Z:

```text
depth = -point.z
```

For world placement using OSC camera pose:

```text
p_world = R_world_camera @ p_camera + t_world_camera
```

The working assumption is currently:

```text
p_camera ~= p_lota_ply
```

This still needs a clean calibration pass before using it for precise object
pose tracking.

## Mark4 Frame Adaptation

Measure these first:

- motor-to-motor diagonal wheelbase
- arm width and thickness
- center plate length, width, and thickness
- stack-hole spacing, usually 30.5 mm or 20 mm
- motor mount diameter

Then edit `configs/frame_config.json` and rerun the generator.

For assembly tracking, keep part names stable:

- `front_right_arm`
- `front_left_arm`
- `rear_left_arm`
- `rear_right_arm`
- `bottom_plate`
- `top_plate`
- `motor_mount_*`
- `stack_standoff_*`
