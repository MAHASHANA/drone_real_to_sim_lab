# Drone Real-to-Sim Lab

Real-to-sim workspace for iPhone LiDAR/LOTA sensing, Mark4 7-inch FPV frame
simulation, calibration, and assembly experiments.

The current stack is intentionally simple:

- iPhone 14 Pro + LOTA for depth, PLY point clouds, and OSC camera pose.
- PyBullet for the drone frame and robot-arm assembly scaffold.
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
