# Intel RealSense D455

This module contains the reproducible WSL2 launch path for the D455. The
camera is attached to WSL with `usbipd-win`, and ROS2 uses a local librealsense
build with the userspace RSUSB backend because the current WSL kernel does not
provide `uvcvideo`.

## Verified Environment

- Ubuntu 22.04 under WSL2
- ROS2 Humble
- `realsense2_camera` 4.55.1
- librealsense 2.55.1 built with `FORCE_RSUSB_BACKEND=ON`
- D455 firmware 5.17.0.9

The verified USB/IP connection currently reports USB 2.1. Use conservative
profiles until the camera is available over USB 3:

- depth: 480x270 at 5 FPS
- color: 424x240 at 5 FPS
- aligned depth enabled
- colored point cloud enabled

Observed ROS2 rates were approximately 4.8 Hz for depth, 4.4 Hz for color, and
3.3 Hz for the registered point cloud.

## Windows Attachment

Run PowerShell as Administrator:

```powershell
usbipd bind --busid 2-1
usbipd attach --wsl --busid 2-1
```

The bus ID can change after reconnecting the camera. Check it with:

```powershell
usbipd list
```

## RSUSB Backend

The ROS2 binary package normally uses the kernel V4L2 backend. On a WSL kernel
without `uvcvideo`, build an ABI-compatible userspace backend from an existing
librealsense source checkout:

```bash
sensors/realsense_d455/build_rsusb_backend.sh "$HOME/librealsense"
```

This creates `build-rsusb` inside the external librealsense checkout. It does
not install over the system ROS2 package.

## ROS2 Launch

With the camera attached:

```bash
sensors/realsense_d455/launch_d455_ros2.sh
```

The launcher prepends the RSUSB library directory to `LD_LIBRARY_PATH`, then
starts the installed `realsense2_camera` node. Override the default paths when
needed:

```bash
ROS_DISTRO=humble \
RSUSB_LIB_DIR="$HOME/librealsense/build-rsusb/Release" \
sensors/realsense_d455/launch_d455_ros2.sh
```

Important topics:

```text
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/depth/color/points
/camera/camera/color/camera_info
/camera/camera/depth/camera_info
/camera/camera/extrinsics/depth_to_color
/tf_static
```

Check live rates in another sourced ROS2 terminal:

```bash
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/color/points
```

## Quest 2 RGB-D Viewer

Keep `launch_d455_ros2.sh` running. In a second terminal with ROS2 Humble
sourced, start the Quest bridge:

```bash
cd /home/satya/ai_agents/drones/drone_real_to_sim_lab
npm install
python3 sensors/realsense_d455/quest_realsense_viewer.py \
  --advertise-ip 10.0.0.6
```

The default subscriptions are:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
```

Open `https://10.0.0.6:8443/` in Meta Quest Browser. Accept the local
certificate warning. The browser page first shows color and colorized aligned
depth in 2D; **Enter VR Workcell** presents the same streams as two world-space
panels. Point at either panel and hold the controller grip to reposition and
rotate it. Use the thumbstick vertically while holding to change its distance,
release grip to leave it in place, and press A to reset both panels.

The bridge JPEG-compresses only the latest frame and never queues old sensor
frames. Adjust the depth display and compression when needed:

```bash
python3 sensors/realsense_d455/quest_realsense_viewer.py \
  --advertise-ip 10.0.0.6 \
  --min-depth-m 0.2 \
  --max-depth-m 2.5 \
  --jpeg-quality 70
```

Port `8443` is shared with the Quest PyBullet teleoperation server, so only one
of these programs can use that port at a time. This first viewer sends RGB-D
images, not the full ROS2 point cloud.

USB control-transfer warnings can occur through USB/IP even while frames are
being published. Validate topic rates before treating the warnings as a stream
failure.
