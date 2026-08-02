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

The current USB/IP connection reports USB 3.2. The launcher provides four
matching RGB/depth profiles:

```text
highres    1280x720 at 30 FPS
balanced    848x480 at 30 FPS
highspeed   640x360 at 90 FPS
low         424x240 at  5 FPS
```

`highres` is the default. The D455 does not support 1280x720 at 90 FPS.
High-resolution 1280x720 streams stop at 30 FPS; 90 FPS requires a lower
resolution. Aligned depth is enabled. Point-cloud publication is disabled by
default because a dense 1280x720 cloud adds substantial CPU and transport
load.

The launcher enables the infrared emitter at 360 mW. In the measured tabletop
scene this increased valid ROI coverage from approximately 69.8% to 70.7% and
reduced table-plane RMS residual from 2.7 mm to 2.5 mm. Override it when close
or reflective parts saturate:

```bash
D455_LASER_POWER=150 \
  sensors/realsense_d455/launch_d455_ros2.sh --profile highres
```

Hole-filling filters are intentionally disabled in the metric perception
stream. They improve visual completeness by inventing values for invalid
pixels, but can move object boundaries and bias pose estimates.

The D455 advertises 848x480 at 60 FPS, but that profile produced UVC watchdog
timeouts through the current WSL USB/IP path. The `balanced` preset therefore
uses 30 FPS.

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

Select a different profile when latency matters more than image resolution:

```bash
sensors/realsense_d455/launch_d455_ros2.sh --profile balanced
sensors/realsense_d455/launch_d455_ros2.sh --profile highspeed
```

Enable the ROS point cloud only when a consumer requires it:

```bash
D455_POINTCLOUD=true \
  sensors/realsense_d455/launch_d455_ros2.sh --profile balanced
```

Do not combine the `highres` profile with the dense ROS point cloud on the
current WSL2/USB-IP path. A measured cloud contained about 482,000 points and a
9.6 MB ROS payload; with high-resolution RGB and aligned depth enabled, actual
delivery fell to approximately 5.5 Hz RGB and 0.5 Hz depth/cloud instead of
30 Hz, accompanied by USB control-transfer and D455 right-MIPI hardware
notifications. The launcher rejects that combination unless
`D455_ALLOW_HIGHRES_POINTCLOUD=true` is explicitly set for diagnostics.

For image-based localization, keep the point cloud disabled and deproject only
the segmented pixels or selected image points from aligned depth. This avoids
publishing a dense cloud when downstream code needs only a small workspace
region.

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

## Camera-Frame 3D Point Inspection

Keep `launch_d455_ros2.sh` running, then open another ROS2 terminal:

```bash
cd /home/satya/ai_agents/drones/drone_real_to_sim_lab
source /opt/ros/humble/setup.bash
python3 sensors/realsense_d455/rgbd_point_inspector.py
```

Click a visible object point to read its metric XYZ coordinate in
`camera_color_optical_frame`. The tool uses aligned depth and the live
`CameraInfo` intrinsics. This validates RGB-depth correspondence but does not
yet transform the point into the robot base frame.

The viewer displays cropped RGB beside a table-relative height map. It fits the
dominant work-surface plane with RANSAC and displays signed height above that
plane from -10 to 120 mm. Press `m` to toggle between table height and raw
aligned depth. Press `r` to refit the table after moving the camera or work
surface. Click either panel to inspect camera-frame XYZ and table-relative
height.

Black geometry pixels are invalid, outside the configured range, or outside
the height interval. Automatic raw-depth contrast uses the 2nd and 95th
percentiles of valid ROI depth and displays the valid-pixel percentage. Neither
visualization modifies the metric depth used for XYZ calculations.

For a tabletop around 0.5-0.7 m from the camera, constrain raw-depth contrast
to the workspace rather than the distant background:

```bash
python3 sensors/realsense_d455/rgbd_point_inspector.py \
  --min-depth-m 0.48 \
  --max-depth-m 0.90
```

Override the table-height interval when parts are taller than 120 mm:

```bash
python3 sensors/realsense_d455/rgbd_point_inspector.py \
  --min-height-m -0.01 \
  --max-height-m 0.20
```

Use a fixed visualization range when comparing frames with identical colors:

```bash
python3 sensors/realsense_d455/rgbd_point_inspector.py \
  --depth-contrast fixed \
  --min-depth-m 0.4 \
  --max-depth-m 1.2
```

The workcell operating region was measured in the original 424x240 image as
`x=[68,368)` and `y=[66,238)`. It is automatically scaled to the active stream
resolution. At 1280x720 the effective ROI is approximately
`x=[205,1111)` and `y=[198,714)`. Override it without changing the source:

```bash
python3 sensors/realsense_d455/rgbd_point_inspector.py \
  --roi 68 66 368 238 \
  --roi-reference-size 424 240
```

The display is cropped, but deprojection continues to use the corresponding
full-image pixel and the original camera intrinsics. Pixels outside the ROI
must also be excluded by downstream segmentation and point-cloud processing.

The optical-frame convention is:

```text
+X right in the image
+Y down in the image
+Z forward from the camera
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
