# D455 to SO-101 Eye-to-Hand Calibration

The D455 is fixed beside the robot and looks down into the workspace. Its
physical position and tilt are represented by one rigid transform:

```text
T_base_camera
```

The transform maps coordinates expressed in the D455 color optical frame into
the SO-101 base frame:

```text
p_base = T_base_camera * p_camera
```

The D455 optical frame follows the ROS optical convention:

```text
+X right in the image
+Y down in the image
+Z forward from the camera
```

Do not enter the camera tilt as an approximate Euler angle. Estimate the full
rotation and translation from a calibration target.

## Initial Static-Target Method

1. Rigidly mount the camera and robot base.
2. Place a printed ChArUco or checkerboard target flat in the workspace.
3. Define the target origin and axes.
4. Align the target axes with the robot base axes and measure
   `T_base_target`.
5. Detect the board over multiple RGB frames and solve PnP to estimate
   `T_camera_target`.
6. Compute:

```text
T_base_camera = T_base_target * inverse(T_camera_target)
```

7. Save the resulting matrix together with the board dimensions, camera serial
   number, image profile, date, and reprojection error.
8. Validate using target points that were not used to calculate the transform.

For the first setup, a measured static target is simpler than full hand-eye
calibration because the current SO-101 integration does not yet provide a
validated forward-kinematics pose for every calibration sample.

## Object Pose

Aligned depth converts an RGB pixel into a camera-frame 3D point:

```text
Z = depth(u, v)
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

After external calibration:

```text
p_base_object = T_base_camera * p_camera_object
```

A clicked depth point provides only object position. A complete six-degree
object pose also requires orientation, estimated from a known CAD model,
geometric features, a fiducial, or a segmented point cloud.

## Recalibration Triggers

Recalculate `T_base_camera` whenever the camera mount, robot base, work surface,
camera resolution, alignment profile, or calibration target reference changes.
