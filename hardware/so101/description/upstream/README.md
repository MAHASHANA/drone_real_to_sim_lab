# Upstream SO-101 Description

Source:

```text
https://github.com/TheRobotStudio/SO-ARM100
```

Pinned revision:

```text
fda892cba81032c46c40976a48c9ceadbf40a9ca
```

Copied from:

```text
Simulation/SO101/so101_new_calib.urdf
Simulation/SO101/assets/
LICENSE
```

The upstream repository is Apache-2.0 licensed. These files are intentionally
kept unchanged so their geometry can be compared with the physical SO-101.

`check_urdf` successfully parses the chain from `base_link` to
`gripper_frame_link`. This is a useful kinematic and visual starting point, but
it is not yet the robot's production ROS2/MoveIt description:

- Mesh-relative paths require this complete directory.
- Transmissions use the older `PositionJointInterface` representation rather
  than a ROS2 `ros2_control` hardware block.
- Detailed visual meshes are also used as collision meshes.
- Joint limits and the `gripper_frame_link` tool-center point must be validated
  against the calibrated physical arm.

Put ROS2 adaptations in a separate sibling directory; do not silently modify
the pinned upstream copy.
