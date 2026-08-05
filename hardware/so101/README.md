# SO-101 hardware workflow

The physical baseline is direct SO-101 leader-to-follower teleoperation through
LeRobot. The project does not maintain a separate motion recorder or blind
trajectory replayer. Demonstration collection will be added later through a
standard LeRobot dataset once the cameras and robot coordinate frames are
calibrated.

## Model

The upstream SO-101 URDF and its meshes are vendored under
`description/upstream/`. See its README for the pinned source revision,
validation results, and limitations.

The URDF uses radians. The hardware scripts use LeRobot's calibrated values:
degrees for the five arm joints and percent for the gripper. Do not copy values
between those representations without an explicit conversion and zero-offset
mapping.

## Prerequisites

Use PowerShell to attach both CH343 controller boards to WSL. Bus IDs can change
after reconnecting USB, so inspect them instead of reusing an old bus ID:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

In WSL, confirm that two stable serial paths exist:

```bash
ls -l /dev/serial/by-id/
```

Then activate the environment and verify both calibration files:

```bash
source .venv-lerobot/bin/activate
test -f ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_awesome_leader_arm.json
test -f ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json
```

Identify which by-id path is the leader and which is the follower before
continuing. Do not use `/dev/ttyACM0` and `/dev/ttyACM1` in saved commands
because those names can swap after reconnecting.

## Verify Connections

Both calibration files must exist:

```bash
test -f ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_awesome_leader_arm.json
test -f ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json
```

Confirm that each controller responds on its identified path before enabling
follower torque. Only one process may open a serial port at a time.

## Direct Teleoperation

Before starting:

- clamp both bases;
- clear the follower workspace;
- keep the follower power switch accessible;
- manually place leader and follower in closely matching poses;
- identify the stable leader and follower paths instead of assuming tty order.

Activate the environment and run the maintained LeRobot loop:

```bash
source .venv-lerobot/bin/activate

lerobot-teleoperate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/serial/by-id/<LEADER> \
  --teleop.id=my_awesome_leader_arm \
  --robot.type=so101_follower \
  --robot.port=/dev/serial/by-id/<FOLLOWER> \
  --robot.id=my_awesome_follower_arm \
  --robot.max_relative_target=2.0 \
  --fps=30
```

The loop reads calibrated leader joint positions and sends them directly as
calibrated follower joint targets. `max_relative_target=2.0` limits each joint
command to at most two normalized units from the measured follower position per
control cycle. It reduces abrupt jumps but is not collision avoidance.

Press `Ctrl+C` to stop. LeRobot disables follower torque during a normal
disconnect. Use the physical power switch if communication fails or motion is
unexpected.

## ROS Telemetry Bridge

For calibration, use the maintained direct-control bridge instead of the ROS
trajectory teleoperation component. Its control loop follows LeRobot's direct
teleoperation sequence and only adds localhost telemetry. Manually pose-match
the arms before launch. It is motion-disabled unless `--enable-motion` is
supplied:

```bash
source .venv-lerobot/bin/activate
unset PYTHONPATH

python hardware/so101/leader_follower_bridge.py --enable-motion
```

The controller is the only process allowed to open the two serial ports. It
sends latest-state telemetry over localhost UDP. The ROS Humble receiver runs
in Python 3.10 and publishes `/leader/joint_states`,
`/follower/joint_states`, and `/so101/control_status`; it never commands the
motors. This split keeps the proven Python 3.12 LeRobot runtime isolated from
ROS Humble's Python 3.10 runtime.

## VLA Demonstrations

The LeRobot dataset, SmolVLA training, and guarded rollout workflow is under
[`vla/`](vla/README.md). Complete direct teleoperation and camera checks before
recording demonstrations.

## Gripper Diagnostics

`gripper_telemetry.py` contains the register-reading helpers used to inspect
gripper current, load, voltage, temperature, motion state, and position error.
Servo current and load are diagnostics, not calibrated fingertip force. A load
cell calibration is required before expressing grip force in Newtons.
