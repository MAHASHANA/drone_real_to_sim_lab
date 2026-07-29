# SO-101 hardware workflow

This directory contains the first hardware-validation workflow:

1. teleoperate the SO-101 follower from the SO-101 leader;
2. record requested, accepted, and measured joint poses;
3. validate the recording without connecting to hardware;
4. replay the same trajectory slowly in an unchanged physical setup.

This is blind joint-space replay. It has no object localization, collision
checking, visual correction, or grasp verification. Use it only as a controlled
baseline before adding the D455 eye-to-hand pipeline.

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

## Record

Before starting:

- clamp both bases;
- clear the workspace;
- keep the follower power switch accessible;
- place the leader and follower in matching poses;
- use slow, continuous motions;
- begin and end in repeatable poses.

Run:

```bash
python hardware/so101/record_leader_follower_demo.py \
  --leader-port /dev/serial/by-id/<LEADER> \
  --follower-port /dev/serial/by-id/<FOLLOWER>
```

The script first checks that all motors and calibrations are available. It also
refuses to start if the leader and follower poses differ by more than the
configured limits. Type `RECORD` only after inspecting the physical setup.

Demonstrate this sequence:

1. start pose;
2. approach above the target with the gripper open;
3. descend;
4. close the gripper;
5. pause briefly;
6. lift;
7. pause in the final pose;
8. press `Ctrl+C`.

The JSONL recording is written under `captures/so101_blind_<timestamp>/`. Each
frame contains the leader action, follower action after safety limiting,
follower feedback, and timestamps around command and observation.

## Validate

Dry-run validation does not connect to or command the robot:

```bash
python hardware/so101/replay_blind_demo.py \
  captures/so101_blind_<timestamp>/demo.jsonl
```

It checks schema, frame ordering, timestamps, and maximum consecutive joint
steps.

## Replay

First remove the object and replay the pick-shaped motion in free space. Put the
unpowered follower at the recorded start pose, clear the workspace, and run:

```bash
python hardware/so101/replay_blind_demo.py \
  captures/so101_blind_<timestamp>/demo.jsonl \
  --execute \
  --follower-port /dev/serial/by-id/<FOLLOWER>
```

The default replay speed is `0.5`, or twice the recorded duration. The script
checks the starting pose, requires typing `REPLAY`, monitors joint tracking
error, and disables torque when it exits.

Only after a successful free-space replay should the same trajectory be tested
with a lightweight, non-fragile object in the identical position. Blind replay
is expected to fail when the camera, arm base, object pose, grasp contact, or
workspace changes. Those failures are the baseline that eye-to-hand calibration
and closed-loop vision will address.
