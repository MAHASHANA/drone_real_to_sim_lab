# SO-101 VLA workflow

This directory adds a LeRobot-native path from leader demonstrations to a
SmolVLA policy. The first task is deliberately narrow:

> Pick up the green block and place it in the marked target.

The policy inputs are the fixed D455 RGB view, the six calibrated follower
joint values, and the task text. The outputs are chunks of six follower joint
targets. The D455 depth stream remains part of the geometry, calibration, and
verification pipeline under `sensors/realsense_d455`; it is not passed to the
pretrained SmolVLA image encoder in this first experiment.

## 1. Install and check

The existing Python 3.10 RSUSB binding owns the D455. LeRobot receives its RGB
frames through a local ZMQ camera stream, so Python 3.10 camera binaries are not
loaded into the Python 3.12 LeRobot process.

Install the small ZMQ client dependency in the LeRobot environment:

```bash
cd /home/satya/ai_agents/drones/drone_real_to_sim_lab
source .venv-lerobot/bin/activate
python -m pip install 'pyzmq>=26,<27'
hardware/so101/vla/preflight.sh
```

The preflight is read-only. It checks stable serial paths, calibration files,
Python dependencies, the RSUSB binding, and CUDA availability. It does not open
the servo buses or enable torque. `transformers` and `accelerate` are needed for
training but not for demonstration recording.

## 2. Record a smoke dataset

Stop the ROS2 D455 node or other camera viewer because only one process can own
the camera. Start the camera bridge in terminal one:

```bash
sensors/realsense_d455/start_lerobot_zmq_rgb_server.sh
```

Keep that process running. In terminal two, keep the task wording, camera name,
camera order, camera mounting, lighting, reset pose, object, and target
consistent. Start with three episodes:

```bash
hardware/so101/vla/record_pick_dataset.sh
```

During recording:

- Right Arrow saves the episode and starts the next one.
- Left Arrow discards the current episode and retries it.
- Escape stops recording and finalizes the dataset.

The default dataset is local and is not uploaded. Inspect episode zero with:

```bash
source .venv-lerobot/bin/activate
lerobot-dataset-viz \
  --repo-id local/so101_green_block_pick \
  --episode-index 0
```

After confirming that RGB, state, action, timing, and gripper closure are
correct, collect at least 50 successful demonstrations with randomized initial
block positions:

```bash
NUM_EPISODES=50 \
DATASET_REPO_ID="$HF_USER/so101_green_block_pick_v1" \
PUSH_TO_HUB=true \
hardware/so101/vla/record_pick_dataset.sh
```

Failed demonstrations should be retried rather than saved. Deliberate recovery
behavior belongs in a later dataset with a clearly defined task and success
criterion.

## 3. Fine-tune SmolVLA

This WSL environment currently has a CPU-only PyTorch build. Use a CUDA machine
or cloud GPU for training. Install the SmolVLA runtime on that machine:

```bash
source .venv-lerobot/bin/activate
python -m pip install 'lerobot[smolvla]==0.6.0'
```

Then train:

```bash
DATASET_REPO_ID="$HF_USER/so101_green_block_pick_v1" \
BATCH_SIZE=8 \
STEPS=20000 \
hardware/so101/vla/train_smolvla.sh
```

Start with a small batch and increase it only when GPU memory allows. The
training script fine-tunes `lerobot/smolvla_base`; it does not train a VLA from
scratch.

## 4. Autonomous rollout

Do not run the base model directly on the arm. Use only a checkpoint trained on
this robot, camera configuration, task wording, and action representation.

```bash
POLICY_PATH=outputs/train/smolvla_green_block_pick/checkpoints/last/pretrained_model \
hardware/so101/vla/rollout_smolvla.sh
```

The rollout requires an explicit `RUN` confirmation, limits each command
relative to the measured follower state, and runs for 30 seconds by default.
Those controls reduce command jumps; they do not provide collision avoidance,
force limiting, or a certified safety system.
