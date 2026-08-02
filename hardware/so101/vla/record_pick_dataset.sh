#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/.venv-lerobot}"
LEADER_PORT="${LEADER_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E113301-if00}"
FOLLOWER_PORT="${FOLLOWER_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E115987-if00}"
CAMERA_SERVER="${CAMERA_SERVER:-127.0.0.1}"
CAMERA_PORT="${CAMERA_PORT:-5555}"
DATASET_REPO_ID="${DATASET_REPO_ID:-local/so101_green_block_pick}"
TASK="${TASK:-Pick up the green block and place it in the marked target.}"
NUM_EPISODES="${NUM_EPISODES:-3}"
EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
RESET_TIME_S="${RESET_TIME_S:-15}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
DISPLAY_DATA="${DISPLAY_DATA:-true}"

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

exec lerobot-record \
    --robot.type=so101_follower \
    --robot.port="$FOLLOWER_PORT" \
    --robot.id=my_awesome_follower_arm \
    --robot.max_relative_target=2.0 \
    --robot.cameras="{overhead: {type: zmq, server_address: \"$CAMERA_SERVER\", port: $CAMERA_PORT, camera_name: overhead, width: 640, height: 480, fps: 30}}" \
    --teleop.type=so101_leader \
    --teleop.port="$LEADER_PORT" \
    --teleop.id=my_awesome_leader_arm \
    --dataset.repo_id="$DATASET_REPO_ID" \
    --dataset.single_task="$TASK" \
    --dataset.fps=30 \
    --dataset.episode_time_s="$EPISODE_TIME_S" \
    --dataset.reset_time_s="$RESET_TIME_S" \
    --dataset.num_episodes="$NUM_EPISODES" \
    --dataset.video=true \
    --dataset.push_to_hub="$PUSH_TO_HUB" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --display_data="$DISPLAY_DATA"
