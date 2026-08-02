#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/.venv-lerobot}"
FOLLOWER_PORT="${FOLLOWER_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E115987-if00}"
CAMERA_SERVER="${CAMERA_SERVER:-127.0.0.1}"
CAMERA_PORT="${CAMERA_PORT:-5555}"
POLICY_PATH="${POLICY_PATH:-}"
TASK="${TASK:-Pick up the green block and place it in the marked target.}"
DURATION_S="${DURATION_S:-30}"

if [[ -z "$POLICY_PATH" ]]; then
    echo "Set POLICY_PATH to a trained checkpoint or Hugging Face model repository." >&2
    exit 2
fi

cat <<EOF
Autonomous policy rollout will enable follower torque for up to ${DURATION_S}s.
Policy: ${POLICY_PATH}
Task:   ${TASK}

Clamp the base, clear the workspace, begin from a demonstrated reset pose, and
keep the follower power switch accessible.
EOF
read -r -p "Type RUN to enable autonomous motion: " confirmation
if [[ "$confirmation" != "RUN" ]]; then
    echo "Rollout cancelled."
    exit 1
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

exec lerobot-rollout \
    --strategy.type=base \
    --inference.type=rtc \
    --inference.rtc.execution_horizon=10 \
    --inference.rtc.max_guidance_weight=10.0 \
    --policy.path="$POLICY_PATH" \
    --robot.type=so101_follower \
    --robot.port="$FOLLOWER_PORT" \
    --robot.id=my_awesome_follower_arm \
    --robot.max_relative_target=1.0 \
    --robot.cameras="{overhead: {type: zmq, server_address: \"$CAMERA_SERVER\", port: $CAMERA_PORT, camera_name: overhead, width: 640, height: 480, fps: 30}}" \
    --task="$TASK" \
    --duration="$DURATION_S" \
    --fps=30 \
    --display_data=true
