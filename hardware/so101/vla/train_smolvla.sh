#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/.venv-lerobot}"
DATASET_REPO_ID="${DATASET_REPO_ID:-local/so101_green_block_pick}"
OUTPUT_DIR="${OUTPUT_DIR:-$ROOT_DIR/outputs/train/smolvla_green_block_pick}"
JOB_NAME="${JOB_NAME:-smolvla_green_block_pick}"
DEVICE="${DEVICE:-cuda}"
BATCH_SIZE="${BATCH_SIZE:-8}"
STEPS="${STEPS:-20000}"

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

if [[ "$DEVICE" == "cuda" ]]; then
    python - <<'PY'
import torch

if not torch.cuda.is_available():
    raise SystemExit(
        "CUDA is unavailable in this environment. Train on a CUDA machine or cloud GPU; "
        "SmolVLA training on this CPU-only WSL torch build is not practical."
    )
print("Training GPU:", torch.cuda.get_device_name(0))
PY
fi

exec lerobot-train \
    --policy.path=lerobot/smolvla_base \
    --dataset.repo_id="$DATASET_REPO_ID" \
    --batch_size="$BATCH_SIZE" \
    --steps="$STEPS" \
    --output_dir="$OUTPUT_DIR" \
    --job_name="$JOB_NAME" \
    --policy.device="$DEVICE" \
    --wandb.enable=false
