#!/usr/bin/env bash

set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/.venv-lerobot}"
LEADER_PORT="${LEADER_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E113301-if00}"
FOLLOWER_PORT="${FOLLOWER_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E115987-if00}"
LEADER_CALIBRATION="${HOME}/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_awesome_leader_arm.json"
FOLLOWER_CALIBRATION="${HOME}/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json"
RSUSB_BINDING="${RSUSB_BINDING:-$HOME/librealsense/build-rsusb/Release/pyrealsense2.cpython-310-x86_64-linux-gnu.so}"

failures=0

check_path() {
    local label="$1"
    local path="$2"
    if [[ -e "$path" ]]; then
        printf 'OK      %-22s %s\n' "$label" "$path"
    else
        printf 'MISSING %-22s %s\n' "$label" "$path"
        failures=$((failures + 1))
    fi
}

check_path "LeRobot environment" "$VENV_DIR/bin/activate"
check_path "leader serial port" "$LEADER_PORT"
check_path "follower serial port" "$FOLLOWER_PORT"
check_path "leader calibration" "$LEADER_CALIBRATION"
check_path "follower calibration" "$FOLLOWER_CALIBRATION"
check_path "RSUSB Python binding" "$RSUSB_BINDING"

if [[ -f "$VENV_DIR/bin/activate" ]]; then
    # shellcheck disable=SC1090
    source "$VENV_DIR/bin/activate"
    python - <<'PY'
import importlib.metadata
import importlib.util

print(f"OK      {'LeRobot version':22s} {importlib.metadata.version('lerobot')}")

missing = []
for module in ("torch", "zmq"):
    installed = importlib.util.find_spec(module) is not None
    print(f"{'OK' if installed else 'MISSING':7s} {module:22s}")
    if not installed:
        missing.append(module)

for module in ("transformers", "accelerate"):
    installed = importlib.util.find_spec(module) is not None
    print(f"{'TRAIN' if installed else 'OPTIONAL':7s} {module:22s}")

if importlib.util.find_spec("torch"):
    import torch

    print(f"INFO    {'torch build':22s} {torch.__version__}")
    print(f"INFO    {'CUDA available':22s} {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"INFO    {'CUDA device':22s} {torch.cuda.get_device_name(0)}")

if missing:
    raise SystemExit(1)
PY
    python_status=$?
    if [[ $python_status -ne 0 ]]; then
        failures=$((failures + 1))
    fi
fi

if [[ $failures -ne 0 ]]; then
    cat <<'EOF'

Preflight failed. Install the missing LeRobot ZMQ camera dependency with:

  source .venv-lerobot/bin/activate
  python -m pip install 'pyzmq>=26,<27'

This check does not enable torque or communicate with the servo motors.
EOF
    exit 1
fi

echo
echo "Preflight passed. This check did not enable torque or move either arm."
