#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RSUSB_DIR="${RSUSB_DIR:-$HOME/librealsense/build-rsusb/Release}"

if [[ ! -f "$RSUSB_DIR/pyrealsense2.cpython-310-x86_64-linux-gnu.so" ]]; then
    echo "Missing RSUSB Python 3.10 binding under $RSUSB_DIR" >&2
    exit 1
fi

export PYTHONPATH="$RSUSB_DIR"
export LD_LIBRARY_PATH="$RSUSB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec /usr/bin/python3 "$ROOT_DIR/sensors/realsense_d455/lerobot_zmq_rgb_server.py" "$@"
