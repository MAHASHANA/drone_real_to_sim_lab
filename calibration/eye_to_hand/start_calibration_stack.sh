#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

LEADER_PORT="${LEADER_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E113301-if00}"
FOLLOWER_PORT="${FOLLOWER_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E115987-if00}"
LEADER_ID="${LEADER_ID:-my_awesome_leader_arm}"
FOLLOWER_ID="${FOLLOWER_ID:-my_awesome_follower_arm}"
D455_PROFILE="${D455_PROFILE:-highres}"
D455_ENABLE_DEPTH="${D455_ENABLE_DEPTH:-false}"
RECORD_SESSION="${RECORD_SESSION:-false}"
ENABLE_WRIST_CAMERA="${ENABLE_WRIST_CAMERA:-false}"
WRIST_CAMERA_VID="${WRIST_CAMERA_VID:-05a3}"
WRIST_CAMERA_PID="${WRIST_CAMERA_PID:-9230}"
WRIST_CAMERA_WIDTH="${WRIST_CAMERA_WIDTH:-640}"
WRIST_CAMERA_HEIGHT="${WRIST_CAMERA_HEIGHT:-480}"
WRIST_CAMERA_FPS="${WRIST_CAMERA_FPS:-30}"
WRIST_CAMERA_FPS_VALUE="$(printf '%.1f' "${WRIST_CAMERA_FPS}")"
WAIT_TIMEOUT_S="${WAIT_TIMEOUT_S:-25}"

LEADER_CALIBRATION_DIR="${HOME}/.cache/huggingface/lerobot/calibration/teleoperators/so_leader"
FOLLOWER_CALIBRATION_DIR="${HOME}/.cache/huggingface/lerobot/calibration/robots/so_follower"
LEADER_CALIBRATION="${LEADER_CALIBRATION_DIR}/${LEADER_ID}.json"
FOLLOWER_CALIBRATION="${FOLLOWER_CALIBRATION_DIR}/${FOLLOWER_ID}.json"
LOG_DIR="${REPO_DIR}/captures/calibration_stack_$(date +%Y%m%d_%H%M%S)"

declare -a CHILD_PIDS=()

fail() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

for setting_name in D455_ENABLE_DEPTH RECORD_SESSION ENABLE_WRIST_CAMERA; do
    setting_value="${!setting_name}"
    case "${setting_value}" in
        true | false) ;;
        *) fail "${setting_name} must be true or false, got: ${setting_value}" ;;
    esac
done

cleanup() {
    local pid
    trap - EXIT INT TERM
    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 -- "-${pid}" 2>/dev/null; then
            kill -INT -- "-${pid}" 2>/dev/null || true
        fi
    done
    sleep 1
    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 -- "-${pid}" 2>/dev/null; then
            kill -TERM -- "-${pid}" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_background() {
    local name="$1"
    shift
    printf 'Starting %s; log: %s/%s.log\n' "${name}" "${LOG_DIR}" "${name}"
    setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
    CHILD_PIDS+=("$!")
}

wait_for_topic() {
    local topic="$1"
    local label="$2"
    local deadline
    local remaining

    printf 'Waiting for %s on %s...\n' "${label}" "${topic}"
    deadline=$((SECONDS + WAIT_TIMEOUT_S))
    while ((SECONDS < deadline)); do
        if ros2 topic list 2>/dev/null | grep -Fx "${topic}" >/dev/null; then
            remaining=$((deadline - SECONDS))
            if ((remaining > 0)) \
                && timeout "${remaining}" ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
                printf '%s is streaming.\n' "${label}"
                return
            fi
            break
        fi
        sleep 0.25
    done
    fail "No ${label} message arrived within ${WAIT_TIMEOUT_S}s. Check ${LOG_DIR}."
}

find_wrist_camera() {
    local device properties

    if [[ -n "${WRIST_VIDEO_DEVICE:-}" ]]; then
        [[ -e "${WRIST_VIDEO_DEVICE}" ]] || fail "WRIST_VIDEO_DEVICE does not exist: ${WRIST_VIDEO_DEVICE}"
        printf '%s\n' "${WRIST_VIDEO_DEVICE}"
        return
    fi

    shopt -s nullglob
    for device in /dev/video*; do
        properties="$(udevadm info --query=property --name="${device}" 2>/dev/null || true)"
        if ! grep -qx "ID_VENDOR_ID=${WRIST_CAMERA_VID}" <<<"${properties}"; then
            continue
        fi
        if ! grep -qx "ID_MODEL_ID=${WRIST_CAMERA_PID}" <<<"${properties}"; then
            continue
        fi
        if v4l2-ctl --device="${device}" --all 2>/dev/null | grep "Video Capture" >/dev/null; then
            printf '%s\n' "${device}"
            return
        fi
    done
    fail "Could not find a capture node for USB camera ${WRIST_CAMERA_VID}:${WRIST_CAMERA_PID}"
}

usb_device_is_attached() {
    local vendor_id="$1"
    local product_id="$2"
    local vendor_file device_dir

    for vendor_file in /sys/bus/usb/devices/*/idVendor; do
        [[ -f "${vendor_file}" ]] || continue
        device_dir="${vendor_file%/idVendor}"
        if [[ "$(<"${vendor_file}")" == "${vendor_id}" ]] \
            && [[ -f "${device_dir}/idProduct" ]] \
            && [[ "$(<"${device_dir}/idProduct")" == "${product_id}" ]]; then
            return 0
        fi
    done
    return 1
}

[[ -e "${LEADER_PORT}" ]] || fail "Leader serial port is missing: ${LEADER_PORT}"
[[ -e "${FOLLOWER_PORT}" ]] || fail "Follower serial port is missing: ${FOLLOWER_PORT}"
[[ -f "${LEADER_CALIBRATION}" ]] || fail "Leader calibration is missing: ${LEADER_CALIBRATION}"
[[ -f "${FOLLOWER_CALIBRATION}" ]] || fail "Follower calibration is missing: ${FOLLOWER_CALIBRATION}"
[[ -f "${REPO_DIR}/ros2/handeye_ws/install/setup.bash" ]] || \
    fail "Hand-eye workspace is not built. Run ros2/handeye_ws/build.sh first."
usb_device_is_attached 8086 0b5c || \
    fail "Intel RealSense D455 8086:0b5c is not attached to WSL"

if fuser "${LEADER_PORT}" "${FOLLOWER_PORT}" >/dev/null 2>&1; then
    fuser -v "${LEADER_PORT}" "${FOLLOWER_PORT}" >&2 || true
    fail "An existing process owns an arm serial port. Stop LeRobot teleoperation first."
fi

if [[ "${RECORD_SESSION}" == "true" ]]; then
    D455_ENABLE_DEPTH=true
    ENABLE_WRIST_CAMERA=true
fi

if [[ "${ENABLE_WRIST_CAMERA}" == "true" ]]; then
    WRIST_VIDEO_DEVICE="$(find_wrist_camera)"
else
    WRIST_VIDEO_DEVICE="disabled"
fi

mkdir -p "${LOG_DIR}"

# shellcheck disable=SC1091
source "${REPO_DIR}/ros2/handeye_ws/env.sh"

command -v ros2 >/dev/null || fail "ros2 is unavailable after sourcing the hand-eye environment"
if [[ "${ENABLE_WRIST_CAMERA}" == "true" ]]; then
    ros2 pkg executables | grep -Fx 'usb_cam usb_cam_node_exe' >/dev/null || \
        fail "ROS usb_cam is missing. Install it with: sudo apt-get install ros-humble-usb-cam"
fi
ros2 pkg executables | grep -Fx 'drone_handeye_calibration so101_telemetry_publisher' >/dev/null || \
    fail "The hand-eye workspace is stale. Rebuild it with ros2/handeye_ws/build.sh."

cat <<EOF
Calibration stack preflight passed.

  leader:       ${LEADER_PORT}
  follower:     ${FOLLOWER_PORT}
  D455 profile: ${D455_PROFILE} (depth=${D455_ENABLE_DEPTH}, point cloud disabled)
  wrist camera: ${WRIST_VIDEO_DEVICE}
  record bag:   ${RECORD_SESSION}
  logs:         ${LOG_DIR}

The fixed D455 is the eye-to-hand calibration camera. The wrist camera is
optional and is not used by the calibration solver.

Clamp both arm bases, clear the entire follower sweep, keep its power switch
accessible, and manually place the leader and follower in closely matching
poses. START authorizes direct LeRobot leader-to-follower mirroring.
EOF

start_background \
    d455 \
    env D455_PROFILE="${D455_PROFILE}" \
        D455_ENABLE_DEPTH="${D455_ENABLE_DEPTH}" \
        D455_POINTCLOUD=false \
    "${REPO_DIR}/sensors/realsense_d455/launch_d455_ros2.sh" \
    --profile "${D455_PROFILE}"

wait_for_topic "/camera/camera/color/image_raw" "D455 color"
wait_for_topic "/camera/camera/color/camera_info" "D455 camera info"

if [[ "${ENABLE_WRIST_CAMERA}" == "true" ]]; then
    start_background \
        wrist_camera \
        ros2 run usb_cam usb_cam_node_exe --ros-args \
        -r __node:=wrist_camera \
        -r image_raw:=/wrist_camera/image_raw \
        -r camera_info:=/wrist_camera/camera_info \
        -p video_device:="${WRIST_VIDEO_DEVICE}" \
        -p framerate:="${WRIST_CAMERA_FPS_VALUE}" \
        -p io_method:="mmap" \
        -p pixel_format:="mjpeg2rgb" \
        -p av_device_format:="YUV422P" \
        -p image_width:="${WRIST_CAMERA_WIDTH}" \
        -p image_height:="${WRIST_CAMERA_HEIGHT}" \
        -p camera_name:="wrist_camera" \
        -p frame_id:="wrist_camera_optical_frame"

    wait_for_topic "/wrist_camera/image_raw" "wrist camera"
fi

if [[ "${RECORD_SESSION}" == "true" ]]; then
    # Record compressed camera transports to avoid writing high-resolution raw
    # RGB-D at more than 100 MB/s during calibration.
    start_background \
        d455_color_compressor \
        ros2 run image_transport republish raw compressed --ros-args \
        -r in:=/camera/camera/color/image_raw \
        -r out/compressed:=/recording/d455/color/compressed

    start_background \
        d455_depth_compressor \
        ros2 run image_transport republish raw compressedDepth --ros-args \
        -r in:=/camera/camera/aligned_depth_to_color/image_raw \
        -r out/compressedDepth:=/recording/d455/aligned_depth/compressedDepth

    start_background \
        wrist_color_compressor \
        ros2 run image_transport republish raw compressed --ros-args \
        -r in:=/wrist_camera/image_raw \
        -r out/compressed:=/recording/wrist/color/compressed

    wait_for_topic "/recording/d455/color/compressed" "compressed D455 color"
    wait_for_topic "/recording/d455/aligned_depth/compressedDepth" "compressed D455 depth"
    wait_for_topic "/recording/wrist/color/compressed" "compressed wrist color"

    start_background \
        rosbag \
        ros2 bag record \
        --output "${LOG_DIR}/session_bag" \
        /recording/d455/color/compressed \
        /recording/d455/aligned_depth/compressedDepth \
        /recording/wrist/color/compressed \
        /camera/camera/color/camera_info \
        /camera/camera/aligned_depth_to_color/camera_info \
        /wrist_camera/camera_info \
        /leader/joint_states \
        /follower/joint_states \
        /so101/control_status \
        /tf \
        /tf_static
fi

printf '\nCamera preflight passed. No follower command has been sent.\n'
read -r -p "Type START to enable the follower and open calibration: " confirmation
[[ "${confirmation}" == "START" ]] || fail "Startup cancelled"

printf '\nStarting ROS telemetry, ChArUco tracking, and easy_handeye2.\n'

setsid ros2 launch drone_handeye_calibration calibrate.launch.py \
    > >(tee "${LOG_DIR}/calibration.log") 2>&1 &
CALIBRATION_PID="$!"
CHILD_PIDS+=("${CALIBRATION_PID}")

sleep 2
printf '\nStarting direct LeRobot mirroring.\n\n'
setsid env -u PYTHONPATH \
    "${REPO_DIR}/.venv-lerobot/bin/python" -u \
    "${REPO_DIR}/hardware/so101/leader_follower_bridge.py" \
    --leader-port "${LEADER_PORT}" \
    --follower-port "${FOLLOWER_PORT}" \
    --leader-id "${LEADER_ID}" \
    --follower-id "${FOLLOWER_ID}" \
    --enable-motion \
    --yes \
    > >(tee "${LOG_DIR}/so101_control.log") 2>&1 &
CONTROL_PID="$!"
CHILD_PIDS+=("${CONTROL_PID}")

wait_for_topic "/follower/joint_states" "follower joint states"
wait_for_topic "/so101/control_status" "SO-101 control status"

while kill -0 "${CALIBRATION_PID}" 2>/dev/null \
    && kill -0 "${CONTROL_PID}" 2>/dev/null; do
    sleep 0.5
done

if ! kill -0 "${CONTROL_PID}" 2>/dev/null \
    && kill -0 "${CALIBRATION_PID}" 2>/dev/null; then
    CONTROL_STATUS=0
    wait "${CONTROL_PID}" || CONTROL_STATUS="$?"
    fail "SO-101 controller exited unexpectedly with status ${CONTROL_STATUS}; calibration stopped"
fi

wait "${CALIBRATION_PID}"
