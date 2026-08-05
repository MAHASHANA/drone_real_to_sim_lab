#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="${WORKSPACE_DIR}/src"
VENDOR_DIR="${WORKSPACE_DIR}/vendor"
VENV_DIR="${WORKSPACE_DIR}/.venv"

EASY_HANDEYE_COMMIT="0ad1ddddbbc9a3cc7d3695ad53d04f1a5b82e598"
SO101_ROS2_COMMIT="b50872d2ba03a3d67fb9a9a7b59af1e72c7040b3"
LEROBOT_COMMIT="8a915c6b6fda506c0a073c495c2971b8de6ffdfa"

clone_at_commit() {
    local url="$1"
    local destination="$2"
    local commit="$3"

    if [[ ! -d "${destination}/.git" ]]; then
        git clone "${url}" "${destination}"
    fi
    git -C "${destination}" fetch origin "${commit}"
    git -C "${destination}" checkout --detach "${commit}"
}

mkdir -p "${SRC_DIR}" "${VENDOR_DIR}"
clone_at_commit \
    "https://github.com/marcoesposito1988/easy_handeye2.git" \
    "${SRC_DIR}/easy_handeye2" \
    "${EASY_HANDEYE_COMMIT}"
clone_at_commit \
    "https://github.com/nimiCurtis/so101_ros2.git" \
    "${SRC_DIR}/so101_ros2" \
    "${SO101_ROS2_COMMIT}"
clone_at_commit \
    "https://github.com/nimiCurtis/lerobot.git" \
    "${VENDOR_DIR}/lerobot" \
    "${LEROBOT_COMMIT}"

LEROBOT_PATCH="${WORKSPACE_DIR}/patches/lerobot-motors-no-ml-utils.patch"
if git -C "${VENDOR_DIR}/lerobot" apply --reverse --check "${LEROBOT_PATCH}" 2>/dev/null; then
    echo "LeRobot motor-only compatibility patch already applied."
else
    git -C "${VENDOR_DIR}/lerobot" apply "${LEROBOT_PATCH}"
fi

EASY_HANDEYE_PATCH="${WORKSPACE_DIR}/patches/easy-handeye2-ros-humble.patch"
if git -C "${SRC_DIR}/easy_handeye2" apply --reverse --check "${EASY_HANDEYE_PATCH}" 2>/dev/null; then
    echo "easy_handeye2 ROS Humble compatibility patch already applied."
else
    git -C "${SRC_DIR}/easy_handeye2" apply "${EASY_HANDEYE_PATCH}"
fi

SO101_ROS2_PATCH="${WORKSPACE_DIR}/patches/so101-bridge-max-relative-float.patch"
if git -C "${SRC_DIR}/so101_ros2" apply --reverse --check "${SO101_ROS2_PATCH}" 2>/dev/null; then
    echo "so101_ros2 max-relative-target compatibility patch already applied."
else
    git -C "${SRC_DIR}/so101_ros2" apply "${SO101_ROS2_PATCH}"
fi

SO101_HUMBLE_PATCH="${WORKSPACE_DIR}/patches/so101-teleoperate-ros-humble.patch"
if grep -q '^def equals(value, expected):' \
    "${SRC_DIR}/so101_ros2/so101_bringup/launch/so101_teleoperate.launch.py"; then
    echo "so101_ros2 ROS Humble launch compatibility patch already applied."
else
    git -C "${SRC_DIR}/so101_ros2" apply "${SO101_HUMBLE_PATCH}"
fi

SO101_LOCAL_CONFIG_PATCH="${WORKSPACE_DIR}/patches/so101-local-hardware-config.patch"
if grep -q '5B8E113301' \
    "${SRC_DIR}/so101_ros2/so101_ros2_bridge/config/so101_leader_params.yaml" \
    && grep -q '^def both_equal' \
    "${SRC_DIR}/so101_ros2/so101_bringup/launch/so101_teleoperate.launch.py"; then
    echo "so101_ros2 local arm configuration patch already applied."
else
    git -C "${SRC_DIR}/so101_ros2" apply "${SO101_LOCAL_CONFIG_PATCH}"
fi

SO101_LAUNCH_NORMALIZATION_PATCH="${WORKSPACE_DIR}/patches/so101-launch-normalization-ros-humble.patch"
if grep -q "PythonExpression(\[\"'so101_\"" \
    "${SRC_DIR}/so101_ros2/so101_ros2_bridge/launch/so101_ros2_bridge.launch.py"; then
    echo "so101_ros2 ROS Humble substitution normalization patch already applied."
else
    git -C "${SRC_DIR}/so101_ros2" apply "${SO101_LAUNCH_NORMALIZATION_PATCH}"
fi

SO101_TELEOP_STABILITY_PATCH="${WORKSPACE_DIR}/patches/so101-teleop-stability.patch"
if grep -q 'last_gripper_goal_position_.has_value' \
    "${SRC_DIR}/so101_ros2/so101_teleop/src/leader_teleop_component.cpp"; then
    echo "so101_ros2 teleoperation stability patch already applied."
else
    git -C "${SRC_DIR}/so101_ros2" apply "${SO101_TELEOP_STABILITY_PATCH}"
fi

SO101_MOTION_OPT_IN_PATCH="${WORKSPACE_DIR}/patches/so101-motion-opt-in.patch"
if grep -q "'enable_teleop'" \
    "${SRC_DIR}/so101_ros2/so101_bringup/launch/so101_teleoperate.launch.py"; then
    echo "so101_ros2 motion opt-in patch already applied."
else
    git -C "${SRC_DIR}/so101_ros2" apply "${SO101_MOTION_OPT_IN_PATCH}"
fi

/bin/python3 -m venv --system-site-packages "${VENV_DIR}"
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python -m pip install \
    "setuptools>=71.0.0,<81.0.0" \
    "draccus==0.10.0" \
    "deepdiff>=7.0.1,<9.0.0" \
    "feetech-servo-sdk>=1.0.0,<2.0.0" \
    "transforms3d>=0.4.2,<0.5.0"
python -m pip uninstall --yes UNKNOWN >/dev/null 2>&1 || true
python -m pip install \
    --no-build-isolation \
    --no-deps \
    --force-reinstall \
    "${VENDOR_DIR}/lerobot"

echo
echo "Python dependencies are ready. Check ROS dependencies with:"
echo "  ${WORKSPACE_DIR}/check_ros_dependencies.sh"
echo "Then build with:"
echo "  ${WORKSPACE_DIR}/build.sh"
