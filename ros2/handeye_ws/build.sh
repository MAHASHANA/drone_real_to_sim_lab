#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${WORKSPACE_DIR}/../.." && pwd)"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

"${WORKSPACE_DIR}/check_ros_dependencies.sh"

export LECONDA_SITE_PACKAGES="$(
    "${WORKSPACE_DIR}/.venv/bin/python" \
        -c 'import site; print(site.getsitepackages()[0])'
)"
# colcon's Python install/uninstall commands require Ubuntu's bundled setuptools.
export PYTHONNOUSERSITE=1

cd "${WORKSPACE_DIR}"
colcon build \
    --base-paths \
        "${WORKSPACE_DIR}/src/easy_handeye2/easy_handeye2_msgs" \
        "${WORKSPACE_DIR}/src/easy_handeye2/easy_handeye2" \
        "${WORKSPACE_DIR}/src/so101_ros2/so101_description" \
        "${WORKSPACE_DIR}/src/so101_ros2/so101_hardware_interface" \
        "${WORKSPACE_DIR}/src/so101_ros2/so101_controller" \
        "${WORKSPACE_DIR}/src/so101_ros2/so101_ros2_bridge" \
        "${WORKSPACE_DIR}/src/so101_ros2/so101_teleop" \
        "${WORKSPACE_DIR}/src/so101_ros2/so101_bringup" \
        "${WORKSPACE_DIR}/src/so101_ros2/so101_ros2" \
        "${REPO_DIR}/calibration/eye_to_hand" \
    --packages-select \
        easy_handeye2_msgs \
        easy_handeye2 \
        so101_description \
        so101_hardware_interface \
        so101_controller \
        so101_ros2_bridge \
        so101_teleop \
        so101_bringup \
        so101_ros2 \
        drone_handeye_calibration
