#!/usr/bin/env bash

set -euo pipefail

ros_distro="${ROS_DISTRO:-humble}"
ros_setup="${ROS_SETUP:-/opt/ros/${ros_distro}/setup.bash}"
rsusb_lib_dir="${RSUSB_LIB_DIR:-${HOME}/librealsense/build-rsusb/Release}"

if [[ ! -f "${ros_setup}" ]]; then
    printf 'ROS2 setup file not found: %s\n' "${ros_setup}" >&2
    exit 1
fi

if [[ ! -e "${rsusb_lib_dir}/librealsense2.so.2.55" ]]; then
    printf 'RSUSB librealsense library not found in %s\n' "${rsusb_lib_dir}" >&2
    printf 'Run sensors/realsense_d455/build_rsusb_backend.sh first.\n' >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

# ROS2's RealSense wrapper loads this ABI-compatible userspace backend at run time.
export LD_LIBRARY_PATH="${rsusb_lib_dir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

exec ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    rgb_camera.color_profile:=424x240x5 \
    rgb_camera.color_format:=RGB8 \
    enable_depth:=true \
    depth_module.depth_profile:=480x270x5 \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    "$@"
