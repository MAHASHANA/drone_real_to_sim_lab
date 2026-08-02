#!/usr/bin/env bash

set -euo pipefail

ros_distro="${ROS_DISTRO:-humble}"
ros_setup="${ROS_SETUP:-/opt/ros/${ros_distro}/setup.bash}"
rsusb_lib_dir="${RSUSB_LIB_DIR:-${HOME}/librealsense/build-rsusb/Release}"
profile="${D455_PROFILE:-highres}"
pointcloud="${D455_POINTCLOUD:-false}"
laser_power="${D455_LASER_POWER:-360.0}"
allow_highres_pointcloud="${D455_ALLOW_HIGHRES_POINTCLOUD:-false}"

if [[ "${1:-}" == "--profile" ]]; then
    if [[ $# -lt 2 ]]; then
        printf '%s\n' "--profile requires a profile name" >&2
        exit 2
    fi
    profile="$2"
    shift 2
fi

case "${profile}" in
    highres)
        color_profile="1280x720x30"
        depth_profile="1280x720x30"
        ;;
    balanced)
        color_profile="848x480x30"
        depth_profile="848x480x30"
        ;;
    highspeed)
        color_profile="640x360x90"
        depth_profile="640x360x90"
        ;;
    low)
        color_profile="424x240x5"
        depth_profile="424x240x5"
        ;;
    *)
        printf 'Unknown D455 profile: %s\n' "${profile}" >&2
        printf '%s\n' "Expected one of: highres, balanced, highspeed, low" >&2
        exit 2
        ;;
esac

if [[ "${profile}" == "highres" && "${pointcloud}" == "true" && "${allow_highres_pointcloud}" != "true" ]]; then
    cat >&2 <<'EOF'
Refusing highres + pointcloud under the default WSL2/USB-IP path.
This combination produces multi-megabyte clouds and has caused severe stream
starvation and D455 hardware notifications. Use one of:

  sensors/realsense_d455/launch_d455_ros2.sh --profile highres
  D455_POINTCLOUD=true sensors/realsense_d455/launch_d455_ros2.sh --profile balanced

Set D455_ALLOW_HIGHRES_POINTCLOUD=true only for an explicit diagnostic test.
EOF
    exit 2
fi

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

printf 'Starting D455 profile=%s color=%s depth=%s pointcloud=%s laser=%s mW\n' \
    "${profile}" "${color_profile}" "${depth_profile}" "${pointcloud}" \
    "${laser_power}"

exec ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    rgb_camera.color_profile:="${color_profile}" \
    rgb_camera.color_format:=RGB8 \
    enable_depth:=true \
    depth_module.depth_profile:="${depth_profile}" \
    depth_module.emitter_enabled:=1 \
    depth_module.laser_power:="${laser_power}" \
    pointcloud.enable:="${pointcloud}" \
    align_depth.enable:=true \
    enable_sync:=true \
    "$@"
