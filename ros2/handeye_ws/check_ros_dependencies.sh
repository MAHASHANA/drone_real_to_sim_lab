#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

required_packages=(
    hardware_interface
    controller_manager
    joint_trajectory_controller
    gripper_controllers
    joint_state_broadcaster
    moveit_core
    moveit_ros_planning_interface
    control_toolbox
    launch_param_builder
    rmw_cyclonedds_cpp
)

missing=()
for package in "${required_packages[@]}"; do
    if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
        missing+=("${package}")
    fi
done

if ((${#missing[@]})); then
    printf 'Missing ROS 2 package: %s\n' "${missing[@]}" >&2
    cat >&2 <<'EOF'

Install the complete SO101 ROS 2 build dependencies with:
  sudo apt-get update
  sudo apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-launch-param-builder
EOF
    exit 1
fi

echo "SO101 ROS 2 build dependencies are installed."
