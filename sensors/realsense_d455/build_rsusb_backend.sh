#!/usr/bin/env bash

set -euo pipefail

source_dir="${1:-${HOME}/librealsense}"
build_dir="${RSUSB_BUILD_DIR:-${source_dir}/build-rsusb}"
install_dir="${RSUSB_INSTALL_DIR:-${HOME}/.local/realsense-rsusb}"

if [[ ! -f "${source_dir}/CMakeLists.txt" ]]; then
    printf 'librealsense source not found at %s\n' "${source_dir}" >&2
    printf 'Pass the source checkout as the first argument.\n' >&2
    exit 1
fi

cmake \
    -S "${source_dir}" \
    -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${install_dir}" \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF

cmake --build "${build_dir}" --parallel

printf '\nRSUSB backend built in %s/Release\n' "${build_dir}"
printf 'No system libraries were replaced.\n'
