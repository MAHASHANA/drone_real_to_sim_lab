#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS and virtualenv generated setup scripts are not compatible with Bash
# nounset mode. Preserve the caller's setting and restore it after activation.
case "$-" in
    *u*) HAND_EYE_RESTORE_NOUNSET=1; set +u ;;
    *) HAND_EYE_RESTORE_NOUNSET=0 ;;
esac

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/.venv/bin/activate"
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/install/setup.bash"

HAND_EYE_SITE_PACKAGES="$(
    "${WORKSPACE_DIR}/.venv/bin/python" \
        -c 'import site; print(site.getsitepackages()[0])'
)"
export LECONDA_SITE_PACKAGES="${HAND_EYE_SITE_PACKAGES}"
export PYTHONPATH="${HAND_EYE_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"

if [[ "${HAND_EYE_RESTORE_NOUNSET}" == "1" ]]; then
    set -u
fi
unset HAND_EYE_RESTORE_NOUNSET HAND_EYE_SITE_PACKAGES
