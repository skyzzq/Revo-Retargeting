#!/usr/bin/env bash
# Install a MANUS dongle .lic into the path expected by the official SDK Client,
# then optionally push it onto the connected dongle via manus_calibration_tool.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_LIC="${1:-}"
DEST_DIR="${HOME}/Documents/manus-licenses"
DEST_LIC="${DEST_DIR}/license.lic"
APPLY="${APPLY_LICENSE:-1}"

if [[ -z "${SRC_LIC}" ]]; then
  cat >&2 <<EOF
Usage: $0 /path/to/XXXX.lic

Copies the license to:
  ${DEST_LIC}

Then (if install/setup.bash exists) runs:
  ros2 run manus_ros2 manus_calibration_tool --set-license ${DEST_LIC} --list
EOF
  exit 1
fi

if [[ ! -f "${SRC_LIC}" ]]; then
  echo "[manus_license] License file not found: ${SRC_LIC}" >&2
  exit 1
fi

mkdir -p "${DEST_DIR}"
cp -f "${SRC_LIC}" "${DEST_LIC}"
chmod 600 "${DEST_LIC}"
echo "[manus_license] Installed ${SRC_LIC} -> ${DEST_LIC}"

if [[ "${APPLY}" != "1" ]]; then
  exit 0
fi

SETUP="${WORKSPACE}/install/setup.bash"
if [[ ! -f "${SETUP}" ]]; then
  echo "[manus_license] Workspace not built yet; file installed. Rebuild and apply with:"
  echo "  source ${SETUP}"
  echo "  ros2 run manus_ros2 manus_calibration_tool --set-license ${DEST_LIC} --list"
  exit 0
fi

# shellcheck disable=SC1090
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${SETUP}"
set -u

echo "[manus_license] Applying license to connected dongle..."
ros2 run manus_ros2 manus_calibration_tool --set-license "${DEST_LIC}" --list --wait-seconds 30
