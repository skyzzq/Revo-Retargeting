#!/usr/bin/env bash
# Record teleop stage timing and physical joint state.
# Does not start driver/teleop — run those first.
set -euo pipefail

MODE="${1:-both}"
if [[ $# -gt 0 ]]; then
  shift
fi

if [[ "${MODE}" != "left" && "${MODE}" != "right" && "${MODE}" != "both" ]]; then
  echo "Usage: probe_teleop.sh [left|right|both] [--duration 10] [--out DIR] [--no-traces]" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${WORKSPACE}/install/setup.bash"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/revo3_common.sh"

if [[ ! -f "${SETUP}" ]]; then
  echo "[probe_teleop] Missing ${SETUP}. Build the workspace first." >&2
  exit 1
fi

revo3_maybe_activate_conda
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
# shellcheck source=/dev/null
source "${SETUP}"
set -u

echo "[probe_teleop] hand_mode=${MODE}"
exec ros2 run manus_revo3_retarget teleop_probe --hand-mode "${MODE}" "$@"
