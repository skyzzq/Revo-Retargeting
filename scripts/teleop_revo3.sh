#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${WORKSPACE}/install/setup.bash"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/revo3_common.sh"

MODE=""
TASK="${TELEOP_TASK:-blocks}"
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    left|right|both)
      MODE="$1"
      shift
      ;;
    blocks|block|unbox|unboxing|express)
      TASK="$1"
      shift
      ;;
    *)
      EXTRA+=("$@")
      break
      ;;
  esac
done
MODE="${MODE:-both}"
set -- "${EXTRA[@]}"

if [[ "${MODE}" != "left" && "${MODE}" != "right" && "${MODE}" != "both" ]]; then
  echo "Usage: teleop_revo3.sh [left|right|both] [blocks|unbox] [extra manus_revo3_retarget launch args...]" >&2
  exit 1
fi
if ! TASK="$(revo3_normalize_task "${TASK}")"; then
  echo "Usage: teleop_revo3.sh [left|right|both] [blocks|unbox] [extra manus_revo3_retarget launch args...]" >&2
  echo "Unknown task: ${TASK}" >&2
  exit 1
fi
export TELEOP_TASK="${TASK}"

if [[ ! -f "${SETUP}" ]]; then
  echo "[teleop_revo3] Missing ${SETUP}. Run python -m colcon build --symlink-install first." >&2
  exit 1
fi

START_MANUS_PUBLISHER="${START_MANUS_PUBLISHER:-1}"
START_REVO3_DRIVER="${START_REVO3_DRIVER:-1}"
# When driver is external (START_REVO3_DRIVER=0), wait/ensure MIT is active first.
REVO3_WAIT_MIT="${REVO3_WAIT_MIT:-1}"
REVO3_ACTIVATE_TIMEOUT_SEC="${REVO3_ACTIVATE_TIMEOUT_SEC:-45}"
if [[ "${MODE}" == "both" ]]; then
  # Sequential left-then-right driver bring-up needs a longer MIT wait.
  REVO3_ACTIVATE_WAIT_SEC="${REVO3_ACTIVATE_WAIT_SEC:-180}"
else
  REVO3_ACTIVATE_WAIT_SEC="${REVO3_ACTIVATE_WAIT_SEC:-90}"
fi
ACTIVATE_PY="${SCRIPT_DIR}/activate_revo3_controllers.py"

revo3_maybe_activate_conda
set +u
# Ensure ROS underlay is present (workspace setup alone is not enough from a bare shell).
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
# shellcheck source=/dev/null
source "${SETUP}"
set -u
REVO3_PYTHON="$(revo3_resolve_python)"

managed_pids=()

start_managed() {
  local label="$1"
  shift
  echo "[teleop_revo3] Starting ${label}..."
  setsid "$@" &
  local pid=$!
  managed_pids+=("${pid}:${label}")
}

signal_process_groups() {
  local signal="$1"
  local entry pgid label
  for ((idx=${#managed_pids[@]}-1; idx>=0; idx--)); do
    entry="${managed_pids[$idx]}"
    pgid="${entry%%:*}"
    label="${entry#*:}"
    if kill -0 "-${pgid}" 2>/dev/null; then
      echo "[teleop_revo3] Sending ${signal} to ${label}..."
      kill "-${signal}" "-${pgid}" 2>/dev/null || true
    fi
  done
}

wait_process_groups() {
  local attempts="$1"
  local entry pgid alive
  for ((attempt=0; attempt<attempts; attempt++)); do
    alive=0
    for entry in "${managed_pids[@]}"; do
      pgid="${entry%%:*}"
      if kill -0 "-${pgid}" 2>/dev/null; then
        alive=1
        break
      fi
    done
    if [[ "${alive}" == "0" ]]; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

reap_managed_pids() {
  local entry pid
  for entry in "${managed_pids[@]}"; do
    pid="${entry%%:*}"
    wait "${pid}" 2>/dev/null || true
  done
}

cleanup() {
  trap - EXIT
  trap '' INT TERM
  signal_process_groups INT
  wait_process_groups 30 || {
    signal_process_groups TERM
    wait_process_groups 20 || signal_process_groups KILL
  }
  reap_managed_pids
}

wait_for_any() {
  while true; do
    local entry pid
    for entry in "${managed_pids[@]}"; do
      pid="${entry%%:*}"
      if ! kill -0 "-${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return
      fi
    done
    sleep 0.2
  done
}

handle_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

echo "[teleop_revo3] mode=${MODE} task=${TASK}"
if [[ "${START_REVO3_DRIVER}" == "1" ]]; then
  start_managed "Revo3 driver" "${SCRIPT_DIR}/start_revo3_driver.sh" "${MODE}" "${TASK}"
fi

# Wait for MIT before MANUS Core / retarget. Starting all three at once
# contends CPU with controller_manager spawners and makes the first seconds stutter.
if [[ "${REVO3_WAIT_MIT}" == "1" ]]; then
  echo "[teleop_revo3] Waiting for MIT controllers (${MODE}) before teleop..."
  "${REVO3_PYTHON}" "${ACTIVATE_PY}" "${MODE}" \
    --wait-loaded "${REVO3_ACTIVATE_WAIT_SEC}" \
    --timeout "${REVO3_ACTIVATE_TIMEOUT_SEC}" \
    --poll 0.25
fi

if [[ "${START_MANUS_PUBLISHER}" == "1" ]]; then
  start_managed "MANUS publisher" ros2 run manus_ros2 manus_data_publisher
fi

start_managed "Revo3 retarget" ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:="${MODE}" \
  task:="${TASK}" \
  launch_manus_publisher:=false \
  "$@"

TELEOP_MONITOR="${TELEOP_MONITOR:-0}"
if [[ "${TELEOP_MONITOR}" == "1" ]]; then
  start_managed "teleop monitor" nice -n 15 "${REVO3_PYTHON}" "${SCRIPT_DIR}/teleop_monitor.py" \
    --hand-mode "${MODE}" --period "${TELEOP_MONITOR_PERIOD:-2}"
fi

ENABLE_KEYBOARD_ACTIONS="${ENABLE_KEYBOARD_ACTIONS:-0}"
if [[ "${ENABLE_KEYBOARD_ACTIONS}" == "1" && -t 0 ]]; then
  echo "[teleop_revo3] Keyboard actions: 1=open 2=fist 3=pinch 4=point 5=ok 0=glove  h=help"
  ros2 run manus_revo3_retarget keyboard_action --hand-mode "${MODE}"
else
  if [[ "${ENABLE_KEYBOARD_ACTIONS}" == "1" ]]; then
    echo "[teleop_revo3] stdin is not a TTY; keyboard actions disabled. Run:"
    echo "  ros2 run manus_revo3_retarget keyboard_action --hand-mode ${MODE}"
  fi
  wait_for_any
fi
