#!/usr/bin/env bash
# Robust Revo3 driver start.
#
# Successful dual-hand bring-up lessons:
# - Kill leftovers / free /dev/ttyUSB* before launch.
# - Prefer sequential left -> activate MIT -> right -> activate MIT
#   (parallel dual launch often starves controller_manager spawners).
# - Spawners may time out after load; activate via activate_revo3_controllers.py.
# - Require non-empty SN in logs (left BCUBL..., right BCUBR...) before "ready".
# - Manual MIT commands need --rate (not --once); command_timeout_sec≈0.25s.
set -euo pipefail

MODE="${1:-right}"
if [[ $# -gt 0 ]]; then
  shift
fi

if [[ "${MODE}" != "left" && "${MODE}" != "right" && "${MODE}" != "both" ]]; then
  echo "Usage: start_revo3_driver.sh [left|right|both] [extra ros2 launch args...]" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${WORKSPACE}/install/setup.bash"
ACTIVATE_PY="${SCRIPT_DIR}/activate_revo3_controllers.py"
STOP_SH="${SCRIPT_DIR}/stop_revo3.sh"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/revo3_common.sh"

if [[ ! -f "${SETUP}" ]]; then
  echo "[revo3_driver] Missing ${SETUP}. Run python -m colcon build --symlink-install first." >&2
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
REVO3_PYTHON="$(revo3_resolve_python)"

REVO3_LAUNCH_RVIZ="${REVO3_LAUNCH_RVIZ:-false}"
# Teleop does not consume TF. Keep RSP off unless RViz (or an explicit override) needs it.
if [[ -z "${REVO3_LAUNCH_RSP:-}" ]]; then
  if [[ "${REVO3_LAUNCH_RVIZ}" == "true" || "${REVO3_LAUNCH_RVIZ}" == "1" ]]; then
    REVO3_LAUNCH_RSP="true"
  else
    REVO3_LAUNCH_RSP="false"
  fi
fi
REVO3_UPDATE_RATE="${REVO3_UPDATE_RATE:-200}"
# both start mode:
#   sequential (default) — left SN, then right SN, then one MIT activate (most reliable)
#   overlap — launch nearly together (faster, can starve CM under load)
#   dual — dual_revo3_system.launch.py
REVO3_START_MODE="${REVO3_START_MODE:-sequential}"
# Backward compat: REVO3_SEQUENTIAL_HANDS=1 → sequential; =0 → dual
if [[ -n "${REVO3_SEQUENTIAL_HANDS:-}" ]]; then
  if [[ "${REVO3_SEQUENTIAL_HANDS}" == "1" ]]; then
    REVO3_START_MODE="sequential"
  else
    REVO3_START_MODE="dual"
  fi
fi
REVO3_CLEAN_START="${REVO3_CLEAN_START:-1}"
REVO3_ACTIVATE_MIT="${REVO3_ACTIVATE_MIT:-1}"
REVO3_SN_TIMEOUT_SEC="${REVO3_SN_TIMEOUT_SEC:-30}"
REVO3_ACTIVATE_WAIT_SEC="${REVO3_ACTIVATE_WAIT_SEC:-120}"
REVO3_ACTIVATE_TIMEOUT_SEC="${REVO3_ACTIVATE_TIMEOUT_SEC:-45}"
REVO3_SPAWN_AUX="${REVO3_SPAWN_AUX:-false}"
REVO3_HAND_STAGGER_SEC="${REVO3_HAND_STAGGER_SEC:-3}"

LOG_DIR="${REVO3_DRIVER_LOG_DIR:-/tmp}"
LEFT_LOG="${LOG_DIR}/revo3_driver_left.log"
RIGHT_LOG="${LOG_DIR}/revo3_driver_right.log"
DUAL_LOG="${LOG_DIR}/revo3_driver_both.log"

managed_pids=()

cleanup() {
  trap - EXIT INT TERM
  local entry pgid
  for ((idx=${#managed_pids[@]}-1; idx>=0; idx--)); do
    entry="${managed_pids[$idx]}"
    pgid="${entry%%:*}"
    if kill -0 "-${pgid}" 2>/dev/null; then
      kill -INT "-${pgid}" 2>/dev/null || true
    fi
  done
  sleep 0.5
  for ((idx=${#managed_pids[@]}-1; idx>=0; idx--)); do
    entry="${managed_pids[$idx]}"
    pgid="${entry%%:*}"
    if kill -0 "-${pgid}" 2>/dev/null; then
      kill -TERM "-${pgid}" 2>/dev/null || true
    fi
  done
  sleep 0.3
  for ((idx=${#managed_pids[@]}-1; idx>=0; idx--)); do
    entry="${managed_pids[$idx]}"
    pgid="${entry%%:*}"
    if kill -0 "-${pgid}" 2>/dev/null; then
      kill -KILL "-${pgid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

start_bg() {
  local label="$1"
  local logfile="$2"
  shift 2
  echo "[revo3_driver] Starting ${label} -> ${logfile}"
  : > "${logfile}"
  setsid "$@" >"${logfile}" 2>&1 &
  local pid=$!
  managed_pids+=("${pid}:${label}")
}

require_ports() {
  if [[ ! -e /dev/ttyUSB0 && ! -e /dev/ttyUSB1 && \
        ! -e /dev/revo3_hand_left && ! -e /dev/revo3_hand_right ]]; then
    echo "[revo3_driver] ERROR: no Revo3 serial devices. Plug USB-RS485 and/or run ./scripts/bind_revo3_ports.sh" >&2
    exit 1
  fi
  if [[ "${MODE}" == "left" || "${MODE}" == "both" ]]; then
    [[ -e /dev/revo3_hand_left ]] || echo "[revo3_driver] WARN: /dev/revo3_hand_left missing"
  fi
  if [[ "${MODE}" == "right" || "${MODE}" == "both" ]]; then
    [[ -e /dev/revo3_hand_right ]] || echo "[revo3_driver] WARN: /dev/revo3_hand_right missing"
  fi
  if fuser /dev/ttyUSB0 /dev/ttyUSB1 >/dev/null 2>&1; then
    echo "[revo3_driver] WARN: serial ports still held after clean:"
    fuser -v /dev/ttyUSB0 /dev/ttyUSB1 2>&1 || true
  fi
}

wait_sn() {
  local side="$1"
  local logfile="$2"
  local expect
  if [[ "${side}" == "left" ]]; then
    expect='sn=BCUBL'
  else
    expect='sn=BCUBR'
  fi
  local sn
  local start="${SECONDS}"
  local elapsed=0
  while (( elapsed < REVO3_SN_TIMEOUT_SEC )); do
    if sn="$(grep -oE "${expect}[A-Z0-9]+" "${logfile}" 2>/dev/null | tail -1)" && [[ -n "${sn}" ]]; then
      echo "[revo3_driver] ${side} device OK (${sn})"
      return 0
    fi
    if grep -qE 'Revo3 device info:.*sn=$' "${logfile}" 2>/dev/null; then
      if ! grep -qE "${expect}[A-Z0-9]+" "${logfile}" 2>/dev/null && (( elapsed >= 8 )); then
        echo "[revo3_driver] ERROR: ${side} device info has empty sn=. Check power/cable/port binding. See ${logfile}" >&2
        return 1
      fi
    fi
    if grep -q 'on_error invoked' "${logfile}" 2>/dev/null \
      && ! grep -qE "${expect}[A-Z0-9]+" "${logfile}" 2>/dev/null \
      && (( elapsed >= 10 )); then
      echo "[revo3_driver] ERROR: ${side} on_error without valid SN. See ${logfile}" >&2
      return 1
    fi
    sleep 0.2
    elapsed=$((SECONDS - start))
  done
  echo "[revo3_driver] ERROR: ${side} SN not seen within ${REVO3_SN_TIMEOUT_SEC}s (${logfile})" >&2
  return 1
}

launch_one_side() {
  local side="$1"
  local logfile="$2"
  shift 2
  local launch_args=(
    "hand_side:=${side}"
    "if_sim:=false"
    "launch_rsp:=${REVO3_LAUNCH_RSP}"
    "launch_rviz:=${REVO3_LAUNCH_RVIZ}"
    "update_rate:=${REVO3_UPDATE_RATE}"
    "spawn_aux_controllers:=${REVO3_SPAWN_AUX}"
  )
  local upper
  upper="$(echo "${side}" | tr '[:lower:]' '[:upper:]')"
  local config_var="REVO3_${upper}_PROTOCOL_CONFIG"
  if [[ -n "${!config_var:-}" ]]; then
    launch_args+=("protocol_config_file:=${!config_var}")
  elif [[ -n "${REVO3_PROTOCOL_CONFIG:-}" ]]; then
    launch_args+=("protocol_config_file:=${REVO3_PROTOCOL_CONFIG}")
  fi
  start_bg "${side} driver" "${logfile}" \
    ros2 launch revo3_driver revo3_system.launch.py "${launch_args[@]}" "$@"
}

activate_sides() {
  local mode="$1"
  if [[ "${REVO3_ACTIVATE_MIT}" != "1" ]]; then
    return 0
  fi
  echo "[revo3_driver] Ensuring joint_state + MIT active (${mode})..."
  if ! "${REVO3_PYTHON}" "${ACTIVATE_PY}" "${mode}" \
    --wait-loaded "${REVO3_ACTIVATE_WAIT_SEC}" \
    --timeout "${REVO3_ACTIVATE_TIMEOUT_SEC}" \
    --poll 0.25; then
    echo "[revo3_driver] ERROR: failed to activate MIT for ${mode}. See logs under ${LOG_DIR}." >&2
    return 1
  fi
}

wait_children() {
  echo "[revo3_driver] Ready. Ctrl-C to stop."
  while true; do
    local entry pid
    for entry in "${managed_pids[@]}"; do
      pid="${entry%%:*}"
      if ! kill -0 "-${pid}" 2>/dev/null; then
        echo "[revo3_driver] Process group ${entry#*:} exited."
        exit 1
      fi
    done
    sleep 1
  done
}

if [[ "${REVO3_CLEAN_START}" == "1" ]]; then
  echo "[revo3_driver] Cleaning previous driver processes..."
  REVO3_STOP_INTERNAL=1 bash "${STOP_SH}" driver || true
  sleep 0.2
fi

require_ports

if [[ "${MODE}" == "both" && "${REVO3_START_MODE}" == "overlap" ]]; then
  echo "[revo3_driver] Overlap both-hand start (stagger ${REVO3_HAND_STAGGER_SEC}s)"
  launch_one_side left "${LEFT_LOG}" "$@"
  sleep "${REVO3_HAND_STAGGER_SEC}"
  launch_one_side right "${RIGHT_LOG}" "$@"
  wait_sn left "${LEFT_LOG}"
  wait_sn right "${RIGHT_LOG}"
  activate_sides both
  wait_children
fi

if [[ "${MODE}" == "both" && "${REVO3_START_MODE}" == "sequential" ]]; then
  # Most reliable: fully bring up left (SN + MIT) before touching right.
  echo "[revo3_driver] Sequential both-hand start (left ready → right ready)"
  launch_one_side left "${LEFT_LOG}" "$@"
  wait_sn left "${LEFT_LOG}"
  activate_sides left

  launch_one_side right "${RIGHT_LOG}" "$@"
  wait_sn right "${RIGHT_LOG}"
  activate_sides right

  wait_children
fi

if [[ "${MODE}" == "both" ]]; then
  # dual launch file
  launch_args=(
    "if_sim:=false"
    "launch_rsp:=${REVO3_LAUNCH_RSP}"
    "launch_rviz:=${REVO3_LAUNCH_RVIZ}"
    "update_rate:=${REVO3_UPDATE_RATE}"
    "spawn_aux_controllers:=${REVO3_SPAWN_AUX}"
  )
  if [[ -n "${REVO3_LEFT_PROTOCOL_CONFIG:-}" ]]; then
    launch_args+=("left_protocol_config_file:=${REVO3_LEFT_PROTOCOL_CONFIG}")
  fi
  if [[ -n "${REVO3_RIGHT_PROTOCOL_CONFIG:-}" ]]; then
    launch_args+=("right_protocol_config_file:=${REVO3_RIGHT_PROTOCOL_CONFIG}")
  fi
  start_bg "both driver" "${DUAL_LOG}" \
    ros2 launch revo3_driver dual_revo3_system.launch.py "${launch_args[@]}" "$@"
  sleep 1
  wait_sn left "${DUAL_LOG}" || true
  wait_sn right "${DUAL_LOG}" || true
  activate_sides both
  wait_children
fi

logfile="${LEFT_LOG}"
[[ "${MODE}" == "right" ]] && logfile="${RIGHT_LOG}"
launch_one_side "${MODE}" "${logfile}" "$@"
wait_sn "${MODE}" "${logfile}"
activate_sides "${MODE}"
wait_children
