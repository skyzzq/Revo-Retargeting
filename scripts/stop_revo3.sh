#!/usr/bin/env bash
# Stop Revo3 driver and/or MANUS teleop processes; free serial ports.
#
# Usage: stop_revo3.sh [all|driver|teleop]
# From start_revo3_driver.sh cleanup, set REVO3_STOP_INTERNAL=1 so this script
# does not pkill the calling wrapper.
set -euo pipefail

TARGET="${1:-all}"
if [[ "${TARGET}" != "all" && "${TARGET}" != "driver" && "${TARGET}" != "teleop" ]]; then
  echo "Usage: stop_revo3.sh [all|driver|teleop]" >&2
  exit 1
fi

kill_patterns() {
  local pat any=0
  for pat in "$@"; do
    if pgrep -f "${pat}" >/dev/null 2>&1; then
      any=1
      break
    fi
  done
  if [[ "${any}" == "0" ]]; then
    return 0
  fi
  for pat in "$@"; do
    pkill -TERM -f "${pat}" 2>/dev/null || true
  done
  sleep 0.3
  for pat in "$@"; do
    pkill -KILL -f "${pat}" 2>/dev/null || true
  done
}

TELEOP_PATS=(
  'manus_data_publisher'
  'retarget_node'
  'manus_revo3_retarget/pipeline_launch'
  'pipeline_launch\.py'
)

DRIVER_PATS=(
  'dual_revo3_system\.launch'
  'revo3_system\.launch'
  'ros2_control_node'
  'activate_revo3_controllers'
  '/opt/ros/.*/lib/controller_manager/spawner .*revo3_'
  'robot_state_publisher --ros-args -r __ns:=/revo3_'
)

# Wrapper scripts only when user invokes stop directly (not from start_*)
if [[ "${REVO3_STOP_INTERNAL:-0}" != "1" ]]; then
  TELEOP_PATS+=('teleop_revo3\.sh' 'teleop\.sh')
  DRIVER_PATS+=('start_revo3_driver\.sh' 'start_driver\.sh')
fi

case "${TARGET}" in
  teleop)
    echo "[stop_revo3] Stopping teleop (MANUS + retarget)..."
    kill_patterns "${TELEOP_PATS[@]}"
    ;;
  driver)
    echo "[stop_revo3] Stopping Revo3 driver..."
    kill_patterns "${DRIVER_PATS[@]}"
    ;;
  all)
    echo "[stop_revo3] Stopping teleop + driver..."
    kill_patterns "${TELEOP_PATS[@]}" "${DRIVER_PATS[@]}"
    ;;
esac

sleep 0.2
echo "[stop_revo3] Remaining related processes:"
if ps -eo pid,cmd | grep -E 'ros2_control_node|teleop_revo3|manus_data_publisher|retarget_node|dual_revo3|revo3_system\.launch' | grep -v grep; then
  echo "[stop_revo3] WARN: some processes still alive; try again or kill by PID." >&2
else
  echo "  (none)"
fi

if [[ -e /dev/ttyUSB0 || -e /dev/ttyUSB1 ]]; then
  if fuser /dev/ttyUSB0 /dev/ttyUSB1 >/dev/null 2>&1; then
    echo "[stop_revo3] WARN: serial ports still held:"
    fuser -v /dev/ttyUSB0 /dev/ttyUSB1 2>&1 || true
  else
    echo "[stop_revo3] Serial ports free."
  fi
else
  echo "[stop_revo3] No /dev/ttyUSB* present (adapter unplugged?)."
fi
