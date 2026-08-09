#!/usr/bin/env bash
# Record Revo3 hand (and optional teleop) topics to MCAP.
# Does NOT start driver/teleop — run those separately first.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${WORKSPACE}/install/setup.bash"

HAND="right"
PRESET="teleop"
DURATION=""
NAME="hand"
NOTE=""
OUT_ROOT="${OUT_ROOT:-${HOME}/Documents/retargeting/bags}"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-revo_retargeting}"

usage() {
  cat <<'EOF'
Usage: record_hand_data.sh [left|right|both] [options]

Record dexterous-hand topics while driver/teleop are already running.
Does not launch ROS nodes itself.

Options:
  --preset hand|teleop|full   Topic set (default: teleop)
  --duration SECONDS          Stop automatically after N seconds
  --name LABEL                Bag folder name prefix (default: hand)
  --note TEXT                 Free-form note stored in meta.json
  --out-root DIR              Output root (default: ~/Documents/retargeting/bags)
  -h, --help                  Show this help

Presets:
  hand    dynamic_joint_states + joint_states + MIT commands
  teleop  hand + retarget_targets + manus_glove_*
  full    teleop + joint_states_aligned

Examples:
  ./scripts/record_hand_data.sh right
  ./scripts/record_hand_data.sh right --preset hand
  ./scripts/record_hand_data.sh right --duration 60 --name grasp_test --note "kp=2.0"
EOF
}

source_with_nounset_disabled() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

maybe_activate_conda() {
  if [[ "${CONDA_ENV_NAME}" == "0" || "${CONDA_ENV_NAME}" == "none" ]]; then
    return 0
  fi
  if [[ "${CONDA_DEFAULT_ENV:-}" == "${CONDA_ENV_NAME}" ]]; then
    return 0
  fi
  local conda_sh=""
  for candidate in \
    "${HOME}/miniforge3/etc/profile.d/conda.sh" \
    "${HOME}/miniconda3/etc/profile.d/conda.sh" \
    "${HOME}/anaconda3/etc/profile.d/conda.sh"; do
    if [[ -f "${candidate}" ]]; then
      conda_sh="${candidate}"
      break
    fi
  done
  if [[ -z "${conda_sh}" ]]; then
    echo "[record_hand_data] conda.sh not found; continuing with current env." >&2
    return 0
  fi
  source_with_nounset_disabled "${conda_sh}"
  conda activate "${CONDA_ENV_NAME}"
}

build_topics() {
  local hand="$1"
  local preset="$2"
  TOPICS=()

  add_side_hand_topics() {
    local side="$1"
    local ns="/revo3_${side}"
    TOPICS+=(
      "${ns}/revo3_joint_state/dynamic_joint_states"
      "${ns}/revo3_joint_state/joint_states"
      "${ns}/joint_forward_mit_controller/commands"
    )
    if [[ "${preset}" == "teleop" || "${preset}" == "full" ]]; then
      TOPICS+=("${ns}/joint_forward_mit_controller/retarget_targets")
    fi
    if [[ "${preset}" == "full" ]]; then
      TOPICS+=("${ns}/revo3_joint_state/joint_states_aligned")
    fi
  }

  case "${hand}" in
    left)
      add_side_hand_topics left
      if [[ "${preset}" == "teleop" || "${preset}" == "full" ]]; then
        TOPICS+=("/manus_glove_1")
      fi
      ;;
    right)
      add_side_hand_topics right
      if [[ "${preset}" == "teleop" || "${preset}" == "full" ]]; then
        TOPICS+=("/manus_glove_0")
      fi
      ;;
    both)
      add_side_hand_topics left
      add_side_hand_topics right
      if [[ "${preset}" == "teleop" || "${preset}" == "full" ]]; then
        TOPICS+=("/manus_glove_0" "/manus_glove_1")
      fi
      ;;
  esac
}

write_meta() {
  local meta_path="$1"
  python3 - "$meta_path" <<'PY'
import json, os, sys
from datetime import datetime, timezone

meta_path = sys.argv[1]
topics = os.environ.get("RECORD_TOPICS_JSON", "[]")
payload = {
    "hand": os.environ.get("RECORD_HAND", ""),
    "preset": os.environ.get("RECORD_PRESET", ""),
    "name": os.environ.get("RECORD_NAME", ""),
    "note": os.environ.get("RECORD_NOTE", ""),
    "started_at": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
    "topics": json.loads(topics),
    "out_dir": os.environ.get("RECORD_OUT_DIR", ""),
    "bag_dir": os.environ.get("RECORD_BAG_DIR", ""),
    "notes_about_data": {
        "current_A": "in dynamic_joint_states interface 'current'",
        "motor_state": "fault bitmask in dynamic_joint_states",
        "joint_states.effort": "NaN — not measured force",
        "commands.effort": "MIT feedforward (mA), not measured force",
    },
}
with open(meta_path, "w", encoding="utf-8") as f:
    json.dump(payload, f, ensure_ascii=False, indent=2)
    f.write("\n")
PY
}

# --- parse args ---
POSITIONAL=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --preset)
      PRESET="${2:-}"
      shift 2
      ;;
    --duration)
      DURATION="${2:-}"
      shift 2
      ;;
    --name)
      NAME="${2:-}"
      shift 2
      ;;
    --note)
      NOTE="${2:-}"
      shift 2
      ;;
    --out-root)
      OUT_ROOT="${2:-}"
      shift 2
      ;;
    left|right|both)
      POSITIONAL+=("$1")
      shift
      ;;
    *)
      echo "[record_hand_data] Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ${#POSITIONAL[@]} -gt 0 ]]; then
  HAND="${POSITIONAL[0]}"
fi

if [[ "${HAND}" != "left" && "${HAND}" != "right" && "${HAND}" != "both" ]]; then
  echo "[record_hand_data] hand must be left|right|both" >&2
  exit 1
fi

if [[ "${PRESET}" != "hand" && "${PRESET}" != "teleop" && "${PRESET}" != "full" ]]; then
  echo "[record_hand_data] preset must be hand|teleop|full" >&2
  exit 1
fi

if [[ -n "${DURATION}" ]] && ! [[ "${DURATION}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[record_hand_data] --duration must be a positive number" >&2
  exit 1
fi

if [[ ! -f "${SETUP}" ]]; then
  echo "[record_hand_data] Missing ${SETUP}. Build the workspace first." >&2
  exit 1
fi

maybe_activate_conda

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source_with_nounset_disabled /opt/ros/humble/setup.bash
fi
source_with_nounset_disabled "${SETUP}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[record_hand_data] ros2 not found after sourcing." >&2
  exit 1
fi

if ! ros2 bag record --help 2>&1 | grep -Eq -- '--storage \{[^}]*mcap|--storage.*mcap'; then
  cat >&2 <<'EOF'
[record_hand_data] ros2 bag MCAP storage plugin is missing. Install:

  sudo apt install ros-humble-rosbag2-storage-mcap

EOF
  exit 2
fi

build_topics "${HAND}" "${PRESET}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
SAFE_NAME="$(echo "${NAME}" | tr -c 'A-Za-z0-9._-' '_')"
RUN_DIR="${OUT_ROOT}/${SAFE_NAME}_${HAND}_${TIMESTAMP}"
BAG_DIR="${RUN_DIR}/mcap"
mkdir -p "${BAG_DIR}"

RECORD_HAND="${HAND}" \
RECORD_PRESET="${PRESET}" \
RECORD_NAME="${SAFE_NAME}" \
RECORD_NOTE="${NOTE}" \
RECORD_OUT_DIR="${RUN_DIR}" \
RECORD_BAG_DIR="${BAG_DIR}" \
RECORD_TOPICS_JSON="$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1:]))' "${TOPICS[@]}")" \
  write_meta "${RUN_DIR}/meta.json"

echo "[record_hand_data] hand=${HAND} preset=${PRESET}"
echo "[record_hand_data] output: ${RUN_DIR}"
echo "[record_hand_data] topics:"
printf '  %s\n' "${TOPICS[@]}"

BAG_PID=""
cleanup() {
  local status=$?
  trap - INT TERM EXIT
  echo
  echo "[record_hand_data] Stopping recorder..."
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    sleep 2
  fi
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -TERM "${BAG_PID}" 2>/dev/null || true
  fi
  wait "${BAG_PID}" 2>/dev/null || true
  echo "[record_hand_data] Saved MCAP under: ${BAG_DIR}"
  echo "[record_hand_data] Meta: ${RUN_DIR}/meta.json"
  exit "${status}"
}
trap cleanup INT TERM EXIT

BAG_COMMAND=(
  ros2 bag record -s mcap --include-unpublished-topics
  -o "${BAG_DIR}"
  "${TOPICS[@]}"
)

printf '[record_hand_data] Recording:'
printf ' %q' "${BAG_COMMAND[@]}"
printf '\n'
echo "[record_hand_data] Ctrl-C to stop."

"${BAG_COMMAND[@]}" &
BAG_PID=$!

if [[ -n "${DURATION}" ]]; then
  echo "[record_hand_data] Will stop after ${DURATION}s"
  sleep "${DURATION}"
  exit 0
fi

wait "${BAG_PID}"
