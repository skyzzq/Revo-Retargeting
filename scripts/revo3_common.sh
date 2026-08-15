#!/usr/bin/env bash
# Shared helpers for Revo3 scripts (sourced, not executed).
# Ensures conda env + a Python that can import Humble rclpy (3.10).

REVO3_CONDA_ENV="${REVO3_CONDA_ENV:-revo_retargeting}"

revo3_normalize_task() {
  case "${1:-}" in
    blocks|block) printf '%s\n' "blocks" ;;
    unbox|unboxing|express) printf '%s\n' "unbox" ;;
    *) return 1 ;;
  esac
}

revo3_unbox_profile_dir() {
  local workspace="$1"
  local share candidate
  if command -v ros2 >/dev/null 2>&1; then
    share="$(ros2 pkg prefix manus_revo3_retarget 2>/dev/null || true)"
    candidate="${share}/share/manus_revo3_retarget/config/profiles/unbox"
    if [[ -n "${share}" && -d "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  fi
  candidate="${workspace}/src/manus_revo3_retarget/config/profiles/unbox"
  if [[ -d "${candidate}" ]]; then
    printf '%s\n' "${candidate}"
    return 0
  fi
  echo "[revo3] ERROR: unbox profile directory not found. Rebuild manus_revo3_retarget." >&2
  return 1
}

revo3_source_nounset() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

revo3_maybe_activate_conda() {
  if [[ "${REVO3_CONDA_ENV}" == "0" || "${REVO3_CONDA_ENV}" == "none" ]]; then
    return 0
  fi
  if [[ "${CONDA_DEFAULT_ENV:-}" == "${REVO3_CONDA_ENV}" ]]; then
    return 0
  fi
  local conda_sh=""
  local candidate
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
    return 0
  fi
  revo3_source_nounset "${conda_sh}"
  set +u
  conda activate "${REVO3_CONDA_ENV}"
  set +u
  echo "[revo3] Activated conda env: ${REVO3_CONDA_ENV}"
}

# Prefer a known-good 3.10 interpreter; only import rclpy once to validate.
revo3_resolve_python() {
  if [[ -n "${REVO3_PYTHON:-}" ]]; then
    if [[ -x "${REVO3_PYTHON}" ]] || command -v "${REVO3_PYTHON}" >/dev/null 2>&1; then
      printf '%s\n' "${REVO3_PYTHON}"
      return 0
    fi
  fi

  local candidates=()
  if [[ -n "${CONDA_PREFIX:-}" ]]; then
    candidates+=("${CONDA_PREFIX}/bin/python3.10" "${CONDA_PREFIX}/bin/python")
  fi
  candidates+=(
    "${HOME}/miniconda3/envs/${REVO3_CONDA_ENV}/bin/python3.10"
    "${HOME}/miniconda3/envs/${REVO3_CONDA_ENV}/bin/python"
    "${HOME}/miniforge3/envs/${REVO3_CONDA_ENV}/bin/python3.10"
    /usr/bin/python3.10
    python3.10
  )

  local py ver
  for py in "${candidates[@]}"; do
    if ! command -v "${py}" >/dev/null 2>&1 && [[ ! -x "${py}" ]]; then
      continue
    fi
    ver="$("${py}" -c 'import sys; print("%d.%d"%sys.version_info[:2])' 2>/dev/null || true)"
    if [[ "${ver}" != "3.10" ]]; then
      continue
    fi
    if "${py}" -c 'import rclpy' >/dev/null 2>&1; then
      export REVO3_PYTHON="${py}"
      printf '%s\n' "${py}"
      return 0
    fi
  done

  echo "[revo3] ERROR: no Python 3.10 can import rclpy." >&2
  echo "  Activate the env first: conda activate ${REVO3_CONDA_ENV}" >&2
  return 1
}
