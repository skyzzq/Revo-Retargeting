#!/usr/bin/env bash
# Convenience wrapper: interactive left/right serial bind for Revo3.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${SCRIPT_DIR}/../src/brainco_revo3_ros2/revo3_driver/setup/bootstrap_revo3.sh"
exec bash "${TARGET}" "$@"
