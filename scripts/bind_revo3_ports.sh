#!/usr/bin/env bash
# Convenience wrapper: interactive left/right serial bind for Revo3.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${SCRIPT_DIR}/../src/brainco_revo3_ros2/revo3_driver/setup/bind_revo3_ports_interactive.sh"
exec bash "${TARGET}" "$@"
