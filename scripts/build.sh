#!/usr/bin/env bash
# build.sh
#
# Thin wrapper: sources njord_env.sh (platform detection) and runs
# colcon build with a Release build type. Extra arguments are forwarded to
# colcon build, e.g.:
#   scripts/build.sh --packages-select njord_platform
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${WS_DIR}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/njord_env.sh"

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
