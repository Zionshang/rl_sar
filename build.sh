#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'HELP'
Usage: ./build.sh [CMAKE_OPTIONS...]
Build Go2/Go2W MuJoCo simulation without ROS.
Dependencies are installed separately with ./setup_deps.sh.

Examples:
  ./build.sh
  ./build.sh -DENABLE_TORCH=OFF        # ONNX Runtime only
  ./build.sh -DBUILD_REAL=ON           # optional hardware deployment
  ./build.sh -DENABLE_JOYSTICK=ON      # optional Linux USB gamepad

Environment: BUILD_DIR (default: build), BUILD_JOBS (default: 2).
HELP
    exit 0
fi
build_dir="${BUILD_DIR:-${project_dir}/build}"
cmake -S "$project_dir" -B "$build_dir" "$@"
cmake --build "$build_dir" --parallel "${BUILD_JOBS:-2}"
