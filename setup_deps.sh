#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
if [[ $# -eq 0 ]]; then set -- all; fi
for dependency in "$@"; do
    case "$dependency" in
        all)
            bash scripts/download_inference_runtime.sh all
            bash scripts/download_mujoco.sh
            bash scripts/download_robot_descriptions.sh
            ;;
        torch|libtorch) bash scripts/download_inference_runtime.sh libtorch ;;
        onnx) bash scripts/download_inference_runtime.sh onnx ;;
        mujoco) bash scripts/download_mujoco.sh ;;
        assets) bash scripts/download_robot_descriptions.sh ;;
        real|joystick)
            if [[ "$dependency" == real ]]; then
                dep_name=unitree_sdk2
                dep_url=https://github.com/unitreerobotics/unitree_sdk2.git
                dep_revision=65e19c625b76b697f94e0b953e296ecc0ce7b4a1
            else
                dep_name=joystick
                dep_url=https://github.com/drewnoakes/joystick.git
                dep_revision=89d13f250d9f4c29ef8070eb69a5f2d1753d08d0
            fi
            dep_dir="${project_dir}/library/${dep_name}"
            if [[ ! -d "$dep_dir" ]]; then
                mkdir -p "$dep_dir"
                git -C "$dep_dir" init -q
                git -C "$dep_dir" remote add origin "$dep_url"
            fi
            if ! git -C "$dep_dir" cat-file -e "${dep_revision}^{commit}" 2>/dev/null; then
                git -C "$dep_dir" fetch --depth 1 origin "$dep_revision"
            fi
            git -C "$dep_dir" checkout --detach "$dep_revision"
            ;;
        -h|--help)
            echo 'Usage: ./setup_deps.sh [all|torch|onnx|mujoco|assets|real|joystick] ...'
            echo 'all installs both backends, MuJoCo and Go2/Go2W assets.'
            echo 'real and joystick are optional and excluded from all.'
            ;;
        *) echo "Unknown dependency: $dependency" >&2; exit 1 ;;
    esac
done
