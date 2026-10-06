#!/usr/bin/env bash
# Download only the Go2/Go2W MJCF models and their mesh/texture resources.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
asset_dir="${1:-${project_dir}/assets}"
asset_revision=7dd30bdc7806898950b354260655d5a7f0ce844e
if [[ -f "$asset_dir/go2/mjcf/scene.xml" && -f "$asset_dir/go2w/mjcf/scene.xml" ]]; then
    echo "Go2/Go2W assets already available: $asset_dir"
    exit 0
fi
cache_dir="${project_dir}/library/robot_assets"
if [[ ! -d "$cache_dir/.git" ]]; then
    mkdir -p "${project_dir}/library"
    git clone --depth 1 --filter=blob:none --sparse https://github.com/fan-ziqi/rl_sar_zoo.git "$cache_dir"
fi
if ! git -C "$cache_dir" cat-file -e "${asset_revision}^{commit}" 2>/dev/null; then
    git -C "$cache_dir" fetch --depth 1 origin "$asset_revision"
fi
git -C "$cache_dir" sparse-checkout set go2_description/mjcf go2w_description/mjcf
git -C "$cache_dir" checkout --detach "$asset_revision"
mkdir -p "$asset_dir"
for robot in go2 go2w; do
    source_dir="$cache_dir/${robot}_description"
    if [[ ! -d "$source_dir/mjcf" ]]; then
        echo "Missing MJCF resources: $source_dir" >&2
        exit 1
    fi
    if [[ -f "$asset_dir/$robot/mjcf/scene.xml" ]]; then continue; fi
    if [[ -e "$asset_dir/$robot" ]]; then
        echo "Assets are incomplete at $asset_dir/$robot; move them aside before downloading." >&2
        exit 1
    fi
    mkdir -p "$asset_dir/$robot"
    for resource in mjcf; do
        if [[ -d "$source_dir/$resource" ]]; then
            cp -R "$source_dir/$resource" "$asset_dir/$robot/"
        fi
    done
    for notice in LICENSE LICENSE.txt README.md; do
        if [[ -f "$source_dir/$notice" ]]; then
            cp "$source_dir/$notice" "$asset_dir/$robot/"
        fi
    done
done
if [[ -f "$cache_dir/LICENSE" ]]; then cp "$cache_dir/LICENSE" "$asset_dir/UPSTREAM_LICENSE"; fi
git -C "$cache_dir" rev-parse HEAD > "$asset_dir/UPSTREAM_REVISION"
echo "Installed only Go2/Go2W resources in $asset_dir"
