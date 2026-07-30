#!/usr/bin/env bash
set -euo pipefail

project_path="${1:?project path is required}"
mode="${2:-upper_body}"
gmr_python="${GMR_PYTHON:-$HOME/g1_isaaclab_project/envs/gmr_zed/bin/python}"
isaac_python="${ISAAC_PYTHON:-$HOME/miniconda3/envs/unitree_isaaclab/bin/python}"

bridge_pid=""
cleanup() {
    if [[ -n "${bridge_pid}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
        kill "${bridge_pid}" 2>/dev/null || true
        wait "${bridge_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

"${gmr_python}" "${project_path}/isaaclab_bridge/gmr_live_bridge.py" \
    --listen-host 0.0.0.0 \
    --listen-port 15050 \
    --output-host 127.0.0.1 \
    --output-port 15051 &
bridge_pid=$!

source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate unitree_isaaclab
export CYCLONEDDS_HOME="$HOME/g1_isaaclab_project/repos/cyclonedds/install"

"${isaac_python}" "${project_path}/isaaclab_bridge/isaac_g1_23dof_live.py" \
    --device cuda:0 \
    --mode "${mode}"
