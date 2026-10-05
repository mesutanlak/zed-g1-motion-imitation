#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export G1_MUJOCO_VENV="${G1_MUJOCO_VENV:-$HOME/g1_isaaclab_project/envs/mujoco}"
exec "$project/sim_mujoco/run_g1_zed_mimic.sh" "$@"
