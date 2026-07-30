#!/usr/bin/env bash
set -euo pipefail

project_path="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_path="${G1_MUJOCO_VENV:-$HOME/unitree_venv/g1}"
source "$venv_path/bin/activate"
exec python "$project_path/sim_mujoco/g1_zed_live_mimic.py" "$@"
