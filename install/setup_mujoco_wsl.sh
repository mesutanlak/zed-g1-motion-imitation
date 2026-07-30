#!/usr/bin/env bash
set -euo pipefail

workspace="${ROS2_WS:-$HOME/ros2_ws}"
repo="$workspace/src/unitree_mujoco"
venv="${G1_MUJOCO_VENV:-$HOME/unitree_venv/g1}"
commit="ae6a8403e272733e9996ef59990880330496177f"

sudo apt-get update
sudo apt-get install -y git python3.10-venv build-essential cmake
mkdir -p "$workspace/src" "$(dirname "$venv")"
if [[ ! -d "$repo/.git" ]]; then
  git clone https://github.com/unitreerobotics/unitree_mujoco.git "$repo"
fi
git -C "$repo" fetch --all --tags
git -C "$repo" checkout "$commit"
if [[ ! -x "$venv/bin/python" ]]; then
  python3.10 -m venv "$venv"
fi
"$venv/bin/python" -m pip install --upgrade pip
"$venv/bin/python" -m pip install mujoco numpy scipy

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$venv/bin/python" "$project/sim_mujoco/generate_g1_23dof_obstacles.py" \
  --unitree-mujoco-root "$repo"
echo "MuJoCo ortami hazir: $repo"
