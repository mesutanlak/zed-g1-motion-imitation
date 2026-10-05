#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
repo="$root/repos/unitree_mujoco"
venv="$root/envs/mujoco"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"
commit="ae6a8403e272733e9996ef59990880330496177f"

sudo apt-get update
sudo apt-get install -y git git-lfs build-essential cmake libgl1 libglfw3
mkdir -p "$root/repos" "$root/envs"
if [[ ! -d "$repo/.git" ]]; then
  git clone https://github.com/unitreerobotics/unitree_mujoco.git "$repo"
fi
if [[ -n "$(git -C "$repo" status --porcelain)" ]]; then
  echo "unitree_mujoco calisma agaci kirli: $repo" >&2
  exit 1
fi
git -C "$repo" fetch --all --tags
git -C "$repo" checkout "$commit"
git -C "$repo" lfs pull

if [[ ! -x "$venv/bin/python" ]]; then
  "$uv_bin" venv --python 3.12 --seed "$venv"
fi
"$venv/bin/python" -m pip install --upgrade pip
"$venv/bin/python" -m pip install mujoco numpy scipy
"$venv/bin/python" "$project/sim_mujoco/generate_g1_23dof_obstacles.py" \
  --unitree-mujoco-root "$repo"

echo "MuJoCo hazir: $venv"
