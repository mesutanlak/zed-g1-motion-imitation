#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
repos="$root/repos"
venv="$root/envs/isaacsim"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"

declare -A repo_url=(
  [IsaacLab]="https://github.com/isaac-sim/IsaacLab.git"
  [unitree_rl_lab]="https://github.com/unitreerobotics/unitree_rl_lab.git"
  [unitree_ros]="https://github.com/unitreerobotics/unitree_ros.git"
  [unitree_sim_isaaclab]="https://github.com/unitreerobotics/unitree_sim_isaaclab.git"
  [unitree_sdk2_python]="https://github.com/unitreerobotics/unitree_sdk2_python.git"
)
declare -A repo_commit=(
  [IsaacLab]="46dff135f44683f031edf346e544fcfd8456b2bb"
  [unitree_rl_lab]="4960b84732b0c2ec593dccbfe963fda1bcd7b1e3"
  [unitree_ros]="ac7714828e1a4cc2a8ddd5c78a1305475b913a73"
  [unitree_sim_isaaclab]="e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc"
  [unitree_sdk2_python]="65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5"
)

if [[ ! -x "$uv_bin" ]]; then
  echo "uv bulunamadi: $uv_bin" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  git git-lfs build-essential cmake ninja-build pkg-config \
  libgl1 libglib2.0-0t64 libx11-6 libxext6 libxrender1 libsm6

mkdir -p "$repos" "$root/envs" "$root/cache/g1_23dof"
git lfs install --skip-repo

for name in IsaacLab unitree_rl_lab unitree_ros unitree_sim_isaaclab unitree_sdk2_python; do
  target="$repos/$name"
  if [[ ! -d "$target/.git" ]]; then
    git clone "${repo_url[$name]}" "$target"
  fi
  if [[ -n "$(git -C "$target" status --porcelain)" ]]; then
    echo "$name calisma agaci kirli; otomatik checkout yapilmadi: $target" >&2
    exit 1
  fi
  git -C "$target" fetch --all --tags
  git -C "$target" checkout "${repo_commit[$name]}"
  git -C "$target" lfs pull
done

if [[ ! -x "$venv/bin/python" ]]; then
  "$uv_bin" venv --python 3.11 --seed "$venv"
fi
if [[ "$("$venv/bin/python" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" != "3.11" ]]; then
  echo "Isaac ortami Python 3.11 olmali: $venv" >&2
  exit 1
fi

python="$venv/bin/python"
"$python" -m pip install --upgrade pip "setuptools==80.9.0" "wheel==0.45.1"
"$python" -m pip install \
  torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 \
  --index-url https://download.pytorch.org/whl/cu128
"$python" -m pip install "isaacsim[all,extscache]==5.0.0" \
  --extra-index-url https://pypi.nvidia.com
"$python" -m pip install \
  numpy==1.26.0 packaging==23.0 psutil==5.9.8 typing_extensions==4.12.2 \
  onnxruntime==1.28.0 PyYAML==6.0.2 toml==0.10.2 "wheel==0.45.1"

for package in isaaclab isaaclab_assets isaaclab_rl isaaclab_tasks; do
  "$python" -m pip install -e "$repos/IsaacLab/source/$package"
done
"$python" -m pip install -e "$repos/unitree_rl_lab/source/unitree_rl_lab"

"$python" -m pip check
"$python" "$project/isaaclab_bridge/test_environment.py"

echo "Isaac Sim 5.0 / Isaac Lab v2.2 ortami hazir: $venv"
echo "Tam SimulationApp testi icin uyumlu R570 surucusu gereklidir."
