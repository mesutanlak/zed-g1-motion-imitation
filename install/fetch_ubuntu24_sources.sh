#!/usr/bin/env bash
set -euo pipefail

root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
repos="$root/repos"

declare -A repo_url=(
  [IsaacLab]="https://github.com/isaac-sim/IsaacLab.git"
  [GMR]="https://github.com/YanjieZe/GMR.git"
  [unitree_rl_lab]="https://github.com/unitreerobotics/unitree_rl_lab.git"
  [unitree_ros]="https://github.com/unitreerobotics/unitree_ros.git"
  [unitree_sim_isaaclab]="https://github.com/unitreerobotics/unitree_sim_isaaclab.git"
  [unitree_sdk2_python]="https://github.com/unitreerobotics/unitree_sdk2_python.git"
  [xr_teleoperate]="https://github.com/unitreerobotics/xr_teleoperate.git"
  [unitree_mujoco]="https://github.com/unitreerobotics/unitree_mujoco.git"
)
declare -A repo_commit=(
  [IsaacLab]="ae37b028ea415c91ea2bc32609efcd759ed2b974"
  [GMR]="bb1bbe40774794fceb2a7c579a3464a28e68c844"
  [unitree_rl_lab]="4960b84732b0c2ec593dccbfe963fda1bcd7b1e3"
  [unitree_ros]="ac7714828e1a4cc2a8ddd5c78a1305475b913a73"
  [unitree_sim_isaaclab]="e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc"
  [unitree_sdk2_python]="65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5"
  [xr_teleoperate]="817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b"
  [unitree_mujoco]="ae6a8403e272733e9996ef59990880330496177f"
)

mkdir -p "$repos"
git lfs install --skip-repo

for name in IsaacLab GMR unitree_rl_lab unitree_ros unitree_sim_isaaclab unitree_sdk2_python xr_teleoperate unitree_mujoco; do
  target="$repos/$name"
  if [[ ! -d "$target/.git" ]]; then
    git clone --recurse-submodules "${repo_url[$name]}" "$target"
  fi
  if [[ -n "$(git -C "$target" status --porcelain)" ]]; then
    echo "$name calisma agaci kirli; sabit commit checkout edilmedi: $target" >&2
    exit 1
  fi
  git -C "$target" fetch --all --tags
  git -C "$target" checkout "${repo_commit[$name]}"
  git -C "$target" submodule update --init --recursive
  git -C "$target" lfs pull
  git -C "$target" submodule foreach --recursive 'git lfs pull'
done

dex3_asset="$repos/unitree_sim_isaaclab/assets/robots/g1-29dof-dex3-base-fix-usd/g1_29dof_with_dex3_base_fix.usd"
if [[ ! -s "$dex3_asset" ]]; then
  (cd "$repos/unitree_sim_isaaclab" && bash ./fetch_assets.sh)
fi

printf 'Kaynaklar hazir: %s\n' "$repos"
