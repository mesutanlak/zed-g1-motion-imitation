#!/usr/bin/env bash
set -euo pipefail

root="${G1_WSL_ROOT:-$HOME/g1_isaaclab_project}"
repos="$root/repos"
venv="$root/envs/gmr_zed"
gmr_commit="bb1bbe40774794fceb2a7c579a3464a28e68c844"

sudo apt-get update
sudo apt-get install -y git python3.10-venv python3-pip build-essential \
  libgl1 libglib2.0-0

mkdir -p "$repos" "$root/envs"
if [[ ! -d "$repos/GMR/.git" ]]; then
  git clone https://github.com/YanjieZe/GMR.git "$repos/GMR"
fi
git -C "$repos/GMR" fetch --all --tags
git -C "$repos/GMR" checkout "$gmr_commit"

if [[ ! -x "$venv/bin/python" ]]; then
  python3.10 -m venv "$venv"
fi
"$venv/bin/python" -m pip install --upgrade pip setuptools wheel
"$venv/bin/python" -m pip install -e "$repos/GMR"

"$venv/bin/python" - <<'PY'
import mujoco, mink, numpy
import general_motion_retargeting
print("GMR ortam testi OK")
PY

echo "GMR hazir: $venv"
