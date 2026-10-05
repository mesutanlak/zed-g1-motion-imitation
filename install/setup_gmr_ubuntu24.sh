#!/usr/bin/env bash
set -euo pipefail

root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
repos="$root/repos"
venv="$root/envs/gmr_zed"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"
gmr_commit="bb1bbe40774794fceb2a7c579a3464a28e68c844"

if [[ ! -x "$uv_bin" ]]; then
  echo "uv bulunamadi: $uv_bin" >&2
  echo "Kurulum: curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y git git-lfs build-essential libgl1 libglib2.0-0t64

mkdir -p "$repos" "$root/envs"
if [[ ! -d "$repos/GMR/.git" ]]; then
  git clone https://github.com/YanjieZe/GMR.git "$repos/GMR"
fi
if [[ -n "$(git -C "$repos/GMR" status --porcelain)" ]]; then
  echo "GMR calisma agaci kirli; otomatik checkout yapilmadi: $repos/GMR" >&2
  exit 1
fi
git -C "$repos/GMR" fetch --all --tags
git -C "$repos/GMR" checkout "$gmr_commit"
git -C "$repos/GMR" lfs pull

if [[ ! -x "$venv/bin/python" ]]; then
  "$uv_bin" venv --python 3.10 --seed "$venv"
fi
if [[ "$("$venv/bin/python" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" != "3.10" ]]; then
  echo "GMR ortami Python 3.10 olmali: $venv" >&2
  exit 1
fi

UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-600}" "$uv_bin" pip install --python "$venv/bin/python" \
  torch==2.7.0 --index-url https://download.pytorch.org/whl/cu128
UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-600}" "$uv_bin" pip install \
  --python "$venv/bin/python" -e "$repos/GMR"

"$venv/bin/python" - <<'PY'
import general_motion_retargeting
import mink
import mujoco
import numpy

print("GMR Ubuntu 24.04 ortam testi: OK")
print("NumPy:", numpy.__version__)
print("MuJoCo:", mujoco.__version__)
PY

echo "GMR hazir: $venv"
