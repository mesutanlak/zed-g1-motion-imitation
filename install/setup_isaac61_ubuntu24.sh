#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
isaaclab_repo="$root/repos/IsaacLab"
venv="$root/envs/isaaclab30"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"
isaaclab_commit="ae37b028ea415c91ea2bc32609efcd759ed2b974"

if [[ ! -x "$uv_bin" ]]; then
  echo "uv bulunamadi: $uv_bin" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  git git-lfs build-essential cmake ninja-build pkg-config \
  libgl1 libglib2.0-0t64 libx11-6 libxext6 libxrender1 libsm6 libglfw3

bash "$project/install/fetch_ubuntu24_sources.sh"
if [[ "$(git -C "$isaaclab_repo" rev-parse HEAD)" != "$isaaclab_commit" ]]; then
  echo "Isaac Lab v3.0.0-EA commiti etkin degil: $isaaclab_repo" >&2
  exit 1
fi

# Isaac Lab 3.0's own uv lock is the source of truth for Python 3.12,
# Isaac Sim 6.1, PyTorch 2.11 and the NVIDIA package index.
UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-600}" \
UV_PROJECT_ENVIRONMENT="$venv" "$uv_bin" sync \
  --project "$isaaclab_repo" --extra isaacsim --no-dev --frozen

"$uv_bin" pip install --python "$venv/bin/python" \
  onnxruntime PyYAML toml

"$venv/bin/python" - <<'PY'
import importlib.metadata as metadata
import torch
from isaaclab.utils.warp import ProxyArray

assert metadata.version("isaacsim") == "6.1.0.0"
assert torch.cuda.is_available()
print("Isaac Sim:", metadata.version("isaacsim"))
print("Isaac Lab source release: 3.0.0-EA")
print("Isaac Lab package:", metadata.version("isaaclab"))
print("PyTorch:", torch.__version__, "CUDA:", torch.version.cuda)
print("GPU:", torch.cuda.get_device_name(0))
PY

echo "Isaac Sim 6.1 / Isaac Lab 3.0 ortami hazir: $venv"
