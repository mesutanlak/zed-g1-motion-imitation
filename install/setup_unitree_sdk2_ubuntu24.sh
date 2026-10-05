#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
repo="$root/repos/unitree_sdk2_python"
venv="$root/envs/unitree_sdk2"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"
commit="65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5"

if [[ ! -x "$uv_bin" ]]; then
  echo "uv bulunamadi: $uv_bin" >&2
  exit 1
fi
if [[ ! -d "$repo/.git" ]]; then
  bash "$project/install/fetch_ubuntu24_sources.sh"
fi
if [[ "$(git -C "$repo" rev-parse HEAD)" != "$commit" ]]; then
  echo "Unitree SDK2 sabit commiti etkin degil: $repo" >&2
  exit 1
fi
if [[ ! -x "$venv/bin/python" ]]; then
  "$uv_bin" venv --python 3.10 --seed "$venv"
fi

UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-600}" "$uv_bin" pip install \
  --python "$venv/bin/python" -e "$repo"
"$venv/bin/python" - <<'PY'
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

print("Unitree SDK2 Python import: OK")
print("DDS baslatilmadi; ag arayuzu ve robot baglantisi bu testte acilmaz.")
PY

echo "Unitree SDK2 ortami hazir: $venv"
