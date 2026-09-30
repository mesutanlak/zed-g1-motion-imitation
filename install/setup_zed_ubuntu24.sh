#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
venv="$root/envs/zed"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"

if [[ ! -x /usr/local/zed/tools/ZED_Explorer ]]; then
  echo "ZED SDK bulunamadi: /usr/local/zed" >&2
  exit 1
fi
if [[ ! -x "$venv/bin/python" ]]; then
  if [[ -x "$uv_bin" ]]; then
    "$uv_bin" venv --python 3.12 --seed "$venv"
  else
    python3 -m venv "$venv"
  fi
fi

"$venv/bin/python" -m pip install --upgrade pip
"$venv/bin/python" -m pip install -r "$project/requirements-zed.txt"

if ! "$venv/bin/python" -c 'import pyzed.sl' >/dev/null 2>&1; then
  "$venv/bin/python" /usr/local/zed/get_python_api.py
fi

"$venv/bin/python" - <<'PY'
import cv2
import numpy
import pyzed.sl as sl

print("ZED Python API: OK")
print("NumPy:", numpy.__version__)
print("OpenCV:", cv2.__version__)
PY

echo "ZED ortami hazir: $venv"
