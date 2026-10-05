#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
venv="$root/envs/rerun"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"

if [[ ! -x "$venv/bin/python" ]]; then
  "$uv_bin" venv --python 3.12 --seed "$venv"
  "$venv/bin/python" -m pip install -r "$project/requirements-rerun.txt"
fi
export PATH="$venv/bin:$PATH"
exec "$venv/bin/python" -m rerun_analysis.app \
  --listen-host 0.0.0.0 --listen-port 15052 \
  --gmr-listen-port 15053 --live-max-hz 15 --gmr-log-max-hz 15 \
  --output-dir "$project/rerun_recordings" "$@"
