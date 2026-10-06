#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
venv="$root/envs/rerun"
uv_bin="${UV_BIN:-$HOME/.local/bin/uv}"
viewer=0
app_args=()
for argument in "$@"; do
  case "$argument" in
    --viewer) viewer=1 ;;
    *) app_args+=("$argument") ;;
  esac
done

if [[ ! -x "$venv/bin/python" ]]; then
  "$uv_bin" venv --python 3.12 --seed "$venv"
  "$venv/bin/python" -m pip install -r "$project/requirements-rerun.txt"
fi
export PATH="$venv/bin:$PATH"
stamp="$(date +%Y%m%d_%H%M%S)"
rrd_path="$project/rerun_recordings/rerun_body38_${stamp}.rrd"
mkdir -p "$project/rerun_recordings"
rerun_port="${G1_RERUN_GRPC_PORT:-9876}"
rerun_url="rerun+http://127.0.0.1:${rerun_port}/proxy"

server_pid=""
viewer_pid=""
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$viewer_pid" ]] || kill "$viewer_pid" 2>/dev/null || true
  [[ -z "$server_pid" ]] || kill "$server_pid" 2>/dev/null || true
  [[ -z "$viewer_pid" ]] || wait "$viewer_pid" 2>/dev/null || true
  [[ -z "$server_pid" ]] || wait "$server_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$venv/bin/rerun" --serve-grpc --save "$rrd_path" --port "$rerun_port" &
server_pid=$!
for _ in {1..40}; do
  kill -0 "$server_pid" 2>/dev/null || {
    echo "Rerun kayit sunucusu erken kapandi." >&2
    exit 1
  }
  ss -ltnH | grep -Eq ":${rerun_port}([[:space:]]|$)" && break
  sleep 0.10
done
ss -ltnH | grep -Eq ":${rerun_port}([[:space:]]|$)" || {
  echo "Rerun gRPC portu acilmadi: $rerun_port" >&2
  exit 1
}
if (( viewer )); then
  "$venv/bin/rerun" --connect "$rerun_url" &
  viewer_pid=$!
fi

echo "Rerun ayrik kayit sunucusu: $rerun_url"
echo "RRD: $rrd_path | Viewer=${viewer} | ham telemetri kaydi=30Hz"
"$venv/bin/python" -m rerun_analysis.app \
  --listen-host 0.0.0.0 --listen-port 15052 \
  --gmr-listen-port 15053 --live-max-hz 12 --gmr-log-max-hz 15 \
  --output-dir "$project/rerun_recordings" \
  --rrd-path "$rrd_path" --rerun-grpc-url "$rerun_url" \
  --no-viewer --headless "${app_args[@]}"
