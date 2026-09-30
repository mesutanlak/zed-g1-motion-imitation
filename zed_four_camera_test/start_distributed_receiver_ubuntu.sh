#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
receiver="$project/zed_four_camera_test/distributed_body38_fusion.py"
extrinsics=""
calibration_record=""
output_host=""
output_port=15050
monitor_host=""
monitor_port=15052
ros_host=""
ros_port=15054
fps=15
minimum_sources=3
preview_hz=10
headless=0
record=0
duration=0
record_stem="four_body38_fusion"
workspace_x_min="2.0"
workspace_x_max="4.0"

usage() {
  cat <<'EOF'
Kullanim: start_distributed_receiver_ubuntu.sh [secenekler]
  --calibration-record PATH       Yeni extrinsic icin ham 4'lu kayit
  --extrinsics PATH               Calibre canli fusion
  --output-host IP [--output-port 15050]
  --monitor-host IP [--monitor-port 15052]
  --ros-host IP [--ros-port 15054]
  --fps 1..15 --minimum-sources 2|3|4 --preview-hz HZ
  --workspace-x-min M --workspace-x-max M
  --record [--record-stem NAME] --headless --duration S
EOF
}

while (($#)); do
  case "$1" in
    --calibration-record) calibration_record="${2:?}"; shift 2 ;;
    --extrinsics) extrinsics="${2:?}"; shift 2 ;;
    --output-host) output_host="${2:?}"; shift 2 ;;
    --output-port) output_port="${2:?}"; shift 2 ;;
    --monitor-host) monitor_host="${2:?}"; shift 2 ;;
    --monitor-port) monitor_port="${2:?}"; shift 2 ;;
    --ros-host) ros_host="${2:?}"; shift 2 ;;
    --ros-port) ros_port="${2:?}"; shift 2 ;;
    --fps) fps="${2:?}"; shift 2 ;;
    --minimum-sources) minimum_sources="${2:?}"; shift 2 ;;
    --preview-hz) preview_hz="${2:?}"; shift 2 ;;
    --workspace-x-min) workspace_x_min="${2:?}"; shift 2 ;;
    --workspace-x-max) workspace_x_max="${2:?}"; shift 2 ;;
    --record) record=1; shift ;;
    --record-stem) record_stem="${2:?}"; shift 2 ;;
    --headless) headless=1; shift ;;
    --duration) duration="${2:?}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -x "$zed_python" ]] || { echo "ZED Python bulunamadi: $zed_python" >&2; exit 1; }
case "$minimum_sources" in 2|3|4) ;; *) echo "--minimum-sources 2, 3 veya 4 olmali." >&2; exit 2 ;; esac
if [[ -n "$calibration_record" && -n "$extrinsics" ]]; then
  echo "Ham kalibrasyon kaydi ile runtime extrinsic ayni anda verilmez." >&2
  exit 2
fi

sources=(39504762:16000 31571870:16002 33773329:16004 34760587:16006)
previews=(39504762:16100 31571870:16102 33773329:16104 34760587:16106)
args=("$receiver")
for item in "${sources[@]}"; do args+=(--source "$item"); done
if (( ! headless )); then
  for item in "${previews[@]}"; do args+=(--preview-source "$item"); done
fi
args+=(
  --minimum-sources "$minimum_sources"
  --max-sync-ms 80
  --preferred-full-set-spread-ms 40
  --full-set-wait-ms 20
  --source-timeout-ms 250
  --max-joint-spread-m 0.18
  --max-pose-disagreement-m 0.22
  --max-alignment-translation-m 0.25
  --max-temporal-prediction-ms 70
  --workspace-x-min-m "$workspace_x_min"
  --workspace-x-max-m "$workspace_x_max"
  --workspace-hysteresis-m 0.15
  --preview-hz "$preview_hz"
  --duration "$duration"
  --record-stem "$record_stem"
  --record-detail research
)
[[ -n "$extrinsics" ]] && args+=(--extrinsics "$extrinsics")
[[ -n "$calibration_record" ]] && args+=(--calibration-record "$calibration_record")
if [[ -n "$output_host" ]]; then
  args+=(--output-host "$output_host" --output-port "$output_port" --output-max-hz "$fps")
fi
if [[ -n "$monitor_host" ]]; then
  args+=(--monitor-host "$monitor_host" --monitor-port "$monitor_port" --monitor-max-hz "$fps")
fi
if [[ -n "$ros_host" ]]; then
  args+=(--ros-host "$ros_host" --ros-port "$ros_port" --ros-max-hz "$fps")
fi
(( record )) && args+=(--record)
(( headless )) && args+=(--headless)

cd "$project"
exec "$zed_python" "${args[@]}"
