#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
serial=""
target_host=""
target_port=""
source_host_id="$(hostname)"
fps=15
model="medium"
depth_mode="neural-light"
frame_integrity_mode="off"
distance_min="1.0"
distance_max="5.25"
enforce_distance_gate=1
preview_host=""
preview_port="16100"
preview_hz="8"
hand_tracking=0
hand_model="$project/models/hand_landmarker.task"
hand_delegate="cpu"
hand_inference_fps="8"
hand_port="16200"
record_local=0
record_svo2=0
output_dir="$project/recordings"
record_stem=""

usage() {
  cat <<'EOF'
Kullanim: start_distributed_source_ubuntu.sh --serial N --target-host IP --target-port PORT [secenekler]
  --source-host-id NAME
  --fps 15|30|60
  --model fast|medium|accurate
  --depth-mode neural-light|neural|performance
  --frame-integrity-mode off|monitor|strict
  --distance-min M --distance-max M
  --disable-distance-gate
  --preview-host IP --preview-port PORT --preview-hz HZ
  --hand-tracking --hand-model PATH --hand-port PORT
  --record-local | --record-svo2
  --output-dir PATH --record-stem NAME
EOF
}

while (($#)); do
  case "$1" in
    --serial) serial="${2:?}"; shift 2 ;;
    --target-host) target_host="${2:?}"; shift 2 ;;
    --target-port) target_port="${2:?}"; shift 2 ;;
    --source-host-id) source_host_id="${2:?}"; shift 2 ;;
    --fps) fps="${2:?}"; shift 2 ;;
    --model) model="${2:?}"; shift 2 ;;
    --depth-mode) depth_mode="${2:?}"; shift 2 ;;
    --frame-integrity-mode) frame_integrity_mode="${2:?}"; shift 2 ;;
    --distance-min) distance_min="${2:?}"; shift 2 ;;
    --distance-max) distance_max="${2:?}"; shift 2 ;;
    --disable-distance-gate) enforce_distance_gate=0; shift ;;
    --preview-host) preview_host="${2:?}"; shift 2 ;;
    --preview-port) preview_port="${2:?}"; shift 2 ;;
    --preview-hz) preview_hz="${2:?}"; shift 2 ;;
    --hand-tracking) hand_tracking=1; shift ;;
    --hand-model) hand_model="${2:?}"; shift 2 ;;
    --hand-delegate) hand_delegate="${2:?}"; shift 2 ;;
    --hand-inference-fps) hand_inference_fps="${2:?}"; shift 2 ;;
    --hand-port) hand_port="${2:?}"; shift 2 ;;
    --record-local) record_local=1; shift ;;
    --record-svo2) record_svo2=1; shift ;;
    --output-dir) output_dir="${2:?}"; shift 2 ;;
    --record-stem) record_stem="${2:?}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$serial" =~ ^[0-9]+$ ]] || { echo "--serial zorunlu ve sayisal olmali." >&2; exit 2; }
[[ -n "$target_host" ]] || { echo "--target-host zorunlu." >&2; exit 2; }
[[ "$target_port" =~ ^[0-9]+$ ]] || { echo "--target-port zorunlu ve sayisal olmali." >&2; exit 2; }
case "$fps" in 15|30|60) ;; *) echo "Gecersiz --fps: $fps" >&2; exit 2 ;; esac
case "$model" in fast|medium|accurate) ;; *) echo "Gecersiz --model: $model" >&2; exit 2 ;; esac
case "$depth_mode" in neural-light|neural|performance) ;; *) echo "Gecersiz --depth-mode: $depth_mode" >&2; exit 2 ;; esac
case "$frame_integrity_mode" in off|monitor|strict) ;; *) echo "Gecersiz --frame-integrity-mode: $frame_integrity_mode" >&2; exit 2 ;; esac

[[ -x "$zed_python" ]] || { echo "ZED Python bulunamadi: $zed_python" >&2; exit 1; }
mkdir -p "$output_dir"

args=(
  "$project/zed_g1_skeleton.py"
  --serial "$serial"
  --fps "$fps"
  --model "$model"
  --depth-mode "$depth_mode"
  --headless
  --frame-integrity-mode "$frame_integrity_mode"
  --source-host-id "$source_host_id"
  --distance-min "$distance_min"
  --distance-max "$distance_max"
  --stream-host "$target_host"
  --stream-port "$target_port"
  --stream-max-hz "$fps"
  --output-dir "$output_dir"
)
(( enforce_distance_gate )) && args+=(--enforce-distance-gate)
if [[ -n "$preview_host" ]]; then
  args+=(
    --preview-stream-host "$preview_host"
    --preview-stream-port "$preview_port"
    --preview-stream-max-hz "$preview_hz"
    --preview-stream-width 640
    --preview-jpeg-quality 65
  )
fi
if [[ -n "$record_stem" ]]; then
  args+=(--record-stem "$record_stem")
fi
if (( record_svo2 )); then
  args+=(--record --record-svo2)
elif (( record_local )); then
  args+=(--record)
fi
if (( hand_tracking )); then
  [[ -f "$hand_model" ]] || { echo "El modeli bulunamadi: $hand_model" >&2; exit 1; }
  args+=(
    --hand-tracking
    --hand-model "$hand_model"
    --hand-delegate "$hand_delegate"
    --hand-inference-fps "$hand_inference_fps"
    --hand-stream-host "$target_host"
    --hand-stream-port "$hand_port"
  )
fi

echo "ZED $serial | BODY_38 -> $target_host:$target_port | preview=${preview_host:-kapali}:$preview_port"
exec "$zed_python" "${args[@]}"
