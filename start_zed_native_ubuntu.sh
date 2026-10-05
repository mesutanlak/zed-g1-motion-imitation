#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
model="medium"
fps="15"
record=0
record_svo2=0
hand_tracking=0
hand_model="${HAND_MODEL:-$project/models/hand_landmarker.task}"
hand_delegate="cpu"
hand_inference_fps="10"
hand_roi_scale="1.35"
hand_roi_min_px="128"
hand_roi_max_px="360"
dex3_retargeting=1

usage() {
  cat <<'EOF'
Kullanim: ./start_zed_native_ubuntu.sh [secenekler]
  --model fast|medium|accurate
  --fps N
  --record
  --record-svo2
  --hand-tracking
  --hand-model PATH
  --hand-delegate cpu|gpu
  --hand-inference-fps N
  --hand-roi-scale N
  --hand-roi-min-px N
  --hand-roi-max-px N
  --no-dex3-retargeting
  --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --model) model="${2:?--model degeri gerekli}"; shift 2 ;;
    --fps) fps="${2:?--fps degeri gerekli}"; shift 2 ;;
    --record) record=1; shift ;;
    --record-svo2) record_svo2=1; shift ;;
    --hand-tracking) hand_tracking=1; shift ;;
    --hand-model) hand_model="${2:?--hand-model degeri gerekli}"; shift 2 ;;
    --hand-delegate) hand_delegate="${2:?--hand-delegate degeri gerekli}"; shift 2 ;;
    --hand-inference-fps) hand_inference_fps="${2:?--hand-inference-fps degeri gerekli}"; shift 2 ;;
    --hand-roi-scale) hand_roi_scale="${2:?--hand-roi-scale degeri gerekli}"; shift 2 ;;
    --hand-roi-min-px) hand_roi_min_px="${2:?--hand-roi-min-px degeri gerekli}"; shift 2 ;;
    --hand-roi-max-px) hand_roi_max_px="${2:?--hand-roi-max-px degeri gerekli}"; shift 2 ;;
    --no-dex3-retargeting) dex3_retargeting=0; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$model" in fast|medium|accurate) ;; *) echo "Gecersiz model: $model" >&2; exit 2 ;; esac
case "$hand_delegate" in cpu|gpu) ;; *) echo "Gecersiz el delegate: $hand_delegate" >&2; exit 2 ;; esac
if [[ ! -x "$zed_python" ]]; then
  echo "ZED Python bulunamadi: $zed_python" >&2
  exit 1
fi
if ! lsusb | grep -Eiq '2b03|stereolabs'; then
  echo "ZED kamera USB aygitlari arasinda gorunmuyor." >&2
  exit 1
fi
if (( hand_tracking )); then
  if [[ ! -f "$hand_model" ]]; then
    echo "MediaPipe hand modeli bulunamadi: $hand_model" >&2
    echo "Resmi hand_landmarker.task dosyasini bu konuma koyun veya --hand-model PATH kullanin." >&2
    exit 1
  fi
  if ! "$zed_python" -c 'import mediapipe' >/dev/null 2>&1; then
    echo "MediaPipe ZED ortaminda kurulu degil." >&2
    echo "Once ./install/setup_hand_tracking_ubuntu24.sh calistirin." >&2
    exit 1
  fi
fi

args=(
  "$project/zed_g1_skeleton.py"
  --model "$model"
  --fps "$fps"
  --stream-host 127.0.0.1
  --stream-port 15050
  --stream-max-hz "$fps"
  --monitor-host 127.0.0.1
  --monitor-port 15052
  --monitor-max-hz "$fps"
)
(( record )) && args+=(--record)
(( record_svo2 )) && args+=(--record-svo2)
if (( hand_tracking )); then
  args+=(
    --hand-tracking
    --hand-model "$hand_model"
    --hand-delegate "$hand_delegate"
    --hand-inference-fps "$hand_inference_fps"
    --hand-roi-scale "$hand_roi_scale"
    --hand-roi-min-px "$hand_roi_min_px"
    --hand-roi-max-px "$hand_roi_max_px"
    --hand-stream-host 127.0.0.1
    --hand-stream-port 16200
  )
  (( dex3_retargeting )) && args+=(--dex3-retargeting) || args+=(--no-dex3-retargeting)
fi

echo "ZED BODY_38 -> GMR 127.0.0.1:15050 + Rerun 127.0.0.1:15052"
if (( hand_tracking )); then
  echo "MediaPipe 21-landmark el takibi -> Rerun + DDS-free Dex3 (${hand_inference_fps} Hz, ${hand_delegate})"
fi
exec "$zed_python" "${args[@]}"
