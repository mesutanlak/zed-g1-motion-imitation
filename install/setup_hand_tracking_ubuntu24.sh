#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
zed_python="${ZED_PYTHON:-$root/envs/zed/bin/python}"
default_model="$project/models/hand_landmarker.task"

if [[ ! -x "$zed_python" ]]; then
  echo "ZED Python bulunamadi: $zed_python" >&2
  echo "Once ./install/setup_zed_ubuntu24.sh calistirin." >&2
  exit 1
fi

"$zed_python" -m pip install -r "$project/requirements-hand.txt"
"$zed_python" - <<'PY'
import cv2
import mediapipe
import numpy
import pyzed.sl

print("ZED Python API: OK")
print("MediaPipe:", mediapipe.__version__)
print("NumPy:", numpy.__version__)
print("OpenCV:", cv2.__version__)
PY
"$zed_python" -m pip check

if [[ -f "$default_model" ]]; then
  echo "MediaPipe modeli hazir: $default_model"
else
  echo
  echo "Bagimliliklar hazir. Resmi hand_landmarker.task modelini su konuma koyun:"
  echo "  $default_model"
  echo "Model otomatik indirilmez. Alternatif konum icin baslaticida --hand-model PATH kullanin."
fi
