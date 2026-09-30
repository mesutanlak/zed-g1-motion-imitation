#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
input=""
output=""
world_poses=""
reference_serial="33773329"
max_samples="5000"
activate=0

usage() {
  cat <<'EOF'
Kullanim: start_distributed_calibration_ubuntu.sh --input CAPTURE.jsonl \
  --output EXTRINSICS.json [--world-poses OUTPUT.jsonl] \
  [--reference-serial 33773329] [--max-samples 5000] [--activate]
EOF
}

while (($#)); do
  case "$1" in
    --input) input="${2:?}"; shift 2 ;;
    --output) output="${2:?}"; shift 2 ;;
    --world-poses) world_poses="${2:?}"; shift 2 ;;
    --reference-serial) reference_serial="${2:?}"; shift 2 ;;
    --max-samples) max_samples="${2:?}"; shift 2 ;;
    --activate) activate=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$input" && -n "$output" ]] || { usage >&2; exit 2; }
[[ -f "$input" ]] || { echo "Kalibrasyon kaydi bulunamadi: $input" >&2; exit 1; }
[[ -x "$zed_python" ]] || { echo "ZED Python bulunamadi: $zed_python" >&2; exit 1; }
if [[ -z "$world_poses" ]]; then
  world_poses="${output%.json}_world_poses.jsonl"
fi

"$zed_python" "$project/zed_four_camera_test/calibrate_distributed_body38.py" \
  --input "$input" \
  --output "$output" \
  --world-poses-jsonl "$world_poses" \
  --reference-serial "$reference_serial" \
  --max-samples "$max_samples"

if (( activate )); then
  active_dir="$project/config/zed_four"
  inbox="$project/four json"
  mkdir -p "$active_dir" "$inbox"
  shopt -s nullglob
  existing=("$inbox"/*.json)
  for item in "${existing[@]}"; do
    [[ "$item" == "$inbox/fourkamera.json" ]] || {
      echo "Etkinlestirme durdu: four json icinde baska JSON var: $item" >&2
      exit 1
    }
  done
  cp -- "$output" "$active_dir/active_distributed_body38_extrinsics.json"
  cp -- "$world_poses" "$active_dir/active_four_camera_world_poses.jsonl"
  cp -- "$output" "$inbox/fourkamera.json"
  echo "AKTIF EXTRINSIC: $active_dir/active_distributed_body38_extrinsics.json"
  echo "FOUR JSON: $inbox/fourkamera.json"
fi
