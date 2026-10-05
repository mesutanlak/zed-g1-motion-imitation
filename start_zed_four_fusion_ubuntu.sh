#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
extrinsics=""
reference_serial="33773329"
minimum_sources=3
fps=15
preview_hz=10
record=0
headless=0
skip_gmr_check=0
hand_tracking=0
hand_max_age_ms="120"
hand_max_spread_ms="70"
disable_dex3_retargeting=0
disable_single_view_hand_depth=0
dex3_official_root=""
dex3_official_python=""
record_detail="research"

usage() {
  cat <<'EOF'
Kullanim: ./start_zed_four_fusion_ubuntu.sh [secenekler]
  --extrinsics PATH
  --reference-serial 33773329
  --minimum-sources 2|3|4
  --fps 1..15 --preview-hz HZ
  --hand-tracking [--hand-max-age-ms 120] [--hand-max-spread-ms 70]
  --disable-dex3-retargeting --disable-single-view-hand-depth
  --dex3-official-root PATH --dex3-official-python PATH
  --record-detail minimal|research|full
  --record --headless --skip-gmr-check
EOF
}

while (($#)); do
  case "$1" in
    --extrinsics) extrinsics="${2:?}"; shift 2 ;;
    --reference-serial) reference_serial="${2:?}"; shift 2 ;;
    --minimum-sources) minimum_sources="${2:?}"; shift 2 ;;
    --fps) fps="${2:?}"; shift 2 ;;
    --preview-hz) preview_hz="${2:?}"; shift 2 ;;
    --hand-tracking) hand_tracking=1; shift ;;
    --hand-max-age-ms) hand_max_age_ms="${2:?}"; shift 2 ;;
    --hand-max-spread-ms) hand_max_spread_ms="${2:?}"; shift 2 ;;
    --disable-dex3-retargeting) disable_dex3_retargeting=1; shift ;;
    --disable-single-view-hand-depth) disable_single_view_hand_depth=1; shift ;;
    --dex3-official-root) dex3_official_root="${2:?}"; shift 2 ;;
    --dex3-official-python) dex3_official_python="${2:?}"; shift 2 ;;
    --record-detail) record_detail="${2:?}"; shift 2 ;;
    --record) record=1; shift ;;
    --headless) headless=1; shift ;;
    --skip-gmr-check) skip_gmr_check=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -x "$zed_python" ]] || { echo "ZED Python bulunamadi: $zed_python" >&2; exit 1; }
case "$minimum_sources" in 2|3|4) ;; *) echo "--minimum-sources 2, 3 veya 4 olmali." >&2; exit 2 ;; esac
case "$record_detail" in minimal|research|full) ;; *) echo "--record-detail minimal, research veya full olmali." >&2; exit 2 ;; esac

if [[ -z "$extrinsics" ]]; then
  inbox="$project/four json"
  shopt -s nullglob
  candidates=("$inbox"/*.json)
  if ((${#candidates[@]} != 1)); then
    echo "'$inbox' icinde tam bir JSON olmali; bulunan=${#candidates[@]}." >&2
    exit 1
  fi
  selected="${candidates[0]}"
  schema="$($zed_python -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8-sig")).get("schema", ""))' "$selected")"
  if [[ "$schema" == "zed_body38_distributed_extrinsics/v1" ]]; then
    extrinsics="$selected"
  else
    mkdir -p "$project/config/zed_four"
    extrinsics="$project/config/zed_four/active_zed360_body38_extrinsics.json"
    "$zed_python" "$project/zed_four_camera_test/convert_zed360_extrinsics.py" \
      --input "$selected" \
      --output "$extrinsics" \
      --world-poses-jsonl "$project/config/zed_four/active_zed360_camera_world_poses.jsonl" \
      --reference-serial "$reference_serial"
  fi
fi
[[ -f "$extrinsics" ]] || { echo "Extrinsic bulunamadi: $extrinsics" >&2; exit 1; }

"$zed_python" "$project/zed_four_camera_test/validate_distributed_extrinsics.py" \
  --input "$extrinsics" \
  --expected-serials "31571870,33773329,34760587,39504762" \
  --reference-serial "$reference_serial"

if (( ! skip_gmr_check )); then
  if ! ss -lunH | grep -Eq '(^|:)15050([[:space:]]|$)'; then
    echo "Yerel GMR UDP 15050 dinleyicisi yok. Once Isaac/GMR baslatin veya --skip-gmr-check kullanin." >&2
    exit 1
  fi
fi

args=(
  --extrinsics "$extrinsics"
  --output-host 127.0.0.1 --output-port 15050
  --monitor-host 127.0.0.1 --monitor-port 15052
  --ros-host 127.0.0.1 --ros-port 15054
  --fps "$fps"
  --minimum-sources "$minimum_sources"
  --preview-hz "$preview_hz"
  --record-detail "$record_detail"
)
if (( hand_tracking )); then
  args+=(
    --hand-tracking
    --hand-max-age-ms "$hand_max_age_ms"
    --hand-max-spread-ms "$hand_max_spread_ms"
  )
  (( disable_dex3_retargeting )) && args+=(--disable-dex3-retargeting)
  (( disable_single_view_hand_depth )) && args+=(--disable-single-view-hand-depth)
  [[ -n "$dex3_official_root" ]] && args+=(--dex3-official-root "$dex3_official_root")
  [[ -n "$dex3_official_python" ]] && args+=(--dex3-official-python "$dex3_official_python")
fi
(( record )) && args+=(--record)
(( headless )) && args+=(--headless)

echo "4-ZED Fusion | GMR=127.0.0.1:15050 | Rerun=:15052 | ROS=:15054 | kaynak>=${minimum_sources}/4"
exec "$project/zed_four_camera_test/start_distributed_receiver_ubuntu.sh" "${args[@]}"
