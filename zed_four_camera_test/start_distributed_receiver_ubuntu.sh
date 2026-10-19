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
fps=30
minimum_sources=3
preview_hz=10
headless=0
record=0
duration=0
record_stem="four_body38_fusion"
record_detail="research"
workspace_x_min="2.0"
workspace_x_max="4.0"
hand_tracking=0
hand_max_age_ms="120"
hand_max_spread_ms="70"
disable_dex3_retargeting=0
disable_single_view_hand_depth=0
dex3_official_root=""
dex3_official_python=""
source_priorities=()
excluded_sources=()
source_priority_recovery=1
max_sync_ms=""
preferred_full_set_spread_ms=""
full_set_wait_ms=""

usage() {
  cat <<'EOF'
Kullanim: start_distributed_receiver_ubuntu.sh [secenekler]
  --calibration-record PATH       Yeni extrinsic icin ham 4'lu kayit
  --extrinsics PATH               Calibre canli fusion
  --output-host IP [--output-port 15050]
  --monitor-host IP [--monitor-port 15052]
  --ros-host IP [--ros-port 15054]
  --fps 1..30 --minimum-sources 2|3|4 --preview-hz HZ
  --max-sync-ms MS --preferred-full-set-spread-ms MS
  --full-set-wait-ms MS
  --source-priority SERIAL:WEIGHT (tekrarlanabilir)
  --exclude-source SERIAL          (yalniz runtime; tekrarlanabilir)
  --no-source-priority-recovery
  --workspace-x-min M --workspace-x-max M
  --hand-tracking [--hand-max-age-ms 120] [--hand-max-spread-ms 70]
  --disable-dex3-retargeting --disable-single-view-hand-depth
  --dex3-official-root PATH --dex3-official-python PATH
  --record [--record-stem NAME] [--record-detail minimal|research|full]
  --headless --duration S
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
    --max-sync-ms) max_sync_ms="${2:?}"; shift 2 ;;
    --preferred-full-set-spread-ms) preferred_full_set_spread_ms="${2:?}"; shift 2 ;;
    --full-set-wait-ms) full_set_wait_ms="${2:?}"; shift 2 ;;
    --preview-hz) preview_hz="${2:?}"; shift 2 ;;
    --source-priority) source_priorities+=("${2:?}"); shift 2 ;;
    --exclude-source) excluded_sources+=("${2:?}"); shift 2 ;;
    --no-source-priority-recovery) source_priority_recovery=0; shift ;;
    --workspace-x-min) workspace_x_min="${2:?}"; shift 2 ;;
    --workspace-x-max) workspace_x_max="${2:?}"; shift 2 ;;
    --hand-tracking) hand_tracking=1; shift ;;
    --hand-max-age-ms) hand_max_age_ms="${2:?}"; shift 2 ;;
    --hand-max-spread-ms) hand_max_spread_ms="${2:?}"; shift 2 ;;
    --disable-dex3-retargeting) disable_dex3_retargeting=1; shift ;;
    --disable-single-view-hand-depth) disable_single_view_hand_depth=1; shift ;;
    --dex3-official-root) dex3_official_root="${2:?}"; shift 2 ;;
    --dex3-official-python) dex3_official_python="${2:?}"; shift 2 ;;
    --record) record=1; shift ;;
    --record-stem) record_stem="${2:?}"; shift 2 ;;
    --record-detail) record_detail="${2:?}"; shift 2 ;;
    --headless) headless=1; shift ;;
    --duration) duration="${2:?}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -x "$zed_python" ]] || { echo "ZED Python bulunamadi: $zed_python" >&2; exit 1; }
case "$minimum_sources" in 2|3|4) ;; *) echo "--minimum-sources 2, 3 veya 4 olmali." >&2; exit 2 ;; esac
case "$record_detail" in minimal|research|full) ;; *) echo "--record-detail minimal, research veya full olmali." >&2; exit 2 ;; esac
if [[ -n "$calibration_record" && -n "$extrinsics" ]]; then
  echo "Ham kalibrasyon kaydi ile runtime extrinsic ayni anda verilmez." >&2
  exit 2
fi
if [[ -n "$calibration_record" && ${#excluded_sources[@]} -gt 0 ]]; then
  echo "Kalibrasyon kaydinda kamera dislanamaz; dort kamera zorunludur." >&2
  exit 2
fi

# Calibration must never pair observations farther apart than one 30 Hz
# capture interval.  Runtime keeps its existing wider dropout-tolerant window;
# callers may override either profile explicitly.
if [[ -n "$calibration_record" ]]; then
  max_sync_ms="${max_sync_ms:-33}"
  preferred_full_set_spread_ms="${preferred_full_set_spread_ms:-20}"
  full_set_wait_ms="${full_set_wait_ms:-35}"
else
  max_sync_ms="${max_sync_ms:-80}"
  preferred_full_set_spread_ms="${preferred_full_set_spread_ms:-40}"
  full_set_wait_ms="${full_set_wait_ms:-20}"
fi
"$zed_python" - "$max_sync_ms" "$preferred_full_set_spread_ms" "$full_set_wait_ms" <<'PY'
import math, sys
values = [float(value) for value in sys.argv[1:]]
if any(not math.isfinite(value) or value < 0.0 for value in values):
    raise SystemExit("Senkronizasyon pencereleri sonlu ve >= 0 olmali.")
if values[1] > values[0]:
    raise SystemExit("--preferred-full-set-spread-ms, --max-sync-ms degerini asamaz.")
PY

sources=(39504762:16000 31571870:16002 33773329:16004 34760587:16006)
previews=(39504762:16100 31571870:16102 33773329:16104 34760587:16106)
hands=(39504762:16200 31571870:16202 33773329:16204 34760587:16206)
declare -A excluded_source_set=()
for serial in "${excluded_sources[@]}"; do
  case "$serial" in
    39504762|31571870|33773329|34760587) ;;
    *) echo "Bilinmeyen --exclude-source seri numarasi: $serial" >&2; exit 2 ;;
  esac
  if [[ -n "${excluded_source_set[$serial]:-}" ]]; then
    echo "Ayni --exclude-source seri numarasi birden cok verildi: $serial" >&2
    exit 2
  fi
  excluded_source_set[$serial]=1
done
if ((${#excluded_source_set[@]})); then
  filtered_sources=()
  filtered_previews=()
  filtered_hands=()
  for index in "${!sources[@]}"; do
    serial="${sources[$index]%%:*}"
    [[ -n "${excluded_source_set[$serial]:-}" ]] && continue
    filtered_sources+=("${sources[$index]}")
    filtered_previews+=("${previews[$index]}")
    filtered_hands+=("${hands[$index]}")
  done
  sources=("${filtered_sources[@]}")
  previews=("${filtered_previews[@]}")
  hands=("${filtered_hands[@]}")
fi
if ((${#sources[@]} < 3)); then
  echo "Runtime fusion icin en az uc kamera kalmali; kalan=${#sources[@]}." >&2
  exit 2
fi
if ((minimum_sources > ${#sources[@]})); then
  echo "--minimum-sources=$minimum_sources kalan kamera sayisini (${#sources[@]}) asamaz." >&2
  exit 2
fi
args=("$receiver")
for item in "${sources[@]}"; do args+=(--source "$item"); done
if (( ! headless )); then
  for item in "${previews[@]}"; do args+=(--preview-source "$item"); done
fi
if (( hand_tracking )); then
  for item in "${hands[@]}"; do args+=(--hand-source "$item"); done
  args+=(
    --hand-tracking
    --hand-max-age-ms "$hand_max_age_ms"
    --hand-max-spread-ms "$hand_max_spread_ms"
  )
  (( disable_dex3_retargeting )) && args+=(--no-dex3-retargeting)
  (( disable_single_view_hand_depth )) && args+=(--no-hand-single-view-depth)
  [[ -n "$dex3_official_root" ]] && args+=(--dex3-official-root "$dex3_official_root")
  [[ -n "$dex3_official_python" ]] && args+=(--dex3-official-python "$dex3_official_python")
fi
args+=(
  --minimum-sources "$minimum_sources"
  --max-sync-ms "$max_sync_ms"
  --preferred-full-set-spread-ms "$preferred_full_set_spread_ms"
  --full-set-wait-ms "$full_set_wait_ms"
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
  --record-detail "$record_detail"
)
for item in "${source_priorities[@]}"; do
  args+=(--source-priority "$item")
done
(( source_priority_recovery )) || args+=(--no-source-priority-recovery)
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
