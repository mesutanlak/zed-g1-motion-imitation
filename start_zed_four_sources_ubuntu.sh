#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
launcher="$project/zed_four_camera_test/start_distributed_source_ubuntu.sh"
role=""
main_pc_host="192.168.50.10"
fps=30
model="medium"
depth_mode="neural-light"
frame_integrity_mode="off"
distance_min="1.0"
distance_max="5.25"
calibration_mode=0
disable_calibration_distance_gate=0
preview_hz=4
record_local=0
record_svo2=0
hand_tracking=0
require_ptp=0
hand_model="$project/models/hand_landmarker.task"

usage() {
  cat <<'EOF'
Kullanim: ./start_zed_four_sources_ubuntu.sh --role laptop|main-pc [secenekler]
  --main-pc-host 192.168.50.10
  --fps 15|30
  --model fast|medium|accurate
  --depth-mode neural-light|neural|performance
  --frame-integrity-mode off|monitor|strict
  --distance-min M --distance-max M
  --calibration-mode [--disable-calibration-distance-gate]
  --preview-hz HZ
  --record-local | --record-svo2
  --hand-tracking [--hand-model PATH]
  --require-ptp  (g1-zed-ptp.service aktif degilse baslatma)

Bu betik iki kamerayi ayni terminalden yonetir. Ctrl+C ikisini de kapatir.
EOF
}

while (($#)); do
  case "$1" in
    --role) role="${2:?}"; shift 2 ;;
    --main-pc-host) main_pc_host="${2:?}"; shift 2 ;;
    --fps) fps="${2:?}"; shift 2 ;;
    --model) model="${2:?}"; shift 2 ;;
    --depth-mode) depth_mode="${2:?}"; shift 2 ;;
    --frame-integrity-mode) frame_integrity_mode="${2:?}"; shift 2 ;;
    --distance-min) distance_min="${2:?}"; shift 2 ;;
    --distance-max) distance_max="${2:?}"; shift 2 ;;
    --calibration-mode) calibration_mode=1; shift ;;
    --disable-calibration-distance-gate) disable_calibration_distance_gate=1; shift ;;
    --preview-hz) preview_hz="${2:?}"; shift 2 ;;
    --record-local) record_local=1; shift ;;
    --record-svo2) record_svo2=1; shift ;;
    --hand-tracking) hand_tracking=1; shift ;;
    --require-ptp) require_ptp=1; shift ;;
    --hand-model) hand_model="${2:?}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$role" in
  laptop)
    serials=(39504762 34760587)
    body_ports=(16000 16006)
    preview_ports=(16100 16106)
    hand_ports=(16200 16206)
    target_host="$main_pc_host"
    source_host_id="Laptop"
    ;;
  main-pc)
    serials=(31571870 33773329)
    body_ports=(16002 16004)
    preview_ports=(16102 16104)
    hand_ports=(16202 16204)
    target_host="127.0.0.1"
    source_host_id="MainPc"
    ;;
  *) echo "--role laptop veya main-pc olmali." >&2; usage >&2; exit 2 ;;
esac
case "$fps" in 15|30) ;; *) echo "Dort kamera icin --fps 15 veya 30 olmali." >&2; exit 2 ;; esac
[[ -x "$zed_python" ]] || { echo "ZED Python bulunamadi: $zed_python" >&2; exit 1; }
if (( require_ptp )); then
  systemctl is-active --quiet g1-zed-ptp.service || {
    echo "PTP aktif degil. Once install/setup_ptp_sync_ubuntu24.sh calistirin." >&2
    exit 1
  }
fi

device_text="$($zed_python "$project/zed_g1_skeleton.py" --list-devices 2>&1)" || {
  echo "$device_text" >&2
  exit 1
}
for serial in "${serials[@]}"; do
  grep -Eq "serial=${serial}([[:space:]]|$)" <<<"$device_text" || {
    echo "Bu hostta beklenen ZED bulunamadi: $serial" >&2
    echo "$device_text" >&2
    exit 1
  }
done
if [[ "$role" == "laptop" ]]; then
  ping -c 2 -W 2 "$main_pc_host" >/dev/null || {
    echo "Ana PC'ye ping yok: $main_pc_host" >&2
    exit 1
  }
fi

pids=()
cleanup() {
  trap - INT TERM EXIT
  if ((${#pids[@]})); then
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

session="$(date +%Y%m%d_%H%M%S)"
for index in 0 1; do
  serial="${serials[$index]}"
  args=(
    --serial "$serial"
    --target-host "$target_host"
    --target-port "${body_ports[$index]}"
    --source-host-id "$source_host_id"
    --fps "$fps"
    --model "$model"
    --depth-mode "$depth_mode"
    --frame-integrity-mode "$frame_integrity_mode"
    --distance-min "$distance_min"
    --distance-max "$distance_max"
    --preview-host "$target_host"
    --preview-port "${preview_ports[$index]}"
    --preview-hz "$preview_hz"
    --record-stem "zed_body38_${serial}_${session}"
  )
  if (( calibration_mode && disable_calibration_distance_gate )); then
    args+=(--disable-distance-gate)
  fi
  (( record_svo2 )) && args+=(--record-svo2)
  (( ! record_svo2 && record_local )) && args+=(--record-local)
  if (( hand_tracking )); then
    args+=(--hand-tracking --hand-model "$hand_model" --hand-port "${hand_ports[$index]}")
  fi
  "$launcher" "${args[@]}" > >(sed -u "s/^/[ZED $serial] /") 2> >(sed -u "s/^/[ZED $serial HATA] /" >&2) &
  pids+=("$!")
  sleep 1
done

echo "4-ZED kaynak rolu=$role | host=$source_host_id | BODY/JPEG hedefi=$target_host"
echo "Seriler: ${serials[*]} | Ctrl+C iki kaynagi da guvenli kapatir."
if (( calibration_mode )); then
  echo "KALIBRASYON MODU: odada tek kisi olsun; iki preview'da da ayni operator LOCKED olmali."
fi

set +e
wait -n "${pids[@]}"
status=$?
set -e
echo "Kaynaklardan biri kapandi (kod=$status); diger kaynak da kapatiliyor." >&2
exit "$status"
