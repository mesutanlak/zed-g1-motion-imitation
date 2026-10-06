#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
role=""
resolution="hd720"
fps=30
seconds=20
require_ptp=0

usage() {
  cat <<'EOF'
Kullanim: ./tools/capture_fixed_reference_board_ubuntu.sh --role main-pc|laptop [secenekler]
  --resolution hd720|hd1080|hd2k
  --fps 15|30|60
  --seconds 20
  --require-ptp

Normal BODY/GMR/Rerun hattini kapatip sabit 17x12, 20 mm checkerboard icin
iki yerel ZED'den guvenli kapanan SVO2 alir. Bu rigde varsayilan HD720, tum
levhayi gorur ve OUTER_QUAD_COARSE kamera-kayma kontrolu saglar.
EOF
}

while (($#)); do
  case "$1" in
    --role) role="${2:?}"; shift 2 ;;
    --resolution) resolution="${2:?}"; shift 2 ;;
    --fps) fps="${2:?}"; shift 2 ;;
    --seconds) seconds="${2:?}"; shift 2 ;;
    --require-ptp) require_ptp=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$role" in
  main-pc) serials=(31571870 33773329) ;;
  laptop) serials=(39504762 34760587) ;;
  *) echo "--role main-pc veya laptop olmali" >&2; exit 2 ;;
esac
[[ -x "$zed_python" ]] || { echo "ZED Python bulunamadi: $zed_python" >&2; exit 1; }
if (( require_ptp )); then
  systemctl is-active --quiet g1-zed-ptp.service || {
    echo "PTP aktif degil; once PTP servisini duzeltin." >&2
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

session="$(date +%Y%m%d_%H%M%S)"
output_dir="$project/recordings/fixed_reference_${session}_${role}"
mkdir -p "$output_dir"
pids=()

cleanup() {
  trap - INT TERM EXIT
  live=()
  for pid in "${pids[@]:-}"; do
    kill -0 "$pid" 2>/dev/null && live+=("$pid")
  done
  if ((${#live[@]})); then
    kill -INT "${live[@]}" 2>/dev/null || true
    wait "${live[@]}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

for serial in "${serials[@]}"; do
  output="$output_dir/zed_reference_${serial}_${session}.svo2"
  "$zed_python" "$project/tools/capture_fixed_reference_svo.py" \
    --serial "$serial" \
    --output "$output" \
    --resolution "$resolution" \
    --fps "$fps" \
    --seconds "$seconds" \
    > >(sed -u "s/^/[ZED $serial] /") \
    2> >(sed -u "s/^/[ZED $serial HATA] /" >&2) &
  pids+=("$!")
  sleep 1
done

status=0
for pid in "${pids[@]}"; do
  wait "$pid" || status=$?
done
pids=()
echo "Sabit referans oturumu: $output_dir"
exit "$status"
