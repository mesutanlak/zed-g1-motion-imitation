#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
role=""
expected_ip=""
peer_ip=""
expected_sdk=""
serials=()

usage() {
  cat <<'EOF'
Kullanim: test_four_camera_preflight_ubuntu.sh --role laptop|main-pc \
  --expected-ip IP --peer-ip IP --serial N --serial N [--expected-sdk 5.4.1]
EOF
}

while (($#)); do
  case "$1" in
    --role) role="${2:?}"; shift 2 ;;
    --expected-ip) expected_ip="${2:?}"; shift 2 ;;
    --peer-ip) peer_ip="${2:?}"; shift 2 ;;
    --serial) serials+=("${2:?}"); shift 2 ;;
    --expected-sdk) expected_sdk="${2:?}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$role" == "laptop" || "$role" == "main-pc" ]] || { usage >&2; exit 2; }
[[ -n "$expected_ip" && -n "$peer_ip" && ${#serials[@]} -eq 2 ]] || { usage >&2; exit 2; }
[[ -x "$zed_python" ]] || { echo "ZED Python bulunamadi: $zed_python" >&2; exit 1; }

failures=0
fail() { echo "HATA $*" >&2; failures=$((failures + 1)); }
ok() { echo "OK   $*"; }

echo "=== ${role^^} DORT KAMERA ON KONTROL ==="
if ip -4 -o address show | grep -Eq "[[:space:]]${expected_ip}/24([[:space:]]|$)"; then
  interface="$(ip -4 -o address show | awk -v value="$expected_ip/24" '$4 == value {print $2; exit}')"
  speed="$(cat "/sys/class/net/$interface/speed" 2>/dev/null || echo '?')"
  ok "Ethernet $interface | IP=$expected_ip/24 | hiz=${speed}Mb/s"
else
  fail "Beklenen Ethernet IPv4 yok: $expected_ip/24"
fi

if ping -c 8 -s 1400 -W 2 "$peer_ip"; then
  ok "Es bilgisayar ping 8/8: $peer_ip"
else
  fail "Es bilgisayara kayipsiz 1400-byte ping yok: $peer_ip"
fi

if command -v timedatectl >/dev/null && timedatectl show -p NTPSynchronized --value | grep -qx yes; then
  ok "Sistem saati NTP ile senkron | UTC=$(date -u +'%F %T.%3N')"
else
  fail "Sistem saati NTP ile senkron degil (timedatectl NTPSynchronized=no)."
fi

if nvidia-smi --query-gpu=name,driver_version --format=csv,noheader; then
  ok "NVIDIA surucusu calisiyor"
else
  fail "nvidia-smi calismiyor"
fi

sdk="$($zed_python -c 'import pyzed.sl as sl; print(sl.Camera.get_sdk_version())' 2>/dev/null || true)"
if [[ -n "$sdk" ]]; then
  ok "ZED SDK=$sdk"
  if [[ -n "$expected_sdk" && "$sdk" != "$expected_sdk" ]]; then
    fail "ZED SDK eslesmiyor: beklenen=$expected_sdk bulunan=$sdk"
  fi
else
  fail "pyzed yuklenemedi"
fi

device_text="$($zed_python "$project/zed_g1_skeleton.py" --list-devices 2>&1 || true)"
echo "$device_text"
for serial in "${serials[@]}"; do
  if grep -Eq "serial=${serial}([[:space:]]|$)" <<<"$device_text"; then
    ok "ZED $serial AVAILABLE"
  else
    fail "ZED $serial AVAILABLE listesinde yok"
  fi
done

ports=(16000 16002 16004 16006 16100 16102 16104 16106 16200 16202 16204 16206)
busy="$(ss -lunpH 2>/dev/null | awk -v list="${ports[*]}" '
  BEGIN { split(list, a); for (i in a) wanted[a[i]]=1 }
  { n=split($4, endpoint, ":"); if (wanted[endpoint[n]]) print }
')"
if [[ -n "$busy" ]]; then
  echo "$busy" >&2
  fail "Dort-kamera UDP portlarindan bazilari kullanimda"
else
  ok "Dort-kamera UDP portlari bos"
fi

if (( failures )); then
  echo "ON KONTROL: BASARISIZ ($failures sorun)" >&2
  exit 2
fi
echo "ON KONTROL: BASARILI"
