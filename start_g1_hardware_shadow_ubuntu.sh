#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
python="${UNITREE_PYTHON:-$root/envs/unitree_sdk2/bin/python}"
interface="${1:-}"
[[ -n "$interface" ]] || {
  echo "Kullanim: ./start_g1_hardware_shadow_ubuntu.sh ROBOT_ETHERNET_INTERFACE" >&2
  exit 2
}
[[ -x "$python" ]] || { echo "Unitree Python bulunamadi: $python" >&2; exit 1; }
output="$project/hardware_recordings/g1_shadow_$(date +%Y%m%d_%H%M%S).jsonl"
mkdir -p "$(dirname "$output")"
echo "SHADOW ONLY: motor komutu gonderilmez. Cikti: $output"
exec "$python" "$project/hardware/g1_shadow_validator.py" \
  --network-interface "$interface" --listen-port 15055 --output "$output"
