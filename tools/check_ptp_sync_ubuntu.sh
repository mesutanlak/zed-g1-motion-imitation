#!/usr/bin/env bash
set -euo pipefail

interface=""
while (($#)); do
  case "$1" in
    --interface) interface="${2:?}"; shift 2 ;;
    --help|-h)
      echo "Kullanim: ./tools/check_ptp_sync_ubuntu.sh [--interface enp129s0]"
      exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; exit 2 ;;
  esac
done
if [[ -z "$interface" ]]; then
  interface="$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.50\./ {print $2; exit}')"
fi
[[ -n "$interface" ]] || { echo "PTP arayuzu bulunamadi." >&2; exit 1; }

echo "Arayuz: $interface"
ethtool -T "$interface" 2>/dev/null | sed -n '1,20p' || true
echo
systemctl --no-pager --full status g1-zed-ptp.service | sed -n '1,18p'
echo
if command -v pmc >/dev/null; then
  sudo pmc -u -b 0 'GET TIME_STATUS_NP' 2>/dev/null || true
  sudo pmc -u -b 0 'GET PORT_DATA_SET' 2>/dev/null || true
fi
echo
journalctl -u g1-zed-ptp.service -n 30 --no-pager | tail -30
