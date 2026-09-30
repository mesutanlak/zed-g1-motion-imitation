#!/usr/bin/env bash
set -euo pipefail

interface=""
address="192.168.50.11/24"

usage() {
  cat <<'EOF'
Kullanim: configure_four_camera_ethernet_ubuntu.sh --interface eno1 [--address 192.168.50.11/24]

Ethernet baglantisini statik, gatewaysiz bir kamera agina cevirir. Wi-Fi/Internet
varsayilan rota olarak kalir. Komut ag baglantisini kisa sure yeniden baslatir.
EOF
}

while (($#)); do
  case "$1" in
    --interface) interface="${2:?}"; shift 2 ;;
    --address) address="${2:?}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$interface" ]] || { usage >&2; exit 2; }
ip link show dev "$interface" >/dev/null 2>&1 || { echo "Arayuz bulunamadi: $interface" >&2; exit 1; }
[[ "$address" =~ ^[0-9.]+/24$ ]] || { echo "Bu rig icin adres /24 olmali: $address" >&2; exit 2; }

connection="$(nmcli -g GENERAL.CONNECTION device show "$interface")"
[[ -n "$connection" && "$connection" != "--" ]] || {
  echo "$interface icin NetworkManager baglantisi bulunamadi." >&2
  exit 1
}

echo "Baglanti=$connection | arayuz=$interface | yeni adres=$address | gateway=yok"
sudo nmcli connection modify "$connection" \
  ipv4.method manual \
  ipv4.addresses "$address" \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv6.method disabled
sudo nmcli connection up "$connection"
ip -4 address show dev "$interface"
