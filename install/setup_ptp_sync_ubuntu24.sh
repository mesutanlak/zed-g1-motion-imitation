#!/usr/bin/env bash
set -euo pipefail

role=""
interface=""
domain="24"

usage() {
  cat <<'EOF'
Kullanim: sudo ./install/setup_ptp_sync_ubuntu24.sh --role main-pc|laptop [--interface enp129s0] [--domain 24]

Ana PC PTP sunucusu, laptop istemci olur. Betik NIC donanim timestamp
destegini otomatik algilar; destek yoksa linuxptp software timestamping
kullanir. Iki bilgisayarda ayni domain ve dogrudan Ethernet hatti kullanin.
EOF
}

while (($#)); do
  case "$1" in
    --role) role="${2:?}"; shift 2 ;;
    --interface) interface="${2:?}"; shift 2 ;;
    --domain) domain="${2:?}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$role" == "main-pc" || "$role" == "laptop" ]] || {
  echo "--role main-pc veya laptop olmali." >&2
  exit 2
}
[[ "$domain" =~ ^[0-9]+$ ]] && (( domain >= 0 && domain <= 127 )) || {
  echo "--domain 0..127 olmali." >&2
  exit 2
}
if [[ -z "$interface" ]]; then
  interface="$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.50\./ {print $2; exit}')"
fi
[[ -n "$interface" && -d "/sys/class/net/$interface" ]] || {
  echo "192.168.50.x Ethernet arayuzu bulunamadi; --interface verin." >&2
  exit 1
}
if [[ "$(id -u)" -ne 0 ]]; then
  echo "Bu kurulum root ister: sudo $0 --role $role --interface $interface" >&2
  exit 1
fi

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y linuxptp ethtool

timestamp_mode="software"
ptp_flag="-S"
if ethtool -T "$interface" 2>/dev/null | grep -q "hardware-transmit" \
   && ethtool -T "$interface" 2>/dev/null | grep -q "hardware-receive" \
   && ethtool -T "$interface" 2>/dev/null | grep -Eq 'PTP Hardware Clock: [0-9]+'; then
  timestamp_mode="hardware"
  ptp_flag="-H"
fi

install -d -m 0755 /etc/linuxptp
config="/etc/linuxptp/g1-zed-${role}.cfg"
{
  echo '[global]'
  echo "domainNumber ${domain}"
  echo 'network_transport UDPv4'
  echo 'delay_mechanism E2E'
  echo 'twoStepFlag 1'
  echo 'logAnnounceInterval 0'
  echo 'logSyncInterval -3'
  echo 'logMinDelayReqInterval -3'
  echo 'summary_interval 0'
  echo 'logging_level 6'
  if [[ "$role" == "main-pc" ]]; then
    echo 'serverOnly 1'
    echo 'priority1 1'
    echo 'priority2 1'
  else
    echo 'clientOnly 1'
    # One initial step is acceptable before camera processes start. Never
    # step CLOCK_REALTIME again during capture; subsequent error is slewed so
    # source timestamps cannot jump backwards in a live session.
    echo 'first_step_threshold 0.001'
    echo 'step_threshold 0.0'
  fi
  echo
  echo "[$interface]"
} >"$config"

service="/etc/systemd/system/g1-zed-ptp.service"
{
  echo '[Unit]'
  echo 'Description=G1 ZED dedicated-link PTP synchronization'
  echo 'After=network-online.target'
  echo 'Wants=network-online.target'
  echo
  echo '[Service]'
  echo 'Type=simple'
  echo "ExecStart=/usr/sbin/ptp4l ${ptp_flag} -4 -i ${interface} -f ${config} -m"
  echo 'Restart=on-failure'
  echo 'RestartSec=1'
  echo
  echo '[Install]'
  echo 'WantedBy=multi-user.target'
} >"$service"

if [[ "$timestamp_mode" == "hardware" ]]; then
  phc_service="/etc/systemd/system/g1-zed-phc2sys.service"
  if [[ "$role" == "main-pc" ]]; then
    phc_command="/usr/sbin/phc2sys -c ${interface} -s CLOCK_REALTIME -w -m"
  else
    phc_command="/usr/sbin/phc2sys -s ${interface} -c CLOCK_REALTIME -w -m"
  fi
  {
    echo '[Unit]'
    echo 'Description=G1 ZED PTP hardware clock to system clock synchronization'
    echo 'After=g1-zed-ptp.service'
    echo 'Requires=g1-zed-ptp.service'
    echo
    echo '[Service]'
    echo 'Type=simple'
    echo "ExecStart=${phc_command}"
    echo 'Restart=on-failure'
    echo 'RestartSec=1'
    echo
    echo '[Install]'
    echo 'WantedBy=multi-user.target'
  } >"$phc_service"
else
  rm -f /etc/systemd/system/g1-zed-phc2sys.service
fi

# A second wall-clock servo would fight PTP on the laptop. The main PC may
# continue using its normal Internet time source and distributes that clock.
if [[ "$role" == "laptop" ]]; then
  timedatectl set-ntp false || true
fi
systemctl daemon-reload
systemctl enable --now g1-zed-ptp.service
if [[ "$timestamp_mode" == "hardware" ]]; then
  systemctl enable --now g1-zed-phc2sys.service
else
  systemctl disable --now g1-zed-phc2sys.service 2>/dev/null || true
fi

echo "PTP hazir: role=$role interface=$interface mode=$timestamp_mode domain=$domain"
echo "Dogrulama: ./tools/check_ptp_sync_ubuntu.sh --interface $interface"
