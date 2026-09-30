#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
zed_installer=""

usage() {
  cat <<'EOF'
Kullanim: setup_four_camera_laptop_ubuntu24.sh [--zed-installer ~/Downloads/ZED_....run]

Laptopa yalniz dort-kamera kaynak rolu icin gereken paketleri ve ZED Python
ortamini kurar. Isaac Sim, GMR, ROS veya Unitree paketlerini laptopa kurmaz.
ZED kurucusu lisans nedeniyle interaktiftir.
EOF
}

while (($#)); do
  case "$1" in
    --zed-installer) zed_installer="${2:?}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Bilinmeyen secenek: $1" >&2; usage >&2; exit 2 ;;
  esac
done

source /etc/os-release
[[ "${VERSION_ID:-}" == "24.04" ]] || {
  echo "Bu kurulum Ubuntu 24.04 icin; bulunan=${VERSION_ID:-bilinmiyor}." >&2
  exit 1
}

sudo apt update
sudo apt install -y \
  git git-lfs curl wget ca-certificates zstd \
  python3 python3-venv python3-pip \
  build-essential pkg-config \
  libgl1 libglib2.0-0t64 libx11-6 libxext6 libxrender1 libsm6 libglfw3 \
  iproute2 iputils-ping ethtool chrony
git lfs install

if ! nvidia-smi >/dev/null 2>&1; then
  echo "NVIDIA surucusu hazir degil. Once ubuntu-drivers ile surucuyu kurup yeniden baslatin." >&2
  exit 1
fi

if [[ ! -x /usr/local/zed/tools/ZED_Explorer ]]; then
  if [[ -z "$zed_installer" ]]; then
    cat >&2 <<'EOF'
ZED SDK kurulu degil. Ana PC ile ayni SDK kurucusunu resmi sayfadan indirin,
sonra bu betigi tekrar su sekilde calistirin:
  ./install/setup_four_camera_laptop_ubuntu24.sh --zed-installer ~/Downloads/ZED_SDK_....run
EOF
    exit 2
  fi
  [[ -f "$zed_installer" ]] || { echo "ZED kurucusu bulunamadi: $zed_installer" >&2; exit 1; }
  chmod +x "$zed_installer"
  echo "ZED lisansini okuyup kurucu sorularini yanitlayin. Body Tracking/AI modulunu atlamayin."
  "$zed_installer"
fi

bash "$project/install/setup_zed_ubuntu24.sh"
zed_python="${ZED_PYTHON:-$HOME/g1_isaaclab_project/envs/zed/bin/python}"
"$zed_python" -c 'import pyzed.sl as sl; print("ZED SDK:", sl.Camera.get_sdk_version())'
"$zed_python" "$project/zed_g1_skeleton.py" --self-test

echo "Laptop kaynak kurulumu hazir. Kameralari baglayip --list-devices ve preflight calistirin."
