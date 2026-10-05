#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
failures=0

check_file() {
  if [[ -e "$1" ]]; then
    printf 'OK   %s\n' "$1"
  else
    printf 'YOK  %s\n' "$1" >&2
    failures=$((failures + 1))
  fi
}

if [[ "$(. /etc/os-release; printf '%s' "$VERSION_ID")" == "24.04" ]]; then
  echo "OK   Ubuntu 24.04"
else
  echo "YOK  Ubuntu 24.04 gerekli" >&2
  failures=$((failures + 1))
fi

check_file /usr/local/zed/tools/ZED_Explorer
check_file /opt/ros/jazzy/setup.bash
check_file "$root/envs/zed/bin/python"
check_file "$root/envs/gmr_zed/bin/python"
check_file "$root/envs/isaaclab30/bin/python"
check_file "$root/repos/IsaacLab/.git"
check_file "$root/repos/GMR/.git"
check_file "$root/repos/unitree_rl_lab/.git"
check_file "$root/repos/unitree_ros/.git"
check_file "$root/repos/unitree_sim_isaaclab/.git"

if [[ -x "$root/envs/zed/bin/python" ]]; then
  "$root/envs/zed/bin/python" -c 'import pyzed.sl; print("OK   pyzed")' || failures=$((failures + 1))
fi
if [[ -x "$root/envs/gmr_zed/bin/python" ]]; then
  "$root/envs/gmr_zed/bin/python" -c 'import general_motion_retargeting, mujoco, mink; print("OK   GMR")' || failures=$((failures + 1))
fi
if [[ -x "$root/envs/isaaclab30/bin/python" ]]; then
  "$root/envs/isaaclab30/bin/python" "$project/isaaclab_bridge/test_environment.py" || failures=$((failures + 1))
fi
if [[ -x "$root/envs/unitree_sdk2/bin/python" ]]; then
  "$root/envs/unitree_sdk2/bin/python" -c 'import unitree_sdk2py; print("OK   Unitree SDK2")' || failures=$((failures + 1))
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -c 'import rclpy; print("OK   ROS 2 Jazzy")' || failures=$((failures + 1))

driver="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n1)"
echo "OK   NVIDIA $driver"
if (( failures > 0 )); then
  echo "Ubuntu gecis dogrulamasi tamamlanmadi: $failures sorun" >&2
  exit 1
fi
echo "Ubuntu 24.04 yerel ortam dogrulamasi: OK"
