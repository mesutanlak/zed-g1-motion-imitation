#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
official_urdf="${G1_URDF:-$HOME/g1_isaaclab_project/repos/unitree_ros/robots/g1_description/g1_23dof.urdf}"
runtime_dir="/tmp/zed_g1_rviz_${USER}"
runtime_urdf="$runtime_dir/g1_23dof_absolute.urdf"
headless="${G1_RVIZ_HEADLESS:-0}"
mkdir -p "$runtime_dir"

if [[ ! -f "$official_urdf" ]]; then
  echo "Unitree G1 URDF bulunamadi: $official_urdf" >&2
  exit 1
fi
python3 "$project/ros2_bridge/prepare_g1_urdf.py" \
  --source "$official_urdf" --output "$runtime_urdf" >/dev/null

pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 "$project/ros2_bridge/body38_udp_ros2.py" \
  --listen-host 0.0.0.0 --listen-port 15054 --frame-id zed_camera &
pids+=("$!")
python3 "$project/ros2_bridge/g1_retarget_udp_ros2.py" \
  --listen-host 0.0.0.0 --listen-port 15053 --frame-id odom --base-frame pelvis &
pids+=("$!")
python3 "$project/ros2_bridge/g1_sensor_frames_ros2.py" &
pids+=("$!")
ros2 run robot_state_publisher robot_state_publisher "$runtime_urdf" \
  --ros-args -r joint_states:=/g1/display/joint_states &
pids+=("$!")

echo "G1 tam RViz Jazzy sistemi hazir (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "BODY_38 UDP=15054, GMR/Isaac telemetry UDP=15053"
if [[ "$headless" == "1" ]]; then
  wait
else
  rviz2 -d "$project/rviz/g1_full_system.rviz" &
  pids+=("$!")
  wait "${pids[-1]}"
fi
