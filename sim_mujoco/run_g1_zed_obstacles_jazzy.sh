#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${G1_NATIVE_ROOT:-$HOME/g1_isaaclab_project}"
repo="${UNITREE_MUJOCO_ROOT:-$root/repos/unitree_mujoco}"
venv="${G1_MUJOCO_VENV:-$root/envs/mujoco}"
model="$repo/unitree_robots/g1/scene_obstacles_23dof.xml"

if [[ ! -x "$venv/bin/python" ]]; then
  echo "MuJoCo ortami bulunamadi: $venv" >&2
  exit 1
fi
if [[ ! -f "$model" ]]; then
  "$venv/bin/python" "$project/sim_mujoco/generate_g1_23dof_obstacles.py" \
    --unitree-mujoco-root "$repo"
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

exec "$venv/bin/python" "$project/sim_mujoco/g1_zed_live_mimic.py" \
  --model "$model" --mode physics --lower-body grounded \
  --command-tau 0.018 --speed-scale 2.0 --arm-kp-scale 2.2 \
  --ik-iterations 16 --ik-damping 0.025 --ik-regularization 0.008 \
  --segment-memory 0.18 --ros-sensors --ros-state-rate 20 --ros-sensor-rate 5 \
  "$@"
