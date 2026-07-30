#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repo="${UNITREE_MUJOCO_ROOT:-$HOME/ros2_ws/src/unitree_mujoco}"
venv="${G1_MUJOCO_VENV:-$HOME/unitree_venv/g1}"
model="$repo/unitree_robots/g1/scene_obstacles_23dof.xml"
if [[ ! -f "$model" ]]; then
  "$venv/bin/python" "$project/sim_mujoco/generate_g1_23dof_obstacles.py" \
    --unitree-mujoco-root "$repo"
fi
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
set -u
source "$venv/bin/activate"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
exec python "$project/sim_mujoco/g1_zed_live_mimic.py" \
  --model "$model" \
  --mode physics \
  --lower-body grounded \
  --command-tau 0.018 \
  --speed-scale 2.0 \
  --arm-kp-scale 2.2 \
  --ik-iterations 16 \
  --ik-damping 0.025 \
  --ik-regularization 0.008 \
  --segment-memory 0.18 \
  --ros-sensors \
  --ros-state-rate 20 \
  --ros-sensor-rate 5 \
  "$@"
