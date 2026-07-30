#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec python3 "$PROJECT/ros2_bridge/body38_udp_ros2.py" \
    --listen-host 0.0.0.0 \
    --listen-port 15054 \
    --frame-id zed_camera
