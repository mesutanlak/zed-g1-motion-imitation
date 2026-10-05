#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy bulunamadi. Resmi Jazzy kurulumunu tamamlayin." >&2
  echo "https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-diagnostic-msgs ros-jazzy-visualization-msgs \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-rmw-cyclonedds-cpp

set +u
source /opt/ros/jazzy/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -c 'import rclpy; print("ROS 2 Jazzy rclpy: OK")'
ros2 pkg prefix rviz2
ros2 pkg prefix robot_state_publisher
ros2 pkg prefix rmw_cyclonedds_cpp

echo "ROS 2 Jazzy hazir. Isaac terminalinde bu setup dosyasini kaynaklamayin."
