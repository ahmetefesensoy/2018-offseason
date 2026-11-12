#!/usr/bin/env bash
# Target-side service entry point; configure team, hardware and camera TF first.
set -euo pipefail
autonomy_ws="${FRC_AUTONOMY_WS:-/opt/frc-autonomy/ws}"
set +u
source "/opt/ros/${FRC_ROS_DISTRO:-jazzy}/setup.bash"
source "$autonomy_ws/.venv/bin/activate"
source "$autonomy_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-18}"
exec ros2 launch frc_bringup dynamic_autonomy.launch.py \
  use_fake_roborio:=false use_synthetic_perception:=false \
  use_detector:=true use_rviz:=false
