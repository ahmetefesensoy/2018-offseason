#!/usr/bin/env bash
set -euo pipefail
sim_ws="${FRC_SIM_WS:-$HOME/frc-portfolio-sim}"
ros_distro="${FRC_ROS_DISTRO:-jazzy}"
if [[ ! -f "$sim_ws/install/setup.bash" ]]; then
  printf 'Run tools/setup_sim.sh first.\n' >&2
  exit 1
fi
set +u
source "/opt/ros/$ros_distro/setup.bash"
source "$sim_ws/.venv/bin/activate"
source "$sim_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-18}"
exec ros2 launch frc_bringup dynamic_autonomy.launch.py \
  use_fake_roborio:=true use_synthetic_perception:=true use_detector:=false "$@"
