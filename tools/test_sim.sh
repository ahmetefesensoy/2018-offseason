#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sim_ws="${FRC_SIM_WS:-$HOME/frc-portfolio-sim}"
set +u
source "/opt/ros/${FRC_ROS_DISTRO:-jazzy}/setup.bash"
source "$sim_ws/.venv/bin/activate"
source "$sim_ws/install/setup.bash"
set -u
python -m pytest "$repo_dir/jetson/ros2_ws/src" -q --import-mode=importlib
python "$repo_dir/tools/check_ros_startup.py"
ros2 run frc_bringup offline_autonomy_demo
