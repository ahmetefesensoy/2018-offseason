#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sim_ws="${FRC_SIM_WS:-$HOME/frc-portfolio-sim}"
set +u
source "/opt/ros/${FRC_ROS_DISTRO:-jazzy}/setup.bash"
source "$sim_ws/.venv/bin/activate"
source "$sim_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-18}"
exec python "$repo_dir/tools/capture_sim.py" "$@"
