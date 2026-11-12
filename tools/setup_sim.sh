#!/usr/bin/env bash
# Build the hardware-free ROS 2 simulator without changing system Python.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sim_ws="${FRC_SIM_WS:-$HOME/frc-portfolio-sim}"
ros_distro="${FRC_ROS_DISTRO:-jazzy}"
if [[ ! -f "/opt/ros/$ros_distro/setup.bash" ]]; then
  printf 'ROS 2 %s is missing. Install ROS and RViz first.\n' "$ros_distro" >&2
  exit 1
fi
mkdir -p -- "$sim_ws"
if [[ -e "$sim_ws/src" && ! -L "$sim_ws/src" ]]; then
  printf 'Refusing to replace the existing source directory: %s/src\n' "$sim_ws" >&2
  exit 1
fi
python3 -m venv --system-site-packages "$sim_ws/.venv"
"$sim_ws/.venv/bin/python" -m pip install -r "$repo_dir/jetson/requirements.txt"
ln -sfn -- "$repo_dir/jetson/ros2_ws/src" "$sim_ws/src"
# ROS setup scripts may reference unset variables.
set +u
source "/opt/ros/$ros_distro/setup.bash"
source "$sim_ws/.venv/bin/activate"
set -u
cd -- "$sim_ws"
python -m colcon build --symlink-install
printf '\nBuilt simulator in %s. Run tools/run_sim.sh from this repository.\n' "$sim_ws"
