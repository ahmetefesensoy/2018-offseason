#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
topic_file="${script_dir}/../ros2_ws/src/frc_bringup/config/record_topics.txt"
output="${1:-autonomy_$(date -u +%Y%m%dT%H%M%SZ)}"
mapfile -t topics < <(grep -Ev '^[[:space:]]*(#|$)' "${topic_file}")
ros2 bag record -o "${output}" "${topics[@]}"
