#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"

export LASTTIME_ROS2_SCRIPT="ft_agent_process.py"
export HOVER_HEIGHT_MM="${HOVER_HEIGHT_MM:-50.0}"

exec "${SCRIPT_DIR}/run_lasttime_ros2.sh" "$@"
