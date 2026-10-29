#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/fairino_ros2/frcobot_ros2-master"
PY_SCRIPT="${SCRIPT_DIR}/fairino_ros2_air_move_demo.py"

if [[ ! -f "${PY_SCRIPT}" ]]; then
  echo "未找到脚本: ${PY_SCRIPT}" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
  echo "未找到 ROS2 环境: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${ROS2_WS}/install/setup.bash" ]]; then
  echo "未找到工作空间环境: ${ROS2_WS}/install/setup.bash" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
set -u

exec python3 "${PY_SCRIPT}" "$@"
