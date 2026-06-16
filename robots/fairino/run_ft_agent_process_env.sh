#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/fairino_ros2/frcobot_ros2-master"

cd "${SCRIPT_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
set -u

if [[ -x ".venv/bin/python" ]]; then
  PYTHON_BIN=".venv/bin/python"
elif [[ -x "/home/franka/massage/env/.venv/bin/python" ]]; then
  PYTHON_BIN="/home/franka/massage/env/.venv/bin/python"
elif [[ -x "/home/franka/anaconda3/envs/llamauav/bin/python" ]]; then
  PYTHON_BIN="/home/franka/anaconda3/envs/llamauav/bin/python"
else
  echo "未找到 FAIRINO Python 环境" >&2
  exit 1
fi

exec "${PYTHON_BIN}" ft_agent_process.py "$@"
