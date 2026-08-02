#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS2_WS="${SCRIPT_DIR}/fairino_ros2/frcobot_ros2-master"

cd "${SCRIPT_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
set -u

PYTHON_BIN="${PYTHON_BIN:-${PROJECT_ROOT}/env/.venv/bin/python}"
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "未找到 FAIRINO Python 环境: ${PYTHON_BIN}" >&2
  exit 1
fi

exec "${PYTHON_BIN}" ft_agent_process.py "$@"
