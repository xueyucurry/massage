#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FAIRINO_DIR="${ROOT_DIR}/robots/fairino"
ROS2_WS="${FAIRINO_DIR}/fairino_ros2/frcobot_ros2-master"
PYTHON_BIN="${PYTHON_BIN:-${ROOT_DIR}/env/.venv/bin/python}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python environment not found: ${PYTHON_BIN}" >&2
  exit 1
fi
if [[ ! -f "${ROS2_WS}/install/setup.bash" ]]; then
  echo "FAIRINO ROS2 environment not built: ${ROS2_WS}/install/setup.bash" >&2
  echo "Build it with: cd ${ROS2_WS} && source /opt/ros/humble/setup.bash && PYTHONNOUSERSITE=1 colcon build --symlink-install" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
set -u

cd "${FAIRINO_DIR}"
"${PYTHON_BIN}" -c '
import fairino
import ft
import lasttime
import rtmpose_detector

print(f"FAIRINO SDK: {fairino.SDK_MODULE}")
print("Runtime imports passed.")
'
