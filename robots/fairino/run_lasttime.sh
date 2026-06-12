#!/bin/bash
# lasttime演示程序启动脚本

export ROBOT_IP="${ROBOT_IP:-192.168.58.2}"
export HOVER_HEIGHT_MM="${HOVER_HEIGHT_MM:-25.0}"
export DIAN_JIN_DEPTH_MM="${DIAN_JIN_DEPTH_MM:-20.0}"
export FEN_JIN_LATERAL_MM="${FEN_JIN_LATERAL_MM:-20.0}"
export LASTTIME_FORCE_CONTROL="${LASTTIME_FORCE_CONTROL:-1}"
export LASTTIME_FORCE_N="${LASTTIME_FORCE_N:-10.0}"
export LASTTIME_FORCE_SOFTWARE_LIMIT_N="${LASTTIME_FORCE_SOFTWARE_LIMIT_N:-20.0}"
export LASTTIME_FORCE_CONTACT_OFFSET_MM="${LASTTIME_FORCE_CONTACT_OFFSET_MM:-0.0}"
export LASTTIME_FORCE_PRESS_LIMIT_MM="${LASTTIME_FORCE_PRESS_LIMIT_MM:-18.0}"
export LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION="${LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION:-1}"

cd "$(dirname "$0")"

if [[ "${LASTTIME_STOP_ROS2_SERVER:-1}" != "0" ]]; then
  pkill -f '(^|/)ros2_cmd_server([[:space:]]|$)|ros2 run fairino_hardware ros2_cmd_server' >/dev/null 2>&1 || true
  sleep 1
fi

if [[ -x ".venv/bin/python" ]]; then
  PYTHON_BIN=".venv/bin/python"
elif [[ -x "/home/franka/massage/env/.venv/bin/python" ]]; then
  PYTHON_BIN="/home/franka/massage/env/.venv/bin/python"
else
  echo "未找到 Python 虚拟环境解释器" >&2
  exit 1
fi

"$PYTHON_BIN" lasttime.py "$@"
