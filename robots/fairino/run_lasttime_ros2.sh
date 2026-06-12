#!/bin/bash

set -eo pipefail

export ROBOT_IP="${ROBOT_IP:-192.168.58.2}"
export ROS_LOCALHOST_ONLY="${LASTTIME_ROS_LOCALHOST_ONLY:-1}"
export LASTTIME_ROS2_SCRIPT="${LASTTIME_ROS2_SCRIPT:-lasttime_ros2.py}"
if [[ -z "${HOVER_HEIGHT_MM+x}" ]]; then
  if [[ "$(basename "${LASTTIME_ROS2_SCRIPT}")" == "ft.py" ]]; then
    export HOVER_HEIGHT_MM="50.0"
  else
    export HOVER_HEIGHT_MM="25.0"
  fi
else
  export HOVER_HEIGHT_MM
fi
export DIAN_JIN_DEPTH_MM="${DIAN_JIN_DEPTH_MM:-20.0}"
export FEN_JIN_LATERAL_MM="${FEN_JIN_LATERAL_MM:-20.0}"
export ROS2_RESET_ERRORS="${ROS2_RESET_ERRORS:-1}"
export ROS2_STATE_WAIT_S="${ROS2_STATE_WAIT_S:-12.0}"
export LASTTIME_ROS2_PROBE_ONLY="${LASTTIME_ROS2_PROBE_ONLY:-0}"
export LASTTIME_ROS2_REUSE_SERVER="${LASTTIME_ROS2_REUSE_SERVER:-1}"
export LASTTIME_FORCE_SENSOR_BUS="${LASTTIME_FORCE_SENSOR_BUS:-1}"
export LASTTIME_FORCE_GUARD="${LASTTIME_FORCE_GUARD:-0}"
export LASTTIME_FORCE_ALLOW_SKIP_ZERO="${LASTTIME_FORCE_ALLOW_SKIP_ZERO:-1}"
export LASTTIME_FORCE_ZERO_MAX_ABS_N="${LASTTIME_FORCE_ZERO_MAX_ABS_N:-3.0}"
export LASTTIME_FORCE_AXIS_SIGN="${LASTTIME_FORCE_AXIS_SIGN:--1.0}"
export LASTTIME_FORCE_MAX_DIS_MM="${LASTTIME_FORCE_MAX_DIS_MM:-8.0}"
export LASTTIME_FORCE_PID_P="${LASTTIME_FORCE_PID_P:-0.003}"
export LASTTIME_FORCE_FEN_LATERAL_MM="${LASTTIME_FORCE_FEN_LATERAL_MM:-8.0}"
export LASTTIME_FORCE_TOL_N="${LASTTIME_FORCE_TOL_N:-2.0}"
export LASTTIME_FORCE_APPROACH_CONTACT_STEP_MM="${LASTTIME_FORCE_APPROACH_CONTACT_STEP_MM:-0.6}"
export LASTTIME_FORCE_APPROACH_FINE_STEP_MM="${LASTTIME_FORCE_APPROACH_FINE_STEP_MM:-1.0}"
export LASTTIME_FORCE_APPROACH_NEAR_STEP_MM="${LASTTIME_FORCE_APPROACH_NEAR_STEP_MM:-0.5}"
export LASTTIME_FORCE_APPROACH_CONTACT_VEL="${LASTTIME_FORCE_APPROACH_CONTACT_VEL:-8.0}"
export LASTTIME_FORCE_APPROACH_NEAR_VEL="${LASTTIME_FORCE_APPROACH_NEAR_VEL:-8.0}"
export LASTTIME_FORCE_APPROACH_PRECONTACT_CLEARANCE_MM="${LASTTIME_FORCE_APPROACH_PRECONTACT_CLEARANCE_MM:-20.0}"
export LASTTIME_FORCE_APPROACH_PRECONTACT_VEL="${LASTTIME_FORCE_APPROACH_PRECONTACT_VEL:-12.0}"
export LASTTIME_FORCE_HOLD_KP_MM_PER_N="${LASTTIME_FORCE_HOLD_KP_MM_PER_N:-0.02}"
export LASTTIME_FORCE_HOLD_MAX_STEP_MM="${LASTTIME_FORCE_HOLD_MAX_STEP_MM:-0.08}"
export LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION="${LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION:-1}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/fairino_ros2/frcobot_ros2-master"
ROS2_LOG="${SCRIPT_DIR}/.ros2_cmd_server.log"
RUNTIME_LIB_DIR="${SCRIPT_DIR}/.ros2_runtime_libs"
SERVICE_REGEX='/?fairino_remote_command_service'
SERVER_PID=""
SERVER_STARTED=0

service_ready() {
  if command -v rg >/dev/null 2>&1; then
    ros2 service list 2>/dev/null | rg -qx "${SERVICE_REGEX}"
  else
    ros2 service list 2>/dev/null | grep -Eq "^${SERVICE_REGEX}$"
  fi
}

server_process_exists() {
  pgrep -f '(^|/)ros2_cmd_server([[:space:]]|$)|ros2 run fairino_hardware ros2_cmd_server' >/dev/null 2>&1
}

reset_ros2_discovery() {
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
}

stop_existing_servers() {
  pkill -f '(^|/)ros2_cmd_server([[:space:]]|$)|ros2 run fairino_hardware ros2_cmd_server' >/dev/null 2>&1 || true
  sleep 1
  reset_ros2_discovery
}

start_server() {
  echo "启动 FAIRINO ROS2 控制服务..."
  : > "${ROS2_LOG}"
  ros2 run fairino_hardware ros2_cmd_server --ros-args -p controller_ip:="${ROBOT_IP}" \
    >"${ROS2_LOG}" 2>&1 &
  SERVER_PID=$!
  SERVER_STARTED=1

  for _ in $(seq 1 30); do
    if ! kill -0 "${SERVER_PID}" >/dev/null 2>&1; then
      echo "ROS2 控制服务启动失败，日志如下："
      tail -n 80 "${ROS2_LOG}" || true
      return 1
    fi
    if service_ready; then
      echo "ROS2 控制服务已就绪"
      return 0
    fi
    sleep 1
  done

  echo "等待 ROS2 控制服务超时，日志如下："
  tail -n 80 "${ROS2_LOG}" || true
  return 1
}

run_probe() {
  local probe_timeout_s="${LASTTIME_ROS2_PROBE_TIMEOUT_S:-35s}"
  FAIRINO_REMOTE_SERVICE="${FAIRINO_REMOTE_SERVICE:-fairino_remote_command_service}" \
  FAIRINO_STATE_TOPIC="${FAIRINO_STATE_TOPIC:-nonrt_state_data}" \
  timeout --kill-after=2s "${probe_timeout_s}" "${PYTHON_BIN:-python3}" - <<'PY'
import os
import sys

import rclpy
from fairino_msgs.msg import RobotNonrtState
from fairino_msgs.srv import RemoteCmdInterface

service_name = os.environ.get("FAIRINO_REMOTE_SERVICE", "fairino_remote_command_service")
state_topic = os.environ.get("FAIRINO_STATE_TOPIC", "nonrt_state_data")

def _exit(code):
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)

rclpy.init(args=None)
node = rclpy.create_node("fairino_ros2_probe")
client = node.create_client(RemoteCmdInterface, service_name)
state = {"msg": None}

def _on_state(msg):
    state["msg"] = msg

sub = node.create_subscription(RobotNonrtState, state_topic, _on_state, 10)

try:
    service_wait_s = float(os.environ.get("LASTTIME_ROS2_PROBE_SERVICE_WAIT_S", "8.0"))
    state_wait_s = float(os.environ.get("LASTTIME_ROS2_PROBE_STATE_WAIT_S", "8.0"))
    call_wait_s = float(os.environ.get("LASTTIME_ROS2_PROBE_CALL_WAIT_S", "8.0"))

    if not client.wait_for_service(timeout_sec=service_wait_s):
        print("PROBE_ERR: service not ready", file=sys.stderr)
        _exit(1)

    deadline = node.get_clock().now().nanoseconds + int(state_wait_s * 1e9)
    while state["msg"] is None and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["msg"] is None:
        print("PROBE_ERR: no state message", file=sys.stderr)
        _exit(2)

    req = RemoteCmdInterface.Request()
    req.cmd_str = "SetSpeed(30)"
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=call_wait_s)
    if not future.done() or future.exception() is not None or future.result() is None:
        print(f"PROBE_ERR: service call failed: {future.exception()}", file=sys.stderr)
        _exit(3)

    cmd_res = future.result().cmd_res
    print(f"ROS2 控制探针返回: {cmd_res}")
    pose = state["msg"]
    print(
        "ROS2 状态探针位姿: "
        f"{pose.cart_x_cur_pos:.3f}, {pose.cart_y_cur_pos:.3f}, {pose.cart_z_cur_pos:.3f}, "
        f"{pose.cart_a_cur_pos:.3f}, {pose.cart_b_cur_pos:.3f}, {pose.cart_c_cur_pos:.3f}"
    )
    if str(cmd_res).strip() != "0":
        print(f"PROBE_ERR: unexpected cmd result: {cmd_res}", file=sys.stderr)
        _exit(4)
    _exit(0)
finally:
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
PY
}

run_probe_with_retries() {
  local attempts="${LASTTIME_ROS2_PROBE_RETRIES:-8}"
  local delay_s="${LASTTIME_ROS2_PROBE_RETRY_DELAY_S:-2}"
  local attempt

  for attempt in $(seq 1 "${attempts}"); do
    if run_probe; then
      return 0
    fi
    if [[ "${attempt}" -lt "${attempts}" ]]; then
      echo "ROS2 控制探针未就绪，${delay_s}s 后重试 (${attempt}/${attempts})"
      sleep "${delay_s}"
    fi
  done

  return 1
}

cleanup() {
  if [[ "${SERVER_STARTED}" -eq 1 && -n "${SERVER_PID}" ]]; then
    kill "${SERVER_PID}" >/dev/null 2>&1 || true
    wait "${SERVER_PID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT INT TERM

cd "${SCRIPT_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
reset_ros2_discovery
mkdir -p "${RUNTIME_LIB_DIR}"
FAIRINO_RUNTIME_LIB="${FAIRINO_RUNTIME_LIB:-${ROS2_WS}/install/fairino_hardware/lib/libfairino.so.2.2.3}"
if [[ ! -f "${FAIRINO_RUNTIME_LIB}" ]]; then
  FAIRINO_RUNTIME_LIB="${ROS2_WS}/install/fairino_hardware/lib/libfairino.so.2"
fi
if [[ ! -f "${FAIRINO_RUNTIME_LIB}" ]]; then
  echo "未找到 FAIRINO 运行库 libfairino.so" >&2
  exit 1
fi
echo "FAIRINO 运行库: ${FAIRINO_RUNTIME_LIB}"
ln -sf "${FAIRINO_RUNTIME_LIB}" "${RUNTIME_LIB_DIR}/libfairino.so.2"
ln -sf "${FAIRINO_RUNTIME_LIB}" "${RUNTIME_LIB_DIR}/libfairino.so"
export LD_LIBRARY_PATH="${RUNTIME_LIB_DIR}:${ROS2_WS}/install/fairino_hardware/lib:${ROS2_WS}/install/fairino_msgs/lib:${LD_LIBRARY_PATH:-}"
if [[ -x ".venv/bin/python" ]]; then
  PYTHON_BIN=".venv/bin/python"
elif [[ -x "/home/franka/massage/env/.venv/bin/python" ]]; then
  PYTHON_BIN="/home/franka/massage/env/.venv/bin/python"
else
  echo "未找到 Python 虚拟环境解释器" >&2
  exit 1
fi
export PYTHON_BIN
set -u

if server_process_exists && service_ready; then
  echo "检测到 FAIRINO ROS2 控制服务已在运行"
elif server_process_exists; then
  echo "检测到未就绪的 FAIRINO ROS2 控制服务，准备重启"
  stop_existing_servers
  start_server || exit 1
elif service_ready; then
  echo "检测到残留 ROS2 服务发现信息，准备重新拉起控制服务"
  reset_ros2_discovery
  start_server || exit 1
else
  start_server || exit 1
fi

if [[ "${SERVER_STARTED}" -eq 0 ]]; then
  if ! run_probe; then
    echo "现有 ROS2 控制服务未通过探针，尝试重启"
    stop_existing_servers
    start_server || exit 1
    if ! run_probe_with_retries; then
      echo "ROS2 控制服务探针失败，日志如下："
      tail -n 80 "${ROS2_LOG}" || true
      exit 1
    fi
  fi
elif ! run_probe_with_retries; then
  echo "ROS2 控制服务探针失败，日志如下："
  tail -n 80 "${ROS2_LOG}" || true
  exit 1
fi

if [[ "${LASTTIME_ROS2_PROBE_ONLY}" == "1" ]]; then
  echo "LASTTIME_ROS2_PROBE_ONLY=1，跳过视觉与按摩流程"
  exit 0
fi

if [[ ! -f "${SCRIPT_DIR}/${LASTTIME_ROS2_SCRIPT}" ]]; then
  echo "找不到 ROS2 演示脚本: ${SCRIPT_DIR}/${LASTTIME_ROS2_SCRIPT}" >&2
  exit 1
fi

"${PYTHON_BIN}" "${LASTTIME_ROS2_SCRIPT}" "$@"
