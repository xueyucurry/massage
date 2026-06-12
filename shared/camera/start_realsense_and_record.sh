#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/rosbags"
ROS_SETUP=""
bag_name=""
record_all=0
storage_id="mcap"
wait_seconds=30
record_duration_sec=""
launch_args=()

find_ros_setup() {
  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    printf '%s\n' "/opt/ros/$ROS_DISTRO/setup.bash"
    return 0
  fi

  local candidate
  for candidate in /opt/ros/*/setup.bash; do
    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

camera_topics_ready() {
  ros2 topic list 2>/dev/null |
    grep -Eq '/(color/image_raw|depth/image_rect_raw|depth/image_raw|aligned_depth_to_color/image_raw)$'
}

usage() {
  cat <<'EOF'
用法:
  ./start_realsense_and_record.sh
  ./start_realsense_and_record.sh --name test_session
  ./start_realsense_and_record.sh --all
  ./start_realsense_and_record.sh --launch-arg align_depth.enable:=true

说明:
  1. 启动 realsense2_camera 节点
  2. 等待图像话题出现
  3. 自动开始录制 rosbag

选项:
  --name NAME          指定录包目录名
  --output-dir DIR     rosbag 保存目录，默认脚本同级 rosbags/
  --all                录制全部话题
  --storage ID         rosbag 存储格式，默认 mcap
  --wait-seconds N     等待相机话题出现的秒数，默认 30
  --launch-arg ARG     透传给 rs_launch.py，可重复传入
  --duration SEC       录制 SEC 秒后自动停止（传给 record_realsense_rosbag.sh）
  -h, --help           显示帮助
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --name)
      bag_name="${2:-}"
      shift 2
      ;;
    --all)
      record_all=1
      shift
      ;;
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    --storage)
      storage_id="${2:-}"
      shift 2
      ;;
    --wait-seconds)
      wait_seconds="${2:-}"
      shift 2
      ;;
    --launch-arg)
      launch_args+=("${2:-}")
      shift 2
      ;;
    --duration)
      record_duration_sec="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1"
      usage
      exit 1
      ;;
  esac
done

ROS_SETUP="$(find_ros_setup || true)"
if [[ -z "$ROS_SETUP" ]] || [[ ! -f "$ROS_SETUP" ]]; then
  echo "未找到 ROS 2 环境，请确认 /opt/ros 下已安装发行版。"
  exit 1
fi

mkdir -p "$OUTPUT_DIR"

if [[ -z "$bag_name" ]]; then
  bag_name="realsense_$(date +%Y%m%d_%H%M%S)"
fi

set +u
source "$ROS_SETUP"
set -u

if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
  echo "未找到 realsense2_camera 包，请先安装 Intel RealSense ROS 2 驱动。"
  exit 1
fi

camera_log="$OUTPUT_DIR/${bag_name}.realsense.log"
launch_cmd=(ros2 launch realsense2_camera rs_launch.py "${launch_args[@]}")

echo "[Start] Launching realsense2_camera..."
echo "[Start] Camera log: $camera_log"
"${launch_cmd[@]}" >"$camera_log" 2>&1 &
camera_pid=$!

cleanup() {
  if kill -0 "$camera_pid" 2>/dev/null; then
    echo "[Start] Stopping realsense2_camera (PID=$camera_pid)"
    kill "$camera_pid" 2>/dev/null || true
    wait "$camera_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[Start] Waiting for camera topics..."
for _ in $(seq 1 "$wait_seconds"); do
  if ! kill -0 "$camera_pid" 2>/dev/null; then
    echo "[Start] realsense2_camera 提前退出，请检查日志: $camera_log"
    tail -n 40 "$camera_log" 2>/dev/null || true
    exit 1
  fi
  if camera_topics_ready; then
    echo "[Start] RealSense topics are ready."
    break
  fi
  sleep 1
done

if ! camera_topics_ready; then
  echo "[Start] RealSense topics did not appear in time."
  exit 1
fi

record_cmd=("$SCRIPT_DIR/record_realsense_rosbag.sh" --output-dir "$OUTPUT_DIR" --name "$bag_name" --storage "$storage_id")
if [[ $record_all -eq 1 ]]; then
  record_cmd+=(--all)
fi
if [[ -n "$record_duration_sec" ]]; then
  record_cmd+=(--duration "$record_duration_sec")
fi

echo "[Start] Recording with: ${record_cmd[*]}"
"${record_cmd[@]}"
