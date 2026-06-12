#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_OUTPUT_DIR="$SCRIPT_DIR/rosbags"
ROS_SETUP=""
storage_id="mcap"
output_dir="$DEFAULT_OUTPUT_DIR"
bag_name=""
record_all=0
max_duration_sec=""

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

discover_realsense_topics() {
  local topic_list_output
  topic_list_output="$(ros2 topic list 2>/dev/null || true)"
  if [[ -z "$topic_list_output" ]]; then
    return 1
  fi

  local roots=()
  mapfile -t roots < <(
    printf '%s\n' "$topic_list_output" |
      sed -nE 's#^(.+)/(color/image_raw|depth/image_rect_raw|depth/image_raw|aligned_depth_to_color/image_raw)$#\1#p' |
      sort -u
  )

  if [[ ${#roots[@]} -eq 0 ]]; then
    return 1
  fi

  local root
  for root in "${roots[@]}"; do
    printf '%s\n' "$topic_list_output" |
      awk -v prefix="$root/" 'index($0, prefix) == 1'
  done | sort -u
}

usage() {
  cat <<'EOF'
用法:
  ./record_realsense_rosbag.sh
  ./record_realsense_rosbag.sh --all
  ./record_realsense_rosbag.sh --name test_session
  ./record_realsense_rosbag.sh --output-dir ./bags
  ./record_realsense_rosbag.sh --storage sqlite3
  ./record_realsense_rosbag.sh --duration 30

选项:
  --all                录制全部 ROS 话题
  --name NAME          指定输出目录名，不指定则按时间生成
  --output-dir DIR     指定 rosbag 保存目录，默认脚本同级 rosbags/
  --storage ID         rosbag 存储格式，默认 mcap
  --duration SEC       最长录制秒数，到点发 SIGINT 停止并闭合 bag
  -h, --help           显示帮助
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      record_all=1
      shift
      ;;
    --name)
      bag_name="${2:-}"
      shift 2
      ;;
    --output-dir)
      output_dir="${2:-}"
      shift 2
      ;;
    --storage)
      storage_id="${2:-}"
      shift 2
      ;;
    --duration)
      max_duration_sec="${2:-}"
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

mkdir -p "$output_dir"

if [[ -z "$bag_name" ]]; then
  bag_name="realsense_$(date +%Y%m%d_%H%M%S)"
fi

bag_path="$output_dir/$bag_name"
if [[ -e "$bag_path" ]]; then
  echo "输出目录已存在: $bag_path"
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

if [[ $record_all -eq 1 ]]; then
  record_cmd=(ros2 bag record -a --storage "$storage_id" -o "$bag_path")
else
  mapfile -t camera_topics < <(discover_realsense_topics || true)
  if [[ ${#camera_topics[@]} -eq 0 ]]; then
    cat <<'EOF'
未检测到 RealSense 话题。
请先启动相机节点后再录制，或者使用 --all 录制全部话题。
EOF
    exit 1
  fi

  current_topics="$(ros2 topic list 2>/dev/null || true)"
  if printf '%s\n' "$current_topics" | grep -Fxq '/tf'; then
    camera_topics+=("/tf")
  fi
  if printf '%s\n' "$current_topics" | grep -Fxq '/tf_static'; then
    camera_topics+=("/tf_static")
  fi

  mapfile -t camera_topics < <(printf '%s\n' "${camera_topics[@]}" | sort -u)
  record_cmd=(ros2 bag record "${camera_topics[@]}" --storage "$storage_id" -o "$bag_path")
fi

cat <<EOF
[Record] Output: $bag_path
[Record] ROS setup: $ROS_SETUP
[Record] Command: ${record_cmd[*]}
[Record] 按 Ctrl+C 停止录制
[Record] 录制结束后可用以下命令回放:
  $SCRIPT_DIR/replay_latest_rosbag.sh --bag "$bag_path" --viewer
EOF

if [[ $record_all -eq 0 ]]; then
  printf '[Record] Topics (%d):\n' "${#camera_topics[@]}"
  printf '  %s\n' "${camera_topics[@]}"
fi

# 不用 exec：保持本 shell 为前台，Ctrl+C 由 trap 转发给录制进程（直接 exec ros2 时部分终端/版本下 SIGINT 无效）
_shutdown_recorder() {
  local pid="${1:-}"
  [[ -z "$pid" ]] && return 0
  if ! kill -0 "$pid" 2>/dev/null; then
    wait "$pid" 2>/dev/null || true
    return 0
  fi
  echo "[Record] 收到停止信号，正在结束录制 (PID $pid)..."
  kill -INT "$pid" 2>/dev/null || true
  local n=0
  while kill -0 "$pid" 2>/dev/null && [[ $n -lt 120 ]]; do
    sleep 0.25
    n=$((n + 1))
  done
  if kill -0 "$pid" 2>/dev/null; then
    echo "[Record] 仍在运行，发送 SIGTERM..."
    kill -TERM "$pid" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "$pid" 2>/dev/null; then
    echo "[Record] 强制 SIGKILL..."
    kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

rec_pid=""
if [[ -n "$max_duration_sec" ]]; then
  timeout -s INT -k 5 "${max_duration_sec}" "${record_cmd[@]}" &
  rec_pid=$!
else
  "${record_cmd[@]}" &
  rec_pid=$!
fi

trap '_shutdown_recorder "$rec_pid"' INT TERM
wait "$rec_pid" || true
wait_rc=$?
trap - INT TERM
# timeout 124 = 超时结束，视为成功落盘
if [[ -n "$max_duration_sec" ]] && [[ "$wait_rc" -eq 124 ]]; then
  exit 0
fi
exit "$wait_rc"
