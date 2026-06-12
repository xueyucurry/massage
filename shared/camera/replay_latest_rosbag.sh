#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BAG_DIR="$SCRIPT_DIR/rosbags"
ROS_SETUP=""
VIEWER_SCRIPT="$SCRIPT_DIR/rosbag_rgbd_viewer.py"

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

bag_path=""
play_rate="1.0"
loop_flag=0
with_viewer=0
# 预读缓冲，减轻磁盘慢时播放断续（可按机器调整）
read_ahead_size="1000"
topic_prefix=""

usage() {
  cat <<'EOF'
用法:
  ./replay_latest_rosbag.sh
  ./replay_latest_rosbag.sh --bag ./rosbags/realsense_20260402_154500
  ./replay_latest_rosbag.sh --viewer
  ./replay_latest_rosbag.sh --viewer --topic-prefix /replay
  ./replay_latest_rosbag.sh --rate 0.5 --loop

选项:
  --bag PATH     指定要回放的 rosbag 目录
  --rate RATE    回放倍率，默认 1.0
  --loop         循环回放
  --viewer       同时启动 OpenCV RGB/深度可视化
  --topic-prefix 为回放图像话题增加前缀，避免与实时相机话题冲突；
                 配合 --viewer 使用时，默认前缀为 /replay
  --read-ahead N 传给 ros2 bag play 的预读队列（默认 1000，磁盘慢时可加大）
  -h, --help     显示帮助
EOF
}

find_latest_bag() {
  BAG_SEARCH_DIR="$DEFAULT_BAG_DIR" python3 - <<'PY2'
import glob
import os

search_dir = os.environ["BAG_SEARCH_DIR"]
candidates = []
patterns = [
    os.path.join(search_dir, "*"),
    "/home/franka/realsense_bag",
    "/home/franka/realsense_bag_*",
    "/home/franka/*.db3",
    "/home/franka/*.mcap",
]

for pattern in patterns:
    for path in glob.glob(pattern):
        if os.path.isdir(path) and os.path.exists(os.path.join(path, "metadata.yaml")):
            candidates.append(path)

if not candidates:
    raise SystemExit(1)

candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
print(candidates[0])
PY2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      bag_path="${2:-}"
      shift 2
      ;;
    --rate)
      play_rate="${2:-}"
      shift 2
      ;;
    --loop)
      loop_flag=1
      shift
      ;;
    --viewer)
      with_viewer=1
      shift
      ;;
    --topic-prefix)
      topic_prefix="${2:-}"
      shift 2
      ;;
    --read-ahead)
      read_ahead_size="${2:-}"
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

if [[ -z "$bag_path" ]]; then
  if ! bag_path="$(find_latest_bag)"; then
    echo "未找到可回放的 rosbag 目录。"
    exit 1
  fi
fi

if [[ ! -d "$bag_path" ]] || [[ ! -f "$bag_path/metadata.yaml" ]]; then
  echo "无效的 rosbag 目录: $bag_path"
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

if [[ $with_viewer -eq 1 ]] && [[ -z "$topic_prefix" ]]; then
  topic_prefix="/replay"
fi

normalize_topic_prefix() {
  local prefix="${1:-}"
  prefix="${prefix%/}"
  if [[ -n "$prefix" ]] && [[ "$prefix" != /* ]]; then
    prefix="/$prefix"
  fi
  printf '%s\n' "$prefix"
}

topic_prefix="$(normalize_topic_prefix "$topic_prefix")"

viewer_pid=""
cleanup() {
  if [[ -n "$viewer_pid" ]] && kill -0 "$viewer_pid" 2>/dev/null; then
    echo "[Replay] Stopping viewer (PID=$viewer_pid)"
    kill "$viewer_pid" 2>/dev/null || true
    wait "$viewer_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

viewer_color_topic="/camera/camera/color/image_raw"
viewer_depth_topic="/camera/camera/depth/image_rect_raw"
remap_rules=()
if [[ -n "$topic_prefix" ]]; then
  viewer_color_topic="$topic_prefix/camera/color/image_raw"
  viewer_depth_topic="$topic_prefix/camera/depth/image_rect_raw"
  remap_rules+=(
    "/camera/camera/color/image_raw:=$viewer_color_topic"
    "/camera/camera/depth/image_rect_raw:=$viewer_depth_topic"
    "/camera/camera/color/camera_info:=$topic_prefix/camera/color/camera_info"
    "/camera/camera/depth/camera_info:=$topic_prefix/camera/depth/camera_info"
    "/camera/camera/color/metadata:=$topic_prefix/camera/color/metadata"
    "/camera/camera/depth/metadata:=$topic_prefix/camera/depth/metadata"
    "/camera/camera/extrinsics/depth_to_color:=$topic_prefix/camera/extrinsics/depth_to_color"
    "/camera/camera/extrinsics/depth_to_depth:=$topic_prefix/camera/extrinsics/depth_to_depth"
  )
fi

if [[ $with_viewer -eq 1 ]]; then
  export DISPLAY="${DISPLAY:-:0}"
  export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
  if [[ -f "$VIEWER_SCRIPT" ]]; then
    mapfile -t existing_viewers < <(pgrep -f "$VIEWER_SCRIPT" || true)
    if [[ ${#existing_viewers[@]} -gt 0 ]]; then
      echo "[Replay] Stopping existing viewer instances: ${existing_viewers[*]}"
      kill "${existing_viewers[@]}" 2>/dev/null || true
      sleep 0.5
    fi
    python3 "$VIEWER_SCRIPT" --color-topic "$viewer_color_topic" --depth-topic "$viewer_depth_topic" &
    viewer_pid=$!
    echo "[Replay] Viewer started (PID=$viewer_pid)"
    echo "[Replay] Viewer topics: color=$viewer_color_topic depth=$viewer_depth_topic"
  else
    echo "[Replay] Viewer script not found: $VIEWER_SCRIPT"
  fi
fi

play_cmd=(ros2 bag play "$bag_path" --rate "$play_rate" --read-ahead-queue-size "$read_ahead_size")
if [[ $loop_flag -eq 1 ]]; then
   play_cmd+=(--loop)
fi
if [[ ${#remap_rules[@]} -gt 0 ]]; then
  play_cmd+=(--remap "${remap_rules[@]}")
fi

echo "[Replay] Bag: $bag_path"
echo "[Replay] Command: ${play_cmd[*]}"
"${play_cmd[@]}"
exit $?
