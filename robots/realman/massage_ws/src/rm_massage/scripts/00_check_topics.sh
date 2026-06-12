#!/usr/bin/env bash
set -euo pipefail

need_topic() {
  local topic="$1"
  if ros2 topic list | grep -qx "$topic"; then
    printf '[OK] %s\n' "$topic"
  else
    printf '[MISS] %s\n' "$topic"
    return 1
  fi
}

need_any() {
  local label="$1"
  shift
  local topic
  for topic in "$@"; do
    if ros2 topic list | grep -qx "$topic"; then
      printf '[OK] %s -> %s\n' "$label" "$topic"
      return 0
    fi
  done
  printf '[MISS] %s candidates: %s\n' "$label" "$*"
  return 1
}

printf 'RealMan topics:\n'
ros2 topic list | grep rm_driver || true
printf '\nCamera topics:\n'
ros2 topic list | grep camera || true
printf '\nTF topics:\n'
ros2 topic list | grep -E '(^/tf$|^/tf_static$)' || true
printf '\nRequired MVP topics:\n'

missing=0
need_any arm_position /rm_driver/udp_arm_position /rm_json/arm_pose || missing=1
need_any six_force /rm_driver/udp_six_force /rm_json/six_force || missing=1
need_topic /joint_states || missing=1
need_any color_image /camera/color/image_raw /camera/camera/color/image_raw || missing=1
need_any color_info /camera/color/camera_info /camera/camera/color/camera_info || missing=1

if [ "$missing" -ne 0 ]; then
  printf '\nSome required topics are missing. Start the missing hardware driver(s) before continuing.\n'
  exit 1
fi

printf '\nAll required MVP topics are present.\n'
