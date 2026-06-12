#!/usr/bin/env bash
set -euo pipefail

cd /home/franka/massage/robots/realman

exec .venv/bin/python view_bladder_detection_docker_ros.py \
  --container "${RM_ROS_CONTAINER:-noetic}" \
  --master-uri "${ROS_MASTER_URI:-http://192.168.1.11:11311}" \
  --ros-ip "${ROS_IP:-192.168.1.250}" \
  --http-host 127.0.0.1 \
  --http-port 8766 \
  --topic /camera/color/image_raw \
  --topic-type raw \
  --model-path /home/franka/massage/robots/realman/yolo11l-pose.pt \
  --rotation-mode cw90 \
  --detect-hz 1.5
