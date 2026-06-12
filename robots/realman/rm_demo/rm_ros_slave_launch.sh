#!/usr/bin/env bash
set -euo pipefail

MASTER_HOST="${1:-}"
BOARD_SSH="${2:-rm@192.168.1.11}"
BOARD_IP="${3:-192.168.1.11}"
LAUNCH_PKG="${4:-rm_healthcare_robot_server_launcher}"
LAUNCH_FILE="${5:-rm_healthcare_robot_server_launcher.launch}"

if [[ -z "${MASTER_HOST}" ]]; then
  echo "Usage: $0 <master-host-ip> [board-ssh] [board-ip] [launch-pkg] [launch-file]" >&2
  exit 2
fi

ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${BOARD_SSH}" \
  "bash -lc '
    set -e
    ROS_SETUP=\$(ls -1d /opt/ros/*/setup.bash 2>/dev/null | tail -n1 || true)
    if [[ -n \"\${ROS_SETUP}\" ]]; then
      source \"\${ROS_SETUP}\"
    fi
    source /home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/setup.bash
    export ROS_MASTER_URI=http://${MASTER_HOST}:11311
    export ROS_IP=${BOARD_IP}
    export ROS_HOSTNAME=${BOARD_IP}
    echo ROS_MASTER_URI=\${ROS_MASTER_URI}
    echo ROS_IP=\${ROS_IP}
    exec roslaunch ${LAUNCH_PKG} ${LAUNCH_FILE}
  '"
