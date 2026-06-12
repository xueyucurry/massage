#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:-}"
LOG="${RM_MASSAGE_D435I_LOG:-/tmp/rm_massage_external_d435i.log}"
PIDFILE="${RM_MASSAGE_D435I_PIDFILE:-/tmp/rm_massage_external_d435i.pid}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
source "${WS_DIR}/install/setup.bash"

if [[ -z "${SERIAL}" ]]; then
  if command -v rs-enumerate-devices >/dev/null 2>&1; then
    SERIAL="$(rs-enumerate-devices -s 2>/dev/null | awk '/Intel RealSense/ {print $(NF-1); exit}')"
  fi
fi

if [[ -z "${SERIAL}" ]]; then
  echo "No RealSense serial detected. Pass it explicitly, e.g. $0 338622072685" >&2
  exit 1
fi

if [[ -f "${PIDFILE}" ]] && kill -0 "$(cat "${PIDFILE}")" 2>/dev/null; then
  echo "External D435i launch already running: pid=$(cat "${PIDFILE}")"
else
  setsid bash -lc "
    echo \\\$\\\$ > '${PIDFILE}'
    source '${WS_DIR}/install/setup.bash'
    exec ros2 launch realsense2_camera rs_launch.py \
      serial_no:=_${SERIAL} \
      align_depth.enable:=true \
      pointcloud.enable:=true
  " >"${LOG}" 2>&1 < /dev/null &
  for _ in $(seq 1 20); do
    [[ -s "${PIDFILE}" ]] && break
    sleep 0.05
  done
  echo "Started external D435i: pid=$(cat "${PIDFILE}" 2>/dev/null || true) serial=${SERIAL} log=${LOG}"
fi

for _ in $(seq 1 25); do
  if ros2 topic list 2>/dev/null | grep -Eq '^/camera/camera/color/image_raw$|^/camera/color/image_raw$'; then
    echo "External D435i topics ready:"
    ros2 topic list | grep camera | sort
    exit 0
  fi
  sleep 1
done

echo "External D435i topics did not appear. Log tail:" >&2
tail -n 100 "${LOG}" >&2 || true
exit 1
