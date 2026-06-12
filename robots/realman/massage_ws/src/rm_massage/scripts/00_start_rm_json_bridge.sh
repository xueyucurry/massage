#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-192.168.1.18}"
LOG="${RM_MASSAGE_JSON_BRIDGE_LOG:-/tmp/rm_massage_json_bridge.log}"
PIDFILE="${RM_MASSAGE_JSON_BRIDGE_PIDFILE:-/tmp/rm_massage_json_bridge.pid}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
source "${WS_DIR}/install/setup.bash"

if [[ -f "${PIDFILE}" ]] && kill -0 "$(cat "${PIDFILE}")" 2>/dev/null; then
  echo "RealMan JSON bridge already running: pid=$(cat "${PIDFILE}")"
else
  setsid bash -lc "
    echo \\\$\\\$ > '${PIDFILE}'
    source '${WS_DIR}/install/setup.bash'
    exec ros2 run rm_massage json_state_bridge --host ${HOST}
  " >"${LOG}" 2>&1 < /dev/null &
  for _ in $(seq 1 20); do
    [[ -s "${PIDFILE}" ]] && break
    sleep 0.05
  done
  echo "Started RealMan JSON bridge: pid=$(cat "${PIDFILE}" 2>/dev/null || true) host=${HOST} log=${LOG}"
fi

for _ in $(seq 1 15); do
  if ros2 topic list 2>/dev/null | grep -qx /rm_json/six_force && ros2 topic list 2>/dev/null | grep -qx /joint_states; then
    echo "RealMan JSON bridge topics ready:"
    ros2 topic list | grep -E '^/rm_json|^/joint_states$|^/tf$' | sort
    exit 0
  fi
  sleep 1
done

echo "RealMan JSON bridge topics did not appear. Log tail:" >&2
tail -n 100 "${LOG}" >&2 || true
exit 1
