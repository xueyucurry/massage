#!/usr/bin/env bash
set -euo pipefail

PIDFILE="${RM_MASSAGE_JSON_BRIDGE_PIDFILE:-/tmp/rm_massage_json_bridge.pid}"

if [[ ! -f "${PIDFILE}" ]]; then
  echo "No RealMan JSON bridge pidfile found: ${PIDFILE}"
  exit 0
fi

PID="$(cat "${PIDFILE}")"
if kill -0 "${PID}" 2>/dev/null; then
  kill "${PID}" 2>/dev/null || true
  wait "${PID}" 2>/dev/null || true
  echo "Stopped RealMan JSON bridge: pid=${PID}"
else
  echo "RealMan JSON bridge pid is not running: ${PID}"
fi
rm -f "${PIDFILE}"
