#!/usr/bin/env bash
set -euo pipefail

PIDFILE="${RM_MASSAGE_D435I_PIDFILE:-/tmp/rm_massage_external_d435i.pid}"

if [[ ! -f "${PIDFILE}" ]]; then
  echo "No D435i pidfile found: ${PIDFILE}"
  exit 0
fi

PID="$(cat "${PIDFILE}")"
if kill -0 "${PID}" 2>/dev/null; then
  kill "${PID}" 2>/dev/null || true
  wait "${PID}" 2>/dev/null || true
  echo "Stopped external D435i launch: pid=${PID}"
else
  echo "External D435i launch pid is not running: ${PID}"
fi
rm -f "${PIDFILE}"
