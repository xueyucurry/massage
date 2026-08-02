#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${SCRIPT_DIR}"

PYTHON_BIN="${PYTHON_BIN:-${PROJECT_ROOT}/env/.venv/bin/python}"
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "未找到 FAIRINO Python 环境: ${PYTHON_BIN}" >&2
  exit 1
fi

exec "${PYTHON_BIN}" thigh_offset_adjust.py "$@"
