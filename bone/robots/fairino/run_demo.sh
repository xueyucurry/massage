#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ENV_PY="${ROOT_DIR}/env/.venv/bin/python"
PROJECT_DIR="$SCRIPT_DIR"

if [ ! -x "$ENV_PY" ]; then
  echo "未找到 Python 环境: $ENV_PY"
  exit 1
fi

cd "$PROJECT_DIR"

if [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:0
fi

exec "$ENV_PY" demo.py "$@"
