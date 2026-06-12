#!/usr/bin/env bash
set -euo pipefail

ENV_PY="/home/franka/anaconda3/envs/llamauav/bin/python"
PROJECT_DIR="/home/franka/massage/robots/fairino"

if [ ! -x "$ENV_PY" ]; then
  echo "未找到 Python 环境: $ENV_PY"
  exit 1
fi

cd "$PROJECT_DIR"

if [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:0
fi

exec "$ENV_PY" demo.py "$@"
