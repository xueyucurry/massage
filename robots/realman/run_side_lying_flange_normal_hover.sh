#!/usr/bin/env bash
set -euo pipefail

cd /home/franka/massage/robots/realman
source .venv/bin/activate

python3 - <<'PY'
from rm_demo.rm_json import query_json

host = "192.168.1.18"
print(query_json(host, {"command": "set_change_tool_frame", "tool_name": "Arm_Tip"}))
print(query_json(host, {"command": "get_current_tool_frame"}))
PY

exec ./run_purple_20_point_press.py \
  --run \
  --run-mode hover \
  --hard-max-z-m 0.38 \
  --allow-high-start-descend \
  --max-z-above-start-m 0.005 \
  --entry-speed 6 \
  --entry-max-step-m 0.01 \
  --entry-max-angle-step-rad 0.06 \
  --entry-orientation-policy position_then_orient \
  --entry-position-motion movej_p \
  --entry-orient-motion movej_p \
  --hover-mm 50 \
  --retreat-mm 50 \
  --order first_to_last \
  --orientation-mode normal \
  --normal-tool-axis pos_z \
  --normal-direction outward \
  --tip-offset-mode base \
  --tcp-to-tip-base-x-mm 0.0 \
  --tcp-to-tip-base-y-mm 0.0 \
  --tcp-to-tip-base-z-mm 0.0 \
  --skip-tool-switch \
  "$@"
