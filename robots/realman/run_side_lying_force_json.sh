#!/usr/bin/env bash
set -euo pipefail

cd /home/franka/massage/robots/realman
source .venv/bin/activate

exec ./run_purple_20_point_press.py \
  --run \
  --run-mode force \
  --allow-contact \
  --force-backend json \
  --target-force-n 2 \
  --max-force-n 5 \
  --hard-max-z-m 0.38 \
  --allow-high-start-descend \
  --max-z-above-start-m 0.005 \
  --entry-speed 8 \
  --contact-speed 2 \
  --entry-max-step-m 0.02 \
  --hover-mm 30 \
  --retreat-mm 30 \
  --max-press-mm 8 \
  --approach-step-mm 1 \
  --order first_to_last \
  --tip-offset-mode base \
  --tcp-to-tip-base-x-mm 12.0 \
  --tcp-to-tip-base-y-mm -142.3 \
  --tcp-to-tip-base-z-mm 2.8 \
  --skip-tool-switch
