#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"

export LASTTIME_ROS2_SCRIPT="ft_agent_process.py"
export HOVER_HEIGHT_MM="${HOVER_HEIGHT_MM:-50.0}"
# The voice/GUI back workflow starts from the recorded high-elbow config-6
# posture.  Keep the lower-level ft.py default opt-in so standalone/manual
# runs remain conservative, but enable the validated posture for this runner.
export BACK_POSTURE_SEED_ENABLE="${BACK_POSTURE_SEED_ENABLE:-1}"

exec "${SCRIPT_DIR}/run_lasttime_ros2.sh" "$@"
