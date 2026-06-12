#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/franka/massage/robots/fairino"
SCRIPT_PATH="$PROJECT_DIR/fairino_compliance_control.py"
DEFAULT_CONFIG_FILE="$PROJECT_DIR/fairino_compliance.env"
CONFIG_FILE="${FAIRINO_COMPLIANCE_ENV:-$DEFAULT_CONFIG_FILE}"
CONFIG_FROM_CLI="0"
PRINT_CONFIG="0"
DRY_RUN="0"
PASSTHROUGH_ARGS=()

# 默认参数。优先级：
# 1. 调用脚本时显式传入的环境变量
# 2. 配置文件
# 3. 这里的内置默认值
DEFAULT_ROBOT_IP="192.168.58.2"
DEFAULT_TOOL_ID="0"
DEFAULT_USER_ID="0"
DEFAULT_POSE_A=""
DEFAULT_POSE_B=""
DEFAULT_CYCLES="3"
DEFAULT_MOVE_VEL="20"
DEFAULT_MOVE_OVL="100"
DEFAULT_MOVE_BLEND_R="-1"
DEFAULT_MOVE_DWELL="0"
DEFAULT_MODE_AUTO="1"
DEFAULT_ROBOT_ENABLE="1"

DEFAULT_SENSOR_COMPANY="24"
DEFAULT_SENSOR_DEVICE="0"
DEFAULT_SENSOR_SOFTVERSION="0"
DEFAULT_SENSOR_BUS="1"
DEFAULT_SENSOR_ID="1"
DEFAULT_SKIP_ZERO="0"
DEFAULT_SLEEP_AFTER_SENSOR_CMD="1.0"
DEFAULT_PAYLOAD_WEIGHT=""
DEFAULT_PAYLOAD_COG=""

DEFAULT_COMPLIANCE_P="0.00005"
DEFAULT_COMPLIANCE_FORCE="30"

DEFAULT_SELECT_DOF="1,1,1,0,0,0"
DEFAULT_TARGET_FT="-10,-10,-10,0,0,0"
DEFAULT_FT_PID="0.0005,0,0,0,0,0"
DEFAULT_MAX_DIS="100"
DEFAULT_MAX_ANG="0"
DEFAULT_MB_M="0,0"
DEFAULT_MB_B="0,0"
DEFAULT_THRESHOLD="0.2,0.2"
DEFAULT_ADJUST_COEFF="1,1"
DEFAULT_FILTER_SIGN="0"
DEFAULT_POS_ADAPT_SIGN="0"
DEFAULT_IS_NO_BLOCK="0"

usage() {
  cat <<'USAGE_EOF'
用法:
  ./run_fairino_compliance.sh
  ./run_fairino_compliance.sh --config /path/to/site_a.env
  ./run_fairino_compliance.sh --cycles 5 --vel 10
  ROBOT_IP=192.168.58.9 ./run_fairino_compliance.sh

示例:
  1. 先编辑:
     /home/franka/massage/robots/fairino/fairino_compliance.env

  2. 然后直接运行:
     ./run_fairino_compliance.sh

  3. 临时覆盖某些参数:
     ./run_fairino_compliance.sh --cycles 2 --vel 10

  4. 用环境变量临时覆盖:
     COMPLIANCE_FORCE=15 ./run_fairino_compliance.sh

  5. 使用另一套现场配置:
     ./run_fairino_compliance.sh --config /home/franka/massage/site_b.env

  6. 只查看当前生效参数:
     ./run_fairino_compliance.sh --print-config

  7. 只打印最终执行命令，不真正运动:
     ./run_fairino_compliance.sh --dry-run

说明:
  - 启动器默认读取: /home/franka/massage/robots/fairino/fairino_compliance.env
  - 也可以通过 --config 或 FAIRINO_COMPLIANCE_ENV 指向别的配置文件
  - --print-config 只打印当前生效参数并退出
  - --dry-run 会打印参数和最终命令，但不会连接机器人
  - 额外命令行参数会追加到最后，可覆盖配置文件中的同名参数
  - 该脚本封装的是官方文档里的 FT_Control + FT_ComplianceStart/Stop 流程
  - 校零前请确保末端未接触任何外物
  - 运动前请先确认 pose-a / pose-b 对当前现场、工具坐标、工件坐标是安全的
USAGE_EOF
}

choose_python() {
  if [[ -x "$PROJECT_DIR/.venv/bin/python" ]]; then
    echo "$PROJECT_DIR/.venv/bin/python"
    return 0
  fi
  if [[ -x "/home/franka/anaconda3/envs/llamauav/bin/python" ]]; then
    echo "/home/franka/anaconda3/envs/llamauav/bin/python"
    return 0
  fi
  if command -v python3 >/dev/null 2>&1; then
    command -v python3
    return 0
  fi
  return 1
}

is_truthy() {
  case "${1:-0}" in
    1|true|TRUE|yes|YES|on|ON|y|Y)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

append_if_nonempty() {
  local -n _cmd_ref="$1"
  local flag="$2"
  local value="$3"
  if [[ -n "$value" ]]; then
    _cmd_ref+=("$flag" "$value")
  fi
}

print_effective_config() {
  cat <<CONFIG_EOF
[Config]
CONFIG_FILE="$CONFIG_FILE"
ROBOT_IP="$ROBOT_IP"
TOOL_ID="$TOOL_ID"
USER_ID="$USER_ID"
MODE_AUTO="$MODE_AUTO"
ROBOT_ENABLE="$ROBOT_ENABLE"
POSE_A="$POSE_A"
POSE_B="$POSE_B"
CYCLES="$CYCLES"
MOVE_VEL="$MOVE_VEL"
MOVE_OVL="$MOVE_OVL"
MOVE_BLEND_R="$MOVE_BLEND_R"
MOVE_DWELL="$MOVE_DWELL"
SENSOR_COMPANY="$SENSOR_COMPANY"
SENSOR_DEVICE="$SENSOR_DEVICE"
SENSOR_SOFTVERSION="$SENSOR_SOFTVERSION"
SENSOR_BUS="$SENSOR_BUS"
SENSOR_ID="$SENSOR_ID"
SKIP_ZERO="$SKIP_ZERO"
SLEEP_AFTER_SENSOR_CMD="$SLEEP_AFTER_SENSOR_CMD"
PAYLOAD_WEIGHT="$PAYLOAD_WEIGHT"
PAYLOAD_COG="$PAYLOAD_COG"
COMPLIANCE_P="$COMPLIANCE_P"
COMPLIANCE_FORCE="$COMPLIANCE_FORCE"
SELECT_DOF="$SELECT_DOF"
TARGET_FT="$TARGET_FT"
FT_PID="$FT_PID"
MAX_DIS="$MAX_DIS"
MAX_ANG="$MAX_ANG"
MB_M="$MB_M"
MB_B="$MB_B"
THRESHOLD="$THRESHOLD"
ADJUST_COEFF="$ADJUST_COEFF"
FILTER_SIGN="$FILTER_SIGN"
POS_ADAPT_SIGN="$POS_ADAPT_SIGN"
IS_NO_BLOCK="$IS_NO_BLOCK"
CONFIG_EOF
}

if [[ ! -f "$SCRIPT_PATH" ]]; then
  echo "未找到脚本: $SCRIPT_PATH" >&2
  exit 1
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --config)
      if [[ $# -lt 2 ]]; then
        echo "--config 需要一个文件路径" >&2
        exit 1
      fi
      CONFIG_FILE="$2"
      CONFIG_FROM_CLI="1"
      shift 2
      ;;
    --print-config)
      PRINT_CONFIG="1"
      shift
      ;;
    --dry-run)
      DRY_RUN="1"
      shift
      ;;
    *)
      PASSTHROUGH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "$CONFIG_FROM_CLI" == "1" && ! -f "$CONFIG_FILE" ]]; then
  echo "指定的配置文件不存在: $CONFIG_FILE" >&2
  exit 1
fi

if [[ -n "${FAIRINO_COMPLIANCE_ENV:-}" && ! -f "$CONFIG_FILE" ]]; then
  echo "FAIRINO_COMPLIANCE_ENV 指向的配置文件不存在: $CONFIG_FILE" >&2
  exit 1
fi

if [[ -f "$CONFIG_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi

ROBOT_IP="${ROBOT_IP:-$DEFAULT_ROBOT_IP}"
TOOL_ID="${TOOL_ID:-$DEFAULT_TOOL_ID}"
USER_ID="${USER_ID:-$DEFAULT_USER_ID}"
POSE_A="${POSE_A:-$DEFAULT_POSE_A}"
POSE_B="${POSE_B:-$DEFAULT_POSE_B}"
CYCLES="${CYCLES:-$DEFAULT_CYCLES}"
MOVE_VEL="${MOVE_VEL:-$DEFAULT_MOVE_VEL}"
MOVE_OVL="${MOVE_OVL:-$DEFAULT_MOVE_OVL}"
MOVE_BLEND_R="${MOVE_BLEND_R:-$DEFAULT_MOVE_BLEND_R}"
MOVE_DWELL="${MOVE_DWELL:-$DEFAULT_MOVE_DWELL}"
MODE_AUTO="${MODE_AUTO:-$DEFAULT_MODE_AUTO}"
ROBOT_ENABLE="${ROBOT_ENABLE:-$DEFAULT_ROBOT_ENABLE}"

SENSOR_COMPANY="${SENSOR_COMPANY:-$DEFAULT_SENSOR_COMPANY}"
SENSOR_DEVICE="${SENSOR_DEVICE:-$DEFAULT_SENSOR_DEVICE}"
SENSOR_SOFTVERSION="${SENSOR_SOFTVERSION:-$DEFAULT_SENSOR_SOFTVERSION}"
SENSOR_BUS="${SENSOR_BUS:-$DEFAULT_SENSOR_BUS}"
SENSOR_ID="${SENSOR_ID:-$DEFAULT_SENSOR_ID}"
SKIP_ZERO="${SKIP_ZERO:-$DEFAULT_SKIP_ZERO}"
SLEEP_AFTER_SENSOR_CMD="${SLEEP_AFTER_SENSOR_CMD:-$DEFAULT_SLEEP_AFTER_SENSOR_CMD}"
PAYLOAD_WEIGHT="${PAYLOAD_WEIGHT:-$DEFAULT_PAYLOAD_WEIGHT}"
PAYLOAD_COG="${PAYLOAD_COG:-$DEFAULT_PAYLOAD_COG}"

COMPLIANCE_P="${COMPLIANCE_P:-$DEFAULT_COMPLIANCE_P}"
COMPLIANCE_FORCE="${COMPLIANCE_FORCE:-$DEFAULT_COMPLIANCE_FORCE}"

SELECT_DOF="${SELECT_DOF:-$DEFAULT_SELECT_DOF}"
TARGET_FT="${TARGET_FT:-$DEFAULT_TARGET_FT}"
FT_PID="${FT_PID:-$DEFAULT_FT_PID}"
MAX_DIS="${MAX_DIS:-$DEFAULT_MAX_DIS}"
MAX_ANG="${MAX_ANG:-$DEFAULT_MAX_ANG}"
MB_M="${MB_M:-$DEFAULT_MB_M}"
MB_B="${MB_B:-$DEFAULT_MB_B}"
THRESHOLD="${THRESHOLD:-$DEFAULT_THRESHOLD}"
ADJUST_COEFF="${ADJUST_COEFF:-$DEFAULT_ADJUST_COEFF}"
FILTER_SIGN="${FILTER_SIGN:-$DEFAULT_FILTER_SIGN}"
POS_ADAPT_SIGN="${POS_ADAPT_SIGN:-$DEFAULT_POS_ADAPT_SIGN}"
IS_NO_BLOCK="${IS_NO_BLOCK:-$DEFAULT_IS_NO_BLOCK}"

if [[ "$PRINT_CONFIG" == "1" ]]; then
  print_effective_config
  exit 0
fi

if ! PYTHON_BIN="$(choose_python)"; then
  echo "未找到可用的 Python 解释器" >&2
  exit 1
fi

cd "$PROJECT_DIR"

cmd=(
  "$PYTHON_BIN"
  "$SCRIPT_PATH"
  --ip "$ROBOT_IP"
  --tool "$TOOL_ID"
  --user "$USER_ID"
  --pose-a "$POSE_A"
  --pose-b "$POSE_B"
  --cycles "$CYCLES"
  --vel "$MOVE_VEL"
  --ovl "$MOVE_OVL"
  --blend-r "$MOVE_BLEND_R"
  --dwell "$MOVE_DWELL"
  --sensor-company "$SENSOR_COMPANY"
  --sensor-device "$SENSOR_DEVICE"
  --sensor-softversion "$SENSOR_SOFTVERSION"
  --sensor-bus "$SENSOR_BUS"
  --sensor-id "$SENSOR_ID"
  --sleep-after-sensor-cmd "$SLEEP_AFTER_SENSOR_CMD"
  --compliance-p "$COMPLIANCE_P"
  --compliance-force "$COMPLIANCE_FORCE"
  --select "$SELECT_DOF"
  --target-ft "$TARGET_FT"
  --ft-pid "$FT_PID"
  --max-dis "$MAX_DIS"
  --max-ang "$MAX_ANG"
  --mb-m "$MB_M"
  --mb-b "$MB_B"
  --threshold "$THRESHOLD"
  --adjust-coeff "$ADJUST_COEFF"
  --filter-sign "$FILTER_SIGN"
  --pos-adapt-sign "$POS_ADAPT_SIGN"
  --is-no-block "$IS_NO_BLOCK"
)

append_if_nonempty cmd --payload-weight "$PAYLOAD_WEIGHT"
append_if_nonempty cmd --payload-cog "$PAYLOAD_COG"

if is_truthy "$MODE_AUTO"; then
  cmd+=(--mode-auto)
fi

if is_truthy "$ROBOT_ENABLE"; then
  cmd+=(--enable)
fi

if is_truthy "$SKIP_ZERO"; then
  cmd+=(--skip-zero)
fi

if [[ "${#PASSTHROUGH_ARGS[@]}" -gt 0 ]]; then
  cmd+=("${PASSTHROUGH_ARGS[@]}")
fi

if [[ "$DRY_RUN" == "1" ]]; then
  print_effective_config
  if [[ -z "$POSE_A" || -z "$POSE_B" ]]; then
    echo "[DryRun] 警告: POSE_A / POSE_B 仍为空，真实执行会失败。" >&2
  fi
  printf '[DryRun] '
  printf '%q ' "${cmd[@]}"
  printf '\n'
  exit 0
fi

if [[ -z "$POSE_A" || -z "$POSE_B" ]]; then
  echo "POSE_A / POSE_B 未配置。" >&2
  echo "请先编辑: $CONFIG_FILE" >&2
  echo "或在运行前通过环境变量设置。" >&2
  exit 1
fi

exec "${cmd[@]}"
