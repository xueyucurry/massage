#!/usr/bin/env bash
# 本机编辑代码 / 板卡运行 / 零污染地二次开发 rm_bladder。
#
# 行为：
#   1. rsync 本机 ~/massage/rm_demo/ 到板卡 ${BOARD_DEV_DIR}/rm_demo/（默认 /home/rm/dev/rm_bladder）
#      * 不同步 *.pt / rm_demo_output / __pycache__ 等大文件或中间产物
#      * 板卡原厂目录 /home/rm/massage/ 和 /home/rm/rm_healthcare_robot/ 全程只读引用
#   2. ssh 进板卡，source ROS1 + 原厂 ws，导出 RM_DEMO_* 环境变量指向原厂资源，
#      再以 package 方式运行 rm_demo.rm_bladder_demo
#   3. 拉回板卡 ${BOARD_DEV_DIR}/rm_demo_output/ 到本机 ~/massage/rm_demo_output/ 方便在本机看 overlay/plan.json
#
# 用法：
#   ./rm_demo/run_bladder_on_board.sh [-- rm_bladder_demo 的 CLI 参数]
#
# 常用参数组合：
#   # 先出检测图 + plan，不运动
#   ./rm_demo/run_bladder_on_board.sh \
#       --transform-backend auto \
#       --capture-positioning prepare \
#       --control-backend json \
#       --side left --line-type outer \
#       --plan-points 4 \
#       --hover-mm 25 --dian-jin-depth-mm 8 --fen-jin-lateral-mm 15 --safe-lift-mm 50 \
#       --speed 2
#
#   # 侧卧拍背：先用示教得到的 6 个关节角只做检测/出图，不运动
#   ./rm_demo/run_bladder_on_board.sh \
#       --capture-positioning prepare \
#       --capture-joints 0 35 -95 20 -70 10 \
#       --transform-backend product_ros \
#       --side left --line-type outer
#
#   # 确认 overlay+plan 无误后，追加 --run 真走
#   ./rm_demo/run_bladder_on_board.sh ... --run
#
# 回滚 / 卸载板卡端（本机执行一次即可）：
#   ssh rm@192.168.1.11 "rm -rf /home/rm/dev/rm_bladder"

set -euo pipefail

BOARD_SSH="${BOARD_SSH:-rm@192.168.1.11}"
BOARD_DEV_DIR="${BOARD_DEV_DIR:-/home/rm/dev/rm_bladder}"
BOARD_ASSETS_DIR="${BOARD_ASSETS_DIR:-/home/rm/massage}"
BOARD_HEALTHCARE_WS="${BOARD_HEALTHCARE_WS:-/home/rm/rm_healthcare_robot/rm_healthcare_robot_server}"
ARM_HOST="${ARM_HOST:-192.168.58.2}"
PULL_OUTPUTS="${PULL_OUTPUTS:-1}"
SYNC_VENDOR="${SYNC_VENDOR:-1}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOCAL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "[rm_bladder] local_root = ${LOCAL_ROOT}"
echo "[rm_bladder] board      = ${BOARD_SSH}:${BOARD_DEV_DIR}"
echo "[rm_bladder] arm_host   = ${ARM_HOST}"

echo "[rm_bladder] 1/3 ensure remote dir"
ssh "${BOARD_SSH}" "mkdir -p '${BOARD_DEV_DIR}/rm_demo_output'"

echo "[rm_bladder] 1/3 rsync rm_demo/"
rsync -az --delete \
    --exclude '__pycache__' --exclude '*.pyc' --exclude '*.pyo' \
    --exclude 'rm_demo_output' --exclude 'rm_demo_debug' \
    --exclude '.venv*' --exclude '*.pt' --exclude 'rosbags' \
    "${LOCAL_ROOT}/rm_demo/" "${BOARD_SSH}:${BOARD_DEV_DIR}/rm_demo/"

if [[ "${SYNC_VENDOR}" == "1" && -d "${LOCAL_ROOT}/ros_vendor" ]]; then
    echo "[rm_bladder] 1/3 rsync ros_vendor/"
    rsync -az --delete \
        --exclude '__pycache__' --exclude '*.pyc' --exclude '*.pyo' \
        "${LOCAL_ROOT}/ros_vendor/" "${BOARD_SSH}:${BOARD_DEV_DIR}/ros_vendor/"
fi

DEMO_ARGS=("$@")
QUOTED_ARGS=""
for arg in "${DEMO_ARGS[@]}"; do
    printf -v piece "%q " "${arg}"
    QUOTED_ARGS+="${piece}"
done

echo "[rm_bladder] 2/3 run on board"
# 环境变量让板卡用原厂的模型 / 标定 / 轨迹 yaml / 机械臂 IP，且所有中间产物只落到 ${BOARD_DEV_DIR}
set +e
ssh -t "${BOARD_SSH}" "bash -lc '
    set -e
    if [ -f /opt/ros/noetic/setup.bash ]; then
        source /opt/ros/noetic/setup.bash
    fi
    if [ -f ${BOARD_HEALTHCARE_WS}/install/setup.bash ]; then
        source ${BOARD_HEALTHCARE_WS}/install/setup.bash
    fi
    export RM_DEMO_MODEL_PATH=${BOARD_ASSETS_DIR}/yolo11l-pose.pt
    export RM_DEMO_MATRIX_PATH=${BOARD_ASSETS_DIR}/camera_to_robot.json
    export RM_DEMO_TRAJECTORY_CONFIG=${BOARD_HEALTHCARE_WS}/install/share/rm_healthcare_robot_server_launcher/config/trajectory_generate.yaml
    export RM_ARM_HOST=${ARM_HOST}
    export PYTHONDONTWRITEBYTECODE=1
    cd ${BOARD_DEV_DIR}
    python3 -m rm_demo.rm_bladder_demo ${QUOTED_ARGS}
'"
REMOTE_RC=$?
set -e

if [[ "${PULL_OUTPUTS}" == "1" ]]; then
    echo "[rm_bladder] 3/3 pull outputs"
    mkdir -p "${LOCAL_ROOT}/rm_demo_output"
    rsync -az "${BOARD_SSH}:${BOARD_DEV_DIR}/rm_demo_output/" "${LOCAL_ROOT}/rm_demo_output/" || true
else
    echo "[rm_bladder] 3/3 skip pull outputs (PULL_OUTPUTS=0)"
fi

exit "${REMOTE_RC}"
