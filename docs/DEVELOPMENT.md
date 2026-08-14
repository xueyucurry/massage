# 开发指南

## 开发环境

- 原生 Ubuntu 22.04（推荐）；也兼容 Windows 11 + WSL2 Ubuntu 22.04
- ROS2 Humble
- 项目目录由实际克隆位置决定，以下用 `/path/to/massage` 表示
- 项目 Python 环境 `/path/to/massage/env/.venv`
- 小智 Python 环境由根启动器中的 `XIAOZHI_PYTHON` 确定
- FAIRINO 控制器默认地址 `192.168.58.2`

先运行以下命令了解本机可用入口：

```bash
cd /path/to/massage
./massage help
```

首次在一台机器上运行时，需要在本机生成 ROS2 构建产物，不要直接复用其他机器或
WSL 中的 `build/`、`install/`、`log/`：

```bash
cd /path/to/massage/robots/fairino/fairino_ros2/frcobot_ros2-master
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
```

`PYTHONNOUSERSITE=1` 可避免用户目录中的新版 Python 工具包覆盖 Ubuntu/ROS2
自带依赖。

原生 Ubuntu 不需要任何 `wsl` 或 `usbipd` 命令。仅在 WSL2 环境中，`wsl -d
Ubuntu-22.04`、`wsl --terminate Ubuntu-22.04` 和 `usbipd` 才应从 Windows
PowerShell 执行。

## 修改顺序

1. 从非归档分支创建功能分支。
2. 先确认修改属于视觉、力控、智能体接口还是设备配置。
3. 优先修改对应模块，不跨层复制功能。
4. 静态验证通过后，再进行无接触测试。
5. 最后在急停可用、低速和专人监护条件下进行接触测试。

业务扩展位置：

| 需求 | 修改位置 |
| --- | --- |
| 新增 `./massage` 命令 | 根目录 `massage` |
| 新增语音工具 | `py-xiaozhi/src/mcp/tools/fairino_massage/` |
| 修改按摩流程 | `robots/fairino/ft.py` |
| 修改恒力控制 | `robots/fairino/force_control.py`，并同步安全边界 |
| 修改背部检测 | `demo.py`、`lasttime.py` 或 `shared/vision/yolo.py` |
| 修改大腿检测 | `thigh_outerline_confirm.py`、`rtmpose_detector.py` |
| 更新厂商 SDK | `robots/fairino/vendor/fairino_sdk/`，记录版本 |

## 最小验证

不连接机械臂也应执行：

```bash
python3 scripts/check_project_structure.py
scripts/check_runtime_imports.sh
bash -n massage
python3 -m py_compile \
  py-xiaozhi/src/application.py \
  py-xiaozhi/src/mcp/mcp_server.py \
  robots/fairino/fairino/__init__.py
git diff --check
./massage help
```

验证项目内 SDK 加载路径：

```bash
env/.venv/bin/python -c "\
import sys; \
sys.path.insert(0, 'robots/fairino'); \
import fairino; \
print(fairino.SDK_MODULE)"
```

输出必须位于当前仓库的 `robots/fairino/vendor/fairino_sdk/Robot.py`，不应指向
其他用户主目录或项目副本。

涉及视觉时先做只读检测；涉及力控时依次做传感器只读、悬停、低速低力接触和
完整动作。不要把 GUI 能启动视为机械臂控制已经验证。

## 小智专用模式

`./massage gui` 默认导出：

```bash
XIAOZHI_PUSH_TO_TALK_ONLY=1
XIAOZHI_MASSAGE_ONLY=1
```

按摩专用模式只注册系统工具和 FAIRINO 按摩工具，并停用日程提醒插件。开发通用
小智功能时可临时设置 `XIAOZHI_MASSAGE_ONLY=0`，但机械臂交互验收应恢复为 `1`。

## 配置管理

- 记录初始位姿前清空机械臂周围区域，并复核单位与姿态。
- 重新标定后检查 `camera_to_robot.json` 的更新时间和内容差异。
- 力度、速度、接触阈值和最大位移属于安全参数，必须单独提交并说明测试条件。
- 不提交日志、PID、任务状态、检测截图或 ROS2 编译目录。

## 安全底线

- 软件停止不能代替实体急停。
- 真人实验前必须完成假人、低力和限速测试。
- 传感器未清零、读数异常、ROS2 状态中断或轨迹超界时禁止继续接触运动。
- 同一时刻只能有一个进程控制机械臂；使用根启动器管理 ROS2 服务和任务进程。
- 任何扩大力、速度、位移或放宽保护阈值的修改，都需要重新完成接触测试。
