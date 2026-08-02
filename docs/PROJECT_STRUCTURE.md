# 项目结构

## 系统边界

项目只维护一条生产链路：FAIRINO 机械臂 + RealSense + ROS2 Humble + 小智
智能体。RealMan、CoTracker 实验、旧柔顺控制和旧脚本式按摩接口已从当前分支
移除；需要追溯时使用 `7.27比赛版本` 标签或 `archive/7.27-competition` 分支。

## 运行流程

```text
./massage gui
  -> py-xiaozhi/main.py
  -> src/mcp/tools/fairino_massage/
  -> robots/fairino/run_ft_agent_process_env.sh
  -> robots/fairino/run_ft_agent_process_ros2.sh
  -> robots/fairino/ft_agent_process.py
  -> robots/fairino/ft_agent_api.py
  -> robots/fairino/ft.py
  -> ROS2 fairino_remote_command_service
  -> FAIRINO 控制器和六维力传感器
```

视觉分为两条明确链路：

```text
背部膀胱经: ft.py -> lasttime.py -> demo.py -> shared/vision/yolo.py
大腿内外侧: ft.py -> thigh_outerline_confirm.py -> rtmpose_detector.py
```

背部使用 YOLO 姿态结果构造四条膀胱经检测线。大腿使用 RTMPose 关键点构造
内外侧中线。当前运行链没有使用 CoTracker。

## 目录职责

### `robots/fairino/`

机械臂业务核心。`ft.py` 负责按摩流程，`force_control.py` 负责恒力控制辅助，
`ft_agent_*` 负责把长时间运行的硬件任务与小智进程隔离。完整文件表见该目录的
`README.md`。

### `py-xiaozhi/`

小智客户端及 GUI。按摩业务只应放在
`src/mcp/tools/fairino_massage/`，不要重新接入已移除的脚本式 `massage` 工具。
根启动器默认设置 `XIAOZHI_MASSAGE_ONLY=1`，减少模型工具选择歧义。

### `shared/`

跨模块资源：

- `shared/calibration/`：RealSense 到机械臂的手眼标定。
- `shared/vision/`：YOLO 实现与权重。
- `shared/camera/`：相机诊断工具。

共享目录不应包含某一机械臂平台专用的运动控制代码。

### `vendor/` 与第三方源码

`robots/fairino/vendor/fairino_sdk/` 是运行时实际加载的 FAIRINO Python SDK。
`robots/fairino/fairino_ros2/` 是厂商 ROS2 源码。修改这些文件前应确认是在升级
厂商依赖，而不是修复业务逻辑。

### 配置、状态与产物

| 类型 | 示例 | 是否提交 |
| --- | --- | --- |
| 设备配置 | `massage_home_pose.json`、`back_line_offset.json` | 是，复核后提交 |
| 标定结果 | `shared/calibration/camera_to_robot.json` | 是，记录设备与日期 |
| 模型资源 | YOLO、RTMPose 权重 | 当前随项目管理 |
| 任务状态 | `ft_agent_state/` | 否 |
| 检测输出 | `ft_locked_trajectory_output/` | 否 |
| 编译产物 | ROS2 `build/`、`install/`、`log/` | 否 |
| 本机环境 | `env/`、`.venv*` | 否 |

## 为什么暂不移动核心 Python 文件

当前核心文件存在直接导入、脚本工作目录和 ROS2 环境路径耦合。没有硬件自动化
回归时直接迁移到新包结构，会同时影响视觉、力控和智能体三条链路。本次整理先
删除无关实现、固定依赖并建立清晰文档；后续只有在具备离线轨迹测试和硬件冒烟
测试后，才将核心代码逐步迁入正式 Python 包。
