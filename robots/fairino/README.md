# FAIRINO 按摩模块

这里是项目当前唯一的机械臂业务模块。为避免在缺少硬件回归测试时引入路径和
导入回归，核心 Python 文件暂时保留在同一目录；文件职责由本页明确，而不是
通过一次大规模移动重新分包。

## 主运行链

| 文件/目录 | 职责 |
| --- | --- |
| `ft.py` | 点筋、分筋、顺筋、视觉轨迹和 ROS2 恒力控制主程序 |
| `ft_agent_api.py` | 将按摩能力封装成智能体 API |
| `ft_agent_process.py` | 独立任务进程及状态、控制文件循环 |
| `run_ft_agent_process_env.sh` | 加载项目与 ROS2 环境 |
| `run_ft_agent_process_ros2.sh` | 通过 ROS2 启动按摩任务进程 |
| `run_lasttime_ros2.sh` | ROS2 服务生命周期和 `ft.py` 启动器 |
| `force_control.py` | 力控、传感器和 SDK 辅助实现 |
| `lasttime.py`、`demo.py` | 背部 YOLO 检测与轨迹生成基础 |
| `dianjing.py` | 相机坐标到机器人坐标的转换辅助 |
| `thigh_outerline_confirm.py` | RTMPose 大腿轨迹生成 |
| `rtmpose_detector.py` | RTMPose 模型、配置与推理实现 |
| `fairino_ros2/frcobot_ros2-master/` | FAIRINO ROS2 源码工作区 |
| `vendor/fairino_sdk/` | 项目固定使用的厂商 Python SDK |

`lasttime.py`、`demo.py` 和 `dianjing.py` 虽然名称带有历史痕迹，但仍位于
`ft.py` 的实际导入链中，不能删除。

## 工具与配置

| 文件 | 入口或用途 |
| --- | --- |
| `force_monitor_gui.py` | `./massage force-monitor`，只读六维力监视器 |
| `adjust_back_line_offset.py` | `./massage adjust-lines`，背部四线偏移调整 |
| `thigh_offset_adjust.py` | 大腿轨迹偏移调试 |
| `test_force_sensor.py` | 六维力传感器独立只读测试 |
| `hover.py` | 非接触悬停诊断 |
| `run_shunjin_only.py` | 单独执行顺筋动作的调试脚本 |
| `massage_home_pose.json` | 机械臂初始与返回位姿 |
| `back_line_offset.json` | 背部检测线人工校正量 |

手眼标定程序和结果统一放在仓库根目录的 `shared/calibration/`。

## 生成数据

以下内容属于当前机器或单次运行，不是源码：

- `.ros2_runtime_libs/`、`.ros2_cmd_server.*`
- `ft_agent_state/`
- `ft_locked_trajectory_output/` 和其他 `*_output/`
- `fairino_ros2/**/build/`、`install/`、`log/`
- `__pycache__/`、`*.log`、`*.pid`

不要通过清空这些目录来修复业务逻辑。需要清理磁盘时，应先停止 GUI、按摩任务
和 ROS2 服务，再只处理已确认可再生成的文件。

## 模块文档

- [运行与参数说明](docs/FT_USAGE.md)
- [技术结构](docs/FT_TECHNICAL.md)
- [智能体接口](docs/FT_AGENT_API.md)
- [RTMPose 部署](docs/RTMPOSE-DEPLOY.md)
