# FAIRINO 按摩模块

这个目录同时包含当前按摩链、诊断工具和早期实验。判断文件是否可清理时，
以本页的分类为准，不要仅按文件名判断。

## 当前核心链

| 文件/目录 | 作用 |
| --- | --- |
| `ft.py` | 点筋、分筋、顺筋、视觉轨迹和ROS2恒力控制主程序 |
| `ft_agent_api.py` | 将按摩能力封装为智能体可调用API |
| `ft_agent_process.py` | 独立按摩任务进程与状态/控制文件循环 |
| `force_control.py` | 力控和SDK辅助实现，`ft.py`直接依赖 |
| `lasttime.py`、`demo.py` | 背部YOLO检测和轨迹生成基础，仍被`ft.py`依赖 |
| `dianjing.py` | 相机到机器人坐标转换辅助，仍被`ft.py`依赖 |
| `thigh_outerline_confirm.py` | RTMPose大腿轨迹生成，仍被`ft.py`依赖 |
| `rtmpose_detector.py` | RTMPose模型、配置和推理实现 |
| `rtmpose.py` | RTMPose独立命令行入口 |
| `fairino_ros2/frcobot_ros2-master/` | FAIRINO ROS2源码工作区 |

虽然`lasttime.py`、`demo.py`和`dianjing.py`名字看起来像旧实验，它们目前仍在
主程序导入链中，不能直接删除。

## 当前诊断工具

| 文件 | 入口/用途 |
| --- | --- |
| `force_monitor_gui.py` | `./massage force-monitor`，只读六维力监视器 |
| `adjust_back_line_offset.py` | `./massage adjust-lines`，背部四线偏移调整 |
| `thigh_offset_adjust.py` | 大腿轨迹偏移调试 |
| `test_force_sensor.py` | 六维力传感器独立测试 |
| `run_shunjin_only.py` | 只运行顺筋动作的调试脚本 |

## 设备配置

- `massage_home_pose.json`：启动、停止和GUI启动前的返回位姿。
- `back_line_offset.json`：背部检测线人工校正量。
- `fairino_compliance.site_template.env`：柔顺控制环境变量模板。
- `shared/calibration/`：实际手眼标定结果，位于仓库共享目录。

## 历史实验

以下文件不在`./massage`主运行链中，保留用于追溯和对照：

- `lasttime_ros2.py`、`lastime_ros2.py`及`run_lasttime*.sh`
- `fairino_spring_assist.py`及`run_fairino_spring.sh`
- `fairino_compliance_control.py`及`run_fairino_compliance.sh`
- `fairino_ros2_air_move_demo.py`
- `hover.py`、`dianjing.py`的独立实验入口和相关快照文档

后续若要进一步缩减仓库，应整体迁移到`archive/`，不要零散删除依赖文件。

## 生成产物

这些内容只属于当前机器，不应提交：

- `.venv/`、`.ros2_runtime_libs/`
- `ft_agent_state/`
- `*_output/`
- `fairino_ros2/**/build/`、`install/`、`log/`
- `*.log`、`*.pid`
