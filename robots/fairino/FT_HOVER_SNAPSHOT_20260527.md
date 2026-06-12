# FT / Hover 功能快照 - 2026-05-27

本快照用于恢复和查看当前 `ft.py` / `hover.py` 已实现的按摩功能。

## 功能范围

- `ft.py`: 统一入口，支持背部膀胱经和腿部大腿外侧中线检测，按 `s` 锁定轨迹，按 `g` 执行机械臂动作。
- `hover.py`: 复用 `ft.py` 的检测和轨迹生成逻辑，强制关闭力控，只执行悬空轨迹检查。
- `dianjing.py`: 标定矩阵加载已加固，优先读取当前目录，其次读取 `robots/fairino/camera_to_robot.json` 软链接和 `shared/calibration/camera_to_robot.json`。
- `calibrate_camera_to_robot_aruco.py`: 兼容当前 OpenCV ArUco 接口，并绕过当前 FAIRINO SDK 包装层读取 TCP 位姿的问题，优先使用底层 XMLRPC 读取位姿。
- `run_shunjin_only.py`: 单独执行顺筋动作的测试入口。

## 当前标定

- 标定文件: `/home/franka/massage/shared/calibration/camera_to_robot.json`
- 时间戳: `2026-05-27 18:39:10`
- 点对数量: `17`
- RMSE: `0.008031535343885433 m`，约 `8.03 mm`
- 离群点: `0`

`robots/fairino/camera_to_robot.json` 是软链接，指向上述 shared 标定文件。

## 当前环境

- Python: `3.10.12`
- venv: `/home/franka/massage/env/.venv`
- 依赖快照: `robots/fairino/ft_hover_env_20260527.txt`
- 关键包:
  - `opencv-python==4.10.0.84`
  - `opencv-contrib-python==4.10.0.84`
  - `pyrealsense2==2.57.7.10387`
  - `torch==2.5.1+cu121`
  - `mmpose==1.3.2`
  - `mmengine==0.10.7`
  - `mmdet==3.3.0`
  - `ultralytics==8.4.32`

大型本地依赖目录没有全部纳入本次快照提交，例如完整 FAIRINO SDK 和 ROS2 工作区构建产物。当前机器上它们仍位于:

- `/home/franka/massage/robots/fairino/fairino-python-sdk-master (1)`
- `/home/franka/massage/robots/fairino/fairino_ros2/frcobot_ros2-master`

## 常用运行命令

运行恒力按摩入口:

```bash
cd /home/franka/massage/robots/fairino
LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
```

运行悬空检查入口:

```bash
cd /home/franka/massage/robots/fairino
LASTTIME_ROS2_SCRIPT=hover.py ./run_lasttime_ros2.sh
```

运行顺筋单独测试:

```bash
cd /home/franka/massage/robots/fairino
LASTTIME_ROS2_SCRIPT=run_shunjin_only.py ./run_lasttime_ros2.sh
```

运行标定程序:

```bash
cd /home/franka/massage/shared/calibration
PYTHONUNBUFFERED=1 PYTHONPATH=/home/franka/massage/robots/fairino:/home/franka/massage/shared/vision /home/franka/massage/env/.venv/bin/python calibrate_camera_to_robot_aruco.py
```

## 恢复方式

本次提交完成后会打 tag。恢复时可使用:

```bash
cd /home/franka/massage
git switch -c restore-ft-hover ft-hover-working-20260527
```

或只查看文件:

```bash
cd /home/franka/massage
git show ft-hover-working-20260527:robots/fairino/ft.py
git show ft-hover-working-20260527:robots/fairino/hover.py
```
