# bone - FAIRINO 按摩机器人语音控制运行快照

这个目录只保留 FAIRINO 按摩机器人 + 小智语音控制链路需要的代码、配置和运行数据，不包含 RealMan/ROS1 Noetic 旧链路，也不包含 Linux 虚拟环境。

## 包含内容

- `massage`：顶层启动脚本，默认从当前 `bone/` 根目录定位 FAIRINO、小智和标定文件。
- `py-xiaozhi/`：小智 GUI、MCP 工具、唤醒词配置、设备配置和本地唤醒词模型。
- `robots/fairino/`：`ft.py`、语音 agent 包装层、FAIRINO ROS2 工作区、回位姿态、轨迹、状态和模型权重。
- `shared/calibration/`：当前相机到机械臂标定结果。
- `shared/vision/`：YOLO 姿态模型和检测代码。
- `shared/third_party/`：背部膀胱经检测使用的 CoTracker 代码。

## 未包含内容

- Python 虚拟环境：`.venv`、`env/.venv`、`py-xiaozhi/.venv-linux`。
- ROS2 构建缓存：`build/`、`log/`。
- RealMan/ROS1 Noetic 相关代码和 Docker 容器。

这些环境应在目标机器上重新安装，避免把当前 Linux 二进制环境直接搬到不兼容的系统。

## 推荐目标系统

实机控制建议使用：

```bash
Ubuntu 22.04 x86_64
ROS2 Humble
Python 3.10
FAIRINO 控制器 IP: 192.168.58.2
```

Apple Silicon Mac 原生环境不适合直接运行当前 FAIRINO 实机控制链路，因为项目内 FAIRINO 动态库是 Linux x86_64 版本。

## 重建 FAIRINO 运行环境

安装 ROS2 Humble、MoveIt2、RealSense、Python 依赖后，在 `bone/` 内重建 FAIRINO 工作区：

```bash
cd /path/to/bone/robots/fairino/fairino_ros2/frcobot_ros2-master
rm -rf build log
colcon build --symlink-install
```

如果只是复用本目录已经带的 `install/`，目标机器路径和 ABI 必须兼容。更稳妥的方式是重新 `colcon build`。

重建 FAIRINO/视觉 Python 环境：

```bash
cd /path/to/bone
python3.10 -m venv env/.venv
source env/.venv/bin/activate
python -m pip install -U pip
python -m pip install -r requirements-fairino.txt
```

`rclpy`、`fairino_msgs`、`moveit_msgs` 来自 ROS2 环境，不应只靠 pip 安装。
背部检测使用的 CoTracker 源码在 `shared/third_party/co-tracker/`，当前代码会通过本目录内的相对链接引用。

## 重建小智环境

```bash
cd /path/to/bone/py-xiaozhi
python3.10 -m venv .venv-linux
source .venv-linux/bin/activate
python -m pip install -U pip
python -m pip install -r requirements.txt
```

`py-xiaozhi/config/` 内含设备身份和 token。不要把这个目录上传到公开仓库。

## 启动

先确认机械臂、RealSense、力传感器和 ROS2 环境可用，然后：

```bash
cd /path/to/bone
./massage gui
```

常用命令：

```bash
./massage calibrate
./massage record-home
./massage home
./massage start
./massage stop
./massage clean
```

## 快照说明

`bone/massage` 已设置：

```bash
FT_TRAJECTORY_OUTPUT_DIR=$BONE_ROOT/robots/fairino/ft_locked_trajectory_output
```

因此检测轨迹会默认写回 `bone/robots/fairino/ft_locked_trajectory_output/`，不会继续写到原来的 `/home/franka/massage/...` 目录。
