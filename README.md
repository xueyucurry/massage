# FAIRINO 智能按摩机器人

当前项目以 FAIRINO 机械臂为唯一主运行链，完成背部膀胱经和大腿内外侧
轨迹检测、点筋/分筋/顺筋恒力按摩，以及小智语音工具调用。

```text
小智语音/GUI
  -> MCP 按摩工具
  -> ft_agent_process.py / ft_agent_api.py
  -> ft.py
  -> YOLO 背部轨迹或 RTMPose 大腿轨迹
  -> FAIRINO ROS2 服务与六维力传感器
```

## 日常入口

所有常用操作都从仓库根目录的 `./massage` 启动：

```bash
cd /home/massage/massage
./massage gui
./massage start
./massage stop
./massage calibrate
./massage force-monitor
./massage adjust-lines
./massage record-home
./massage home
./massage help
```

`stop` 不是物理急停。真人或假人实验时仍需保证机械臂急停按钮可立即操作。

## 当前目录

| 路径 | 作用 | 当前状态 |
| --- | --- | --- |
| `massage` | 统一启动器、ROS2/GUI/相机进程管理 | 主入口 |
| `robots/fairino/` | 视觉、轨迹、力控、按摩执行和诊断工具 | 核心 |
| `py-xiaozhi/` | 小智客户端、GUI和按摩MCP工具 | 核心 |
| `shared/calibration/` | RealSense到机械臂的标定结果与标定程序 | 核心 |
| `shared/vision/` | YOLO权重和通用视觉资源 | 核心资源 |
| `robots/realman/` | 早期RealMan机械臂方案 | 历史保留，不参与当前运行 |
| `shared/third_party/co-tracker/` | 早期跟踪实验 | 历史保留，当前按摩链未启用 |
| `docs/` | 专利和项目材料 | 资料 |

FAIRINO核心文件和历史实验的详细边界见
[`robots/fairino/README.md`](robots/fairino/README.md)。

## 运行约束

- 目标环境为 WSL2 Ubuntu 22.04 + ROS2 Humble。
- `robots/fairino/ft.py` 是按摩执行主程序。
- ROS2 `build/`、`install/`、`log/`，Python虚拟环境、日志和运行状态均为本机产物，不进入Git。
- `massage_home_pose.json`、`back_line_offset.json`和`shared/calibration/*.json`是设备相关的有效配置，修改后应有意识地提交。
- RTMPose实现统一为 `rtmpose_detector.py`；`rtmpose.py`仅作为命令行入口。

## 版本快照

7月27日比赛版本保存在Git标签：

```bash
git show 7.27-competition-version
```

该标签保存比赛源码、按摩参数、初始位姿、标定结果和运行库链接；虚拟环境及
ROS2编译产物需要在目标机器重新安装或构建。

## 进一步文档

- 使用说明：`robots/fairino/FT_USAGE.md`
- 技术结构：`robots/fairino/FT_TECHNICAL.md`
- 智能体接口：`robots/fairino/FT_AGENT_API.md`
- 历史目录调整记录：`REORG_MANIFEST.md`
