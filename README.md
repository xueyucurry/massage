# FAIRINO 智能按摩机器人

本项目以 FAIRINO 机械臂为唯一运行平台，实现背部膀胱经和大腿内外侧
轨迹检测、点筋/分筋/顺筋恒力按摩，以及小智语音智能体的工具调用。

```text
小智语音 / GUI
  -> MCP FAIRINO 按摩工具
  -> ft_agent_process.py / ft_agent_api.py
  -> ft.py
  -> YOLO 背部轨迹或 RTMPose 大腿轨迹
  -> FAIRINO ROS2 服务与六维力传感器
```

## 快速入口

所有日常操作都从仓库根目录的 `./massage` 启动：

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

`./massage stop` 是软件停止命令，不是物理急停。机械臂实验期间必须保证操作员
能够立即按下实体急停按钮。

## 目录边界

| 路径 | 内容 | 修改原则 |
| --- | --- | --- |
| `massage` | 统一启动器和进程管理 | 新增用户命令时修改 |
| `robots/fairino/` | 视觉、轨迹、力控和按摩执行 | 当前业务核心 |
| `robots/fairino/vendor/` | 随项目固定的 FAIRINO SDK | 仅升级厂商 SDK 时修改 |
| `py-xiaozhi/` | 小智客户端、GUI和 MCP 集成 | 保留上游结构，业务接口集中在 `fairino_massage` |
| `shared/calibration/` | 手眼标定程序与设备标定结果 | 标定后有意识地提交 JSON |
| `shared/vision/` | YOLO 实现和模型资源 | 视觉公共资源 |
| `docs/` | 项目结构、开发说明、论文和专利资料 | 面向开发者的文档入口 |

详细的依赖链和文件职责见 [项目结构](docs/PROJECT_STRUCTURE.md)，开发、验证和
安全约束见 [开发指南](docs/DEVELOPMENT.md)。FAIRINO 模块内部说明见
[robots/fairino/README.md](robots/fairino/README.md)。

## 运行约束

- 目标环境为 WSL2 Ubuntu 22.04 + ROS2 Humble。
- `robots/fairino/ft.py` 是按摩执行主程序。
- 小智由 `./massage gui` 启动时默认启用按摩专用模式，只暴露系统和 FAIRINO
  按摩工具，并禁用日程提醒插件。
- ROS2 `build/`、`install/`、`log/`、Python 虚拟环境、日志和运行状态都是
  本机产物，不应提交到 Git。
- `massage_home_pose.json`、`back_line_offset.json` 和
  `shared/calibration/*.json` 是设备相关有效配置，修改后需要单独复核。
- RTMPose 实现文件为 `rtmpose_detector.py`，`rtmpose.py` 只是命令行入口。

## 文档

- [项目结构](docs/PROJECT_STRUCTURE.md)
- [开发指南](docs/DEVELOPMENT.md)
- [运行与参数说明](robots/fairino/docs/FT_USAGE.md)
- [技术结构](robots/fairino/docs/FT_TECHNICAL.md)
- [智能体接口](robots/fairino/docs/FT_AGENT_API.md)
- [RTMPose 部署](robots/fairino/docs/RTMPOSE-DEPLOY.md)

## 历史版本

2026 年 7 月 27 日比赛前的完整版本同时保存在标签和归档分支中：

```bash
git show 7.27比赛版本
git switch archive/7.27-competition
```

归档包含当时已提交的源码、参数、初始位姿和标定结果。虚拟环境与 ROS2 编译
产物没有进入 Git，需要在目标机器重新安装或构建。
