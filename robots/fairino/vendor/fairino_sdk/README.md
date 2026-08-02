# FAIRINO Python SDK

本目录保存项目运行时使用的厂商 Python SDK。版本变更记录见 `README.txt`；该
记录开头标明 SDK 从 V2.0.8 开始的适配历史。

`robots/fairino/fairino/__init__.py` 按以下优先级加载 SDK：

1. `FAIRINO_SDK_MODULE` 指定的模块文件。
2. `FAIRINO_SDK_BASE` 指定的完整 SDK 发行目录。
3. 项目内完整 SDK 发行目录。
4. `FAIRINO_SDK_ROOT` 指定的 Python 模块目录。
5. 本目录的 `Robot.py`。

正常运行不依赖 `/home/franka` 或 `py-xiaozhi/src/user_functions/` 中的副本。升级
SDK 时应整体替换厂商文件，记录 SDK 与机器人控制器版本，并重新执行只读连接和
运动测试。
