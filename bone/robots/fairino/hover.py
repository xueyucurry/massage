"""
hover.py - 悬空轨迹检查入口（ROS 2 控制版）

复用 ft.py 的统一交互检测入口，但强制关闭恒力控制。
流程：选择背部/腿部 -> 实时检测 -> 按 s 保存轨迹 -> 按 g 执行悬空动作。
"""

import os
import sys


os.environ["LASTTIME_ROS2_FORCE"] = "0"
os.environ["LASTTIME_FORCE_CONTROL"] = "0"

from ft import main  # noqa: E402


if __name__ == "__main__":
    sys.exit(main())
