#!/usr/bin/env python3
"""
unlock_robot.py - 解锁机械臂

当程序异常退出导致机械臂锁住时，运行此脚本解锁
"""

import sys
import os

# 导入机械臂SDK
try:
    from fairino import Robot
except Exception:
    _dir = os.path.dirname(__file__)
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from fairino import Robot

ROBOT_IP = "192.168.58.2"

def unlock_robot():
    """解锁机械臂"""
    print(f"连接机械臂 {ROBOT_IP}...")

    try:
        robot = Robot.RPC(ROBOT_IP)
        print("连接成功")

        print("关闭RPC连接...")
        robot.CloseRPC()
        print("机械臂已解锁")

        return True

    except Exception as e:
        print(f"解锁失败: {e}")
        return False

if __name__ == "__main__":
    success = unlock_robot()
    sys.exit(0 if success else 1)
