#!/usr/bin/env python3
"""
力传感器读数自检：连接法奥控制器，激活六维力传感器并循环打印 Fx~Fz / Mx~Mz。

默认「最小初始化」：FT_SetConfig + FT_Activate(1)，不强制 FT_SetZero（便于先确认总线/读数是否正常）。
若需与 demo 一致流程，加 --full-init（会走 init_force_sensor，含校零与负载置零）。

用法:
  cd ~/massage && .venv/bin/python test_force_sensor.py
  .venv/bin/python test_force_sensor.py --ip 192.168.58.2 --duration 15 --hz 10
  .venv/bin/python test_force_sensor.py --full-init
  .venv/bin/python test_force_sensor.py --ros2
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import time

# Robot SDK（与 force_control / demo 一致）
try:
    from fairino import Robot
except Exception:
    try:
        from src.user_functions.fairino import Robot
    except Exception:
        import importlib
        import os

        _dir = os.path.dirname(os.path.abspath(__file__))
        if _dir not in sys.path:
            sys.path.insert(0, _dir)
        Robot = importlib.import_module("fairino").Robot

# 与 force_control 一致
FORCE_SENSOR_COMPANY = 24
FORCE_SENSOR_DEVICE = 0
FORCE_SENSOR_BUS = 1
ROS2_WORKSPACE = "/home/franka/massage/robots/fairino/fairino_ros2/frcobot_ros2-master"
ROS2_SERVICE_NAME = "fairino_remote_command_service"


def _set_config(robot) -> bool:
    """配置力传感器；优先使用显式 bus，旧 SDK 则回退到两参数调用。"""
    candidates = (
        (FORCE_SENSOR_COMPANY, FORCE_SENSOR_DEVICE, 0, FORCE_SENSOR_BUS),
        (FORCE_SENSOR_COMPANY, FORCE_SENSOR_DEVICE),
    )
    for args in candidates:
        try:
            rtn = robot.FT_SetConfig(*args)
        except TypeError:
            continue
        if rtn == 0:
            print(f"[Test] FT_SetConfig OK args={args}")
            return True
        print(f"[Test] FT_SetConfig{args} 失败: err={rtn}")
    return False


def _minimal_activate(robot) -> bool:
    """仅配置 + 激活，用于快速读数测试。"""
    if not _set_config(robot):
        return False

    rtn = robot.FT_Activate(0)
    if rtn != 0:
        print(f"[Test] FT_Activate(0) 警告: err={rtn}，继续尝试激活/读数")
    time.sleep(0.4)
    rtn = robot.FT_Activate(1)
    if rtn != 0:
        print(f"[Test] FT_Activate(1) 警告: err={rtn}，继续尝试读数")
    else:
        print("[Test] FT_Activate(1) OK，等待传感器稳定...")
    print("[Test] 等待传感器稳定...")
    time.sleep(0.6)
    return True


def _ros2_server_running() -> bool:
    try:
        return subprocess.run(
            [
                "pgrep",
                "-f",
                r"(^|/)ros2_cmd_server([[:space:]]|$)|ros2 run fairino_hardware ros2_cmd_server",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode == 0
    except Exception:
        return False


def _ros2_import_available() -> bool:
    try:
        import rclpy  # noqa: F401
        from fairino_msgs.srv import RemoteCmdInterface  # noqa: F401
        return True
    except Exception:
        return False


def _reexec_with_ros2_env(args) -> int:
    cmd = [
        shlex.quote(sys.executable),
        shlex.quote(os.path.abspath(__file__)),
        "--ros2",
        "--duration",
        shlex.quote(str(args.duration)),
        "--hz",
        shlex.quote(str(args.hz)),
        "--ros2-service",
        shlex.quote(str(args.ros2_service)),
    ]
    shell_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(ROS2_WORKSPACE)}/install/setup.bash && "
        "exec " + " ".join(cmd)
    )
    return subprocess.call(["bash", "-lc", shell_cmd])


def _parse_ros2_force_response(cmd_res):
    parts = [p.strip() for p in str(cmd_res).split(",")]
    try:
        ret = int(float(parts[0]))
    except Exception:
        ret = -9999
    data = []
    for value in parts[1:7]:
        try:
            data.append(float(value))
        except Exception:
            data.append(float("nan"))
    return ret, data


def _run_ros2_reader(args) -> int:
    if not _ros2_import_available():
        return _reexec_with_ros2_env(args)

    import rclpy
    from fairino_msgs.srv import RemoteCmdInterface

    def call(node, client, cmd, timeout=3.0):
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"ROS2 指令超时: {cmd}")
        if future.exception() is not None:
            raise RuntimeError(f"ROS2 指令异常 {cmd}: {future.exception()}")
        return future.result().cmd_res

    period = 1.0 / max(1.0, float(args.hz))
    rclpy.init(args=None)
    node = rclpy.create_node("force_sensor_test_ros2")
    client = node.create_client(RemoteCmdInterface, args.ros2_service)

    try:
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"[Test] ROS2 服务未就绪: {args.ros2_service}")
            return 1

        for cmd in (
            f"FT_SetConfig({FORCE_SENSOR_COMPANY},{FORCE_SENSOR_DEVICE},0,{FORCE_SENSOR_BUS})",
            "FT_Activate(1)",
        ):
            try:
                print(f"[Test] {cmd} -> {call(node, client, cmd, timeout=5.0)}")
            except Exception as exc:
                print(f"[Test] {cmd} 警告: {exc}")

        print("\n[Test] ROS2 模式读取力传感器，不发送运动指令。")
        print(f"\n{'Time':>8} {'Fx':>8} {'Fy':>8} {'Fz':>8} {'Mx':>8} {'My':>8} {'Mz':>8}  (N / Nm)")
        print("-" * 68)

        t0 = time.time()
        n_ok = 0
        n_bad = 0
        while time.time() - t0 < float(args.duration):
            elapsed = time.time() - t0
            try:
                cmd_res = call(node, client, "FT_GetForceTorqueRCS(1)", timeout=2.0)
                ret, data = _parse_ros2_force_response(cmd_res)
                if ret == 0 and len(data) >= 6:
                    print(
                        f"{elapsed:>8.2f} {data[0]:>8.2f} {data[1]:>8.2f} {data[2]:>8.2f} "
                        f"{data[3]:>8.3f} {data[4]:>8.3f} {data[5]:>8.3f}"
                    )
                    n_ok += 1
                else:
                    n_bad += 1
                    print(f"[Test] FT_GetForceTorqueRCS 异常: {cmd_res}")
            except KeyboardInterrupt:
                print("\n[Test] 用户中断")
                break
            except Exception as exc:
                n_bad += 1
                print(f"[Test] FT_GetForceTorqueRCS 异常: {exc}")
            time.sleep(period)

        print("-" * 68)
        print(f"[Test] 有效样本: {n_ok}, 失败次数: {n_bad}")
        if n_ok > 0:
            print("[Test] 读数正常：若数值随按压/扰动变化，说明传感器与总线工作正常。")
        else:
            print("[Test] 未读到有效数据：检查 ROS2 控制服务、RS485 与力传感器配置。")
        print("[Test] 完成")
        return 0 if n_ok > 0 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    p = argparse.ArgumentParser(description="力传感器读数测试（法奥 XJC 经控制柜 RS485）")
    p.add_argument("--ip", default="192.168.58.2", help="机器人控制器 IP")
    p.add_argument("--duration", type=float, default=10.0, help="读数时长 (秒)")
    p.add_argument("--hz", type=float, default=10.0, help="打印频率 (Hz)")
    p.add_argument("--ros2", action="store_true", help="通过已运行的 ROS2 控制服务读取，避免 SDK 直连冲突")
    p.add_argument("--sdk", action="store_true", help="强制使用 SDK 直连读取")
    p.add_argument("--ros2-service", default=ROS2_SERVICE_NAME, help="ROS2 控制服务名")
    p.add_argument(
        "--full-init",
        action="store_true",
        help="使用 force_control.init_force_sensor 完整流程（含负载置零与 FT_SetZero）",
    )
    args = p.parse_args()

    if args.ros2 or (not args.sdk and _ros2_server_running()):
        if not args.ros2:
            print("[Test] 检测到 ROS2 控制服务正在运行，自动切换 ROS2 读数模式。", flush=True)
        return _run_ros2_reader(args)

    period = 1.0 / max(1.0, float(args.hz))

    print(f"[Test] 连接 {args.ip} ...")
    robot = Robot.RPC(args.ip)

    if args.full_init:
        from force_control import ForceControlConfig, init_force_sensor

        cfg = ForceControlConfig()
        if not init_force_sensor(robot, cfg):
            print("[Test] init_force_sensor 失败，退出")
            try:
                robot.FT_Activate(0)
                robot.CloseRPC()
            except Exception:
                pass
            return 1
    else:
        if not _minimal_activate(robot):
            try:
                robot.CloseRPC()
            except Exception:
                pass
            return 1

    print(f"\n{'Time':>8} {'Fx':>8} {'Fy':>8} {'Fz':>8} {'Mx':>8} {'My':>8} {'Mz':>8}  (N / Nm)")
    print("-" * 68)

    t0 = time.time()
    n_ok = 0
    n_bad = 0
    try:
        while time.time() - t0 < float(args.duration):
            ret = robot.FT_GetForceTorqueRCS()
            if isinstance(ret, tuple) and ret[0] == 0:
                d = ret[1]
                elapsed = time.time() - t0
                print(
                    f"{elapsed:>8.2f} {d[0]:>8.2f} {d[1]:>8.2f} {d[2]:>8.2f} "
                    f"{d[3]:>8.3f} {d[4]:>8.3f} {d[5]:>8.3f}"
                )
                n_ok += 1
            else:
                n_bad += 1
                print(f"[Test] FT_GetForceTorqueRCS 异常: {ret}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[Test] 用户中断")

    print("-" * 68)
    print(f"[Test] 有效样本: {n_ok}, 失败次数: {n_bad}")
    if n_ok > 0:
        print("[Test] 读数正常：若数值随按压/扰动变化，说明传感器与总线工作正常。")
    else:
        print("[Test] 未读到有效数据：检查 RS485、传感器型号、FT_SetConfig 参数及示教器力传感器状态。")

    try:
        robot.FT_Activate(0)
        robot.CloseRPC()
    except Exception:
        pass
    print("[Test] 完成")
    return 0 if n_ok > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
