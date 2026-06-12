#!/usr/bin/env python3
"""
FAIRINO 柔顺控制示例脚本。

按官方文档的“柔顺控制”流程组织：

1. RPC 连接
2. 力传感器配置 / 激活 / 校零
3. 开启 FT_Control
4. 开启 FT_ComplianceStart
5. 在两点之间执行 MoveL 往返
6. 关闭 FT_ComplianceStop
7. 关闭 FT_Control

主要参考：
- https://fairino-doc-zhs.readthedocs.io/latest/SDKManual/PythonRobotForceControl.html
- https://fairino-doc-zhs.readthedocs.io/latest/SDKManual/PythonRobotMovement.html
- https://fairino-doc-zhs.readthedocs.io/latest/SDKManual/PythonRobotBase.html

说明：
- 文档“柔顺控制代码示例”里先开启 FT_Control，再开启 FT_ComplianceStart。
- 文档对 FT_SetZero 的接口说明写的是：
  0=去除零点，1=零点矫正。
  因此本脚本在真正校零时使用 1。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

from fairino import Robot


DEFAULT_SELECT = [1, 1, 1, 0, 0, 0]
DEFAULT_TARGET_FT = [-10.0, -10.0, -10.0, 0.0, 0.0, 0.0]
DEFAULT_FT_PID = [0.0005, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_MB_M = [0.0, 0.0]
DEFAULT_MB_B = [0.0, 0.0]
DEFAULT_THRESHOLD = [0.2, 0.2]
DEFAULT_ADJUST = [1.0, 1.0]


def parse_float_list(text: str, expected_len: int, name: str) -> list[float]:
    values = [item.strip() for item in text.split(",") if item.strip()]
    if len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"{name} 需要 {expected_len} 个逗号分隔数值，当前得到 {len(values)} 个"
        )
    try:
        return [float(item) for item in values]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} 存在非数字项: {text}") from exc


def parse_int_list(text: str, expected_len: int, name: str) -> list[int]:
    values = [item.strip() for item in text.split(",") if item.strip()]
    if len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"{name} 需要 {expected_len} 个逗号分隔整数，当前得到 {len(values)} 个"
        )
    try:
        return [int(item) for item in values]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} 存在非整数项: {text}") from exc


def require_zero(name: str, errcode: int) -> None:
    if errcode != 0:
        raise RuntimeError(f"{name} 失败，errcode={errcode}")


def print_step(message: str) -> None:
    print(f"[Compliance] {message}", flush=True)


def sleep_s(seconds: float) -> None:
    if seconds > 0:
        time.sleep(seconds)


def move_l(robot, pose: Iterable[float], tool: int, user: int, vel: float, ovl: float, blend_r: float) -> None:
    err = robot.MoveL(
        desc_pos=list(pose),
        tool=tool,
        user=user,
        vel=vel,
        ovl=ovl,
        blendR=blend_r,
    )
    require_zero("MoveL", err)


def configure_sensor(robot, args: argparse.Namespace) -> None:
    print_step(
        "配置力传感器 "
        f"(company={args.sensor_company}, device={args.sensor_device}, bus={args.sensor_bus})"
    )
    require_zero(
        "FT_SetConfig",
        robot.FT_SetConfig(
            args.sensor_company,
            args.sensor_device,
            args.sensor_softversion,
            args.sensor_bus,
        ),
    )
    sleep_s(args.sleep_after_sensor_cmd)

    err, config = robot.FT_GetConfig()
    require_zero("FT_GetConfig", err)
    print_step(f"当前传感器配置: {config}")

    require_zero("FT_Activate(0)", robot.FT_Activate(0))
    sleep_s(args.sleep_after_sensor_cmd)
    require_zero("FT_Activate(1)", robot.FT_Activate(1))
    sleep_s(args.sleep_after_sensor_cmd)

    if not args.skip_zero:
        print_step("执行力传感器校零，确保当前末端未接触外物")
        require_zero("FT_SetZero(1)", robot.FT_SetZero(1))
        sleep_s(args.sleep_after_sensor_cmd)

    require_zero("FT_SetRCS(0)", robot.FT_SetRCS(0))

    if args.payload_weight is not None:
        require_zero("SetForceSensorPayload", robot.SetForceSensorPayload(args.payload_weight))

    if args.payload_cog is not None:
        cog = args.payload_cog
        require_zero("SetForceSensorPayloadCog", robot.SetForceSensorPayloadCog(cog[0], cog[1], cog[2]))


def set_force_control(robot, args: argparse.Namespace, enabled: bool) -> None:
    flag = 1 if enabled else 0
    err = robot.FT_Control(
        flag,
        args.sensor_id,
        args.select,
        args.target_ft,
        args.ft_pid,
        0,
        0,
        args.max_dis,
        args.max_ang,
        args.mb_m,
        args.mb_b,
        args.threshold,
        args.adjust_coeff,
        0,
        args.filter_sign,
        args.pos_adapt_sign,
        args.is_no_block,
    )
    require_zero(f"FT_Control({flag})", err)


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="根据 FAIRINO 官方文档编写的柔顺控制示例脚本。"
    )
    parser.add_argument("--ip", default="192.168.58.2", help="机器人控制器 IP")
    parser.add_argument("--tool", type=int, default=0, help="工具号")
    parser.add_argument("--user", type=int, default=0, help="工件号")
    parser.add_argument("--pose-a", required=True, help="起点位姿 x,y,z,rx,ry,rz")
    parser.add_argument("--pose-b", required=True, help="终点位姿 x,y,z,rx,ry,rz")
    parser.add_argument("--cycles", type=int, default=3, help="往返次数")
    parser.add_argument("--vel", type=float, default=20.0, help="MoveL 速度百分比")
    parser.add_argument("--ovl", type=float, default=100.0, help="MoveL 速度缩放")
    parser.add_argument("--blend-r", type=float, default=-1.0, help="MoveL blendR，-1 表示阻塞到位")
    parser.add_argument("--dwell", type=float, default=0.0, help="每次到位后的停留秒数")
    parser.add_argument("--mode-auto", action="store_true", help="脚本开始时切到自动模式")
    parser.add_argument("--enable", action="store_true", help="脚本开始时执行 RobotEnable(1)")

    parser.add_argument("--sensor-company", type=int, default=24, help="力传感器厂商编号，默认 24")
    parser.add_argument("--sensor-device", type=int, default=0, help="力传感器设备编号")
    parser.add_argument("--sensor-softversion", type=int, default=0, help="力传感器软件版本")
    parser.add_argument("--sensor-bus", type=int, default=1, help="力传感器总线编号")
    parser.add_argument("--sensor-id", type=int, default=1, help="力传感器编号")
    parser.add_argument("--skip-zero", action="store_true", help="跳过 FT_SetZero(1) 校零")
    parser.add_argument(
        "--sleep-after-sensor-cmd",
        type=float,
        default=1.0,
        help="每次传感器配置指令后的等待时间（秒）",
    )
    parser.add_argument("--payload-weight", type=float, default=None, help="力传感器负载重量 kg")
    parser.add_argument("--payload-cog", default=None, help="力传感器负载质心 x,y,z (mm)")

    parser.add_argument("--compliance-p", type=float, default=0.00005, help="FT_ComplianceStart 的 p")
    parser.add_argument("--compliance-force", type=float, default=30.0, help="FT_ComplianceStart 的力阈值 N")

    parser.add_argument("--select", default="1,1,1,0,0,0", help="FT_Control select 六维开关")
    parser.add_argument("--target-ft", default="-10,-10,-10,0,0,0", help="FT_Control 目标力/力矩")
    parser.add_argument("--ft-pid", default="0.0005,0,0,0,0,0", help="FT_Control PID 参数")
    parser.add_argument("--max-dis", type=float, default=100.0, help="FT_Control 最大调整距离 mm")
    parser.add_argument("--max-ang", type=float, default=0.0, help="FT_Control 最大调整角度 deg")
    parser.add_argument("--mb-m", default="0,0", help="FT_Control 的 M 参数")
    parser.add_argument("--mb-b", default="0,0", help="FT_Control 的 B 参数")
    parser.add_argument("--threshold", default="0.2,0.2", help="FT_Control rx/ry 启动阈值")
    parser.add_argument("--adjust-coeff", default="1,1", help="FT_Control rx/ry 调节系数")
    parser.add_argument("--filter-sign", type=int, choices=[0, 1], default=0, help="FT_Control 滤波开关")
    parser.add_argument(
        "--pos-adapt-sign",
        type=int,
        choices=[0, 1],
        default=0,
        help="FT_Control 姿态顺应开关",
    )
    parser.add_argument(
        "--is-no-block",
        type=int,
        choices=[0, 1],
        default=0,
        help="FT_Control 阻塞标志，0=阻塞，1=非阻塞",
    )
    return parser


def main() -> int:
    parser = build_argparser()
    args = parser.parse_args()

    args.pose_a = parse_float_list(args.pose_a, 6, "pose-a")
    args.pose_b = parse_float_list(args.pose_b, 6, "pose-b")
    args.select = parse_int_list(args.select, 6, "select")
    args.target_ft = parse_float_list(args.target_ft, 6, "target-ft")
    args.ft_pid = parse_float_list(args.ft_pid, 6, "ft-pid")
    args.mb_m = parse_float_list(args.mb_m, 2, "mb-m")
    args.mb_b = parse_float_list(args.mb_b, 2, "mb-b")
    args.threshold = parse_float_list(args.threshold, 2, "threshold")
    args.adjust_coeff = parse_float_list(args.adjust_coeff, 2, "adjust-coeff")
    if args.payload_cog is not None:
        args.payload_cog = parse_float_list(args.payload_cog, 3, "payload-cog")

    if args.cycles <= 0:
        parser.error("--cycles 必须大于 0")

    robot = None
    ft_control_enabled = False
    compliance_enabled = False

    try:
        print_step(f"连接机器人: {args.ip}")
        robot = Robot.RPC(args.ip)
        if robot is None:
            raise RuntimeError("RPC 连接失败，未返回机器人对象")

        if args.mode_auto:
            print_step("切换自动模式 Mode(0)")
            require_zero("Mode(0)", robot.Mode(0))

        if args.enable:
            print_step("机器人上使能 RobotEnable(1)")
            require_zero("RobotEnable(1)", robot.RobotEnable(1))
            sleep_s(0.5)

        configure_sensor(robot, args)

        print_step("开启 FT_Control")
        set_force_control(robot, args, True)
        ft_control_enabled = True

        print_step(
            f"开启柔顺控制 FT_ComplianceStart(p={args.compliance_p}, force={args.compliance_force})"
        )
        require_zero(
            "FT_ComplianceStart",
            robot.FT_ComplianceStart(args.compliance_p, args.compliance_force),
        )
        compliance_enabled = True

        for index in range(args.cycles):
            print_step(f"开始第 {index + 1}/{args.cycles} 次往返")
            move_l(robot, args.pose_a, args.tool, args.user, args.vel, args.ovl, args.blend_r)
            sleep_s(args.dwell)
            move_l(robot, args.pose_b, args.tool, args.user, args.vel, args.ovl, args.blend_r)
            sleep_s(args.dwell)

        print_step("动作完成")
        return 0

    except Exception as exc:
        print(f"[Compliance] 错误: {exc}", file=sys.stderr, flush=True)
        return 1

    finally:
        if robot is not None:
            if compliance_enabled:
                try:
                    print_step("关闭柔顺控制 FT_ComplianceStop")
                    err = robot.FT_ComplianceStop()
                    if err != 0:
                        print(f"[Compliance] FT_ComplianceStop 返回 errcode={err}", file=sys.stderr)
                except Exception as exc:
                    print(f"[Compliance] FT_ComplianceStop 异常: {exc}", file=sys.stderr)

            if ft_control_enabled:
                try:
                    print_step("关闭 FT_Control")
                    err = robot.FT_Control(
                        0,
                        args.sensor_id,
                        args.select,
                        args.target_ft,
                        args.ft_pid,
                        0,
                        0,
                        args.max_dis,
                        args.max_ang,
                        args.mb_m,
                        args.mb_b,
                        args.threshold,
                        args.adjust_coeff,
                        0,
                        args.filter_sign,
                        args.pos_adapt_sign,
                        args.is_no_block,
                    )
                    if err != 0:
                        print(f"[Compliance] FT_Control(0) 返回 errcode={err}", file=sys.stderr)
                except Exception as exc:
                    print(f"[Compliance] FT_Control(0) 异常: {exc}", file=sys.stderr)

            try:
                print_step("关闭 RPC 连接")
                robot.CloseRPC()
            except Exception as exc:
                print(f"[Compliance] CloseRPC 异常: {exc}", file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
