#!/usr/bin/env python3
"""
FAIRINO 原位弹簧模拟 / 托举顺应示例。

设计目标：
- 机器人停在当前位置，不执行预设轨迹
- 优先尝试控制器原生笛卡尔阻抗控制
- 若控制器不支持，则回退到“拖动 + 关节伺服回中”的混合弹簧模式
- 当前位置视作弹簧平衡点
- 人手托起机械臂后，撤力时自动回到启动参考位

说明：
- 当前控制器不支持 ImpedanceControlStartStop，因此实际主要依赖回退模式
- 回退模式不是控制器原生全闭环弹簧，而是：
  1. 手动拖动示教让机械臂可被手托动
  2. 检测到撤力后，退出拖动并切到 ServoJ(UDP) 回参考关节位
  3. 回到位或再次受力后，再切回拖动模式
- 手托机械臂本体时，外力未必能完整反映到末端六维力传感器，所以撤力判定同时参考速度和力值
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from fairino import Robot


STOP_REQUESTED = False


def handle_signal(signum, frame) -> None:
    del signum, frame
    global STOP_REQUESTED
    STOP_REQUESTED = True


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


def require_zero(name: str, errcode: int) -> None:
    if errcode != 0:
        raise RuntimeError(f"{name} 失败，errcode={errcode}")


def print_step(message: str) -> None:
    print(f"[Spring] {message}", flush=True)


def sleep_s(seconds: float) -> None:
    if seconds > 0:
        time.sleep(seconds)


def wait_for_drag_state(robot, target_state: int, timeout_s: float, poll_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() <= deadline:
        err, state = robot.IsInDragTeach()
        if err == 0 and state == target_state:
            return True
        sleep_s(poll_s)
    return False


def switch_mode(robot, mode: int, wait_s: float = 0.5) -> None:
    require_zero(f"Mode({mode})", robot.Mode(mode))
    sleep_s(wait_s)


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


def get_tcp_pose(robot) -> list[float] | None:
    ret = robot.GetActualTCPPose()
    if isinstance(ret, tuple) and ret[0] == 0:
        return list(ret[1])
    return None


def get_force(robot) -> list[float] | None:
    ret = robot.FT_GetForceTorqueRCS()
    if isinstance(ret, tuple) and ret[0] == 0:
        return list(ret[1])
    return None


def get_joint_pos_degree(robot) -> list[float] | None:
    ret = robot.GetActualJointPosDegree()
    if isinstance(ret, tuple) and ret[0] == 0:
        return list(ret[1])
    return None


def get_joint_speed_degree(robot) -> list[float] | None:
    ret = robot.GetActualJointSpeedsDegree()
    if isinstance(ret, tuple) and ret[0] == 0:
        return list(ret[1])
    return None


def get_joint_torques(robot) -> list[float] | None:
    ret = robot.GetJointTorques(1)
    if isinstance(ret, tuple) and ret[0] == 0:
        return list(ret[1])
    return None


def force_norm(ft: list[float]) -> float:
    return math.sqrt(ft[0] ** 2 + ft[1] ** 2 + ft[2] ** 2)


def translational_error_mm(reference: list[float], current: list[float]) -> float:
    return math.sqrt(
        (reference[0] - current[0]) ** 2
        + (reference[1] - current[1]) ** 2
        + (reference[2] - current[2]) ** 2
    )


def rotational_error_deg(reference: list[float], current: list[float]) -> float:
    return math.sqrt(
        (reference[3] - current[3]) ** 2
        + (reference[4] - current[4]) ** 2
        + (reference[5] - current[5]) ** 2
    )


def clamp_abs(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def build_return_step(reference: list[float], current: list[float], args: argparse.Namespace) -> list[float]:
    step = [0.0] * 6
    for idx in range(3):
        err = reference[idx] - current[idx]
        step[idx] = clamp_abs(err * args.return_k_xyz, args.return_max_step_xyz)
    for idx in range(3, 6):
        err = reference[idx] - current[idx]
        step[idx] = clamp_abs(err * args.return_k_rpy, args.return_max_step_rpy)
    return step


def joint_error_norm_deg(reference: list[float], current: list[float]) -> float:
    return math.sqrt(sum((reference[idx] - current[idx]) ** 2 for idx in range(6)))


def joint_speed_norm_deg_s(current_speed: list[float]) -> float:
    return math.sqrt(sum(item ** 2 for item in current_speed))


def build_return_joint_target(
    reference_joint: list[float],
    current_joint: list[float],
    args: argparse.Namespace,
) -> list[float]:
    target_joint = [0.0] * 6
    for idx in range(6):
        err = reference_joint[idx] - current_joint[idx]
        step = clamp_abs(err * args.return_joint_k[idx], args.return_joint_max_step[idx])
        target_joint[idx] = current_joint[idx] + step
    return target_joint


def build_servojt_command(
    reference_joint: list[float],
    reference_torque: list[float],
    current_joint: list[float],
    current_speed: list[float],
    args: argparse.Namespace,
) -> tuple[list[float], list[float]]:
    torque_cmd = [0.0] * 6
    torque_delta = [0.0] * 6
    for idx in range(6):
        joint_err = reference_joint[idx] - current_joint[idx]
        if abs(joint_err) < args.torque_spring_deadband[idx]:
            joint_err = 0.0
        delta = args.torque_spring_k[idx] * joint_err - args.torque_spring_d[idx] * current_speed[idx]
        delta = clamp_abs(delta, args.torque_spring_max_delta[idx])
        torque_delta[idx] = delta
        torque_cmd[idx] = reference_torque[idx] + delta
    return torque_cmd, torque_delta


def start_torque_spring(robot, args: argparse.Namespace) -> dict[str, list[float]]:
    reference_joint = get_joint_pos_degree(robot)
    if reference_joint is None:
        raise RuntimeError('GetActualJointPosDegree 失败，无法读取当前关节角')

    reference_torque = get_joint_torques(robot)
    if reference_torque is None:
        raise RuntimeError('GetJointTorques 失败，无法读取当前关节扭矩')

    err = robot.SetPowerLimit(1, args.torque_power_limit)
    if err == 0:
        print_step(f'已开启关节功率限制 power={args.torque_power_limit:.1f}W')
    else:
        print(f'[Spring] SetPowerLimit(1, {args.torque_power_limit:.1f}) 返回 errcode={err}', file=sys.stderr)

    require_zero(f'ServoJTStart(cmdType={args.torque_cmd_type})', robot.ServoJTStart(args.torque_cmd_type))
    print_step(
        'ServoJT 扭矩弹簧已启动，参考关节='
        f'[{reference_joint[0]:.2f}, {reference_joint[1]:.2f}, {reference_joint[2]:.2f}, '
        f'{reference_joint[3]:.2f}, {reference_joint[4]:.2f}, {reference_joint[5]:.2f}]'
    )
    return {
        'reference_joint': reference_joint,
        'reference_torque': reference_torque,
    }


def stop_torque_spring(robot, args: argparse.Namespace) -> None:
    try:
        err = robot.ServoJTEnd(args.torque_cmd_type)
        if err == 0:
            print_step('ServoJT 扭矩弹簧已停止')
        else:
            print(f'[Spring] ServoJTEnd() 返回 errcode={err}', file=sys.stderr)
    except Exception as exc:
        print(f'[Spring] 停止 ServoJT 异常: {exc}', file=sys.stderr)

    try:
        err = robot.SetPowerLimit(0, args.torque_power_limit)
        if err != 0:
            print(f'[Spring] SetPowerLimit(0, {args.torque_power_limit:.1f}) 返回 errcode={err}', file=sys.stderr)
    except Exception:
        pass


def enter_joint_drag(robot, args: argparse.Namespace) -> None:
    print_step(f"切换手动模式 Mode(1)，进入托举拖动 (backend={args.drag_backend})")
    switch_mode(robot, 1, args.sleep_after_mode_cmd)
    require_zero("DragTeachSwitch(1)", robot.DragTeachSwitch(1))
    if args.drag_backend == "force_joint":
        require_zero(
            "ForceAndJointImpedanceStartStop(1)",
            robot.ForceAndJointImpedanceStartStop(
                1,
                args.drag_impedance_flag,
                args.drag_lambda,
                args.drag_k,
                args.drag_b,
                args.drag_max_tcp_vel,
                args.drag_max_tcp_ori_vel,
            ),
        )
    elif args.drag_backend == "plain":
        print_step("当前使用纯拖动示教，不叠加混合拖动阻尼")
    else:
        print_step("当前使用拖动示教 + ServoJT 关节扭矩弹簧")
    err, drag_state = robot.IsInDragTeach()
    if err == 0:
        print_step(f"当前拖动示教状态: {drag_state}")


def exit_joint_drag(robot, args: argparse.Namespace) -> None:
    try:
        err = robot.DragTeachSwitch(0)
        if err == 0:
            print_step("已请求退出拖动示教，等待控制器确认")
        else:
            print(f"[Spring] DragTeachSwitch(0) 返回 errcode={err}", file=sys.stderr)
    except Exception as exc:
        print(f"[Spring] 关闭拖动示教异常: {exc}", file=sys.stderr)

    if wait_for_drag_state(robot, 0, args.drag_exit_timeout, args.drag_exit_poll_interval):
        print_step("拖动示教已退出")
    else:
        print(
            f"[Spring] 等待退出拖动示教超时 ({args.drag_exit_timeout:.2f}s)，继续尝试清理拖动状态",
            file=sys.stderr,
        )

    sleep_s(args.sleep_after_drag_exit)

    if args.drag_backend == "force_joint":
        try:
            err = robot.ForceAndJointImpedanceStartStop(
                0,
                args.drag_impedance_flag,
                args.drag_lambda,
                args.drag_k,
                args.drag_b,
                args.drag_max_tcp_vel,
                args.drag_max_tcp_ori_vel,
            )
            if err == 0:
                print_step("关节拖动模式已关闭")
            else:
                print(f"[Spring] ForceAndJointImpedanceStartStop(0) 返回 errcode={err}", file=sys.stderr)
        except Exception as exc:
            print(f"[Spring] 关闭关节拖动异常: {exc}", file=sys.stderr)


def start_servo_return(robot, args: argparse.Namespace) -> None:
    print_step(f"切换自动模式 Mode(0)，开始 ServoJ 回参考位 (cmdType={args.return_cmd_type})")
    mode_err = robot.Mode(0)
    if mode_err == 123:
        print_step("Mode(0) 仍提示未退出拖动，追加等待后重试")
        wait_for_drag_state(robot, 0, args.drag_exit_timeout, args.drag_exit_poll_interval)
        sleep_s(args.sleep_after_drag_exit)
        mode_err = robot.Mode(0)
    require_zero("Mode(0)", mode_err)
    sleep_s(args.sleep_after_mode_cmd)
    require_zero(
        f"ServoMoveStart(cmdType={args.return_cmd_type})",
        robot.ServoMoveStart(args.return_cmd_type),
    )


def stop_servo_return(robot, args: argparse.Namespace) -> None:
    try:
        err = robot.ServoMoveEnd(args.return_cmd_type)
        if err == 0:
            print_step("ServoJ 回中已停止")
        else:
            print(f"[Spring] ServoMoveEnd(cmdType={args.return_cmd_type}) 返回 errcode={err}", file=sys.stderr)
    except Exception as exc:
        print(f"[Spring] 停止 ServoJ 回中异常: {exc}", file=sys.stderr)

    try:
        err = robot.StopMotion()
        if err != 0:
            print(f"[Spring] StopMotion() 返回 errcode={err}", file=sys.stderr)
    except Exception:
        pass


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FAIRINO 原位弹簧模拟 / 托举顺应示例")
    parser.add_argument("--ip", default="192.168.58.2", help="机器人控制器 IP")
    parser.add_argument("--mode-auto", action="store_true", help="脚本开始时切到自动模式")
    parser.add_argument("--enable", action="store_true", help="脚本开始时执行 RobotEnable(1)")
    parser.add_argument(
        "--sleep-after-mode-cmd",
        type=float,
        default=0.5,
        help="模式切换后的等待时间（秒）",
    )

    parser.add_argument("--sensor-company", type=int, default=24, help="力传感器厂商编号")
    parser.add_argument("--sensor-device", type=int, default=0, help="力传感器设备编号")
    parser.add_argument("--sensor-softversion", type=int, default=0, help="力传感器软件版本")
    parser.add_argument("--sensor-bus", type=int, default=1, help="力传感器总线编号")
    parser.add_argument("--skip-zero", action="store_true", help="跳过 FT_SetZero(1) 校零")
    parser.add_argument(
        "--sleep-after-sensor-cmd",
        type=float,
        default=1.0,
        help="每次传感器配置指令后的等待时间（秒）",
    )
    parser.add_argument("--payload-weight", type=float, default=None, help="负载重量 kg")
    parser.add_argument("--payload-cog", default=None, help="负载质心 x,y,z (mm)")

    parser.add_argument(
        "--control-mode",
        choices=["auto", "impedance", "joint_drag"],
        default="auto",
        help="auto=优先阻抗，不支持则回退到混合弹簧模式",
    )

    parser.add_argument("--workspace", type=int, choices=[0, 1], default=1, help="0=关节空间，1=笛卡尔空间")
    parser.add_argument("--force-threshold", default="80,80,4,8,8,8", help="六维触发力阈值")
    parser.add_argument("--mass", default="2,2,0.8,0.2,0.2,0.2", help="六维质量参数")
    parser.add_argument("--damping", default="120,120,35,2,2,2", help="六维阻尼参数")
    parser.add_argument("--stiffness", default="1000,1000,20,80,80,80", help="六维刚度参数")
    parser.add_argument("--max-v", type=float, default=60.0, help="最大线速度 mm/s")
    parser.add_argument("--max-va", type=float, default=120.0, help="最大线加速度 mm/s^2")
    parser.add_argument("--max-w", type=float, default=20.0, help="最大角速度 deg/s")
    parser.add_argument("--max-wa", type=float, default=40.0, help="最大角加速度 deg/s^2")

    parser.add_argument("--drag-backend", choices=["plain", "force_joint", "servojt"], default="plain", help="拖动态实现：plain=纯拖动示教，force_joint=叠加六维力和关节阻抗混合拖动，servojt=拖动示教 + 关节扭矩弹簧")
    parser.add_argument("--drag-lambda", default="3,2,2,2,2,3", help="关节拖动增益")
    parser.add_argument(
        "--drag-impedance-flag",
        type=int,
        choices=[0, 1],
        default=0,
        help="控制器混合拖动里的原生阻抗开关，默认关闭，改由脚本做回中",
    )
    parser.add_argument("--drag-k", default="0,0,0,0,0,0", help="关节拖动刚度")
    parser.add_argument("--drag-b", default="150,150,150,5,5,1", help="关节拖动阻尼")
    parser.add_argument("--drag-max-tcp-vel", type=float, default=1000.0, help="拖动末端最大线速度")
    parser.add_argument("--drag-max-tcp-ori-vel", type=float, default=180.0, help="拖动末端最大角速度")
    parser.add_argument("--drag-exit-timeout", type=float, default=5.0, help="退出拖动示教的最长等待时间 s")
    parser.add_argument("--drag-exit-poll-interval", type=float, default=0.1, help="轮询拖动退出状态的周期 s")
    parser.add_argument("--sleep-after-drag-exit", type=float, default=1.0, help="确认退出拖动后再切自动模式前额外等待 s")
    parser.add_argument("--torque-spring-k", default="0.03,0.12,0.12,0.015,0.015,0.01", help="ServoJT 弹簧刚度 Nm/deg")
    parser.add_argument("--torque-spring-d", default="0.004,0.02,0.02,0.002,0.002,0.001", help="ServoJT 阻尼 Nm/(deg/s)")
    parser.add_argument("--torque-spring-max-delta", default="0.8,3.0,3.0,0.5,0.5,0.3", help="ServoJT 每关节最大附加扭矩 Nm")
    parser.add_argument("--torque-spring-deadband", default="0.2,0.2,0.2,0.1,0.1,0.1", help="ServoJT 关节误差死区 deg")
    parser.add_argument("--torque-loop-interval", type=float, default=0.008, help="ServoJT 指令周期 s")
    parser.add_argument("--torque-cmd-type", type=int, choices=[0, 1], default=1, help="ServoJT 命令通道，0=XML-RPC，1=UDP 透传")
    parser.add_argument("--torque-power-limit", type=float, default=150.0, help="ServoJT 功率限制 W")

    parser.add_argument("--return-enabled", type=int, choices=[0, 1], default=1, help="撤力后是否主动回参考位")
    parser.add_argument("--return-force-threshold", type=float, default=1.5, help="判定撤力的力阈值 N")
    parser.add_argument("--return-reacquire-force", type=float, default=30.0, help="回中过程中重新受力阈值 N")
    parser.add_argument("--return-reacquire-delay", type=float, default=1.0, help="开始回中后多久才允许重新抢回拖动 s")
    parser.add_argument("--return-reacquire-speed", type=float, default=6.0, help="重新抢回拖动所需的关节速度阈值 deg/s")
    parser.add_argument("--return-release-hold", type=float, default=0.5, help="撤力保持多久后开始回中（秒）")
    parser.add_argument("--return-translation-band", type=float, default=12.0, help="位移超过多少 mm 才触发回中")
    parser.add_argument("--return-rotation-band", type=float, default=4.0, help="姿态偏差超过多少 deg 才触发回中")
    parser.add_argument("--return-k-xyz", type=float, default=0.08, help="XYZ 回中比例增益")
    parser.add_argument("--return-k-rpy", type=float, default=0.05, help="姿态回中比例增益")
    parser.add_argument("--return-max-step-xyz", type=float, default=3.0, help="单周期最大 XYZ 回中步长 mm")
    parser.add_argument("--return-max-step-rpy", type=float, default=0.6, help="单周期最大姿态回中步长 deg")
    parser.add_argument("--return-joint-k", default="0.25,0.35,0.35,0.18,0.18,0.12", help="ServoJ 回中比例增益")
    parser.add_argument("--return-joint-max-step", default="0.6,1.0,1.0,0.4,0.4,0.3", help="ServoJ 单周期最大关节步长 deg")
    parser.add_argument("--return-joint-band", type=float, default=1.0, help="关节误差低于该阈值视为已回中 deg")
    parser.add_argument("--return-release-speed", type=float, default=1.2, help="拖动态下速度低于该阈值并保持一段时间视为已撤力 deg/s")
    parser.add_argument("--return-cmdt", type=float, default=0.008, help="ServoJ 指令周期 s")
    parser.add_argument("--return-cmd-type", type=int, choices=[0, 1], default=1, help="ServoJ 命令通道，0=XML-RPC，1=UDP 透传")

    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="保持时间，0 表示一直运行到 Ctrl-C",
    )
    parser.add_argument("--log-interval", type=float, default=0.2, help="状态打印周期（秒）")
    return parser


def main() -> int:
    parser = build_argparser()
    args = parser.parse_args()

    args.force_threshold = parse_float_list(args.force_threshold, 6, "force-threshold")
    args.mass = parse_float_list(args.mass, 6, "mass")
    args.damping = parse_float_list(args.damping, 6, "damping")
    args.stiffness = parse_float_list(args.stiffness, 6, "stiffness")
    args.drag_lambda = parse_float_list(args.drag_lambda, 6, "drag-lambda")
    args.drag_k = parse_float_list(args.drag_k, 6, "drag-k")
    args.drag_b = parse_float_list(args.drag_b, 6, "drag-b")
    args.torque_spring_k = parse_float_list(args.torque_spring_k, 6, "torque-spring-k")
    args.torque_spring_d = parse_float_list(args.torque_spring_d, 6, "torque-spring-d")
    args.torque_spring_max_delta = parse_float_list(args.torque_spring_max_delta, 6, "torque-spring-max-delta")
    args.torque_spring_deadband = parse_float_list(args.torque_spring_deadband, 6, "torque-spring-deadband")
    args.return_joint_k = parse_float_list(args.return_joint_k, 6, "return-joint-k")
    args.return_joint_max_step = parse_float_list(args.return_joint_max_step, 6, "return-joint-max-step")
    if args.payload_cog is not None:
        args.payload_cog = parse_float_list(args.payload_cog, 3, "payload-cog")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    robot = None
    active_mode = None
    runtime_state = None
    servo_active = False
    drag_active = False
    torque_spring_active = False
    torque_spring_ctx: dict[str, list[float]] | None = None

    try:
        print_step(f"连接机器人: {args.ip}")
        robot = Robot.RPC(args.ip)
        if robot is None:
            raise RuntimeError("RPC 连接失败，未返回机器人对象")

        if args.mode_auto:
            print_step("切换自动模式 Mode(0)")
            switch_mode(robot, 0, args.sleep_after_mode_cmd)

        if args.enable:
            print_step("机器人上使能 RobotEnable(1)")
            require_zero("RobotEnable(1)", robot.RobotEnable(1))
            sleep_s(0.5)

        configure_sensor(robot, args)

        reference_pose = get_tcp_pose(robot)
        if reference_pose is None:
            raise RuntimeError("GetActualTCPPose 失败，无法读取当前位姿")
        reference_joint = get_joint_pos_degree(robot)
        if reference_joint is None:
            raise RuntimeError("GetActualJointPosDegree 失败，无法读取当前关节角")
        print_step(
            "当前参考点 TCP="
            f"[{reference_pose[0]:.2f}, {reference_pose[1]:.2f}, {reference_pose[2]:.2f}, "
            f"{reference_pose[3]:.2f}, {reference_pose[4]:.2f}, {reference_pose[5]:.2f}]"
        )
        print_step(
            "当前参考关节="
            f"[{reference_joint[0]:.2f}, {reference_joint[1]:.2f}, {reference_joint[2]:.2f}, "
            f"{reference_joint[3]:.2f}, {reference_joint[4]:.2f}, {reference_joint[5]:.2f}]"
        )

        requested_mode = args.control_mode
        if requested_mode in ("auto", "impedance"):
            try:
                print_step(
                    "开启阻抗控制 "
                    f"(workspace={args.workspace}, threshold={args.force_threshold}, "
                    f"m={args.mass}, b={args.damping}, k={args.stiffness})"
                )
                require_zero(
                    "ImpedanceControlStartStop(1)",
                    robot.ImpedanceControlStartStop(
                        1,
                        args.workspace,
                        args.force_threshold,
                        args.mass,
                        args.damping,
                        args.stiffness,
                        args.max_v,
                        args.max_va,
                        args.max_w,
                        args.max_wa,
                    ),
                )
                active_mode = "impedance"
                runtime_state = "impedance"
            except Exception as exc:
                if requested_mode == "impedance":
                    raise
                print_step(f"阻抗控制不可用，自动回退到混合弹簧模式: {exc}")

        if active_mode is None:
            print_step(
                "开启混合弹簧模式 "
                f"(backend={args.drag_backend}, drag_impedance={args.drag_impedance_flag}, lambda={args.drag_lambda}, "
                f"k={args.drag_k}, b={args.drag_b}, return={args.return_enabled})"
            )
            if args.drag_backend == "servojt":
                active_mode = "torque_spring"
                runtime_state = "torque_spring"
                enter_joint_drag(robot, args)
                drag_active = True
                torque_spring_ctx = start_torque_spring(robot, args)
                torque_spring_active = True
                print_step("现在可以托起机械臂。ServoJT 扭矩弹簧会持续把机械臂拉回启动参考位。按 Ctrl-C 退出。")
            else:
                active_mode = "hybrid_spring"
                runtime_state = "drag"
                enter_joint_drag(robot, args)
                drag_active = True
                print_step("现在可以托起机械臂。撤力后脚本会退出拖动并通过 ServoJ 把机械臂拉回启动参考位。按 Ctrl-C 退出。")

        if active_mode == "impedance":
            print_step("阻抗控制已开启。现在可以从下方托住机械臂，观察 Z 向顺应。按 Ctrl-C 退出。")

        start_time = time.time()
        next_log_time = 0.0
        release_candidate_since: float | None = None
        return_started_at: float | None = None

        while not STOP_REQUESTED:
            elapsed = time.time() - start_time
            if args.hold_seconds > 0 and elapsed >= args.hold_seconds:
                break

            pose = get_tcp_pose(robot)
            ft = get_force(robot)
            joint_pos = get_joint_pos_degree(robot)
            joint_speed = get_joint_speed_degree(robot)
            trans_err = None
            rot_err = None
            force_mag = None
            dz = None
            joint_err_deg = None
            joint_speed_mag = None
            torque_delta_max = None

            if pose is not None:
                dz = pose[2] - reference_pose[2]
                trans_err = translational_error_mm(reference_pose, pose)
                rot_err = rotational_error_deg(reference_pose, pose)
            if ft is not None:
                force_mag = force_norm(ft)
            if joint_pos is not None:
                joint_err_deg = joint_error_norm_deg(reference_joint, joint_pos)
            if joint_speed is not None:
                joint_speed_mag = joint_speed_norm_deg_s(joint_speed)

            if active_mode == "torque_spring":
                if torque_spring_ctx is not None and joint_pos is not None:
                    joint_err_deg = joint_error_norm_deg(torque_spring_ctx["reference_joint"], joint_pos)
                if torque_spring_ctx is not None and joint_pos is not None and joint_speed is not None:
                    torque_cmd, torque_delta = build_servojt_command(
                        torque_spring_ctx["reference_joint"],
                        torque_spring_ctx["reference_torque"],
                        joint_pos,
                        joint_speed,
                        args,
                    )
                    torque_delta_max = max(abs(item) for item in torque_delta)
                    require_zero("ServoJT", robot.ServoJT(torque_cmd, args.torque_loop_interval, cmdType=args.torque_cmd_type))

            elif active_mode == "hybrid_spring" and joint_pos is not None:
                moved_far = False
                if trans_err is not None and rot_err is not None:
                    moved_far = trans_err >= args.return_translation_band or rot_err >= args.return_rotation_band
                if joint_err_deg is not None and joint_err_deg >= args.return_joint_band:
                    moved_far = True

                release_force_ok = force_mag is None or force_mag <= args.return_force_threshold
                release_speed_ok = joint_speed_mag is not None and joint_speed_mag <= args.return_release_speed

                if runtime_state == "drag" and args.return_enabled == 1:
                    if moved_far and release_speed_ok and release_force_ok:
                        if release_candidate_since is None:
                            release_candidate_since = time.monotonic()
                        elif time.monotonic() - release_candidate_since >= args.return_release_hold:
                            force_text = 'n/a' if force_mag is None else f'{force_mag:.2f}N'
                            print_step(
                                '检测到撤力，开始 ServoJ 回中 '
                                f'(joint_err={joint_err_deg:.2f}deg, speed={joint_speed_mag:.2f}deg/s, force={force_text})'
                            )
                            exit_joint_drag(robot, args)
                            drag_active = False
                            start_servo_return(robot, args)
                            servo_active = True
                            runtime_state = "return"
                            return_started_at = time.monotonic()
                            release_candidate_since = None
                    else:
                        release_candidate_since = None

                elif runtime_state == "return":
                    reacquire_ready = (
                        return_started_at is not None
                        and time.monotonic() - return_started_at >= args.return_reacquire_delay
                    )
                    reacquire_force_hit = force_mag is not None and force_mag >= args.return_reacquire_force
                    reacquire_speed_hit = joint_speed_mag is not None and joint_speed_mag >= args.return_reacquire_speed
                    if reacquire_ready and reacquire_force_hit and reacquire_speed_hit:
                        print_step(
                            f"检测到重新受力 {force_mag:.2f}N 且 joint_speed={joint_speed_mag:.2f}deg/s，停止回中并恢复拖动"
                        )
                        stop_servo_return(robot, args)
                        servo_active = False
                        enter_joint_drag(robot, args)
                        drag_active = True
                        runtime_state = "drag"
                        return_started_at = None
                        release_candidate_since = None
                    elif (
                        joint_err_deg is not None
                        and joint_err_deg <= args.return_joint_band
                        and (trans_err is None or trans_err <= args.return_translation_band)
                        and (rot_err is None or rot_err <= args.return_rotation_band)
                    ):
                        print_step("已回到参考位附近，恢复拖动")
                        stop_servo_return(robot, args)
                        servo_active = False
                        enter_joint_drag(robot, args)
                        drag_active = True
                        runtime_state = "drag"
                        return_started_at = None
                        release_candidate_since = None
                    else:
                        target_joint = build_return_joint_target(reference_joint, joint_pos, args)
                        require_zero(
                            f"ServoJ(cmdType={args.return_cmd_type})",
                            robot.ServoJ(
                                target_joint,
                                [0.0, 0.0, 0.0, 0.0],
                                0.0,
                                0.0,
                                args.return_cmdt,
                                0.0,
                                0.0,
                                0,
                                args.return_cmd_type,
                            ),
                        )
                        time.sleep(args.return_cmdt)

            if elapsed >= next_log_time:
                if active_mode == "torque_spring" and pose is not None and dz is not None and joint_err_deg is not None:
                    torque_text = 'n/a' if torque_delta_max is None else f'{torque_delta_max:.2f}Nm'
                    print_step(
                        f"state={runtime_state} "
                        f"t={elapsed:.1f}s "
                        f"dz={dz:.2f}mm "
                        f"tcp_z={pose[2]:.2f} "
                        f"joint_err={joint_err_deg:.2f}deg "
                        f"tau_delta_max={torque_text}"
                    )
                elif pose is not None and dz is not None:
                    force_text = 'n/a' if force_mag is None else f'{force_mag:.2f}N'
                    joint_err_text = 'n/a' if joint_err_deg is None else f'{joint_err_deg:.2f}deg'
                    speed_text = 'n/a' if joint_speed_mag is None else f'{joint_speed_mag:.2f}deg/s'
                    trans_text = 'n/a' if trans_err is None else f'{trans_err:.1f}mm'
                    rot_text = 'n/a' if rot_err is None else f'{rot_err:.1f}deg'
                    print_step(
                        f"state={runtime_state} "
                        f"t={elapsed:.1f}s "
                        f"dz={dz:.2f}mm "
                        f"err={trans_text}/{rot_text} "
                        f"joint_err={joint_err_text} "
                        f"joint_speed={speed_text} "
                        f"|F|={force_text} "
                        f"tcp_z={pose[2]:.2f}"
                    )
                next_log_time = elapsed + args.log_interval

            if active_mode == "torque_spring":
                time.sleep(args.torque_loop_interval)
            else:
                time.sleep(0.02)

        print_step(f"准备退出 {active_mode}/{runtime_state}")
        return 0

    except Exception as exc:
        print(f"[Spring] 错误: {exc}", file=sys.stderr, flush=True)
        return 1

    finally:
        if robot is not None and active_mode == "impedance":
            try:
                err = robot.ImpedanceControlStartStop(
                    0,
                    args.workspace,
                    args.force_threshold,
                    args.mass,
                    args.damping,
                    args.stiffness,
                    args.max_v,
                    args.max_va,
                    args.max_w,
                    args.max_wa,
                )
                if err != 0:
                    print(f"[Spring] ImpedanceControlStartStop(0) 返回 errcode={err}", file=sys.stderr)
                else:
                    print_step("阻抗控制已关闭")
            except Exception as exc:
                print(f"[Spring] 关闭阻抗控制异常: {exc}", file=sys.stderr)

        if robot is not None and servo_active:
            stop_servo_return(robot, args)

        if robot is not None and torque_spring_active:
            stop_torque_spring(robot, args)

        if robot is not None and drag_active:
            exit_joint_drag(robot, args)

        if robot is not None:
            try:
                robot.CloseRPC()
                print_step("RPC 连接已关闭")
            except Exception as exc:
                print(f"[Spring] CloseRPC 异常: {exc}", file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
