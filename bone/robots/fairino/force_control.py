"""
力控柔顺按摩模块：基于法奥 SDK 内置力传感器实现阻抗/恒力控制

硬件前提：六维力传感器(XJC-6F-D82)必须从 PC USB-RS485 改接到
          法奥机器人控制柜的 RS485 端口。

两种力控模式：
  - ft_control : FT_Control 恒力控制（推荐），Z轴力控 + XY位置跟踪
  - impedance  : ImpedanceControlStartStop 阻抗控制，6-DOF 质量-弹簧-阻尼
"""

import os
import sys
import time
import argparse
import threading
import math
from dataclasses import dataclass, field

# Robot SDK
try:
    from fairino import Robot
except Exception:
    try:
        from src.user_functions.fairino import Robot
    except Exception:
        import importlib
        _dir = os.path.dirname(__file__)
        if _dir not in sys.path:
            sys.path.insert(0, _dir)
        Robot = importlib.import_module("fairino").Robot

from dianjing import (
    _load_camera_to_robot_matrix,
    _transform_points,
    _to_mm_points,
    _load_points_prefer_current_calibration,
)

# ===================== 常量 =====================

FORCE_SENSOR_COMPANY = 24   # 鑫精诚 XJC
FORCE_SENSOR_DEVICE  = 0    # XJC-6F-D82
FORCE_SENSOR_BUS     = int(os.environ.get("FORCE_SENSOR_BUS", "1"))
SENSOR_ID            = 1
FORCE_ALLOW_SKIP_ZERO = os.environ.get("FORCE_ALLOW_SKIP_ZERO", "1").strip().lower() in {
    "1", "true", "yes", "on",
}
FORCE_ZERO_MAX_ABS_N = float(os.environ.get("FORCE_ZERO_MAX_ABS_N", "3.0"))

# MoveCart blendT：SDK 文档 -1=阻塞到位；0~500 为非阻塞(ms)。力控轨迹连发非阻塞易触发 err=112。
MOVE_CART_BLEND_BLOCKING = -1.0


# ===================== 配置数据类 =====================

@dataclass
class ForceControlConfig:
    """力控参数配置，所有可调参数集中管理。"""

    # --- 恒力控制 (FT_Control) ---
    target_force_z: float = 5.0       # 目标接触力 (N)，正值表示向下按压
    ft_pid: list = field(default_factory=lambda: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
    max_dis: float = 100.0            # 最大 Z 调整距离 (mm)
    max_ang: float = 5.0              # 最大角度调整 (deg)
    filter_sign: int = 1              # 滤波开关: 0-关, 1-开
    # M/B 参数
    ft_m: list = field(default_factory=lambda: [0.001, 0.001])
    ft_b: list = field(default_factory=lambda: [0.9, 0.9])

    # --- 阻抗控制 (ImpedanceControlStartStop) ---
    impedance_m: list = field(default_factory=lambda: [0.0, 0.0, 3.0, 0.0, 0.0, 0.0])
    impedance_b: list = field(default_factory=lambda: [0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
    impedance_k: list = field(default_factory=lambda: [0.0, 0.0, 200.0, 0.0, 0.0, 0.0])
    impedance_force_threshold: list = field(default_factory=lambda: [5.0, 5.0, 2.0, 1.0, 1.0, 1.0])
    impedance_max_v: float = 20.0
    impedance_max_va: float = 50.0
    impedance_max_w: float = 5.0
    impedance_max_wa: float = 10.0

    # --- 碰撞守护 (FT_Guard，与恒力按压共用传感器时易误触发，可按需关闭) ---
    enable_collision_guard: bool = True
    guard_force_limit: float = 30.0   # N
    guard_torque_limit: float = 3.0   # Nm

    # --- 表面探测 ---
    find_surface_force: float = 3.5   # 探面接触力阈值 (N)，需高于运动噪声
    find_surface_max_dis: float = 100.0  # 最大下探距离 (mm)
    find_surface_speed: float = 2.0   # 探面速度 (mm/s)，慢速减少振动误触发

    # --- 末端负载 (重力补偿) ---
    payload_weight: float = 0.0       # 末端负载质量 (kg)，默认不做额外负载补偿
    payload_cog: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # 质心 (mm)，传感器直接法兰安装时的临时近似

    # --- 安全 ---
    software_force_limit: float = 25.0  # 软件力监控阈值 (N)
    contact_loss_timeout: float = 0.5   # 失接触判定时间 (s)
    contact_loss_force_n: float = 1.5   # 低于该力值视为可能失接触 (N)

    # --- 力渐进 ---
    force_ramp_steps: list = field(default_factory=lambda: [2.0, 3.5, 5.0])
    force_ramp_dwell: float = 0.5     # 每步停留时间 (s)

    # --- 姿态贴合 (基于曲面斜率估计) ---
    orient_follow_enable: bool = True
    orient_max_tilt_deg: float = 5.0    # 最大倾斜角 (deg)
    orient_smooth_alpha: float = 0.25   # 平滑系数 (0-1, 越大越跟手)

    # --- 轨迹稳健性 ---
    cart_max_xy_step_mm: float = 8.0    # 相邻轨迹点最大 XY 步长
    disable_orient_on_retry: bool = True
    skip_unreachable_points: bool = True
    max_skip_points_per_pass: int = 6
    skip_after_failures: int = 2
    abort_fail_streak: int = 8


# ===================== 传感器初始化 =====================

def _set_force_sensor_config(robot):
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
            print(f"[Force] FT_SetConfig OK args={args}")
            return True
        print(f"[Force] FT_SetConfig{args} 失败, err={rtn}")
    return False


def _force_reading_available(robot):
    ret = robot.FT_GetForceTorqueRCS()
    return isinstance(ret, tuple) and ret[0] == 0 and len(ret) >= 2


def _read_force_sample(robot):
    ret = robot.FT_GetForceTorqueRCS()
    if isinstance(ret, tuple) and ret[0] == 0 and len(ret) >= 2:
        try:
            return [float(v) for v in ret[1][:6]]
        except Exception:
            return None
    return None

def init_force_sensor(robot, config: ForceControlConfig = None):
    """
    初始化法奥 SDK 内置力传感器。
    传感器必须连接到机器人控制柜 RS485 端口。

    返回 True 成功，False 失败。
    """
    if config is None:
        config = ForceControlConfig()

    print("[Force] 初始化力传感器...")

    # 1. 配置传感器类型
    if not _set_force_sensor_config(robot):
        return False

    # 2. 激活传感器
    rtn = robot.FT_Activate(0)
    if rtn != 0:
        print(f"[Force] FT_Activate(0) 警告, err={rtn}（继续尝试激活/读数）")
    time.sleep(0.3)
    rtn = robot.FT_Activate(1)
    if rtn != 0:
        if _force_reading_available(robot):
            print(f"[Force] FT_Activate(1) 警告, err={rtn}；但力传感器读数可用，继续")
        else:
            print(f"[Force] FT_Activate 失败, err={rtn}")
            return False
    else:
        print("[Force] FT_Activate OK")
    time.sleep(0.5)  # 等待传感器稳定

    # 3. 末端负载先置零（法奥 err=62：力矩传感器负载未设置为零，须先负载清零再 FT_SetZero）
    rtn = robot.SetForceSensorPayload(0.0)
    if rtn != 0:
        print(f"[Force] SetForceSensorPayload(0) 失败, err={rtn}（请检查示教器力传感器/负载相关设置）")
        return False
    rtn = robot.SetForceSensorPayloadCog(0.0, 0.0, 0.0)
    if rtn != 0:
        print(f"[Force] SetForceSensorPayloadCog(0,0,0) 失败, err={rtn}")
        return False
    print("[Force] 负载参数已置零，准备传感器校零")

    # 4. 零点校准：官方示例顺序 FT_SetZero(0) → FT_SetZero(1)；机器人静止、无外部接触
    zero_retries = 5
    zero_wait_s = 0.6
    rtn = robot.FT_SetZero(0)
    if rtn != 0:
        print(f"[Force] FT_SetZero(0) 失败, err={rtn}（部分固件可忽略，继续尝试）")
    time.sleep(0.3)

    rtn = -1
    for attempt in range(1, zero_retries + 1):
        time.sleep(zero_wait_s)
        rtn = robot.FT_SetZero(1)
        if rtn == 0:
            print(f"[Force] FT_SetZero(1) OK (attempt {attempt}/{zero_retries})")
            break
        print(f"[Force] FT_SetZero(1) 失败, err={rtn} (attempt {attempt}/{zero_retries})")
    if rtn != 0:
        data = _read_force_sample(robot)
        if data is not None and FORCE_ALLOW_SKIP_ZERO:
            max_force = max(abs(data[0]), abs(data[1]), abs(data[2]))
            if max_force <= FORCE_ZERO_MAX_ABS_N:
                print(
                    "[Force] FT_SetZero 失败但当前读数接近零点，继续执行 "
                    f"(Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N, "
                    f"threshold={FORCE_ZERO_MAX_ABS_N:.1f}N)"
                )
            else:
                print(
                    "[Force] FT_SetZero 失败且当前力偏置过大，停止 "
                    f"(Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N)"
                )
                return False
        else:
            print("[Force] 若持续 err=62：请在示教器完成力传感器负载清零，末端悬空无接触后再试。")
            return False

    # 5. 设置参考坐标系为工具坐标系
    rtn = robot.FT_SetRCS(ref=0)
    if rtn != 0:
        if _force_reading_available(robot):
            print(f"[Force] FT_SetRCS 警告, err={rtn}；但力传感器读数可用，继续")
        else:
            print(f"[Force] FT_SetRCS 失败, err={rtn}")
            return False
    else:
        print("[Force] FT_SetRCS OK (工具坐标系)")

    # 6. 写入实际末端负载 (重力补偿)
    rtn = robot.SetForceSensorPayload(config.payload_weight)
    if rtn != 0:
        print(f"[Force] SetForceSensorPayload 失败, err={rtn}")
        return False
    print(f"[Force] SetForceSensorPayload OK (weight={config.payload_weight} kg)")

    # 7. 负载质心
    cx, cy, cz = config.payload_cog
    rtn = robot.SetForceSensorPayloadCog(cx, cy, cz)
    if rtn != 0:
        print(f"[Force] SetForceSensorPayloadCog 失败, err={rtn}")
        return False
    print(f"[Force] SetForceSensorPayloadCog OK ({cx}, {cy}, {cz})")

    # 8. 验证读数
    ret = robot.FT_GetForceTorqueRCS()
    if isinstance(ret, tuple) and ret[0] == 0:
        data = ret[1]
        print(f"[Force] 当前力读数: Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f} "
              f"Mx={data[3]:.3f} My={data[4]:.3f} Mz={data[5]:.3f}")
    else:
        print(f"[Force] 警告: FT_GetForceTorqueRCS 返回异常: {ret}")

    print("[Force] 传感器初始化完成")
    return True


# ===================== 碰撞守护 =====================

def setup_collision_guard(robot, config: ForceControlConfig = None):
    """开启碰撞守护，超过阈值机器人急停。"""
    if config is None:
        config = ForceControlConfig()

    fl = config.guard_force_limit
    tl = config.guard_torque_limit
    rtn = robot.FT_Guard(
        flag=1,
        sensor_num=SENSOR_ID,
        select=[1, 1, 1, 1, 1, 1],
        force_torque=[0, 0, 0, 0, 0, 0],
        max_threshold=[fl, fl, fl, tl, tl, tl],
        min_threshold=[fl, fl, fl, tl, tl, tl],
    )
    if rtn != 0:
        print(f"[Force] FT_Guard 开启失败, err={rtn}")
        return False
    print(f"[Force] 碰撞守护已开启 (力阈值={fl}N, 力矩阈值={tl}Nm)")
    return True


def disable_collision_guard(robot):
    """关闭碰撞守护。"""
    rtn = robot.FT_Guard(
        flag=0,
        sensor_num=SENSOR_ID,
        select=[0, 0, 0, 0, 0, 0],
        force_torque=[0, 0, 0, 0, 0, 0],
        max_threshold=[0, 0, 0, 0, 0, 0],
        min_threshold=[0, 0, 0, 0, 0, 0],
    )
    if rtn != 0:
        print(f"[Force] FT_Guard 关闭失败, err={rtn}")
    else:
        print("[Force] 碰撞守护已关闭")
    return rtn


# ===================== 表面探测 =====================

def find_surface(robot, config: ForceControlConfig = None):
    """
    沿工具坐标系 Z 正方向慢速下探，直到接触力达到阈值。
    返回 True/False。
    """
    if config is None:
        config = ForceControlConfig()

    print(f"[Force] 表面探测: 最大距离={config.find_surface_max_dis}mm, "
          f"接触力={config.find_surface_force}N, 速度={config.find_surface_speed}mm/s")

    rtn = robot.FT_FindSurface(
        rcs=0,      # 工具坐标系
        dir=1,      # 正方向
        axis=3,     # Z 轴
        disMax=config.find_surface_max_dis,
        ft=config.find_surface_force,
        lin_v=config.find_surface_speed,
    )
    if rtn != 0:
        print(f"[Force] FT_FindSurface 失败, err={rtn}")
        return False

    # 读取接触位置
    ret = robot.GetActualTCPPose()
    if isinstance(ret, tuple) and ret[0] == 0:
        pos = ret[1]
        print(f"[Force] 表面接触位置: X={pos[0]:.1f} Y={pos[1]:.1f} Z={pos[2]:.1f}")
    print("[Force] 表面探测完成")
    return True


# ===================== 力控启停 =====================

def start_force_control(robot, mode: str, config: ForceControlConfig = None):
    """
    启动力控制。

    mode: "ft_control" - 恒力控制 (推荐)
          "impedance"  - 阻抗控制
    """
    if config is None:
        config = ForceControlConfig()

    if mode == "ft_control":
        # 恒力控制: 仅 Z 轴力控，非阻塞
        target_fz = -abs(config.target_force_z)  # 负值 = 向下按压
        rtn = robot.FT_Control(
            flag=1,
            sensor_id=SENSOR_ID,
            select=[0, 0, 1, 0, 0, 0],           # 仅 Z 轴力控
            ft=[0, 0, target_fz, 0, 0, 0],
            ft_pid=config.ft_pid,
            adj_sign=0,
            ILC_sign=0,
            max_dis=config.max_dis,
            max_ang=config.max_ang,
            M=config.ft_m,
            B=config.ft_b,
            filter_Sign=config.filter_sign,
            posAdapt_sign=0,
            isNoBlock=1,    # 非阻塞，允许同时执行轨迹运动
        )
        if rtn != 0:
            print(f"[Force] FT_Control 启动失败, err={rtn}")
            return False
        print(f"[Force] 恒力控制已启动 (目标力={target_fz}N, "
              f"PID_P={config.ft_pid[0]}, max_dis={config.max_dis}mm)")

    elif mode == "impedance":
        rtn = robot.ImpedanceControlStartStop(
            status=1,
            workSpace=1,    # 笛卡尔空间
            forceThreshold=config.impedance_force_threshold,
            m=config.impedance_m,
            b=config.impedance_b,
            k=config.impedance_k,
            maxV=config.impedance_max_v,
            maxVA=config.impedance_max_va,
            maxW=config.impedance_max_w,
            maxWA=config.impedance_max_wa,
        )
        if rtn != 0:
            print(f"[Force] ImpedanceControl 启动失败, err={rtn}")
            return False
        print(f"[Force] 阻抗控制已启动 (m_z={config.impedance_m[2]}, "
              f"b_z={config.impedance_b[2]}, k_z={config.impedance_k[2]})")
    else:
        print(f"[Force] 未知力控模式: {mode}")
        return False

    return True


def stop_force_control(robot, mode: str):
    """停止力控制。"""
    if mode == "ft_control":
        rtn = robot.FT_Control(
            flag=0,
            sensor_id=SENSOR_ID,
            select=[0, 0, 0, 0, 0, 0],
            ft=[0, 0, 0, 0, 0, 0],
            ft_pid=[0, 0, 0, 0, 0, 0],
            adj_sign=0,
            ILC_sign=0,
            max_dis=0,
            max_ang=0,
        )
        if rtn != 0:
            print(f"[Force] FT_Control 停止失败, err={rtn}")
        else:
            print("[Force] 恒力控制已停止")
    elif mode == "impedance":
        rtn = robot.ImpedanceControlStartStop(
            status=0,
            workSpace=1,
            forceThreshold=[0, 0, 0, 0, 0, 0],
            m=[0, 0, 0, 0, 0, 0],
            b=[0, 0, 0, 0, 0, 0],
            k=[0, 0, 0, 0, 0, 0],
            maxV=0, maxVA=0, maxW=0, maxWA=0,
        )
        if rtn != 0:
            print(f"[Force] ImpedanceControl 停止失败, err={rtn}")
        else:
            print("[Force] 阻抗控制已停止")

    # 关闭碰撞守护
    disable_collision_guard(robot)


# ===================== 力读数辅助 =====================

def get_force_z(robot):
    """读取 Z 轴力，返回 float 或 None。"""
    ret = robot.FT_GetForceTorqueRCS()
    if isinstance(ret, tuple) and ret[0] == 0:
        return ret[1][2]
    return None


def get_force_torque_all(robot):
    """读取完整六轴力/力矩 [fx, fy, fz, mx, my, mz]，失败返回 None。"""
    ret = robot.FT_GetForceTorqueRCS()
    if isinstance(ret, tuple) and ret[0] == 0:
        return ret[1]
    return None


def get_actual_tcp_pose(robot):
    """读取当前 TCP 位姿 [x, y, z, rx, ry, rz]，失败返回 None。"""
    ret = robot.GetActualTCPPose()
    if isinstance(ret, tuple) and ret[0] == 0:
        return ret[1]
    return None


def _densify_path_xy(points, max_xy_step_mm: float):
    """在相邻点之间按 XY 直线细分，降低单步 MoveCart 难度。"""
    if not points or max_xy_step_mm <= 0:
        return [list(map(float, p)) for p in points]
    out = [list(map(float, points[0]))]
    for i in range(1, len(points)):
        p0 = out[-1]
        p1 = list(map(float, points[i]))
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dz = p1[2] - p0[2]
        dist_xy = math.hypot(dx, dy)
        if dist_xy < 1e-6:
            out.append(p1)
            continue
        n_seg = max(1, int(math.ceil(dist_xy / max_xy_step_mm)))
        for s in range(1, n_seg):
            t = s / n_seg
            out.append([p0[0] + dx * t, p0[1] + dy * t, p0[2] + dz * t])
        out.append(p1)
    return out


def _tool_z_unit_from_rpy(rx_deg, ry_deg, rz_deg):
    """
    由欧拉角(度)计算工具坐标系 Z 轴在基坐标系下的单位向量。
    采用常见 Z-Y-X 组合: R = Rz * Ry * Rx。
    """
    rx = math.radians(float(rx_deg))
    ry = math.radians(float(ry_deg))
    rz = math.radians(float(rz_deg))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    # R 的第三列即工具 Z 轴在基坐标中的方向
    zx = cz * sy * cx + sz * sx
    zy = sz * sy * cx - cz * sx
    zz = cy * cx
    norm = math.sqrt(zx * zx + zy * zy + zz * zz)
    if norm < 1e-9:
        return [0.0, 0.0, 1.0]
    return [zx / norm, zy / norm, zz / norm]


def _apply_tool_z_offset(base_pose, tool_z_unit, offset_mm):
    """在保持姿态不变的前提下，沿工具 Z 轴偏移 offset_mm。"""
    return [
        float(base_pose[0]) + tool_z_unit[0] * float(offset_mm),
        float(base_pose[1]) + tool_z_unit[1] * float(offset_mm),
        float(base_pose[2]) + tool_z_unit[2] * float(offset_mm),
        float(base_pose[3]),
        float(base_pose[4]),
        float(base_pose[5]),
    ]


# ===================== 力控轨迹执行 Worker =====================

def force_controlled_worker(
    state,
    json_path: str,
    sides: list = None,
    robot_ip: str = "192.168.58.2",
    speed: int = 8,
    tool: int = 0,
    user: int = 0,
    rx: float = -178.190,
    ry: float = 1.724,
    rz: float = -1.187,
    approach_height_mm: float = 50.0,
    sample_step: int = 1,
    passes: int = 1,
    force_mode: str = "ft_control",
    target_force_n: float = 5.0,
    config: ForceControlConfig = None,
):
    """
    力控版轨迹执行 worker，替代 demo.py 中的 _robot_worker。

    核心设计：FT_Control (固件 kHz 闭环) 为唯一 Z 控制器。
    软件层只负责 XY 轨迹 + 监控，不调 Z。
    """
    if sides is None:
        sides = ["left", "right"]
    if config is None:
        config = ForceControlConfig()
    config.target_force_z = target_force_n
    soft_margin = 10.0 if config.enable_collision_guard else 22.0
    config.software_force_limit = max(
        float(config.software_force_limit), abs(float(target_force_n)) + soft_margin
    )
    if config.enable_collision_guard:
        config.guard_force_limit = max(
            float(config.guard_force_limit), abs(float(target_force_n)) + 20.0
        )

    SAFE_Z_MM = 300.0

    robot = None

    try:
        state.status = "running"
        state.error_msg = ""

        # --- 连接 ---
        robot = Robot.RPC(robot_ip)
        TRANSIT_SPEED = 50   # 准备阶段高速
        TRAJ_SPEED = speed   # 轨迹阶段低速(柔顺)
        robot.SetSpeed(TRANSIT_SPEED)

        def _fmt_pose(pose6):
            return (
                f"({float(pose6[0]):.1f}, {float(pose6[1]):.1f}, {float(pose6[2]):.1f}, "
                f"{float(pose6[3]):.1f}, {float(pose6[4]):.1f}, {float(pose6[5]):.1f})"
            )

        def _print_move_diag(tag, target6, actual6=None, err=None):
            msg = f"[Force][Diag] {tag} target={_fmt_pose(target6)} tool={tool} user={user}"
            if err is not None:
                msg += f" err={err}"
            print(msg)
            if actual6 is not None and len(actual6) >= 6:
                dx = float(target6[0]) - float(actual6[0])
                dy = float(target6[1]) - float(actual6[1])
                dz = float(target6[2]) - float(actual6[2])
                print(
                    f"[Force][Diag] {tag} actual={_fmt_pose(actual6)} "
                    f"delta=(dx={dx:+.1f}, dy={dy:+.1f}, dz={dz:+.1f})"
                )

        def _segmented_move_to_pose(target6, max_step_mm=6.0, max_steps=80):
            """从当前 TCP 分段笛卡尔逼近目标，缓解单次大步 MoveCart 的 IK/规划失败 (如 err=112)。"""
            for _ in range(max_steps):
                ap = get_actual_tcp_pose(robot)
                if ap is None or len(ap) < 6:
                    return False
                dx = float(target6[0]) - float(ap[0])
                dy = float(target6[1]) - float(ap[1])
                dz = float(target6[2]) - float(ap[2])
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < 0.4:
                    r = robot.MoveCart(
                        desc_pos=list(target6),
                        tool=tool,
                        user=user,
                        blendT=MOVE_CART_BLEND_BLOCKING,
                    )
                    return r == 0
                scale = min(1.0, max_step_mm / dist)
                wp = [
                    float(ap[0]) + dx * scale,
                    float(ap[1]) + dy * scale,
                    float(ap[2]) + dz * scale,
                    float(target6[3]),
                    float(target6[4]),
                    float(target6[5]),
                ]
                r = robot.MoveCart(
                    desc_pos=wp, tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING
                )
                if r != 0:
                    return False
            return False

        def _move_pose_with_fallback(tag, target6, prefer_segmented=False, max_step_mm=6.0):
            actual = get_actual_tcp_pose(robot)
            _print_move_diag(tag, target6, actual6=actual)

            if not prefer_segmented:
                rtn = robot.MoveCart(
                    desc_pos=list(target6),
                    tool=tool,
                    user=user,
                    blendT=MOVE_CART_BLEND_BLOCKING,
                )
                if rtn == 0:
                    return True
                actual = get_actual_tcp_pose(robot)
                _print_move_diag(tag, target6, actual6=actual, err=rtn)
                if rtn not in (14, 112):
                    return False

            if _segmented_move_to_pose(target6, max_step_mm=max_step_mm):
                print(f"[Force][Retry] {tag}: segmented move OK")
                return True

            actual = get_actual_tcp_pose(robot)
            if actual is not None and len(actual) >= 6:
                fallback_pose = [
                    float(target6[0]),
                    float(target6[1]),
                    float(target6[2]),
                    float(actual[3]),
                    float(actual[4]),
                    float(actual[5]),
                ]
                if any(abs(float(fallback_pose[i]) - float(target6[i])) > 0.1 for i in range(3, 6)):
                    print(
                        f"[Force][Retry] {tag}: retry with current TCP orientation "
                        f"{_fmt_pose(fallback_pose)}"
                    )
                    if _segmented_move_to_pose(fallback_pose, max_step_mm=max_step_mm):
                        print(f"[Force][Retry] {tag}: current-orientation segmented move OK")
                        return True

            return False

        def _stopped():
            return state.stop_event.is_set()

        # --- 传感器初始化 ---
        if not init_force_sensor(robot, config):
            raise RuntimeError("力传感器初始化失败")

        # --- 碰撞守护 ---
        if config.enable_collision_guard:
            if not setup_collision_guard(robot, config):
                raise RuntimeError("碰撞守护设置失败")
        else:
            print("[Force] 碰撞守护已关闭 (enable_collision_guard=False)，请注意按压安全")

        total_pass_count = len(sides) * passes
        state.pass_total = total_pass_count
        global_pass = 0

        for side_idx, side in enumerate(sides):
            if _stopped():
                raise InterruptedError

            print(f"\n[Force] ========== Side {side_idx+1}/{len(sides)}: {side} ==========")

            # --- 加载轨迹 ---
            try:
                points, frame = _load_points_prefer_current_calibration(
                    json_path, side=side, prefer_camera_retransform=True
                )
            except Exception as e:
                print(f"[Force] Skip {side}: {e}")
                global_pass += passes
                continue

            print(f"[Force] Loaded {len(points)} pts (frame={frame})")

            if frame == "camera":
                T4 = _load_camera_to_robot_matrix()
                if T4 is None:
                    raise RuntimeError("camera_to_robot.json not found")
                points = _transform_points(points, T4)

            points_mm, scale = _to_mm_points(points)

            # 线性化轨迹（仅 XY，保留原始 Z 作为引导高度）
            from demo import _linearize_trajectory
            points_mm = _linearize_trajectory(points_mm)

            raw_zs = [p[2] for p in points_mm]
            print(f"[Force] Z range: [{min(raw_zs):.1f}, {max(raw_zs):.1f}] mm "
                  f"(delta={max(raw_zs)-min(raw_zs):.1f}mm)")
            guide_z = max(raw_zs)

            sampled = points_mm[:: max(1, int(sample_step))]
            if sampled[-1] != points_mm[-1]:
                sampled.append(points_mm[-1])
            sampled0 = len(sampled)
            sampled = _densify_path_xy(sampled, config.cart_max_xy_step_mm)
            if len(sampled) != sampled0:
                print(
                    f"[Force] 轨迹加密: {sampled0} -> {len(sampled)} pts "
                    f"(max_xy_step={config.cart_max_xy_step_mm:.1f}mm)"
                )

            xs = [p[0] for p in sampled]
            ys = [p[1] for p in sampled]
            print(f"[Force] Trajectory: {len(sampled)} pts")
            print(f"[Force]   X: [{min(xs):.1f}, {max(xs):.1f}]")
            print(f"[Force]   Y: [{min(ys):.1f}, {max(ys):.1f}]")
            print(f"[Force]   Guide Z: {guide_z:.1f} mm")

            state.total = len(sampled)

            # === Step 0: 垂直抬起到安全高度 ===
            if _stopped():
                raise InterruptedError
            cur = get_actual_tcp_pose(robot)
            transit_rx, transit_ry, transit_rz = rx, ry, rz
            if cur is not None and len(cur) >= 6:
                cur_z = float(cur[2])
                transit_rx, transit_ry, transit_rz = float(cur[3]), float(cur[4]), float(cur[5])
                if cur_z < SAFE_Z_MM:
                    up_pose = [float(cur[0]), float(cur[1]), SAFE_Z_MM,
                               float(cur[3]), float(cur[4]), float(cur[5])]
                    print(f"[Force] Lifting from Z={cur_z:.1f} to Z={SAFE_Z_MM}")
                    if not _move_pose_with_fallback("lift-to-safe-z", up_pose, prefer_segmented=False, max_step_mm=8.0):
                        raise RuntimeError("MoveCart lift err during transit preparation")

            demo_return_mode = (passes == 1 and len(sides) == 2 and side_idx == 1)
            first_path = list(reversed(sampled)) if demo_return_mode else sampled

            # === Step 1: 水平移到轨迹起点上方 ===
            if _stopped():
                raise InterruptedError
            first = first_path[0]
            safe_pose = [first[0], first[1], SAFE_Z_MM, transit_rx, transit_ry, transit_rz]
            print(f"[Force] Transit to {side} start: {safe_pose[:3]}")
            if not _move_pose_with_fallback("transit-to-start", safe_pose, prefer_segmented=True, max_step_mm=8.0):
                raise RuntimeError(
                    "MoveCart transit err=112/14; target start pose unreachable. "
                    "Check camera_to_robot calibration or tool/user frame."
                )

            # === Step 1.5: FT_SetZero 在 Z=300 (保证空中) ===
            print(f"[Force] FT_SetZero at Z={SAFE_Z_MM} (in-air)")
            robot.FT_SetZero(1)
            time.sleep(0.3)
            fz_check = get_force_z(robot)
            print(f"[Force] Post-zero Fz={fz_check:.2f}N" if fz_check is not None else "[Force] Post-zero Fz=?")

            # === Step 2: 下降到估计表面上方 approach_height_mm ===
            if _stopped():
                raise InterruptedError
            approach_z = guide_z + approach_height_mm
            approach_pose = [first[0], first[1], approach_z, transit_rx, transit_ry, transit_rz]
            print(f"[Force] Descend to approach Z={approach_z:.1f}")
            if not _move_pose_with_fallback("descend-to-approach", approach_pose, prefer_segmented=False, max_step_mm=5.0):
                raise RuntimeError("MoveCart approach err=112/14")

            # === Step 3: 直接从接近高度开始PI力控收敛 ===
            if _stopped():
                raise InterruptedError
            contact_pose = approach_pose
            contact_z = approach_z

            # 切换到轨迹低速（柔顺）
            robot.SetSpeed(TRAJ_SPEED)

            # === Step 5+6: 软件PI力控 + 姿态贴合轨迹执行 ===
            # Z 由 Fz 反馈闭环调节，rx/ry 由曲面斜率估计调节
            # XY 沿轨迹走，实现完全贴合曲面

            CTRL_KP = 0.5         # mm/N
            CTRL_KI = 0.03
            CTRL_IMAX = 20.0
            CTRL_MAX_STEP = 3.0   # mm

            ctrl_z = contact_z
            ctrl_err_i = 0.0

            # 姿态贴合: 从相邻点Z变化估计曲面法线方向
            ctrl_rx = rx
            ctrl_ry = ry
            ORI_MAX = config.orient_max_tilt_deg
            ORI_ALPHA = config.orient_smooth_alpha
            prev_xyz = None
            ctrl_z_floor = guide_z - 20.0

            def _ctrl_step(fz_val, target_n):
                """PI 力控单步，返回 Z 调整量 (mm)。fz 负=按压。"""
                nonlocal ctrl_err_i
                target_fz = -abs(target_n)
                err = target_fz - fz_val
                ctrl_err_i += err
                ctrl_err_i = max(-CTRL_IMAX, min(CTRL_IMAX, ctrl_err_i))
                dz = CTRL_KP * err + CTRL_KI * ctrl_err_i
                return max(-CTRL_MAX_STEP, min(CTRL_MAX_STEP, dz))

            def _update_orientation(cur_x, cur_y, cur_z):
                """从相邻点Z差异计算曲面倾斜，更新ctrl_rx/ctrl_ry。"""
                nonlocal ctrl_rx, ctrl_ry, prev_xyz
                if prev_xyz is None:
                    prev_xyz = (cur_x, cur_y, cur_z)
                    return
                dx = cur_x - prev_xyz[0]
                dy = cur_y - prev_xyz[1]
                dz = cur_z - prev_xyz[2]
                dist_xy = math.sqrt(dx * dx + dy * dy)
                prev_xyz = (cur_x, cur_y, cur_z)
                if dist_xy < 0.5:
                    return

                slope_x = dz / dist_xy * (dx / dist_xy)
                slope_y = dz / dist_xy * (dy / dist_xy)

                # rx≈-180° 时工具Z指向-Z，增ry→toolZ偏-X，增rx→toolZ偏+Y
                target_dry = math.degrees(math.atan(-slope_x))
                target_drx = math.degrees(math.atan(slope_y))

                target_dry = max(-ORI_MAX, min(ORI_MAX, target_dry))
                target_drx = max(-ORI_MAX, min(ORI_MAX, target_drx))

                cur_drx = ctrl_rx - rx
                cur_dry = ctrl_ry - ry
                new_drx = cur_drx + ORI_ALPHA * (target_drx - cur_drx)
                new_dry = cur_dry + ORI_ALPHA * (target_dry - cur_dry)

                ctrl_rx = rx + new_drx
                ctrl_ry = ry + new_dry

            def _settle_force(target_n, xy, max_iter=30, tol_n=1.0):
                """在当前XY位置反复调Z直到力收敛到目标±tol_n。"""
                nonlocal ctrl_z, ctrl_err_i
                for it in range(max_iter):
                    fz = get_force_z(robot)
                    if fz is None:
                        time.sleep(0.05)
                        continue
                    state.force_z = fz
                    err = abs(fz) - abs(target_n)
                    if abs(err) < tol_n and abs(fz) > 0.5:
                        return True
                    ctrl_z += _ctrl_step(fz, target_n)
                    if ctrl_z < ctrl_z_floor:
                        ctrl_z = ctrl_z_floor
                        ctrl_err_i = 0.0
                    pose = [xy[0], xy[1], ctrl_z, ctrl_rx, ctrl_ry, rz]
                    robot.MoveCart(desc_pos=pose, tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING)
                    time.sleep(0.08)
                return False

            # 收敛到目标力
            ramp_xy = [float(contact_pose[0]), float(contact_pose[1])]
            target_n = config.target_force_z
            print(f"[Force] 收敛到目标力 {target_n}N ...")
            ok = _settle_force(target_n, ramp_xy, max_iter=50, tol_n=1.5)
            fz_now = get_force_z(robot)
            fz_s = f"{fz_now:.2f}" if fz_now is not None else "?"
            status = "OK" if ok else "未收敛"
            print(f"[Force] 目标力 {target_n}N → fz={fz_s}N, Z={ctrl_z:.1f} ({status})")
            print(f"[Force] 软件PI就绪 (target={target_n}N, Z={ctrl_z:.1f}mm)")

            last_path = first_path
            for p_idx in range(passes):
                if _stopped():
                    raise InterruptedError
                global_pass += 1
                state.pass_cur = global_pass
                state.progress = 0
                reverse_path = demo_return_mode if p_idx % 2 == 0 else not demo_return_mode
                path = list(reversed(sampled)) if reverse_path else sampled
                direction = "rev(tail->neck)" if reverse_path else "fwd(neck->tail)"
                last_path = path
                print(f"[Force] Pass {global_pass}/{total_pass_count} "
                      f"({side} {direction}), {len(path)} pts")

                ctrl_err_i = 0.0
                prev_xyz = None
                ctrl_rx = rx
                ctrl_ry = ry
                pass_orient_follow = config.orient_follow_enable
                skipped_points = 0

                start_pt = path[0]
                print(f"[Force] Pass {global_pass} 起点力收敛...")
                ok = _settle_force(target_n, [start_pt[0], start_pt[1]],
                                   max_iter=20, tol_n=1.5)
                fz_chk = get_force_z(robot)
                fz_s = f"{fz_chk:.2f}" if fz_chk is not None else "?"
                print(f"[Force] Pass {global_pass} 起点: Z={ctrl_z:.1f} fz={fz_s}N "
                      f"({'OK' if ok else '未收敛'})")

                move_fail_streak = 0

                for i, p in enumerate(path):
                    if _stopped():
                        raise InterruptedError
                    state.progress = i + 1

                    pose = [p[0], p[1], ctrl_z, ctrl_rx, ctrl_ry, rz]
                    rtn = robot.MoveCart(desc_pos=pose, tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING)
                    if rtn != 0:
                        actual_pose = get_actual_tcp_pose(robot)
                        drx = ctrl_rx - rx
                        dry = ctrl_ry - ry
                        retried = False
                        if actual_pose is not None and len(actual_pose) >= 6:
                            dx_err = pose[0] - actual_pose[0]
                            dy_err = pose[1] - actual_pose[1]
                            dz_err = pose[2] - actual_pose[2]
                            print(f"[Force] MoveCart err={rtn} at [{i+1}/{len(path)}] "
                                  f"Z={ctrl_z:.1f} (streak={move_fail_streak + 1})")
                            print(
                                "[Force][Diag] target="
                                f"({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}, "
                                f"{pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f}) "
                                "actual="
                                f"({actual_pose[0]:.1f}, {actual_pose[1]:.1f}, {actual_pose[2]:.1f}, "
                                f"{actual_pose[3]:.1f}, {actual_pose[4]:.1f}, {actual_pose[5]:.1f})"
                            )
                            print(
                                "[Force][Diag] delta="
                                f"(dx={dx_err:+.1f}, dy={dy_err:+.1f}, dz={dz_err:+.1f}) "
                                f"tilt=(drx={drx:+.1f}°, dry={dry:+.1f}°)"
                            )

                            # 对局部不可达/插补失败做一次半步过渡重试。
                            if rtn in (14, 112) and abs(dx_err) < 40.0 and abs(dy_err) < 40.0:
                                mid_pose = [
                                    (pose[0] + actual_pose[0]) * 0.5,
                                    (pose[1] + actual_pose[1]) * 0.5,
                                    pose[2],
                                    pose[3],
                                    pose[4],
                                    pose[5],
                                ]
                                mid_rtn = robot.MoveCart(
                                    desc_pos=mid_pose, tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING
                                )
                                if mid_rtn == 0:
                                    time.sleep(0.05)
                                    retry_rtn = robot.MoveCart(
                                        desc_pos=pose, tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING
                                    )
                                    retried = (retry_rtn == 0)
                                    if retried:
                                        move_fail_streak = 0
                                        print(f"[Force][Retry] half-step recovery OK at [{i+1}/{len(path)}]")
                                    else:
                                        print(f"[Force][Retry] target retry err={retry_rtn} at [{i+1}/{len(path)}]")
                                else:
                                    print(f"[Force][Retry] mid-step err={mid_rtn} at [{i+1}/{len(path)}]")
                            if not retried and rtn in (14, 112):
                                if _segmented_move_to_pose(pose):
                                    retried = True
                                    move_fail_streak = 0
                                    print(f"[Force][Retry] segmented move OK at [{i+1}/{len(path)}]")
                            if (
                                not retried
                                and rtn in (14, 112)
                                and config.disable_orient_on_retry
                                and pass_orient_follow
                            ):
                                flat_pose = [pose[0], pose[1], pose[2], rx, ry, rz]
                                if _segmented_move_to_pose(flat_pose):
                                    retried = True
                                    move_fail_streak = 0
                                    ctrl_rx = rx
                                    ctrl_ry = ry
                                    prev_xyz = None
                                    pass_orient_follow = False
                                    print(
                                        f"[Force][Fallback] orientation follow disabled at [{i+1}/{len(path)}]"
                                    )
                        else:
                            print(f"[Force] MoveCart err={rtn} at [{i+1}/{len(path)}] "
                                  f"Z={ctrl_z:.1f} (streak={move_fail_streak + 1})")
                            print(
                                "[Force][Diag] actual pose unavailable, "
                                f"target=({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}) "
                                f"tilt=(drx={drx:+.1f}°, dry={dry:+.1f}°)"
                            )

                        if retried:
                            pass
                        else:
                            move_fail_streak += 1
                            if (
                                config.skip_unreachable_points
                                and rtn in (14, 112)
                                and move_fail_streak >= config.skip_after_failures
                                and skipped_points < config.max_skip_points_per_pass
                            ):
                                skipped_points += 1
                                move_fail_streak = 0
                                print(
                                    f"[Force][Fallback] skip unreachable point [{i+1}/{len(path)}] "
                                    f"(skipped={skipped_points}/{config.max_skip_points_per_pass})"
                                )
                                continue
                        if move_fail_streak >= config.abort_fail_streak:
                            print(f"[Force] 连续 {config.abort_fail_streak} 次 MoveCart 失败，终止当前 pass")
                            break
                    else:
                        move_fail_streak = 0
                    time.sleep(0.05)

                    for _ in range(5):
                        fz = get_force_z(robot)
                        if fz is None:
                            break
                        state.force_z = fz
                        dz = _ctrl_step(fz, target_n)
                        if abs(dz) < 0.05:
                            break
                        ctrl_z += dz
                        if ctrl_z < ctrl_z_floor:
                            ctrl_z = ctrl_z_floor
                            ctrl_err_i = 0.0
                        adj_pose = [p[0], p[1], ctrl_z, ctrl_rx, ctrl_ry, rz]
                        robot.MoveCart(desc_pos=adj_pose, tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING)
                        time.sleep(0.05)

                    if pass_orient_follow:
                        _update_orientation(p[0], p[1], ctrl_z)

                    if (i + 1) % 4 == 0 or (i + 1) == len(path):
                        fz_s = f"{state.force_z:.2f}" if hasattr(state, 'force_z') else "?"
                        drx = ctrl_rx - rx
                        dry = ctrl_ry - ry
                        print(
                            f"[Force]   [{i+1}/{len(path)}] "
                            f"Z={ctrl_z:.1f} fz={fz_s}N "
                            f"drx={drx:+.1f}° dry={dry:+.1f}°"
                        )

                    fz = get_force_z(robot)
                    if fz is not None:
                        if abs(fz) > config.software_force_limit:
                            print(f"[Force] 安全停止: |Fz|={abs(fz):.1f}N > {config.software_force_limit}N")
                            robot.StopMotion()
                            raise RuntimeError(f"力超限 |Fz|={abs(fz):.1f}N")

                print(f"[Force] Pass {global_pass} done ({side})")

            # === 该侧完成，切回高速 ===
            robot.SetSpeed(TRANSIT_SPEED)
            time.sleep(0.2)

            # 抬升到安全高度
            if _stopped():
                raise InterruptedError
            last = last_path[-1]
            lift_pose = [last[0], last[1], SAFE_Z_MM, rx, ry, rz]
            print(f"[Force] {side} done, lifting to Z={SAFE_Z_MM}")
            robot.MoveCart(desc_pos=lift_pose, tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING)

        # --- 所有侧完成 ---
        robot.FT_Activate(0)
        robot.CloseRPC()
        state.status = "done"
        print(f"\n[Force] All {len(sides)} sides complete ({total_pass_count} passes total)")

    except InterruptedError:
        state.status = "idle"
        print("[Force] Stopped by user")
        if robot:
            try:
                robot.StopMotion()
                ret = robot.GetActualTCPPose()
                if isinstance(ret, tuple) and ret[0] == 0:
                    cur = ret[1]
                    robot.MoveCart(
                        desc_pos=[float(cur[0]), float(cur[1]), SAFE_Z_MM,
                                  float(cur[3]), float(cur[4]), float(cur[5])],
                        tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING,
                    )
                robot.FT_Activate(0)
                robot.CloseRPC()
            except Exception:
                pass

    except Exception as e:
        state.error_msg = str(e)
        state.status = "error"
        print(f"[Force] ERROR: {e}")
        if robot:
            try:
                robot.StopMotion()
                ret = robot.GetActualTCPPose()
                if isinstance(ret, tuple) and ret[0] == 0:
                    cur = ret[1]
                    robot.MoveCart(
                        desc_pos=[float(cur[0]), float(cur[1]), SAFE_Z_MM,
                                  float(cur[3]), float(cur[4]), float(cur[5])],
                        tool=tool, user=user, blendT=MOVE_CART_BLEND_BLOCKING,
                    )
                robot.FT_Activate(0)
                robot.CloseRPC()
            except Exception:
                pass


# ===================== 独立测试函数 =====================

def test_sensor_readings(robot_ip="192.168.58.2", duration_s=10):
    """连接、初始化传感器，打印力数据持续 duration_s 秒。"""
    print(f"[Test] 传感器读数测试 ({duration_s}s)")
    robot = Robot.RPC(robot_ip)

    config = ForceControlConfig()
    if not init_force_sensor(robot, config):
        robot.CloseRPC()
        return

    print(f"\n{'Time':>8} {'Fx':>8} {'Fy':>8} {'Fz':>8} {'Mx':>8} {'My':>8} {'Mz':>8}")
    print("-" * 60)

    t_start = time.time()
    try:
        while time.time() - t_start < duration_s:
            ret = robot.FT_GetForceTorqueRCS()
            if isinstance(ret, tuple) and ret[0] == 0:
                d = ret[1]
                elapsed = time.time() - t_start
                print(f"{elapsed:>8.2f} {d[0]:>8.2f} {d[1]:>8.2f} {d[2]:>8.2f} "
                      f"{d[3]:>8.3f} {d[4]:>8.3f} {d[5]:>8.3f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Test] 用户中断")

    robot.FT_Activate(0)
    robot.CloseRPC()
    print("[Test] 完成")


def test_find_surface(robot_ip="192.168.58.2"):
    """下探，接触，报告 Z 坐标。"""
    print("[Test] 表面探测测试")
    robot = Robot.RPC(robot_ip)

    config = ForceControlConfig()
    if not init_force_sensor(robot, config):
        robot.CloseRPC()
        return

    setup_collision_guard(robot, config)

    print("[Test] 开始探面...")
    if find_surface(robot, config):
        ret = robot.GetActualTCPPose()
        if isinstance(ret, tuple) and ret[0] == 0:
            pos = ret[1]
            print(f"[Test] 接触位置: X={pos[0]:.1f} Y={pos[1]:.1f} Z={pos[2]:.1f}")
        # 读取接触力
        ret = robot.FT_GetForceTorqueRCS()
        if isinstance(ret, tuple) and ret[0] == 0:
            d = ret[1]
            print(f"[Test] 接触力: Fz={d[2]:.2f}N")
    else:
        print("[Test] 探面失败")

    disable_collision_guard(robot)
    robot.FT_Activate(0)
    robot.CloseRPC()
    print("[Test] 完成")


def test_force_hold(robot_ip="192.168.58.2", hold_s=10, target_force=5.0):
    """探面后维持恒力 hold_s 秒，打印力变化。"""
    print(f"[Test] 恒力维持测试 ({hold_s}s, target={target_force}N)")
    robot = Robot.RPC(robot_ip)

    config = ForceControlConfig(target_force_z=target_force)
    if not init_force_sensor(robot, config):
        robot.CloseRPC()
        return

    setup_collision_guard(robot, config)

    print("[Test] 探面...")
    if not find_surface(robot, config):
        print("[Test] 探面失败")
        robot.FT_Activate(0)
        robot.CloseRPC()
        return

    # 接触后执行微量下压，避免停在“刚接触临界点”导致后续缓慢失接触
    # 注意：这里不再校零，保留接触力偏置用于后续恒力闭环。
    prepress_mm = 1.5
    ret = robot.GetActualTCPPose()
    if isinstance(ret, tuple) and ret[0] == 0:
        p = ret[1]
        prepress_pose = [float(p[0]), float(p[1]), float(p[2]) - prepress_mm,
                         float(p[3]), float(p[4]), float(p[5])]
        rtn = robot.MoveCart(desc_pos=prepress_pose, tool=0, user=0, blendT=MOVE_CART_BLEND_BLOCKING)
        if rtn == 0:
            print(f"[Test] 预压位移已执行: {prepress_mm:.1f}mm")
            time.sleep(0.3)
        else:
            print(f"[Test] 预压位移失败, err={rtn} (继续测试)")

    # 预压阶段：先用较小目标力建立稳定接触，避免一上来大目标导致控制器响应慢
    preload_force = min(2.0, max(0.8, abs(target_force) * 0.4))
    preload_cfg = ForceControlConfig(
        target_force_z=preload_force,
        ft_pid=[0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
        max_dis=max(config.max_dis, 60.0),
        max_ang=config.max_ang,
        filter_sign=config.filter_sign,
        ft_m=config.ft_m,
        ft_b=config.ft_b,
    )
    if not start_force_control(robot, "ft_control", preload_cfg):
        print("[Test] 预压启动失败")
        robot.FT_Activate(0)
        robot.CloseRPC()
        return
    time.sleep(0.8)

    # 正式恒力：提高 PID_P 与最大调整距离，提升达到目标力的能力
    tuned_cfg = ForceControlConfig(
        target_force_z=target_force,
        ft_pid=[0.02, 0.0, 0.0, 0.0, 0.0, 0.0],
        max_dis=max(config.max_dis, 100.0),
        max_ang=config.max_ang,
        filter_sign=config.filter_sign,
        ft_m=config.ft_m,
        ft_b=config.ft_b,
    )
    if not start_force_control(robot, "ft_control", tuned_cfg):
        print("[Test] 力控启动失败")
        robot.FT_Activate(0)
        robot.CloseRPC()
        return

    print(f"\n{'Time':>8} {'Fz':>8}")
    print("-" * 20)

    t_start = time.time()
    boosted = False
    try:
        while time.time() - t_start < hold_s:
            fz = get_force_z(robot)
            if fz is not None:
                elapsed = time.time() - t_start
                print(f"{elapsed:>8.2f} {fz:>8.2f}")

                # 若 2s 后仍明显达不到目标力，自动提升一次控制增益
                err = abs(fz - (-abs(target_force)))
                if (not boosted) and elapsed > 2.0 and err > max(1.5, abs(target_force) * 0.5):
                    boost_cfg = ForceControlConfig(
                        target_force_z=target_force,
                        ft_pid=[0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
                        max_dis=max(tuned_cfg.max_dis, 140.0),
                        max_ang=tuned_cfg.max_ang,
                        filter_sign=tuned_cfg.filter_sign,
                        ft_m=tuned_cfg.ft_m,
                        ft_b=tuned_cfg.ft_b,
                    )
                    if start_force_control(robot, "ft_control", boost_cfg):
                        boosted = True
                        print("[Test] 自动增益提升: PID_P=0.05, max_dis=140mm")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Test] 用户中断")

    stop_force_control(robot, "ft_control")
    robot.FT_Activate(0)
    robot.CloseRPC()
    print("[Test] 完成")


# ===================== CLI 入口 =====================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="力控模块测试")
    parser.add_argument("--test", choices=["sensor", "surface", "hold"],
                        required=True, help="测试项目")
    parser.add_argument("--ip", default="192.168.58.2", help="机器人 IP")
    parser.add_argument("--duration", type=float, default=10, help="测试持续时间(s)")
    parser.add_argument("--force", type=float, default=5.0, help="目标力(N)")
    args = parser.parse_args()

    if args.test == "sensor":
        test_sensor_readings(robot_ip=args.ip, duration_s=args.duration)
    elif args.test == "surface":
        test_find_surface(robot_ip=args.ip)
    elif args.test == "hold":
        test_force_hold(robot_ip=args.ip, hold_s=args.duration, target_force=args.force)
