"""
lasttime_ros2.py - 膀胱经按摩动作演示程序（ROS 2 控制版）

复用 lasttime.py 的视觉检测与轨迹生成流程，
将机械臂控制从 Python SDK 直连切换到 FAIRINO ROS 2 service/topic。
"""

import os
import sys
import time
import math

import rclpy
from rclpy.node import Node

from fairino_msgs.msg import RobotNonrtState
from fairino_msgs.srv import RemoteCmdInterface
from force_control import (
    FORCE_SENSOR_COMPANY,
    FORCE_SENSOR_DEVICE,
    ForceControlConfig,
    SENSOR_ID,
)
from lasttime import (
    BLEND_BLOCKING,
    DIAN_JIN_DEPTH_MM,
    ENABLE_LIVE_PREVIEW_WINDOW,
    FEN_JIN_LATERAL_MM,
    HOVER_HEIGHT_MM,
    INIT_POSE_P24,
    INIT_SAFE_Z_MM,
    LastTimeDemo as _SdkLastTimeDemo,
    MOVE_VEL_FAST,
    MOVE_VEL_SLOW,
    PLANE_FIT_MIN_POINTS,
    PLANE_FIT_RADIUS_PX,
    PLANE_FIT_STEP_PX,
    ROBOT_IP,
    SAMPLE_POINTS,
    build_pose_from_frame,
)


ROS2_WORKSPACE = "/home/franka/massage/robots/fairino/fairino_ros2/frcobot_ros2-master"
ROS2_SERVICE_NAME = os.environ.get("FAIRINO_REMOTE_SERVICE", "fairino_remote_command_service")
ROS2_STATE_TOPIC = os.environ.get("FAIRINO_STATE_TOPIC", "nonrt_state_data")
ROS2_SERVICE_WAIT_S = float(os.environ.get("ROS2_SERVICE_WAIT_S", "20.0"))
ROS2_CALL_TIMEOUT_S = float(os.environ.get("ROS2_CALL_TIMEOUT_S", "20.0"))
ROS2_STATE_WAIT_S = float(os.environ.get("ROS2_STATE_WAIT_S", "3.0"))
ROS2_MOTION_DONE_WAIT_S = float(os.environ.get("ROS2_MOTION_DONE_WAIT_S", "15.0"))
ROS2_MOTION_DONE_POSE_TOL_MM = float(os.environ.get("ROS2_MOTION_DONE_POSE_TOL_MM", "3.0"))
ROS2_MOTION_DONE_ORI_TOL_DEG = float(os.environ.get("ROS2_MOTION_DONE_ORI_TOL_DEG", "3.0"))
ROS2_MOVE_ACC = float(os.environ.get("ROS2_MOVE_ACC", "0"))
ROS2_MOVE_OVL = float(os.environ.get("ROS2_MOVE_OVL", "100"))
ROS2_TOOL = int(os.environ.get("ROBOT_TOOL_ID", "0"))
ROS2_USER = int(os.environ.get("ROBOT_USER_ID", "0"))
ROS2_LIFT_SAFE_Z_MM = float(os.environ.get("ROS2_LIFT_SAFE_Z_MM", str(INIT_SAFE_Z_MM)))
ROS2_KEEP_CURRENT_ORIENTATION = os.environ.get(
    "LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION", "1"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
ROS2_TRANSIT_MARGIN_MM = float(os.environ.get("ROS2_TRANSIT_MARGIN_MM", "80.0"))
ROS2_SEGMENT_MAX_STEP_MM = float(os.environ.get("ROS2_SEGMENT_MAX_STEP_MM", "50.0"))
ROS2_SEGMENT_MAX_STEPS = int(os.environ.get("ROS2_SEGMENT_MAX_STEPS", "80"))
ROS2_USE_LEGACY_SAFE_POSE = os.environ.get(
    "ROS2_USE_LEGACY_SAFE_POSE", "0"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
ROS2_RESET_ERRORS = os.environ.get("ROS2_RESET_ERRORS", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
LASTTIME_ROS2_FORCE = os.environ.get("LASTTIME_ROS2_FORCE", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
FORCE_TARGET_N = float(os.environ.get("LASTTIME_FORCE_N", "10.0"))
FORCE_CONTACT_OFFSET_MM = float(
    os.environ.get(
        "LASTTIME_FORCE_CONTACT_OFFSET_MM",
        "0.0",
    )
)
FORCE_FEN_LATERAL_MM = float(
    os.environ.get("LASTTIME_FORCE_FEN_LATERAL_MM", str(min(abs(FEN_JIN_LATERAL_MM), 8.0)))
)
FORCE_MAX_DIS_MM = float(os.environ.get("LASTTIME_FORCE_MAX_DIS_MM", "8.0"))
FORCE_MAX_ANG_DEG = float(os.environ.get("LASTTIME_FORCE_MAX_ANG_DEG", "3.0"))
FORCE_PID_P = float(os.environ.get("LASTTIME_FORCE_PID_P", "0.003"))
FORCE_SOFTWARE_LIMIT_N = float(
    os.environ.get("LASTTIME_FORCE_SOFTWARE_LIMIT_N", str(abs(FORCE_TARGET_N) + 6.0))
)
FORCE_PRESTART_LIMIT_N = float(
    os.environ.get("LASTTIME_FORCE_PRESTART_LIMIT_N", str(abs(FORCE_TARGET_N) + 4.0))
)
FORCE_START_SETTLE_S = float(os.environ.get("LASTTIME_FORCE_START_SETTLE_S", "0.03"))
FORCE_STOP_SETTLE_S = float(os.environ.get("LASTTIME_FORCE_STOP_SETTLE_S", "0.25"))
FORCE_SOFTWARE_TORQUE_LIMIT_NM = float(
    os.environ.get("LASTTIME_FORCE_SOFTWARE_TORQUE_LIMIT_NM", "3.0")
)
FORCE_GUARD_ENABLE = os.environ.get("LASTTIME_FORCE_GUARD", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
FORCE_GUARD_LIMIT_N = float(os.environ.get("LASTTIME_FORCE_GUARD_LIMIT_N", "30.0"))
FORCE_GUARD_TORQUE_LIMIT_NM = float(
    os.environ.get("LASTTIME_FORCE_GUARD_TORQUE_LIMIT_NM", "3.5")
)
FORCE_DIAN_DWELL_S = float(os.environ.get("LASTTIME_FORCE_DIAN_DWELL_S", "0.6"))
FORCE_FEN_DWELL_S = float(os.environ.get("LASTTIME_FORCE_FEN_DWELL_S", "0.25"))
FORCE_SHUN_DWELL_S = float(os.environ.get("LASTTIME_FORCE_SHUN_DWELL_S", "0.05"))
FORCE_MONITOR_HZ = float(os.environ.get("LASTTIME_FORCE_MONITOR_HZ", "20.0"))
FORCE_SENSOR_BUS = int(os.environ.get("LASTTIME_FORCE_SENSOR_BUS", "1"))
FORCE_ALLOW_SKIP_ZERO = os.environ.get("LASTTIME_FORCE_ALLOW_SKIP_ZERO", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
FORCE_ZERO_MAX_ABS_N = float(os.environ.get("LASTTIME_FORCE_ZERO_MAX_ABS_N", "3.0"))
FORCE_AXIS_SIGN = -1.0 if float(os.environ.get("LASTTIME_FORCE_AXIS_SIGN", "-1.0")) < 0 else 1.0


def _fmt_value(value):
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.6f}".rstrip("0").rstrip(".")


def _parse_ret_code(cmd_res):
    if not cmd_res:
        return -9999
    head = str(cmd_res).split(",", 1)[0].strip()
    try:
        return int(float(head))
    except ValueError:
        return -9999


def _state_field(msg, *names, default=None):
    if msg is None:
        return default
    for name in names:
        if hasattr(msg, name):
            return getattr(msg, name)
    return default


def _state_pose(msg):
    pose = [
        _state_field(msg, "cart_x_cur_pos"),
        _state_field(msg, "cart_y_cur_pos"),
        _state_field(msg, "cart_z_cur_pos"),
        _state_field(msg, "cart_a_cur_pos"),
        _state_field(msg, "cart_b_cur_pos"),
        _state_field(msg, "cart_c_cur_pos"),
    ]
    if any(value is None for value in pose):
        raise RuntimeError("状态话题缺少当前 TCP 位姿字段")
    return [float(value) for value in pose]


class Ros2RobotProxy:
    """用 ROS 2 service/topic 模拟旧 SDK 的最小控制接口。"""

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self._owns_context = False
        self.node = None
        self.client = None
        self._sub = None
        self.latest_state = None
        self.latest_state_time = 0.0

    def connect(self):
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self.node = Node("lasttime_ros2_client")
        self.client = self.node.create_client(RemoteCmdInterface, ROS2_SERVICE_NAME)
        self._sub = self.node.create_subscription(
            RobotNonrtState,
            ROS2_STATE_TOPIC,
            self._state_callback,
            10,
        )

        print(f"等待 ROS2 控制服务: {ROS2_SERVICE_NAME}")
        deadline = time.time() + ROS2_SERVICE_WAIT_S
        while time.time() < deadline:
            if self.client.wait_for_service(timeout_sec=0.5):
                break
            self.spin_once(0.05)

        if not self.client.service_is_ready():
            raise RuntimeError(f"未找到 ROS2 控制服务: {ROS2_SERVICE_NAME}")

        self.wait_for_state(ROS2_STATE_WAIT_S, required=True)
        self._ensure_robot_ready()
        pose = self.get_actual_tcp_pose()
        pose_text = ", ".join(_fmt_value(v) for v in pose)
        print(f"机械臂初始化完成（ROS 2 控制服务已连接）")
        print(f"当前 TCP 位姿: [{pose_text}]")

    def _state_callback(self, msg):
        self.latest_state = msg
        self.latest_state_time = time.time()

    def spin_once(self, timeout_sec=0.1):
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def wait_for_state(self, timeout_sec, required):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.latest_state is not None:
                return True
            self.spin_once(0.1)
        if required:
            raise RuntimeError(f"在 {timeout_sec:.1f}s 内未收到状态话题: {ROS2_STATE_TOPIC}")
        return False

    def _call(self, cmd_str, timeout_sec=ROS2_CALL_TIMEOUT_S, raise_on_error=True):
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd_str
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"ROS2 指令超时: {cmd_str}")

        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"ROS2 指令异常 {cmd_str}: {exc}")

        cmd_res = future.result().cmd_res
        ret = _parse_ret_code(cmd_res)
        if raise_on_error and ret != 0:
            raise RuntimeError(f"ROS2 指令失败 {cmd_str}: {cmd_res}")
        return ret, cmd_res

    def _ensure_robot_ready(self):
        self.wait_for_state(ROS2_STATE_WAIT_S, required=False)

        if self.latest_state is None or int(_state_field(self.latest_state, "robot_mode", default=0)) != 0:
            self._call("Mode(0)")
            time.sleep(0.2)
            self.wait_for_state(ROS2_STATE_WAIT_S, required=False)

        ret, cmd_res = self._call("RobotEnable(1)", raise_on_error=False)
        if ret != 0 and ROS2_RESET_ERRORS:
            self._call("ResetAllError()", raise_on_error=False)
            time.sleep(0.5)
            ret, cmd_res = self._call("RobotEnable(1)", raise_on_error=False)
        if ret != 0:
            raise RuntimeError(f"机械臂使能失败: {cmd_res}")
        time.sleep(0.5)
        self.wait_for_state(ROS2_STATE_WAIT_S, required=False)

        self.SetSpeed(MOVE_VEL_FAST)

    def get_actual_tcp_pose(self):
        self.wait_for_state(ROS2_STATE_WAIT_S, required=True)
        return _state_pose(self.latest_state)

    def wait_motion_done(self, timeout_sec=ROS2_MOTION_DONE_WAIT_S):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            self.spin_once(0.1)
            motion_done = _state_field(
                self.latest_state,
                "motion_done",
                "robot_motion_done",
                default=None,
            )
            if motion_done is not None and int(motion_done) == 1:
                return True
        return False

    def pose_close_to(self, target_pose):
        try:
            current = self.get_actual_tcp_pose()
        except Exception:
            return False
        pos_dist = math.sqrt(
            (float(current[0]) - float(target_pose[0])) ** 2
            + (float(current[1]) - float(target_pose[1])) ** 2
            + (float(current[2]) - float(target_pose[2])) ** 2
        )
        ori_dist = max(
            abs(float(current[3]) - float(target_pose[3])),
            abs(float(current[4]) - float(target_pose[4])),
            abs(float(current[5]) - float(target_pose[5])),
        )
        if pos_dist <= ROS2_MOTION_DONE_POSE_TOL_MM and ori_dist <= ROS2_MOTION_DONE_ORI_TOL_DEG:
            print(
                f"MoveL 等待 motion_done 超时，但实际位姿已接近目标 "
                f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)，按成功处理"
            )
            return True
        print(
            f"MoveL 等待 motion_done 超时，实际位姿未到目标 "
            f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)"
        )
        return False

    def SetSpeed(self, vel):
        ret, _ = self._call(f"SetSpeed({_fmt_value(int(vel))})")
        return ret

    def MoveCart(
        self,
        desc_pos,
        tool=ROS2_TOOL,
        user=ROS2_USER,
        vel=MOVE_VEL_FAST,
        acc=ROS2_MOVE_ACC,
        ovl=ROS2_MOVE_OVL,
        blendT=BLEND_BLOCKING,
        config=-1,
    ):
        point_cmd = "CARTPoint(1," + ",".join(_fmt_value(v) for v in desc_pos) + ")"
        ret, _ = self._call(point_cmd, raise_on_error=False)
        if ret != 0:
            return ret

        cmd = "MoveL(" + ",".join(
            [
                "CART1",
                _fmt_value(int(vel)),
                _fmt_value(int(tool)),
                _fmt_value(int(user)),
            ]
        ) + ")"
        ret, _ = self._call(cmd, raise_on_error=False)
        if ret == 0 and float(blendT) < 0:
            if not self.wait_motion_done():
                if self.pose_close_to(desc_pos):
                    return 0
                return -1001
        return ret

    def CloseRPC(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()
            self._owns_context = False


class Ros2ForceController:
    """通过同一个 ROS2 command server 控制六维力传感器和恒力控制。"""

    def __init__(self, robot_proxy, target_force_n=FORCE_TARGET_N):
        self.robot = robot_proxy
        self.target_force_n = abs(float(target_force_n))
        self.active = False
        self.guard_active = False
        self.config = ForceControlConfig()
        self.config.target_force_z = self.target_force_n
        self.config.ft_pid = [FORCE_PID_P, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.config.max_dis = FORCE_MAX_DIS_MM
        self.config.max_ang = FORCE_MAX_ANG_DEG
        self.config.software_force_limit = max(
            abs(FORCE_SOFTWARE_LIMIT_N),
            self.target_force_n + 5.0,
        )
        self.config.enable_collision_guard = FORCE_GUARD_ENABLE
        self.config.guard_force_limit = max(abs(FORCE_GUARD_LIMIT_N), self.target_force_n + 10.0)
        self.config.guard_torque_limit = abs(FORCE_GUARD_TORQUE_LIMIT_NM)
        self._monitor_period = 1.0 / max(1.0, FORCE_MONITOR_HZ)

    def _cmd(self, name, *values):
        return f"{name}(" + ",".join(_fmt_value(v) for v in values) + ")"

    def _call_force(self, cmd, context, required=True):
        try:
            ret, cmd_res = self.robot._call(cmd, raise_on_error=False)
        except Exception as exc:
            if required:
                raise
            print(f"[Force] {context} 异常: {exc}")
            return -9999, str(exc)
        if ret != 0:
            message = f"{context} 失败: {cmd_res}"
            if required:
                raise RuntimeError(message)
            print(f"[Force] {message}")
        return ret, cmd_res

    def _parse_force_response(self, cmd_res):
        parts = [p.strip() for p in str(cmd_res).split(",")]
        if len(parts) < 7:
            return None
        try:
            ret = int(float(parts[0]))
            values = [float(v) for v in parts[1:7]]
        except ValueError:
            return None
        if ret != 0:
            return None
        return values

    def _reading_available(self):
        data = self.read()
        return data is not None

    def _allow_zero_failure(self):
        data = self.read()
        if data is None or not FORCE_ALLOW_SKIP_ZERO:
            return False
        max_force = max(abs(data[0]), abs(data[1]), abs(data[2]))
        if max_force <= FORCE_ZERO_MAX_ABS_N:
            print(
                "[Force] FT_SetZero 失败但当前读数接近零点，继续执行 "
                f"(Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N, "
                f"threshold={FORCE_ZERO_MAX_ABS_N:.1f}N)"
            )
            return True
        print(
            "[Force] FT_SetZero 失败且当前力偏置过大，停止 "
            f"(Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N)"
        )
        return False

    def connect_and_init(self):
        print("\n[Force] 使用 ROS2 控制服务初始化力传感器...")
        print("[Force] 校零要求：末端必须悬空、无外部接触。")
        self._call_force(
            self._cmd("FT_SetConfig", FORCE_SENSOR_COMPANY, FORCE_SENSOR_DEVICE, 0, FORCE_SENSOR_BUS),
            "FT_SetConfig",
        )
        print(
            f"[Force] FT_SetConfig OK "
            f"(company={FORCE_SENSOR_COMPANY}, device={FORCE_SENSOR_DEVICE}, bus={FORCE_SENSOR_BUS})"
        )
        self._call_force(self._cmd("FT_Activate", 0), "FT_Activate(0)", required=False)
        time.sleep(0.3)
        ret, _ = self._call_force(self._cmd("FT_Activate", 1), "FT_Activate(1)", required=False)
        if ret != 0:
            if self._reading_available():
                print(f"[Force] FT_Activate(1) 警告 err={ret}；但力传感器读数可用，继续")
            else:
                raise RuntimeError(f"FT_Activate(1) 失败 err={ret}")
        else:
            print("[Force] FT_Activate OK")
        time.sleep(0.5)

        self._call_force(self._cmd("SetForceSensorPayload", 0.0), "SetForceSensorPayload(0)")
        self._call_force(self._cmd("SetForceSensorPayloadCog", 0.0, 0.0, 0.0), "SetForceSensorPayloadCog(0,0,0)")
        print("[Force] 负载参数已置零，准备传感器校零")

        self._call_force(self._cmd("FT_SetZero", 0), "FT_SetZero(0)", required=False)
        time.sleep(0.3)
        zero_ok = False
        for attempt in range(1, 6):
            time.sleep(0.6)
            ret, _ = self._call_force(self._cmd("FT_SetZero", 1), "FT_SetZero(1)", required=False)
            if ret == 0:
                print(f"[Force] FT_SetZero(1) OK (attempt {attempt}/5)")
                zero_ok = True
                break
        if not zero_ok:
            if not self._allow_zero_failure():
                raise RuntimeError("FT_SetZero(1) 多次失败，请确认末端悬空且传感器负载已置零")

        ret, _ = self._call_force(self._cmd("FT_SetRCS", 0, 0, 0, 0, 0, 0, 0), "FT_SetRCS", required=False)
        if ret != 0:
            if self._reading_available():
                print(f"[Force] FT_SetRCS 警告 err={ret}；但力传感器读数可用，继续")
            else:
                raise RuntimeError(f"FT_SetRCS 失败 err={ret}")
        else:
            print("[Force] FT_SetRCS OK (工具坐标系)")

        if self.config.enable_collision_guard:
            self._enable_guard()
        data = self.read()
        if data is not None:
            print(
                "[Force] 校零后读数: "
                f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f} "
                f"Mx={data[3]:.3f} My={data[4]:.3f} Mz={data[5]:.3f}"
            )
        print(
            f"[Force] 10N 恒力参数: target={self.target_force_n:.1f}N, "
            f"max_dis={self.config.max_dis:.1f}mm, "
            f"software_limit={self.config.software_force_limit:.1f}N, "
            f"axis_sign={FORCE_AXIS_SIGN:+.0f}"
        )

    def _enable_guard(self):
        if not self.config.enable_collision_guard or self.guard_active:
            return
        fl = self.config.guard_force_limit
        tl = self.config.guard_torque_limit
        self._call_force(
            self._cmd(
                "FT_Guard",
                1,
                SENSOR_ID,
                1, 1, 1, 1, 1, 1,
                0, 0, 0, 0, 0, 0,
                fl, fl, fl, tl, tl, tl,
                fl, fl, fl, tl, tl, tl,
            ),
            "FT_Guard(1)",
        )
        print(f"[Force] 碰撞守护已开启 (力阈值={fl:.1f}N, 力矩阈值={tl:.1f}Nm)")
        self.guard_active = True

    def _disable_guard(self):
        if not self.guard_active:
            return
        fl = self.config.guard_force_limit
        tl = self.config.guard_torque_limit
        self._call_force(
            self._cmd(
                "FT_Guard",
                0,
                SENSOR_ID,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                fl, fl, fl, tl, tl, tl,
                fl, fl, fl, tl, tl, tl,
            ),
            "FT_Guard(0)",
            required=False,
        )
        print("[Force] 碰撞守护已关闭")
        self.guard_active = False

    def _ft_control_cmd(self, flag, select, ft, ft_pid, max_dis, max_ang, is_no_block):
        return self._cmd(
            "FT_Control",
            int(flag),
            SENSOR_ID,
            *select,
            *ft,
            *ft_pid,
            0,
            0,
            max_dis,
            max_ang,
            self.config.filter_sign,
            0,
            int(is_no_block),
        )

    def _stop_robot_motion(self, context):
        self._call_force(self._cmd("StopMotion"), f"{context}: StopMotion", required=False)

    def start(self, context):
        if self.robot is None:
            raise RuntimeError("力控通道未初始化")
        if self.active:
            return
        data = self.read()
        if data is not None:
            max_force = max(abs(data[0]), abs(data[1]), abs(data[2]))
            if max_force > FORCE_PRESTART_LIMIT_N:
                self._stop_robot_motion("力控启动前力偏大")
                raise RuntimeError(
                    f"{context}: 力控启动前已有过大接触力，"
                    f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N"
                )
        self._enable_guard()
        target_fz = FORCE_AXIS_SIGN * abs(self.target_force_n)
        cmd = self._ft_control_cmd(
            flag=1,
            select=[0, 0, 1, 0, 0, 0],
            ft=[0, 0, target_fz, 0, 0, 0],
            ft_pid=self.config.ft_pid,
            max_dis=self.config.max_dis,
            max_ang=self.config.max_ang,
            is_no_block=1,
        )
        self._call_force(cmd, f"{context}: FT_Control(1)")
        print(
            f"[Force] 恒力控制已启动 (目标力={target_fz:.1f}N, "
            f"PID_P={self.config.ft_pid[0]}, max_dis={self.config.max_dis:.1f}mm)"
        )
        self.active = True
        time.sleep(FORCE_START_SETTLE_S)
        self.check_limits(context)

    def stop(self, context=""):
        if self.robot is None:
            return
        if self.active:
            cmd = self._ft_control_cmd(
                flag=0,
                select=[0, 0, 0, 0, 0, 0],
                ft=[0, 0, 0, 0, 0, 0],
                ft_pid=[0, 0, 0, 0, 0, 0],
                max_dis=0,
                max_ang=0,
                is_no_block=1,
            )
            ret, _ = self._call_force(cmd, f"{context}: FT_Control(0)", required=False)
            if ret == 0:
                print(f"[Force] 恒力控制已停止 {context}".rstrip())
            self.active = False
            time.sleep(FORCE_STOP_SETTLE_S)
            try:
                self.robot.spin_once(0.05)
            except Exception:
                pass

        self._disable_guard()

    def read(self):
        if self.robot is None:
            return None
        ret, cmd_res = self._call_force(self._cmd("FT_GetForceTorqueRCS", 1), "FT_GetForceTorqueRCS", required=False)
        if ret == 0:
            data = self._parse_force_response(cmd_res)
            if data is not None:
                return data

        self.robot.spin_once(0.02)
        msg = self.robot.latest_state
        data = [
            _state_field(msg, "ft_fx_data"),
            _state_field(msg, "ft_fy_data"),
            _state_field(msg, "ft_fz_data"),
            _state_field(msg, "ft_tx_data"),
            _state_field(msg, "ft_ty_data"),
            _state_field(msg, "ft_tz_data"),
        ]
        if any(v is None for v in data):
            return None
        return [float(v) for v in data]

    def check_limits(self, context):
        data = self.read()
        if data is None:
            raise RuntimeError(f"{context}: 无法读取六维力数据")
        max_force = max(abs(data[0]), abs(data[1]), abs(data[2]))
        max_torque = max(abs(data[3]), abs(data[4]), abs(data[5]))
        if max_force > self.config.software_force_limit:
            self._stop_robot_motion("软件力限触发")
            self.stop("软件力限触发")
            self._stop_robot_motion("软件力限触发")
            raise RuntimeError(
                f"{context}: 软件力限触发，"
                f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N"
            )
        if max_torque > FORCE_SOFTWARE_TORQUE_LIMIT_NM:
            self._stop_robot_motion("软件力矩限触发")
            self.stop("软件力矩限触发")
            self._stop_robot_motion("软件力矩限触发")
            raise RuntimeError(
                f"{context}: 软件力矩限触发，"
                f"Mx={data[3]:.3f} My={data[4]:.3f} Mz={data[5]:.3f}Nm"
            )
        return data

    def dwell(self, seconds, context):
        deadline = time.time() + max(0.0, float(seconds))
        last_print = 0.0
        while time.time() < deadline:
            data = self.check_limits(context)
            now = time.time()
            if now - last_print > 0.5:
                print(
                    f"[Force] {context}: "
                    f"Fx={data[0]:.1f} Fy={data[1]:.1f} Fz={data[2]:.1f}N"
                )
                last_print = now
            time.sleep(self._monitor_period)

    def close(self):
        if self.robot is None:
            return
        try:
            self.stop("关闭")
            print("[Force] 力控通道已关闭")
        except Exception as exc:
            print(f"[Force] 关闭力控通道失败: {exc}")
        finally:
            self.robot = None


class LastTimeRos2Demo(_SdkLastTimeDemo):
    """复用原 lasttime 的视觉逻辑，替换为 ROS 2 机械臂控制。"""

    def __init__(self):
        super().__init__()
        self.force_controller = None

    def init_robot(self):
        print(f"连接机械臂 {ROBOT_IP}...")
        print(f"ROS2 控制服务: {ROS2_SERVICE_NAME}")
        print(f"ROS2 状态话题: {ROS2_STATE_TOPIC}")
        self.robot = Ros2RobotProxy(ROBOT_IP)
        self.robot.connect()

    def init_force_controller(self):
        if not LASTTIME_ROS2_FORCE:
            return True
        if self.force_controller is not None:
            return True
        self.force_controller = Ros2ForceController(self.robot, FORCE_TARGET_N)
        self.force_controller.connect_and_init()
        return True

    def close_force_controller(self):
        if self.force_controller is not None:
            self.force_controller.close()
            self.force_controller = None

    def _contact_pose_from_frame(self, frame, split_offset_mm=0.0):
        return self._pose_from_frame_offset(
            frame,
            FORCE_CONTACT_OFFSET_MM,
            split_offset_mm=split_offset_mm,
        )

    def _apply_motion_orientation(self, pose):
        if ROS2_KEEP_CURRENT_ORIENTATION and self.motion_orientation is not None:
            pose[3], pose[4], pose[5] = self.motion_orientation
        return pose

    def _pose_from_frame_offset(self, frame, offset_mm, split_offset_mm=0.0):
        pose = build_pose_from_frame(
            frame,
            tool_offset_mm=float(offset_mm),
            split_offset_mm=float(split_offset_mm),
        )
        return self._apply_motion_orientation(pose)

    def _move_cart_force_checked(self, pose, context, vel=MOVE_VEL_SLOW):
        ret = self.robot.MoveCart(
            desc_pos=pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=vel,
            blendT=BLEND_BLOCKING,
        )
        if ret != 0:
            print(f"    警告：{context}失败 (err={ret})")
            return False
        if self.force_controller is not None and self.force_controller.active:
            self.force_controller.check_limits(context)
        return True

    def _fmt_pose(self, pose):
        return "[" + ", ".join(_fmt_value(v) for v in pose) + "]"

    def _build_session_safe_pose(self):
        if ROS2_USE_LEGACY_SAFE_POSE:
            safe_pose = INIT_POSE_P24.copy()
            safe_pose[2] = ROS2_LIFT_SAFE_Z_MM
            self.motion_orientation = [safe_pose[3], safe_pose[4], safe_pose[5]]
            print(f"使用旧 P24 安全位: {self._fmt_pose(safe_pose)}")
            return safe_pose, True

        current_pose = self.robot.get_actual_tcp_pose()
        safe_pose = [float(v) for v in current_pose]
        self.motion_orientation = [safe_pose[3], safe_pose[4], safe_pose[5]]
        if safe_pose[2] < ROS2_LIFT_SAFE_Z_MM:
            safe_pose[2] = ROS2_LIFT_SAFE_Z_MM
            print(
                f"当前 TCP Z={current_pose[2]:.1f}mm，"
                f"先在当前位置竖直抬升到 Z={ROS2_LIFT_SAFE_Z_MM:.1f}mm"
            )
            return safe_pose, True

        print(
            f"当前 TCP 已在安全高度 Z={safe_pose[2]:.1f}mm，"
            "跳过旧 P24 固定安全位"
            )
        return safe_pose, False

    def _recover_robot_ready(self, context):
        print(f"{context}: 尝试恢复机械臂状态")
        for cmd in (
            "StopMotion()",
            "ResetAllError()",
            "Mode(0)",
            "RobotEnable(1)",
            f"SetSpeed({_fmt_value(MOVE_VEL_FAST)})",
        ):
            try:
                ret, _ = self.robot._call(cmd, raise_on_error=False)
                if ret != 0:
                    print(f"{context}: {cmd} 返回 {ret}")
            except Exception as exc:
                print(f"{context}: {cmd} 异常: {exc}")
            time.sleep(0.1)
        self.robot.wait_for_state(ROS2_STATE_WAIT_S, required=False)

    def _move_pose_segmented(self, pose, context, vel=MOVE_VEL_FAST, max_step_mm=ROS2_SEGMENT_MAX_STEP_MM):
        target = [float(v) for v in pose]
        for step in range(max(1, ROS2_SEGMENT_MAX_STEPS)):
            current = self.robot.get_actual_tcp_pose()
            dx = target[0] - current[0]
            dy = target[1] - current[1]
            dz = target[2] - current[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist <= max(1.0, float(max_step_mm)):
                waypoint = target
            else:
                scale = float(max_step_mm) / max(dist, 1e-6)
                waypoint = [
                    current[0] + dx * scale,
                    current[1] + dy * scale,
                    current[2] + dz * scale,
                    target[3],
                    target[4],
                    target[5],
                ]

            ret = self.robot.MoveCart(
                desc_pos=waypoint,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=vel,
                blendT=BLEND_BLOCKING,
            )
            if ret == 0:
                if waypoint is target:
                    return True
                continue

            if ret == 14:
                self._recover_robot_ready(f"{context} 分段{step + 1}")
                ret = self.robot.MoveCart(
                    desc_pos=waypoint,
                    tool=ROS2_TOOL,
                    user=ROS2_USER,
                    vel=vel,
                    blendT=BLEND_BLOCKING,
                )
                if ret == 0:
                    if waypoint is target:
                        return True
                    continue

            print(f"{context}: 分段移动失败 step={step + 1}, err={ret}, target={self._fmt_pose(waypoint)}")
            return False

        print(f"{context}: 分段移动超过最大步数 {ROS2_SEGMENT_MAX_STEPS}")
        return False

    def _move_cart_checked(self, pose, context, vel=MOVE_VEL_FAST, required=True):
        print(f"{context}: target={self._fmt_pose(pose)}")
        ret = self.robot.MoveCart(
            desc_pos=pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=vel,
            blendT=BLEND_BLOCKING,
        )
        if ret == 0:
            return True

        if ret == 14:
            self._recover_robot_ready(context)
            ret = self.robot.MoveCart(
                desc_pos=pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=vel,
                blendT=BLEND_BLOCKING,
            )
            if ret == 0:
                return True

        try:
            current = self.robot.get_actual_tcp_pose()
            print(f"{context}: current={self._fmt_pose(current)}")
        except Exception:
            pass

        message = f"{context}失败 (err={ret})"
        if required:
            print(f"错误：{message}")
        else:
            print(f"警告：{message}")
        return False

    def _move_to_work_pose(self, target_pose, context, vel=MOVE_VEL_FAST):
        target = [float(v) for v in target_pose]
        current = self.robot.get_actual_tcp_pose()
        transit_z = max(
            float(current[2]),
            float(target[2]) + ROS2_TRANSIT_MARGIN_MM,
            ROS2_LIFT_SAFE_Z_MM,
        )
        transit_pose = [
            target[0],
            target[1],
            transit_z,
            target[3],
            target[4],
            target[5],
        ]
        print(f"{context}: 分段靠近，过渡Z={transit_z:.1f}mm")
        if not self._move_pose_segmented(transit_pose, f"{context} 高位平移", vel=vel):
            return False
        return self._move_pose_segmented(target, f"{context} 下降", vel=vel)

    def execute_dian_jin(self, frame):
        self.update_preview_status("点筋", frame.get("index"))
        hover_pose = self._pose_from_frame_offset(frame, -HOVER_HEIGHT_MM)
        dian_jin_pose = self._pose_from_frame_offset(
            frame,
            -(HOVER_HEIGHT_MM - DIAN_JIN_DEPTH_MM),
        )

        if LASTTIME_ROS2_FORCE:
            dian_jin_pose = self._contact_pose_from_frame(frame)
            try:
                if not self._move_cart_force_checked(dian_jin_pose, "点筋下压"):
                    return False
                self.force_controller.start("点筋")
                self.force_controller.dwell(FORCE_DIAN_DWELL_S, "点筋保压")
            except Exception as exc:
                print(f"    警告：点筋力控失败 ({exc})")
                return False
            finally:
                self.force_controller.stop("点筋")

            ret = self.robot.MoveCart(
                desc_pos=hover_pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=MOVE_VEL_SLOW,
                blendT=BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：回到悬空位失败 (err={ret})")
                return False
            return True

        ret = self.robot.MoveCart(
            desc_pos=dian_jin_pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=MOVE_VEL_SLOW,
            blendT=BLEND_BLOCKING,
        )
        if ret != 0:
            print(f"    警告：点筋失败 (err={ret})")
            return False
        time.sleep(0.3)

        ret = self.robot.MoveCart(
            desc_pos=hover_pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=MOVE_VEL_SLOW,
            blendT=BLEND_BLOCKING,
        )
        if ret != 0:
            print(f"    警告：回到悬空位失败 (err={ret})")
            return False
        return True

    def execute_fen_jin(self, frame):
        self.update_preview_status("分筋", frame.get("index"))
        hover_pose = self._pose_from_frame_offset(frame, -HOVER_HEIGHT_MM)
        center_pose = self._contact_pose_from_frame(frame)
        positive_pose = self._contact_pose_from_frame(frame, FEN_JIN_LATERAL_MM)
        negative_pose = self._contact_pose_from_frame(frame, -FEN_JIN_LATERAL_MM)

        if LASTTIME_ROS2_FORCE:
            positive_pose = self._contact_pose_from_frame(frame, FORCE_FEN_LATERAL_MM)
            negative_pose = self._contact_pose_from_frame(frame, -FORCE_FEN_LATERAL_MM)
            try:
                if not self._move_cart_force_checked(center_pose, "分筋接触"):
                    return False
                self.force_controller.start("分筋")
                self.force_controller.dwell(FORCE_FEN_DWELL_S, "分筋中心保压")
                if not self._move_cart_force_checked(positive_pose, "分筋偏移+"):
                    return False
                self.force_controller.dwell(FORCE_FEN_DWELL_S, "分筋偏移+保压")
                if not self._move_cart_force_checked(negative_pose, "分筋偏移-"):
                    return False
                self.force_controller.dwell(FORCE_FEN_DWELL_S, "分筋偏移-保压")
                if not self._move_cart_force_checked(center_pose, "分筋回中心"):
                    return False
            except Exception as exc:
                print(f"    警告：分筋力控失败 ({exc})")
                return False
            finally:
                self.force_controller.stop("分筋")

            ret = self.robot.MoveCart(
                desc_pos=hover_pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=MOVE_VEL_SLOW,
                blendT=BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：分筋回悬空位失败 (err={ret})")
                return False
            return True

        positive_pose = self._pose_from_frame_offset(
            frame,
            -HOVER_HEIGHT_MM,
            split_offset_mm=FEN_JIN_LATERAL_MM,
        )
        negative_pose = self._pose_from_frame_offset(
            frame,
            -HOVER_HEIGHT_MM,
            split_offset_mm=-FEN_JIN_LATERAL_MM,
        )

        ret = self.robot.MoveCart(
            desc_pos=positive_pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=MOVE_VEL_SLOW,
            blendT=BLEND_BLOCKING,
        )
        if ret != 0:
            print(f"    警告：分筋偏移+失败 (err={ret})")
            return False
        time.sleep(0.2)

        ret = self.robot.MoveCart(
            desc_pos=negative_pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=MOVE_VEL_SLOW,
            blendT=BLEND_BLOCKING,
        )
        if ret != 0:
            print(f"    警告：分筋偏移-失败 (err={ret})")
            return False
        time.sleep(0.2)

        ret = self.robot.MoveCart(
            desc_pos=hover_pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=MOVE_VEL_SLOW,
            blendT=BLEND_BLOCKING,
        )
        if ret != 0:
            print(f"    警告：分筋回悬空位失败 (err={ret})")
            return False
        return True

    def execute_shun_jin(self):
        print("顺筋动作...")
        if LASTTIME_ROS2_FORCE:
            last_hover_pose = None
            try:
                start_pose = self._contact_pose_from_frame(self.massage_frames[0])
                if not self._move_cart_force_checked(start_pose, "顺筋接触起点"):
                    return False
                self.force_controller.start("顺筋")
                for i, frame in enumerate(self.massage_frames):
                    self.update_preview_status("顺筋", i)
                    print(f"  移动到点{i}...")
                    pose = self._contact_pose_from_frame(frame)
                    last_hover_pose = self._pose_from_frame_offset(
                        frame,
                        -HOVER_HEIGHT_MM,
                    )
                    if not self._move_cart_force_checked(pose, "顺筋移动"):
                        return False
                    self.force_controller.dwell(FORCE_SHUN_DWELL_S, "顺筋保压")
                return True
            except Exception as exc:
                print(f"    警告：顺筋力控失败 ({exc})")
                return False
            finally:
                self.force_controller.stop("顺筋")
                if last_hover_pose is not None:
                    ret = self.robot.MoveCart(
                        desc_pos=last_hover_pose,
                        tool=ROS2_TOOL,
                        user=ROS2_USER,
                        vel=MOVE_VEL_SLOW,
                        blendT=BLEND_BLOCKING,
                    )
                    if ret != 0:
                        print(f"    警告：顺筋结束回悬空位失败 (err={ret})")

        for i, frame in enumerate(self.massage_frames):
            self.update_preview_status("顺筋", i)
            print(f"  移动到点{i}...")
            pose = self._pose_from_frame_offset(frame, -HOVER_HEIGHT_MM)
            ret = self.robot.MoveCart(
                desc_pos=pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=MOVE_VEL_SLOW,
                blendT=BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：移动失败 (err={ret})")
                return False
        return True

    def execute_massage_sequence(self):
        print("\n开始执行按摩序列...")

        try:
            print("移动到安全高度...")
            self.update_preview_status("移动到安全高度")
            safe_pose, should_move_to_safe = self._build_session_safe_pose()
            if should_move_to_safe:
                if not self._move_cart_checked(safe_pose, "移动到安全高度", MOVE_VEL_FAST):
                    return False

            if LASTTIME_ROS2_FORCE:
                print(f"初始化 {FORCE_TARGET_N:.1f}N 恒力控制（请确认末端悬空无接触）...")
                self.init_force_controller()

            print("移动到起始位置...")
            first_frame = self.massage_frames[0]
            self.update_preview_status("移动到起始位置", first_frame.get("index", 0))
            first_pose = self._pose_from_frame_offset(first_frame, -HOVER_HEIGHT_MM)
            if not self._move_to_work_pose(first_pose, "移动到起始位置", MOVE_VEL_FAST):
                return False

            print("\n执行点筋+分筋动作...")
            for i, frame in enumerate(self.massage_frames):
                self.update_preview_status("到达悬空位", i)
                print(f"\n处理点 {i + 1}/{len(self.massage_points_mm)}...")
                hover_pose = self._pose_from_frame_offset(frame, -HOVER_HEIGHT_MM)
                ret = self.robot.MoveCart(
                    desc_pos=hover_pose,
                    tool=ROS2_TOOL,
                    user=ROS2_USER,
                    vel=MOVE_VEL_SLOW,
                    blendT=BLEND_BLOCKING,
                )
                if ret != 0:
                    print(f"  错误：移动到悬空位失败 (err={ret})")
                    return False

                print("  点筋...")
                if not self.execute_dian_jin(frame):
                    return False

                print("  分筋...")
                if not self.execute_fen_jin(frame):
                    return False

            print("\n回到起点...")
            self.update_preview_status("回到起点", first_frame.get("index", 0))
            if not self._move_cart_checked(first_pose, "回到起点", MOVE_VEL_FAST):
                return False

            print("\n执行顺筋动作...")
            if not self.execute_shun_jin():
                return False

            print("\n返回安全位置...")
            self.update_preview_status("返回安全位置")
            if not self._move_cart_checked(safe_pose, "返回安全位置", MOVE_VEL_FAST):
                return False

            print("\n按摩序列执行完成！")
            return True

        except Exception as e:
            print(f"\n错误：{e}")
            return False
        finally:
            self.close_force_controller()


def main():
    print("=" * 60)
    print("lasttime_ros2.py - 膀胱经按摩动作演示（ROS 2 控制版）")
    print("=" * 60)
    print()
    print("配置参数：")
    print(f"  机械臂IP: {ROBOT_IP}")
    print(f"  悬空高度: {HOVER_HEIGHT_MM}mm")
    print(f"  点筋深度: {DIAN_JIN_DEPTH_MM}mm")
    print(f"  分筋偏移: {FEN_JIN_LATERAL_MM}mm")
    print(f"  采样点数: {SAMPLE_POINTS}")
    print(f"  ROS2工作空间: {ROS2_WORKSPACE}")
    print(f"  ROS2控制服务: {ROS2_SERVICE_NAME}")
    print(f"  ROS2状态话题: {ROS2_STATE_TOPIC}")
    print(f"  末端姿态: {'保持当前TCP姿态' if ROS2_KEEP_CURRENT_ORIENTATION else '局部深度平面法向'}")
    print(
        f"  平面拟合: radius={PLANE_FIT_RADIUS_PX}px "
        f"step={PLANE_FIT_STEP_PX}px min_pts={PLANE_FIT_MIN_POINTS}"
    )
    print(f"  演示预览窗口: {'开启（实时跟踪，仅展示）' if ENABLE_LIVE_PREVIEW_WINDOW else '关闭'}")
    print(f"  工具/工件坐标系: tool={ROS2_TOOL}, user={ROS2_USER}")
    print(
        f"  安全位策略: {'旧P24固定安全位' if ROS2_USE_LEGACY_SAFE_POSE else '当前位置竖直抬升'} "
        f"safe_z={ROS2_LIFT_SAFE_Z_MM:.1f}mm"
    )
    if LASTTIME_ROS2_FORCE:
        print(
            f"  恒力控制: 开启 target={FORCE_TARGET_N:.1f}N "
            f"contact_offset={FORCE_CONTACT_OFFSET_MM:.1f}mm "
            f"limit={FORCE_SOFTWARE_LIMIT_N:.1f}N "
            f"prestart_limit={FORCE_PRESTART_LIMIT_N:.1f}N "
            f"max_dis={FORCE_MAX_DIS_MM:.1f}mm "
            f"PID_P={FORCE_PID_P:.4f} "
            f"force_fen={FORCE_FEN_LATERAL_MM:.1f}mm "
            f"guard={'on' if FORCE_GUARD_ENABLE else 'off'}"
        )
    else:
        print("  恒力控制: 关闭")
    print()

    demo = LastTimeRos2Demo()
    success = demo.run()

    if success:
        print("\n演示完成！")
        return 0

    print("\n演示失败")
    return 1


if __name__ == "__main__":
    sys.exit(main())
