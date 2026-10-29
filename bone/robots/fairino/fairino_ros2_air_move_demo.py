#!/usr/bin/env python3
"""
FAIRINO ROS2 最小空中运动 demo。

用途：
1. 读取 ROS2 远程状态话题，打印当前 TCP 位姿、关节角和故障标志。
2. 按标准链路尝试恢复：Mode(0) -> ResetAllError() -> RobotEnable(1) -> SetSpeed().
3. 以当前 TCP 为基准，做一段小幅悬空运动：
   当前位姿 -> 抬高 -> 左右小摆动 -> 回到起点。

依赖的远程命令字符串：
- Mode(0)
- ResetAllError()
- RobotEnable(1)
- SetSpeed(vel)
- CARTPoint(id,x,y,z,rx,ry,rz)
- MoveL(CART{id},vel,tool,user)
"""

from __future__ import annotations

import argparse
import os
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from fairino_msgs.msg import RobotNonrtState
    from fairino_msgs.srv import RemoteCmdInterface
except ImportError as exc:
    print(
        "缺少 ROS2/Fairino Python 环境。"
        "请先执行: source /opt/ros/humble/setup.bash && source <workspace>/install/setup.bash",
        file=sys.stderr,
    )
    raise


DEFAULT_SERVICE_NAME = os.environ.get("FAIRINO_REMOTE_SERVICE", "fairino_remote_command_service")
DEFAULT_STATE_TOPIC = os.environ.get("FAIRINO_STATE_TOPIC", "nonrt_state_data")
DEFAULT_WAIT_SERVICE_S = float(os.environ.get("ROS2_SERVICE_WAIT_S", "10.0"))
DEFAULT_WAIT_STATE_S = float(os.environ.get("ROS2_STATE_WAIT_S", "3.0"))
DEFAULT_CALL_TIMEOUT_S = float(os.environ.get("ROS2_CALL_TIMEOUT_S", "10.0"))
DEFAULT_MOTION_TIMEOUT_S = float(os.environ.get("ROS2_MOTION_DONE_WAIT_S", "20.0"))
EPS = 1e-6


def fmt_value(value: float) -> str:
    text = f"{float(value):.6f}"
    return text.rstrip("0").rstrip(".")


def pose_text(values: list[float]) -> str:
    return "[" + ", ".join(fmt_value(v) for v in values) + "]"


def parse_ret_code(cmd_res: str) -> int:
    if not cmd_res:
        return -9999
    head = str(cmd_res).split(",", 1)[0].strip()
    try:
        return int(float(head))
    except ValueError:
        return -9999


def offset_pose(
    pose: list[float],
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    da: float = 0.0,
    db: float = 0.0,
    dc: float = 0.0,
) -> list[float]:
    return [
        pose[0] + dx,
        pose[1] + dy,
        pose[2] + dz,
        pose[3] + da,
        pose[4] + db,
        pose[5] + dc,
    ]


class FairinoRos2Client:
    def __init__(
        self,
        service_name: str,
        state_topic: str,
        wait_service_s: float,
        wait_state_s: float,
        call_timeout_s: float,
    ) -> None:
        self.service_name = service_name
        self.state_topic = state_topic
        self.wait_service_s = wait_service_s
        self.wait_state_s = wait_state_s
        self.call_timeout_s = call_timeout_s
        self._owns_context = False
        self.node: Node | None = None
        self.client = None
        self.sub = None
        self.latest_state: RobotNonrtState | None = None

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self.node = Node("fairino_ros2_air_move_demo")
        self.client = self.node.create_client(RemoteCmdInterface, self.service_name)
        self.sub = self.node.create_subscription(
            RobotNonrtState,
            self.state_topic,
            self._state_callback,
            10,
        )

        print(f"[ROS2] 等待服务: {self.service_name}")
        deadline = time.time() + self.wait_service_s
        while time.time() < deadline:
            if self.client.wait_for_service(timeout_sec=0.5):
                break
            self.spin_once(0.05)
        if not self.client.service_is_ready():
            raise RuntimeError(f"未找到 ROS2 控制服务: {self.service_name}")

        self.wait_for_state(required=True)

    def close(self) -> None:
        if self.node is not None:
            if self.sub is not None:
                self.node.destroy_subscription(self.sub)
                self.sub = None
            self.node.destroy_node()
            self.node = None
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()
            self._owns_context = False

    def _state_callback(self, msg: RobotNonrtState) -> None:
        self.latest_state = msg

    def spin_once(self, timeout_sec: float = 0.1) -> None:
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def wait_for_state(self, required: bool) -> bool:
        deadline = time.time() + self.wait_state_s
        while time.time() < deadline:
            if self.latest_state is not None:
                return True
            self.spin_once(0.1)
        if required:
            raise RuntimeError(f"在 {self.wait_state_s:.1f}s 内未收到状态话题: {self.state_topic}")
        return False

    def call(self, cmd_str: str, raise_on_error: bool = True) -> tuple[int, str]:
        if self.node is None or self.client is None:
            raise RuntimeError("ROS2 client 尚未连接")
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd_str
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.call_timeout_s)
        if not future.done():
            raise RuntimeError(f"ROS2 指令超时: {cmd_str}")
        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"ROS2 指令异常 {cmd_str}: {exc}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"ROS2 指令无返回: {cmd_str}")
        cmd_res = result.cmd_res
        ret = parse_ret_code(cmd_res)
        if raise_on_error and ret != 0:
            raise RuntimeError(f"ROS2 指令失败 {cmd_str}: {cmd_res}")
        return ret, cmd_res

    def pose(self) -> list[float]:
        self.wait_for_state(required=True)
        state = self.latest_state
        if state is None:
            raise RuntimeError("未收到状态消息")
        return [
            float(state.cart_x_cur_pos),
            float(state.cart_y_cur_pos),
            float(state.cart_z_cur_pos),
            float(state.cart_a_cur_pos),
            float(state.cart_b_cur_pos),
            float(state.cart_c_cur_pos),
        ]

    def joints(self) -> list[float]:
        self.wait_for_state(required=True)
        state = self.latest_state
        if state is None:
            raise RuntimeError("未收到状态消息")
        return [
            float(state.j1_cur_pos),
            float(state.j2_cur_pos),
            float(state.j3_cur_pos),
            float(state.j4_cur_pos),
            float(state.j5_cur_pos),
            float(state.j6_cur_pos),
        ]

    def soft_limit_active(self) -> bool:
        state = self.latest_state
        if state is None:
            return False
        return abs(float(state.out_sflimit_err)) > EPS or int(state.exaxis_out_slimit_error) != 0

    def blocking_faults(self) -> list[str]:
        state = self.latest_state
        if state is None:
            return ["未收到状态消息"]

        issues: list[str] = []
        if int(state.emg) != 0 or int(state.btn_box_stop_signa) != 0:
            issues.append("急停信号仍然有效")
        if int(state.abnormal_stop) != 0:
            issues.append("机械臂处于异常停止")
        if abs(float(state.out_sflimit_err)) > EPS:
            issues.append(f"检测到关节软限位标志 out_sflimit_err={fmt_value(state.out_sflimit_err)}")
        if int(state.exaxis_out_slimit_error) != 0:
            issues.append("检测到外部轴软限位")
        if int(state.main_error_code) != 0 or int(state.sub_error_code) != 0:
            issues.append(
                f"主/子错误码非零: main={int(state.main_error_code)} sub={int(state.sub_error_code)}"
            )
        if int(state.alarm) != 0:
            issues.append("alarm 标志非零")
        if int(state.motionalarm) != 0:
            issues.append("motionalarm 标志非零")
        if int(state.safetyplanealarm) != 0:
            issues.append("safetyplanealarm 标志非零")
        if int(state.interferealarm) != 0:
            issues.append("interferealarm 标志非零")
        if int(state.drag_alarm) != 0:
            issues.append("drag_alarm 标志非零")
        return issues

    def print_state(self, title: str) -> None:
        self.wait_for_state(required=True)
        state = self.latest_state
        if state is None:
            return
        print(f"\n[State] {title}")
        print(f"  pose={pose_text(self.pose())}")
        print(f"  joints={pose_text(self.joints())}")
        print(
            "  mode="
            f"{int(state.robot_mode)} motion_done={int(state.robot_motion_done)} "
            f"abnormal_stop={int(state.abnormal_stop)} emg={int(state.emg)}"
        )
        print(
            "  alarm="
            f"{int(state.alarm)} motionalarm={int(state.motionalarm)} "
            f"safetyplanealarm={int(state.safetyplanealarm)} interferealarm={int(state.interferealarm)}"
        )
        print(
            "  errors="
            f"main={int(state.main_error_code)} sub={int(state.sub_error_code)} "
            f"soft_limit={fmt_value(state.out_sflimit_err)} exaxis_soft_limit={int(state.exaxis_out_slimit_error)}"
        )

    def recover(self, speed: int, reset_errors: bool) -> None:
        print("\n[Recover] 切到自动模式: Mode(0)")
        self.call("Mode(0)", raise_on_error=False)
        time.sleep(0.2)
        self.wait_for_state(required=False)

        issues = self.blocking_faults()
        if reset_errors and issues:
            print("[Recover] 检测到报警/停止标志，执行 ResetAllError()")
            self.call("ResetAllError()", raise_on_error=False)
            time.sleep(0.5)
            self.wait_for_state(required=False)

        print("[Recover] 机械臂使能: RobotEnable(1)")
        ret, cmd_res = self.call("RobotEnable(1)", raise_on_error=False)
        if ret != 0:
            raise RuntimeError(f"RobotEnable(1) 失败: {cmd_res}")

        print(f"[Recover] 设置速度: SetSpeed({speed})")
        self.call(f"SetSpeed({speed})", raise_on_error=False)
        time.sleep(0.2)
        self.wait_for_state(required=False)

    def wait_motion_done(self, motion_timeout_s: float) -> bool:
        deadline = time.time() + motion_timeout_s
        while time.time() < deadline:
            self.spin_once(0.1)
            state = self.latest_state
            if state is not None and int(state.robot_motion_done) == 1:
                return True
        return False

    def move_l(
        self,
        pose: list[float],
        speed: int,
        tool: int,
        user: int,
        point_id: int,
        motion_timeout_s: float,
    ) -> None:
        point_cmd = f"CARTPoint({point_id}," + ",".join(fmt_value(v) for v in pose) + ")"
        ret, cmd_res = self.call(point_cmd, raise_on_error=False)
        if ret != 0:
            raise RuntimeError(f"CARTPoint 失败: {cmd_res}")

        move_cmd = f"MoveL(CART{point_id},{speed},{tool},{user})"
        ret, cmd_res = self.call(move_cmd, raise_on_error=False)
        if ret != 0:
            raise RuntimeError(f"MoveL 失败: {cmd_res}")

        if not self.wait_motion_done(motion_timeout_s):
            raise RuntimeError(f"等待运动完成超时: {move_cmd}")


def bounded_speed(value: str) -> int:
    speed = int(value)
    if speed < 1 or speed > 100:
        raise argparse.ArgumentTypeError("速度必须在 1~100 之间")
    return speed


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FAIRINO ROS2 远程空中运动 demo")
    parser.add_argument("--service-name", default=DEFAULT_SERVICE_NAME, help="RemoteCmdInterface 服务名")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC, help="RobotNonrtState 话题名")
    parser.add_argument("--tool", type=int, default=int(os.environ.get("ROBOT_TOOL_ID", "0")), help="工具号")
    parser.add_argument("--user", type=int, default=int(os.environ.get("ROBOT_USER_ID", "0")), help="工件号")
    parser.add_argument("--speed", type=bounded_speed, default=10, help="运动速度百分比 1~100")
    parser.add_argument("--lift-mm", type=float, default=20.0, help="先沿基座 Z 轴抬高的距离 mm")
    parser.add_argument("--sway-mm", type=float, default=15.0, help="悬空后沿 X 或 Y 轴摆动的距离 mm")
    parser.add_argument("--axis", choices=["x", "y"], default="x", help="摆动轴，默认 x")
    parser.add_argument("--hold-s", type=float, default=0.2, help="每个到位点的停留时间 s")
    parser.add_argument("--point-id", type=int, default=1, help="CARTPoint 缓存点编号")
    parser.add_argument("--wait-service-s", type=float, default=DEFAULT_WAIT_SERVICE_S, help="等待服务秒数")
    parser.add_argument("--wait-state-s", type=float, default=DEFAULT_WAIT_STATE_S, help="等待状态秒数")
    parser.add_argument("--call-timeout-s", type=float, default=DEFAULT_CALL_TIMEOUT_S, help="单条指令超时秒数")
    parser.add_argument(
        "--motion-timeout-s",
        type=float,
        default=DEFAULT_MOTION_TIMEOUT_S,
        help="等待单段运动完成秒数",
    )
    parser.add_argument("--no-reset", action="store_true", help="启动时不执行 ResetAllError()")
    parser.add_argument("--check-only", action="store_true", help="只做状态检查和恢复，不执行运动")
    parser.add_argument("--keep-hover", action="store_true", help="运动结束停在悬空中点，不回起点")
    return parser


def main() -> int:
    args = build_argparser().parse_args()

    client = FairinoRos2Client(
        service_name=args.service_name,
        state_topic=args.state_topic,
        wait_service_s=args.wait_service_s,
        wait_state_s=args.wait_state_s,
        call_timeout_s=args.call_timeout_s,
    )

    try:
        client.connect()
        client.print_state("初始状态")

        issues = client.blocking_faults()
        if issues:
            print("[Diag] 当前检测到以下问题:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("[Diag] 当前状态未见明显报警/限位标志")

        client.recover(speed=args.speed, reset_errors=not args.no_reset)
        client.print_state("恢复后状态")

        issues = client.blocking_faults()
        if issues:
            print("[Diag] 恢复后仍存在阻塞问题:")
            for issue in issues:
                print(f"  - {issue}")
            print("[Diag] 建议先在远端控制机上确认是否已经完全退出限位区，再重新运行。")
            return 2

        if args.check_only:
            print("[Done] 已完成状态检查，未执行运动。")
            return 0

        start_pose = client.pose()
        hover_pose = offset_pose(start_pose, dz=args.lift_mm)
        if args.axis == "x":
            positive_pose = offset_pose(hover_pose, dx=args.sway_mm)
            negative_pose = offset_pose(hover_pose, dx=-args.sway_mm)
        else:
            positive_pose = offset_pose(hover_pose, dy=args.sway_mm)
            negative_pose = offset_pose(hover_pose, dy=-args.sway_mm)

        waypoints: list[tuple[str, list[float]]] = [
            ("抬高到悬空位", hover_pose),
            (f"{args.axis} 轴正向摆动", positive_pose),
            (f"{args.axis} 轴反向摆动", negative_pose),
            ("回到悬空中点", hover_pose),
        ]
        if not args.keep_hover:
            waypoints.append(("回到起点", start_pose))

        print("\n[Motion] 规划轨迹:")
        print(f"  起点: {pose_text(start_pose)}")
        for name, pose in waypoints:
            print(f"  {name}: {pose_text(pose)}")

        for name, pose in waypoints:
            print(f"\n[Motion] {name}")
            client.move_l(
                pose=pose,
                speed=args.speed,
                tool=args.tool,
                user=args.user,
                point_id=args.point_id,
                motion_timeout_s=args.motion_timeout_s,
            )
            if args.hold_s > 0:
                time.sleep(args.hold_s)
            client.print_state(f"{name} 后")

        print("\n[Done] 空中运动 demo 执行完成。")
        return 0
    except Exception as exc:
        print(f"\n[Error] {exc}", file=sys.stderr)
        state = client.latest_state
        if state is not None:
            print(
                "[Error] 当前故障摘要: "
                f"main={int(state.main_error_code)} sub={int(state.sub_error_code)} "
                f"soft_limit={fmt_value(state.out_sflimit_err)} "
                f"abnormal_stop={int(state.abnormal_stop)} alarm={int(state.alarm)}",
                file=sys.stderr,
            )
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
