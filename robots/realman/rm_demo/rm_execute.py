from __future__ import annotations

import glob
import math
import os
import sys
import time
from dataclasses import asdict
from typing import Any

from .config import (
    ROS_FORCE_POSE_CMD_TOPIC,
    ROS_FORCE_STATE_TOPIC,
    ROS_GET_SIX_FORCE_CMD_TOPIC,
    ROS_GET_SIX_FORCE_TOPIC,
    ROS_SET_FORCE_POSITION_NEW_TOPIC,
    ROS_SET_FORCE_POSITION_TOPIC,
    ROS_SET_FORCE_SENSOR_TOPIC,
    ROS_START_FORCE_POSITION_MOVE_TOPIC,
    ROS_STOP_FORCE_POSITION_MOVE_TOPIC,
    ROS_VENDOR_PYTHON_DIR,
)
from .rm_plan import StaticMassagePlan
from .rm_ros import create_arm_backend
from .rm_speed import normalize_motion_speed


def _import_ros_modules():
    try:
        import rospy  # type: ignore
        from geometry_msgs.msg import Pose  # type: ignore
        from rm_msgs.msg import Force_Position, Force_Position_Move_Pose, Force_Position_New, Force_Position_State, Six_Force  # type: ignore
        from std_msgs.msg import Bool, Empty  # type: ignore
    except Exception:
        candidates = []
        candidates.extend(glob.glob("/opt/ros/*/lib/python3/dist-packages"))
        candidates.append(ROS_VENDOR_PYTHON_DIR)
        candidates.append("/home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/lib/python3/dist-packages")
        for candidate in candidates:
            if candidate and os.path.isdir(candidate) and candidate not in sys.path:
                sys.path.append(candidate)
        import rospy  # type: ignore
        from geometry_msgs.msg import Pose  # type: ignore
        from rm_msgs.msg import Force_Position, Force_Position_Move_Pose, Force_Position_New, Force_Position_State, Six_Force  # type: ignore
        from std_msgs.msg import Bool, Empty  # type: ignore

    return rospy, Pose, Force_Position, Force_Position_Move_Pose, Force_Position_New, Force_Position_State, Six_Force, Bool, Empty


class RosForceBridge:
    def __init__(self) -> None:
        modules = _import_ros_modules()
        (
            self.rospy,
            self.Pose,
            self.Force_Position,
            self.Force_Position_Move_Pose,
            self.Force_Position_New,
            self.Force_Position_State,
            self.Six_Force,
            self.Bool,
            self.Empty,
        ) = modules
        if not self.rospy.core.is_initialized():
            self.rospy.init_node("rm_demo_force_bridge", anonymous=True, disable_signals=True)
        self.last_force: dict[str, float] | None = None
        self.last_force_time = 0.0
        self.last_force_state: dict[str, Any] | None = None
        self.last_force_state_time = 0.0
        self.last_force_result: bool | None = None
        self.last_force_result_time = 0.0

        self.sub_force = self.rospy.Subscriber(ROS_GET_SIX_FORCE_TOPIC, self.Six_Force, self._on_force, queue_size=5)
        self.sub_force_state = self.rospy.Subscriber(ROS_FORCE_STATE_TOPIC, self.Force_Position_State, self._on_force_state, queue_size=5)
        self.sub_force_result = self.rospy.Subscriber("/rm_driver/SetForcePosition_result", self.Bool, self._on_force_result, queue_size=5)

        self.pub_get_force = self.rospy.Publisher(ROS_GET_SIX_FORCE_CMD_TOPIC, self.Empty, queue_size=5)
        self.pub_set_force_sensor = self.rospy.Publisher(ROS_SET_FORCE_SENSOR_TOPIC, self.Empty, queue_size=5)
        self.pub_set_force_new = self.rospy.Publisher(ROS_SET_FORCE_POSITION_NEW_TOPIC, self.Force_Position_New, queue_size=5)
        self.pub_set_force = self.rospy.Publisher(ROS_SET_FORCE_POSITION_TOPIC, self.Force_Position, queue_size=5)
        self.pub_start_force = self.rospy.Publisher(ROS_START_FORCE_POSITION_MOVE_TOPIC, self.Empty, queue_size=5)
        self.pub_stop_force = self.rospy.Publisher(ROS_STOP_FORCE_POSITION_MOVE_TOPIC, self.Empty, queue_size=5)
        self.pub_force_pose = self.rospy.Publisher(ROS_FORCE_POSE_CMD_TOPIC, self.Force_Position_Move_Pose, queue_size=5)
        self.rospy.sleep(0.2)

    def _on_force(self, msg) -> None:
        self.last_force = {
            "fx": float(msg.force_Fx),
            "fy": float(msg.force_Fy),
            "fz": float(msg.force_Fz),
            "mx": float(msg.force_Mx),
            "my": float(msg.force_My),
            "mz": float(msg.force_Mz),
        }
        self.last_force_time = time.time()

    def _on_force_state(self, msg) -> None:
        self.last_force_state = {
            "joint": [float(v) for v in msg.joint],
            "force": float(msg.force),
            "arm_err": int(msg.arm_err),
        }
        self.last_force_state_time = time.time()

    def _on_force_result(self, msg) -> None:
        self.last_force_result = bool(msg.data)
        self.last_force_result_time = time.time()

    def request_force_sample(self, timeout: float = 0.6) -> dict[str, float] | None:
        before = time.time()
        self.pub_get_force.publish(self.Empty())
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.last_force_time >= before and self.last_force is not None:
                return dict(self.last_force)
            time.sleep(0.02)
        return None

    def enable_force_sensor(self) -> None:
        self.pub_set_force_sensor.publish(self.Empty())
        self.rospy.sleep(0.2)

    def configure_force_tracking(self, target_force_n: int, coordinate: int = 1, z_control_mode: int = 0, sensor: int = 1) -> None:
        msg = self.Force_Position_New()
        msg.sensor = int(sensor)
        msg.coordinate = int(coordinate)
        msg.z_control_mode = int(z_control_mode)
        msg.force = int(target_force_n)
        self.pub_set_force_new.publish(msg)
        self.rospy.sleep(0.2)

    def configure_force_position(self, target_force_n: int, mode: int = 0, direction: int = 2, sensor: int = 1) -> None:
        msg = self.Force_Position()
        msg.sensor = int(sensor)
        msg.mode = int(mode)
        msg.direction = int(direction)
        msg.force = int(target_force_n)
        self.pub_set_force.publish(msg)
        self.rospy.sleep(0.2)

    def start_force_position(self, wait_s: float = 0.2) -> None:
        self.pub_start_force.publish(self.Empty())
        self.rospy.sleep(max(0.0, float(wait_s)))

    def stop_force_position(self) -> None:
        self.pub_stop_force.publish(self.Empty())
        self.rospy.sleep(0.2)

    def publish_force_pose(self, pose_m: list[float], target_force_n: int, mode: int = 0, direction: int = 2, sensor: int = 1) -> None:
        msg = self.Force_Position_Move_Pose()
        msg.Pose = self.Pose()
        msg.Pose.position.x = float(pose_m[0])
        msg.Pose.position.y = float(pose_m[1])
        msg.Pose.position.z = float(pose_m[2])
        msg.Pose.orientation.x = float(pose_m[3])
        msg.Pose.orientation.y = float(pose_m[4])
        msg.Pose.orientation.z = float(pose_m[5])
        msg.Pose.orientation.w = 0.0
        msg.sensor = int(sensor)
        msg.mode = int(mode)
        msg.dir = int(direction)
        msg.force = int(target_force_n)
        self.pub_force_pose.publish(msg)


def _sample_force(bridge: RosForceBridge | None) -> dict[str, float] | None:
    if bridge is None:
        return None
    return bridge.request_force_sample(timeout=0.8)


def _print_force(prefix: str, sample: dict[str, float] | None) -> None:
    if sample is None:
        print(f"{prefix} force=n/a")
        return
    print(
        f"{prefix} "
        f"Fx={sample['fx']:.2f} Fy={sample['fy']:.2f} Fz={sample['fz']:.2f} "
        f"Mx={sample['mx']:.2f} My={sample['my']:.2f} Mz={sample['mz']:.2f}"
    )


def _force_delta_n(sample: dict[str, float] | None, baseline: dict[str, float] | None) -> float | None:
    if sample is None or baseline is None:
        return None
    dfx = float(sample["fx"] - baseline["fx"])
    dfy = float(sample["fy"] - baseline["fy"])
    dfz = float(sample["fz"] - baseline["fz"])
    return float(math.sqrt(dfx * dfx + dfy * dfy + dfz * dfz))


def _lerp_pose(start_pose: list[float], end_pose: list[float], ratio: float) -> list[float]:
    t = float(max(0.0, min(1.0, ratio)))
    return [float(start_pose[i]) * (1.0 - t) + float(end_pose[i]) * t for i in range(6)]


def _touch_monitor_point(
    arm,
    host: str,
    point,
    speed: int,
    bridge: RosForceBridge,
    target_force_n: int,
    max_force_n: int,
    touch_step_m: float,
    max_press_m: float,
) -> None:
    baseline = _sample_force(bridge)
    _print_force(f"point {point.index} baseline", baseline)

    descent_m = math.sqrt(
        sum((float(point.work_pose_m[i]) - float(point.approach_pose_m[i])) ** 2 for i in range(3))
    )
    steps = max(1, int(math.ceil(descent_m / max(0.001, float(touch_step_m)))))
    reached_target = False

    for step in range(1, steps + 1):
        pose = _lerp_pose(point.approach_pose_m, point.work_pose_m, step / steps)
        arm.movel(host, pose, speed=max(1, speed), timeout=20.0)
        sample = _sample_force(bridge)
        delta_n = _force_delta_n(sample, baseline)
        _print_force(f"point {point.index} probe {step}/{steps}", sample)
        if delta_n is not None:
            print(f"point {point.index} probe_delta={delta_n:.2f}N")
            if delta_n >= float(max_force_n):
                print(f"point {point.index} max_force_reached={delta_n:.2f}N")
                break
            if delta_n >= float(target_force_n):
                print(f"point {point.index} target_force_reached={delta_n:.2f}N")
                reached_target = True
                break

    if not reached_target and point.press_direction_m:
        extra_steps = max(1, int(math.ceil(float(max_press_m) / max(0.001, float(touch_step_m)))))
        for step in range(1, extra_steps + 1):
            pose = list(point.work_pose_m)
            pose[0] += float(point.press_direction_m[0]) * float(touch_step_m) * step
            pose[1] += float(point.press_direction_m[1]) * float(touch_step_m) * step
            pose[2] += float(point.press_direction_m[2]) * float(touch_step_m) * step
            arm.movel(host, pose, speed=max(1, speed), timeout=20.0)
            sample = _sample_force(bridge)
            delta_n = _force_delta_n(sample, baseline)
            _print_force(f"point {point.index} extra_probe {step}/{extra_steps}", sample)
            if delta_n is not None:
                print(f"point {point.index} extra_probe_delta={delta_n:.2f}N")
                if delta_n >= float(max_force_n):
                    print(f"point {point.index} max_force_reached={delta_n:.2f}N")
                    break
                if delta_n >= float(target_force_n):
                    print(f"point {point.index} target_force_reached={delta_n:.2f}N")
                    reached_target = True
                    break

    if reached_target:
        dwell_deadline = time.time() + point.dwell_s
        while time.time() < dwell_deadline:
            sample = _sample_force(bridge)
            delta_n = _force_delta_n(sample, baseline)
            _print_force(f"point {point.index} contact", sample)
            if delta_n is not None:
                print(f"point {point.index} contact_delta={delta_n:.2f}N")
            time.sleep(0.15)


def preview_plan(plan: StaticMassagePlan) -> None:
    print(
        f"side={plan.side} points={plan.point_count} hover_m={plan.hover_m:.4f} "
        f"safe_z_m={plan.safe_z_m:.4f}"
    )
    for point in plan.points:
        print(
            f"{point.index:02d}: "
            f"robot={point.robot_point_m} "
            f"approach={point.approach_pose_m[:3]} work={point.work_pose_m[:3]}"
        )


def execute_plan(
    host: str,
    plan: StaticMassagePlan,
    speed: int,
    monitor_force: bool,
    target_force_n: int,
    mode: str,
    control_backend: str = "json",
    force_direction: int = 2,
    force_mode: int = 0,
    max_force_n: int = 12,
    touch_step_m: float = 0.003,
    max_press_m: float = 0.01,
) -> None:
    arm = create_arm_backend(control_backend)
    motion_speed = normalize_motion_speed(control_backend, float(speed), ros_default=0.3)
    arm.recover_if_needed(host)
    _, current_pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(host)
    print(
        f"current_pose={[round(v, 6) for v in current_pose]} "
        f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
    )
    bridge = RosForceBridge() if monitor_force or mode in ("ros_force_pose", "touch_monitor") else None
    if bridge is not None:
        bridge.enable_force_sensor()
        _print_force("initial", _sample_force(bridge))

    if current_pose[2] < plan.safe_z_m:
        lift_pose = [float(current_pose[0]), float(current_pose[1]), float(plan.safe_z_m)] + [float(v) for v in current_pose[3:6]]
        arm.movel(host, lift_pose, speed=motion_speed, timeout=20.0)

    if mode == "ros_force_pose" and bridge is not None:
        bridge.configure_force_tracking(target_force_n=target_force_n)
        bridge.configure_force_position(target_force_n=target_force_n, mode=force_mode, direction=force_direction)
        bridge.start_force_position()

    try:
        for point in plan.points:
            print(f"point {point.index}/{plan.point_count} approach={point.approach_pose_m[:3]}")
            arm.movel(host, point.approach_pose_m, speed=motion_speed, timeout=20.0)
            _print_force(f"point {point.index} hover", _sample_force(bridge))

            if mode == "hover":
                continue

            if mode == "ros_force_pose" and bridge is not None:
                bridge.publish_force_pose(point.work_pose_m, target_force_n=target_force_n, mode=force_mode, direction=force_direction)
                time.sleep(max(0.2, point.dwell_s))
            elif mode == "touch_monitor" and bridge is not None:
                _touch_monitor_point(
                    arm=arm,
                    host=host,
                    point=point,
                    speed=speed,
                    bridge=bridge,
                    target_force_n=target_force_n,
                    max_force_n=max_force_n,
                    touch_step_m=touch_step_m,
                    max_press_m=max_press_m,
                )
            else:
                arm.movel(host, point.work_pose_m, speed=max(1, speed), timeout=20.0)
                dwell_deadline = time.time() + point.dwell_s
                while time.time() < dwell_deadline:
                    _print_force(f"point {point.index} contact", _sample_force(bridge))
                    time.sleep(0.15)

            arm.movel(host, point.retreat_pose_m, speed=max(2, speed), timeout=20.0)
            _print_force(f"point {point.index} retreat", _sample_force(bridge))
    finally:
        if mode == "ros_force_pose" and bridge is not None:
            bridge.stop_force_position()


def plan_summary(plan: StaticMassagePlan) -> dict[str, object]:
    return asdict(plan)
