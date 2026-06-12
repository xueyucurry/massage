from __future__ import annotations

import glob
import math
import os
import subprocess
import sys
import tempfile
import time

from .config import (
    ROS_ARM_CURRENT_STATE_TOPIC,
    ROS_ARM_STATE_TOPIC,
    ROS_GET_ARM_STATE_CMD_TOPIC,
    ROS_MOVEL_CMD_TOPIC,
    ROS_MOVEJ_CMD_TOPIC,
    ROS_MOVEJ_P_CMD_TOPIC,
    ROS_STOP_CMD_TOPIC,
    ROS_VENDOR_PYTHON_DIR,
)


def _append_ros_python_candidates() -> None:
    candidates: list[str] = []
    candidates.extend(glob.glob("/opt/ros/*/lib/python3/dist-packages"))
    extra_pythonpath = os.environ.get("RM_DEMO_ROS_PYTHONPATH", "").strip()
    if extra_pythonpath:
        candidates.extend(path for path in extra_pythonpath.split(":") if path)
    candidates.append(ROS_VENDOR_PYTHON_DIR)
    for candidate in candidates:
        if candidate and os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.append(candidate)


def _import_ros_modules():
    try:
        import rospy  # type: ignore
        from geometry_msgs.msg import Pose  # type: ignore
        from rm_msgs.msg import (  # type: ignore
            ArmState,
            Arm_Current_State,
            ChangeWorkFrame_Name,
            ChangeWorkFrame_State,
            GetArmState_Command,
            MoveJ,
            MoveJ_P,
            MoveL,
            Stop,
        )
        from rm_msgs.srv import Change_Tool_Frame_Srv  # type: ignore
        from std_msgs.msg import Empty  # type: ignore
    except Exception:
        _append_ros_python_candidates()
        import rospy  # type: ignore
        from geometry_msgs.msg import Pose  # type: ignore
        from rm_msgs.msg import (  # type: ignore
            ArmState,
            Arm_Current_State,
            ChangeWorkFrame_Name,
            ChangeWorkFrame_State,
            GetArmState_Command,
            MoveJ,
            MoveJ_P,
            MoveL,
            Stop,
        )
        from rm_msgs.srv import Change_Tool_Frame_Srv  # type: ignore
        from std_msgs.msg import Empty  # type: ignore

    return (
        rospy,
        Pose,
        ArmState,
        Arm_Current_State,
        ChangeWorkFrame_Name,
        ChangeWorkFrame_State,
        Change_Tool_Frame_Srv,
        GetArmState_Command,
        MoveJ,
        MoveJ_P,
        MoveL,
        Stop,
        Empty,
    )


def _pose_close(current_pose: list[float], target_pose: list[float], pos_tol_m: float = 0.003, ang_tol_rad: float = 0.02) -> bool:
    pos_ok = all(abs(float(current_pose[i]) - float(target_pose[i])) <= float(pos_tol_m) for i in range(3))
    ang_ok = all(abs(float(current_pose[i]) - float(target_pose[i])) <= float(ang_tol_rad) for i in range(3, 6))
    return pos_ok and ang_ok


def _joint_close(current_joint_deg: list[float], target_joint_deg: list[float], tol_deg: float = 0.5) -> bool:
    return all(abs(float(curr) - float(tgt)) <= float(tol_deg) for curr, tgt in zip(current_joint_deg, target_joint_deg))


def _is_planning_residue(arm_err: int, sys_err: int) -> bool:
    # 4116 is reported by rm_driver after a failed IK/planning request. It can
    # remain in state even when joints are enabled and no system fault exists.
    return int(arm_err) == 4116 and int(sys_err) == 0


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    cr = math.cos(float(roll) * 0.5)
    sr = math.sin(float(roll) * 0.5)
    cp = math.cos(float(pitch) * 0.5)
    sp = math.sin(float(pitch) * 0.5)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [float(qx), float(qy), float(qz), float(qw)]


def _quat_to_rpy(quat_xyzw: list[float]) -> list[float]:
    qx, qy, qz, qw = [float(v) for v in quat_xyzw[:4]]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [float(roll), float(pitch), float(yaw)]


class RosArmBridge:
    def __init__(self) -> None:
        (
            self.rospy,
            self.Pose,
            self.ArmState,
            self.Arm_Current_State,
            self.ChangeWorkFrame_Name,
            self.ChangeWorkFrame_State,
            self.Change_Tool_Frame_Srv,
            self.GetArmState_Command,
            self.MoveJ,
            self.MoveJ_P,
            self.MoveL,
            self.Stop,
            self.Empty,
        ) = _import_ros_modules()
        if not self.rospy.core.is_initialized():
            self.rospy.init_node("rm_demo_arm_bridge", anonymous=True, disable_signals=True)

        self.last_state: tuple[list[float], list[float], int, int, int] | None = None
        self.last_state_time = 0.0
        self.last_arm_state: tuple[list[float], list[float], int, int, int] | None = None
        self.last_arm_state_time = 0.0
        self.last_current_state: tuple[list[float], list[float], int, int, int] | None = None
        self.last_current_state_time = 0.0
        self.last_change_work_frame_state: bool | None = None
        self.last_change_work_frame_time = 0.0

        self._state_topic_specs = self._build_state_topic_specs()
        self._subscribers = [
            self.rospy.Subscriber(topic, msg_type, callback, queue_size=5)
            for topic, msg_type, callback in self._state_topic_specs
        ]
        self.pub_get_arm_state = self.rospy.Publisher(ROS_GET_ARM_STATE_CMD_TOPIC, self.GetArmState_Command, queue_size=5)
        self.pub_get_current_arm_state = self.rospy.Publisher("/rm_driver/GetCurrentArmState", self.Empty, queue_size=5)
        self.pub_movej = self.rospy.Publisher(ROS_MOVEJ_CMD_TOPIC, self.MoveJ, queue_size=5)
        self.pub_movej_p = self.rospy.Publisher(ROS_MOVEJ_P_CMD_TOPIC, self.MoveJ_P, queue_size=5)
        self.pub_movel = self.rospy.Publisher(ROS_MOVEL_CMD_TOPIC, self.MoveL, queue_size=5)
        self.pub_stop = self.rospy.Publisher(ROS_STOP_CMD_TOPIC, self.Stop, queue_size=5)
        self.pub_change_work_frame = self.rospy.Publisher("/rm_driver/ChangeWorkFrame_Cmd", self.ChangeWorkFrame_Name, queue_size=5)
        self.srv_change_tool = self.rospy.ServiceProxy("/change_arm_tool_frame", self.Change_Tool_Frame_Srv)
        self.rospy.sleep(0.2)

    def _build_state_topic_specs(self):
        specs = [
            (ROS_ARM_CURRENT_STATE_TOPIC, self.ArmState, self._on_arm_state),
            ("/rm_driver/ArmCurrentState", self.ArmState, self._on_arm_state),
            ("/rm_driver/Arm_Current_State", self.Arm_Current_State, self._on_arm_current_state),
            (ROS_ARM_STATE_TOPIC, self.ArmState, self._on_arm_state),
            ("/rm_driver/ChangeWorkFrame_State", self.ChangeWorkFrame_State, self._on_change_work_frame_state),
        ]
        seen: set[tuple[str, str]] = set()
        out = []
        for topic, msg_type, callback in specs:
            key = (str(topic), getattr(msg_type, "_type", msg_type.__name__))
            if key in seen:
                continue
            seen.add(key)
            out.append((str(topic), msg_type, callback))
        return out

    def _on_arm_state(self, msg) -> None:
        joints = [float(math.degrees(v)) for v in msg.joint]
        pose = [
            float(msg.Pose.position.x),
            float(msg.Pose.position.y),
            float(msg.Pose.position.z),
        ] + _quat_to_rpy(
            [
                float(msg.Pose.orientation.x),
                float(msg.Pose.orientation.y),
                float(msg.Pose.orientation.z),
                float(msg.Pose.orientation.w),
            ]
        )
        state = (joints, pose, int(msg.arm_err), int(msg.sys_err), -1)
        now = time.time()
        self.last_arm_state = state
        self.last_arm_state_time = now
        self.last_state = state
        self.last_state_time = now

    def _on_arm_current_state(self, msg) -> None:
        joints = [float(v) for v in msg.joint]
        pose = [float(v) for v in msg.Pose]
        state = (joints, pose, int(msg.arm_err), int(msg.sys_err), -1)
        now = time.time()
        self.last_current_state = state
        self.last_current_state_time = now
        self.last_state = state
        self.last_state_time = now

    def _on_change_work_frame_state(self, msg) -> None:
        self.last_change_work_frame_state = bool(msg.state)
        self.last_change_work_frame_time = time.time()

    def _request_arm_state(self) -> None:
        msg = self.GetArmState_Command()
        msg.command = "get_current_arm_state"
        self.pub_get_arm_state.publish(msg)
        self.pub_get_current_arm_state.publish(self.Empty())
        self.rospy.sleep(0.05)

    def _latest_state_since(
        self,
        before_time: float,
    ) -> tuple[list[float], list[float], int, int, int] | None:
        if self.last_current_state is not None and self.last_current_state_time >= before_time:
            return self.last_current_state
        if self.last_arm_state is not None and self.last_arm_state_time >= before_time:
            return self.last_arm_state
        if self.last_state is not None and self.last_state_time >= before_time:
            return self.last_state
        return None

    def _read_state_via_rostopic_cli(
        self,
        timeout: float,
    ) -> tuple[list[float], list[float], int, int, int] | None:
        env = dict(os.environ)
        pythonpath = env.get("PYTHONPATH", "")
        vendor_parts = [ROS_VENDOR_PYTHON_DIR]
        if pythonpath:
            vendor_parts.append(pythonpath)
        env["PYTHONPATH"] = ":".join(vendor_parts)
        with tempfile.NamedTemporaryFile(prefix="rm_arm_state_", suffix=".txt", delete=False) as f:
            tmp_path = f.name
        try:
            cmd = (
                f"timeout {max(2, int(math.ceil(timeout + 2.0)))} "
                f"rostopic echo -n 1 /rm_driver/ArmCurrentState > {tmp_path} & "
                "pid=$!; "
                "sleep 1; "
                "rostopic pub -1 /rm_driver/GetCurrentArmState std_msgs/Empty '{}' >/dev/null 2>&1 || true; "
                "sleep 1; "
                "wait $pid >/dev/null 2>&1 || true; "
                f"cat {tmp_path}"
            )
            proc = subprocess.run(
                ["bash", "-lc", cmd],
                env=env,
                text=True,
                capture_output=True,
                timeout=max(6.0, float(timeout) + 4.0),
                check=False,
            )
            text = (proc.stdout or "").strip()
            if not text:
                return None
            try:
                import yaml  # type: ignore

                data = yaml.safe_load(text)
            except Exception:
                return None
            if not isinstance(data, dict):
                return None
            joint_rad = [float(v) for v in list(data.get("joint", []))[:6]]
            pose_data = data.get("Pose", {}) or {}
            pos = pose_data.get("position", {}) or {}
            ori = pose_data.get("orientation", {}) or {}
            if len(joint_rad) < 6:
                return None
            pose = [
                float(pos.get("x", 0.0)),
                float(pos.get("y", 0.0)),
                float(pos.get("z", 0.0)),
            ] + _quat_to_rpy(
                [
                    float(ori.get("x", 0.0)),
                    float(ori.get("y", 0.0)),
                    float(ori.get("z", 0.0)),
                    float(ori.get("w", 1.0)),
                ]
            )
            joints_deg = [float(math.degrees(v)) for v in joint_rad]
            state = (
                joints_deg,
                pose,
                int(data.get("arm_err", 0)),
                int(data.get("sys_err", 0)),
                -1,
            )
            self.last_state = state
            self.last_state_time = time.time()
            return state
        finally:
            try:
                os.remove(tmp_path)
            except OSError:
                pass

    def _change_work_frame_via_rostopic_cli(self, work_frame_name: str, timeout: float) -> bool | None:
        env = dict(os.environ)
        pythonpath = env.get("PYTHONPATH", "")
        vendor_parts = [ROS_VENDOR_PYTHON_DIR]
        if pythonpath:
            vendor_parts.append(pythonpath)
        env["PYTHONPATH"] = ":".join(vendor_parts)
        with tempfile.NamedTemporaryFile(prefix="rm_change_work_frame_", suffix=".txt", delete=False) as f:
            tmp_path = f.name
        try:
            timeout_s = max(3, int(math.ceil(float(timeout) + 2.0)))
            cmd = (
                f"timeout {timeout_s} "
                f"rostopic echo -n 1 /rm_driver/ChangeWorkFrame_State > {tmp_path} & "
                "pid=$!; "
                "sleep 1; "
                "rostopic pub -1 /rm_driver/ChangeWorkFrame_Cmd rm_msgs/ChangeWorkFrame_Name "
                f"\"{{WorkFrame_name: '{str(work_frame_name)}'}}\" >/dev/null 2>&1 || true; "
                "sleep 1; "
                "wait $pid >/dev/null 2>&1 || true; "
                f"cat {tmp_path}"
            )
            proc = subprocess.run(
                ["bash", "-lc", cmd],
                env=env,
                text=True,
                capture_output=True,
                timeout=max(8.0, float(timeout) + 5.0),
                check=False,
            )
            text = (proc.stdout or "").strip()
            if not text:
                return None
            try:
                import yaml  # type: ignore

                data = yaml.safe_load(text)
            except Exception:
                return None
            if not isinstance(data, dict):
                return None
            if "state" not in data:
                return None
            return bool(data.get("state"))
        finally:
            try:
                os.remove(tmp_path)
            except OSError:
                pass

    def can_connect(self, host: str | None = None, timeout: float = 1.5) -> bool:
        try:
            self.get_current_arm_state(host=host, timeout=timeout)
            return True
        except Exception:
            return False

    def get_current_arm_state(
        self,
        host: str | None = None,
        timeout: float = 1.5,
    ) -> tuple[list[float], list[float], int, int, int]:
        del host
        before_time = time.time()
        self._request_arm_state()
        end_time = time.time() + float(timeout)
        next_request_time = time.time() + 0.25
        while time.time() < end_time:
            state = self._latest_state_since(before_time)
            if state is not None:
                return state
            if time.time() >= next_request_time:
                self._request_arm_state()
                next_request_time = time.time() + 0.25
            self.rospy.sleep(0.05)
        fallback = self._read_state_via_rostopic_cli(timeout=float(timeout))
        if fallback is not None:
            return fallback
        raise RuntimeError(
            "ROS arm state timed out; "
            f"master={os.environ.get('ROS_MASTER_URI', '')} "
            f"topics=({ROS_GET_ARM_STATE_CMD_TOPIC}, {ROS_ARM_CURRENT_STATE_TOPIC}, {ROS_ARM_STATE_TOPIC})"
        )

    def recover_if_needed(self, host: str | None = None) -> None:
        _, _, arm_err, sys_err, inverse_km_err = self.get_current_arm_state(host=host, timeout=1.5)
        if arm_err == 0 and sys_err == 0 and inverse_km_err in (0, -1):
            return

        if host:
            try:
                from . import rm_json

                rm_json.recover_if_needed(str(host))
                time.sleep(0.4)
                _, _, arm_err, sys_err, inverse_km_err = self.get_current_arm_state(host=host, timeout=2.0)
                if arm_err == 0 and sys_err == 0 and inverse_km_err in (0, -1):
                    return
            except Exception as exc:
                raise RuntimeError(
                    "arm state is not clean over ROS and JSON recovery failed: "
                    f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}; "
                    f"recovery_error={type(exc).__name__}: {exc}"
                ) from exc

        raise RuntimeError(
            "arm state is not clean over ROS: "
            f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
        )

    def movej(self, host: str | None, joint_deg: list[float], speed: int, timeout: float = 18.0) -> dict[str, object]:
        msg = self.MoveJ()
        msg.joint = [float(math.radians(float(v))) for v in joint_deg[:6]]
        msg.speed = float(speed)
        connect_deadline = time.time() + 2.0
        while self.pub_movej.get_num_connections() <= 0 and time.time() < connect_deadline:
            self.rospy.sleep(0.05)
        for _ in range(3):
            self.pub_movej.publish(msg)
            self.rospy.sleep(0.05)
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            joints, _, arm_err, sys_err, _ = self.get_current_arm_state(host=host, timeout=0.8)
            if (arm_err != 0 or sys_err != 0) and not _is_planning_residue(arm_err, sys_err):
                raise RuntimeError(f"movej failed over ROS: arm_err={arm_err} sys_err={sys_err}")
            if _joint_close(joints, joint_deg):
                return {"state": "ros_polled_joint_state", "trajectory_state": True}
            time.sleep(0.08)
        raise RuntimeError(f"movej did not reach target over ROS: target={joint_deg}")

    def movej_p(
        self,
        host: str | None,
        pose: list[float],
        speed: int,
        timeout: float = 24.0,
    ) -> dict[str, object]:
        msg = self.MoveJ_P()
        quat = _rpy_to_quat(float(pose[3]), float(pose[4]), float(pose[5]))
        msg.Pose = self.Pose()
        msg.Pose.position.x = float(pose[0])
        msg.Pose.position.y = float(pose[1])
        msg.Pose.position.z = float(pose[2])
        msg.Pose.orientation.x = float(quat[0])
        msg.Pose.orientation.y = float(quat[1])
        msg.Pose.orientation.z = float(quat[2])
        msg.Pose.orientation.w = float(quat[3])
        msg.speed = float(speed)
        connect_deadline = time.time() + 2.0
        while self.pub_movej_p.get_num_connections() <= 0 and time.time() < connect_deadline:
            self.rospy.sleep(0.05)
        for _ in range(3):
            self.pub_movej_p.publish(msg)
            self.rospy.sleep(0.05)
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            _, current_pose, arm_err, sys_err, _ = self.get_current_arm_state(host=host, timeout=0.8)
            if (arm_err != 0 or sys_err != 0) and not _is_planning_residue(arm_err, sys_err):
                raise RuntimeError(f"movej_p failed over ROS: arm_err={arm_err} sys_err={sys_err}")
            if _pose_close(current_pose, pose):
                return {"state": "ros_polled_pose_state_movej_p", "trajectory_state": True}
            time.sleep(0.08)
        if host:
            try:
                from . import rm_json

                _, current_pose, arm_err, sys_err, _ = rm_json.get_current_arm_state(str(host))
                if (arm_err == 0 and sys_err == 0) and _pose_close(current_pose, pose):
                    return {"state": "json_fallback_pose_state_movej_p", "trajectory_state": True}
            except Exception:
                pass
        raise RuntimeError(f"movej_p did not reach target over ROS: target={pose}")

    def movel(
        self,
        host: str | None,
        pose: list[float],
        speed: int,
        blend_radius: float = 0.0,
        timeout: float = 18.0,
    ) -> dict[str, object]:
        del blend_radius
        msg = self.MoveL()
        quat = _rpy_to_quat(float(pose[3]), float(pose[4]), float(pose[5]))
        msg.Pose = self.Pose()
        msg.Pose.position.x = float(pose[0])
        msg.Pose.position.y = float(pose[1])
        msg.Pose.position.z = float(pose[2])
        msg.Pose.orientation.x = float(quat[0])
        msg.Pose.orientation.y = float(quat[1])
        msg.Pose.orientation.z = float(quat[2])
        msg.Pose.orientation.w = float(quat[3])
        msg.speed = float(speed)
        msg.trajectory_connect = 0
        connect_deadline = time.time() + 2.0
        while self.pub_movel.get_num_connections() <= 0 and time.time() < connect_deadline:
            self.rospy.sleep(0.05)
        for _ in range(3):
            self.pub_movel.publish(msg)
            self.rospy.sleep(0.05)
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            _, current_pose, arm_err, sys_err, _ = self.get_current_arm_state(host=host, timeout=0.8)
            if (arm_err != 0 or sys_err != 0) and not _is_planning_residue(arm_err, sys_err):
                raise RuntimeError(f"movel failed over ROS: arm_err={arm_err} sys_err={sys_err}")
            if _pose_close(current_pose, pose):
                return {"state": "ros_polled_pose_state", "trajectory_state": True}
            time.sleep(0.08)
        if host:
            try:
                from . import rm_json

                _, current_pose, arm_err, sys_err, _ = rm_json.get_current_arm_state(str(host))
                if (arm_err == 0 and sys_err == 0) and _pose_close(current_pose, pose):
                    return {"state": "json_fallback_pose_state_movel", "trajectory_state": True}
            except Exception:
                pass
        raise RuntimeError(f"movel did not reach target over ROS: target={pose}")

    def stop_motion(self, host: str | None = None) -> dict[str, object]:
        msg = self.Stop()
        msg.state = True
        self.pub_stop.publish(msg)
        return {"state": "ros_stop_published"}

    def change_work_frame(self, work_frame_name: str, timeout: float = 3.0) -> dict[str, object]:
        before_time = time.time()
        self.last_change_work_frame_state = None
        msg = self.ChangeWorkFrame_Name()
        msg.WorkFrame_name = str(work_frame_name)
        connect_deadline = time.time() + min(1.0, float(timeout))
        while self.pub_change_work_frame.get_num_connections() <= 0 and time.time() < connect_deadline:
            self.rospy.sleep(0.05)
        self.pub_change_work_frame.publish(msg)
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            if self.last_change_work_frame_time >= before_time and self.last_change_work_frame_state is not None:
                if not self.last_change_work_frame_state:
                    raise RuntimeError(f"change_work_frame failed: {work_frame_name}")
                return {"state": "ros_change_work_frame_ok", "work_frame_name": str(work_frame_name)}
            self.rospy.sleep(0.05)
        fallback = self._change_work_frame_via_rostopic_cli(work_frame_name=work_frame_name, timeout=float(timeout))
        if fallback is True:
            self.rospy.sleep(0.2)
            return {"state": "ros_change_work_frame_ok_cli", "work_frame_name": str(work_frame_name)}
        if fallback is False:
            raise RuntimeError(f"change_work_frame failed: {work_frame_name}")
        raise RuntimeError(f"change_work_frame timed out: {work_frame_name}")

    def change_tool(self, tool_name: str, timeout: float = 5.0) -> dict[str, object]:
        self.rospy.wait_for_service("/change_arm_tool_frame", timeout=timeout)
        response = self.srv_change_tool(str(tool_name))
        if not bool(response.state):
            raise RuntimeError(f"change_tool failed: {tool_name}")
        self.rospy.sleep(0.2)
        return {"state": "ros_change_tool_ok", "tool_name": str(tool_name)}


def create_arm_backend(control_backend: str):
    backend = control_backend.strip().lower()
    if backend == "json":
        from . import rm_json

        return rm_json
    if backend == "ros":
        return RosArmBridge()
    raise ValueError(f"unsupported control backend: {control_backend}")
