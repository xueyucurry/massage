#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import subprocess
import threading
import textwrap
import time
from dataclasses import asdict
from typing import Any

import cv2
import numpy as np
import roslibpy

from rm_demo.config import (
    DEFAULT_CAMERA_TOOL_NAME,
    DEFAULT_CAPTURE_PREPARE_SECTION,
    DEFAULT_CAPTURE_SETTLE_S,
    DEFAULT_FINGER_WIDTH_MM,
    DEFAULT_HOVER_MM,
    DEFAULT_INSTALL_ANG,
    DEFAULT_OUTPUT_DIR,
    DEFAULT_PLAN_POINTS,
    DEFAULT_POSITION_SPEED,
    DEFAULT_RESTORE_TOOL_NAME,
    DEFAULT_SAFE_LIFT_MM,
    DEFAULT_SAMPLE_POINTS,
    DEFAULT_SHIFTING_NUMBER,
    DEFAULT_SPEED,
    DEFAULT_TRAJECTORY_CONFIG,
)
from rm_demo.rm_bladder import (
    bladder_plan_to_dict,
    build_bladder_massage_plan,
    detect_bladder_lines,
    preview_bladder_plan,
    save_bladder_artifacts,
    select_bladder_line,
)
from rm_demo.rm_positioning import load_prepare_joints
from rm_demo.rm_speed import normalize_motion_speed


DEFAULT_BOARD_HOST = os.environ.get("RM_BOARD_HOST", "192.168.1.11")
DEFAULT_ROSBRIDGE_PORT = int(os.environ.get("RM_ROSBRIDGE_PORT", "9090"))
DEFAULT_ARM_HOST = os.environ.get("RM_ARM_HOST", "192.168.58.2")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Local-only RealMan product bladder demo over rosbridge"
    )
    parser.add_argument("--board-host", default=DEFAULT_BOARD_HOST, help="board IP that runs rosbridge")
    parser.add_argument("--board-ssh", default="", help="board SSH target, default rm@<board-host>")
    parser.add_argument("--rosbridge-port", type=int, default=DEFAULT_ROSBRIDGE_PORT, help="rosbridge websocket port")
    parser.add_argument(
        "--frame-source",
        choices=("ssh_snapshot", "rosbridge_stream"),
        default="ssh_snapshot",
        help="image source: default uses SSH one-shot capture without writing files on board",
    )
    parser.add_argument("--host", default=DEFAULT_ARM_HOST, help="RM controller IP (metadata only)")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="local artifact directory")
    parser.add_argument("--side", choices=("left", "right"), default="left", help="meridian side to execute")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer", help="bladder meridian layer")
    parser.add_argument("--finger-width", type=float, default=DEFAULT_FINGER_WIDTH_MM, help="base offset in mm")
    parser.add_argument("--sample-points", type=int, default=DEFAULT_SAMPLE_POINTS, help="points sampled along selected line")
    parser.add_argument("--plan-points", type=int, default=DEFAULT_PLAN_POINTS, help="massage points executed along one side")
    parser.add_argument("--hover-mm", type=float, default=DEFAULT_HOVER_MM, help="hover height above body surface")
    parser.add_argument("--dian-jin-depth-mm", type=float, default=8.0, help="press depth for dian jin")
    parser.add_argument("--fen-jin-lateral-mm", type=float, default=15.0, help="lateral split offset for fen jin")
    parser.add_argument("--safe-lift-mm", type=float, default=DEFAULT_SAFE_LIFT_MM, help="safe lift before transit")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="motion speed")
    parser.add_argument(
        "--capture-positioning",
        choices=("none", "prepare", "service", "prepare_then_service"),
        default="service",
        help="pre-position camera above body using product flow",
    )
    parser.add_argument("--trajectory-config", default=DEFAULT_TRAJECTORY_CONFIG, help="local trajectory_generate.yaml path")
    parser.add_argument(
        "--capture-prepare-section",
        default=DEFAULT_CAPTURE_PREPARE_SECTION,
        help="trajectory config section used by --capture-positioning prepare",
    )
    parser.add_argument(
        "--capture-joints",
        nargs=6,
        type=float,
        default=None,
        metavar=("J0", "J1", "J2", "J3", "J4", "J5"),
        help="override prepare section with 6 capture-pose joint angles in degrees",
    )
    parser.add_argument("--position-speed", type=float, default=DEFAULT_POSITION_SPEED, help="speed for prepare joint move")
    parser.add_argument("--camera-tool-name", default=DEFAULT_CAMERA_TOOL_NAME, help="tool frame name for camera")
    parser.add_argument("--restore-tool-name", default=DEFAULT_RESTORE_TOOL_NAME, help="tool frame restored after service move")
    parser.add_argument("--shifting-number", type=int, default=DEFAULT_SHIFTING_NUMBER, help="move_camera_above_person shifting_number")
    parser.add_argument("--capture-settle-s", type=float, default=DEFAULT_CAPTURE_SETTLE_S, help="settle time after positioning")
    parser.add_argument(
        "--install-ang",
        nargs=3,
        type=float,
        default=DEFAULT_INSTALL_ANG,
        metavar=("RX", "RY", "RZ"),
        help="robot install angles for calc_poses service",
    )
    parser.add_argument("--dian-jin-dwell-s", type=float, default=0.5, help="dwell after dian jin press")
    parser.add_argument("--fen-jin-dwell-s", type=float, default=0.3, help="dwell at each fen jin side pose")
    parser.add_argument("--shun-jin-dwell-s", type=float, default=0.0, help="optional dwell on shun jin path")
    parser.add_argument("--snapshot-period-s", type=float, default=0.8, help="snapshot refresh period for ssh_snapshot mode")
    return parser.parse_args()


def _save_json(path: str, data: dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _decode_uint8_array(data_field) -> bytes:
    if isinstance(data_field, str):
        try:
            return base64.b64decode(data_field)
        except Exception:
            return bytes()
    if isinstance(data_field, list):
        return bytes(int(v) & 0xFF for v in data_field)
    return bytes()


def _image_msg_to_array(msg: dict[str, Any]) -> np.ndarray:
    height = int(msg["height"])
    width = int(msg["width"])
    encoding = str(msg["encoding"])
    raw = _decode_uint8_array(msg["data"])
    if encoding == "bgr8":
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)
        return arr.copy()
    if encoding == "rgb8":
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if encoding in ("16UC1", "mono16"):
        arr = np.frombuffer(raw, dtype=np.uint16).reshape(height, width)
        return arr.copy()
    if encoding in ("32FC1",):
        arr = np.frombuffer(raw, dtype=np.float32).reshape(height, width)
        return arr.copy()
    raise RuntimeError(f"unsupported image encoding: {encoding}")


def _camera_info_to_intrinsics(msg: dict[str, Any]) -> dict[str, object]:
    distortion_model = str(msg.get("distortion_model", "none")).lower().strip()
    if distortion_model in ("plumb_bob", "brown_conrady"):
        model_name = "brown_conrady"
    elif distortion_model in ("inverse_brown_conrady",):
        model_name = "inverse_brown_conrady"
    else:
        model_name = "none"
    k = list(msg.get("K", [0.0] * 9))
    d = list(msg.get("D", []))
    return {
        "width": int(msg["width"]),
        "height": int(msg["height"]),
        "ppx": float(k[2]),
        "ppy": float(k[5]),
        "fx": float(k[0]),
        "fy": float(k[4]),
        "model_name": model_name,
        "coeffs": [float(v) for v in d[:5]] + [0.0] * max(0, 5 - len(d[:5])),
        "depth_scale": 0.001,
    }


def _q_from_rpy(roll: float, pitch: float, yaw: float) -> list[float]:
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


def _rpy_from_q(qx: float, qy: float, qz: float, qw: float) -> list[float]:
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


def _pose_close(current_pose: list[float], target_pose: list[float], pos_tol_m: float = 0.003, ang_tol_rad: float = 0.02) -> bool:
    pos_ok = all(abs(float(current_pose[i]) - float(target_pose[i])) <= float(pos_tol_m) for i in range(3))
    ang_ok = all(abs(float(current_pose[i]) - float(target_pose[i])) <= float(ang_tol_rad) for i in range(3, 6))
    return pos_ok and ang_ok


def _joint_close(current_joint_deg: list[float], target_joint_deg: list[float], tol_deg: float = 0.5) -> bool:
    return all(abs(float(curr) - float(tgt)) <= float(tol_deg) for curr, tgt in zip(current_joint_deg, target_joint_deg))


class RosbridgeFrameCollector:
    def __init__(self, ros: roslibpy.Ros) -> None:
        self.ros = ros
        self.lock = threading.Lock()
        self.latest_color_msg: dict[str, Any] | None = None
        self.latest_depth_msg: dict[str, Any] | None = None
        self.intrinsics: dict[str, object] | None = None
        self.seq = 0

        self.color_topic = roslibpy.Topic(self.ros, "/camera/color/image_raw", "sensor_msgs/Image")
        self.depth_topic = roslibpy.Topic(self.ros, "/camera/aligned_depth_to_color/image_raw", "sensor_msgs/Image")
        self.cam_info_topic = roslibpy.Topic(self.ros, "/camera/color/camera_info", "sensor_msgs/CameraInfo")
        self.color_topic.subscribe(self._on_color)
        self.depth_topic.subscribe(self._on_depth)
        self.cam_info_topic.subscribe(self._on_camera_info)

    def _on_color(self, msg: dict[str, Any]) -> None:
        with self.lock:
            self.latest_color_msg = dict(msg)
            self.seq += 1

    def _on_depth(self, msg: dict[str, Any]) -> None:
        with self.lock:
            self.latest_depth_msg = dict(msg)
            self.seq += 1

    def _on_camera_info(self, msg: dict[str, Any]) -> None:
        intrinsics = _camera_info_to_intrinsics(msg)
        with self.lock:
            self.intrinsics = intrinsics

    def wait_until_ready(self, timeout_s: float = 10.0) -> None:
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            with self.lock:
                ready = self.latest_color_msg is not None and self.latest_depth_msg is not None and self.intrinsics is not None
            if ready:
                return
            time.sleep(0.05)
        raise RuntimeError("rosbridge image streams not ready")

    def get_latest(self) -> tuple[dict[str, Any] | None, dict[str, object] | None]:
        with self.lock:
            if self.latest_color_msg is None or self.latest_depth_msg is None:
                return None, None if self.intrinsics is None else dict(self.intrinsics)
            color_msg = dict(self.latest_color_msg)
            depth_msg = dict(self.latest_depth_msg)
            intrinsics = None if self.intrinsics is None else dict(self.intrinsics)

        color_bgr = _image_msg_to_array(color_msg)
        depth_raw = _image_msg_to_array(depth_msg)
        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) * 0.001
        else:
            depth_m = depth_raw.astype(np.float32)
        return {
            "stamp": time.time(),
            "color_bgr": color_bgr,
            "depth_m": depth_m,
        }, intrinsics

    def close(self) -> None:
        self.color_topic.unsubscribe()
        self.depth_topic.unsubscribe()
        self.cam_info_topic.unsubscribe()


class SshSnapshotGrabber:
    def __init__(self, board_ssh: str, refresh_period_s: float = 0.8) -> None:
        self.board_ssh = board_ssh
        self.refresh_period_s = max(0.2, float(refresh_period_s))
        self._latest_frame: dict[str, Any] | None = None
        self._latest_intrinsics: dict[str, object] | None = None
        self._last_fetch_time = 0.0

    def wait_until_ready(self, timeout_s: float = 12.0) -> None:
        deadline = time.time() + float(timeout_s)
        last_err = None
        while time.time() < deadline:
            try:
                self._fetch(force=True)
                return
            except Exception as exc:
                last_err = exc
                time.sleep(0.2)
        raise RuntimeError(f"ssh snapshot not ready: {last_err}")

    def get_latest(self) -> tuple[dict[str, Any] | None, dict[str, object] | None]:
        try:
            self._fetch(force=False)
        except Exception:
            pass
        return self._latest_frame, self._latest_intrinsics

    def close(self) -> None:
        return

    def _fetch(self, force: bool) -> None:
        now = time.time()
        if not force and self._latest_frame is not None and (now - self._last_fetch_time) < self.refresh_period_s:
            return
        payload = self._capture_once()
        color_bytes = base64.b64decode(payload["color_jpg_b64"])
        color_buf = np.frombuffer(color_bytes, dtype=np.uint8)
        color_bgr = cv2.imdecode(color_buf, cv2.IMREAD_COLOR)
        if color_bgr is None:
            raise RuntimeError("failed to decode color jpg from ssh snapshot")
        depth_bytes = base64.b64decode(payload["depth_npy_b64"])
        depth_m = np.load(io.BytesIO(depth_bytes))
        self._latest_frame = {
            "stamp": float(payload.get("stamp", time.time())),
            "color_bgr": color_bgr,
            "depth_m": depth_m.astype(np.float32),
        }
        self._latest_intrinsics = dict(payload["intrinsics"])
        self._last_fetch_time = now

    def _capture_once(self) -> dict[str, Any]:
        remote_cmd = (
            "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
            "source /home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/setup.bash >/dev/null 2>&1; "
            "python3 -"
        )
        remote_py = textwrap.dedent(
            """
            import base64
            import io
            import json
            import time

            import cv2
            import numpy as np
            import rospy
            from cv_bridge import CvBridge
            from sensor_msgs.msg import CameraInfo, Image

            if not rospy.core.is_initialized():
                rospy.init_node("rm_product_bladder_demo_local_snapshot", anonymous=True, disable_signals=True)

            bridge = CvBridge()
            default_intrinsics = {
                "width": 640,
                "height": 480,
                "ppx": 318.4939270019531,
                "ppy": 245.07725524902344,
                "fx": 609.5076904296875,
                "fy": 609.6961059570312,
                "model_name": "brown_conrady",
                "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
                "depth_scale": 0.001,
            }
            intrinsics = dict(default_intrinsics)
            try:
                cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=2.0)
                distortion_model = str(getattr(cam_info, "distortion_model", "none")).lower().strip()
                if distortion_model in ("plumb_bob", "brown_conrady"):
                    model_name = "brown_conrady"
                elif distortion_model in ("inverse_brown_conrady",):
                    model_name = "inverse_brown_conrady"
                else:
                    model_name = "none"
                intrinsics = {
                    "width": int(cam_info.width),
                    "height": int(cam_info.height),
                    "ppx": float(cam_info.K[2]),
                    "ppy": float(cam_info.K[5]),
                    "fx": float(cam_info.K[0]),
                    "fy": float(cam_info.K[4]),
                    "model_name": model_name,
                    "coeffs": [float(v) for v in cam_info.D[:5]] + [0.0] * max(0, 5 - len(cam_info.D[:5])),
                    "depth_scale": 0.001,
                }
            except Exception:
                pass
            color_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=8.0)
            depth_msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=8.0)

            color_bgr = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_raw = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth_raw.dtype == np.uint16:
                depth_m = depth_raw.astype(np.float32) * 0.001
            else:
                depth_m = depth_raw.astype(np.float32)

            ok, color_buf = cv2.imencode(".jpg", color_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                raise RuntimeError("jpg encode failed")
            depth_io = io.BytesIO()
            np.save(depth_io, depth_m)

            payload = {
                "stamp": time.time(),
                "intrinsics": intrinsics,
                "color_jpg_b64": base64.b64encode(color_buf.tobytes()).decode("ascii"),
                "depth_npy_b64": base64.b64encode(depth_io.getvalue()).decode("ascii"),
            }
            print(json.dumps(payload, ensure_ascii=False))
            """
        )
        proc = subprocess.run(
            ["ssh", self.board_ssh, "bash", "-lc", remote_cmd],
            input=remote_py,
            text=True,
            capture_output=True,
            timeout=20.0,
            check=False,
        )
        if proc.returncode != 0:
            raise RuntimeError(f"ssh snapshot failed: {proc.stderr.strip() or proc.stdout.strip()}")
        text = (proc.stdout or "").strip().splitlines()
        if not text:
            raise RuntimeError("ssh snapshot returned empty stdout")
        return json.loads(text[-1])


class RosbridgeArmBridge:
    def __init__(self, ros: roslibpy.Ros) -> None:
        self.ros = ros
        self.lock = threading.Lock()
        self.last_state: tuple[list[float], list[float], int, int, int] | None = None
        self.last_state_time = 0.0
        self.last_plan_state: bool | None = None
        self.last_plan_state_time = 0.0

        self.topic_arm_state = roslibpy.Topic(self.ros, "/rm_driver/Arm_Current_State", "rm_msgs/Arm_Current_State")
        self.topic_arm_state.subscribe(self._on_arm_state)
        self.topic_plan_state = roslibpy.Topic(self.ros, "/rm_driver/Plan_State", "rm_msgs/Plan_State")
        self.topic_plan_state.subscribe(self._on_plan_state)

        self.pub_get_state = roslibpy.Topic(self.ros, "/rm_driver/GetCurrentArmState", "std_msgs/Empty")
        self.pub_movel = roslibpy.Topic(self.ros, "/rm_driver/MoveL_Cmd", "rm_msgs/MoveL")
        self.pub_movej = roslibpy.Topic(self.ros, "/rm_driver/MoveJ_Cmd", "rm_msgs/MoveJ")
        self.pub_stop = roslibpy.Topic(self.ros, "/rm_driver/Stop_Cmd", "rm_msgs/Stop")
        for pub in (self.pub_get_state, self.pub_movel, self.pub_movej, self.pub_stop):
            pub.advertise()

    def _on_arm_state(self, msg: dict[str, Any]) -> None:
        joints = [float(v) for v in list(msg.get("joint", []))[:6]]
        pose = [float(v) for v in list(msg.get("Pose", []))[:6]]
        arm_err = int(msg.get("arm_err", 0))
        sys_err = int(msg.get("sys_err", 0))
        with self.lock:
            self.last_state = (joints, pose, arm_err, sys_err, -1)
            self.last_state_time = time.time()

    def _on_plan_state(self, msg: dict[str, Any]) -> None:
        with self.lock:
            self.last_plan_state = bool(msg.get("state", False))
            self.last_plan_state_time = time.time()

    def can_connect(self, host: str | None = None, timeout: float = 2.0) -> bool:
        del host
        try:
            self.get_current_arm_state(timeout=timeout)
            return True
        except Exception:
            return False

    def get_current_arm_state(self, host: str | None = None, timeout: float = 2.0) -> tuple[list[float], list[float], int, int, int]:
        del host
        before = time.time()
        self.pub_get_state.publish(roslibpy.Message({}))
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            with self.lock:
                state = self.last_state
                t_state = self.last_state_time
            if state is not None and t_state >= before:
                return state
            time.sleep(0.05)
        raise RuntimeError("rosbridge arm state timed out")

    def recover_if_needed(self, host: str | None = None) -> None:
        _, _, arm_err, sys_err, inverse_km_err = self.get_current_arm_state(host=host, timeout=2.0)
        if arm_err != 0 or sys_err != 0 or inverse_km_err not in (0, -1):
            raise RuntimeError(
                f"arm state not clean: arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
            )

    def movej(self, host: str | None, joint_deg: list[float], speed: float, timeout: float = 30.0) -> dict[str, object]:
        del host
        msg = {
            "joint": [float(math.radians(float(v))) for v in joint_deg[:6]],
            "speed": float(speed),
        }
        self.pub_movej.publish(roslibpy.Message(msg))
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            joints, _, arm_err, sys_err, _ = self.get_current_arm_state(timeout=1.0)
            if arm_err != 0 or sys_err != 0:
                raise RuntimeError(f"movej failed: arm_err={arm_err} sys_err={sys_err}")
            if _joint_close(joints, joint_deg):
                return {"state": "rosbridge_polled_joint_state", "trajectory_state": True}
            time.sleep(0.08)
        raise RuntimeError(f"movej did not reach target: target={joint_deg}")

    def movel(self, host: str | None, pose: list[float], speed: float, blend_radius: float = 0.0, timeout: float = 30.0) -> dict[str, object]:
        del host, blend_radius
        quat = _q_from_rpy(float(pose[3]), float(pose[4]), float(pose[5]))
        msg = {
            "Pose": {
                "position": {"x": float(pose[0]), "y": float(pose[1]), "z": float(pose[2])},
                "orientation": {"x": float(quat[0]), "y": float(quat[1]), "z": float(quat[2]), "w": float(quat[3])},
            },
            "speed": float(speed),
            "trajectory_connect": 0,
        }
        self.pub_movel.publish(roslibpy.Message(msg))
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            _, current_pose, arm_err, sys_err, _ = self.get_current_arm_state(timeout=1.0)
            if arm_err != 0 or sys_err != 0:
                raise RuntimeError(f"movel failed: arm_err={arm_err} sys_err={sys_err}")
            if _pose_close(current_pose, pose):
                return {"state": "rosbridge_polled_pose_state", "trajectory_state": True}
            time.sleep(0.08)
        raise RuntimeError(f"movel did not reach target: target={pose}")

    def stop_motion(self, host: str | None = None) -> dict[str, object]:
        del host
        self.pub_stop.publish(roslibpy.Message({"state": True}))
        return {"state": "rosbridge_stop_published"}

    def close(self) -> None:
        self.topic_arm_state.unsubscribe()
        self.topic_plan_state.unsubscribe()
        for pub in (self.pub_get_state, self.pub_movel, self.pub_movej, self.pub_stop):
            try:
                pub.unadvertise()
            except Exception:
                pass


class RosbridgeProductClient:
    def __init__(self, ros: roslibpy.Ros) -> None:
        self.ros = ros
        self.srv_move_camera = roslibpy.Service(
            self.ros,
            "/ai_service/move_camera_above_person",
            "rm_healthcare_robot_msgs/MoveCameraAbovePerson",
        )
        self.srv_calc_poses = roslibpy.Service(
            self.ros,
            "/calc_poses",
            "rm_healthcare_robot_msgs/WaypointPosesCalc",
        )
        self.action_calc_position = roslibpy.ActionClient(
            self.ros,
            "/ai_service/calc_position_normal",
            "rm_healthcare_robot_msgs/CalcPositionVectorAction",
        )

    def move_camera_above_person(self, tool_name_camera: str, tool_name_mas: str, shifting_number: int) -> dict[str, Any]:
        req = roslibpy.ServiceRequest(
            {
                "tool_name_camera": str(tool_name_camera),
                "tool_name_mas": str(tool_name_mas),
                "shifting_number": int(shifting_number),
            }
        )
        return dict(self.srv_move_camera.call(req))

    def calc_position_normal(
        self,
        color_bgr: np.ndarray,
        depth_m: np.ndarray,
        bbox: list[list[int]],
        sampled_pixels: list[list[int]],
    ) -> list[dict[str, Any]]:
        color_msg = _np_image_to_rosbridge_image(color_bgr, encoding="bgr8")
        depth_u16 = np.clip(np.round(depth_m * 1000.0), 0, 65535).astype(np.uint16)
        depth_msg = _np_image_to_rosbridge_image(depth_u16, encoding="16UC1")
        goal = roslibpy.Goal(
            {
                "color_image": color_msg,
                "depth_image": depth_msg,
                "diagonal_point_coor": [{"x": int(x), "y": int(y)} for x, y in bbox],
                "waypoints_pixel_coor": [{"x": int(x), "y": int(y)} for x, y in sampled_pixels],
            }
        )

        result_box: dict[str, Any] = {"result": None, "error": None}
        done = threading.Event()

        def _resultback(result):
            result_box["result"] = result
            done.set()

        def _feedback(_feedback):
            return

        def _errback(err):
            result_box["error"] = err
            done.set()

        goal_id = self.action_calc_position.send_goal(goal, _resultback, _feedback, _errback)
        if not goal_id:
            raise RuntimeError("failed to send calc_position_normal action goal")
        if not done.wait(20.0):
            raise RuntimeError("calc_position_normal action timed out")
        if result_box["error"] is not None:
            raise RuntimeError(f"calc_position_normal action error: {result_box['error']}")
        result = result_box["result"] or {}
        if isinstance(result, dict) and "result" in result:
            return list(result["result"].get("waypoints_position_vector", []))
        return list(result.get("waypoints_position_vector", []))

    def calc_poses(self, joints_deg: list[float], install_ang: list[float], waypoints: list[dict[str, Any]]) -> list[dict[str, Any]]:
        req = roslibpy.ServiceRequest(
            {
                "arm_curr_joint_ang": [float(v) for v in joints_deg[:6]],
                "install_ang": [float(v) for v in install_ang[:3]],
                "waypoints": waypoints,
            }
        )
        resp = dict(self.srv_calc_poses.call(req))
        return list(resp.get("waypoint_poses", []))


def _np_image_to_rosbridge_image(arr: np.ndarray, encoding: str) -> dict[str, Any]:
    h, w = arr.shape[:2]
    if arr.ndim == 2:
        step = int(arr.strides[0])
    else:
        step = int(arr.strides[0])
    return {
        "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
        "height": int(h),
        "width": int(w),
        "encoding": str(encoding),
        "is_bigendian": 0,
        "step": int(step),
        "data": base64.b64encode(arr.tobytes()).decode("ascii"),
    }


def _build_body_bbox_pixel(image_bgr: np.ndarray, pose_kpts: np.ndarray | None, meridian_lines) -> list[list[int]]:
    h, w = image_bgr.shape[:2]
    pts = []
    if pose_kpts is not None:
        for idx in (5, 6, 11, 12):
            if len(pose_kpts) > idx and float(pose_kpts[idx][2]) > 0.2:
                pts.append((int(round(float(pose_kpts[idx][0]))), int(round(float(pose_kpts[idx][1])))))
    if not pts and meridian_lines is not None:
        for line in meridian_lines:
            pts.extend(
                [
                    (int(round(float(line[0][0]))), int(round(float(line[0][1])))),
                    (int(round(float(line[1][0]))), int(round(float(line[1][1])))),
                ]
            )
    if not pts:
        return [[0, 0], [w - 1, 0], [0, h - 1], [w - 1, h - 1]]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    pad_x = max(20, int((max(xs) - min(xs)) * 0.6))
    pad_y = max(40, int((max(ys) - min(ys)) * 0.8))
    x1 = max(0, min(xs) - pad_x)
    x2 = min(w - 1, max(xs) + pad_x)
    y1 = max(0, min(ys) - pad_y)
    y2 = min(h - 1, max(ys) + pad_y)
    return [[x1, y1], [x2, y1], [x1, y2], [x2, y2]]


def _attach_robot_points_via_product_ros_local(
    product: RosbridgeProductClient,
    arm: RosbridgeArmBridge,
    *,
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    detection_result: dict[str, Any],
    install_ang: list[float],
) -> dict[str, Any]:
    selected_side = str(detection_result.get("selected_side", ""))
    if selected_side not in ("left", "right"):
        raise RuntimeError("selected_side missing before product ROS transform")
    selected_pixels = list(detection_result.get("selected_meridian_pixel", []))
    if len(selected_pixels) < 2:
        raise RuntimeError("selected_meridian_pixel empty before product ROS transform")

    bbox = detection_result.get("body_bbox_pixel")
    if not isinstance(bbox, list) or len(bbox) != 4:
        bbox = _build_body_bbox_pixel(color_bgr, None, None)

    waypoints = product.calc_position_normal(
        color_bgr=color_bgr,
        depth_m=depth_m,
        bbox=bbox,
        sampled_pixels=selected_pixels,
    )
    if not waypoints:
        raise RuntimeError("calc_position_normal returned zero waypoints")

    joints_deg, _, _, _, _ = arm.get_current_arm_state(timeout=2.0)
    world_poses = product.calc_poses(joints_deg, list(install_ang[:3]), waypoints)
    if not world_poses:
        raise RuntimeError("calc_poses returned zero waypoint poses")

    updated = dict(detection_result)
    robot_points = [
        [
            float(p["position"]["x"]),
            float(p["position"]["y"]),
            float(p["position"]["z"]),
        ]
        for p in world_poses
    ]
    robot_pose_quat = [
        [
            float(p["position"]["x"]),
            float(p["position"]["y"]),
            float(p["position"]["z"]),
            float(p["orientation"]["x"]),
            float(p["orientation"]["y"]),
            float(p["orientation"]["z"]),
            float(p["orientation"]["w"]),
        ]
        for p in world_poses
    ]
    updated["body_bbox_pixel"] = bbox
    updated["selected_meridian_robot"] = robot_points
    updated["selected_meridian_robot_pose_quat"] = robot_pose_quat
    updated[f"{selected_side}_meridian_robot"] = robot_points
    updated[f"{selected_side}_meridian_robot_pose_quat"] = robot_pose_quat
    updated["robot_frame_unit"] = "meters"
    updated["transform_backend"] = "product_ros_rosbridge"
    return updated


def _lift_to_safe_z(arm: RosbridgeArmBridge, pose: list[float], safe_z_m: float, speed: float) -> list[float]:
    if float(pose[2]) >= float(safe_z_m):
        return list(pose)
    lift_pose = [float(pose[0]), float(pose[1]), float(safe_z_m), float(pose[3]), float(pose[4]), float(pose[5])]
    arm.movel(None, lift_pose, speed=speed, timeout=25.0)
    return lift_pose


def _goto_hover_via_safe(arm: RosbridgeArmBridge, hover_pose: list[float], safe_z_m: float, speed: float) -> None:
    try:
        _, current_pose, _, _, _ = arm.get_current_arm_state(timeout=2.0)
    except Exception:
        current_pose = list(hover_pose)
    _lift_to_safe_z(arm, current_pose, safe_z_m, speed)
    waypoint = [
        float(hover_pose[0]),
        float(hover_pose[1]),
        float(max(float(safe_z_m), float(hover_pose[2]))),
        float(hover_pose[3]),
        float(hover_pose[4]),
        float(hover_pose[5]),
    ]
    arm.movel(None, waypoint, speed=speed, timeout=25.0)
    arm.movel(None, list(hover_pose), speed=speed, timeout=25.0)


def _emergency_retreat(arm: RosbridgeArmBridge, safe_z_m: float, speed: float) -> None:
    try:
        arm.stop_motion()
    except Exception:
        pass
    try:
        _, pose, _, _, _ = arm.get_current_arm_state(timeout=2.0)
        _lift_to_safe_z(arm, pose, safe_z_m, speed)
    except Exception:
        pass


def execute_bladder_plan_local(
    *,
    arm: RosbridgeArmBridge,
    plan,
    speed: float,
    dian_jin_dwell_s: float,
    fen_jin_dwell_s: float,
    shun_jin_dwell_s: float,
) -> None:
    arm.recover_if_needed()
    motion_speed = normalize_motion_speed("ros", float(speed), ros_default=0.3)
    _, current_pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(timeout=2.0)
    print(
        f"current_pose={[round(v, 6) for v in current_pose]} "
        f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
    )
    try:
        _lift_to_safe_z(arm, current_pose, plan.safe_z_m, motion_speed)
        _goto_hover_via_safe(arm, plan.frames[0].hover_pose_m, plan.safe_z_m, motion_speed)
        for idx, frame in enumerate(plan.frames):
            print(f"point {frame.index}/{plan.point_count} hover={frame.hover_pose_m[:3]}")
            if idx > 0:
                _goto_hover_via_safe(arm, frame.hover_pose_m, plan.safe_z_m, motion_speed)
            arm.movel(None, frame.dian_jin_pose_m, speed=motion_speed, timeout=25.0)
            time.sleep(max(0.0, float(dian_jin_dwell_s)))
            arm.movel(None, frame.hover_pose_m, speed=motion_speed, timeout=25.0)

            arm.movel(None, frame.fen_positive_pose_m, speed=motion_speed, timeout=25.0)
            time.sleep(max(0.0, float(fen_jin_dwell_s)))
            arm.movel(None, frame.hover_pose_m, speed=motion_speed, timeout=25.0)
            arm.movel(None, frame.fen_negative_pose_m, speed=motion_speed, timeout=25.0)
            time.sleep(max(0.0, float(fen_jin_dwell_s)))
            arm.movel(None, frame.hover_pose_m, speed=motion_speed, timeout=25.0)

        print("shun_jin start")
        _goto_hover_via_safe(arm, plan.frames[0].hover_pose_m, plan.safe_z_m, motion_speed)
        for frame in plan.frames[1:]:
            _goto_hover_via_safe(arm, frame.hover_pose_m, plan.safe_z_m, motion_speed)
            time.sleep(max(0.0, float(shun_jin_dwell_s)))

        _, final_pose, _, _, _ = arm.get_current_arm_state(timeout=2.0)
        _lift_to_safe_z(arm, final_pose, plan.safe_z_m, motion_speed)
    except BaseException as exc:
        print(f"execute_bladder_plan_local aborted: {type(exc).__name__}: {exc}")
        _emergency_retreat(arm, plan.safe_z_m, motion_speed)
        raise


def _annotate_preview(image_bgr: np.ndarray, lines: list[str], color: tuple[int, int, int]) -> np.ndarray:
    out = image_bgr.copy()
    y = 28
    for line in lines:
        cv2.putText(out, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        y += 26
    return out


def _run_positioning_once(args: argparse.Namespace, product: RosbridgeProductClient, arm: RosbridgeArmBridge) -> None:
    mode = args.capture_positioning.strip().lower()
    if mode == "none":
        return
    if mode in ("prepare", "prepare_then_service"):
        joints = [float(v) for v in args.capture_joints] if args.capture_joints is not None else load_prepare_joints(
            args.trajectory_config,
            section_name=args.capture_prepare_section,
        )
        if joints is None:
            raise RuntimeError(f"{args.capture_prepare_section} not found in trajectory config: {args.trajectory_config}")
        arm.recover_if_needed()
        arm.movej(None, joints, speed=normalize_motion_speed("ros", float(args.position_speed), ros_default=0.2), timeout=40.0)
        print(
            f"capture_positioning={json.dumps({'step': 'prepare', 'prepare_section': args.capture_prepare_section, 'joint_source': 'cli' if args.capture_joints is not None else 'yaml', 'joints_deg': [round(float(v), 3) for v in joints]}, ensure_ascii=False)}"
        )
    if mode in ("service", "prepare_then_service"):
        resp = product.move_camera_above_person(args.camera_tool_name, args.restore_tool_name, args.shifting_number)
        print(f"capture_positioning={json.dumps({'step': 'service', **resp}, ensure_ascii=False)}")
        if not bool(resp.get("state", False)) and mode == "service":
            raise RuntimeError(f"move_camera_above_person failed: {resp}")
    if args.capture_settle_s > 0:
        time.sleep(args.capture_settle_s)


def _prepare_plan_from_detection(
    args: argparse.Namespace,
    product: RosbridgeProductClient,
    arm: RosbridgeArmBridge,
    frame: dict[str, Any],
    detect_result: dict[str, Any],
    overlay: np.ndarray,
) -> tuple[Any, dict[str, Any]]:
    os.makedirs(args.output_dir, exist_ok=True)
    prefix = f"bladder_product_local_{detect_result['timestamp']}"
    overlay_path, detect_json_path = save_bladder_artifacts(args.output_dir, detect_result, overlay, prefix=prefix)
    transformed = _attach_robot_points_via_product_ros_local(
        product,
        arm,
        color_bgr=frame["color_bgr"],
        depth_m=frame["depth_m"],
        detection_result=detect_result,
        install_ang=list(args.install_ang),
    )
    transform_json_path = os.path.join(args.output_dir, f"{prefix}_transform.json")
    _save_json(transform_json_path, transformed)

    if not arm.can_connect(timeout=2.0):
        raise RuntimeError("rosbridge arm backend is not reachable")
    _, current_pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(timeout=2.0)
    print(
        f"anchor_pose={[round(v, 6) for v in current_pose]} "
        f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
    )

    selected_points = list(transformed.get("selected_meridian_robot", []))
    selected_pixels = list(transformed.get("selected_meridian_pixel", []))
    if len(selected_points) < 2:
        raise RuntimeError("selected_meridian_robot has insufficient valid points")
    plan = build_bladder_massage_plan(
        side=args.side,
        line_type=args.line_type,
        meridian_points_robot_m=selected_points,
        meridian_pixels=selected_pixels,
        anchor_pose_m=current_pose,
        point_count=args.plan_points,
        hover_m=args.hover_mm / 1000.0,
        dian_jin_depth_m=args.dian_jin_depth_mm / 1000.0,
        fen_jin_lateral_m=args.fen_jin_lateral_mm / 1000.0,
        safe_lift_m=args.safe_lift_mm / 1000.0,
        meridian_pose_quat=list(transformed.get("selected_meridian_robot_pose_quat", [])),
    )
    plan_json_path = os.path.join(args.output_dir, f"{prefix}_plan.json")
    _save_json(plan_json_path, bladder_plan_to_dict(plan))
    return plan, {
        "overlay_path": overlay_path,
        "detect_json_path": detect_json_path,
        "transform_json_path": transform_json_path,
        "plan_json_path": plan_json_path,
        "transform_backend": str(transformed.get("transform_backend", "unknown")),
    }


def main() -> None:
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    if not args.board_ssh:
        args.board_ssh = f"rm@{args.board_host}"

    ros = roslibpy.Ros(host=args.board_host, port=int(args.rosbridge_port))
    ros.run()
    if not ros.is_connected:
        raise RuntimeError(f"failed to connect rosbridge ws://{args.board_host}:{args.rosbridge_port}")

    if args.frame_source == "rosbridge_stream":
        collector = RosbridgeFrameCollector(ros)
    else:
        collector = SshSnapshotGrabber(args.board_ssh, refresh_period_s=args.snapshot_period_s)
    arm = RosbridgeArmBridge(ros)
    product = RosbridgeProductClient(ros)
    collector.wait_until_ready(timeout_s=12.0)
    print(f"rosbridge_connected=ws://{args.board_host}:{args.rosbridge_port}")
    print(f"frame_source={args.frame_source}")

    try:
        try:
            _run_positioning_once(args, product, arm)
        except Exception as exc:
            print(f"capture_positioning_failed={exc}")

        last_ok: tuple[dict[str, Any], dict[str, Any], np.ndarray] | None = None
        last_saved_plan_json = ""
        print("keys: c=reposition  s=save trajectory+plan  r=run simulate massage  q=quit")

        while True:
            frame, intrinsics = collector.get_latest()
            if frame is None or intrinsics is None:
                time.sleep(0.03)
                continue

            try:
                detect_result, overlay = detect_bladder_lines(
                    color_bgr=frame["color_bgr"],
                    depth_m=frame["depth_m"],
                    intrinsics_data=intrinsics,
                    finger_width_mm=args.finger_width,
                    sample_points=args.sample_points,
                )
                detect_result["capture"] = {
                    "backend": "rosbridge_live",
                    "stamp": float(frame["stamp"]),
                    "board_host": str(args.board_host),
                }
                detect_result = select_bladder_line(detect_result, args.side, args.line_type)
                overlay = _annotate_preview(
                    overlay,
                    [
                        f"side={args.side} line={args.line_type} sample={args.sample_points} plan={args.plan_points}",
                        f"rosbridge={args.board_host}:{args.rosbridge_port}",
                        f"board_ssh={args.board_ssh}",
                        f"keys: c reposition | s save | r run | q quit",
                        f"saved_plan={os.path.basename(last_saved_plan_json) if last_saved_plan_json else 'none'}",
                    ],
                    (0, 255, 255),
                )
                last_ok = (frame, detect_result, overlay)
            except Exception as exc:
                overlay = _annotate_preview(
                    frame["color_bgr"],
                    [
                        f"detect failed: {exc}",
                        "keep full back / shoulders / hips in frame",
                        "keys: c reposition | q quit",
                    ],
                    (0, 0, 255),
                )

            cv2.imshow("RM Product Bladder Demo Local", overlay)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c"):
                try:
                    _run_positioning_once(args, product, arm)
                    print("capture_positioning=ok")
                except Exception as exc:
                    print(f"capture_positioning_failed={exc}")
                continue
            if key == ord("s"):
                if last_ok is None:
                    print("当前没有可用检测结果，未保存")
                    continue
                try:
                    plan, summary = _prepare_plan_from_detection(args, product, arm, *last_ok)
                    preview_bladder_plan(plan)
                    last_saved_plan_json = str(summary["plan_json_path"])
                    print(f"saved_overlay={summary['overlay_path']}")
                    print(f"saved_detect={summary['detect_json_path']}")
                    print(f"saved_transform={summary['transform_json_path']}")
                    print(f"saved_plan={summary['plan_json_path']}")
                    print(f"transform_backend={summary['transform_backend']}")
                except Exception as exc:
                    print(f"save_failed={exc}")
                continue
            if key == ord("r"):
                if last_ok is None:
                    print("当前没有可用检测结果，无法执行")
                    continue
                try:
                    plan, summary = _prepare_plan_from_detection(args, product, arm, *last_ok)
                    last_saved_plan_json = str(summary["plan_json_path"])
                    preview_bladder_plan(plan)
                    print(
                        f"execute_plan side={args.side} line={args.line_type} "
                        f"points={plan.point_count} backend={summary['transform_backend']}"
                    )
                    execute_bladder_plan_local(
                        arm=arm,
                        plan=plan,
                        speed=args.speed,
                        dian_jin_dwell_s=args.dian_jin_dwell_s,
                        fen_jin_dwell_s=args.fen_jin_dwell_s,
                        shun_jin_dwell_s=args.shun_jin_dwell_s,
                    )
                except Exception as exc:
                    print(f"execute_failed={exc}")
                continue
    finally:
        try:
            collector.close()
        except Exception:
            pass
        try:
            arm.close()
        except Exception:
            pass
        try:
            ros.terminate()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
