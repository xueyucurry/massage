#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rm_msgs.msg import ArmState
from rm_msgs.srv import Change_Tool_Frame_Srv
from sensor_msgs.msg import CameraInfo, Image


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_CONFIG = PROJECT_DIR / "ros_vendor" / "trajectory_generate.yaml"
DEFAULT_OUTPUT_JSON = PROJECT_DIR / "rm_demo_output" / "aruco_latest.json"

FLANGE_T_MAS_RUB = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, math.cos(0.785), -math.sin(0.785), -0.073916],
        [0.0, math.sin(0.785), math.cos(0.785), 0.110916],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def load_eye_on_hand(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    calib = data.get("eye_on_hand_calibrate", {})
    mat = np.array(
        [[float(calib.get(f"tf{r}_{c}", 0.0)) for c in range(4)] for r in range(4)],
        dtype=np.float64,
    )
    if mat.shape != (4, 4) or not np.isfinite(mat).all():
        raise RuntimeError(f"invalid eye_on_hand_calibrate matrix: {path}")
    return mat


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat(rot: np.ndarray) -> tuple[float, float, float, float]:
    m = rot
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)


def matrix_to_rpy(rot: np.ndarray) -> list[float]:
    sy = max(-1.0, min(1.0, -float(rot[2, 0])))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) > 1e-6:
        roll = math.atan2(float(rot[2, 1]), float(rot[2, 2]))
        yaw = math.atan2(float(rot[1, 0]), float(rot[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(rot[0, 1]), float(rot[1, 1]))
    return [roll, pitch, yaw]


def make_transform(parent: str, child: str, mat: np.ndarray, stamp: rospy.Time) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(mat[0, 3])
    msg.transform.translation.y = float(mat[1, 3])
    msg.transform.translation.z = float(mat[2, 3])
    qx, qy, qz, qw = matrix_to_quat(mat[:3, :3])
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw
    return msg


def hmat(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rotation
    out[:3, 3] = position
    return out


def depth_to_m(depth_img: np.ndarray) -> np.ndarray:
    if depth_img.dtype == np.uint16:
        return depth_img.astype(np.float32) * 0.001
    return depth_img.astype(np.float32)


def sample_depth(depth_m: np.ndarray, u: float, v: float, radius: int = 3) -> float | None:
    h, w = depth_m.shape[:2]
    cu, cv = int(round(u)), int(round(v))
    x0, x1 = max(0, cu - radius), min(w, cu + radius + 1)
    y0, y1 = max(0, cv - radius), min(h, cv + radius + 1)
    vals = depth_m[y0:y1, x0:x1]
    vals = vals[np.isfinite(vals) & (vals > 0.05) & (vals < 2.5)]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def pixel_to_camera(u: float, v: float, z: float, k: np.ndarray) -> np.ndarray:
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float64)


def normalize(vec: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    if n < 1e-8:
        raise RuntimeError("zero-length vector")
    return vec / n


def aruco_pose_from_depth(corners: np.ndarray, depth_m: np.ndarray, k: np.ndarray) -> np.ndarray | None:
    pts = []
    for u, v in corners:
        z = sample_depth(depth_m, float(u), float(v), radius=4)
        if z is None:
            return None
        pts.append(pixel_to_camera(float(u), float(v), z, k))
    p0, p1, _p2, p3 = pts
    center = np.mean(np.asarray(pts), axis=0)
    x_axis = normalize(p1 - p0)
    y_axis = normalize(p3 - p0)
    z_axis = normalize(np.cross(x_axis, y_axis))
    if float(np.dot(z_axis, center)) < 0.0:
        z_axis = -z_axis
        y_axis = -y_axis
    y_axis = normalize(np.cross(z_axis, x_axis))
    rot = np.column_stack([x_axis, y_axis, z_axis])
    return hmat(center, rot)


def aruco_dictionaries():
    names = [
        "DICT_4X4_50",
        "DICT_4X4_100",
        "DICT_5X5_50",
        "DICT_5X5_100",
        "DICT_6X6_50",
        "DICT_6X6_100",
        "DICT_ARUCO_ORIGINAL",
    ]
    out = []
    for name in names:
        if hasattr(cv2.aruco, name):
            out.append((name, cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))))
    return out


class ArucoTfBroadcaster:
    def __init__(self):
        self.bridge = CvBridge()
        self.br = tf2_ros.TransformBroadcaster()
        self.flange_from_camera = load_eye_on_hand(Path(rospy.get_param("~trajectory_config", str(DEFAULT_TRAJECTORY_CONFIG))))
        self.hover_m = float(rospy.get_param("~hover_m", 0.08))
        self.marker_id = int(rospy.get_param("~marker_id", -1))
        self.output_json = Path(rospy.get_param("~output_json", str(DEFAULT_OUTPUT_JSON)))
        self.camera_k: np.ndarray | None = None
        self.depth_m: np.ndarray | None = None
        self.base_from_mas: np.ndarray | None = None
        self.last_write = 0.0
        self.dicts = aruco_dictionaries()
        self.params = cv2.aruco.DetectorParameters_create()

        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.on_depth, queue_size=1)
        rospy.Subscriber("/rm_driver/ArmCurrentState", ArmState, self.on_arm, queue_size=5)
        rospy.Subscriber("/camera/color/image_raw", Image, self.on_color, queue_size=1)

        try:
            rospy.wait_for_service("/change_arm_tool_frame", timeout=3.0)
            change_tool = rospy.ServiceProxy("/change_arm_tool_frame", Change_Tool_Frame_Srv)
            resp = change_tool("mas_rub")
            rospy.loginfo("change_arm_tool_frame mas_rub state=%s", bool(resp.state))
        except Exception as exc:
            rospy.logwarn("could not switch to mas_rub before aruco TF: %s", exc)

    def on_info(self, msg: CameraInfo):
        self.camera_k = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)

    def on_depth(self, msg: Image):
        self.depth_m = depth_to_m(self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))

    def on_arm(self, msg: ArmState):
        p = msg.Pose.position
        q = msg.Pose.orientation
        self.base_from_mas = hmat(
            np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64),
            quat_to_matrix(float(q.x), float(q.y), float(q.z), float(q.w)),
        )

    def select_marker(self, color_bgr: np.ndarray):
        gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
        for dict_name, dictionary in self.dicts:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=self.params)
            if ids is None or len(ids) == 0:
                continue
            ids_flat = [int(v) for v in ids.flatten()]
            chosen = 0
            if self.marker_id >= 0:
                if self.marker_id not in ids_flat:
                    continue
                chosen = ids_flat.index(self.marker_id)
            return dict_name, ids_flat[chosen], np.asarray(corners[chosen][0], dtype=np.float64)
        return None

    def on_color(self, msg: Image):
        if self.camera_k is None or self.depth_m is None or self.base_from_mas is None:
            return
        color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        selected = self.select_marker(color)
        if selected is None:
            return
        dict_name, marker_id, corners = selected
        camera_from_aruco = aruco_pose_from_depth(corners, self.depth_m, self.camera_k)
        if camera_from_aruco is None:
            return
        base_from_flange = self.base_from_mas @ np.linalg.inv(FLANGE_T_MAS_RUB)
        base_from_camera = base_from_flange @ self.flange_from_camera
        base_from_aruco = base_from_camera @ camera_from_aruco

        aruco_from_target = np.eye(4, dtype=np.float64)
        aruco_from_target[:3, 3] = [0.0, 0.0, -self.hover_m]
        base_from_target = base_from_aruco @ aruco_from_target

        stamp = rospy.Time.now()
        self.br.sendTransform(
            [
                make_transform("world", "aruco_back", base_from_aruco, stamp),
                make_transform("aruco_back", "aruco_tool_target", aruco_from_target, stamp),
            ]
        )

        now = time.time()
        if now - self.last_write > 0.2:
            self.output_json.parent.mkdir(parents=True, exist_ok=True)
            target_rpy = matrix_to_rpy(base_from_target[:3, :3])
            data = {
                "stamp": now,
                "marker_id": marker_id,
                "dictionary": dict_name,
                "hover_m": self.hover_m,
                "aruco_pose_m_rpy": [float(v) for v in list(base_from_aruco[:3, 3]) + matrix_to_rpy(base_from_aruco[:3, :3])],
                "target_pose_m_rpy": [float(v) for v in list(base_from_target[:3, 3]) + target_rpy],
                "corners_px": [[float(x), float(y)] for x, y in corners.tolist()],
            }
            self.output_json.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
            self.last_write = now


def main():
    rospy.init_node("rm_aruco_tf_broadcaster", anonymous=False)
    ArucoTfBroadcaster()
    rospy.loginfo("Publishing ArUco TF frames: world -> aruco_back -> aruco_tool_target")
    rospy.spin()


if __name__ == "__main__":
    main()
