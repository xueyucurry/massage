#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path
import struct
import time

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rm_msgs.msg import ArmState
from rm_msgs.srv import Change_Tool_Frame_Srv
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_CONFIG = PROJECT_DIR / "ros_vendor" / "trajectory_generate.yaml"

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


def hmat(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rotation
    out[:3, 3] = position
    return out


def depth_to_m(depth_img: np.ndarray) -> np.ndarray:
    if depth_img.dtype == np.uint16:
        return depth_img.astype(np.float32) * 0.001
    return depth_img.astype(np.float32)


def pack_rgb_float(r: int, g: int, b: int) -> float:
    rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack("f", struct.pack("I", rgb_uint32))[0]


def transform_to_hmat(msg: TransformStamped) -> np.ndarray:
    t = msg.transform.translation
    q = msg.transform.rotation
    return hmat(
        np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64),
        quat_to_matrix(float(q.x), float(q.y), float(q.z), float(q.w)),
    )


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


class RgbdPointCloudWorld:
    def __init__(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_br = tf2_ros.TransformBroadcaster()
        self.flange_from_camera = load_eye_on_hand(Path(rospy.get_param("~trajectory_config", str(DEFAULT_TRAJECTORY_CONFIG))))
        self.stride = max(1, int(rospy.get_param("~stride", 3)))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.15))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 1.8))
        self.max_rate_hz = float(rospy.get_param("~max_rate_hz", 2.0))
        self.camera_k: np.ndarray | None = None
        self.depth_m: np.ndarray | None = None
        self.world_from_camera_color_optical: np.ndarray | None = None
        self.last_publish = 0.0
        self.last_camera_tf_warn = 0.0

        self.pub = rospy.Publisher("/rm_demo/back_rgbd_points", PointCloud2, queue_size=1)
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
            rospy.logwarn("could not switch to mas_rub before RGBD point cloud: %s", exc)

    def on_info(self, msg: CameraInfo):
        self.camera_k = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)

    def on_depth(self, msg: Image):
        self.depth_m = depth_to_m(self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))

    def on_arm(self, msg: ArmState):
        p = msg.Pose.position
        q = msg.Pose.orientation
        base_from_mas = hmat(
            np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64),
            quat_to_matrix(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        base_from_flange = base_from_mas @ np.linalg.inv(FLANGE_T_MAS_RUB)
        self.world_from_camera_color_optical = base_from_flange @ self.flange_from_camera
        self.publish_camera_link_tf(rospy.Time.now())

    def publish_camera_link_tf(self, stamp: rospy.Time):
        if self.world_from_camera_color_optical is None:
            return
        try:
            camera_link_from_color = transform_to_hmat(
                self.tf_buffer.lookup_transform(
                    "camera_link",
                    "camera_color_optical_frame",
                    rospy.Time(0),
                    rospy.Duration(0.05),
                )
            )
            world_from_camera_link = self.world_from_camera_color_optical @ np.linalg.inv(camera_link_from_color)
            self.tf_br.sendTransform(make_transform("world", "camera_link", world_from_camera_link, stamp))
        except Exception as exc:
            now = time.time()
            if now - self.last_camera_tf_warn > 3.0:
                rospy.logwarn("could not connect camera_link to world yet: %s", exc)
                self.last_camera_tf_warn = now

    def on_color(self, msg: Image):
        now = time.time()
        if now - self.last_publish < 1.0 / max(self.max_rate_hz, 0.1):
            return
        if self.camera_k is None or self.depth_m is None or self.world_from_camera_color_optical is None:
            return
        self.publish_camera_link_tf(rospy.Time.now())

        color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        depth = self.depth_m
        if depth.shape[:2] != color_bgr.shape[:2]:
            depth = cv2.resize(depth, (color_bgr.shape[1], color_bgr.shape[0]), interpolation=cv2.INTER_NEAREST)

        fx, fy = float(self.camera_k[0, 0]), float(self.camera_k[1, 1])
        cx, cy = float(self.camera_k[0, 2]), float(self.camera_k[1, 2])
        rot = self.world_from_camera_color_optical[:3, :3]
        trans = self.world_from_camera_color_optical[:3, 3]

        points = []
        height, width = depth.shape[:2]
        for v in range(0, height, self.stride):
            for u in range(0, width, self.stride):
                z = float(depth[v, u])
                if not np.isfinite(z) or z < self.min_depth_m or z > self.max_depth_m:
                    continue
                camera_xyz = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float64)
                world_xyz = rot @ camera_xyz + trans
                b, g, r = color_bgr[v, u]
                points.append((float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2]), pack_rgb_float(int(r), int(g), int(b))))

        if not points:
            return

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]
        header = msg.header
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        self.pub.publish(pc2.create_cloud(header, fields, points))
        self.last_publish = now


def main():
    rospy.init_node("rm_rgbd_pointcloud_world", anonymous=False)
    RgbdPointCloudWorld()
    rospy.loginfo("Publishing RGBD point cloud in world frame on /rm_demo/back_rgbd_points")
    rospy.spin()


if __name__ == "__main__":
    main()
