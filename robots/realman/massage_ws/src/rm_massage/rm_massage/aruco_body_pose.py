from __future__ import annotations

import argparse
from typing import Any

from .config_io import deep_get, load_yaml
from .transforms import (
    Transform,
    compose,
    from_translation_quat,
    inverse,
    matrix_to_quat,
    to_translation_quat,
)


def _cv_matrix_tuple(R) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(tuple(float(R[i, j]) for j in range(3)) for i in range(3))  # type: ignore[return-value]


class ArucoBodyPoseNode:
    def __init__(self, node: Any, args: argparse.Namespace) -> None:
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import TransformBroadcaster

        self.cv2 = cv2
        self.np = np
        self.node = node
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(node)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_T: Transform | None = None

        frames = load_yaml(args.frames)
        markers = load_yaml(args.markers)
        self.camera_frame = deep_get(frames, "frames.camera", "camera_color_optical_frame")
        self.body_frame = deep_get(frames, "frames.body", "body_frame")
        self.marker_size_m = float(deep_get(markers, "aruco.marker_size_m", 0.05))
        self.marker_id = deep_get(markers, "aruco.marker_id", None)
        self.alpha = float(deep_get(markers, "filter.alpha", 1.0))
        self.T_body_marker = from_translation_quat(
            deep_get(markers, "aruco.T_body_marker.translation", [0.0, 0.0, 0.0]),
            deep_get(markers, "aruco.T_body_marker.quaternion", [0.0, 0.0, 0.0, 1.0]),
        )

        dict_name = deep_get(markers, "aruco.dictionary", "DICT_4X4_50")
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        else:
            self.detector = None
            self.dictionary = dictionary
            self.params = cv2.aruco.DetectorParameters_create()

        self.sub_info = node.create_subscription(CameraInfo, args.camera_info_topic, self._on_info, 10)
        self.sub_image = node.create_subscription(Image, args.image_topic, self._on_image, 10)
        node.get_logger().info(f"subscribed {args.image_topic}, broadcasting {self.camera_frame} -> {self.body_frame}")

    def _on_info(self, msg: Any) -> None:
        self.camera_matrix = self.np.array(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = self.np.array(msg.d, dtype=float)

    def _detect(self, gray: Any):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = self.cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        return corners, ids

    def _on_image(self, msg: Any) -> None:
        if self.camera_matrix is None or self.dist_coeffs is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = self.cv2.cvtColor(image, self.cv2.COLOR_BGR2GRAY)
            corners, ids = self._detect(gray)
            if ids is None or len(ids) == 0:
                return
            selected_index = 0
            if self.marker_id is not None:
                matches = [i for i, mid in enumerate(ids.flatten().tolist()) if int(mid) == int(self.marker_id)]
                if not matches:
                    return
                selected_index = matches[0]

            rvecs, tvecs, _ = self.cv2.aruco.estimatePoseSingleMarkers(
                [corners[selected_index]],
                self.marker_size_m,
                self.camera_matrix,
                self.dist_coeffs,
            )
            R, _ = self.cv2.Rodrigues(rvecs[0][0])
            t = tvecs[0][0]
            T_camera_marker = (_cv_matrix_tuple(R), (float(t[0]), float(t[1]), float(t[2])))
            T_camera_body = compose(T_camera_marker, inverse(self.T_body_marker))
            T_camera_body = self._filtered(T_camera_body)
            self._broadcast(T_camera_body, msg.header.stamp)
        except Exception as exc:
            self.node.get_logger().warn(f"aruco update failed: {exc}")

    def _filtered(self, current: Transform) -> Transform:
        if self.last_T is None or self.alpha >= 1.0:
            self.last_T = current
            return current
        R, t = current
        _, last_t = self.last_T
        a = max(0.0, min(1.0, self.alpha))
        smooth_t = tuple(last_t[i] * (1.0 - a) + t[i] * a for i in range(3))
        self.last_T = (R, smooth_t)  # type: ignore[assignment]
        return self.last_T

    def _broadcast(self, T_camera_body: Transform, stamp: Any) -> None:
        from geometry_msgs.msg import TransformStamped

        t, q = to_translation_quat(T_camera_body)
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        msg.child_frame_id = self.body_frame
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect ArUco and broadcast body_frame.")
    parser.add_argument("--frames", default="config/frames.yaml")
    parser.add_argument("--markers", default="config/body_markers.yaml")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("rm_aruco_body_pose")
    aruco_node = ArucoBodyPoseNode(node, args)
    del aruco_node
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
