#!/usr/bin/env python3
import argparse
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_msg_to_array(msg: Image) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if msg.encoding in ("rgb8", "bgr8"):
        img = data.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    if msg.encoding in ("rgba8", "bgra8"):
        img = data.reshape(msg.height, msg.width, 4)
        if msg.encoding == "rgba8":
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img

    if msg.encoding == "mono8":
        return data.reshape(msg.height, msg.width)

    if msg.encoding in ("16UC1", "mono16"):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def depth_to_bgr(depth: np.ndarray, max_depth_mm: float) -> np.ndarray:
    if depth.dtype == np.uint16:
        depth_mm = depth.astype(np.float32)
    elif depth.dtype == np.float32:
        depth_mm = depth * 1000.0
    else:
        depth_mm = depth.astype(np.float32)

    valid = np.isfinite(depth_mm) & (depth_mm > 0.0)
    vis = np.zeros(depth_mm.shape, dtype=np.uint8)
    if np.any(valid):
        clipped = np.clip(depth_mm, 0.0, max_depth_mm)
        vis[valid] = np.round(clipped[valid] / max_depth_mm * 255.0).astype(np.uint8)
    colored = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    colored[~valid] = 0
    return colored


class RGBDViewer(Node):
    def __init__(self, color_topic: str, depth_topic: str, max_depth_mm: float):
        super().__init__("rosbag_rgbd_viewer")
        self.max_depth_mm = float(max_depth_mm)
        self._lock = threading.Lock()
        self._latest_color = None
        self._latest_depth = None
        self._latest_color_stamp = None
        self._latest_depth_stamp = None

        # 与 RealSense / rosbag2 回放常用 QoS 一致，避免默认订阅 QoS 不匹配导致收不到图
        qos = qos_profile_sensor_data

        self.create_subscription(Image, color_topic, self._on_color, qos)
        self.create_subscription(Image, depth_topic, self._on_depth, qos)
        self.create_timer(1.0 / 60.0, self._display)

        self.get_logger().info(f"Subscribed color: {color_topic}")
        self.get_logger().info(f"Subscribed depth: {depth_topic}")
        self.get_logger().info("Press q or ESC in OpenCV window to quit.")

    def _on_color(self, msg: Image):
        try:
            img = image_msg_to_array(msg)
        except Exception as e:
            self.get_logger().warning(f"Color decode failed: {e}")
            return
        with self._lock:
            self._latest_color = img
            self._latest_color_stamp = msg.header.stamp

    def _on_depth(self, msg: Image):
        try:
            img = image_msg_to_array(msg)
        except Exception as e:
            self.get_logger().warning(f"Depth decode failed: {e}")
            return
        with self._lock:
            self._latest_depth = img
            self._latest_depth_stamp = msg.header.stamp

    def _display(self):
        # 每帧显示「当前最新」图像，避免仅用 dirty 位时低帧率 bag 在窗口上像「卡住」
        with self._lock:
            color = self._latest_color.copy() if self._latest_color is not None else None
            depth = self._latest_depth.copy() if self._latest_depth is not None else None

        if color is not None:
            cv2.imshow("RGB", color)
        if depth is not None:
            cv2.imshow("Depth", depth_to_bgr(depth, self.max_depth_mm))

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self.get_logger().info("Exit requested by user.")
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(description="Subscribe ROS bag RGB/depth images and visualize them with OpenCV.")
    parser.add_argument(
        "--color-topic",
        default="/camera/camera/color/image_raw",
        help="RGB image topic",
    )
    parser.add_argument(
        "--depth-topic",
        default="/camera/camera/depth/image_rect_raw",
        help="Depth image topic",
    )
    parser.add_argument(
        "--max-depth-mm",
        type=float,
        default=3000.0,
        help="Depth visualization clip range in millimeters",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RGBDViewer(args.color_topic, args.depth_topic, args.max_depth_mm)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
