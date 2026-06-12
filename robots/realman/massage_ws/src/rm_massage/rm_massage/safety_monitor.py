from __future__ import annotations

import argparse
import math
import time
from typing import Any

from .config_io import deep_get, load_yaml
from .ros_helpers import extract_force, get_message_class


class SafetyMonitor:
    def __init__(self, node: Any, frames: dict[str, Any], safety: dict[str, Any]) -> None:
        self.node = node
        self.force_axis = deep_get(safety, "force.normal_axis", "fz")
        self.force_sign = float(deep_get(safety, "force.normal_sign", 1.0))
        self.force_max_n = float(deep_get(safety, "safety.force_max_n", 10.0))
        self.torque_max_nm = float(deep_get(safety, "safety.torque_max_nm", 1.0))
        self.triggered = False

        topics = frames.get("topics", {})
        types = frames.get("message_types", {})
        self.empty_type = get_message_class(types.get("empty", "std_msgs/msg/Empty"))
        force_type = get_message_class(types.get("six_force", "rm_ros_interfaces/msg/Sixforce"))
        self.sub_force = node.create_subscription(
            force_type,
            topics.get("six_force", "/rm_driver/udp_six_force"),
            self._on_force,
            10,
        )
        self.pub_stop_force = node.create_publisher(
            self.empty_type,
            topics.get("stop_force_position_cmd", "/rm_driver/stop_force_postion_cmd"),
            10,
        )
        self.pub_motion_stop = node.create_publisher(
            self.empty_type,
            topics.get("motion_stop_cmd", "/rm_driver/stop_cmd"),
            10,
        )

    def _on_force(self, msg: Any) -> None:
        if self.triggered:
            return
        try:
            force = extract_force(msg)
        except Exception as exc:
            self.node.get_logger().warn(f"force parse failed: {exc}")
            return
        f_normal = self.force_sign * float(force.get(self.force_axis, 0.0))
        torque = math.sqrt(
            float(force.get("mx", 0.0)) ** 2
            + float(force.get("my", 0.0)) ** 2
            + float(force.get("mz", 0.0)) ** 2
        )
        if f_normal > self.force_max_n or torque > self.torque_max_nm:
            self.triggered = True
            self.node.get_logger().error(
                f"safety stop: F_normal={f_normal:.2f} N torque_norm={torque:.2f} Nm"
            )
            for _ in range(3):
                self.pub_stop_force.publish(self.empty_type())
                self.pub_motion_stop.publish(self.empty_type())
                time.sleep(0.05)


def main() -> None:
    parser = argparse.ArgumentParser(description="Independent force safety monitor.")
    parser.add_argument("--frames", default="config/frames.yaml")
    parser.add_argument("--safety", default="config/safety.yaml")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node

    frames = load_yaml(args.frames)
    safety = load_yaml(args.safety)

    rclpy.init()
    node = Node("rm_safety_monitor")
    monitor = SafetyMonitor(node, frames, safety)
    del monitor
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
