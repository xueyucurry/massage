from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

from .transforms import rpy_to_quat


def _append_realman_root() -> None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "rm_demo").is_dir():
            if str(parent) not in sys.path:
                sys.path.append(str(parent))
            return


_append_realman_root()


class RmJsonBridge:
    def __init__(self, node: Any, host: str, rate_hz: float, base_frame: str, tcp_frame: str) -> None:
        from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped
        from sensor_msgs.msg import JointState
        from tf2_ros import TransformBroadcaster

        from rm_demo import rm_json

        self.node = node
        self.host = host
        self.rm_json = rm_json
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame
        self.pose_pub = node.create_publisher(PoseStamped, "/rm_json/arm_pose", 10)
        self.force_pub = node.create_publisher(WrenchStamped, "/rm_json/six_force", 10)
        self.joint_pub = node.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(node)
        self.timer = node.create_timer(1.0 / max(1e-6, float(rate_hz)), self._tick)
        node.get_logger().info(f"RealMan JSON bridge polling {host}:8080 at {rate_hz:.1f} Hz")

    def _tick(self) -> None:
        try:
            self._publish_arm_state()
            self._publish_force()
        except Exception as exc:
            self.node.get_logger().warn(f"RealMan JSON bridge poll failed: {type(exc).__name__}: {exc}")

    def _publish_arm_state(self) -> None:
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from sensor_msgs.msg import JointState

        joints, pose, arm_err, sys_err, ik_err = self.rm_json.get_current_arm_state(self.host)
        now = self.node.get_clock().now().to_msg()
        quat = rpy_to_quat(pose[3], pose[4], pose[5])

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.base_frame
        pose_msg.pose.position.x = float(pose[0])
        pose_msg.pose.position.y = float(pose[1])
        pose_msg.pose.position.z = float(pose[2])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.pose_pub.publish(pose_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = [f"joint_{idx}" for idx in range(1, len(joints) + 1)]
        joint_msg.position = [float(v) * 3.141592653589793 / 180.0 for v in joints]
        self.joint_pub.publish(joint_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = self.tcp_frame
        tf_msg.transform.translation.x = float(pose[0])
        tf_msg.transform.translation.y = float(pose[1])
        tf_msg.transform.translation.z = float(pose[2])
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        if arm_err or sys_err or ik_err not in (0, -1):
            self.node.get_logger().warn(f"arm state error arm_err={arm_err} sys_err={sys_err} ik_err={ik_err}")

    def _publish_force(self) -> None:
        from geometry_msgs.msg import WrenchStamped

        data = self.rm_json.query_json(self.host, {"command": "get_force_data"}, timeout=1.0)
        raw = data.get("tool_zero_force_data") or data.get("zero_force_data") or data.get("force_data")
        if not isinstance(raw, list) or len(raw) < 6:
            raise RuntimeError(f"unexpected force reply: {data}")
        values = [float(v) / 1000.0 for v in raw[:6]]

        msg = WrenchStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.tcp_frame
        msg.wrench.force.x = values[0]
        msg.wrench.force.y = values[1]
        msg.wrench.force.z = values[2]
        msg.wrench.torque.x = values[3]
        msg.wrench.torque.y = values[4]
        msg.wrench.torque.z = values[5]
        self.force_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Bridge RealMan JSON state into ROS2 standard topics.")
    parser.add_argument("--host", default="192.168.1.18")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--base-frame", default="robot_base")
    parser.add_argument("--tcp-frame", default="massage_tcp")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("rm_json_bridge")
    bridge = RmJsonBridge(node, args.host, args.rate_hz, args.base_frame, args.tcp_frame)
    del bridge
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
