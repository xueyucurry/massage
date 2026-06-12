from __future__ import annotations

import argparse
import time
from typing import Any

from .config_io import deep_get, load_yaml
from .trajectory import build_tcp_trajectory, check_workspace
from .transforms import IDENTITY_T, add, scale, transform_from_ros


def _point_msg(xyz):
    from geometry_msgs.msg import Point

    p = Point()
    p.x = float(xyz[0])
    p.y = float(xyz[1])
    p.z = float(xyz[2])
    return p


def _color(r: float, g: float, b: float, a: float = 1.0):
    from std_msgs.msg import ColorRGBA

    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def _new_marker(node, frame: str, ns: str, marker_id: int, marker_type: int):
    from visualization_msgs.msg import Marker

    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = node.get_clock().now().to_msg()
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def _line_marker(node, frame: str, ns: str, marker_id: int, points, color):
    from visualization_msgs.msg import Marker

    m = _new_marker(node, frame, ns, marker_id, Marker.LINE_STRIP)
    m.scale.x = 0.004
    m.color = color
    m.points = [_point_msg(p) for p in points]
    return m


def _sphere_list_marker(node, frame: str, ns: str, marker_id: int, points, color, diameter: float):
    from visualization_msgs.msg import Marker

    m = _new_marker(node, frame, ns, marker_id, Marker.SPHERE_LIST)
    m.scale.x = diameter
    m.scale.y = diameter
    m.scale.z = diameter
    m.color = color
    m.points = [_point_msg(p) for p in points]
    return m


def _arrow_marker(node, frame: str, ns: str, marker_id: int, start, vector, color):
    from visualization_msgs.msg import Marker

    m = _new_marker(node, frame, ns, marker_id, Marker.ARROW)
    m.scale.x = 0.006
    m.scale.y = 0.012
    m.scale.z = 0.018
    m.color = color
    m.points = [_point_msg(start), _point_msg(add(start, vector))]
    return m


def _lookup_line_transform(tf_buffer, node, base_frame: str, line_frame: str, timeout_s: float):
    import rclpy

    if line_frame == base_frame:
        return IDENTITY_T
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            msg = tf_buffer.lookup_transform(base_frame, line_frame, rclpy.time.Time())
            return transform_from_ros(msg)
        except Exception as exc:
            last_error = exc
            rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError(f"TF lookup failed {base_frame} <- {line_frame}: {last_error}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish RViz markers for massage lines and TCP trajectory.")
    parser.add_argument("--frames", default="config/frames.yaml")
    parser.add_argument("--safety", default="config/safety.yaml")
    parser.add_argument("--lines", default="config/massage_lines.yaml")
    parser.add_argument("--topic", default="/visualization_marker_array")
    parser.add_argument("--include-disabled", action="store_true")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import MarkerArray

    frames = load_yaml(args.frames)
    safety = load_yaml(args.safety)
    lines_cfg = load_yaml(args.lines)
    base_frame = deep_get(frames, "frames.robot_base", "robot_base")
    tcp_radius = float(deep_get(frames, "tcp.radius_m", 0.0))
    tcp_type = deep_get(frames, "tcp.type", "sphere")
    limits = deep_get(safety, "safety.workspace_limits_base", {})

    rclpy.init()
    node = Node("rm_preview_line_rviz")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = node.create_publisher(MarkerArray, args.topic, qos)
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)
    del listener

    def publish_once() -> None:
        markers = MarkerArray()
        marker_id = 0
        for line in lines_cfg.get("lines", []):
            if not args.include_disabled and line.get("enabled") is False:
                continue
            line_frame = line.get("frame") or deep_get(frames, "frames.body", "body_frame")
            T_base_line = _lookup_line_transform(tf_buffer, node, base_frame, line_frame, 1.0)
            trajectory = build_tcp_trajectory(
                line,
                T_base_line=T_base_line,
                tcp_radius_m=tcp_radius,
                approach_distance_m=float(deep_get(safety, "safety.approach_distance_m", 0.03)),
                retreat_distance_m=float(deep_get(safety, "safety.retreat_distance_m", 0.03)),
                tcp_type=tcp_type,
            )
            if limits:
                check_workspace(trajectory, limits)
            surface = [p.surface_base for p in trajectory]
            contact = [p.contact_base for p in trajectory]
            pre = [p.pre_base for p in trajectory]
            markers.markers.append(_line_marker(node, base_frame, "surface_line", marker_id, surface, _color(0.1, 0.3, 1.0)))
            marker_id += 1
            markers.markers.append(_sphere_list_marker(node, base_frame, "contact_points", marker_id, contact, _color(0.0, 0.8, 0.2), 0.012))
            marker_id += 1
            markers.markers.append(_sphere_list_marker(node, base_frame, "pre_points", marker_id, pre, _color(1.0, 0.8, 0.0), 0.010))
            marker_id += 1
            for point in trajectory:
                markers.markers.append(
                    _arrow_marker(
                        node,
                        base_frame,
                        "normal",
                        marker_id,
                        point.surface_base,
                        scale(point.normal_base, 0.04),
                        _color(1.0, 0.0, 0.0),
                    )
                )
                marker_id += 1
        pub.publish(markers)
        node.get_logger().info(f"published {len(markers.markers)} markers to {args.topic}")

    try:
        while rclpy.ok():
            publish_once()
            if args.once:
                break
            end = time.monotonic() + 1.0
            while time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
