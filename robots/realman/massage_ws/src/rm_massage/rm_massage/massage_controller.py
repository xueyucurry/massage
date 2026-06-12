from __future__ import annotations

import argparse
import time
from typing import Any

from .config_io import deep_get, load_yaml
from .ros_helpers import (
    assign_force_position_fields,
    assign_pose_command_fields,
    extract_force,
    get_message_class,
    make_pose_msg,
)
from .trajectory import build_tcp_trajectory, check_workspace
from .transforms import IDENTITY_T, add, quat_to_rpy, scale, transform_from_ros


class ForceWatcher:
    def __init__(self, node: Any, topic: str, msg_type: Any) -> None:
        self.node = node
        self.last: dict[str, float] | None = None
        self.last_time = 0.0
        self.sub = node.create_subscription(msg_type, topic, self._on_msg, 10)

    def _on_msg(self, msg: Any) -> None:
        try:
            self.last = extract_force(msg)
            self.last_time = time.monotonic()
        except Exception as exc:
            self.node.get_logger().warn(f"force parse failed: {exc}")

    def normal_force(self, axis: str, sign: float, timeout_s: float) -> float:
        age = time.monotonic() - self.last_time
        if self.last is None or age > timeout_s:
            raise RuntimeError("force sample timeout")
        return float(sign) * float(self.last.get(axis, 0.0))


class RealManCommandClient:
    def __init__(self, node: Any, frames: dict[str, Any]) -> None:
        self.node = node
        topics = frames.get("topics", {})
        types = frames.get("message_types", {})
        self.empty_type = get_message_class(types.get("empty", "std_msgs/msg/Empty"))
        self.movej_p_type = get_message_class(types.get("movej_p", "rm_ros_interfaces/msg/Movejp"))
        self.movel_type = get_message_class(types.get("movel", "rm_ros_interfaces/msg/Movel"))
        self.force_type = get_message_class(types.get("set_force_position", "rm_ros_interfaces/msg/Setforceposition"))

        self.pub_movej_p = node.create_publisher(self.movej_p_type, topics.get("movej_p_cmd", "/rm_driver/movej_p_cmd"), 10)
        self.pub_movel = node.create_publisher(self.movel_type, topics.get("movel_cmd", "/rm_driver/movel_cmd"), 10)
        self.pub_start_force = node.create_publisher(
            self.force_type,
            topics.get("set_force_position_cmd", "/rm_driver/set_force_postion_cmd"),
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

    def movej_p(self, position, quat, speed: float, wait_s: float) -> None:
        msg = self.movej_p_type()
        assign_pose_command_fields(msg, make_pose_msg(position, quat), speed)
        self._publish_repeated(self.pub_movej_p, msg)
        self.node.get_logger().info(f"MoveJ_P -> {[round(float(v), 4) for v in position]}")
        time.sleep(wait_s)

    def movel(self, position, quat, speed: float, wait_s: float, trajectory_connect: int = 0) -> None:
        msg = self.movel_type()
        assign_pose_command_fields(msg, make_pose_msg(position, quat), speed, trajectory_connect=trajectory_connect)
        self._publish_repeated(self.pub_movel, msg)
        self.node.get_logger().info(f"MoveL -> {[round(float(v), 4) for v in position]}")
        time.sleep(wait_s)

    def start_force(self, force_n: float) -> None:
        msg = self.force_type()
        assign_force_position_fields(msg, force_n=force_n, sensor=1, mode=1, direction=2)
        self._publish_repeated(self.pub_start_force, msg)
        self.node.get_logger().info(f"force position control on: {force_n:.1f} N")
        time.sleep(0.2)

    def stop_force(self) -> None:
        self._publish_repeated(self.pub_stop_force, self.empty_type())
        self.node.get_logger().info("force position control off")
        time.sleep(0.2)

    def motion_stop(self) -> None:
        self._publish_repeated(self.pub_motion_stop, self.empty_type())
        self.node.get_logger().warn("motion stop published")

    @staticmethod
    def _publish_repeated(pub: Any, msg: Any) -> None:
        for _ in range(3):
            pub.publish(msg)
            time.sleep(0.05)


class JsonCommandClient:
    def __init__(self, node: Any, host: str) -> None:
        import sys
        from pathlib import Path

        project_root = Path(__file__).resolve().parents[4]
        if str(project_root) not in sys.path:
            sys.path.append(str(project_root))
        from rm_demo import rm_json

        self.node = node
        self.host = host
        self.rm_json = rm_json
        node.get_logger().info(f"using RealMan JSON command backend: {host}:8080")

    def movej_p(self, position, quat, speed: float, wait_s: float) -> None:
        pose = self._pose6(position, quat)
        self.node.get_logger().info(f"JSON MoveJ_P -> {[round(float(v), 4) for v in position]}")
        self.rm_json.movej_p(self.host, pose, speed=max(1, int(round(float(speed)))), timeout=max(8.0, wait_s * 8.0))

    def movel(self, position, quat, speed: float, wait_s: float, trajectory_connect: int = 0) -> None:
        del trajectory_connect
        pose = self._pose6(position, quat)
        self.node.get_logger().info(f"JSON MoveL -> {[round(float(v), 4) for v in position]}")
        self.rm_json.movel(self.host, pose, speed=max(1, int(round(float(speed)))), timeout=max(8.0, wait_s * 8.0))

    def start_force(self, force_n: float) -> None:
        raise RuntimeError(
            "JSON backend force-position control is not enabled. "
            "Use --air-run or --contact-test, or provide a validated ROS2 rm_driver backend."
        )

    def stop_force(self) -> None:
        return

    def motion_stop(self) -> None:
        self.rm_json.stop_motion(self.host)
        self.node.get_logger().warn("JSON stop_motion sent")

    @staticmethod
    def _pose6(position, quat) -> list[float]:
        rpy = quat_to_rpy(quat)
        return [
            float(position[0]),
            float(position[1]),
            float(position[2]),
            float(rpy[0]),
            float(rpy[1]),
            float(rpy[2]),
        ]


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


def _select_line(lines_cfg: dict[str, Any], name: str | None, include_disabled: bool) -> dict[str, Any]:
    candidates = lines_cfg.get("lines", [])
    if name:
        for line in candidates:
            if line.get("name") == name:
                return line
        raise ValueError(f"line not found: {name}")
    for line in candidates:
        if include_disabled or line.get("enabled") is not False:
            return line
    raise ValueError("no enabled massage line found")


def _print_plan(line: dict[str, Any], trajectory) -> None:
    print(f"line: {line.get('name', '<unnamed>')}")
    print(f"points: {len(trajectory)}")
    for p in trajectory:
        print(
            f"  {p.index:02d} surface={_round3(p.surface_base)} "
            f"pre={_round3(p.pre_base)} contact={_round3(p.contact_base)}"
        )


def _round3(values) -> list[float]:
    return [round(float(v), 4) for v in values]


def _spin_for(node, seconds: float) -> None:
    import rclpy

    deadline = time.monotonic() + float(seconds)
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one force-guided massage line. Default is dry-run only.")
    parser.add_argument("--frames", default="config/frames.yaml")
    parser.add_argument("--safety", default="config/safety.yaml")
    parser.add_argument("--lines", default="config/massage_lines.yaml")
    parser.add_argument("--line", default=None)
    parser.add_argument("--include-disabled", action="store_true")
    parser.add_argument("--execute", action="store_true", help="actually publish arm and force commands")
    parser.add_argument("--backend", choices=("auto", "ros2", "json"), default="auto")
    parser.add_argument("--json-host", default="192.168.1.18")
    parser.add_argument("--air-run", action="store_true", help="execute pre-contact path only, no contact")
    parser.add_argument("--contact-test", action="store_true", help="approach until contact, then retreat")
    parser.add_argument("--motion-wait-s", type=float, default=2.0)
    parser.add_argument("--approach-step-m", type=float, default=0.001)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    frames = load_yaml(args.frames)
    safety = load_yaml(args.safety)
    lines_cfg = load_yaml(args.lines)
    line = _select_line(lines_cfg, args.line, args.include_disabled)

    base_frame = deep_get(frames, "frames.robot_base", "robot_base")
    line_frame = line.get("frame") or deep_get(frames, "frames.body", "body_frame")
    tcp_radius = float(deep_get(frames, "tcp.radius_m", 0.0))
    tcp_type = deep_get(frames, "tcp.type", "sphere")
    limits = deep_get(safety, "safety.workspace_limits_base", {})
    force_axis = deep_get(safety, "force.normal_axis", "fz")
    force_sign = float(deep_get(safety, "force.normal_sign", 1.0))
    force_timeout_s = float(deep_get(safety, "safety.force_timeout_s", 0.5))
    contact_force_n = float(deep_get(safety, "safety.contact_force_n", 1.0))
    max_force_n = float(deep_get(safety, "safety.force_max_n", 10.0))
    line_force_n = float(line.get("force_n", deep_get(safety, "safety.massage_force_n", 3.0)))

    rclpy.init()
    node = Node("rm_massage_controller")
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)
    del listener

    try:
        T_base_line = _lookup_line_transform(tf_buffer, node, base_frame, line_frame, float(deep_get(safety, "safety.tf_timeout_s", 0.2)) + 1.0)
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
        _print_plan(line, trajectory)

        if not args.execute:
            print("dry-run only. Add --execute to publish robot commands.")
            return

        backend = args.backend
        if backend == "auto":
            backend = "json" if deep_get(frames, "topics.six_force", "").startswith("/rm_json/") else "ros2"
        if backend == "json" and not (args.air_run or args.contact_test):
            raise RuntimeError(
                "Full force-line execution is disabled on JSON backend. "
                "Run --execute --air-run first, then --execute --contact-test. "
                "Use a validated rm_driver ROS2 backend for built-in force-position line following."
            )

        types = frames.get("message_types", {})
        force_type = get_message_class(types.get("six_force", "rm_ros_interfaces/msg/Sixforce"))
        force_topic = deep_get(frames, "topics.six_force", "/rm_driver/udp_six_force")
        watcher = ForceWatcher(node, force_topic, force_type)
        client = JsonCommandClient(node, args.json_host) if backend == "json" else RealManCommandClient(node, frames)
        _spin_for(node, 0.5)

        first = trajectory[0]
        last = trajectory[-1]
        speeds = safety.get("driver_speed", {})
        entry_speed = float(speeds.get("entry", 10.0))
        approach_speed = float(speeds.get("approach", 2.0))
        line_speed = float(speeds.get("line", 5.0))
        retreat_speed = float(speeds.get("retreat", 10.0))

        if args.air_run:
            for point in trajectory:
                client.movel(point.pre_base, point.quaternion_xyzw, line_speed, args.motion_wait_s)
                _spin_for(node, 0.1)
            return

        client.movej_p(first.pre_base, first.quaternion_xyzw, entry_speed, args.motion_wait_s)
        _spin_for(node, 0.2)

        current = first.pre_base
        max_approach = float(deep_get(safety, "safety.approach_distance_m", 0.03)) + tcp_radius + 0.010
        steps = max(1, int(max_approach / max(0.0005, float(args.approach_step_m))))
        touched = False
        for _ in range(steps):
            _spin_for(node, 0.05)
            f_normal = watcher.normal_force(force_axis, force_sign, force_timeout_s)
            if f_normal > max_force_n:
                client.motion_stop()
                raise RuntimeError(f"force too large during approach: {f_normal:.2f} N")
            if f_normal >= contact_force_n:
                touched = True
                node.get_logger().info(f"contact detected: {f_normal:.2f} N")
                break
            current = add(current, scale(first.normal_base, -float(args.approach_step_m)))
            client.movel(current, first.quaternion_xyzw, approach_speed, args.motion_wait_s)

        if not touched:
            client.movel(first.retreat_base, first.quaternion_xyzw, retreat_speed, args.motion_wait_s)
            raise RuntimeError("contact not detected within approach distance")

        if args.contact_test:
            client.movel(first.retreat_base, first.quaternion_xyzw, retreat_speed, args.motion_wait_s)
            return

        client.start_force(line_force_n)
        try:
            for point in trajectory:
                _spin_for(node, 0.05)
                f_normal = watcher.normal_force(force_axis, force_sign, force_timeout_s)
                if f_normal > max_force_n:
                    client.stop_force()
                    client.motion_stop()
                    raise RuntimeError(f"force too large while following line: {f_normal:.2f} N")
                client.movel(point.contact_base, point.quaternion_xyzw, line_speed, args.motion_wait_s)
        finally:
            client.stop_force()

        client.movel(last.retreat_base, last.quaternion_xyzw, retreat_speed, args.motion_wait_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
