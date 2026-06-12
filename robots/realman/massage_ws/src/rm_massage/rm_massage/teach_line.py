from __future__ import annotations

import argparse
import time
from pathlib import Path

from .config_io import load_yaml, save_yaml


def _lookup_transform(tf_buffer, node, target_frame: str, source_frame: str, timeout_s: float):
    import rclpy

    deadline = time.monotonic() + float(timeout_s)
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except Exception as exc:
            last_error = exc
            rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError(f"TF lookup failed {target_frame} <- {source_frame}: {last_error}")


def _load_or_empty(path: str) -> dict:
    resolved = Path(path).expanduser()
    if resolved.exists():
        return load_yaml(resolved)
    return {"lines": []}


def main() -> None:
    parser = argparse.ArgumentParser(description="Teach a massage line by recording current TCP TF positions.")
    parser.add_argument("--frames", default="config/frames.yaml")
    parser.add_argument("--output", default="config/massage_lines.yaml")
    parser.add_argument("--name", default="line_1")
    parser.add_argument("--target-frame", default=None, help="usually body_frame or robot_base")
    parser.add_argument("--source-frame", default=None, help="usually massage_tcp")
    parser.add_argument("--normal", nargs=3, type=float, default=[0.0, 0.0, 1.0])
    parser.add_argument("--speed-mps", type=float, default=0.005)
    parser.add_argument("--force-n", type=float, default=3.0)
    parser.add_argument("--count", type=int, default=0, help="0 means interactive until q")
    parser.add_argument("--tf-timeout-s", type=float, default=2.0)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    frames = load_yaml(args.frames)
    target_frame = args.target_frame or frames.get("frames", {}).get("body", "body_frame")
    source_frame = args.source_frame or frames.get("frames", {}).get("tcp", "massage_tcp")

    rclpy.init()
    node = Node("rm_teach_line")
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)
    del listener

    try:
        points: list[list[float]] = []
        print(f"Teaching {args.name}: record {source_frame} in {target_frame}.")
        print("Move TCP to a point, then press Enter. Type q and Enter to finish.")

        while True:
            if args.count > 0 and len(points) >= args.count:
                break
            if args.count == 0:
                text = input(f"point {len(points) + 1}> ").strip().lower()
                if text in ("q", "quit", "done"):
                    break
            else:
                input(f"point {len(points) + 1}/{args.count}> press Enter to capture ")

            tf_msg = _lookup_transform(tf_buffer, node, target_frame, source_frame, args.tf_timeout_s)
            tr = tf_msg.transform.translation
            point = [float(tr.x), float(tr.y), float(tr.z)]
            points.append(point)
            print(f"captured {len(points)}: {[round(v, 6) for v in point]}")

        if len(points) < 2:
            raise RuntimeError("need at least two points")

        data = _load_or_empty(args.output)
        lines = list(data.get("lines") or [])
        line = {
            "name": args.name,
            "frame": target_frame,
            "normal_body": [float(v) for v in args.normal],
            "speed_mps": float(args.speed_mps),
            "force_n": float(args.force_n),
            "enabled": True,
            "points_body": points,
        }
        lines = [existing for existing in lines if existing.get("name") != args.name]
        lines.append(line)
        data["lines"] = lines
        save_yaml(args.output, data)
        print(f"wrote {len(points)} points to {args.output}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
