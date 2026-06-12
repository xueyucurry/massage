from __future__ import annotations

import argparse
import statistics
import time
from pathlib import Path
from typing import Any

from .config_io import deep_get, load_yaml, save_yaml
from .ros_helpers import extract_force, get_message_class


class ForceSampler:
    def __init__(self, node: Any, topic: str, msg_type: Any) -> None:
        self.node = node
        self.samples: list[dict[str, float]] = []
        self.last: dict[str, float] | None = None
        self.last_time = 0.0
        self.sub = node.create_subscription(msg_type, topic, self._on_msg, 10)

    def _on_msg(self, msg: Any) -> None:
        try:
            sample = extract_force(msg)
        except Exception as exc:
            self.node.get_logger().error(f"force parse failed: {exc}")
            return
        self.last = sample
        self.last_time = time.monotonic()
        self.samples.append(sample)

    def collect(self, seconds: float, spin_once) -> list[dict[str, float]]:
        start_len = len(self.samples)
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            spin_once()
        return self.samples[start_len:]


def _mean(samples: list[dict[str, float]]) -> dict[str, float]:
    if not samples:
        raise RuntimeError("no force samples received")
    keys = sorted({key for sample in samples for key in sample.keys()})
    return {key: statistics.fmean(sample[key] for sample in samples if key in sample) for key in keys}


def _fmt_force(sample: dict[str, float]) -> str:
    return " ".join(f"{key}={sample.get(key, 0.0):+.3f}" for key in ("fx", "fy", "fz", "mx", "my", "mz"))


def _write_sign(path: str, axis: str, sign: float) -> None:
    data = load_yaml(path)
    data.setdefault("force", {})
    data["force"]["normal_axis"] = axis
    data["force"]["normal_sign"] = float(sign)
    save_yaml(path, data)


def main() -> None:
    parser = argparse.ArgumentParser(description="Check six-axis force sign for the massage TCP.")
    parser.add_argument("--frames", default="config/frames.yaml")
    parser.add_argument("--safety", default="config/safety.yaml")
    parser.add_argument("--topic", default=None)
    parser.add_argument("--type", default=None, dest="type_name")
    parser.add_argument("--axis", default=None, choices=["auto", "fx", "fy", "fz"])
    parser.add_argument("--baseline-s", type=float, default=2.0)
    parser.add_argument("--press-s", type=float, default=6.0)
    parser.add_argument("--min-delta-n", type=float, default=0.8)
    parser.add_argument("--force-save", action="store_true", help="save even when the measured force change is small")
    parser.add_argument("--save", action="store_true", help="write the suggested sign into safety.yaml")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node

    frames = load_yaml(args.frames)
    safety = load_yaml(args.safety)
    topic = args.topic or deep_get(frames, "topics.six_force", "/rm_driver/udp_six_force")
    type_name = args.type_name or deep_get(frames, "message_types.six_force", "rm_ros_interfaces/msg/Sixforce")
    requested_axis = args.axis or deep_get(safety, "force.normal_axis", "fz")

    rclpy.init()
    node = Node("rm_check_force_axis")
    try:
        msg_type = get_message_class(type_name)
        sampler = ForceSampler(node, topic, msg_type)

        def spin_once() -> None:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().info(f"subscribing {topic} [{type_name}]")
        node.get_logger().info("keep the massage head unloaded for baseline")
        baseline_samples = sampler.collect(args.baseline_s, spin_once)
        baseline = _mean(baseline_samples)
        print(f"baseline: {_fmt_force(baseline)}")

        print(f"Press the massage head along tool +Z now. Recording {args.press_s:.1f}s...")
        press_samples = sampler.collect(args.press_s, spin_once)
        if not press_samples:
            raise RuntimeError("no press samples received")

        axis_peaks: dict[str, tuple[float, dict[str, float]]] = {}
        for candidate in ("fx", "fy", "fz"):
            deltas = [
                (sample.get(candidate, 0.0) - baseline.get(candidate, 0.0), sample)
                for sample in press_samples
            ]
            axis_peaks[candidate] = max(deltas, key=lambda item: abs(item[0]))

        if requested_axis == "auto":
            axis = max(axis_peaks, key=lambda key: abs(axis_peaks[key][0]))
        else:
            axis = requested_axis
        peak_delta, peak_sample = axis_peaks[axis]
        sign = 1.0 if peak_delta >= 0.0 else -1.0
        print(f"peak:     {_fmt_force(peak_sample)}")
        print(
            "axis_peaks: "
            + " ".join(
                f"{candidate}={axis_peaks[candidate][0]:+.3f}N"
                for candidate in ("fx", "fy", "fz")
            )
        )
        print(f"selected_axis: {axis}")
        print(f"axis_delta_{axis}: {peak_delta:+.3f} N")
        print(f"suggested force.normal_axis: {axis}")
        print(f"suggested force.normal_sign: {sign:+.1f}")
        strong_enough = abs(peak_delta) >= float(args.min_delta_n)
        if not strong_enough:
            print(
                "WARNING: force change is small. Repeat this check and press more clearly "
                f"before contact tests; expected >= {float(args.min_delta_n):.2f} N."
            )

        if args.save:
            if strong_enough or args.force_save:
                _write_sign(str(Path(args.safety)), axis, sign)
                print(f"updated {args.safety}")
            else:
                print(f"not updating {args.safety}; use --force-save to override")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
