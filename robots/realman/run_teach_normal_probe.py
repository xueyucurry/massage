#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from rm_demo import rm_json
from rm_demo.rm_bladder import _average_force_samples, _force_delta_n, _format_force_sample, _normalize_vec, _rpy_to_matrix
from rm_demo.rm_execute import RosForceBridge


PROJECT_DIR = Path(__file__).resolve().parent
AXES = ("x", "y", "z")


def _can_import_rospy() -> bool:
    try:
        import rospy  # type: ignore  # noqa: F401

        return True
    except Exception:
        return False


def _docker_command(argv: list[str]) -> list[str]:
    try:
        subprocess.run(["docker", "version"], check=True, capture_output=True, timeout=3)
        return ["docker", *argv]
    except Exception:
        return ["sg", "docker", "-c", " ".join(shlex.quote(part) for part in ["docker", *argv])]


def _delegate_to_docker(args: argparse.Namespace, plan_path: Path) -> int | None:
    if os.environ.get("RUN_TEACH_NORMAL_PROBE_IN_DOCKER") == "1" or _can_import_rospy():
        return None
    if not args.run:
        return None

    subprocess.run(_docker_command(["start", "noetic"]), check=True)
    inner_args = [
        "python3",
        str(PROJECT_DIR / "run_teach_normal_probe.py"),
        "--plan-json",
        str(plan_path),
        "--arm-host",
        str(args.arm_host),
        "--frame-index",
        str(args.frame_index),
        "--target-force-n",
        str(args.target_force_n),
        "--max-force-n",
        str(args.max_force_n),
        "--normal-step-mm",
        str(args.normal_step_mm),
        "--max-extra-mm",
        str(args.max_extra_mm),
        "--teach-speed",
        str(args.teach_speed),
        "--axis-tolerance-mm",
        str(args.axis_tolerance_mm),
        "--run",
    ]
    if args.tool_name:
        inner_args.extend(["--tool-name", str(args.tool_name)])
    if args.work_frame:
        inner_args.extend(["--work-frame", str(args.work_frame)])
    if args.allow_experimental_teach:
        inner_args.append("--allow-experimental-teach")
    inner_cmd = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        "export RUN_TEACH_NORMAL_PROBE_IN_DOCKER=1; "
        f"export PYTHONPATH={shlex.quote(str(PROJECT_DIR / 'ros_vendor' / 'python'))}:$PYTHONPATH; "
        f"cd {shlex.quote(str(PROJECT_DIR))}; "
        + " ".join(shlex.quote(part) for part in inner_args)
    )
    docker_args = [
        "exec",
        "-i",
        "-e",
        "ROS_MASTER_URI=http://192.168.1.11:11311",
        "-e",
        "ROS_IP=192.168.1.250",
        "-e",
        "ROS_HOSTNAME=192.168.1.250",
        "noetic",
        "bash",
        "-lc",
        inner_cmd,
    ]
    print("local Python has no rospy; delegating teach normal probe to Docker container noetic")
    return subprocess.run(_docker_command(docker_args)).returncode


def _load_frame(plan_path: Path, frame_index: int) -> tuple[dict, dict]:
    data = json.loads(plan_path.read_text(encoding="utf-8"))
    frames = list(data["frames"])
    if not frames:
        raise RuntimeError("saved plan has no frames")
    idx = max(1, min(len(frames), int(frame_index))) - 1
    return data, frames[idx]


def _pose_dist_m(a: list[float], b: list[float]) -> float:
    return float(np.linalg.norm(np.asarray(a[:3], dtype=np.float64) - np.asarray(b[:3], dtype=np.float64)))


def _setup_frames(tool_name: str, work_frame: str) -> None:
    if not tool_name and not work_frame:
        return
    from rm_demo.rm_ros import create_arm_backend

    arm = create_arm_backend("ros")
    if work_frame:
        print(f"switching work frame to {work_frame}")
        arm.change_work_frame(work_frame)
    if tool_name:
        print(f"switching tool frame to {tool_name}")
        arm.change_tool(tool_name)


def _force_delta(bridge: RosForceBridge, baseline: dict[str, float]) -> tuple[float | None, dict[str, float] | None]:
    sample = _average_force_samples(bridge, count=2, timeout=0.5, interval_s=0.02)
    return _force_delta_n(sample, baseline), sample


def _stop_teach_quiet(host: str) -> None:
    try:
        rm_json.stop_teach(host)
    except Exception as exc:
        print(f"stop_teach failed: {exc}")


def _jog_axis_delta(
    *,
    host: str,
    axis: int,
    delta_m: float,
    speed: int,
    tolerance_m: float,
    timeout_s: float,
    bridge: RosForceBridge | None = None,
    baseline: dict[str, float] | None = None,
    target_force_n: float = math.inf,
    max_force_n: float = math.inf,
) -> tuple[str, float, float]:
    if abs(float(delta_m)) <= float(tolerance_m):
        return "skip", 0.0, 0.0
    axis_name = AXES[int(axis)]
    sign = 1.0 if float(delta_m) >= 0.0 else -1.0
    direction = "pos" if sign > 0.0 else "neg"
    start_pose = rm_json.get_current_arm_state(host)[1]
    start_coord = float(start_pose[axis])
    target_abs = abs(float(delta_m))
    max_seen = 0.0
    status = "reached"
    rm_json.set_pos_teach(host, axis_name, direction, int(speed))
    start_time = time.time()
    try:
        while True:
            pose = rm_json.get_current_arm_state(host)[1]
            moved = sign * (float(pose[axis]) - start_coord)
            if bridge is not None and baseline is not None:
                delta_n, sample = _force_delta(bridge, baseline)
                if delta_n is not None:
                    max_seen = max(max_seen, float(delta_n))
                    if delta_n >= float(max_force_n):
                        status = "max_force"
                        break
                    if delta_n >= float(target_force_n):
                        status = "target_force"
                        break
            if moved >= target_abs - float(tolerance_m):
                break
            if time.time() - start_time > float(timeout_s):
                status = "timeout"
                break
            time.sleep(0.02)
    finally:
        _stop_teach_quiet(host)
    end_pose = rm_json.get_current_arm_state(host)[1]
    actual = sign * (float(end_pose[axis]) - start_coord)
    return status, float(actual), float(max_seen)


def _jog_vector_components(
    *,
    host: str,
    vector_m: np.ndarray,
    speed: int,
    tolerance_m: float,
    bridge: RosForceBridge | None = None,
    baseline: dict[str, float] | None = None,
    target_force_n: float = math.inf,
    max_force_n: float = math.inf,
) -> tuple[str, float]:
    order = sorted(range(3), key=lambda i: abs(float(vector_m[i])), reverse=True)
    max_seen = 0.0
    for axis in order:
        status, actual, axis_max = _jog_axis_delta(
            host=host,
            axis=axis,
            delta_m=float(vector_m[axis]),
            speed=int(speed),
            tolerance_m=float(tolerance_m),
            timeout_s=max(3.0, abs(float(vector_m[axis])) * 2500.0),
            bridge=bridge,
            baseline=baseline,
            target_force_n=target_force_n,
            max_force_n=max_force_n,
        )
        max_seen = max(max_seen, axis_max)
        if status in ("target_force", "max_force", "timeout"):
            return status, max_seen
    return "reached", max_seen


def _retreat_to_hover(host: str, hover_xyz: np.ndarray, speed: int, tolerance_m: float) -> None:
    current = np.asarray(rm_json.get_current_arm_state(host)[1][:3], dtype=np.float64)
    delta = hover_xyz - current
    print(f"retreat_to_hover delta={[round(float(v), 5) for v in delta.tolist()]}")
    status, _ = _jog_vector_components(
        host=host,
        vector_m=delta,
        speed=max(1, int(speed)),
        tolerance_m=max(float(tolerance_m), 0.0005),
    )
    print(f"retreat_to_hover status={status} pose={rm_json.get_current_arm_state(host)[1][:3]}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe along a saved side-lying normal using low-speed position teach jogs.")
    parser.add_argument("--plan-json", required=True)
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--frame-index", type=int, default=8)
    parser.add_argument("--tool-name", default="")
    parser.add_argument("--work-frame", default="")
    parser.add_argument("--target-force-n", type=float, default=0.8)
    parser.add_argument("--max-force-n", type=float, default=3.0)
    parser.add_argument("--normal-step-mm", type=float, default=1.0)
    parser.add_argument("--max-extra-mm", type=float, default=0.0)
    parser.add_argument("--teach-speed", type=int, default=1)
    parser.add_argument("--axis-tolerance-mm", type=float, default=0.15)
    parser.add_argument("--allow-experimental-teach", action="store_true")
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()

    plan_path = Path(args.plan_json).resolve()
    delegated_rc = _delegate_to_docker(args, plan_path)
    if delegated_rc is not None:
        return delegated_rc

    data, frame = _load_frame(plan_path, args.frame_index)
    hover_pose = [float(v) for v in frame["hover_pose_m"][:6]]
    hover_xyz = np.asarray(hover_pose[:3], dtype=np.float64)
    press = _normalize_vec(np.asarray(frame["press_direction_m"][:3], dtype=np.float64))
    if press is None:
        raise RuntimeError("invalid press_direction_m")
    tool_z = _normalize_vec(_rpy_to_matrix(*hover_pose[3:6])[:, 2])
    dot = float(np.dot(press, tool_z)) if tool_z is not None else float("nan")
    max_depth_m = float(data["hover_m"]) + max(0.0, float(args.max_extra_mm) / 1000.0)
    step_m = max(0.0005, float(args.normal_step_mm) / 1000.0)
    tolerance_m = max(0.00005, float(args.axis_tolerance_mm) / 1000.0)
    print(
        f"frame={frame.get('index')} hover={[round(v, 6) for v in hover_pose[:3]]} "
        f"press={[round(float(v), 6) for v in press.tolist()]} tool_z_dot={dot:.4f} "
        f"max_depth_m={max_depth_m:.4f} step_m={step_m:.4f}"
    )
    if dot < 0.985:
        raise RuntimeError("saved press direction is not aligned with tool +Z")
    if not args.run:
        print("preview only; add --run to execute teach normal probe")
        return 0
    if not args.allow_experimental_teach:
        raise RuntimeError("teach normal probe is experimental and disabled without --allow-experimental-teach")

    _setup_frames(args.tool_name, args.work_frame)
    current_pose = rm_json.get_current_arm_state(args.arm_host)[1]
    dist = _pose_dist_m(current_pose, hover_pose)
    print(f"current_pose={[round(v, 6) for v in current_pose[:6]]} dist_to_hover_m={dist:.4f}")
    if dist > 0.008:
        raise RuntimeError("current TCP is not at the selected hover point; run hover_path first")

    bridge = RosForceBridge()
    bridge.enable_force_sensor()
    time.sleep(0.2)
    baseline = _average_force_samples(bridge, count=5, timeout=1.0, interval_s=0.05)
    if baseline is None:
        raise RuntimeError("force baseline unavailable")
    print(f"baseline {_format_force_sample(baseline)}")

    reached = False
    max_seen = 0.0
    try:
        while True:
            current_xyz = np.asarray(rm_json.get_current_arm_state(args.arm_host)[1][:3], dtype=np.float64)
            depth = max(0.0, float(np.dot(current_xyz - hover_xyz, press)))
            if depth >= max_depth_m:
                print(f"max depth reached without target force: depth_m={depth:.4f}")
                break
            inc = press * min(step_m, max_depth_m - depth)
            status, step_max = _jog_vector_components(
                host=args.arm_host,
                vector_m=inc,
                speed=max(1, int(args.teach_speed)),
                tolerance_m=tolerance_m,
                bridge=bridge,
                baseline=baseline,
                target_force_n=float(args.target_force_n),
                max_force_n=float(args.max_force_n),
            )
            max_seen = max(max_seen, step_max)
            sample = _average_force_samples(bridge, count=2, timeout=0.5, interval_s=0.02)
            delta_n = _force_delta_n(sample, baseline)
            if delta_n is not None:
                max_seen = max(max_seen, float(delta_n))
            current_xyz = np.asarray(rm_json.get_current_arm_state(args.arm_host)[1][:3], dtype=np.float64)
            depth = max(0.0, float(np.dot(current_xyz - hover_xyz, press)))
            delta_text = "n/a" if delta_n is None else f"{delta_n:.2f}N"
            print(
                f"probe status={status} depth_m={depth:.4f} delta={delta_text} "
                f"pose={[round(float(v), 6) for v in current_xyz.tolist()]}"
            )
            if status == "max_force" or (delta_n is not None and delta_n >= float(args.max_force_n)):
                raise RuntimeError(f"max force exceeded: max_seen={max_seen:.2f}N")
            if status == "target_force" or (delta_n is not None and delta_n >= float(args.target_force_n)):
                reached = True
                print(f"target reached: max_seen={max_seen:.2f}N")
                break
            if status == "timeout":
                raise RuntimeError("teach jog timed out")
    finally:
        _stop_teach_quiet(args.arm_host)
        _retreat_to_hover(args.arm_host, hover_xyz, speed=max(1, int(args.teach_speed)), tolerance_m=tolerance_m)
    return 0 if reached else 2


if __name__ == "__main__":
    raise SystemExit(main())
