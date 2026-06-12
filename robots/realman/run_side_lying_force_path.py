#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rm_demo import rm_json
from rm_demo.config import (
    ROS_GET_SIX_FORCE_CMD_TOPIC,
    ROS_GET_SIX_FORCE_TOPIC,
    ROS_SET_FORCE_SENSOR_TOPIC,
    ROS_VENDOR_PYTHON_DIR,
)
from rm_demo.rm_bladder import BladderMassageFrame, BladderMassagePlan


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_CONTACT_POSE = PROJECT_DIR / "rm_demo_output" / "user_confirmed_side_lying_contact_pose.json"


@dataclass
class PathTarget:
    index: int
    surface_m: np.ndarray
    press_m: np.ndarray


def _normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        raise RuntimeError(f"zero-length vector: {vec.tolist()}")
    return vec / norm


def _axis_vector(axis_name: str) -> np.ndarray:
    axes = {
        "pos_x": [1.0, 0.0, 0.0],
        "neg_x": [-1.0, 0.0, 0.0],
        "pos_y": [0.0, 1.0, 0.0],
        "neg_y": [0.0, -1.0, 0.0],
        "pos_z": [0.0, 0.0, 1.0],
        "neg_z": [0.0, 0.0, -1.0],
    }
    key = str(axis_name or "").strip().lower()
    if key not in axes:
        raise ValueError(f"unsupported tool axis: {axis_name}")
    return np.asarray(axes[key], dtype=np.float64)


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def _rotation_error_rad(actual_rpy: list[float], target_rpy: list[float]) -> float:
    actual = _rpy_to_matrix(*actual_rpy[:3])
    target = _rpy_to_matrix(*target_rpy[:3])
    delta = target.T @ actual
    cos_angle = (float(np.trace(delta)) - 1.0) * 0.5
    return float(math.acos(max(-1.0, min(1.0, cos_angle))))


def _lerp_angle(start: float, target: float, ratio: float) -> float:
    delta = (float(target) - float(start) + math.pi) % (2.0 * math.pi) - math.pi
    return float(start) + delta * float(ratio)


def _pose(surface_m: np.ndarray, press_m: np.ndarray, offset_m: float, rpy: list[float]) -> list[float]:
    xyz = np.asarray(surface_m, dtype=np.float64) + np.asarray(press_m, dtype=np.float64) * float(offset_m)
    return [float(xyz[0]), float(xyz[1]), float(xyz[2]), float(rpy[0]), float(rpy[1]), float(rpy[2])]


def _latest_plan() -> Path:
    candidates = sorted(
        glob.glob(str(PROJECT_DIR / "rm_demo_output" / "external_depth_normal_live_*_top_outer_depth_normal_plan.json")),
        key=os.path.getmtime,
        reverse=True,
    )
    if not candidates:
        raise RuntimeError("no external depth-normal top_outer plan found")
    return Path(candidates[0])


def load_plan(path: Path) -> BladderMassagePlan:
    data = json.loads(path.read_text(encoding="utf-8"))
    frame_fields = set(BladderMassageFrame.__dataclass_fields__.keys())
    frames = [
        BladderMassageFrame(**{key: value for key, value in frame.items() if key in frame_fields})
        for frame in data["frames"]
    ]
    return BladderMassagePlan(
        side=str(data["side"]),
        line_type=str(data["line_type"]),
        point_count=int(data["point_count"]),
        hover_m=float(data["hover_m"]),
        dian_jin_depth_m=float(data["dian_jin_depth_m"]),
        fen_jin_lateral_m=float(data["fen_jin_lateral_m"]),
        safe_z_m=float(data["safe_z_m"]),
        anchor_pose_m=[float(v) for v in data["anchor_pose_m"]],
        frames=frames,
        hover_offset_mode=str(data.get("hover_offset_mode", "normal")),
    )


def select_frames(plan: BladderMassagePlan, frame_start: int, frame_count: int) -> list[BladderMassageFrame]:
    start = max(1, int(frame_start)) if int(frame_start) > 0 else 1
    start_idx = start - 1
    if start_idx >= len(plan.frames):
        raise RuntimeError(f"frame_start {start} outside frame count {len(plan.frames)}")
    if int(frame_count) > 0:
        frames = plan.frames[start_idx : start_idx + int(frame_count)]
    else:
        frames = plan.frames[start_idx:]
    if not frames:
        raise RuntimeError("selected frame range is empty")
    return frames


def load_contact_rpy_from_json(path: Path) -> list[float]:
    data = json.loads(path.read_text(encoding="utf-8"))
    pose = data.get("tcp_pose_m_rpy")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"contact pose JSON missing tcp_pose_m_rpy: {path}")
    return [float(v) for v in pose[3:6]]


def load_contact_rpy(args: argparse.Namespace) -> list[float]:
    source = str(args.contact_pose_source).strip().lower()
    if source == "json":
        return load_contact_rpy_from_json(Path(args.contact_pose_json).resolve())
    if source == "current":
        _joints, pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(str(args.arm_host))
        if arm_err != 0 or sys_err != 0:
            raise RuntimeError(f"cannot use current contact pose while arm has errors: arm_err={arm_err} sys_err={sys_err}")
        return [float(v) for v in pose[3:6]]
    raise ValueError(f"unsupported contact pose source: {args.contact_pose_source}")


def save_current_contact_pose(host: str, output_dir: Path) -> Path:
    joints, pose, arm_err, sys_err, ik = rm_json.get_current_arm_state(str(host))
    if arm_err != 0 or sys_err != 0:
        raise RuntimeError(f"cannot lock current contact pose while arm has errors: arm_err={arm_err} sys_err={sys_err}")
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"locked_current_contact_pose_{time.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(
        json.dumps(
            {
                "tcp_pose_m_rpy": [float(v) for v in pose[:6]],
                "joint_deg": [float(v) for v in joints[:6]],
                "arm_error_state": {"arm_err": int(arm_err), "sys_err": int(sys_err), "inverse_km_err": int(ik)},
                "source": "locked before run_side_lying_force_path Docker delegation",
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    return path


def build_targets(
    frames: list[BladderMassageFrame],
    *,
    normal_mode: str,
    max_tangent_step_m: float,
    fixed_press_m: np.ndarray | None = None,
) -> list[PathTarget]:
    raw: list[PathTarget] = []
    presses = []
    for frame in frames:
        surface = np.asarray(frame.robot_point_m[:3], dtype=np.float64)
        press = _normalize(np.asarray(frame.press_direction_m[:3], dtype=np.float64))
        raw.append(PathTarget(index=int(frame.index), surface_m=surface, press_m=press))
        presses.append(press)
    if normal_mode == "tool_axis":
        if fixed_press_m is None:
            raise RuntimeError("normal_mode=tool_axis requires fixed_press_m")
        fixed = _normalize(np.asarray(fixed_press_m, dtype=np.float64))
        for target in raw:
            target.press_m = fixed
    elif normal_mode == "first":
        fixed = raw[0].press_m.copy()
        for target in raw:
            target.press_m = fixed
    elif normal_mode == "mean":
        fixed = _normalize(np.mean(np.asarray(presses, dtype=np.float64), axis=0))
        for target in raw:
            target.press_m = fixed
    elif normal_mode != "per_point":
        raise ValueError(f"unsupported normal mode: {normal_mode}")

    if len(raw) <= 1:
        return raw

    out: list[PathTarget] = []
    max_step = max(1e-6, float(max_tangent_step_m))
    for a, b in zip(raw[:-1], raw[1:]):
        if not out:
            out.append(a)
        dist = float(np.linalg.norm(b.surface_m - a.surface_m))
        step_count = max(1, int(math.ceil(dist / max_step)))
        for step_idx in range(1, step_count + 1):
            ratio = float(step_idx) / float(step_count)
            surface = a.surface_m * (1.0 - ratio) + b.surface_m * ratio
            press = _normalize(a.press_m * (1.0 - ratio) + b.press_m * ratio)
            out.append(PathTarget(index=b.index, surface_m=surface, press_m=press))
    return out


def _check_orientation_lock(host: str, rpy: list[float], max_error_rad: float) -> None:
    _joints, pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(host)
    if arm_err != 0 or sys_err != 0:
        raise RuntimeError(f"arm state is not clean: arm_err={arm_err} sys_err={sys_err}")
    err = _rotation_error_rad([float(v) for v in pose[3:6]], [float(v) for v in rpy[:3]])
    if err > float(max_error_rad):
        raise RuntimeError(
            f"orientation lock violated: error_rad={err:.4f} "
            f"current_rpy={[round(float(v), 6) for v in pose[3:6]]} "
            f"target_rpy={[round(float(v), 6) for v in rpy[:3]]}"
        )


def _move_segmented(
    host: str,
    target_pose: list[float],
    *,
    speed: int,
    max_step_m: float,
    timeout: float,
    max_orientation_error_rad: float,
) -> None:
    _joints, start_pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(host)
    if arm_err != 0 or sys_err != 0:
        raise RuntimeError(f"arm state is not clean before move: arm_err={arm_err} sys_err={sys_err}")
    target = [float(v) for v in target_pose[:6]]
    dist = float(np.linalg.norm(np.asarray(target[:3], dtype=np.float64) - np.asarray(start_pose[:3], dtype=np.float64)))
    steps = max(1, int(math.ceil(dist / max(1e-6, float(max_step_m)))))
    for step_idx in range(1, steps + 1):
        ratio = float(step_idx) / float(steps)
        pose = [0.0] * 6
        for axis in range(3):
            pose[axis] = float(start_pose[axis]) * (1.0 - ratio) + float(target[axis]) * ratio
        for axis in range(3, 6):
            pose[axis] = _lerp_angle(float(start_pose[axis]), float(target[axis]), ratio)
        rm_json.movel(host, pose, speed=int(speed), timeout=float(timeout))
        _check_orientation_lock(host, [float(v) for v in target[3:6]], max_orientation_error_rad)


def _force_vec(sample: dict[str, float], baseline: dict[str, float]) -> np.ndarray:
    return np.asarray(
        [
            float(sample["fx"] - baseline["fx"]),
            float(sample["fy"] - baseline["fy"]),
            float(sample["fz"] - baseline["fz"]),
        ],
        dtype=np.float64,
    )


def _force_value(
    sample: dict[str, float],
    baseline: dict[str, float],
    *,
    mode: str,
    tool_axis: str,
) -> float:
    delta = _force_vec(sample, baseline)
    key = str(mode).strip().lower()
    if key == "norm":
        return float(np.linalg.norm(delta))
    if key == "tool_axis_abs":
        return abs(float(np.dot(delta, _axis_vector(tool_axis))))
    if key == "fx_abs":
        return abs(float(delta[0]))
    if key == "fy_abs":
        return abs(float(delta[1]))
    if key == "fz_abs":
        return abs(float(delta[2]))
    if key == "fx_signed":
        return float(delta[0])
    if key == "fy_signed":
        return float(delta[1])
    if key == "fz_signed":
        return float(delta[2])
    raise ValueError(f"unsupported force measure mode: {mode}")


def _fmt_force(sample: dict[str, float] | None) -> str:
    if sample is None:
        return "force=n/a"
    return (
        f"Fx={sample['fx']:.2f} Fy={sample['fy']:.2f} Fz={sample['fz']:.2f} "
        f"Mx={sample['mx']:.2f} My={sample['my']:.2f} Mz={sample['mz']:.2f}"
    )


def _average_force(reader, *, count: int, timeout: float, interval_s: float) -> dict[str, float] | None:
    samples = []
    for _ in range(max(1, int(count))):
        sample = reader.request_sample(timeout=timeout)
        if sample is not None:
            samples.append(sample)
        time.sleep(max(0.0, float(interval_s)))
    if not samples:
        return None
    return {
        key: float(sum(sample[key] for sample in samples) / len(samples))
        for key in ("fx", "fy", "fz", "mx", "my", "mz")
    }


def _import_ros_modules():
    try:
        import rospy  # type: ignore
        from rm_msgs.msg import Six_Force  # type: ignore
        from std_msgs.msg import Empty  # type: ignore
    except Exception:
        candidates = []
        candidates.extend(glob.glob("/opt/ros/*/lib/python3/dist-packages"))
        candidates.append(str(ROS_VENDOR_PYTHON_DIR))
        for candidate in candidates:
            if candidate and os.path.isdir(candidate) and candidate not in sys.path:
                sys.path.append(candidate)
        import rospy  # type: ignore
        from rm_msgs.msg import Six_Force  # type: ignore
        from std_msgs.msg import Empty  # type: ignore
    return rospy, Six_Force, Empty


class ForceReader:
    def __init__(self) -> None:
        self.rospy, self.SixForce, self.Empty = _import_ros_modules()
        if not self.rospy.core.is_initialized():
            self.rospy.init_node("side_lying_force_reader", anonymous=True, disable_signals=True)
        self.last_force: dict[str, float] | None = None
        self.last_force_time = 0.0
        self.sub_force = self.rospy.Subscriber(ROS_GET_SIX_FORCE_TOPIC, self.SixForce, self._on_force, queue_size=5)
        self.pub_get_force = self.rospy.Publisher(ROS_GET_SIX_FORCE_CMD_TOPIC, self.Empty, queue_size=5)
        self.pub_set_force_sensor = self.rospy.Publisher(ROS_SET_FORCE_SENSOR_TOPIC, self.Empty, queue_size=5)
        self.rospy.sleep(0.2)

    def _on_force(self, msg) -> None:
        self.last_force = {
            "fx": float(msg.force_Fx),
            "fy": float(msg.force_Fy),
            "fz": float(msg.force_Fz),
            "mx": float(msg.force_Mx),
            "my": float(msg.force_My),
            "mz": float(msg.force_Mz),
        }
        self.last_force_time = time.time()

    def enable(self) -> None:
        self.pub_set_force_sensor.publish(self.Empty())
        self.rospy.sleep(0.2)

    def request_sample(self, timeout: float = 0.6) -> dict[str, float] | None:
        before = time.time()
        self.pub_get_force.publish(self.Empty())
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            if self.last_force is not None and self.last_force_time >= before:
                return dict(self.last_force)
            time.sleep(0.02)
        return None


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


def _delegate_run_to_docker(argv: list[str]) -> int | None:
    if os.environ.get("RUN_SIDE_LYING_FORCE_PATH_IN_DOCKER") == "1":
        return None
    if "--run" not in argv or _can_import_rospy():
        return None
    delegated_argv = list(argv)
    for idx, value in enumerate(list(delegated_argv)):
        if value == "--contact-pose-source" and idx + 1 < len(delegated_argv) and delegated_argv[idx + 1] == "current":
            lock_path = save_current_contact_pose("192.168.1.18", PROJECT_DIR / "rm_demo_output")
            delegated_argv[idx + 1] = "json"
            delegated_argv.extend(["--contact-pose-json", str(lock_path)])
            print(f"locked current contact pose before Docker delegation: {lock_path}")
            break
        if value == "--contact-pose-source=current":
            lock_path = save_current_contact_pose("192.168.1.18", PROJECT_DIR / "rm_demo_output")
            delegated_argv[idx] = "--contact-pose-source=json"
            delegated_argv.extend(["--contact-pose-json", str(lock_path)])
            print(f"locked current contact pose before Docker delegation: {lock_path}")
            break
    subprocess.run(_docker_command(["start", "noetic"]), check=True)
    inner_cmd = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        "export RUN_SIDE_LYING_FORCE_PATH_IN_DOCKER=1; "
        f"export PYTHONPATH={shlex.quote(str(PROJECT_DIR / 'ros_vendor' / 'python'))}:$PYTHONPATH; "
        f"cd {shlex.quote(str(PROJECT_DIR))}; "
        + " ".join(shlex.quote(part) for part in ["python3", "-u", str(PROJECT_DIR / "run_side_lying_force_path.py"), *delegated_argv])
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
    print("local Python has no rospy; delegating force-path run to Docker container noetic")
    return subprocess.run(_docker_command(docker_args)).returncode


def execute_force_path(args: argparse.Namespace, plan: BladderMassagePlan, targets: list[PathTarget], rpy: list[float]) -> None:
    if not args.allow_contact:
        raise RuntimeError("refusing contact run without --allow-contact")
    if float(args.target_force_n) <= 0.0:
        raise RuntimeError("--target-force-n must be positive")
    if float(args.max_force_n) <= float(args.target_force_n):
        raise RuntimeError("--max-force-n must be greater than --target-force-n")

    host = str(args.arm_host)
    hover_m = float(args.hover_mm) / 1000.0 if float(args.hover_mm) > 0.0 else float(plan.hover_m)
    max_press_m = float(args.max_press_mm) / 1000.0
    retreat_m = float(args.retreat_mm) / 1000.0
    approach_step_m = float(args.approach_step_mm) / 1000.0
    normal_step_m = float(args.normal_step_mm) / 1000.0
    deadband_n = float(args.force_deadband_n)
    kp_m_per_n = float(args.force_kp_mm_per_n) / 1000.0
    min_offset_m = -max(0.0, float(args.max_relief_mm) / 1000.0)

    if approach_step_m <= 0.0 or normal_step_m <= 0.0:
        raise RuntimeError("normal/approach step must be positive")

    rm_json.recover_if_needed(host)
    _joints, current_pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(host)
    if arm_err != 0 or sys_err != 0:
        raise RuntimeError(f"arm state is not clean: arm_err={arm_err} sys_err={sys_err}")

    first = targets[0]
    first_hover = _pose(first.surface_m, first.press_m, -hover_m, rpy)
    entry_dist = float(np.linalg.norm(np.asarray(first_hover[:3], dtype=np.float64) - np.asarray(current_pose[:3], dtype=np.float64)))
    print(f"entry_dist_m={entry_dist:.4f} first_hover={[round(v, 6) for v in first_hover[:3]]}")
    if entry_dist > float(args.max_entry_distance_m) and not args.allow_long_entry:
        raise RuntimeError(
            f"entry distance {entry_dist:.3f}m exceeds --max-entry-distance-m "
            f"{float(args.max_entry_distance_m):.3f}m; use --allow-long-entry only after checking the corridor"
        )

    reader = ForceReader()
    reader.enable()

    last_surface = first.surface_m
    last_press = first.press_m
    active_offset_m = -hover_m
    try:
        print("moving_to_first_hover")
        _move_segmented(
            host,
            first_hover,
            speed=int(args.entry_speed),
            max_step_m=float(args.entry_max_step_m),
            timeout=float(args.move_timeout_s),
            max_orientation_error_rad=float(args.max_orientation_error_rad),
        )
        baseline = _average_force(reader, count=8, timeout=1.0, interval_s=0.04)
        if baseline is None:
            raise RuntimeError("force baseline unavailable")
        print(f"force_baseline {_fmt_force(baseline)}")

        print("acquiring_contact")
        reached = False
        while active_offset_m < max_press_m:
            sample = _average_force(reader, count=max(1, int(args.force_filter_count)), timeout=0.8, interval_s=0.02)
            if sample is None:
                raise RuntimeError("force sample unavailable during contact acquisition")
            force_n = _force_value(sample, baseline, mode=args.force_measure, tool_axis=args.tool_contact_axis)
            print(f"approach offset_m={active_offset_m:.4f} force={force_n:.2f}N {_fmt_force(sample)}")
            if force_n >= float(args.max_force_n):
                raise RuntimeError(f"force exceeded during approach: {force_n:.2f}N")
            if force_n >= float(args.target_force_n) - deadband_n:
                reached = True
                break
            active_offset_m = min(max_press_m, active_offset_m + approach_step_m)
            pose = _pose(first.surface_m, first.press_m, active_offset_m, rpy)
            rm_json.movel(host, pose, speed=int(args.contact_speed), timeout=float(args.move_timeout_s))
            _check_orientation_lock(host, rpy, float(args.max_orientation_error_rad))
        if not reached:
            raise RuntimeError(
                f"target force not reached before max press offset {max_press_m:.4f}m; "
                "check visual surface depth and contact direction"
            )

        print("following_surface_path")
        last_force = 0.0
        for path_idx, target in enumerate(targets, start=1):
            sample = _average_force(reader, count=max(1, int(args.force_filter_count)), timeout=0.8, interval_s=0.02)
            if sample is None:
                raise RuntimeError("force sample unavailable during path")
            force_n = _force_value(sample, baseline, mode=args.force_measure, tool_axis=args.tool_contact_axis)
            if force_n >= float(args.max_force_n):
                raise RuntimeError(f"force exceeded during path: {force_n:.2f}N")
            if abs(force_n - last_force) > float(args.max_force_jump_n) and path_idx > 1:
                raise RuntimeError(f"force jump too large: prev={last_force:.2f}N now={force_n:.2f}N")
            last_force = force_n

            err_n = float(args.target_force_n) - force_n
            if abs(err_n) > deadband_n:
                signed_step = max(-normal_step_m, min(normal_step_m, err_n * kp_m_per_n))
                active_offset_m = max(min_offset_m, min(max_press_m, active_offset_m + signed_step))
            pose = _pose(target.surface_m, target.press_m, active_offset_m, rpy)
            rm_json.movel(host, pose, speed=int(args.contact_speed), timeout=float(args.move_timeout_s))
            _check_orientation_lock(host, rpy, float(args.max_orientation_error_rad))
            last_surface = target.surface_m
            last_press = target.press_m
            if path_idx == 1 or path_idx == len(targets) or path_idx % max(1, int(args.log_every)) == 0:
                print(
                    f"path {path_idx}/{len(targets)} offset_m={active_offset_m:.4f} "
                    f"force={force_n:.2f}N xyz={[round(v, 6) for v in pose[:3]]} {_fmt_force(sample)}"
                )
            time.sleep(max(0.0, float(args.control_period_s)))

        print("force_path_done")
    except BaseException as exc:
        print(f"force_path_aborted: {type(exc).__name__}: {exc}")
        try:
            rm_json.stop_motion(host)
        except Exception as stop_exc:
            print(f"stop_motion_failed: {stop_exc}")
        raise
    finally:
        try:
            retreat_pose = _pose(last_surface, last_press, -retreat_m, rpy)
            print(f"retreating_to_offset_m={-retreat_m:.4f} xyz={[round(v, 6) for v in retreat_pose[:3]]}")
            rm_json.movel(host, retreat_pose, speed=int(args.entry_speed), timeout=float(args.move_timeout_s))
            _check_orientation_lock(host, rpy, float(args.max_orientation_error_rad))
        except Exception as retreat_exc:
            print(f"retreat_failed: {retreat_exc}")


def print_preview(
    *,
    plan_path: Path,
    plan: BladderMassagePlan,
    targets: list[PathTarget],
    rpy: list[float],
    args: argparse.Namespace,
) -> None:
    hover_m = float(args.hover_mm) / 1000.0 if float(args.hover_mm) > 0.0 else float(plan.hover_m)
    rot = _rpy_to_matrix(*rpy)
    tool_axis_world = _normalize(rot @ _axis_vector(args.tool_contact_axis))
    dots = [float(np.dot(tool_axis_world, target.press_m)) for target in targets]
    first_hover = _pose(targets[0].surface_m, targets[0].press_m, -hover_m, rpy)
    first_surface_pose = _pose(targets[0].surface_m, targets[0].press_m, 0.0, rpy)
    last_surface_pose = _pose(targets[-1].surface_m, targets[-1].press_m, 0.0, rpy)
    print(f"plan_json={plan_path}")
    print(f"selected_frames={len(set(t.index for t in targets))} interpolated_targets={len(targets)}")
    print(f"target_force_n={float(args.target_force_n):.2f} max_force_n={float(args.max_force_n):.2f}")
    print(f"hover_m={hover_m:.4f} max_press_m={float(args.max_press_mm) / 1000.0:.4f}")
    print(f"contact_pose_source={args.contact_pose_source}")
    print(f"normal_mode={args.normal_mode}")
    print(f"fixed_contact_rpy={[round(v, 6) for v in rpy]}")
    print(
        f"tool_axis={args.tool_contact_axis} axis_world={[round(float(v), 6) for v in tool_axis_world.tolist()]} "
        f"dot_press_min={min(dots):.4f} dot_press_max={max(dots):.4f}"
    )
    print(f"first_hover_xyz={[round(v, 6) for v in first_hover[:3]]}")
    print(f"first_surface_xyz={[round(v, 6) for v in first_surface_pose[:3]]}")
    print(f"last_surface_xyz={[round(v, 6) for v in last_surface_pose[:3]]}")
    print(f"force_measure={args.force_measure}; no product ForcePosition topics are used by this script")
    if args.normal_mode != "tool_axis" and max(abs(v) for v in dots) < 0.80:
        print("warning: selected tool axis is not strongly aligned with the visual press normal; "
              "the script still uses visual press direction for motion")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Side-lying bladder meridian force path with an external force loop.")
    parser.add_argument("--plan-json", default="", help="saved depth-normal *_plan.json; defaults to newest top_outer plan")
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--contact-pose-source", choices=("json", "current"), default="json")
    parser.add_argument("--contact-pose-json", default=str(DEFAULT_CONTACT_POSE))
    parser.add_argument("--frame-start", type=int, default=0)
    parser.add_argument("--frame-count", type=int, default=0)
    parser.add_argument(
        "--normal-mode",
        choices=("tool_axis", "per_point", "first", "mean"),
        default="tool_axis",
        help="tool_axis uses the fixed contact pose/tool axis as the single approach direction",
    )
    parser.add_argument("--tool-contact-axis", choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"), default="pos_x")
    parser.add_argument(
        "--force-measure",
        choices=("norm", "tool_axis_abs", "fx_abs", "fy_abs", "fz_abs", "fx_signed", "fy_signed", "fz_signed"),
        default="norm",
    )
    parser.add_argument("--target-force-n", type=float, default=10.0)
    parser.add_argument("--max-force-n", type=float, default=14.0)
    parser.add_argument("--force-deadband-n", type=float, default=0.8)
    parser.add_argument("--force-kp-mm-per-n", type=float, default=0.25)
    parser.add_argument("--force-filter-count", type=int, default=3)
    parser.add_argument("--max-force-jump-n", type=float, default=6.0)
    parser.add_argument("--hover-mm", type=float, default=0.0, help="<=0 uses plan.hover_m")
    parser.add_argument("--max-press-mm", type=float, default=8.0)
    parser.add_argument("--max-relief-mm", type=float, default=20.0)
    parser.add_argument("--retreat-mm", type=float, default=60.0)
    parser.add_argument("--approach-step-mm", type=float, default=1.0)
    parser.add_argument("--normal-step-mm", type=float, default=0.6)
    parser.add_argument("--max-tangent-step-m", type=float, default=0.010)
    parser.add_argument("--entry-max-step-m", type=float, default=0.020)
    parser.add_argument("--entry-speed", type=int, default=4)
    parser.add_argument("--contact-speed", type=int, default=2)
    parser.add_argument("--move-timeout-s", type=float, default=20.0)
    parser.add_argument("--control-period-s", type=float, default=0.02)
    parser.add_argument("--max-entry-distance-m", type=float, default=0.35)
    parser.add_argument("--max-orientation-error-rad", type=float, default=0.25)
    parser.add_argument("--allow-long-entry", action="store_true")
    parser.add_argument("--log-every", type=int, default=5)
    parser.add_argument("--allow-contact", action="store_true")
    parser.add_argument("--run", action="store_true")
    return parser.parse_args()


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    delegated = _delegate_run_to_docker(argv)
    if delegated is not None:
        return int(delegated)

    args = parse_args()
    plan_path = Path(args.plan_json).resolve() if args.plan_json else _latest_plan()
    plan = load_plan(plan_path)
    rpy = load_contact_rpy(args)
    fixed_press_m = _normalize(_rpy_to_matrix(*rpy) @ _axis_vector(args.tool_contact_axis))
    frames = select_frames(plan, args.frame_start, args.frame_count)
    targets = build_targets(
        frames,
        normal_mode=args.normal_mode,
        max_tangent_step_m=args.max_tangent_step_m,
        fixed_press_m=fixed_press_m,
    )
    print_preview(plan_path=plan_path, plan=plan, targets=targets, rpy=rpy, args=args)
    if not args.run:
        print("preview only; add --run --allow-contact to execute")
        return 0
    execute_force_path(args, plan, targets, rpy)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
