#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import select as select_mod
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from build_bladder_depth_normal_diagnostic import select_line as select_bladder_line
from rm_demo import rm_json
from rm_demo.rm_bladder import detect_bladder_lines, save_bladder_artifacts
from rm_demo.rm_transform import load_transform_matrix, transform_points
from run_depth_normal_bladder_live_hover import _build_depth_normal_plan
from run_external_bladder_live_hover import _intrinsics_from_profile, _read_aligned_frame, _stream_profile


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_CONTACT_POSE = PROJECT_DIR / "rm_demo_output" / "user_confirmed_side_lying_contact_pose.json"


@dataclass
class PointTarget:
    index: int
    source_index: int
    pixel: list[float]
    hover_pixel: list[float]
    surface_m: np.ndarray
    press_m: np.ndarray
    rpy: list[float]


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


def _force_vec(sample: dict[str, float], baseline: dict[str, float]) -> np.ndarray:
    return np.asarray(
        [
            float(sample["fx"] - baseline["fx"]),
            float(sample["fy"] - baseline["fy"]),
            float(sample["fz"] - baseline["fz"]),
        ],
        dtype=np.float64,
    )


def _force_value(sample: dict[str, float], baseline: dict[str, float], mode: str) -> float:
    delta = _force_vec(sample, baseline)
    key = str(mode).strip().lower()
    if key == "norm":
        return float(np.linalg.norm(delta))
    if key == "fx_abs":
        return abs(float(delta[0]))
    if key == "fy_abs":
        return abs(float(delta[1]))
    if key == "fz_abs":
        return abs(float(delta[2]))
    raise ValueError(f"unsupported force measure: {mode}")


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
        _check_orientation_lock(host, [float(v) for v in pose[3:6]], max_orientation_error_rad)


class DockerForceReader:
    """Long-lived Docker/ROS force sampler.

    This lets the OpenCV/RealSense loop stay on the host while ROS force topics
    are read inside the existing noetic container.
    """

    def __init__(self) -> None:
        subprocess.run(["docker", "start", "noetic"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        code = r'''
import json
import sys
from rm_demo.rm_execute import RosForceBridge

bridge = RosForceBridge()
bridge.enable_force_sensor()
print("READY", flush=True)
for raw in sys.stdin:
    cmd = raw.strip()
    if cmd == "sample":
        sample = bridge.request_force_sample(timeout=1.0)
        print(json.dumps(sample, ensure_ascii=False), flush=True)
    elif cmd == "quit":
        print("BYE", flush=True)
        break
    else:
        print(json.dumps({"error": "unknown command", "cmd": cmd}), flush=True)
'''
        inner = (
            "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
            "export PYTHONPATH=/home/franka/massage/robots/realman/ros_vendor/python:$PYTHONPATH; "
            "cd /home/franka/massage/robots/realman; "
            f"python3 -u -c {shlex.quote(code)}"
        )
        cmd = [
            "docker",
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
            inner,
        ]
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        ready = self._readline(timeout=8.0)
        if ready != "READY":
            raise RuntimeError(f"force reader did not become ready: {ready!r}")

    def _readline(self, timeout: float) -> str:
        if self.proc.stdout is None:
            raise RuntimeError("force reader stdout is closed")
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            ready, _w, _x = select_mod.select([self.proc.stdout], [], [], 0.1)
            if ready:
                line = self.proc.stdout.readline()
                if line == "":
                    raise RuntimeError("force reader exited")
                return line.strip()
            if self.proc.poll() is not None:
                stderr = "" if self.proc.stderr is None else self.proc.stderr.read()
                raise RuntimeError(f"force reader exited with code {self.proc.returncode}: {stderr}")
        raise TimeoutError("force reader timed out")

    def request_sample(self, timeout: float = 1.2) -> dict[str, float] | None:
        if self.proc.stdin is None:
            raise RuntimeError("force reader stdin is closed")
        self.proc.stdin.write("sample\n")
        self.proc.stdin.flush()
        line = self._readline(timeout=timeout + 1.0)
        if not line or line in ("null", "None"):
            return None
        data = json.loads(line)
        if not isinstance(data, dict) or "error" in data:
            return None
        return {key: float(data[key]) for key in ("fx", "fy", "fz", "mx", "my", "mz")}

    def close(self) -> None:
        try:
            if self.proc.stdin is not None and self.proc.poll() is None:
                self.proc.stdin.write("quit\n")
                self.proc.stdin.flush()
        except Exception:
            pass
        try:
            self.proc.terminate()
        except Exception:
            pass


def _even_indices(length: int, count: int) -> list[int]:
    if length <= 0:
        raise RuntimeError("cannot sample from empty line")
    if count <= 1:
        return [length // 2]
    return [int(round(i * (length - 1) / float(count - 1))) for i in range(count)]


def _valid_camera_point(point: object) -> bool:
    if not isinstance(point, (list, tuple)) or len(point) < 3:
        return False
    arr = np.asarray(point[:3], dtype=np.float64)
    return bool(np.all(np.isfinite(arr)) and float(arr[2]) > 0.0)


def _load_contact_rpy(args: argparse.Namespace) -> tuple[list[float], list[float] | None]:
    source = str(args.contact_pose_source).strip().lower()
    if source == "current":
        joints, pose, arm_err, sys_err, ik = rm_json.get_current_arm_state(str(args.arm_host))
        if arm_err != 0 or sys_err != 0:
            raise RuntimeError(f"cannot lock current contact pose: arm_err={arm_err} sys_err={sys_err}")
        return [float(v) for v in pose[3:6]], [float(v) for v in pose[:6]]
    path = Path(args.contact_pose_json).resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    pose = data.get("tcp_pose_m_rpy")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"contact pose JSON missing tcp_pose_m_rpy: {path}")
    return [float(v) for v in pose[3:6]], [float(v) for v in pose[:6]]


def _build_top_outer_targets(
    *,
    args: argparse.Namespace,
    detection: dict[str, object],
    depth_m: np.ndarray,
    intrinsics: dict[str, object],
    matrix: np.ndarray,
    reference_pose: list[float],
) -> tuple[list[PointTarget], dict[str, object]]:
    plan, selected, surface_pixels, hover_pixels, _all_surface_pixels = _build_depth_normal_plan(
        detection=detection,
        depth_m=depth_m,
        intrinsics=intrinsics,
        matrix=matrix,
        line_selector=args.line_selector,
        side=args.side,
        line_type=args.line_type,
        plan_points=int(args.point_count),
        hover_m=float(args.hover_mm) / 1000.0,
        dian_jin_depth_m=0.0,
        fen_jin_lateral_m=0.0,
        safe_lift_m=0.0,
        window_px=int(args.normal_window_px),
        stride_px=int(args.normal_stride_px),
        depth_band_m=float(args.normal_depth_band_m),
        min_points=int(args.normal_min_points),
        normal_smooth_iterations=int(args.normal_smooth_iterations),
        tool_contact_axis=args.tool_contact_axis,
        reference_pose=reference_pose,
        orientation_mode=args.orientation_mode,
    )
    targets = [
        PointTarget(
            index=int(frame.index),
            source_index=int(selected["selected_plan_indices"][int(frame.index) - 1]),
            pixel=[float(v) for v in frame.pixel[:2]],
            hover_pixel=[float(v) for v in hover_pixels[int(frame.index) - 1][:2]],
            surface_m=np.asarray(frame.robot_point_m[:3], dtype=np.float64),
            press_m=_normalize(np.asarray(frame.press_direction_m[:3], dtype=np.float64)),
            rpy=[float(v) for v in frame.hover_pose_m[3:6]],
        )
        for frame in plan.frames
    ]
    meta = {
        "selected_side": selected["side"],
        "selected_line_type": selected["line_type"],
        "selected_prefix": selected["selected_prefix"],
        "selected_source_indices": [int(v) for v in selected["selected_plan_indices"]],
        "selected_pixels": [target.pixel for target in targets],
        "selected_surface_m": [[float(v) for v in target.surface_m.tolist()] for target in targets],
        "selected_hover_pixels": [target.hover_pixel for target in targets],
        "selected_press_direction_m": [[float(v) for v in target.press_m.tolist()] for target in targets],
        "normal_source": selected["normal_source"],
        "orientation_source": selected["orientation_source"],
    }
    return targets, meta


def _draw_polyline(image: np.ndarray, pixels: list[list[float]], color: tuple[int, int, int], thickness: int) -> None:
    pts = [
        [int(round(float(p[0]))), int(round(float(p[1])))]
        for p in pixels
        if isinstance(p, (list, tuple)) and len(p) >= 2 and np.all(np.isfinite(np.asarray(p[:2], dtype=np.float64)))
    ]
    if len(pts) >= 2:
        arr = np.asarray(pts, dtype=np.int32)
        cv2.polylines(image, [arr.reshape(-1, 1, 2)], False, color, thickness)
    for idx, p in enumerate(pts, start=1):
        cv2.circle(image, tuple(p), 4, color, -1)
        if idx in (1, len(pts)) or idx % 5 == 0:
            cv2.putText(image, str(idx), (p[0] + 5, p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def _draw_status(image: np.ndarray, lines: list[str]) -> None:
    y = 24
    for line in lines[:8]:
        cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        y += 23


def _save_capture_artifacts(
    *,
    args: argparse.Namespace,
    stamp: str,
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    detection: dict[str, object],
    overlay: np.ndarray,
    intrinsics: dict[str, object],
    targets: list[PointTarget],
    meta: dict[str, object],
    contact_pose: list[float] | None,
) -> Path:
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = f"live_top_outer_point_force_{stamp}"
    overlay_path, detection_json = save_bladder_artifacts(str(out_dir), detection, overlay, prefix=prefix)
    raw_path = out_dir / f"{prefix}_raw.png"
    depth_path = out_dir / f"{prefix}_depth.npy"
    intrinsics_path = out_dir / f"{prefix}_intrinsics.json"
    plan_path = out_dir / f"{prefix}_20pt_force_plan.json"
    cv2.imwrite(str(raw_path), color_bgr)
    np.save(str(depth_path), depth_m)
    intrinsics_path.write_text(json.dumps(intrinsics, ensure_ascii=False, indent=2), encoding="utf-8")
    plan = {
        "timestamp": stamp,
        "source_detection_json": str(detection_json),
        "source_overlay_png": str(overlay_path),
        "source_raw_png": str(raw_path),
        "source_depth_npy": str(depth_path),
        "source_intrinsics_json": str(intrinsics_path),
        "matrix_path": str(Path(args.matrix_path).resolve()),
        "point_count": len(targets),
        "line": meta,
        "contact_pose_source": args.contact_pose_source,
        "contact_pose_m_rpy": contact_pose,
        "orientation_mode": args.orientation_mode,
        "reference_contact_rpy": None if contact_pose is None else [float(v) for v in contact_pose[3:6]],
        "tool_contact_axis": args.tool_contact_axis,
        "target_force_n": float(args.target_force_n),
        "max_force_n": float(args.max_force_n),
        "hover_m": float(args.hover_mm) / 1000.0,
        "retreat_m": float(args.retreat_mm) / 1000.0,
        "max_press_m": float(args.max_press_mm) / 1000.0,
        "points": [
            {
                "index": int(target.index),
                "source_index": int(target.source_index),
                "pixel": [float(v) for v in target.pixel],
                "hover_pixel": [float(v) for v in target.hover_pixel],
                "surface_m": [float(v) for v in target.surface_m.tolist()],
                "press_direction_m": [float(v) for v in target.press_m.tolist()],
                "rpy": [float(v) for v in target.rpy],
                "hover_pose_m_rpy": [
                    *[float(v) for v in _pose(target.surface_m, target.press_m, -float(args.hover_mm) / 1000.0, target.rpy)[:3]],
                    *[float(v) for v in target.rpy],
                ],
            }
            for target in targets
        ],
    }
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    return plan_path


def execute_point_force_sequence(
    *,
    args: argparse.Namespace,
    targets: list[PointTarget],
    status: dict[str, object],
    stop_event: threading.Event,
) -> None:
    if not args.allow_contact:
        raise RuntimeError("refusing contact execution without --allow-contact")
    if float(args.max_force_n) <= float(args.target_force_n):
        raise RuntimeError("--max-force-n must be greater than --target-force-n")

    host = str(args.arm_host)
    hover_m = float(args.hover_mm) / 1000.0
    retreat_m = float(args.retreat_mm) / 1000.0
    max_press_m = float(args.max_press_mm) / 1000.0
    approach_step_m = float(args.approach_step_mm) / 1000.0
    adjust_step_m = float(args.adjust_step_mm) / 1000.0
    deadband_n = float(args.force_deadband_n)
    dwell_s = float(args.dwell_s)

    reader = DockerForceReader()
    try:
        rm_json.recover_if_needed(host)
        for target in targets:
            if stop_event.is_set():
                raise RuntimeError("stop requested")
            status.update({"state": "move_hover", "point": target.index, "force": None})
            hover_pose = _pose(target.surface_m, target.press_m, -hover_m, target.rpy)
            _move_segmented(
                host,
                hover_pose,
                speed=int(args.entry_speed),
                max_step_m=float(args.entry_max_step_m),
                timeout=float(args.move_timeout_s),
                max_orientation_error_rad=float(args.max_orientation_error_rad),
            )
            baseline = _average_force(reader, count=8, timeout=1.0, interval_s=0.03)
            if baseline is None:
                raise RuntimeError("force baseline unavailable")

            status.update({"state": "press", "point": target.index, "force": 0.0})
            offset_m = -hover_m
            reached = False
            while offset_m < max_press_m:
                if stop_event.is_set():
                    raise RuntimeError("stop requested")
                sample = _average_force(reader, count=max(1, int(args.force_filter_count)), timeout=0.8, interval_s=0.02)
                if sample is None:
                    raise RuntimeError("force sample unavailable")
                force_n = _force_value(sample, baseline, args.force_measure)
                status.update({"state": "press", "point": target.index, "force": force_n, "offset_m": offset_m})
                print(
                    f"point {target.index}/{len(targets)} approach offset_m={offset_m:.4f} "
                    f"force={force_n:.2f}N {_fmt_force(sample)}",
                    flush=True,
                )
                if force_n >= float(args.max_force_n):
                    raise RuntimeError(f"point {target.index} exceeded max force during approach: {force_n:.2f}N")
                if force_n >= float(args.target_force_n) - deadband_n:
                    reached = True
                    break
                offset_m = min(max_press_m, offset_m + approach_step_m)
                rm_json.movel(host, _pose(target.surface_m, target.press_m, offset_m, target.rpy), speed=int(args.contact_speed), timeout=float(args.move_timeout_s))
                _check_orientation_lock(host, target.rpy, float(args.max_orientation_error_rad))
            if not reached:
                raise RuntimeError(f"point {target.index} target force not reached before max press")

            deadline = time.time() + max(0.0, dwell_s)
            while time.time() < deadline:
                if stop_event.is_set():
                    raise RuntimeError("stop requested")
                sample = _average_force(reader, count=max(1, int(args.force_filter_count)), timeout=0.8, interval_s=0.02)
                if sample is None:
                    raise RuntimeError("force sample unavailable during dwell")
                force_n = _force_value(sample, baseline, args.force_measure)
                status.update({"state": "dwell", "point": target.index, "force": force_n, "offset_m": offset_m})
                if force_n >= float(args.max_force_n):
                    raise RuntimeError(f"point {target.index} exceeded max force during dwell: {force_n:.2f}N")
                if force_n < float(args.target_force_n) - deadband_n:
                    offset_m = min(max_press_m, offset_m + adjust_step_m)
                    rm_json.movel(host, _pose(target.surface_m, target.press_m, offset_m, target.rpy), speed=int(args.contact_speed), timeout=float(args.move_timeout_s))
                    _check_orientation_lock(host, target.rpy, float(args.max_orientation_error_rad))
                elif force_n > float(args.target_force_n) + deadband_n:
                    offset_m = max(-hover_m, offset_m - adjust_step_m)
                    rm_json.movel(host, _pose(target.surface_m, target.press_m, offset_m, target.rpy), speed=int(args.contact_speed), timeout=float(args.move_timeout_s))
                    _check_orientation_lock(host, target.rpy, float(args.max_orientation_error_rad))
                time.sleep(0.05)

            status.update({"state": "retreat", "point": target.index, "force": None})
            retreat_pose = _pose(target.surface_m, target.press_m, -retreat_m, target.rpy)
            rm_json.movel(host, retreat_pose, speed=int(args.entry_speed), timeout=float(args.move_timeout_s))
            _check_orientation_lock(host, target.rpy, float(args.max_orientation_error_rad))

        status.update({"state": "done", "point": len(targets), "force": None})
    except BaseException as exc:
        status.update({"state": f"error:{type(exc).__name__}", "error": str(exc)})
        print(f"point_force_sequence_aborted: {type(exc).__name__}: {exc}", flush=True)
        try:
            rm_json.stop_motion(host)
        except Exception as stop_exc:
            print(f"stop_motion_failed: {stop_exc}", flush=True)
        raise
    finally:
        reader.close()


def execute_hover_sequence(
    *,
    args: argparse.Namespace,
    targets: list[PointTarget],
    status: dict[str, object],
    stop_event: threading.Event,
) -> None:
    host = str(args.arm_host)
    hover_m = float(args.hover_mm) / 1000.0
    try:
        rm_json.recover_if_needed(host)
        for target in targets:
            if stop_event.is_set():
                raise RuntimeError("stop requested")
            status.update({"state": "hover", "point": target.index, "force": None})
            hover_pose = _pose(target.surface_m, target.press_m, -hover_m, target.rpy)
            print(
                f"hover point {target.index}/{len(targets)} "
                f"surface={[round(float(v), 6) for v in target.surface_m.tolist()]} "
                f"hover={[round(float(v), 6) for v in hover_pose[:3]]} "
                f"press={[round(float(v), 6) for v in target.press_m.tolist()]}",
                flush=True,
            )
            _move_segmented(
                host,
                hover_pose,
                speed=int(args.entry_speed),
                max_step_m=float(args.entry_max_step_m),
                timeout=float(args.move_timeout_s),
                max_orientation_error_rad=float(args.max_orientation_error_rad),
            )
            time.sleep(max(0.0, float(args.hover_dwell_s)))
        status.update({"state": "done", "point": len(targets), "force": None})
    except BaseException as exc:
        status.update({"state": f"error:{type(exc).__name__}", "error": str(exc)})
        print(f"hover_sequence_aborted: {type(exc).__name__}: {exc}", flush=True)
        try:
            rm_json.stop_motion(host)
        except Exception as stop_exc:
            print(f"stop_motion_failed: {stop_exc}", flush=True)
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live top-outer bladder detection, save with s, run hover/force trajectory with r.")
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--matrix-path", default="camera_to_robot.json")
    parser.add_argument("--output-dir", default="rm_demo_output")
    parser.add_argument("--model-path", default="yolo11l-pose.pt")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    parser.add_argument("--line-selector", choices=("semantic", "top_outer", "bottom_outer"), default="top_outer")
    parser.add_argument("--point-count", type=int, default=20)
    parser.add_argument("--finger-width", type=float, default=45.0)
    parser.add_argument("--sample-points", type=int, default=40)
    parser.add_argument("--detect-every-s", type=float, default=0.8)
    parser.add_argument("--contact-pose-source", choices=("current", "json"), default="json")
    parser.add_argument("--contact-pose-json", default=str(DEFAULT_CONTACT_POSE))
    parser.add_argument("--tool-contact-axis", choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"), default="neg_z")
    parser.add_argument(
        "--orientation-mode",
        choices=("fixed_reference", "depth_normal"),
        default="fixed_reference",
        help="fixed_reference keeps the saved massage-head RPY; depth_normal rotates it to the fitted back normal",
    )
    parser.add_argument("--normal-window-px", type=int, default=31)
    parser.add_argument("--normal-stride-px", type=int, default=2)
    parser.add_argument("--normal-depth-band-m", type=float, default=0.08)
    parser.add_argument("--normal-min-points", type=int, default=40)
    parser.add_argument("--normal-smooth-iterations", type=int, default=1)
    parser.add_argument("--force-measure", choices=("norm", "fx_abs", "fy_abs", "fz_abs"), default="norm")
    parser.add_argument("--target-force-n", type=float, default=2.0)
    parser.add_argument("--max-force-n", type=float, default=5.0)
    parser.add_argument("--force-deadband-n", type=float, default=0.4)
    parser.add_argument("--force-filter-count", type=int, default=3)
    parser.add_argument("--hover-mm", type=float, default=120.0)
    parser.add_argument("--retreat-mm", type=float, default=60.0)
    parser.add_argument("--max-press-mm", type=float, default=8.0)
    parser.add_argument("--approach-step-mm", type=float, default=1.0)
    parser.add_argument("--adjust-step-mm", type=float, default=0.5)
    parser.add_argument("--dwell-s", type=float, default=0.4)
    parser.add_argument("--hover-dwell-s", type=float, default=0.15)
    parser.add_argument("--entry-speed", type=int, default=2)
    parser.add_argument("--contact-speed", type=int, default=1)
    parser.add_argument("--entry-max-step-m", type=float, default=0.02)
    parser.add_argument("--move-timeout-s", type=float, default=20.0)
    parser.add_argument("--max-orientation-error-rad", type=float, default=0.20)
    parser.add_argument("--max-entry-distance-m", type=float, default=0.35)
    parser.add_argument("--max-entry-orientation-delta-rad", type=float, default=0.9)
    parser.add_argument(
        "--max-entry-z-lift-m",
        type=float,
        default=0.05,
        help="refuse entry motion if the first hover point raises TCP by more than this in base Z",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--realsense-serial", default="")
    parser.add_argument("--window-name", default="Live top outer point force")
    parser.add_argument("--run", action="store_true", help="allow pressing r to execute the saved trajectory")
    parser.add_argument(
        "--run-mode",
        choices=("hover", "force"),
        default="hover",
        help="hover verifies the 20-point suspended path; force performs point-by-point pressing",
    )
    parser.add_argument("--allow-contact", action="store_true", help="required together with --run for contact execution")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    matrix = load_transform_matrix(str(Path(args.matrix_path).resolve()))
    if matrix is None:
        raise RuntimeError(f"camera->robot matrix not found: {args.matrix_path}")

    pipeline, profile, color_format = _stream_profile(args)
    align = rs.align(rs.stream.color)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = float(depth_sensor.get_depth_scale())
    intrinsics = _intrinsics_from_profile(profile, depth_scale)
    print(f"realsense_started color_format={color_format} depth_scale={depth_scale}", flush=True)

    latest_detection: dict[str, object] | None = None
    latest_overlay: np.ndarray | None = None
    latest_error = ""
    last_detect_t = 0.0
    saved_pixels: list[list[float]] = []
    saved_hover_pixels: list[list[float]] = []
    saved_targets: list[PointTarget] = []
    saved_plan_path: Path | None = None
    motion_thread: threading.Thread | None = None
    stop_event = threading.Event()
    motion_status: dict[str, object] = {"state": "idle", "point": 0, "force": None}

    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
    try:
        for _ in range(max(1, int(args.warmup_frames))):
            _read_aligned_frame(pipeline, align, color_format, depth_scale)

        while True:
            frame_pair = _read_aligned_frame(pipeline, align, color_format, depth_scale)
            if frame_pair is None:
                continue
            color_bgr, depth_m = frame_pair
            now = time.time()
            display = color_bgr.copy()

            if motion_thread is None and now - last_detect_t >= float(args.detect_every_s):
                last_detect_t = now
                try:
                    detection, overlay = detect_bladder_lines(
                        color_bgr=color_bgr,
                        depth_m=depth_m,
                        intrinsics_data=intrinsics,
                        finger_width_mm=float(args.finger_width),
                        model_path=args.model_path,
                        sample_points=int(args.sample_points),
                    )
                    selected_side, selected_type = select_bladder_line(detection, args.line_selector, args.side, args.line_type)
                    prefix = f"{selected_side}_{selected_type}"
                    latest_detection = detection
                    latest_overlay = overlay
                    latest_error = ""
                    display = overlay.copy()
                    _draw_polyline(display, list(detection.get(f"{prefix}_pixel", [])), (255, 0, 255), 3)
                except Exception as exc:
                    latest_error = f"{type(exc).__name__}: {exc}"
                    if latest_overlay is not None:
                        display = latest_overlay.copy()
            elif latest_overlay is not None and motion_thread is None:
                display = latest_overlay.copy()

            if saved_pixels:
                _draw_polyline(display, saved_pixels, (255, 0, 255), 3)
            if saved_hover_pixels:
                _draw_polyline(display, saved_hover_pixels, (255, 255, 0), 2)

            state = str(motion_status.get("state", "idle"))
            force = motion_status.get("force")
            force_text = "n/a" if force is None else f"{float(force):.2f}N"
            status_lines = [
                "s: save top_outer 20-point trajectory",
                "r: run saved trajectory (--run required; --run-mode force also needs --allow-contact)",
                "q/ESC: stop and quit",
                f"motion={state} point={motion_status.get('point', 0)} force={force_text}",
                f"mode={args.run_mode} target={float(args.target_force_n):.1f}N max={float(args.max_force_n):.1f}N axis={args.tool_contact_axis}",
            ]
            if saved_plan_path is not None:
                status_lines.append(f"saved={saved_plan_path.name}")
            if latest_error:
                status_lines.append(f"detect_error={latest_error[:90]}")
            _draw_status(display, status_lines)
            cv2.imshow(args.window_name, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                stop_event.set()
                try:
                    rm_json.stop_motion(args.arm_host)
                except Exception:
                    pass
                break

            if key == ord("s"):
                if latest_detection is None:
                    print("no detection available to save", flush=True)
                    continue
                stamp = time.strftime("%Y%m%d_%H%M%S")
                rpy, contact_pose = _load_contact_rpy(args)
                reference_pose = contact_pose if contact_pose is not None else [0.0, 0.0, 0.0, *rpy]
                targets, meta = _build_top_outer_targets(
                    args=args,
                    detection=latest_detection,
                    depth_m=depth_m,
                    intrinsics=intrinsics,
                    matrix=np.asarray(matrix, dtype=np.float64),
                    reference_pose=reference_pose,
                )
                saved_pixels = [target.pixel for target in targets]
                saved_hover_pixels = [target.hover_pixel for target in targets]
                saved_targets = targets
                saved_plan_path = _save_capture_artifacts(
                    args=args,
                    stamp=stamp,
                    color_bgr=color_bgr,
                    depth_m=depth_m,
                    detection=latest_detection,
                    overlay=display,
                    intrinsics=intrinsics,
                    targets=targets,
                    meta=meta,
                    contact_pose=contact_pose,
                )
                print(f"saved 20-point force plan: {saved_plan_path}", flush=True)
                print(
                    f"first_surface={[round(float(v), 6) for v in targets[0].surface_m.tolist()]} "
                    f"first_hover={[round(float(v), 6) for v in _pose(targets[0].surface_m, targets[0].press_m, -float(args.hover_mm) / 1000.0, targets[0].rpy)[:3]]} "
                    f"first_press={[round(float(v), 6) for v in targets[0].press_m.tolist()]}",
                    flush=True,
                )

            if key == ord("r"):
                if not args.run:
                    print("refusing motion: start with --run, then press s and r", flush=True)
                    continue
                if args.run_mode == "force" and not args.allow_contact:
                    print("refusing force motion: start with --run --run-mode force --allow-contact", flush=True)
                    continue
                if not saved_targets:
                    print("no saved trajectory: press s first", flush=True)
                    continue
                if motion_thread is not None and motion_thread.is_alive():
                    print("motion is already running", flush=True)
                    continue
                try:
                    _joints, current_pose, arm_err, sys_err, ik_err = rm_json.get_current_arm_state(args.arm_host)
                    first_hover = _pose(
                        saved_targets[0].surface_m,
                        saved_targets[0].press_m,
                        -float(args.hover_mm) / 1000.0,
                        saved_targets[0].rpy,
                    )
                    entry_dist = float(
                        np.linalg.norm(np.asarray(first_hover[:3], dtype=np.float64) - np.asarray(current_pose[:3], dtype=np.float64))
                    )
                    entry_z_lift = float(first_hover[2]) - float(current_pose[2])
                    entry_rot = _rotation_error_rad(current_pose[3:6], saved_targets[0].rpy)
                    print(
                        f"entry_check current_pose={[round(float(v), 6) for v in current_pose]} "
                        f"first_hover={[round(float(v), 6) for v in first_hover[:3]]} "
                        f"entry_dist_m={entry_dist:.4f} entry_z_lift_m={entry_z_lift:.4f} "
                        f"entry_rot_rad={entry_rot:.4f} "
                        f"arm_err={arm_err} sys_err={sys_err} ik_err={ik_err}",
                        flush=True,
                    )
                    if arm_err != 0 or sys_err != 0:
                        print("refusing motion: arm state is not clean", flush=True)
                        continue
                    if entry_dist > float(args.max_entry_distance_m):
                        print("refusing motion: first hover is too far from current TCP", flush=True)
                        continue
                    if entry_z_lift > float(args.max_entry_z_lift_m):
                        print("refusing motion: first hover would lift TCP too much in base Z", flush=True)
                        continue
                    if entry_rot > float(args.max_entry_orientation_delta_rad):
                        print("refusing motion: current orientation is too far from saved contact orientation", flush=True)
                        continue
                except Exception as exc:
                    print(f"refusing motion: entry check failed: {type(exc).__name__}: {exc}", flush=True)
                    continue
                stop_event.clear()
                motion_status.update({"state": "starting", "point": 0, "force": None})
                print(f"starting saved trajectory: {saved_plan_path}", flush=True)
                target_func = execute_point_force_sequence if args.run_mode == "force" else execute_hover_sequence
                motion_thread = threading.Thread(
                    target=target_func,
                    kwargs={
                        "args": args,
                        "targets": saved_targets,
                        "status": motion_status,
                        "stop_event": stop_event,
                    },
                    daemon=True,
                )
                motion_thread.start()

            if motion_thread is not None and not motion_thread.is_alive():
                motion_thread.join(timeout=0.1)
                motion_thread = None
    finally:
        stop_event.set()
        if motion_thread is not None and motion_thread.is_alive():
            try:
                rm_json.stop_motion(args.arm_host)
            except Exception:
                pass
            motion_thread.join(timeout=2.0)
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
