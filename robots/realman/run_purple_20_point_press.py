#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import select as select_mod
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from build_bladder_depth_normal_diagnostic import (
    fit_local_normal_camera,
    normalize,
    project_camera,
    select_line,
    transform_direction,
)
from rm_demo import rm_json
from rm_demo.rm_bladder import detect_bladder_lines, save_bladder_artifacts
from rm_demo.rm_transform import load_transform_matrix, transform_points
from run_external_bladder_live_hover import _intrinsics_from_profile, _read_aligned_frame, _stream_profile


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_CONTACT_POSE = PROJECT_DIR / "rm_demo_output" / "user_confirmed_side_lying_contact_pose.json"


@dataclass
class PressPoint:
    index: int
    source_index: int
    pixel: list[float]
    hover_pixel: list[float]
    surface_m: np.ndarray
    press_m: np.ndarray
    rpy: list[float]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def matrix_to_rpy(rot: np.ndarray) -> list[float]:
    r = np.asarray(rot, dtype=np.float64)
    pitch = math.asin(max(-1.0, min(1.0, -float(r[2, 0]))))
    cp = math.cos(pitch)
    if abs(cp) > 1e-6:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(r[0, 1]), float(r[1, 1]))
    return [float(roll), float(pitch), float(yaw)]


def rpy_from_tool_z(tool_z_base: np.ndarray, tangent_base: np.ndarray) -> list[float]:
    z_axis = normalize(tool_z_base)
    tangent = np.asarray(tangent_base, dtype=np.float64)
    x_axis = tangent - z_axis * float(np.dot(tangent, z_axis))
    if float(np.linalg.norm(x_axis)) < 1e-6:
        for candidate in (
            np.asarray([0.0, 0.0, 1.0], dtype=np.float64),
            np.asarray([0.0, 1.0, 0.0], dtype=np.float64),
            np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
        ):
            x_axis = candidate - z_axis * float(np.dot(candidate, z_axis))
            if float(np.linalg.norm(x_axis)) >= 1e-6:
                break
    x_axis = normalize(x_axis)
    y_axis = normalize(np.cross(z_axis, x_axis))
    x_axis = normalize(np.cross(y_axis, z_axis))
    return matrix_to_rpy(np.column_stack([x_axis, y_axis, z_axis]))


def rotation_error_rad(actual_rpy: list[float], target_rpy: list[float]) -> float:
    actual = rpy_to_matrix(*actual_rpy[:3])
    target = rpy_to_matrix(*target_rpy[:3])
    delta = target.T @ actual
    cos_angle = (float(np.trace(delta)) - 1.0) * 0.5
    return float(math.acos(max(-1.0, min(1.0, cos_angle))))


def rpy_error_norm(start_rpy: list[float], target_rpy: list[float]) -> float:
    return max(abs((float(target_rpy[i]) - float(start_rpy[i]) + math.pi) % (2.0 * math.pi) - math.pi) for i in range(3))


def lerp_angle(start: float, target: float, ratio: float) -> float:
    delta = (float(target) - float(start) + math.pi) % (2.0 * math.pi) - math.pi
    return float(start) + delta * float(ratio)


def pose_at(point: PressPoint, offset_m: float) -> list[float]:
    xyz = point.surface_m + point.press_m * float(offset_m)
    return [float(xyz[0]), float(xyz[1]), float(xyz[2]), *[float(v) for v in point.rpy]]


def tcp_to_tip_offset_tool(args: argparse.Namespace) -> np.ndarray:
    return np.asarray(
        [
            float(args.tcp_to_tip_x_mm) / 1000.0,
            float(args.tcp_to_tip_y_mm) / 1000.0,
            float(args.tcp_to_tip_z_mm) / 1000.0,
        ],
        dtype=np.float64,
    )


def signed_tcp_to_tip_offset_tool(args: argparse.Namespace) -> np.ndarray:
    offset = tcp_to_tip_offset_tool(args)
    if args.tip_command_sign == "normal":
        return offset
    if args.tip_command_sign == "inverted":
        return -offset
    raise RuntimeError(f"unsupported tip_command_sign: {args.tip_command_sign}")


def tcp_to_tip_offset_base(args: argparse.Namespace) -> np.ndarray:
    return np.asarray(
        [
            float(args.tcp_to_tip_base_x_mm) / 1000.0,
            float(args.tcp_to_tip_base_y_mm) / 1000.0,
            float(args.tcp_to_tip_base_z_mm) / 1000.0,
        ],
        dtype=np.float64,
    )


def command_pose_at(point: PressPoint, offset_m: float, args: argparse.Namespace) -> list[float]:
    desired_tip = np.asarray(pose_at(point, offset_m)[:3], dtype=np.float64)
    if args.tip_offset_mode == "base":
        tcp_xyz = desired_tip - tcp_to_tip_offset_base(args)
    elif args.tip_offset_mode == "tool":
        tip_offset_base = rpy_to_matrix(*point.rpy) @ signed_tcp_to_tip_offset_tool(args)
        tcp_xyz = desired_tip - tip_offset_base
    else:
        raise RuntimeError(f"unsupported tip_offset_mode: {args.tip_offset_mode}")
    return [float(tcp_xyz[0]), float(tcp_xyz[1]), float(tcp_xyz[2]), *[float(v) for v in point.rpy]]


def predicted_tip_from_tcp_pose(tcp_pose: list[float], args: argparse.Namespace) -> np.ndarray:
    tcp = np.asarray(tcp_pose[:3], dtype=np.float64)
    if args.tip_offset_mode == "base":
        return tcp + tcp_to_tip_offset_base(args)
    rot = rpy_to_matrix(*tcp_pose[3:6])
    return tcp + rot @ signed_tcp_to_tip_offset_tool(args)


def base_to_pixel(point_base: np.ndarray, camera_from_base: np.ndarray, intrinsics: dict[str, object]) -> list[float]:
    p = camera_from_base @ np.asarray([float(point_base[0]), float(point_base[1]), float(point_base[2]), 1.0])
    uv = project_camera(p[:3], intrinsics)
    return [float("nan"), float("nan")] if uv is None else [float(v) for v in uv]


def load_contact_rpy(path: str) -> list[float]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    pose = data.get("tcp_pose_m_rpy")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"contact pose JSON missing tcp_pose_m_rpy: {path}")
    return [float(v) for v in pose[3:6]]


def even_indices(length: int, count: int) -> list[int]:
    if length <= 0:
        raise RuntimeError("empty purple line")
    if count <= 1:
        return [length // 2]
    return [int(round(i * (length - 1) / float(count - 1))) for i in range(count)]


def is_valid_camera_point(point: object) -> bool:
    if not isinstance(point, (list, tuple)) or len(point) < 3:
        return False
    arr = np.asarray(point[:3], dtype=np.float64)
    return bool(np.all(np.isfinite(arr)) and float(arr[2]) > 0.1)


def build_press_points(
    *,
    args: argparse.Namespace,
    detection: dict[str, object],
    depth_m: np.ndarray,
    intrinsics: dict[str, object],
    matrix: np.ndarray,
) -> tuple[list[PressPoint], dict[str, object]]:
    selected_side, selected_type = select_line(detection, args.line_selector, args.side, args.line_type)
    prefix = f"{selected_side}_{selected_type}"
    pixels_all = list(detection[f"{prefix}_pixel"])
    camera_all = list(detection[f"{prefix}_camera"])
    valid_rows = [
        (idx, [float(v) for v in pixels_all[idx][:2]], [float(v) for v in camera_all[idx][:3]])
        for idx in range(min(len(pixels_all), len(camera_all)))
        if is_valid_camera_point(camera_all[idx])
    ]
    if len(valid_rows) < 2:
        raise RuntimeError(f"{prefix} has insufficient valid RGBD points")

    selected_rows = [valid_rows[idx] for idx in even_indices(len(valid_rows), int(args.point_count))]
    selected_camera = [row[2] for row in selected_rows]
    selected_base = np.asarray(transform_points(selected_camera, matrix), dtype=np.float64)
    camera_from_base = np.linalg.inv(np.asarray(matrix, dtype=np.float64))
    fixed_rpy = load_contact_rpy(args.contact_pose_json)

    points: list[PressPoint] = []
    prev_front: np.ndarray | None = None
    for out_idx, row in enumerate(selected_rows, start=1):
        source_idx, pixel, _camera_point = row
        normal_camera, _centroid, count, rmse = fit_local_normal_camera(
            depth_m=depth_m,
            intrinsics=intrinsics,
            u=float(pixel[0]),
            v=float(pixel[1]),
            window_px=int(args.normal_window_px),
            stride_px=int(args.normal_stride_px),
            depth_band_m=float(args.normal_depth_band_m),
            min_points=int(args.normal_min_points),
        )
        front_normal = transform_direction(matrix, normal_camera)
        if prev_front is not None and float(np.dot(front_normal, prev_front)) < 0.0:
            front_normal = -front_normal
        prev_front = front_normal
        press = -front_normal if args.press_side == "into_body" else front_normal
        press = normalize(press)
        surface = np.asarray(selected_base[out_idx - 1], dtype=np.float64)
        hover = surface - press * (float(args.hover_mm) / 1000.0)
        if args.orientation_mode == "normal":
            if len(selected_base) == 1:
                tangent = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
            elif out_idx == 1:
                tangent = np.asarray(selected_base[1] - selected_base[0], dtype=np.float64)
            elif out_idx == len(selected_base):
                tangent = np.asarray(selected_base[-1] - selected_base[-2], dtype=np.float64)
            else:
                tangent = np.asarray(selected_base[out_idx] - selected_base[out_idx - 2], dtype=np.float64)
            normal_target = front_normal if args.normal_direction == "outward" else press
            tool_z_target = normal_target if args.normal_tool_axis == "pos_z" else -normal_target
            rpy = rpy_from_tool_z(tool_z_target, tangent)
        else:
            rpy = list(fixed_rpy)
        points.append(
            PressPoint(
                index=out_idx,
                source_index=int(source_idx),
                pixel=[float(v) for v in pixel],
                hover_pixel=base_to_pixel(hover, camera_from_base, intrinsics),
                surface_m=surface,
                press_m=press,
                rpy=list(rpy),
            )
        )
        print(
            f"plan point {out_idx:02d} src={source_idx} pixel={pixel} "
            f"surface={[round(float(v), 6) for v in surface.tolist()]} "
                f"hover={[round(float(v), 6) for v in hover.tolist()]} "
                f"press={[round(float(v), 6) for v in press.tolist()]} "
                f"rpy={[round(float(v), 4) for v in rpy]} "
                f"normal_count={count} rmse_mm={rmse * 1000.0:.2f}",
                flush=True,
            )

    meta = {
        "selected_prefix": prefix,
        "selected_side": selected_side,
        "selected_line_type": selected_type,
        "press_side": args.press_side,
        "hover_mm": float(args.hover_mm),
        "point_count": len(points),
        "orientation_mode": str(args.orientation_mode),
        "normal_tool_axis": str(args.normal_tool_axis),
        "normal_direction": str(args.normal_direction),
    }
    return points, meta


def draw_polyline(image: np.ndarray, pixels: list[list[float]], color: tuple[int, int, int], thickness: int) -> None:
    pts = []
    for pixel in pixels:
        arr = np.asarray(pixel[:2], dtype=np.float64)
        if np.all(np.isfinite(arr)):
            pts.append([int(round(float(arr[0]))), int(round(float(arr[1])))])
    if len(pts) >= 2:
        cv2.polylines(image, [np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)], False, color, thickness)
    for idx, pt in enumerate(pts, start=1):
        cv2.circle(image, tuple(pt), 4, color, -1)
        if idx in (1, len(pts)) or idx % 5 == 0:
            cv2.putText(image, str(idx), (pt[0] + 5, pt[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def draw_marker(image: np.ndarray, pixel: list[float] | None, label: str, color: tuple[int, int, int], style: str) -> None:
    if pixel is None:
        return
    arr = np.asarray(pixel[:2], dtype=np.float64)
    if not np.all(np.isfinite(arr)):
        return
    x, y = int(round(float(arr[0]))), int(round(float(arr[1])))
    if x < -50 or y < -50 or x > image.shape[1] + 50 or y > image.shape[0] + 50:
        return
    black = (0, 0, 0)
    if style == "cross":
        cv2.drawMarker(image, (x, y), black, markerType=cv2.MARKER_TILTED_CROSS, markerSize=30, thickness=6)
        cv2.drawMarker(image, (x, y), color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=30, thickness=3)
    elif style == "box":
        cv2.rectangle(image, (x - 13, y - 13), (x + 13, y + 13), black, 6)
        cv2.rectangle(image, (x - 13, y - 13), (x + 13, y + 13), color, 3)
        cv2.circle(image, (x, y), 3, color, -1)
    else:
        cv2.circle(image, (x, y), 10, black, 5)
        cv2.circle(image, (x, y), 10, color, 3)
    cv2.putText(image, label, (x + 11, y - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.75, black, 5, cv2.LINE_AA)
    cv2.putText(image, label, (x + 11, y - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)


def draw_status(image: np.ndarray, lines: list[str]) -> None:
    y = 24
    for line in lines[:8]:
        cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        y += 23


def save_plan(
    *,
    args: argparse.Namespace,
    stamp: str,
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: dict[str, object],
    detection: dict[str, object],
    overlay: np.ndarray,
    points: list[PressPoint],
    meta: dict[str, object],
) -> Path:
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = f"purple_20_point_press_{stamp}"
    overlay_path, detection_path = save_bladder_artifacts(str(out_dir), detection, overlay, prefix=prefix)
    raw_path = out_dir / f"{prefix}_raw.png"
    depth_path = out_dir / f"{prefix}_depth.npy"
    intrinsics_path = out_dir / f"{prefix}_intrinsics.json"
    plan_path = out_dir / f"{prefix}_plan.json"
    cv2.imwrite(str(raw_path), color_bgr)
    np.save(str(depth_path), depth_m)
    intrinsics_path.write_text(json.dumps(intrinsics, ensure_ascii=False, indent=2), encoding="utf-8")
    plan = {
        "timestamp": stamp,
        "source_detection_json": str(detection_path),
        "source_overlay_png": str(overlay_path),
        "source_raw_png": str(raw_path),
        "source_depth_npy": str(depth_path),
        "source_intrinsics_json": str(intrinsics_path),
        "matrix_path": str(Path(args.matrix_path).resolve()),
        "contact_pose_json": str(Path(args.contact_pose_json).resolve()),
        "target_force_n": float(args.target_force_n),
        "max_force_n": float(args.max_force_n),
        "meta": meta,
        "points": [
            {
                "index": int(point.index),
                "source_index": int(point.source_index),
                "pixel": [float(v) for v in point.pixel],
                "hover_pixel": [float(v) for v in point.hover_pixel],
                "surface_m": [float(v) for v in point.surface_m.tolist()],
                "hover_m": [float(v) for v in pose_at(point, -float(args.hover_mm) / 1000.0)[:3]],
                "command_tcp_hover_m": [
                    float(v) for v in command_pose_at(point, -float(args.hover_mm) / 1000.0, args)[:3]
                ],
                "press_direction_m": [float(v) for v in point.press_m.tolist()],
                "rpy": [float(v) for v in point.rpy],
            }
            for point in points
        ],
    }
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    return plan_path


class DockerForceReader:
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
        print(json.dumps(bridge.request_force_sample(timeout=1.0), ensure_ascii=False), flush=True)
    elif cmd == "quit":
        print("BYE", flush=True)
        break
'''
        inner = (
            "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
            "export PYTHONPATH=/home/franka/massage/robots/realman/ros_vendor/python:$PYTHONPATH; "
            "cd /home/franka/massage/robots/realman; "
            f"python3 -u -c {shlex.quote(code)}"
        )
        self.proc = subprocess.Popen(
            [
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
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        ready = self._readline(8.0)
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
                    stderr = "" if self.proc.stderr is None else self.proc.stderr.read()
                    raise RuntimeError(f"force reader exited: {stderr}")
                return line.strip()
            if self.proc.poll() is not None:
                stderr = "" if self.proc.stderr is None else self.proc.stderr.read()
                raise RuntimeError(f"force reader exited with code {self.proc.returncode}: {stderr}")
        raise TimeoutError("force reader timed out")

    def sample(self, timeout: float = 1.2) -> dict[str, float] | None:
        if self.proc.stdin is None:
            raise RuntimeError("force reader stdin is closed")
        self.proc.stdin.write("sample\n")
        self.proc.stdin.flush()
        line = self._readline(timeout + 1.0)
        if not line or line in ("null", "None"):
            return None
        data = json.loads(line)
        if not isinstance(data, dict):
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


class JsonForceReader:
    def __init__(self, host: str, scale: float = 0.001) -> None:
        self.host = str(host)
        self.scale = float(scale)

    def sample(self, timeout: float = 1.2) -> dict[str, float] | None:
        try:
            data = rm_json.query_json(self.host, {"command": "get_force_data"}, timeout=float(timeout))
        except Exception:
            return None
        values = data.get("force_data")
        if not isinstance(values, list) or len(values) < 6:
            return None
        scaled = [float(v) * self.scale for v in values[:6]]
        return {
            "fx": scaled[0],
            "fy": scaled[1],
            "fz": scaled[2],
            "mx": scaled[3],
            "my": scaled[4],
            "mz": scaled[5],
        }

    def close(self) -> None:
        return


ForceReader = DockerForceReader | JsonForceReader


def average_force(reader: ForceReader, count: int, interval_s: float = 0.02) -> dict[str, float] | None:
    samples = []
    for _ in range(max(1, int(count))):
        sample = reader.sample()
        if sample is not None:
            samples.append(sample)
        time.sleep(interval_s)
    if not samples:
        return None
    return {key: float(sum(sample[key] for sample in samples) / len(samples)) for key in ("fx", "fy", "fz", "mx", "my", "mz")}


def force_delta_norm(sample: dict[str, float], baseline: dict[str, float]) -> float:
    delta = np.asarray([sample["fx"] - baseline["fx"], sample["fy"] - baseline["fy"], sample["fz"] - baseline["fz"]], dtype=np.float64)
    return float(np.linalg.norm(delta))


def change_tool_in_docker(tool_name: str) -> None:
    name = str(tool_name).strip()
    if not name:
        return
    subprocess.run(["docker", "start", "noetic"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    code = f'''
from rm_demo.rm_ros import RosArmBridge
arm = RosArmBridge()
print(arm.change_tool({name!r}), flush=True)
'''
    inner = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        "export PYTHONPATH=/home/franka/massage/robots/realman/ros_vendor/python:$PYTHONPATH; "
        "cd /home/franka/massage/robots/realman; "
        f"python3 -u -c {shlex.quote(code)}"
    )
    subprocess.run(
        [
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
        ],
        check=True,
    )


def guarded_movel(host: str, target_pose: list[float], speed: int, timeout: float, args: argparse.Namespace) -> None:
    guarded_pose_motion(host, target_pose, speed, timeout, args, motion="movel")


def guarded_movej_p(host: str, target_pose: list[float], speed: int, timeout: float, args: argparse.Namespace) -> None:
    guarded_pose_motion(host, target_pose, speed, timeout, args, motion="movej_p")


def guarded_pose_motion(
    host: str,
    target_pose: list[float],
    speed: int,
    timeout: float,
    args: argparse.Namespace,
    *,
    motion: str,
) -> None:
    target = [float(v) for v in target_pose[:6]]
    hard_max_z = float(args.hard_max_z_m)
    max_above_target = float(args.max_z_above_target_m)

    _joints, current_pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(host)
    if arm_err != 0 or sys_err != 0:
        raise RuntimeError(f"arm state is not clean before movel: arm_err={arm_err} sys_err={sys_err}")
    current_z = float(current_pose[2])
    target_z = float(target[2])
    high_descend = bool(args.allow_high_start_descend) and current_z > hard_max_z and target_z <= current_z + 0.002
    if target_z > hard_max_z and not high_descend:
        raise RuntimeError(f"refusing target above hard max z: target_z={target_z:.4f} hard_max_z={hard_max_z:.4f}")
    if current_z > hard_max_z and not high_descend:
        raise RuntimeError(f"refusing motion while current z is above hard max: current_z={current_pose[2]:.4f}")

    result: dict[str, object] = {}
    done = threading.Event()

    def _run() -> None:
        try:
            if motion == "movej_p":
                result["reply"] = rm_json.movej_p(host, target, speed=int(speed), timeout=float(timeout))
            else:
                result["reply"] = rm_json.movel(host, target, speed=int(speed), timeout=float(timeout))
        except BaseException as exc:
            result["error"] = exc
        finally:
            done.set()

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    deadline = time.time() + float(timeout) + 2.0
    while not done.is_set() and time.time() < deadline:
        try:
            _j, pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(host)
            z = float(pose[2])
            if arm_err != 0 or sys_err != 0:
                rm_json.stop_motion(host)
                raise RuntimeError(f"arm error during movel: arm_err={arm_err} sys_err={sys_err}")
            if high_descend and z > current_z + float(args.max_z_above_start_m):
                rm_json.stop_motion(host)
                raise RuntimeError(
                    f"z rose during high-start descent: current_z={z:.4f} "
                    f"start_z={current_z:.4f} limit={float(args.max_z_above_start_m):.4f}"
                )
            if z > hard_max_z and not high_descend:
                rm_json.stop_motion(host)
                raise RuntimeError(f"hard z limit exceeded during movel: current_z={z:.4f} hard_max_z={hard_max_z:.4f}")
            if (not high_descend or z <= hard_max_z) and z > target_z + max_above_target:
                rm_json.stop_motion(host)
                raise RuntimeError(
                    f"z rose too far above target during movel: current_z={z:.4f} "
                    f"target_z={target_z:.4f} limit={max_above_target:.4f}"
                )
        except RuntimeError:
            thread.join(timeout=1.0)
            raise
        except Exception:
            pass
        time.sleep(max(0.02, float(args.watchdog_period_s)))
    thread.join(timeout=1.0)
    if "error" in result:
        raise result["error"]  # type: ignore[misc]
    if not done.is_set():
        rm_json.stop_motion(host)
        raise RuntimeError(f"{motion} watchdog timed out")


def move_segmented(host: str, target_pose: list[float], speed: int, max_step_m: float, timeout: float, args: argparse.Namespace) -> None:
    _joints, start_pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(host)
    if arm_err != 0 or sys_err != 0:
        raise RuntimeError(f"arm state is not clean: arm_err={arm_err} sys_err={sys_err}")
    target = [float(v) for v in target_pose[:6]]
    dist = float(np.linalg.norm(np.asarray(target[:3]) - np.asarray(start_pose[:3])))
    angle = rpy_error_norm([float(v) for v in start_pose[3:6]], [float(v) for v in target[3:6]])
    steps = max(
        1,
        int(math.ceil(dist / max(1e-6, float(max_step_m)))),
        int(math.ceil(angle / max(1e-6, float(args.entry_max_angle_step_rad)))),
    )
    for step_idx in range(1, steps + 1):
        ratio = float(step_idx) / float(steps)
        pose = [0.0] * 6
        for axis in range(3):
            pose[axis] = float(start_pose[axis]) * (1.0 - ratio) + float(target[axis]) * ratio
        for axis in range(3, 6):
            pose[axis] = lerp_angle(float(start_pose[axis]), float(target[axis]), ratio)
        guarded_movel(host, pose, int(speed), float(timeout), args)


def move_pose_direct(
    host: str,
    target_pose: list[float],
    speed: int,
    timeout: float,
    args: argparse.Namespace,
    *,
    motion: str,
) -> None:
    if motion == "movej_p":
        print(f"direct movej_p cmd_tcp={[round(float(v), 6) for v in target_pose[:3]]}", flush=True)
        guarded_movej_p(host, target_pose, int(speed), float(timeout), args)
    elif motion == "movel":
        print(f"direct movel cmd_tcp={[round(float(v), 6) for v in target_pose[:3]]}", flush=True)
        move_segmented(host, target_pose, int(speed), float(args.entry_max_step_m), float(timeout), args)
    else:
        raise RuntimeError(f"unsupported pose motion: {motion}")


def move_to_hover(host: str, target_pose: list[float], speed: int, max_step_m: float, timeout: float, args: argparse.Namespace) -> None:
    if args.entry_orientation_policy == "blend":
        move_segmented(host, target_pose, speed, max_step_m, timeout, args)
        return
    _joints, current_pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(host)
    if arm_err != 0 or sys_err != 0:
        raise RuntimeError(f"arm state is not clean before position-first move: arm_err={arm_err} sys_err={sys_err}")
    position_first = [float(v) for v in target_pose[:3]] + [float(v) for v in current_pose[3:6]]
    print(
        f"position_first_hover cmd_tcp={[round(float(v), 6) for v in position_first[:3]]} "
        f"keep_rpy={[round(float(v), 4) for v in position_first[3:6]]}",
        flush=True,
    )
    if args.entry_position_motion == "movej_p":
        move_pose_direct(host, position_first, speed, timeout, args, motion="movej_p")
    else:
        move_segmented(host, position_first, speed, max_step_m, timeout, args)
    print(
        f"orient_at_hover target_rpy={[round(float(v), 4) for v in target_pose[3:6]]}",
        flush=True,
    )
    if args.entry_orient_motion == "movej_p":
        move_pose_direct(host, target_pose, speed, timeout, args, motion="movej_p")
    else:
        move_segmented(host, target_pose, speed, max_step_m, timeout, args)


def order_points(points: list[PressPoint], current_xyz: list[float], hover_m: float, mode: str, args: argparse.Namespace) -> list[PressPoint]:
    if mode == "first_to_last":
        return list(points)
    if mode == "last_to_first":
        return list(reversed(points))
    current = np.asarray(current_xyz[:3], dtype=np.float64)
    first_hover = np.asarray(command_pose_at(points[0], -hover_m, args)[:3], dtype=np.float64)
    last_hover = np.asarray(command_pose_at(points[-1], -hover_m, args)[:3], dtype=np.float64)
    return list(points) if float(np.linalg.norm(first_hover - current)) <= float(np.linalg.norm(last_hover - current)) else list(reversed(points))


def execute_points(args: argparse.Namespace, points: list[PressPoint], status: dict[str, object], stop_event: threading.Event) -> None:
    host = str(args.arm_host)
    hover_m = float(args.hover_mm) / 1000.0
    max_press_m = float(args.max_press_mm) / 1000.0
    step_m = float(args.approach_step_mm) / 1000.0
    retreat_m = float(args.retreat_mm) / 1000.0
    if args.run_mode == "force" and args.force_backend == "ros":
        reader: ForceReader | None = DockerForceReader()
    elif args.run_mode == "force":
        reader = JsonForceReader(host, scale=float(args.force_json_scale))
    else:
        reader = None
    try:
        if args.tip_offset_mode == "base":
            print("base tip-offset mode: keeping current tool frame unchanged", flush=True)
        elif args.skip_tool_switch:
            print("tool tip-offset mode: keeping current tool frame unchanged", flush=True)
        elif not args.skip_tool_switch:
            print(f"switching tool frame to {args.tool_name}", flush=True)
            change_tool_in_docker(args.tool_name)
        rm_json.recover_if_needed(host)
        _joints, current_pose, _arm_err, _sys_err, _ik = rm_json.get_current_arm_state(host)
        ordered = order_points(points, current_pose[:3], hover_m, args.order, args)
        print(f"execution_order={[point.index for point in ordered]}", flush=True)
        for seq, point in enumerate(ordered, start=1):
            if stop_event.is_set():
                raise RuntimeError("stop requested")
            tip_hover = pose_at(point, -hover_m)
            hover_pose = command_pose_at(point, -hover_m, args)
            status.update({"state": "hover", "point": point.index, "force": None})
            print(
                f"move hover seq={seq}/{len(ordered)} point={point.index} "
                f"tip_hover={[round(v, 6) for v in tip_hover[:3]]} "
                f"cmd_tcp={[round(v, 6) for v in hover_pose[:3]]}",
                flush=True,
            )
            move_to_hover(host, hover_pose, int(args.entry_speed), float(args.entry_max_step_m), float(args.move_timeout_s), args)
            _joints_after, pose_after, arm_err_after, sys_err_after, _ik_after = rm_json.get_current_arm_state(host)
            if arm_err_after == 0 and sys_err_after == 0:
                actual_tcp = np.asarray(pose_after[:3], dtype=np.float64)
                actual_tip = predicted_tip_from_tcp_pose([float(v) for v in pose_after[:6]], args)
                desired_tip = np.asarray(tip_hover[:3], dtype=np.float64)
                desired_tcp = np.asarray(hover_pose[:3], dtype=np.float64)
                print(
                    f"after hover point={point.index} "
                    f"actual_tcp={[round(float(v), 6) for v in actual_tcp.tolist()]} "
                    f"actual_tip={[round(float(v), 6) for v in actual_tip.tolist()]} "
                    f"tcp_err_mm={float(np.linalg.norm(actual_tcp - desired_tcp)) * 1000.0:.1f} "
                    f"tip_err_mm={float(np.linalg.norm(actual_tip - desired_tip)) * 1000.0:.1f}",
                    flush=True,
                )
            else:
                print(f"after hover state error arm_err={arm_err_after} sys_err={sys_err_after}", flush=True)
            time.sleep(max(0.0, float(args.hover_dwell_s)))

            if args.run_mode == "hover":
                continue
            if not args.allow_contact:
                raise RuntimeError("force mode requires --allow-contact")
            assert reader is not None
            baseline = average_force(reader, int(args.force_filter_count) + 5)
            if baseline is None:
                raise RuntimeError("force baseline unavailable")
            print(
                f"force baseline backend={args.force_backend} "
                f"fx={baseline['fx']:.3f} fy={baseline['fy']:.3f} fz={baseline['fz']:.3f}",
                flush=True,
            )
            reached = False
            offset = -hover_m
            while offset < max_press_m:
                if stop_event.is_set():
                    raise RuntimeError("stop requested")
                offset = min(max_press_m, offset + step_m)
                pose = command_pose_at(point, offset, args)
                guarded_movel(host, pose, int(args.contact_speed), float(args.move_timeout_s), args)
                sample = average_force(reader, int(args.force_filter_count))
                if sample is None:
                    raise RuntimeError("force sample unavailable")
                force_n = force_delta_norm(sample, baseline)
                status.update({"state": "press", "point": point.index, "force": force_n})
                print(f"press point={point.index} offset_m={offset:.4f} force={force_n:.2f}N", flush=True)
                if force_n >= float(args.max_force_n):
                    raise RuntimeError(f"max force exceeded at point {point.index}: {force_n:.2f}N")
                if force_n >= float(args.target_force_n):
                    reached = True
                    break
            if not reached:
                raise RuntimeError(f"target force not reached at point {point.index}")
            time.sleep(max(0.0, float(args.dwell_s)))
            retreat_pose = command_pose_at(point, -retreat_m, args)
            status.update({"state": "retreat", "point": point.index, "force": None})
            guarded_movel(host, retreat_pose, int(args.entry_speed), float(args.move_timeout_s), args)
        status.update({"state": "done", "point": len(points), "force": None})
    except BaseException as exc:
        status.update({"state": f"error:{type(exc).__name__}", "error": str(exc)})
        print(f"purple_press_aborted: {type(exc).__name__}: {exc}", flush=True)
        try:
            rm_json.stop_motion(host)
        except Exception:
            pass
        raise
    finally:
        if reader is not None:
            reader.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect purple bladder line and press 20 points without product motion logic.")
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--matrix-path", default="camera_to_robot.json")
    parser.add_argument("--contact-pose-json", default=str(DEFAULT_CONTACT_POSE))
    parser.add_argument("--tool-name", default="mas_rub")
    parser.set_defaults(skip_tool_switch=True)
    parser.add_argument("--skip-tool-switch", dest="skip_tool_switch", action="store_true")
    parser.add_argument("--switch-tool", dest="skip_tool_switch", action="store_false")
    parser.add_argument("--tcp-to-tip-x-mm", type=float, default=0.0)
    parser.add_argument("--tcp-to-tip-y-mm", type=float, default=0.0)
    parser.add_argument("--tcp-to-tip-z-mm", type=float, default=0.0)
    parser.add_argument("--tip-adjust-step-mm", type=float, default=5.0)
    parser.add_argument("--tip-offset-mode", choices=("tool", "base"), default="tool")
    parser.add_argument("--tcp-to-tip-base-x-mm", type=float, default=0.0)
    parser.add_argument("--tcp-to-tip-base-y-mm", type=float, default=0.0)
    parser.add_argument("--tcp-to-tip-base-z-mm", type=float, default=0.0)
    parser.add_argument("--tip-command-sign", choices=("normal", "inverted"), default="normal")
    parser.add_argument(
        "--single-use-saved-rpy",
        action="store_true",
        help="for key 1, use saved contact RPY instead of current RPY",
    )
    parser.add_argument(
        "--run-use-saved-rpy",
        action="store_true",
        help="for key r, use saved contact RPY instead of locking the current RPY",
    )
    parser.add_argument("--output-dir", default="rm_demo_output")
    parser.add_argument("--model-path", default="yolo11l-pose.pt")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    parser.add_argument("--line-selector", choices=("semantic", "top_outer", "bottom_outer"), default="top_outer")
    parser.add_argument("--point-count", type=int, default=20)
    parser.add_argument("--finger-width", type=float, default=45.0)
    parser.add_argument("--sample-points", type=int, default=40)
    parser.add_argument("--detect-every-s", type=float, default=0.8)
    parser.add_argument("--press-side", choices=("into_body", "outward"), default="into_body")
    parser.add_argument("--orientation-mode", choices=("fixed", "normal"), default="fixed")
    parser.add_argument("--normal-tool-axis", choices=("pos_z", "neg_z"), default="pos_z")
    parser.add_argument("--normal-direction", choices=("outward", "press"), default="outward")
    parser.add_argument("--normal-window-px", type=int, default=31)
    parser.add_argument("--normal-stride-px", type=int, default=2)
    parser.add_argument("--normal-depth-band-m", type=float, default=0.08)
    parser.add_argument("--normal-min-points", type=int, default=40)
    parser.add_argument("--hover-mm", type=float, default=30.0)
    parser.add_argument("--retreat-mm", type=float, default=30.0)
    parser.add_argument("--max-press-mm", type=float, default=8.0)
    parser.add_argument("--approach-step-mm", type=float, default=2.0)
    parser.add_argument("--target-force-n", type=float, default=2.0)
    parser.add_argument("--max-force-n", type=float, default=5.0)
    parser.add_argument("--force-filter-count", type=int, default=3)
    parser.add_argument("--force-backend", choices=("json", "ros"), default="json")
    parser.add_argument("--force-json-scale", type=float, default=0.001)
    parser.add_argument("--dwell-s", type=float, default=0.3)
    parser.add_argument("--hover-dwell-s", type=float, default=0.05)
    parser.add_argument("--entry-speed", type=int, default=5)
    parser.add_argument("--contact-speed", type=int, default=1)
    parser.add_argument("--entry-max-step-m", type=float, default=0.02)
    parser.add_argument("--entry-max-angle-step-rad", type=float, default=0.12)
    parser.add_argument("--entry-orientation-policy", choices=("blend", "position_then_orient"), default="blend")
    parser.add_argument("--entry-position-motion", choices=("movel", "movej_p"), default="movel")
    parser.add_argument("--entry-orient-motion", choices=("movel", "movej_p"), default="movel")
    parser.add_argument("--move-timeout-s", type=float, default=20.0)
    parser.add_argument("--hard-max-z-m", type=float, default=0.38)
    parser.add_argument("--max-z-above-target-m", type=float, default=0.04)
    parser.add_argument("--allow-high-start-descend", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-z-above-start-m", type=float, default=0.005)
    parser.add_argument("--watchdog-period-s", type=float, default=0.04)
    parser.add_argument("--tcp-overlay-period-s", type=float, default=0.25)
    parser.add_argument("--order", choices=("nearest_endpoint", "first_to_last", "last_to_first"), default="nearest_endpoint")
    parser.add_argument(
        "--no-freeze-after-save",
        dest="freeze_after_save",
        action="store_false",
        help="keep live bladder detection running after pressing s",
    )
    parser.set_defaults(freeze_after_save=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--realsense-serial", default="")
    parser.add_argument("--window-name", default="Purple 20 point press")
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--run-mode", choices=("force", "hover"), default="force")
    parser.add_argument("--allow-contact", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    matrix = load_transform_matrix(str(Path(args.matrix_path).resolve()))
    if matrix is None:
        raise RuntimeError(f"camera->robot matrix not found: {args.matrix_path}")
    camera_from_base = np.linalg.inv(np.asarray(matrix, dtype=np.float64))

    pipeline, profile, color_format = _stream_profile(args)
    align = rs.align(rs.stream.color)
    depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
    intrinsics = _intrinsics_from_profile(profile, depth_scale)
    print(f"realsense_started color_format={color_format} depth_scale={depth_scale}", flush=True)

    latest_detection: dict[str, object] | None = None
    latest_overlay: np.ndarray | None = None
    latest_error = ""
    last_detect_t = 0.0
    saved_points: list[PressPoint] = []
    saved_plan: Path | None = None
    detection_frozen = False
    motion_thread: threading.Thread | None = None
    stop_event = threading.Event()
    status: dict[str, object] = {"state": "idle", "point": 0, "force": None}
    current_tcp_pixel: list[float] | None = None
    current_tip_pixel: list[float] | None = None
    current_tcp_pose: list[float] | None = None
    tcp_error = ""
    last_tcp_t = 0.0

    cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
    try:
        for _ in range(max(1, int(args.warmup_frames))):
            _read_aligned_frame(pipeline, align, color_format, depth_scale)
        while True:
            frame_pair = _read_aligned_frame(pipeline, align, color_format, depth_scale)
            if frame_pair is None:
                continue
            color_bgr, depth_m = frame_pair
            display = color_bgr.copy()
            now = time.time()
            if now - last_tcp_t >= float(args.tcp_overlay_period_s):
                last_tcp_t = now
                try:
                    _joints, pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(args.arm_host)
                    current_tcp_pose = [float(v) for v in pose[:6]]
                    if arm_err == 0 and sys_err == 0:
                        current_tcp_pixel = base_to_pixel(np.asarray(pose[:3], dtype=np.float64), camera_from_base, intrinsics)
                        current_tip_pixel = base_to_pixel(predicted_tip_from_tcp_pose(current_tcp_pose, args), camera_from_base, intrinsics)
                        tcp_error = ""
                    else:
                        tcp_error = f"tcp_state arm_err={arm_err} sys_err={sys_err}"
                except Exception as exc:
                    tcp_error = f"tcp_error={type(exc).__name__}"
            if not detection_frozen and motion_thread is None and now - last_detect_t >= float(args.detect_every_s):
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
                    side, line_type = select_line(detection, args.line_selector, args.side, args.line_type)
                    prefix = f"{side}_{line_type}"
                    latest_detection = detection
                    latest_overlay = overlay
                    latest_error = ""
                    display = overlay.copy()
                    draw_polyline(display, list(detection.get(f"{prefix}_pixel", [])), (255, 0, 255), 3)
                except Exception as exc:
                    latest_error = f"{type(exc).__name__}: {exc}"
                    if latest_overlay is not None:
                        display = latest_overlay.copy()
            elif latest_overlay is not None and motion_thread is None:
                display = latest_overlay.copy()

            if saved_points:
                draw_polyline(display, [p.pixel for p in saved_points], (255, 0, 255), 3)
                draw_polyline(display, [p.hover_pixel for p in saved_points], (255, 255, 0), 2)
            draw_marker(display, current_tcp_pixel, "TCP", (0, 0, 255), "cross")
            draw_marker(display, current_tip_pixel, "TIP", (0, 255, 0), "box")

            force = status.get("force")
            force_text = "n/a" if force is None else f"{float(force):.2f}N"
            lines = [
                "s: save/freeze | f: unfreeze | o: learn base TIP | 1: first hover | r: run | q/ESC",
                f"mode={args.run_mode} order={args.order} target={args.target_force_n:.1f}N max={args.max_force_n:.1f}N",
                f"motion={status.get('state')} point={status.get('point')} force={force_text} frozen={detection_frozen}",
                f"tip_mode={args.tip_offset_mode} tool=({args.tcp_to_tip_x_mm:.1f},{args.tcp_to_tip_y_mm:.1f},{args.tcp_to_tip_z_mm:.1f}) sign={args.tip_command_sign}",
                f"base_tip_mm=({args.tcp_to_tip_base_x_mm:.1f},{args.tcp_to_tip_base_y_mm:.1f},{args.tcp_to_tip_base_z_mm:.1f})",
            ]
            if current_tcp_pose is not None:
                lines.append(f"tcp=({current_tcp_pose[0]:.3f},{current_tcp_pose[1]:.3f},{current_tcp_pose[2]:.3f})")
            if saved_plan is not None:
                lines.append(f"saved={saved_plan.name}")
            if tcp_error:
                lines.append(tcp_error[:90])
            if latest_error:
                lines.append(f"detect_error={latest_error[:90]}")
            draw_status(display, lines)
            cv2.imshow(args.window_name, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                stop_event.set()
                try:
                    rm_json.stop_motion(args.arm_host)
                except Exception:
                    pass
                break

            if key in (ord("x"), ord("a"), ord("y"), ord("h"), ord("z"), ord("c"), ord("X"), ord("Y"), ord("Z")):
                step = float(args.tip_adjust_step_mm)
                changed_axis = ""
                if key == ord("x"):
                    args.tcp_to_tip_x_mm += step
                    changed_axis = "+tool_x"
                elif key in (ord("a"), ord("X")):
                    args.tcp_to_tip_x_mm -= step
                    changed_axis = "-tool_x"
                elif key == ord("y"):
                    args.tcp_to_tip_y_mm += step
                    changed_axis = "+tool_y"
                elif key in (ord("h"), ord("Y")):
                    args.tcp_to_tip_y_mm -= step
                    changed_axis = "-tool_y"
                elif key == ord("z"):
                    args.tcp_to_tip_z_mm += step
                    changed_axis = "+tool_z"
                elif key in (ord("c"), ord("Z")):
                    args.tcp_to_tip_z_mm -= step
                    changed_axis = "-tool_z"
                print(
                    f"tip_adjust={changed_axis} "
                    "tcp_to_tip_mm="
                    f"({args.tcp_to_tip_x_mm:.1f},{args.tcp_to_tip_y_mm:.1f},{args.tcp_to_tip_z_mm:.1f}) "
                    "rerun_args="
                    f"--tip-offset-mode tool "
                    f"--tcp-to-tip-x-mm {args.tcp_to_tip_x_mm:.1f} "
                    f"--tcp-to-tip-y-mm {args.tcp_to_tip_y_mm:.1f} "
                    f"--tcp-to-tip-z-mm {args.tcp_to_tip_z_mm:.1f} "
                    f"--tip-command-sign {args.tip_command_sign}",
                    flush=True,
                )
                continue

            if key == ord("v"):
                args.tip_command_sign = "inverted" if args.tip_command_sign == "normal" else "normal"
                print(f"tip_command_sign={args.tip_command_sign}", flush=True)
                continue

            if key == ord("f"):
                detection_frozen = not detection_frozen
                print(f"detection_frozen={detection_frozen}", flush=True)
                continue

            if key == ord("o"):
                if not saved_points:
                    print("cannot learn base tip offset: press s first", flush=True)
                    continue
                if current_tcp_pose is None:
                    print("cannot learn base tip offset: current TCP unavailable", flush=True)
                    continue
                target_tip = np.asarray(pose_at(saved_points[0], -float(args.hover_mm) / 1000.0)[:3], dtype=np.float64)
                current_tcp = np.asarray(current_tcp_pose[:3], dtype=np.float64)
                offset_base = target_tip - current_tcp
                args.tip_offset_mode = "base"
                args.tcp_to_tip_base_x_mm = float(offset_base[0]) * 1000.0
                args.tcp_to_tip_base_y_mm = float(offset_base[1]) * 1000.0
                args.tcp_to_tip_base_z_mm = float(offset_base[2]) * 1000.0
                print(
                    "learned_base_tip_offset "
                    f"target_tip={[round(float(v), 6) for v in target_tip.tolist()]} "
                    f"current_tcp={[round(float(v), 6) for v in current_tcp.tolist()]} "
                    f"base_tip_mm=({args.tcp_to_tip_base_x_mm:.1f},{args.tcp_to_tip_base_y_mm:.1f},{args.tcp_to_tip_base_z_mm:.1f}) "
                    "rerun_args="
                    f"--tip-offset-mode base "
                    f"--tcp-to-tip-base-x-mm {args.tcp_to_tip_base_x_mm:.1f} "
                    f"--tcp-to-tip-base-y-mm {args.tcp_to_tip_base_y_mm:.1f} "
                    f"--tcp-to-tip-base-z-mm {args.tcp_to_tip_base_z_mm:.1f}",
                    flush=True,
                )
                continue

            if key == ord("s"):
                if latest_detection is None:
                    print("no detection available", flush=True)
                    continue
                stamp = time.strftime("%Y%m%d_%H%M%S")
                points, meta = build_press_points(
                    args=args,
                    detection=latest_detection,
                    depth_m=depth_m,
                    intrinsics=intrinsics,
                    matrix=np.asarray(matrix, dtype=np.float64),
                )
                saved_points = points
                saved_plan = save_plan(
                    args=args,
                    stamp=stamp,
                    color_bgr=color_bgr,
                    depth_m=depth_m,
                    intrinsics=intrinsics,
                    detection=latest_detection,
                    overlay=display,
                    points=points,
                    meta=meta,
                )
                detection_frozen = bool(args.freeze_after_save)
                print(f"saved_plan={saved_plan}", flush=True)
                print(f"detection_frozen={detection_frozen}", flush=True)

            if key == ord("1"):
                if not args.run:
                    print("refusing motion: start with --run", flush=True)
                    continue
                if not saved_points:
                    print("no saved points: press s first", flush=True)
                    continue
                if motion_thread is not None and motion_thread.is_alive():
                    print("motion already running", flush=True)
                    continue
                single_args = argparse.Namespace(**vars(args))
                single_args.run_mode = "hover"
                single_args.order = "first_to_last"
                single_point = saved_points[0]
                if not args.single_use_saved_rpy and args.orientation_mode != "normal":
                    try:
                        _joints, latest_pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(args.arm_host)
                        if arm_err == 0 and sys_err == 0:
                            single_point = replace(single_point, rpy=[float(v) for v in latest_pose[3:6]])
                            print(
                                f"single_point uses current_rpy={[round(float(v), 6) for v in single_point.rpy]}",
                                flush=True,
                            )
                        else:
                            print(f"single_point cannot use current_rpy: arm_err={arm_err} sys_err={sys_err}", flush=True)
                    except Exception as exc:
                        print(f"single_point current_rpy unavailable: {type(exc).__name__}: {exc}", flush=True)
                stop_event.clear()
                status.update({"state": "first_hover", "point": single_point.index, "force": None})
                print(
                    f"starting first hover only: point={single_point.index} sign={single_args.tip_command_sign} "
                    f"tip_hover={[round(float(v), 6) for v in pose_at(single_point, -float(args.hover_mm) / 1000.0)[:3]]} "
                    f"cmd_tcp={[round(float(v), 6) for v in command_pose_at(single_point, -float(args.hover_mm) / 1000.0, single_args)[:3]]}",
                    flush=True,
                )
                motion_thread = threading.Thread(
                    target=execute_points,
                    kwargs={"args": single_args, "points": [single_point], "status": status, "stop_event": stop_event},
                    daemon=True,
                )
                motion_thread.start()

            if key == ord("r"):
                if not args.run:
                    print("refusing motion: start with --run", flush=True)
                    continue
                if args.run_mode == "force" and not args.allow_contact:
                    print("refusing force motion: add --allow-contact", flush=True)
                    continue
                if not saved_points:
                    print("no saved points: press s first", flush=True)
                    continue
                if motion_thread is not None and motion_thread.is_alive():
                    print("motion already running", flush=True)
                    continue
                run_points = saved_points
                if not args.run_use_saved_rpy and args.orientation_mode != "normal":
                    try:
                        _joints, latest_pose, arm_err, sys_err, _ik = rm_json.get_current_arm_state(args.arm_host)
                        if arm_err == 0 and sys_err == 0:
                            current_rpy = [float(v) for v in latest_pose[3:6]]
                            run_points = [replace(point, rpy=list(current_rpy)) for point in saved_points]
                            print(
                                f"run_points uses current_rpy={[round(float(v), 6) for v in current_rpy]}",
                                flush=True,
                            )
                        else:
                            print(f"run_points cannot use current_rpy: arm_err={arm_err} sys_err={sys_err}", flush=True)
                    except Exception as exc:
                        print(f"run_points current_rpy unavailable: {type(exc).__name__}: {exc}", flush=True)
                stop_event.clear()
                status.update({"state": "starting", "point": 0, "force": None})
                motion_thread = threading.Thread(
                    target=execute_points,
                    kwargs={"args": args, "points": run_points, "status": status, "stop_event": stop_event},
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
