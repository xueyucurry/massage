from __future__ import annotations

import json
import os

import numpy as np


def load_transform_matrix(path: str) -> np.ndarray | None:
    if not path or not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        if str(path).lower().endswith((".yaml", ".yml")):
            try:
                import yaml  # type: ignore
            except Exception:
                return None
            data = yaml.safe_load(f) or {}
            calib = data.get("eye_on_hand_calibrate", {}) if isinstance(data, dict) else {}
            matrix = np.asarray(
                [
                    [float(calib.get(f"tf{r}_{c}", 0.0)) for c in range(4)]
                    for r in range(4)
                ],
                dtype=np.float64,
            )
        else:
            data = json.load(f)
            matrix = np.asarray(data.get("matrix", []), dtype=np.float64)
    if matrix.shape != (4, 4):
        return None
    return matrix


def transform_points(points_xyz: list[list[float]], matrix: np.ndarray) -> list[list[float]]:
    out: list[list[float]] = []
    for point in points_xyz:
        p4 = np.asarray([float(point[0]), float(point[1]), float(point[2]), 1.0], dtype=np.float64)
        q = matrix @ p4
        out.append([float(q[0]), float(q[1]), float(q[2])])
    return out


def pose_to_matrix(pose_xyzrpy: list[float]) -> np.ndarray:
    x, y, z, roll, pitch, yaw = [float(v) for v in pose_xyzrpy[:6]]
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rot = np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[:3, 3] = [x, y, z]
    return mat


def transform_points_eye_in_hand(
    points_xyz: list[list[float]],
    tool_pose_m: list[float],
    tool_from_camera_matrix: np.ndarray,
) -> list[list[float]]:
    base_from_tool = pose_to_matrix(tool_pose_m)
    base_from_camera = base_from_tool @ np.asarray(tool_from_camera_matrix, dtype=np.float64)
    return transform_points(points_xyz, base_from_camera)


def attach_robot_points(result: dict[str, object], matrix: np.ndarray) -> dict[str, object]:
    updated = dict(result)
    for side in ("left", "right"):
        key = f"{side}_meridian_camera"
        robot_key = f"{side}_meridian_robot"
        updated[robot_key] = transform_points(list(updated.get(key, [])), matrix)
    side = str(updated.get("selected_side", ""))
    if side in ("left", "right"):
        updated["selected_meridian_robot"] = updated[f"{side}_meridian_robot"]
        updated["robot_frame_unit"] = "meters"
    return updated

