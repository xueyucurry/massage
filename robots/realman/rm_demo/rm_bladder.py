from __future__ import annotations

import json
import math
import os
import time
from dataclasses import asdict, dataclass, field
from typing import Any

import cv2
import numpy as np

from .config import (
    DEFAULT_CONF,
    DEFAULT_FINGER_WIDTH_MM,
    DEFAULT_MODEL_PATH,
    DEPTH_MEDIAN_RADIUS,
    ESTIMATED_SHOULDER_MM,
    MIN_VISUAL_OFFSET_PX,
)
from .rm_capture import ensure_output_dir
from .rm_ros import create_arm_backend
from .rm_speed import normalize_motion_speed
from .rm_transform import transform_points, transform_points_eye_in_hand


POSE_ROTATION_MODES = ("none", "cw90", "ccw90", "180")
REQUIRED_KEYPOINTS = (5, 6, 11, 12)

_MODEL_CACHE: dict[str, Any] = {}


@dataclass
class BladderMassageFrame:
    index: int
    pixel: list[float]
    robot_point_m: list[float]
    hover_pose_m: list[float]
    dian_jin_pose_m: list[float]
    fen_positive_pose_m: list[float]
    fen_negative_pose_m: list[float]
    press_direction_m: list[float] = field(default_factory=list)
    split_axis_m: list[float] = field(default_factory=list)
    tangent_axis_m: list[float] = field(default_factory=list)
    base_pose_m: list[float] = field(default_factory=list)
    source_pose_quat: list[float] = field(default_factory=list)


@dataclass
class BladderMassagePlan:
    side: str
    line_type: str
    point_count: int
    hover_m: float
    dian_jin_depth_m: float
    fen_jin_lateral_m: float
    safe_z_m: float
    anchor_pose_m: list[float]
    frames: list[BladderMassageFrame]
    hover_offset_mode: str = "normal"


def _load_model(model_path: str):
    cached = _MODEL_CACHE.get(model_path)
    if cached is not None:
        return cached

    from ultralytics import YOLO
    import torch

    model = YOLO(model_path)
    if torch.cuda.is_available():
        model.to("cuda")
    _MODEL_CACHE[model_path] = model
    return model


def _rotate_image_for_pose(img: np.ndarray, mode: str) -> np.ndarray:
    if mode == "none":
        return img
    if mode == "cw90":
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if mode == "ccw90":
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if mode == "180":
        return cv2.rotate(img, cv2.ROTATE_180)
    raise ValueError(f"unsupported rotation mode: {mode}")


def _map_keypoints_to_original(kpts: np.ndarray, orig_w: int, orig_h: int, mode: str) -> np.ndarray:
    mapped = kpts.copy()
    if mode == "none":
        return mapped
    for point in mapped:
        x = float(point[0])
        y = float(point[1])
        if mode == "cw90":
            point[0] = y
            point[1] = orig_h - 1 - x
        elif mode == "ccw90":
            point[0] = orig_w - 1 - y
            point[1] = x
        elif mode == "180":
            point[0] = orig_w - 1 - x
            point[1] = orig_h - 1 - y
        else:
            raise ValueError(f"unsupported rotation mode: {mode}")
    return mapped


def _pose_candidate_score(kpts: np.ndarray | None) -> float:
    if kpts is None or len(kpts) <= max(REQUIRED_KEYPOINTS):
        return -1.0
    return float(sum(float(kpts[idx][2]) for idx in REQUIRED_KEYPOINTS))


def _extract_best_pose_keypoints(result) -> tuple[np.ndarray | None, float]:
    if result.keypoints is None or len(result.keypoints.data) == 0:
        return None, -1.0
    best_kpts = None
    best_score = -1.0
    for person_kpts in result.keypoints.data.cpu().numpy():
        score = _pose_candidate_score(person_kpts)
        if score > best_score:
            best_score = score
            best_kpts = person_kpts
    return best_kpts, best_score


def _infer_best_pose_with_rotations(model, img: np.ndarray, conf: float = DEFAULT_CONF) -> dict[str, object]:
    orig_h, orig_w = img.shape[:2]
    best: dict[str, object] = {"kpts": None, "score": -1.0, "rotation": "none"}
    for mode in POSE_ROTATION_MODES:
        rotated = _rotate_image_for_pose(img, mode)
        results = model(rotated, verbose=False, conf=conf)
        kpts, score = _extract_best_pose_keypoints(results[0])
        if kpts is None:
            continue
        mapped = _map_keypoints_to_original(kpts, orig_w, orig_h, mode)
        if score > float(best["score"]):
            best["kpts"] = mapped
            best["score"] = score
            best["rotation"] = mode
    return best


def _normalize_vec(vec: list[float] | np.ndarray, eps: float = 1e-6) -> np.ndarray | None:
    arr = np.asarray(vec, dtype=np.float64)
    norm = float(np.linalg.norm(arr))
    if norm <= eps:
        return None
    return arr / norm


def _median_depth(depth_m: np.ndarray, pixel_u: float, pixel_v: float, radius: int = DEPTH_MEDIAN_RADIUS) -> float:
    h, w = depth_m.shape[:2]
    cu = int(round(pixel_u))
    cv = int(round(pixel_v))
    if cu < 0 or cu >= w or cv < 0 or cv >= h:
        return 0.0
    window = depth_m[max(0, cv - radius) : min(h, cv + radius + 1), max(0, cu - radius) : min(w, cu + radius + 1)]
    values = window[window > 0.1]
    if values.size == 0:
        return 0.0
    return float(np.median(values))


def _point3d_from_depth(
    intrinsics_data: dict[str, object],
    pixel_u: float,
    pixel_v: float,
    depth_m: np.ndarray,
) -> list[float] | None:
    h, w = depth_m.shape[:2]
    if pixel_u < 0 or pixel_u >= w or pixel_v < 0 or pixel_v >= h:
        return None
    dist = _median_depth(depth_m, pixel_u, pixel_v)
    if dist <= 0.0:
        return None
    fx = float(intrinsics_data["fx"])
    fy = float(intrinsics_data["fy"])
    ppx = float(intrinsics_data["ppx"])
    ppy = float(intrinsics_data["ppy"])
    if abs(fx) < 1e-9 or abs(fy) < 1e-9:
        return None
    x = (float(pixel_u) - ppx) / fx * dist
    y = (float(pixel_v) - ppy) / fy * dist
    return [float(x), float(y), float(dist)]


def _sample_line_pixels(start: tuple[float, float], end: tuple[float, float], num_points: int) -> list[list[int]]:
    pts: list[list[int]] = []
    steps = max(1, int(num_points) - 1)
    for idx in range(max(1, int(num_points))):
        t = idx / max(1, steps)
        pts.append(
            [
                int(round(float(start[0]) * (1.0 - t) + float(end[0]) * t)),
                int(round(float(start[1]) * (1.0 - t) + float(end[1]) * t)),
            ]
        )
    return pts


def _pixels_to_points3d(
    pixels: list[list[int]],
    depth_m: np.ndarray,
    intrinsics_data: dict[str, object],
) -> list[list[float]]:
    out: list[list[float]] = []
    for u, v in pixels:
        point = _point3d_from_depth(intrinsics_data, float(u), float(v), depth_m)
        if point is not None:
            out.append(point)
    return out


def _build_spine_seed_from_torso(kpts: np.ndarray, finger_width_mm: float) -> dict[str, object] | None:
    if any(float(kpts[idx][2]) < 0.35 for idx in REQUIRED_KEYPOINTS):
        return None

    ls = np.asarray([float(kpts[5][0]), float(kpts[5][1])], dtype=np.float64)
    rs = np.asarray([float(kpts[6][0]), float(kpts[6][1])], dtype=np.float64)
    lh = np.asarray([float(kpts[11][0]), float(kpts[11][1])], dtype=np.float64)
    rh = np.asarray([float(kpts[12][0]), float(kpts[12][1])], dtype=np.float64)

    shoulder_mid = (ls + rs) * 0.5
    hip_mid = (lh + rh) * 0.5
    shoulder_vec = rs - ls
    hip_vec = rh - lh
    torso_vec = hip_mid - shoulder_mid

    shoulder_px = float(np.linalg.norm(shoulder_vec))
    hip_px = float(np.linalg.norm(hip_vec))
    width_px = max(20.0, float(np.median([max(shoulder_px, 1.0), max(hip_px, 1.0)])))
    torso_len_px = float(np.linalg.norm(torso_vec))
    if torso_len_px < width_px * 0.85:
        return None

    axis_hint = _normalize_vec(torso_vec)
    if axis_hint is None:
        return None

    torso_pts = np.stack([ls, rs, lh, rh], axis=0)
    torso_center = np.mean(torso_pts, axis=0)
    centered = torso_pts - torso_center
    try:
        _, eigvecs = np.linalg.eigh(centered.T @ centered)
        pca_axis = np.asarray(eigvecs[:, -1], dtype=np.float64)
    except Exception:
        pca_axis = axis_hint
    if float(np.dot(pca_axis, axis_hint)) < 0.0:
        pca_axis = -pca_axis
    axis_2d = _normalize_vec(axis_hint * 0.6 + pca_axis * 0.4)
    if axis_2d is None:
        axis_2d = axis_hint

    lateral_2d = _normalize_vec(np.array([-axis_2d[1], axis_2d[0]], dtype=np.float64))
    if lateral_2d is None:
        return None

    neck_pt = shoulder_mid - axis_2d * (0.10 * torso_len_px)
    tail_pt = hip_mid + axis_2d * (0.12 * torso_len_px)
    pixels_per_mm = shoulder_px / ESTIMATED_SHOULDER_MM if ESTIMATED_SHOULDER_MM > 0 else 1.0
    body_offset_px = max(MIN_VISUAL_OFFSET_PX, float(finger_width_mm) * pixels_per_mm)

    return {
        "spine_line": (
            (float(neck_pt[0]), float(neck_pt[1])),
            (float(tail_pt[0]), float(tail_pt[1])),
        ),
        "lateral_direction_2d": lateral_2d.tolist(),
        "body_offset_px": float(body_offset_px),
        "shoulder_px": float(shoulder_px),
    }


def _weighted_midpoint(kpts: np.ndarray, indices: tuple[int, int], min_conf: float = 0.005) -> np.ndarray | None:
    pts: list[np.ndarray] = []
    weights: list[float] = []
    for idx in indices:
        x, y, conf = float(kpts[idx][0]), float(kpts[idx][1]), float(kpts[idx][2])
        if not np.isfinite(x) or not np.isfinite(y) or conf < min_conf:
            continue
        pts.append(np.asarray([x, y], dtype=np.float64))
        weights.append(max(0.05, conf))
    if not pts:
        return None
    arr = np.stack(pts, axis=0)
    w = np.asarray(weights, dtype=np.float64)
    return np.sum(arr * w[:, None], axis=0) / max(1e-6, float(np.sum(w)))


def _pair_width(kpts: np.ndarray, indices: tuple[int, int], min_conf: float = 0.005) -> float | None:
    a, b = indices
    if float(kpts[a][2]) < min_conf or float(kpts[b][2]) < min_conf:
        return None
    pa = np.asarray([float(kpts[a][0]), float(kpts[a][1])], dtype=np.float64)
    pb = np.asarray([float(kpts[b][0]), float(kpts[b][1])], dtype=np.float64)
    if not np.all(np.isfinite(pa)) or not np.all(np.isfinite(pb)):
        return None
    width = float(np.linalg.norm(pb - pa))
    return width if width > 1.0 else None


def _build_relaxed_side_spine_seed(
    kpts: np.ndarray,
    frame_shape: tuple[int, int, int],
    finger_width_mm: float,
) -> dict[str, object] | None:
    shoulder_mid = _weighted_midpoint(kpts, (5, 6))
    hip_mid = _weighted_midpoint(kpts, (11, 12))
    if shoulder_mid is None or hip_mid is None:
        return None

    torso_vec = hip_mid - shoulder_mid
    torso_len_px = float(np.linalg.norm(torso_vec))
    if torso_len_px < 40.0:
        return None
    axis_2d = _normalize_vec(torso_vec)
    if axis_2d is None:
        return None
    lateral_2d = _normalize_vec(np.asarray([-axis_2d[1], axis_2d[0]], dtype=np.float64))
    if lateral_2d is None:
        return None

    widths = [
        value
        for value in (
            _pair_width(kpts, (5, 6)),
            _pair_width(kpts, (11, 12)),
        )
        if value is not None
    ]
    width_px = float(np.median(widths)) if widths else max(80.0, min(220.0, torso_len_px * 0.35))
    pixels_per_mm = width_px / ESTIMATED_SHOULDER_MM if ESTIMATED_SHOULDER_MM > 0 else 1.0
    body_offset_px = max(MIN_VISUAL_OFFSET_PX, float(finger_width_mm) * pixels_per_mm)

    h, w = frame_shape[:2]
    margin = 0.04 * max(w, h)
    neck_pt = shoulder_mid - axis_2d * min(0.10 * torso_len_px, margin)
    tail_pt = hip_mid + axis_2d * min(0.12 * torso_len_px, margin * 1.2)

    return {
        "spine_line": (
            (float(neck_pt[0]), float(neck_pt[1])),
            (float(tail_pt[0]), float(tail_pt[1])),
        ),
        "lateral_direction_2d": lateral_2d.tolist(),
        "body_offset_px": float(body_offset_px),
        "shoulder_px": float(width_px),
        "relaxed_side_lying": True,
    }


def _expand_meridian_lines_from_spine(
    spine_line: tuple[tuple[float, float], tuple[float, float]] | None,
    inner_lines: tuple[
        tuple[tuple[float, float], tuple[float, float]],
        tuple[tuple[float, float], tuple[float, float]],
    ]
    | None,
    scale: float = 2.0,
):
    if spine_line is None or inner_lines is None:
        return None

    spine_neck = np.asarray(spine_line[0], dtype=np.float64)
    spine_tail = np.asarray(spine_line[1], dtype=np.float64)
    outer_lines = []
    for inner_line in inner_lines:
        neck_inner = np.asarray(inner_line[0], dtype=np.float64)
        tail_inner = np.asarray(inner_line[1], dtype=np.float64)
        neck_outer = spine_neck + (neck_inner - spine_neck) * float(scale)
        tail_outer = spine_tail + (tail_inner - spine_tail) * float(scale)
        outer_lines.append(
            (
                (float(neck_outer[0]), float(neck_outer[1])),
                (float(tail_outer[0]), float(tail_outer[1])),
            )
        )
    return tuple(outer_lines)


def _clamp_pixel(x: float, y: float, width: int, height: int) -> list[int]:
    return [
        int(max(0, min(width - 1, round(x)))),
        int(max(0, min(height - 1, round(y)))),
    ]


def _build_body_bbox_pixel(
    image_width: int,
    image_height: int,
    kpts: np.ndarray | None,
    line_pixels: dict[str, list[list[int]]] | None,
) -> list[list[int]]:
    points: list[tuple[int, int]] = []
    if kpts is not None:
        for idx in REQUIRED_KEYPOINTS:
            if len(kpts) > idx and float(kpts[idx][2]) > 0.2:
                points.append((int(round(float(kpts[idx][0]))), int(round(float(kpts[idx][1])))))
    if line_pixels:
        for pixels in line_pixels.values():
            if not pixels:
                continue
            for px in (pixels[0], pixels[-1]):
                if len(px) >= 2:
                    points.append((int(px[0]), int(px[1])))
            mid = pixels[len(pixels) // 2]
            if len(mid) >= 2:
                points.append((int(mid[0]), int(mid[1])))

    if not points:
        return [[0, 0], [image_width - 1, 0], [0, image_height - 1], [image_width - 1, image_height - 1]]

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    span_x = max(1, max(xs) - min(xs))
    span_y = max(1, max(ys) - min(ys))
    pad_x = max(30, int(span_x * 0.35))
    pad_y = max(30, int(span_y * 0.35))
    x1 = max(0, min(xs) - pad_x)
    x2 = min(image_width - 1, max(xs) + pad_x)
    y1 = max(0, min(ys) - pad_y)
    y2 = min(image_height - 1, max(ys) + pad_y)
    return [[int(x1), int(y1)], [int(x2), int(y1)], [int(x1), int(y2)], [int(x2), int(y2)]]


def _build_overlay(
    image_bgr: np.ndarray,
    spine_line: tuple[tuple[float, float], tuple[float, float]],
    inner_lines,
    outer_lines,
    pose_rotation: str,
    finger_width_mm: float,
    shoulder_cm_real: float | None,
) -> np.ndarray:
    overlay = image_bgr.copy()
    cv2.line(overlay, _clamp_tuple(spine_line[0]), _clamp_tuple(spine_line[1]), (0, 0, 255), 2)
    cv2.putText(
        overlay,
        "spine",
        _clamp_tuple(spine_line[0]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 0, 255),
        1,
    )

    def draw_labeled(line, label: str, color: tuple[int, int, int]) -> None:
        cv2.line(overlay, _clamp_tuple(line[0]), _clamp_tuple(line[1]), color, 2)
        mid = ((float(line[0][0]) + float(line[1][0])) * 0.5, (float(line[0][1]) + float(line[1][1])) * 0.5)
        cv2.putText(
            overlay,
            label,
            _clamp_tuple(mid),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
        )

    for label, line in zip(("left_inner", "right_inner"), inner_lines or []):
        draw_labeled(line, label, (0, 255, 0))
    for label, line in zip(("left_outer", "right_outer"), outer_lines or []):
        draw_labeled(line, label, (255, 0, 255))
    shoulder_txt = f"{shoulder_cm_real:.1f}cm" if shoulder_cm_real and shoulder_cm_real > 0 else "N/A"
    cv2.putText(
        overlay,
        f"Bladder | Offset: {finger_width_mm:.1f}mm | Shoulder: {shoulder_txt}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        overlay,
        f"Pose rot: {pose_rotation} | Inner x1 | Outer x2",
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        1,
    )
    return overlay


def _clamp_tuple(point: tuple[float, float]) -> tuple[int, int]:
    return int(round(point[0])), int(round(point[1]))


def detect_bladder_lines(
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics_data: dict[str, object],
    finger_width_mm: float = DEFAULT_FINGER_WIDTH_MM,
    model_path: str = DEFAULT_MODEL_PATH,
    sample_points: int = 30,
    conf: float = DEFAULT_CONF,
) -> tuple[dict[str, object], np.ndarray]:
    model = _load_model(model_path)
    pose_info = _infer_best_pose_with_rotations(model, color_bgr, conf=conf)
    if pose_info["kpts"] is None:
        raise RuntimeError("no valid body pose detected in current frame")

    kpts = pose_info["kpts"]
    seed = _build_spine_seed_from_torso(kpts, finger_width_mm=float(finger_width_mm))
    detection_mode = "strict"
    if seed is None:
        seed = _build_relaxed_side_spine_seed(kpts, color_bgr.shape, finger_width_mm=float(finger_width_mm))
        detection_mode = "relaxed_side_lying"
    if seed is None:
        raise RuntimeError("pose confidence is too low for bladder meridian generation")

    h, w = color_bgr.shape[:2]
    spine_line = seed["spine_line"]
    lateral_direction_2d = np.asarray(seed["lateral_direction_2d"], dtype=np.float64)
    body_offset_px = float(seed["body_offset_px"])

    neck = np.asarray(spine_line[0], dtype=np.float64)
    tail = np.asarray(spine_line[1], dtype=np.float64)
    neck_l = neck - lateral_direction_2d * body_offset_px
    neck_r = neck + lateral_direction_2d * body_offset_px
    tail_l = tail - lateral_direction_2d * body_offset_px
    tail_r = tail + lateral_direction_2d * body_offset_px
    inner_lines = (
        ((float(neck_l[0]), float(neck_l[1])), (float(tail_l[0]), float(tail_l[1]))),
        ((float(neck_r[0]), float(neck_r[1])), (float(tail_r[0]), float(tail_r[1]))),
    )
    outer_lines = _expand_meridian_lines_from_spine(spine_line, inner_lines, scale=2.0)

    line_pixels = {
        "left_inner": _sample_line_pixels(inner_lines[0][0], inner_lines[0][1], sample_points),
        "right_inner": _sample_line_pixels(inner_lines[1][0], inner_lines[1][1], sample_points),
        "left_outer": _sample_line_pixels(outer_lines[0][0], outer_lines[0][1], sample_points),
        "right_outer": _sample_line_pixels(outer_lines[1][0], outer_lines[1][1], sample_points),
    }
    camera_points = {
        name: _pixels_to_points3d(pixels, depth_m, intrinsics_data)
        for name, pixels in line_pixels.items()
    }

    shoulder_cm_real = None
    left_pt3 = _point3d_from_depth(intrinsics_data, float(kpts[5][0]), float(kpts[5][1]), depth_m)
    right_pt3 = _point3d_from_depth(intrinsics_data, float(kpts[6][0]), float(kpts[6][1]), depth_m)
    if left_pt3 is not None and right_pt3 is not None:
        shoulder_cm_real = float(np.linalg.norm(np.asarray(right_pt3) - np.asarray(left_pt3)) * 100.0)

    overlay = _build_overlay(
        color_bgr,
        spine_line=spine_line,
        inner_lines=inner_lines,
        outer_lines=outer_lines,
        pose_rotation=str(pose_info["rotation"]),
        finger_width_mm=float(finger_width_mm),
        shoulder_cm_real=shoulder_cm_real,
    )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    result: dict[str, object] = {
        "timestamp": timestamp,
        "detector_backend": "bladder_pose",
        "model_path": os.path.abspath(model_path),
        "image_size": {"width": int(w), "height": int(h)},
        "pose_rotation": str(pose_info["rotation"]),
        "spine_detection_mode": detection_mode,
        "finger_width_mm": float(finger_width_mm),
        "sample_points": int(sample_points),
        "camera_frame_unit": "meters",
        "intrinsics": intrinsics_data,
        "spine_line_pixel": _sample_line_pixels(spine_line[0], spine_line[1], sample_points),
        "left_inner_pixel": line_pixels["left_inner"],
        "right_inner_pixel": line_pixels["right_inner"],
        "left_outer_pixel": line_pixels["left_outer"],
        "right_outer_pixel": line_pixels["right_outer"],
        "left_inner_camera": camera_points["left_inner"],
        "right_inner_camera": camera_points["right_inner"],
        "left_outer_camera": camera_points["left_outer"],
        "right_outer_camera": camera_points["right_outer"],
        "body_bbox_pixel": _build_body_bbox_pixel(
            image_width=w,
            image_height=h,
            kpts=kpts,
            line_pixels=line_pixels,
        ),
        "keypoints": {str(idx): [float(v) for v in kpts[idx][:3]] for idx in REQUIRED_KEYPOINTS},
    }
    return result, overlay


def select_bladder_line(result: dict[str, object], side: str, line_type: str) -> dict[str, object]:
    side = side.strip().lower()
    line_type = line_type.strip().lower()
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    if line_type not in ("inner", "outer"):
        raise ValueError("line_type must be inner or outer")

    updated = dict(result)
    key_prefix = f"{side}_{line_type}"
    updated["selected_side"] = side
    updated["selected_line_type"] = line_type
    updated["selected_meridian_pixel"] = list(result.get(f"{key_prefix}_pixel", []))
    updated["selected_meridian_camera"] = list(result.get(f"{key_prefix}_camera", []))
    return updated


def select_topmost_bladder_line(result: dict[str, object]) -> dict[str, object]:
    candidates: list[tuple[float, str, str, int]] = []
    for side in ("left", "right"):
        for line_type in ("inner", "outer"):
            pixels = list(result.get(f"{side}_{line_type}_pixel", []))
            ys = [float(pt[1]) for pt in pixels if isinstance(pt, (list, tuple)) and len(pt) >= 2]
            if ys:
                candidates.append((float(sum(ys) / len(ys)), side, line_type, len(ys)))
    if not candidates:
        raise RuntimeError("no bladder meridian pixels available for topmost-line selection")

    avg_y, side, line_type, count = min(candidates, key=lambda item: item[0])
    updated = select_bladder_line(result, side, line_type)
    updated["selected_line_reason"] = {
        "mode": "topmost_pixel_y",
        "avg_y": float(avg_y),
        "sample_count": int(count),
    }
    return updated


def filter_selected_meridian_continuity(
    result: dict[str, object],
    *,
    max_step_m: float,
) -> dict[str, object]:
    if float(max_step_m) <= 0.0:
        return result
    points = list(result.get("selected_meridian_robot", []))
    if len(points) < 3:
        return result

    segments: list[list[int]] = [[0]]
    for idx in range(1, len(points)):
        prev = np.asarray(points[idx - 1][:3], dtype=np.float64)
        curr = np.asarray(points[idx][:3], dtype=np.float64)
        step_m = float(np.linalg.norm(curr - prev))
        if np.isfinite(step_m) and step_m <= float(max_step_m):
            segments[-1].append(idx)
        else:
            segments.append([idx])
    keep_indices = max(segments, key=len)
    if len(keep_indices) == len(points):
        return result
    if len(keep_indices) < 2:
        raise RuntimeError(
            "selected meridian continuity filter removed too many points: "
            f"kept={len(keep_indices)} original={len(points)} max_step_m={max_step_m}"
        )

    keep_set = set(keep_indices)
    updated = dict(result)

    def filter_field(name: str) -> None:
        values = list(updated.get(name, []))
        if len(values) == len(points):
            updated[name] = [values[idx] for idx in keep_indices]

    for field_name in (
        "selected_meridian_pixel",
        "selected_meridian_camera",
        "selected_meridian_robot",
        "selected_meridian_robot_pose_quat",
        "product_camera_waypoints",
        "product_camera_vectors",
    ):
        filter_field(field_name)

    side = str(updated.get("selected_side", "")).strip().lower()
    if side in ("left", "right"):
        for suffix in ("meridian_robot", "meridian_robot_pose_quat"):
            filter_field(f"{side}_{suffix}")

    updated["selected_meridian_filter"] = {
        "mode": "largest_contiguous_robot_segment",
        "max_step_m": float(max_step_m),
        "original_count": int(len(points)),
        "kept_count": int(len(keep_indices)),
        "removed_indices_1based": [idx + 1 for idx in range(len(points)) if idx not in keep_set],
    }
    return updated


def trim_selected_meridian_ends(
    result: dict[str, object],
    *,
    trim_count: int,
) -> dict[str, object]:
    count = int(trim_count)
    if count <= 0:
        return result
    points = list(result.get("selected_meridian_robot", []))
    if len(points) <= count * 2 + 1:
        raise RuntimeError(
            "cannot trim selected meridian ends: "
            f"trim_count={count} point_count={len(points)}"
        )

    updated = dict(result)
    keep_slice = slice(count, len(points) - count)

    def trim_field(name: str) -> None:
        values = list(updated.get(name, []))
        if len(values) == len(points):
            updated[name] = values[keep_slice]

    for field_name in (
        "selected_meridian_pixel",
        "selected_meridian_camera",
        "selected_meridian_robot",
        "selected_meridian_robot_pose_quat",
        "product_camera_waypoints",
        "product_camera_vectors",
    ):
        trim_field(field_name)

    side = str(updated.get("selected_side", "")).strip().lower()
    if side in ("left", "right"):
        for suffix in ("meridian_robot", "meridian_robot_pose_quat"):
            trim_field(f"{side}_{suffix}")

    updated["selected_meridian_trim"] = {
        "mode": "drop_both_ends",
        "trim_count_each_end": int(count),
        "original_count": int(len(points)),
        "kept_count": int(len(points) - count * 2),
    }
    return updated


def offset_selected_meridian_robot(
    result: dict[str, object],
    *,
    offset_m: list[float] | tuple[float, float, float],
) -> dict[str, object]:
    if len(offset_m) < 3:
        raise RuntimeError(f"offset_m must contain 3 values, got {len(offset_m)}")
    offset = [float(v) for v in list(offset_m)[:3]]
    if all(abs(v) < 1e-12 for v in offset):
        return result
    updated = dict(result)

    def offset_points(name: str) -> None:
        values = list(updated.get(name, []))
        out = []
        changed = False
        for value in values:
            if isinstance(value, (list, tuple)) and len(value) >= 3:
                row = [float(value[0]) + offset[0], float(value[1]) + offset[1], float(value[2]) + offset[2]]
                row.extend(float(v) for v in list(value)[3:])
                out.append(row)
                changed = True
            else:
                out.append(value)
        if changed:
            updated[name] = out

    offset_points("selected_meridian_robot")
    offset_points("selected_meridian_robot_pose_quat")
    side = str(updated.get("selected_side", "")).strip().lower()
    if side in ("left", "right"):
        offset_points(f"{side}_meridian_robot")
        offset_points(f"{side}_meridian_robot_pose_quat")
    updated["selected_meridian_robot_offset_m"] = offset
    return updated


def save_bladder_artifacts(
    output_dir: str,
    detection_result: dict[str, object],
    overlay_bgr: np.ndarray,
    prefix: str,
) -> tuple[str, str]:
    ensure_output_dir(output_dir)
    overlay_path = os.path.join(output_dir, f"{prefix}_overlay.png")
    json_path = os.path.join(output_dir, f"{prefix}_detect.json")
    cv2.imwrite(overlay_path, overlay_bgr)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(detection_result, f, ensure_ascii=False, indent=2)
    return overlay_path, json_path


def attach_selected_robot_points_static(result: dict[str, object], matrix: np.ndarray) -> dict[str, object]:
    updated = dict(result)
    selected_camera = list(updated.get("selected_meridian_camera", []))
    updated["selected_meridian_robot"] = transform_points(selected_camera, matrix)
    updated["robot_frame_unit"] = "meters"
    updated["transform_backend"] = "static"
    return updated


def attach_selected_robot_points_eye_in_hand(
    result: dict[str, object],
    tool_from_camera_matrix: np.ndarray,
    tool_pose_m: list[float],
) -> dict[str, object]:
    updated = dict(result)
    selected_camera = list(updated.get("selected_meridian_camera", []))
    updated["selected_meridian_robot"] = transform_points_eye_in_hand(
        selected_camera,
        tool_pose_m=tool_pose_m,
        tool_from_camera_matrix=tool_from_camera_matrix,
    )
    updated["robot_frame_unit"] = "meters"
    updated["transform_backend"] = "static_eye_in_hand"
    updated["tool_pose_m_for_transform"] = [float(v) for v in tool_pose_m[:6]]
    return updated


def _pick_evenly_indices(length: int, count: int) -> list[int]:
    if length < 1:
        raise RuntimeError("not enough bladder meridian points for plan generation")
    if count <= 1:
        return [length // 2]
    if count >= length:
        return list(range(length))
    out: list[int] = []
    for idx in range(count):
        ratio = idx / max(1, count - 1)
        out.append(int(round(ratio * (length - 1))))
    return out


def _quat_to_rpy(quat_xyzw: list[float]) -> list[float]:
    qx, qy, qz, qw = [float(v) for v in quat_xyzw[:4]]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [float(roll), float(pitch), float(yaw)]


def _quat_to_local_z_axis(quat_xyzw: list[float]) -> list[float]:
    qx, qy, qz, qw = [float(v) for v in quat_xyzw[:4]]
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    z_axis = [2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)]
    norm = math.sqrt(sum(v * v for v in z_axis))
    if norm <= 1e-8:
        return [0.0, 0.0, -1.0]
    return [float(v / norm) for v in z_axis]


def _quat_to_local_axes(quat_xyzw: list[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    qx, qy, qz, qw = [float(v) for v in quat_xyzw[:4]]
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    rot = np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    return rot[:, 0], rot[:, 1], rot[:, 2]


def _axis_vector(axis_name: str) -> np.ndarray:
    axes = {
        "pos_x": [1.0, 0.0, 0.0],
        "neg_x": [-1.0, 0.0, 0.0],
        "pos_y": [0.0, 1.0, 0.0],
        "neg_y": [0.0, -1.0, 0.0],
        "pos_z": [0.0, 0.0, 1.0],
        "neg_z": [0.0, 0.0, -1.0],
        "local_x": [1.0, 0.0, 0.0],
        "neg_local_x": [-1.0, 0.0, 0.0],
        "local_y": [0.0, 1.0, 0.0],
        "neg_local_y": [0.0, -1.0, 0.0],
        "local_z": [0.0, 0.0, 1.0],
        "neg_local_z": [0.0, 0.0, -1.0],
    }
    key = str(axis_name or "").strip().lower()
    if key not in axes:
        raise ValueError(f"unsupported axis: {axis_name}")
    return np.asarray(axes[key], dtype=np.float64)


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def _matrix_to_rpy(rot: np.ndarray) -> list[float]:
    m = np.asarray(rot, dtype=np.float64)
    sy = max(-1.0, min(1.0, -float(m[2, 0])))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) > 1e-6:
        roll = math.atan2(float(m[2, 1]), float(m[2, 2]))
        yaw = math.atan2(float(m[1, 0]), float(m[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(m[0, 1]), float(m[1, 1]))
    return [float(roll), float(pitch), float(yaw)]


def _align_axis_rotation(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    src = _normalize_vec(np.asarray(source, dtype=np.float64))
    dst = _normalize_vec(np.asarray(target, dtype=np.float64))
    if src is None or dst is None:
        raise RuntimeError("cannot align zero-length axes")
    cross = np.cross(src, dst)
    dot = max(-1.0, min(1.0, float(np.dot(src, dst))))
    sin = float(np.linalg.norm(cross))
    if sin < 1e-9:
        if dot > 0.0:
            return np.eye(3, dtype=np.float64)
        helper = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(src, helper))) > 0.9:
            helper = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        cross = _normalize_vec(np.cross(src, helper))
        if cross is None:
            raise RuntimeError("cannot build 180-degree axis correction")
        sin = 1.0
        dot = -1.0
    vx = np.asarray(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + vx + (vx @ vx) * ((1.0 - dot) / (sin * sin))


def _correct_rpy_for_tool_contact_axis(
    *,
    rpy: list[float],
    product_normal_axis: str,
    tool_contact_axis: str,
    press_direction: list[float],
) -> list[float]:
    rot = _rpy_to_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    normal_axis = _axis_vector(product_normal_axis)
    contact_axis = _axis_vector(tool_contact_axis)
    world_normal = rot @ normal_axis
    press = _normalize_vec(np.asarray(press_direction[:3], dtype=np.float64))
    if press is not None and float(np.dot(world_normal, press)) < 0.0:
        normal_axis = -normal_axis
    correction = _align_axis_rotation(contact_axis, normal_axis)
    return _matrix_to_rpy(rot @ correction)


def _align_rpy_tool_axis_to_world(
    rpy: list[float],
    tool_contact_axis: str,
    target_world_axis: list[float] | np.ndarray,
) -> list[float]:
    rot = _rpy_to_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    contact_axis = _axis_vector(tool_contact_axis)
    current_world_axis = rot @ contact_axis
    target = _normalize_vec(np.asarray(target_world_axis, dtype=np.float64))
    if target is None:
        raise RuntimeError("target world axis is invalid")
    correction = _align_axis_rotation(current_world_axis, target)
    return _matrix_to_rpy(correction @ rot)


def _press_direction_from_product_pose(
    quat_xyzw: list[float],
    point_m: list[float],
    anchor_pose_m: list[float],
    normal_axis: str = "neg_local_z",
) -> list[float]:
    """Return the inward press direction from product waypoint orientation.

    The sign is oriented so the hover point moves from the body surface toward
    the camera/arm side in the horizontal plane, while press/dian-jin moves
    back toward the body.
    """
    local_x, local_y, local_z = _quat_to_local_axes(quat_xyzw)
    axes = {
        "local_x": local_x,
        "local_y": local_y,
        "local_z": local_z,
        "neg_local_x": -local_x,
        "neg_local_y": -local_y,
        "neg_local_z": -local_z,
    }
    axis_key = str(normal_axis or "local_x").strip().lower()
    if axis_key not in axes:
        raise ValueError(f"unsupported product normal axis: {normal_axis}")

    outward = _normalize_vec(axes[axis_key])
    if outward is None:
        return _quat_to_local_z_axis(quat_xyzw)

    anchor_xyz = np.asarray([float(v) for v in anchor_pose_m[:3]], dtype=np.float64)
    point_xyz = np.asarray([float(v) for v in point_m[:3]], dtype=np.float64)
    toward_anchor = anchor_xyz - point_xyz
    if np.linalg.norm(toward_anchor) > 1e-8:
        if float(np.dot(outward, toward_anchor)) < 0.0:
            outward = -outward

    inward = -outward
    return [float(v) for v in inward.tolist()]


def _build_split_axis(press_direction: list[float], tangent: list[float]) -> list[float]:
    split_axis = _normalize_vec(np.cross(np.asarray(press_direction, dtype=np.float64), np.asarray(tangent, dtype=np.float64)))
    if split_axis is not None:
        return [float(v) for v in split_axis]
    for fallback in (
        np.array([0.0, 0.0, 1.0], dtype=np.float64),
        np.array([0.0, 1.0, 0.0], dtype=np.float64),
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
    ):
        split_axis = _normalize_vec(np.cross(np.asarray(press_direction, dtype=np.float64), fallback))
        if split_axis is not None:
            return [float(v) for v in split_axis]
    return [0.0, 1.0, 0.0]


def _build_pose_from_frame(
    point_m: list[float],
    base_pose_m: list[float],
    press_direction_m: list[float],
    split_axis_m: list[float],
    tool_offset_m: float = 0.0,
    split_offset_m: float = 0.0,
) -> list[float]:
    point = np.asarray(point_m[:3], dtype=np.float64)
    press_direction = np.asarray(press_direction_m[:3], dtype=np.float64)
    split_axis = np.asarray(split_axis_m[:3], dtype=np.float64)
    pos = point + press_direction * float(tool_offset_m) + split_axis * float(split_offset_m)
    return [
        float(pos[0]),
        float(pos[1]),
        float(pos[2]),
        float(base_pose_m[3]),
        float(base_pose_m[4]),
        float(base_pose_m[5]),
    ]


def build_bladder_massage_plan(
    *,
    side: str,
    line_type: str,
    meridian_points_robot_m: list[list[float]],
    meridian_pixels: list[list[int]] | None,
    anchor_pose_m: list[float],
    point_count: int,
    hover_m: float,
    dian_jin_depth_m: float,
    fen_jin_lateral_m: float,
    safe_lift_m: float,
    meridian_pose_quat: list[list[float]] | None = None,
    product_normal_axis: str = "neg_local_z",
    tool_contact_axis: str = "",
    start_nearest_anchor: bool = False,
    hover_offset_mode: str = "normal",
) -> BladderMassagePlan:
    if hover_m <= 0.0:
        raise RuntimeError("hover_m must be > 0")
    if dian_jin_depth_m < 0.0 or dian_jin_depth_m >= hover_m:
        raise RuntimeError("dian_jin_depth_m must be >= 0 and < hover_m")
    if len(meridian_points_robot_m) < 2:
        raise RuntimeError("selected_meridian_robot has insufficient valid points")

    pick_indices = _pick_evenly_indices(len(meridian_points_robot_m), max(1, int(point_count)))
    selected_points = [[float(v) for v in meridian_points_robot_m[src]] for src in pick_indices]
    selected_pixels: list[list[float]] = []
    if meridian_pixels and len(meridian_pixels) == len(meridian_points_robot_m):
        selected_pixels = [[float(v) for v in meridian_pixels[src][:2]] for src in pick_indices]
    else:
        selected_pixels = [[0.0, 0.0] for _ in selected_points]

    selected_pose_quat: list[list[float]] = []
    if meridian_pose_quat and len(meridian_pose_quat) == len(meridian_points_robot_m):
        selected_pose_quat = [[float(v) for v in meridian_pose_quat[src]] for src in pick_indices]

    if start_nearest_anchor and len(selected_points) >= 2:
        anchor_xyz = np.asarray(anchor_pose_m[:3], dtype=np.float64)
        first_dist = float(np.linalg.norm(np.asarray(selected_points[0][:3], dtype=np.float64) - anchor_xyz))
        last_dist = float(np.linalg.norm(np.asarray(selected_points[-1][:3], dtype=np.float64) - anchor_xyz))
        if last_dist < first_dist:
            selected_points.reverse()
            selected_pixels.reverse()
            selected_pose_quat.reverse()

    hover_offset_mode = str(hover_offset_mode or "normal").strip().lower()
    if hover_offset_mode not in ("normal", "base_z"):
        raise ValueError(f"unsupported hover_offset_mode: {hover_offset_mode}")

    anchor_rpy = [float(v) for v in anchor_pose_m[3:6]]
    frames: list[BladderMassageFrame] = []
    safe_candidates = [float(anchor_pose_m[2]) + float(safe_lift_m)]
    prev_split_axis: np.ndarray | None = None

    for idx, point in enumerate(selected_points):
        prev_point = np.asarray(selected_points[max(0, idx - 1)], dtype=np.float64)
        next_point = np.asarray(selected_points[min(len(selected_points) - 1, idx + 1)], dtype=np.float64)
        tangent = _normalize_vec(next_point - prev_point)
        if tangent is None:
            tangent = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)

        pose_quat = selected_pose_quat[idx] if idx < len(selected_pose_quat) else []
        if pose_quat and len(pose_quat) >= 7:
            rpy = _quat_to_rpy(pose_quat[3:7])
            press_direction = _press_direction_from_product_pose(
                pose_quat[3:7],
                point,
                anchor_pose_m,
                normal_axis=product_normal_axis,
            )
            if str(tool_contact_axis or "").strip():
                rpy = _correct_rpy_for_tool_contact_axis(
                    rpy=rpy,
                    product_normal_axis=product_normal_axis,
                    tool_contact_axis=tool_contact_axis,
                    press_direction=press_direction,
                )
        else:
            rpy = list(anchor_rpy)
            press_direction = [0.0, 0.0, -1.0]

        split_axis = np.asarray(_build_split_axis(press_direction, tangent.tolist()), dtype=np.float64)
        if prev_split_axis is not None and float(np.dot(split_axis, prev_split_axis)) < 0.0:
            split_axis = -split_axis
        prev_split_axis = split_axis

        base_pose = [float(point[0]), float(point[1]), float(point[2]), float(rpy[0]), float(rpy[1]), float(rpy[2])]
        if hover_offset_mode == "base_z":
            hover_pose = [
                float(point[0]),
                float(point[1]),
                float(point[2] + hover_m),
                float(base_pose[3]),
                float(base_pose[4]),
                float(base_pose[5]),
            ]
            dian_jin_pose = [
                float(point[0]),
                float(point[1]),
                float(point[2] + max(0.0, hover_m - dian_jin_depth_m)),
                float(base_pose[3]),
                float(base_pose[4]),
                float(base_pose[5]),
            ]
        else:
            hover_pose = _build_pose_from_frame(
                point_m=point,
                base_pose_m=base_pose,
                press_direction_m=press_direction,
                split_axis_m=split_axis.tolist(),
                tool_offset_m=-hover_m,
            )
            dian_jin_pose = _build_pose_from_frame(
                point_m=point,
                base_pose_m=base_pose,
                press_direction_m=press_direction,
                split_axis_m=split_axis.tolist(),
                tool_offset_m=-(hover_m - dian_jin_depth_m),
            )
        fen_positive_pose = _build_pose_from_frame(
            point_m=point,
            base_pose_m=base_pose,
            press_direction_m=press_direction,
            split_axis_m=split_axis.tolist(),
            tool_offset_m=-hover_m,
            split_offset_m=fen_jin_lateral_m,
        )
        fen_negative_pose = _build_pose_from_frame(
            point_m=point,
            base_pose_m=base_pose,
            press_direction_m=press_direction,
            split_axis_m=split_axis.tolist(),
            tool_offset_m=-hover_m,
            split_offset_m=-fen_jin_lateral_m,
        )
        safe_candidates.append(float(hover_pose[2]) + 0.01)
        frames.append(
            BladderMassageFrame(
                index=idx + 1,
                pixel=[float(selected_pixels[idx][0]), float(selected_pixels[idx][1])],
                robot_point_m=[float(v) for v in point],
                hover_pose_m=hover_pose,
                dian_jin_pose_m=dian_jin_pose,
                fen_positive_pose_m=fen_positive_pose,
                fen_negative_pose_m=fen_negative_pose,
                press_direction_m=[float(v) for v in press_direction],
                split_axis_m=[float(v) for v in split_axis.tolist()],
                tangent_axis_m=[float(v) for v in tangent.tolist()],
                base_pose_m=base_pose,
                source_pose_quat=list(pose_quat),
            )
        )

    safe_z_m = max(safe_candidates)
    return BladderMassagePlan(
        side=str(side),
        line_type=str(line_type),
        point_count=len(frames),
        hover_m=float(hover_m),
        dian_jin_depth_m=float(dian_jin_depth_m),
        fen_jin_lateral_m=float(fen_jin_lateral_m),
        safe_z_m=float(safe_z_m),
        anchor_pose_m=[float(v) for v in anchor_pose_m[:6]],
        frames=frames,
        hover_offset_mode=hover_offset_mode,
    )


def bladder_plan_to_dict(plan: BladderMassagePlan) -> dict[str, object]:
    data = asdict(plan)
    data["bbox_robot_m"] = {
        axis: [min(p.robot_point_m[idx] for p in plan.frames), max(p.robot_point_m[idx] for p in plan.frames)]
        for idx, axis in enumerate(("x", "y", "z"))
    }
    return data


def build_aligned_contact_preview(
    plan: BladderMassagePlan,
    *,
    tool_contact_axis: str = "pos_z",
    contact_motion_axis: str | None = None,
    max_press_m: float = 0.0,
    touch_step_m: float = 0.0,
    probe_depth_m: float | None = None,
) -> dict[str, object]:
    """Summarize the exact contact targets derived from the same hover plan."""
    axis_name = str(tool_contact_axis or "pos_z").strip().lower()
    tool_axis_local = _axis_vector(axis_name)
    motion_axis_name = str(contact_motion_axis or axis_name).strip().lower()
    motion_axis_local = _axis_vector(motion_axis_name)
    extra_press_m = max(0.0, float(max_press_m))
    step_m = max(0.0, float(touch_step_m))
    depth_override_m = None if probe_depth_m is None or float(probe_depth_m) <= 0.0 else float(probe_depth_m)
    frames: list[dict[str, object]] = []
    min_dot = 1.0

    for frame in plan.frames:
        press = _normalize_vec(np.asarray(frame.press_direction_m[:3], dtype=np.float64))
        if press is None:
            raise RuntimeError(f"point {frame.index} press_direction is invalid")
        hover = np.asarray(frame.hover_pose_m[:3], dtype=np.float64)
        rot = _rpy_to_matrix(*frame.hover_pose_m[3:6])
        surface = hover + press * float(plan.hover_m)
        tool_axis_world = _normalize_vec(rot @ tool_axis_local)
        motion_axis_world = _normalize_vec(rot @ motion_axis_local)
        if motion_axis_world is None:
            raise RuntimeError(f"point {frame.index} contact motion axis is invalid")
        approach_surface = hover + motion_axis_world * float(plan.hover_m)
        total_depth_m = depth_override_m if depth_override_m is not None else float(plan.hover_m) + extra_press_m
        contact_target = hover + motion_axis_world * total_depth_m
        dot = None
        if tool_axis_world is not None:
            dot = float(np.dot(tool_axis_world, press))
            min_dot = min(min_dot, dot)
        motion_dot = float(np.dot(motion_axis_world, press))
        frames.append(
            {
                "index": int(frame.index),
                "hover_pose_m": [float(v) for v in frame.hover_pose_m[:6]],
                "visual_surface_xyz_m": [float(v) for v in surface.tolist()],
                "approach_surface_xyz_m": [float(v) for v in approach_surface.tolist()],
                "contact_target_pose_m": [
                    float(contact_target[0]),
                    float(contact_target[1]),
                    float(contact_target[2]),
                    float(frame.hover_pose_m[3]),
                    float(frame.hover_pose_m[4]),
                    float(frame.hover_pose_m[5]),
                ],
                "press_direction_m": [float(v) for v in press.tolist()],
                "tool_axis_world_m": None if tool_axis_world is None else [float(v) for v in tool_axis_world.tolist()],
                "contact_motion_axis_world_m": [float(v) for v in motion_axis_world.tolist()],
                "tool_axis_dot_press": dot,
                "motion_axis_dot_press": motion_dot,
                "hover_to_surface_m": float(plan.hover_m),
                "surface_overtravel_m": extra_press_m,
                "hover_to_contact_target_m": total_depth_m,
                "touch_steps": None if step_m <= 0.0 else int(math.ceil(total_depth_m / step_m)),
            }
        )

    return {
        "source": "same BladderMassagePlan as hover_path",
        "tool_contact_axis": axis_name,
        "contact_motion_axis": motion_axis_name,
        "hover_m": float(plan.hover_m),
        "max_press_m": extra_press_m,
        "touch_step_m": step_m,
        "probe_depth_m": depth_override_m,
        "min_tool_axis_dot_press": float(min_dot),
        "frames": frames,
    }


def validate_aligned_contact_preview(
    plan: BladderMassagePlan,
    *,
    tool_contact_axis: str = "pos_z",
    contact_motion_axis: str | None = None,
    max_press_m: float = 0.0,
    touch_step_m: float = 0.0,
    probe_depth_m: float | None = None,
    min_axis_dot: float = 0.985,
) -> dict[str, object]:
    preview = build_aligned_contact_preview(
        plan,
        tool_contact_axis=tool_contact_axis,
        contact_motion_axis=contact_motion_axis,
        max_press_m=max_press_m,
        touch_step_m=touch_step_m,
        probe_depth_m=probe_depth_m,
    )
    dot = float(preview["min_tool_axis_dot_press"])
    if dot < float(min_axis_dot):
        raise RuntimeError(
            f"tool contact axis {tool_contact_axis} is not aligned with press direction: "
            f"min_dot={dot:.4f} required>={float(min_axis_dot):.4f}"
        )
    return preview


def rebuild_plan_with_fixed_first_normal(
    plan: BladderMassagePlan,
    source_index: int = 0,
    *,
    press_direction_override: list[float] | None = None,
    tool_contact_axis: str = "pos_x",
) -> BladderMassagePlan:
    """Use one stable side-lying surface normal and tool orientation for all sampled points."""
    if not plan.frames:
        return plan
    ref_idx = min(max(0, int(source_index)), len(plan.frames) - 1)
    ref = plan.frames[ref_idx]
    source_press = press_direction_override if press_direction_override is not None else ref.press_direction_m[:3]
    press = _normalize_vec(np.asarray(source_press, dtype=np.float64))
    if press is None:
        raise RuntimeError("fixed-first-normal requires a valid reference press_direction")
    if len(ref.base_pose_m) >= 6:
        ref_rpy = [float(v) for v in ref.base_pose_m[3:6]]
    else:
        ref_rpy = [float(v) for v in ref.hover_pose_m[3:6]]
    if press_direction_override is not None:
        ref_rpy = _align_rpy_tool_axis_to_world(ref_rpy, tool_contact_axis, press)

    frames: list[BladderMassageFrame] = []
    safe_candidates = [float(plan.safe_z_m)]
    prev_split_axis: np.ndarray | None = None
    for frame in plan.frames:
        point = [float(v) for v in frame.robot_point_m[:3]]
        tangent = _normalize_vec(np.asarray(frame.tangent_axis_m[:3], dtype=np.float64))
        if tangent is None:
            tangent = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        split_axis = np.asarray(_build_split_axis(press.tolist(), tangent.tolist()), dtype=np.float64)
        if prev_split_axis is not None and float(np.dot(split_axis, prev_split_axis)) < 0.0:
            split_axis = -split_axis
        prev_split_axis = split_axis

        base_pose = [float(point[0]), float(point[1]), float(point[2]), *ref_rpy]
        if plan.hover_offset_mode == "base_z":
            hover_pose = [
                float(point[0]),
                float(point[1]),
                float(point[2] + plan.hover_m),
                *ref_rpy,
            ]
            dian_jin_pose = [
                float(point[0]),
                float(point[1]),
                float(point[2] + max(0.0, plan.hover_m - plan.dian_jin_depth_m)),
                *ref_rpy,
            ]
        else:
            hover_pose = _build_pose_from_frame(
                point_m=point,
                base_pose_m=base_pose,
                press_direction_m=press.tolist(),
                split_axis_m=split_axis.tolist(),
                tool_offset_m=-plan.hover_m,
            )
            dian_jin_pose = _build_pose_from_frame(
                point_m=point,
                base_pose_m=base_pose,
                press_direction_m=press.tolist(),
                split_axis_m=split_axis.tolist(),
                tool_offset_m=-(plan.hover_m - plan.dian_jin_depth_m),
            )
        fen_positive_pose = _build_pose_from_frame(
            point_m=point,
            base_pose_m=base_pose,
            press_direction_m=press.tolist(),
            split_axis_m=split_axis.tolist(),
            tool_offset_m=-plan.hover_m,
            split_offset_m=plan.fen_jin_lateral_m,
        )
        fen_negative_pose = _build_pose_from_frame(
            point_m=point,
            base_pose_m=base_pose,
            press_direction_m=press.tolist(),
            split_axis_m=split_axis.tolist(),
            tool_offset_m=-plan.hover_m,
            split_offset_m=-plan.fen_jin_lateral_m,
        )
        safe_candidates.append(float(hover_pose[2]) + 0.01)
        frames.append(
            BladderMassageFrame(
                index=int(frame.index),
                pixel=[float(v) for v in frame.pixel[:2]],
                robot_point_m=point,
                hover_pose_m=hover_pose,
                dian_jin_pose_m=dian_jin_pose,
                fen_positive_pose_m=fen_positive_pose,
                fen_negative_pose_m=fen_negative_pose,
                press_direction_m=[float(v) for v in press.tolist()],
                split_axis_m=[float(v) for v in split_axis.tolist()],
                tangent_axis_m=[float(v) for v in tangent.tolist()],
                base_pose_m=base_pose,
                source_pose_quat=list(frame.source_pose_quat),
            )
        )

    return BladderMassagePlan(
        side=plan.side,
        line_type=plan.line_type,
        point_count=len(frames),
        hover_m=float(plan.hover_m),
        dian_jin_depth_m=float(plan.dian_jin_depth_m),
        fen_jin_lateral_m=float(plan.fen_jin_lateral_m),
        safe_z_m=float(max(safe_candidates)),
        anchor_pose_m=list(plan.anchor_pose_m),
        frames=frames,
        hover_offset_mode=plan.hover_offset_mode,
    )


def rebuild_plan_with_horizontal_press(
    plan: BladderMassagePlan,
    *,
    source_index: int = 0,
    tool_contact_axis: str = "pos_x",
) -> BladderMassagePlan:
    """Project side-lying contact motion onto Base-XY to avoid upward/downward probing."""
    if not plan.frames:
        return plan
    ref_idx = min(max(0, int(source_index)), len(plan.frames) - 1)
    press = np.asarray(plan.frames[ref_idx].press_direction_m[:3], dtype=np.float64)
    press[2] = 0.0
    projected = _normalize_vec(press)
    if projected is None:
        raise RuntimeError("cannot project press direction to horizontal plane")
    return rebuild_plan_with_fixed_first_normal(
        plan,
        source_index=source_index,
        press_direction_override=[float(v) for v in projected.tolist()],
        tool_contact_axis=tool_contact_axis,
    )


def preview_bladder_plan(plan: BladderMassagePlan) -> None:
    print(
        f"side={plan.side} line_type={plan.line_type} points={plan.point_count} "
        f"hover_m={plan.hover_m:.4f} dian_jin_depth_m={plan.dian_jin_depth_m:.4f} "
        f"fen_jin_lateral_m={plan.fen_jin_lateral_m:.4f} safe_z_m={plan.safe_z_m:.4f} "
        f"hover_offset_mode={plan.hover_offset_mode}"
    )
    for frame in plan.frames:
        print(
            f"{frame.index:02d}: robot={frame.robot_point_m} "
            f"pixel={frame.pixel} "
            f"hover={frame.hover_pose_m[:3]} dian={frame.dian_jin_pose_m[:3]} "
            f"press_dir={[round(v, 4) for v in frame.press_direction_m]}"
        )


def _lift_to_safe_z(arm, host: str, pose: list[float], safe_z_m: float, speed) -> list[float]:
    """If pose[2] < safe_z_m, movel straight up so we stop hovering over (or pressing into) the body."""
    if float(pose[2]) >= float(safe_z_m):
        return list(pose)
    lift_pose = [
        float(pose[0]),
        float(pose[1]),
        float(safe_z_m),
        float(pose[3]),
        float(pose[4]),
        float(pose[5]),
    ]
    arm.movel(host, lift_pose, speed=speed, timeout=20.0)
    return lift_pose


def _goto_hover_via_safe(arm, host: str, hover_pose: list[float], safe_z_m: float, speed) -> None:
    """Reach hover_pose without cutting a diagonal line across the body:
    lift current to safe_z -> horizontal move at safe_z above hover xy -> descend to hover."""
    try:
        _, current_pose, _, _, _ = arm.get_current_arm_state(host)
    except Exception:
        current_pose = list(hover_pose)
    _lift_to_safe_z(arm, host, current_pose, safe_z_m, speed)
    waypoint = [
        float(hover_pose[0]),
        float(hover_pose[1]),
        float(max(float(safe_z_m), float(hover_pose[2]))),
        float(hover_pose[3]),
        float(hover_pose[4]),
        float(hover_pose[5]),
    ]
    arm.movel(host, waypoint, speed=speed, timeout=20.0)
    arm.movel(host, list(hover_pose), speed=speed, timeout=20.0)


def _emergency_retreat(arm, host: str, safe_z_m: float, speed) -> None:
    """Best-effort: stop any queued motion and lift straight up above safe_z_m."""
    try:
        arm.stop_motion(host)
    except Exception as exc:
        print(f"emergency_retreat: stop_motion failed: {exc}")
    try:
        _, pose, _, _, _ = arm.get_current_arm_state(host)
        _lift_to_safe_z(arm, host, pose, safe_z_m, speed)
    except Exception as exc:
        print(f"emergency_retreat: lift failed: {exc}")


def _move_pose_segmented(
    arm,
    host: str,
    target_pose: list[float],
    speed,
    *,
    max_step_m: float = 0.02,
    keep_current_orientation: bool = False,
) -> None:
    _, start_pose, _, _, _ = arm.get_current_arm_state(host)
    target = [float(v) for v in target_pose[:6]]
    if keep_current_orientation:
        target[3:6] = [float(v) for v in start_pose[3:6]]
    dist_m = float(np.linalg.norm(np.asarray(target[:3], dtype=np.float64) - np.asarray(start_pose[:3], dtype=np.float64)))
    step_count = max(1, int(math.ceil(dist_m / max(1e-6, float(max_step_m)))))
    for step_idx in range(1, step_count + 1):
        ratio = float(step_idx) / float(step_count)
        step_pose = list(target)
        step_pose[:3] = [
            float(start_pose[i] + (target[i] - start_pose[i]) * ratio)
            for i in range(3)
        ]
        if not keep_current_orientation:
            step_pose[3:6] = [
                _lerp_angle_rad(start_pose[i], target[i], ratio)
                for i in range(3, 6)
            ]
        arm.movel(host, step_pose, speed=speed, timeout=20.0)


def _force_delta_n(sample: dict[str, float] | None, baseline: dict[str, float] | None) -> float | None:
    if sample is None or baseline is None:
        return None
    dfx = float(sample["fx"] - baseline["fx"])
    dfy = float(sample["fy"] - baseline["fy"])
    dfz = float(sample["fz"] - baseline["fz"])
    return float(math.sqrt(dfx * dfx + dfy * dfy + dfz * dfz))


def _format_force_sample(sample: dict[str, float] | None) -> str:
    if sample is None:
        return "force=n/a"
    return (
        f"Fx={sample['fx']:.2f} Fy={sample['fy']:.2f} Fz={sample['fz']:.2f} "
        f"Mx={sample['mx']:.2f} My={sample['my']:.2f} Mz={sample['mz']:.2f}"
    )


def _average_force_samples(bridge, *, count: int = 3, timeout: float = 0.8, interval_s: float = 0.04) -> dict[str, float] | None:
    samples: list[dict[str, float]] = []
    for _ in range(max(1, int(count))):
        sample = bridge.request_force_sample(timeout=timeout)
        if sample is not None:
            samples.append(sample)
        time.sleep(max(0.0, float(interval_s)))
    if not samples:
        return None
    keys = ("fx", "fy", "fz", "mx", "my", "mz")
    return {key: float(sum(sample[key] for sample in samples) / len(samples)) for key in keys}


def _lerp_angle_rad(start: float, target: float, ratio: float) -> float:
    delta = (float(target) - float(start) + math.pi) % (2.0 * math.pi) - math.pi
    return float(start) + delta * float(ratio)


def execute_bladder_plan(
    *,
    host: str,
    plan: BladderMassagePlan,
    speed: int,
    control_backend: str = "json",
    dian_jin_dwell_s: float = 0.5,
    fen_jin_dwell_s: float = 0.3,
    shun_jin_dwell_s: float = 0.0,
) -> None:
    arm = create_arm_backend(control_backend)
    arm.recover_if_needed(host)
    motion_speed = normalize_motion_speed(control_backend, float(speed), ros_default=0.3)
    _, current_pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(host)
    print(
        f"current_pose={[round(v, 6) for v in current_pose]} "
        f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
    )

    try:
        _lift_to_safe_z(arm, host, current_pose, plan.safe_z_m, motion_speed)
        _goto_hover_via_safe(arm, host, plan.frames[0].hover_pose_m, plan.safe_z_m, motion_speed)

        for idx, frame in enumerate(plan.frames):
            print(f"point {frame.index}/{plan.point_count} hover={frame.hover_pose_m[:3]}")
            if idx > 0:
                _goto_hover_via_safe(arm, host, frame.hover_pose_m, plan.safe_z_m, motion_speed)
            arm.movel(host, frame.dian_jin_pose_m, speed=motion_speed, timeout=20.0)
            time.sleep(max(0.0, float(dian_jin_dwell_s)))
            arm.movel(host, frame.hover_pose_m, speed=motion_speed, timeout=20.0)

            arm.movel(host, frame.fen_positive_pose_m, speed=motion_speed, timeout=20.0)
            time.sleep(max(0.0, float(fen_jin_dwell_s)))
            arm.movel(host, frame.hover_pose_m, speed=motion_speed, timeout=20.0)
            arm.movel(host, frame.fen_negative_pose_m, speed=motion_speed, timeout=20.0)
            time.sleep(max(0.0, float(fen_jin_dwell_s)))
            arm.movel(host, frame.hover_pose_m, speed=motion_speed, timeout=20.0)

        print("shun_jin start")
        _goto_hover_via_safe(arm, host, plan.frames[0].hover_pose_m, plan.safe_z_m, motion_speed)
        for frame in plan.frames[1:]:
            _goto_hover_via_safe(arm, host, frame.hover_pose_m, plan.safe_z_m, motion_speed)
            time.sleep(max(0.0, float(shun_jin_dwell_s)))

        _, final_pose, _, _, _ = arm.get_current_arm_state(host)
        _lift_to_safe_z(arm, host, final_pose, plan.safe_z_m, motion_speed)
    except BaseException as exc:
        print(f"execute_bladder_plan aborted: {type(exc).__name__}: {exc}")
        _emergency_retreat(arm, host, plan.safe_z_m, motion_speed)
        raise


def execute_bladder_touch_probe_plan(
    *,
    host: str,
    plan: BladderMassagePlan,
    speed: int,
    control_backend: str = "json",
    target_force_n: float = 2.0,
    max_force_n: float = 6.0,
    touch_step_m: float = 0.002,
    max_press_m: float = 0.01,
    dwell_s: float = 0.2,
    max_step_m: float = 0.02,
    keep_current_orientation: bool = False,
    entry_motion: str = "movej_p",
    tool_contact_axis: str = "pos_z",
    contact_motion_axis: str | None = None,
    probe_depth_m: float | None = None,
    min_axis_dot: float = 0.985,
) -> None:
    """Probe selected side-lying bladder points from hover until force threshold.

    This mode deliberately does not enable closed-loop force motion. It is a
    conservative validation step: move along the per-frame body normal, stop as
    soon as force rises, dwell briefly, and retreat to the same hover pose.
    """
    from .rm_execute import RosForceBridge

    if str(control_backend).strip().lower() != "ros":
        raise RuntimeError("side-lying touch_probe requires ROS Cartesian motion; JSON pose motion is not safe here")
    arm = create_arm_backend(control_backend)
    arm.recover_if_needed(host)
    motion_speed = normalize_motion_speed(control_backend, float(speed), ros_default=0.15)
    bridge: RosForceBridge | None = None
    entry_motion = str(entry_motion or "movej_p").strip().lower()
    if entry_motion not in ("movej_p", "movel"):
        raise ValueError(f"unsupported touch_probe entry_motion: {entry_motion}")
    contact_preview = validate_aligned_contact_preview(
        plan,
        tool_contact_axis=tool_contact_axis,
        contact_motion_axis=contact_motion_axis,
        max_press_m=max_press_m,
        touch_step_m=touch_step_m,
        probe_depth_m=probe_depth_m,
        min_axis_dot=min_axis_dot,
    )
    contact_frames = list(contact_preview.get("frames", []))
    print(
        "touch_probe aligned with hover plan: "
        f"tool_axis={contact_preview['tool_contact_axis']} "
        f"motion_axis={contact_preview['contact_motion_axis']} "
        f"min_tool_axis_dot_press={float(contact_preview['min_tool_axis_dot_press']):.4f} "
        f"hover_m={float(contact_preview['hover_m']):.4f} "
        f"max_press_m={float(contact_preview['max_press_m']):.4f} "
        f"probe_depth_m={contact_preview['probe_depth_m']}"
    )
    last_hover_pose: list[float] | None = None
    try:
        for idx, frame in enumerate(plan.frames, start=1):
            hover_pose = list(frame.hover_pose_m)
            last_hover_pose = hover_pose
            print(f"touch_probe point {idx}/{plan.point_count} hover={hover_pose[:3]}")
            contact_info = contact_frames[idx - 1] if idx - 1 < len(contact_frames) else {}
            if contact_info:
                print(
                    f"touch_probe point {idx} visual_surface={contact_info['visual_surface_xyz_m']} "
                    f"approach_surface={contact_info['approach_surface_xyz_m']} "
                    f"contact_target={contact_info['contact_target_pose_m'][:3]} "
                    f"tool_axis_dot_press={float(contact_info['tool_axis_dot_press']):.4f} "
                    f"motion_axis_dot_press={float(contact_info['motion_axis_dot_press']):.4f}"
                )
            if idx == 1 and entry_motion == "movej_p":
                entry_arm = create_arm_backend("ros")
                entry_arm.recover_if_needed(host)
                if not hasattr(entry_arm, "movej_p"):
                    raise RuntimeError("ROS entry backend does not support movej_p")
                entry_speed = normalize_motion_speed("ros", float(speed), ros_default=0.12)
                entry_arm.movej_p(host, hover_pose, speed=entry_speed, timeout=60.0)
            else:
                _move_pose_segmented(
                    arm,
                    host,
                    hover_pose,
                    motion_speed,
                    max_step_m=max_step_m,
                    keep_current_orientation=keep_current_orientation,
                )
            if bridge is None:
                bridge = RosForceBridge()
                bridge.enable_force_sensor()
            time.sleep(0.2)
            baseline = _average_force_samples(bridge, count=5, timeout=1.0, interval_s=0.05)
            if baseline is None:
                raise RuntimeError("force baseline unavailable; refusing to probe contact")
            print(f"touch_probe point {idx} baseline {_format_force_sample(baseline)}")

            press_direction = _normalize_vec(np.asarray(frame.press_direction_m[:3], dtype=np.float64))
            if press_direction is None:
                raise RuntimeError(f"point {idx} press_direction is invalid")
            if contact_info:
                motion_direction = _normalize_vec(np.asarray(contact_info["contact_motion_axis_world_m"][:3], dtype=np.float64))
                if motion_direction is not None:
                    press_direction = motion_direction
            probe_depth_m = float(contact_info.get("hover_to_contact_target_m", float(plan.hover_m) + max(0.0, float(max_press_m))))
            step_m = max(0.0005, float(touch_step_m))
            step_count = max(1, int(math.ceil(probe_depth_m / step_m)))
            reached = False

            for step_idx in range(1, step_count + 1):
                offset_m = min(probe_depth_m, float(step_idx) * step_m)
                probe_pose = list(hover_pose)
                for axis in range(3):
                    probe_pose[axis] = float(hover_pose[axis]) + float(press_direction[axis]) * offset_m
                arm.movel(host, probe_pose, speed=motion_speed, timeout=20.0)
                time.sleep(0.08)
                sample = _average_force_samples(bridge, count=3, timeout=0.8, interval_s=0.04)
                delta_n = _force_delta_n(sample, baseline)
                delta_text = "n/a" if delta_n is None else f"{delta_n:.2f}N"
                print(
                    f"touch_probe point {idx} step {step_idx}/{step_count} "
                    f"offset_m={offset_m:.4f} delta={delta_text} {_format_force_sample(sample)}"
                )
                if delta_n is None:
                    continue
                if delta_n >= float(max_force_n):
                    try:
                        arm.movel(host, hover_pose, speed=motion_speed, timeout=20.0)
                    except Exception as retreat_exc:
                        print(f"touch_probe point {idx} max-force retreat failed: {retreat_exc}")
                    raise RuntimeError(
                        f"point {idx} exceeded max force: delta={delta_n:.2f}N max={float(max_force_n):.2f}N"
                    )
                if delta_n >= float(target_force_n):
                    reached = True
                    print(f"touch_probe point {idx} target reached: delta={delta_n:.2f}N")
                    break

            if reached and dwell_s > 0.0:
                deadline = time.time() + float(dwell_s)
                while time.time() < deadline:
                    sample = bridge.request_force_sample(timeout=0.5)
                    delta_n = _force_delta_n(sample, baseline)
                    delta_text = "n/a" if delta_n is None else f"{delta_n:.2f}N"
                    print(f"touch_probe point {idx} dwell delta={delta_text} {_format_force_sample(sample)}")
                    time.sleep(0.1)
            elif not reached:
                print(f"touch_probe point {idx} target force not reached within probe depth; retreating")

            _move_pose_segmented(
                arm,
                host,
                hover_pose,
                motion_speed,
                max_step_m=max_step_m,
                keep_current_orientation=keep_current_orientation,
            )
    except BaseException as exc:
        print(f"execute_bladder_touch_probe_plan aborted: {type(exc).__name__}: {exc}")
        try:
            arm.stop_motion(host)
        except Exception as stop_exc:
            print(f"touch_probe stop failed: {stop_exc}")
        try:
            create_arm_backend("ros").stop_motion(host)
        except Exception as stop_exc:
            print(f"touch_probe ros stop failed: {stop_exc}")
        raise


def _request_force_sample_via_docker(
    *,
    container: str = "noetic",
    timeout: float = 3.0,
) -> dict[str, float] | None:
    import subprocess

    code = r'''
import json
import sys

sys.path.insert(0, "/home/franka/massage/robots/realman")
sys.path.insert(0, "/home/franka/massage/robots/realman/ros_vendor/python")

from rm_demo.rm_execute import RosForceBridge

bridge = RosForceBridge()
bridge.enable_force_sensor()
sample = bridge.request_force_sample(timeout=1.2)
print("##FORCE_JSON##" + json.dumps(sample, ensure_ascii=False))
'''
    cmd = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        "export PYTHONPATH=/home/franka/massage/robots/realman/ros_vendor/python:$PYTHONPATH; "
        "export ROS_MASTER_URI=http://192.168.1.11:11311; "
        "export ROS_IP=192.168.1.250; "
        "cd /home/franka/massage/robots/realman; "
        "python3 - <<'PY'\n"
        + code
        + "\nPY"
    )
    proc = subprocess.run(
        ["docker", "exec", str(container), "bash", "-lc", cmd],
        text=True,
        capture_output=True,
        timeout=max(5.0, float(timeout)),
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"force sample via docker failed: {proc.stderr.strip()} {proc.stdout.strip()}")
    marker = "##FORCE_JSON##"
    if marker not in proc.stdout:
        raise RuntimeError(f"force sample via docker returned no marker: {proc.stdout.strip()}")
    payload = proc.stdout.rsplit(marker, 1)[-1].strip().splitlines()[0]
    if payload in ("", "null", "None"):
        return None
    data = json.loads(payload)
    if not isinstance(data, dict):
        return None
    return {key: float(data[key]) for key in ("fx", "fy", "fz", "mx", "my", "mz")}


def _average_force_samples_via_docker(
    *,
    container: str = "noetic",
    count: int = 2,
) -> dict[str, float] | None:
    samples: list[dict[str, float]] = []
    for _ in range(max(1, int(count))):
        sample = _request_force_sample_via_docker(container=container)
        if sample is not None:
            samples.append(sample)
    if not samples:
        return None
    return {
        key: float(sum(sample[key] for sample in samples) / len(samples))
        for key in ("fx", "fy", "fz", "mx", "my", "mz")
    }


def execute_bladder_tool_axis_probe_remote(
    *,
    host: str,
    plan: BladderMassagePlan,
    speed: int = 3,
    target_force_n: float = 1.0,
    max_force_n: float = 3.0,
    touch_step_m: float = 0.001,
    probe_depth_m: float = 0.008,
    dwell_s: float = 0.1,
    tool_contact_axis: str = "pos_z",
    contact_motion_axis: str = "neg_z",
    entry_tolerance_m: float = 0.015,
    force_container: str = "noetic",
    remote_ssh: str = "rm@192.168.1.11",
    remote_sdk_dir: str = "/home/rm/rm_healthcare_robot/collection/data_collection_d435_arm",
    sdk_code: int = 65,
) -> None:
    """Probe contact with SDK tool-frame relative motion, avoiding Cartesian IK branch switches."""
    from . import rm_json
    from .rm_remote_sdk import move_cartesian_tool

    if os.environ.get("RM_ALLOW_UNSAFE_TOOL_AXIS_PROBE", "0") != "1":
        raise RuntimeError(
            "tool_axis_probe_remote is disabled after hardware validation showed unsafe displacement. "
            "Set RM_ALLOW_UNSAFE_TOOL_AXIS_PROBE=1 only for controlled diagnostics."
        )
    if not plan.frames:
        raise RuntimeError("plan has no frames")
    if len(plan.frames) != 1:
        raise RuntimeError("tool_axis_probe_remote is intentionally limited to one frame per run")

    frame = plan.frames[0]
    preview = validate_aligned_contact_preview(
        plan,
        tool_contact_axis=tool_contact_axis,
        contact_motion_axis=contact_motion_axis,
        max_press_m=0.0,
        touch_step_m=touch_step_m,
        probe_depth_m=probe_depth_m,
    )
    contact_info = dict(list(preview["frames"])[0])
    motion_axis_local = _axis_vector(str(preview["contact_motion_axis"]))
    motion_axis_world = np.asarray(contact_info["contact_motion_axis_world_m"], dtype=np.float64)
    motion_axis_world = motion_axis_world / max(1e-9, float(np.linalg.norm(motion_axis_world)))
    hover_pose = [float(v) for v in frame.hover_pose_m[:6]]

    start_joints, start_pose, arm_err, sys_err, _ = rm_json.get_current_arm_state(host)
    if arm_err != 0 or sys_err != 0:
        raise RuntimeError(f"arm state is not clean: arm_err={arm_err} sys_err={sys_err}")
    entry_dist = float(np.linalg.norm(np.asarray(start_pose[:3], dtype=np.float64) - np.asarray(hover_pose[:3], dtype=np.float64)))
    print(
        "tool_axis_probe_remote "
        f"tool_axis={preview['tool_contact_axis']} motion_axis={preview['contact_motion_axis']} "
        f"probe_depth_m={float(probe_depth_m):.4f} touch_step_m={float(touch_step_m):.4f} "
        f"entry_dist_m={entry_dist:.5f}"
    )
    print(
        f"hover={hover_pose[:3]} current={start_pose[:3]} "
        f"motion_axis_world={[round(float(v), 6) for v in motion_axis_world.tolist()]}"
    )
    if entry_dist > float(entry_tolerance_m):
        raise RuntimeError(
            "current TCP is not at the first hover point; run hover_path to the target point first. "
            f"entry_dist_m={entry_dist:.5f} tolerance={float(entry_tolerance_m):.5f}"
        )

    baseline = _average_force_samples_via_docker(container=force_container, count=2)
    if baseline is None:
        raise RuntimeError("force baseline unavailable; refusing to probe contact")
    print(f"tool_axis_probe baseline {_format_force_sample(baseline)}")

    step_m = max(0.0005, float(touch_step_m))
    total_m = max(0.0, float(probe_depth_m))
    step_count = max(1, int(math.ceil(total_m / step_m)))
    moved_m = 0.0
    reached = False
    try:
        for step_idx in range(1, step_count + 1):
            target_m = min(total_m, float(step_idx) * step_m)
            this_step_m = max(0.0, target_m - moved_m)
            if this_step_m <= 0.0:
                continue
            joints, before_pose, arm_err, sys_err, _ = rm_json.get_current_arm_state(host)
            if arm_err != 0 or sys_err != 0:
                raise RuntimeError(f"arm state is not clean before step: arm_err={arm_err} sys_err={sys_err}")
            move_cartesian_tool(
                host=host,
                joints_deg=joints,
                dx_m=float(motion_axis_local[0]) * this_step_m,
                dy_m=float(motion_axis_local[1]) * this_step_m,
                dz_m=float(motion_axis_local[2]) * this_step_m,
                speed=max(1, int(speed)),
                code=int(sdk_code),
                remote_ssh=remote_ssh,
                remote_dir=remote_sdk_dir,
                timeout=30.0,
            )
            moved_m = target_m
            _, after_pose, arm_err, sys_err, _ = rm_json.get_current_arm_state(host)
            if arm_err != 0 or sys_err != 0:
                raise RuntimeError(f"arm state is not clean after step: arm_err={arm_err} sys_err={sys_err}")
            displacement = np.asarray(after_pose[:3], dtype=np.float64) - np.asarray(before_pose[:3], dtype=np.float64)
            along = float(np.dot(displacement, motion_axis_world))
            lateral = float(np.linalg.norm(displacement - motion_axis_world * along))
            sample = _average_force_samples_via_docker(container=force_container, count=1)
            delta_n = _force_delta_n(sample, baseline)
            delta_text = "n/a" if delta_n is None else f"{delta_n:.2f}N"
            print(
                f"tool_axis_probe step {step_idx}/{step_count} moved_m={moved_m:.4f} "
                f"actual_along_m={along:.5f} lateral_m={lateral:.5f} "
                f"delta={delta_text} {_format_force_sample(sample)}"
            )
            if along < -0.0005:
                raise RuntimeError(f"tool-axis motion moved opposite expected direction: actual_along_m={along:.5f}")
            if lateral > max(0.006, this_step_m * 3.0):
                raise RuntimeError(f"tool-axis motion lateral drift too large: lateral_m={lateral:.5f}")
            if delta_n is None:
                continue
            if delta_n >= float(max_force_n):
                raise RuntimeError(f"exceeded max force: delta={delta_n:.2f}N max={float(max_force_n):.2f}N")
            if delta_n >= float(target_force_n):
                reached = True
                print(f"tool_axis_probe target reached: delta={delta_n:.2f}N")
                if dwell_s > 0.0:
                    time.sleep(float(dwell_s))
                break

        if not reached:
            print("tool_axis_probe target force not reached within probe depth; retreating")
    except BaseException as exc:
        print(f"execute_bladder_tool_axis_probe_remote aborted: {type(exc).__name__}: {exc}")
        try:
            rm_json.stop_motion(host)
        except Exception as stop_exc:
            print(f"tool_axis_probe stop failed: {stop_exc}")
        raise
    finally:
        try:
            rm_json.movej(host, start_joints, speed=max(1, int(speed)), timeout=30.0)
            print("tool_axis_probe retreated to start hover joints")
        except Exception as retreat_exc:
            print(f"tool_axis_probe retreat failed: {retreat_exc}")


def execute_bladder_constant_force_plan(
    *,
    host: str,
    plan: BladderMassagePlan,
    speed: float = 2.0,
    target_force_n: float = 2.0,
    max_force_n: float = 8.0,
    force_direction: int = 2,
    force_mode: int = 1,
    force_coordinate: int = 1,
    z_control_mode: int = 1,
    touch_step_m: float = 0.001,
    max_press_m: float = 0.003,
    dwell_s: float = 0.8,
    point_limit: int = 1,
    entry_motion: str = "movej_p",
) -> None:
    """Use the product force-position controller for low-force side-lying contact.

    The side-lying plan must already have a contact direction that is safe for
    the patient setup. For the current mas_rub side-lying setup, the verified
    contact axis is the tool Z axis, so the force-position controller defaults
    to direction=2 in tool coordinates.
    """
    from .rm_execute import RosForceBridge

    if not plan.frames:
        raise RuntimeError("constant force plan has no frames")

    motion_speed = normalize_motion_speed("ros", float(speed), ros_default=0.12)
    entry_speed = normalize_motion_speed("ros", 0.12, ros_default=0.12)
    arm = create_arm_backend("ros")
    entry_arm = arm
    bridge = RosForceBridge()
    active_force = False
    last_hover_pose: list[float] | None = None

    try:
        for idx, frame in enumerate(plan.frames[: max(1, int(point_limit))], start=1):
            hover_pose = [float(v) for v in frame.hover_pose_m[:6]]
            last_hover_pose = hover_pose
            press = _normalize_vec(np.asarray(frame.press_direction_m[:3], dtype=np.float64))
            if press is None:
                raise RuntimeError(f"point {idx} press_direction is invalid")
            tool_z = _normalize_vec(_rpy_to_matrix(*hover_pose[3:6])[:, 2])
            if int(force_direction) == 2 and tool_z is not None:
                axis_dot = float(abs(np.dot(press, tool_z)))
                if axis_dot < 0.985:
                    raise RuntimeError(
                        "constant force side-lying press_direction is not aligned with tool Z: "
                        f"dot={axis_dot:.4f} press={press.tolist()} tool_z={tool_z.tolist()}"
                    )
            else:
                axis_dot = float("nan")

            contact_depth_m = float(plan.hover_m) + max(0.0, float(max_press_m))
            print(
                f"constant_force point {idx}/{plan.point_count} "
                f"hover={hover_pose[:3]} press={[round(float(v), 5) for v in press.tolist()]} "
                f"depth_m={contact_depth_m:.4f} force={float(target_force_n):.2f} "
                f"direction={int(force_direction)} coordinate={int(force_coordinate)} "
                f"tool_z_dot={axis_dot:.4f}"
            )

            if idx == 1 and str(entry_motion).strip().lower() == "movej_p":
                if not hasattr(entry_arm, "movej_p"):
                    raise RuntimeError("ROS entry backend does not support movej_p")
                entry_arm.movej_p(host, hover_pose, speed=entry_speed, timeout=60.0)
            else:
                arm.movel(host, hover_pose, speed=motion_speed, timeout=30.0)

            bridge.enable_force_sensor()
            baseline = bridge.request_force_sample(timeout=1.0)
            if baseline is None:
                raise RuntimeError("force baseline unavailable; refusing constant-force contact")
            print(f"constant_force point {idx} baseline {_format_force_sample(baseline)}")

            bridge.configure_force_tracking(
                target_force_n=int(round(float(target_force_n))),
                coordinate=int(force_coordinate),
                z_control_mode=int(z_control_mode),
                sensor=1,
            )
            bridge.configure_force_position(
                target_force_n=int(round(float(target_force_n))),
                mode=int(force_mode),
                direction=int(force_direction),
                sensor=1,
            )
            bridge.start_force_position(wait_s=0.02)
            active_force = True

            step_m = max(0.0005, float(touch_step_m))
            step_count = max(1, int(math.ceil(contact_depth_m / step_m)))
            max_seen = 0.0
            for step_idx in range(1, step_count + 1):
                offset_m = min(contact_depth_m, float(step_idx) * step_m)
                target_pose = list(hover_pose)
                for axis in range(3):
                    target_pose[axis] = float(hover_pose[axis]) + float(press[axis]) * offset_m
                arm.movel(host, target_pose, speed=motion_speed, timeout=20.0)
                sample = bridge.request_force_sample(timeout=0.8)
                delta_n = _force_delta_n(sample, baseline)
                if delta_n is not None:
                    max_seen = max(max_seen, float(delta_n))
                delta_text = "n/a" if delta_n is None else f"{delta_n:.2f}N"
                state = bridge.last_force_state
                state_text = "" if state is None else f" state_force={float(state.get('force', 0.0)):.2f} arm_err={int(state.get('arm_err', 0))}"
                print(
                    f"constant_force point {idx} step {step_idx}/{step_count} "
                    f"offset_m={offset_m:.4f} delta={delta_text}{state_text} {_format_force_sample(sample)}"
                )
                if delta_n is not None and delta_n >= float(max_force_n):
                    raise RuntimeError(
                        f"point {idx} exceeded max force while constant-force moving: "
                        f"delta={delta_n:.2f}N max={float(max_force_n):.2f}N"
                    )

            if dwell_s > 0.0:
                deadline = time.time() + float(dwell_s)
                while time.time() < deadline:
                    sample = bridge.request_force_sample(timeout=0.5)
                    delta_n = _force_delta_n(sample, baseline)
                    delta_text = "n/a" if delta_n is None else f"{delta_n:.2f}N"
                    print(f"constant_force point {idx} dwell delta={delta_text} {_format_force_sample(sample)}")
                    if delta_n is not None and delta_n >= float(max_force_n):
                        raise RuntimeError(
                            f"point {idx} exceeded max force during dwell: "
                            f"delta={delta_n:.2f}N max={float(max_force_n):.2f}N"
                        )
                    time.sleep(0.1)

            print(f"constant_force point {idx} retreat hover max_seen={max_seen:.2f}N")
            bridge.stop_force_position()
            active_force = False
            arm.movel(host, hover_pose, speed=motion_speed, timeout=30.0)
    except BaseException as exc:
        print(f"execute_bladder_constant_force_plan aborted: {type(exc).__name__}: {exc}")
        try:
            if active_force:
                bridge.stop_force_position()
        except Exception as stop_force_exc:
            print(f"constant_force stop force failed: {stop_force_exc}")
        try:
            arm.stop_motion(host)
        except Exception as stop_exc:
            print(f"constant_force stop failed: {stop_exc}")
        try:
            create_arm_backend("ros").stop_motion(host)
        except Exception as stop_exc:
            print(f"constant_force ros stop failed: {stop_exc}")
        raise


def execute_bladder_hover_path(
    *,
    host: str,
    plan: BladderMassagePlan,
    speed: int,
    control_backend: str = "json",
    dwell_s: float = 0.0,
    use_global_safe_z: bool = True,
    keep_current_orientation: bool = False,
    max_step_m: float = 0.03,
    entry_motion: str = "movel",
) -> None:
    """Traverse the selected bladder line using hover poses only.

    This is intended as a safe verification mode:
    - no point press
    - no split motion
    - no contact with the body
    """
    arm = create_arm_backend(control_backend)
    arm.recover_if_needed(host)
    motion_speed = normalize_motion_speed(control_backend, float(speed), ros_default=0.3)
    _, current_pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(host)
    print(
        f"current_pose={[round(v, 6) for v in current_pose]} "
        f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
    )

    try:
        if use_global_safe_z:
            _lift_to_safe_z(arm, host, current_pose, plan.safe_z_m, motion_speed)
            print(f"hover_path start point 1/{plan.point_count} hover={plan.frames[0].hover_pose_m[:3]}")
            _goto_hover_via_safe(arm, host, plan.frames[0].hover_pose_m, plan.safe_z_m, motion_speed)
            time.sleep(max(0.0, float(dwell_s)))

            for idx, frame in enumerate(plan.frames[1:], start=2):
                print(f"hover_path point {idx}/{plan.point_count} hover={frame.hover_pose_m[:3]}")
                _goto_hover_via_safe(arm, host, frame.hover_pose_m, plan.safe_z_m, motion_speed)
                time.sleep(max(0.0, float(dwell_s)))

            _, final_pose, _, _, _ = arm.get_current_arm_state(host)
            _lift_to_safe_z(arm, host, final_pose, plan.safe_z_m, motion_speed)
            return

        print("hover_path direct mode: using product hover poses without global safe-z lift")
        for idx, frame in enumerate(plan.frames, start=1):
            target_pose = list(frame.hover_pose_m)
            if keep_current_orientation:
                _, latest_pose, _, _, _ = arm.get_current_arm_state(host)
                target_pose[3:6] = [float(v) for v in latest_pose[3:6]]
            _, start_pose, _, _, _ = arm.get_current_arm_state(host)
            entry_mode = str(entry_motion or "movel").strip().lower()
            if idx == 1 and entry_mode == "movej_p":
                print(f"hover_path entry movej_p point 1/{plan.point_count} hover={target_pose[:3]}")
                if not hasattr(arm, "movej_p"):
                    raise RuntimeError("selected arm backend does not support movej_p")
                arm.movej_p(host, target_pose, speed=motion_speed, timeout=40.0)
                time.sleep(max(0.0, float(dwell_s)))
                continue
            if entry_mode not in ("movel", "movej_p"):
                raise ValueError(f"unsupported hover_path entry_motion: {entry_motion}")
            dist_m = float(np.linalg.norm(np.asarray(target_pose[:3], dtype=np.float64) - np.asarray(start_pose[:3], dtype=np.float64)))
            step_count = max(1, int(math.ceil(dist_m / max(1e-6, float(max_step_m)))))
            print(
                f"hover_path point {idx}/{plan.point_count} hover={target_pose[:3]} "
                f"dist_m={dist_m:.4f} steps={step_count}"
            )
            for step_idx in range(1, step_count + 1):
                ratio = float(step_idx) / float(step_count)
                step_pose = list(target_pose)
                step_pose[:3] = [
                    float(start_pose[i] + (target_pose[i] - start_pose[i]) * ratio)
                    for i in range(3)
                ]
                if not keep_current_orientation:
                    step_pose[3:6] = [
                        _lerp_angle_rad(start_pose[i], target_pose[i], ratio)
                        for i in range(3, 6)
                    ]
                print(f"  step {step_idx}/{step_count} xyz={step_pose[:3]} rpy={step_pose[3:6]}")
                arm.movel(host, step_pose, speed=motion_speed, timeout=20.0)
            time.sleep(max(0.0, float(dwell_s)))
    except BaseException as exc:
        print(f"execute_bladder_hover_path aborted: {type(exc).__name__}: {exc}")
        if use_global_safe_z:
            _emergency_retreat(arm, host, plan.safe_z_m, motion_speed)
        else:
            try:
                arm.stop_motion(host)
            except Exception as stop_exc:
                print(f"hover_path direct stop failed: {stop_exc}")
        raise
