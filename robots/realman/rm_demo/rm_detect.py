from __future__ import annotations

import json
import os
import time
import glob
import sys
from typing import Any

import cv2
import numpy as np

from .config import (
    DEFAULT_CONF,
    DEFAULT_FINGER_WIDTH_MM,
    DEFAULT_MODEL_PATH,
    DEFAULT_SAMPLE_POINTS,
    DEPTH_MEDIAN_RADIUS,
    ESTIMATED_SHOULDER_MM,
    MIN_VISUAL_OFFSET_PX,
    AREA_TOP_TRIM_RATIO,
    AREA_BOTTOM_TRIM_RATIO,
)
from .rm_capture import ensure_output_dir


POSE_ROTATION_MODES = ("none", "cw90", "ccw90", "180")
REQUIRED_KEYPOINTS = (5, 6, 11, 12)


_MODEL_CACHE: dict[str, Any] = {}


def _import_ros_area_modules():
    try:
        import rospy  # type: ignore
    except Exception:
        candidates = []
        candidates.extend(glob.glob("/opt/ros/*/lib/python3/dist-packages"))
        candidates.append("/home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/lib/python3/dist-packages")
        for candidate in candidates:
            if candidate and os.path.isdir(candidate) and candidate not in sys.path:
                sys.path.append(candidate)
        import rospy  # type: ignore

    from cv_bridge import CvBridge  # type: ignore
    from rm_healthcare_robot_msgs.srv import AreaDetection  # type: ignore

    return rospy, CvBridge, AreaDetection


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
    best: dict[str, object] = {
        "kpts": None,
        "score": -1.0,
        "rotation": "none",
    }
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


def _normalize_vector(vec: list[float] | np.ndarray, eps: float = 1e-6) -> np.ndarray | None:
    arr = np.asarray(vec, dtype=np.float64)
    norm = float(np.linalg.norm(arr))
    if norm <= eps:
        return None
    return arr / norm


def _get_body_lateral_direction_2d(kpts: np.ndarray) -> np.ndarray:
    for left_idx, right_idx in ((5, 6), (11, 12)):
        if float(kpts[left_idx][2]) > 0.3 and float(kpts[right_idx][2]) > 0.3:
            direction = _normalize_vector(
                [
                    float(kpts[right_idx][0] - kpts[left_idx][0]),
                    float(kpts[right_idx][1] - kpts[left_idx][1]),
                ]
            )
            if direction is not None:
                return direction
    return np.asarray([1.0, 0.0], dtype=np.float64)


def _clamp_pixel(x: float, y: float, width: int, height: int) -> list[int]:
    return [
        int(max(0, min(width - 1, round(x)))),
        int(max(0, min(height - 1, round(y)))),
    ]


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


def _point3d_from_depth(intrinsics_data: dict[str, object], pixel_u: float, pixel_v: float, depth_m: np.ndarray) -> list[float] | None:
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
    z = dist
    return [float(x), float(y), float(z)]


def _sample_line_pixels(p1: list[int], p2: list[int], num_points: int) -> list[list[int]]:
    pts: list[list[int]] = []
    for idx in range(max(1, int(num_points)) + 1):
        t = idx / max(1, int(num_points))
        pts.append(
            [
                int(round(float(p1[0]) * (1.0 - t) + float(p2[0]) * t)),
                int(round(float(p1[1]) * (1.0 - t) + float(p2[1]) * t)),
            ]
        )
    return pts


def _pixels_to_points3d(pixels: list[list[int]], depth_m: np.ndarray, intrinsics_data: dict[str, object]) -> list[list[float]]:
    points: list[list[float]] = []
    for u, v in pixels:
        point = _point3d_from_depth(intrinsics_data, float(u), float(v), depth_m)
        if point is not None:
            points.append(point)
    return points


def _extract_body_component(
    image_bgr: np.ndarray,
    seed_point: tuple[int, int] | None = None,
) -> tuple[np.ndarray, tuple[int, int, int, int]] | None:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)
    _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    kernel = np.ones((7, 7), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if num_labels <= 1:
        return None

    h, w = gray.shape[:2]
    chosen_label = -1
    if seed_point is not None:
        sx = int(max(0, min(w - 1, seed_point[0])))
        sy = int(max(0, min(h - 1, seed_point[1])))
        seed_label = int(labels[sy, sx])
        if seed_label > 0 and int(stats[seed_label, cv2.CC_STAT_AREA]) > 1000:
            chosen_label = seed_label

    if chosen_label <= 0:
        center = np.asarray([w / 2.0, h / 2.0], dtype=np.float64)
        best_score = None
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < 5000:
                continue
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            ww = int(stats[label, cv2.CC_STAT_WIDTH])
            hh = int(stats[label, cv2.CC_STAT_HEIGHT])
            comp_center = np.asarray([x + ww / 2.0, y + hh / 2.0], dtype=np.float64)
            center_dist = float(np.linalg.norm(comp_center - center))
            score = center_dist - area * 0.002
            if best_score is None or score < best_score:
                best_score = score
                chosen_label = label

    if chosen_label <= 0:
        return None

    mask = np.uint8(labels == chosen_label) * 255
    ys, xs = np.where(mask > 0)
    if xs.size == 0 or ys.size == 0:
        return None
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
    return mask, bbox


def _tail_from_component_mask(
    body_mask: np.ndarray,
    bbox: tuple[int, int, int, int],
) -> np.ndarray:
    x1, y1, x2, y2 = bbox
    crop = body_mask[y1 : y2 + 1, x1 : x2 + 1]
    rows = np.where(crop.max(axis=1) > 0)[0]
    if rows.size == 0:
        return np.asarray([(x1 + x2) / 2.0, float(y2)], dtype=np.float64)

    bottom_band_start = int(rows.max() * 0.82)
    band_rows = [row for row in rows if row >= bottom_band_start]
    xs_all: list[int] = []
    ys_all: list[int] = []
    for row in band_rows:
        cols = np.where(crop[row] > 0)[0]
        if cols.size == 0:
            continue
        xs_all.extend((cols + x1).tolist())
        ys_all.extend([row + y1] * int(cols.size))
    if not xs_all:
        return np.asarray([(x1 + x2) / 2.0, float(y2)], dtype=np.float64)
    return np.asarray([float(np.median(xs_all)), float(np.median(ys_all))], dtype=np.float64)


def _build_overlay(
    image_bgr: np.ndarray,
    lines: dict[str, dict[str, list[int]]],
    keypoints: np.ndarray,
    finger_width_mm: float,
    pose_rotation: str,
    shoulder_cm_real: float | None,
    offset_px: float,
) -> np.ndarray:
    overlay = image_bgr.copy()
    colors = {
        "center": (0, 0, 255),
        "left": (0, 255, 0),
        "right": (0, 255, 0),
    }
    if "center" in lines:
        cv2.line(overlay, tuple(lines["center"]["start"]), tuple(lines["center"]["end"]), colors["center"], 2)
    for side in ("left", "right"):
        cv2.line(overlay, tuple(lines[side]["start"]), tuple(lines[side]["end"]), colors[side], 2)
    for idx, name, color in (
        (5, "LS", (0, 200, 255)),
        (6, "RS", (0, 200, 255)),
        (11, "LH", (200, 255, 0)),
        (12, "RH", (200, 255, 0)),
    ):
        if float(keypoints[idx][2]) <= 0.2:
            continue
        x = int(round(float(keypoints[idx][0])))
        y = int(round(float(keypoints[idx][1])))
        cv2.circle(overlay, (x, y), 5, color, -1)
        cv2.putText(overlay, name, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    shoulder_txt = f"{shoulder_cm_real:.1f}cm" if shoulder_cm_real and shoulder_cm_real > 0 else "N/A"
    cv2.putText(
        overlay,
        f"Offset: {finger_width_mm:.1f}mm | Shoulder: {shoulder_txt}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        overlay,
        f"Pose rot: {pose_rotation} | Body offset: {int(round(offset_px))} px",
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        1,
    )
    return overlay


def _call_area_detection_service(color_bgr: np.ndarray, weizhi: str = "beibu"):
    rospy, CvBridge, AreaDetection = _import_ros_area_modules()
    if not rospy.core.is_initialized():
        rospy.init_node("rm_demo_area_detection", anonymous=True, disable_signals=True)
    bridge = CvBridge()
    rospy.wait_for_service("/ai_service/area_detection", timeout=10.0)
    proxy = rospy.ServiceProxy("/ai_service/area_detection", AreaDetection)
    response = proxy(bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8"), str(weizhi))
    annotated_bgr = None
    if getattr(response.annotated_frame, "width", 0) and getattr(response.annotated_frame, "height", 0):
        annotated_bgr = bridge.imgmsg_to_cv2(response.annotated_frame, desired_encoding="bgr8")
    return response, annotated_bgr


def _build_area_lines(
    diagonal_points: list[list[int]],
    image_width: int,
    image_height: int,
    finger_width_mm: float,
) -> tuple[dict[str, dict[str, list[int]]], float]:
    xs = [int(p[0]) for p in diagonal_points]
    ys = [int(p[1]) for p in diagonal_points]
    x1 = max(0, min(xs))
    x2 = min(image_width - 1, max(xs))
    y1 = max(0, min(ys))
    y2 = min(image_height - 1, max(ys))
    if x2 - x1 < 20 or y2 - y1 < 20:
        raise RuntimeError(f"invalid body area bbox: {(x1, y1, x2, y2)}")

    box_width = float(x2 - x1)
    top_y = float(y1) + max(4.0, float(y2 - y1) * AREA_TOP_TRIM_RATIO)
    bottom_y = float(y2) - max(4.0, float(y2 - y1) * AREA_BOTTOM_TRIM_RATIO)
    center_x = (float(x1) + float(x2)) / 2.0
    pixels_per_mm = box_width / ESTIMATED_SHOULDER_MM if ESTIMATED_SHOULDER_MM > 0 else 1.0
    body_offset_px = max(MIN_VISUAL_OFFSET_PX, float(finger_width_mm) * pixels_per_mm)

    lines = {
        "center": {
            "start": _clamp_pixel(center_x, top_y, image_width, image_height),
            "end": _clamp_pixel(center_x, bottom_y, image_width, image_height),
        },
        "left": {
            "start": _clamp_pixel(center_x - body_offset_px, top_y, image_width, image_height),
            "end": _clamp_pixel(center_x - body_offset_px, bottom_y, image_width, image_height),
        },
        "right": {
            "start": _clamp_pixel(center_x + body_offset_px, top_y, image_width, image_height),
            "end": _clamp_pixel(center_x + body_offset_px, bottom_y, image_width, image_height),
        },
    }
    return lines, body_offset_px


def _build_area_overlay(
    image_bgr: np.ndarray,
    base_overlay: np.ndarray | None,
    diagonal_points: list[list[int]],
    lines: dict[str, dict[str, list[int]]],
    finger_width_mm: float,
    offset_px: float,
) -> np.ndarray:
    overlay = image_bgr.copy() if base_overlay is None else base_overlay.copy()
    xs = [int(p[0]) for p in diagonal_points]
    ys = [int(p[1]) for p in diagonal_points]
    x1 = min(xs)
    x2 = max(xs)
    y1 = min(ys)
    y2 = max(ys)
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 165, 255), 2)
    cv2.line(overlay, tuple(lines["center"]["start"]), tuple(lines["center"]["end"]), (0, 0, 255), 2)
    cv2.line(overlay, tuple(lines["left"]["start"]), tuple(lines["left"]["end"]), (0, 255, 0), 2)
    cv2.line(overlay, tuple(lines["right"]["start"]), tuple(lines["right"]["end"]), (0, 255, 0), 2)
    cv2.putText(
        overlay,
        f"Area backend | Offset: {finger_width_mm:.1f}mm -> {int(round(offset_px))} px",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )
    return overlay


def _detect_static_meridians_pose(
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics_data: dict[str, object],
    finger_width_mm: float = DEFAULT_FINGER_WIDTH_MM,
    model_path: str = DEFAULT_MODEL_PATH,
    sample_points: int = DEFAULT_SAMPLE_POINTS,
    conf: float = DEFAULT_CONF,
) -> tuple[dict[str, object], np.ndarray]:
    model = _load_model(model_path)
    pose_info = _infer_best_pose_with_rotations(model, color_bgr, conf=conf)
    if pose_info["kpts"] is None:
        raise RuntimeError("no valid body pose detected in current frame")

    kpts = pose_info["kpts"]
    shoulder_required = (5, 6)
    if any(float(kpts[idx][2]) <= 0.5 for idx in shoulder_required):
        raise RuntimeError("pose confidence is too low for meridian generation")

    h, w = color_bgr.shape[:2]
    ls = np.array([float(kpts[5][0]), float(kpts[5][1])], dtype=np.float64)
    rs_pt = np.array([float(kpts[6][0]), float(kpts[6][1])], dtype=np.float64)
    neck = (ls + rs_pt) / 2.0
    tail_source = "hips"
    if float(kpts[11][2]) > 0.3 and float(kpts[12][2]) > 0.3:
        lh = np.array([float(kpts[11][0]), float(kpts[11][1])], dtype=np.float64)
        rh = np.array([float(kpts[12][0]), float(kpts[12][1])], dtype=np.float64)
        tail = (lh + rh) / 2.0
        body_bbox = [
            _clamp_pixel(min(ls[0], rs_pt[0], lh[0], rh[0]) - 30.0, min(ls[1], rs_pt[1], lh[1], rh[1]) - 30.0, w, h),
            _clamp_pixel(max(ls[0], rs_pt[0], lh[0], rh[0]) + 30.0, min(ls[1], rs_pt[1], lh[1], rh[1]) - 30.0, w, h),
            _clamp_pixel(min(ls[0], rs_pt[0], lh[0], rh[0]) - 30.0, max(ls[1], rs_pt[1], lh[1], rh[1]) + 30.0, w, h),
            _clamp_pixel(max(ls[0], rs_pt[0], lh[0], rh[0]) + 30.0, max(ls[1], rs_pt[1], lh[1], rh[1]) + 30.0, w, h),
        ]
    else:
        seed_point = (int(round(float(neck[0]))), int(round(float(neck[1] + 40.0))))
        component = _extract_body_component(color_bgr, seed_point=seed_point)
        if component is None:
            raise RuntimeError("pose confidence is too low for meridian generation")
        body_mask, bbox = component
        tail = _tail_from_component_mask(body_mask, bbox)
        tail_source = "contour"
        body_bbox = [[bbox[0], bbox[1]], [bbox[2], bbox[1]], [bbox[0], bbox[3]], [bbox[2], bbox[3]]]
    lateral_direction = _get_body_lateral_direction_2d(kpts)
    shoulder_px = float(np.linalg.norm(rs_pt - ls))
    pixels_per_mm = shoulder_px / ESTIMATED_SHOULDER_MM if ESTIMATED_SHOULDER_MM > 0 else 1.0
    body_offset_px = max(MIN_VISUAL_OFFSET_PX, float(finger_width_mm) * pixels_per_mm)

    neck_l = _clamp_pixel(neck[0] - lateral_direction[0] * body_offset_px, neck[1] - lateral_direction[1] * body_offset_px, w, h)
    neck_r = _clamp_pixel(neck[0] + lateral_direction[0] * body_offset_px, neck[1] + lateral_direction[1] * body_offset_px, w, h)
    tail_l = _clamp_pixel(tail[0] - lateral_direction[0] * body_offset_px, tail[1] - lateral_direction[1] * body_offset_px, w, h)
    tail_r = _clamp_pixel(tail[0] + lateral_direction[0] * body_offset_px, tail[1] + lateral_direction[1] * body_offset_px, w, h)
    neck_c = _clamp_pixel(neck[0], neck[1], w, h)
    tail_c = _clamp_pixel(tail[0], tail[1], w, h)

    lines = {
        "center": {"start": neck_c, "end": tail_c},
        "left": {"start": neck_l, "end": tail_l},
        "right": {"start": neck_r, "end": tail_r},
    }
    pixels = {side: _sample_line_pixels(lines[side]["start"], lines[side]["end"], sample_points) for side in ("left", "right")}
    camera_points = {side: _pixels_to_points3d(pixels[side], depth_m, intrinsics_data) for side in ("left", "right")}

    shoulder_cm_real = None
    left_pt3 = _point3d_from_depth(intrinsics_data, float(kpts[5][0]), float(kpts[5][1]), depth_m)
    right_pt3 = _point3d_from_depth(intrinsics_data, float(kpts[6][0]), float(kpts[6][1]), depth_m)
    if left_pt3 is not None and right_pt3 is not None:
        shoulder_cm_real = float(np.linalg.norm(np.asarray(right_pt3) - np.asarray(left_pt3)) * 100.0)

    overlay = _build_overlay(
        image_bgr=color_bgr,
        lines=lines,
        keypoints=kpts,
        finger_width_mm=float(finger_width_mm),
        pose_rotation=str(pose_info["rotation"]),
        shoulder_cm_real=shoulder_cm_real,
        offset_px=body_offset_px,
    )
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    result: dict[str, object] = {
        "timestamp": timestamp,
        "detector_backend": "pose",
        "model_path": os.path.abspath(model_path),
        "image_size": {"width": int(w), "height": int(h)},
        "pose_rotation": str(pose_info["rotation"]),
        "tail_source": tail_source,
        "body_bbox_pixel": body_bbox,
        "finger_width_mm": float(finger_width_mm),
        "sample_points": int(sample_points),
        "camera_frame_unit": "meters",
        "intrinsics": intrinsics_data,
        "center_line_pixel": pixels["left"] if False else _sample_line_pixels(lines["center"]["start"], lines["center"]["end"], sample_points),
        "left_meridian_pixel": pixels["left"],
        "right_meridian_pixel": pixels["right"],
        "left_meridian_camera": camera_points["left"],
        "right_meridian_camera": camera_points["right"],
        "keypoints": {str(idx): [float(v) for v in kpts[idx][:3]] for idx in (5, 6, 11, 12)},
    }
    return result, overlay


def _detect_static_meridians_area(
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics_data: dict[str, object],
    finger_width_mm: float = DEFAULT_FINGER_WIDTH_MM,
    sample_points: int = DEFAULT_SAMPLE_POINTS,
) -> tuple[dict[str, object], np.ndarray]:
    response, annotated_bgr = _call_area_detection_service(color_bgr=color_bgr, weizhi="beibu")
    if not bool(response.success):
        raise RuntimeError("area_detection service returned success=False")

    diagonal_points = [[int(point.x), int(point.y)] for point in response.diagonal_point_coor]
    if len(diagonal_points) != 4:
        raise RuntimeError(f"unexpected area_detection points: {diagonal_points}")

    h, w = color_bgr.shape[:2]
    lines, body_offset_px = _build_area_lines(
        diagonal_points=diagonal_points,
        image_width=w,
        image_height=h,
        finger_width_mm=finger_width_mm,
    )
    pixels = {side: _sample_line_pixels(lines[side]["start"], lines[side]["end"], sample_points) for side in ("left", "right")}
    camera_points = {side: _pixels_to_points3d(pixels[side], depth_m, intrinsics_data) for side in ("left", "right")}
    overlay = _build_area_overlay(
        image_bgr=color_bgr,
        base_overlay=annotated_bgr,
        diagonal_points=diagonal_points,
        lines=lines,
        finger_width_mm=float(finger_width_mm),
        offset_px=body_offset_px,
    )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    result: dict[str, object] = {
        "timestamp": timestamp,
        "detector_backend": "area",
        "model_path": "",
        "image_size": {"width": int(w), "height": int(h)},
        "pose_rotation": "none",
        "finger_width_mm": float(finger_width_mm),
        "sample_points": int(sample_points),
        "camera_frame_unit": "meters",
        "intrinsics": intrinsics_data,
        "body_bbox_pixel": diagonal_points,
        "area_bbox_pixel": diagonal_points,
        "center_line_pixel": _sample_line_pixels(lines["center"]["start"], lines["center"]["end"], sample_points),
        "left_meridian_pixel": pixels["left"],
        "right_meridian_pixel": pixels["right"],
        "left_meridian_camera": camera_points["left"],
        "right_meridian_camera": camera_points["right"],
        "keypoints": {},
    }
    return result, overlay


def detect_static_meridians(
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics_data: dict[str, object],
    finger_width_mm: float = DEFAULT_FINGER_WIDTH_MM,
    model_path: str = DEFAULT_MODEL_PATH,
    sample_points: int = DEFAULT_SAMPLE_POINTS,
    conf: float = DEFAULT_CONF,
    backend: str = "auto",
) -> tuple[dict[str, object], np.ndarray]:
    backend = backend.strip().lower()
    if backend not in ("auto", "pose", "area"):
        raise ValueError(f"unsupported detector backend: {backend}")

    errors: list[str] = []
    if backend in ("auto", "pose"):
        try:
            return _detect_static_meridians_pose(
                color_bgr=color_bgr,
                depth_m=depth_m,
                intrinsics_data=intrinsics_data,
                finger_width_mm=finger_width_mm,
                model_path=model_path,
                sample_points=sample_points,
                conf=conf,
            )
        except Exception as exc:
            errors.append(f"pose: {exc}")
            if backend == "pose":
                raise

    if backend in ("auto", "area"):
        try:
            return _detect_static_meridians_area(
                color_bgr=color_bgr,
                depth_m=depth_m,
                intrinsics_data=intrinsics_data,
                finger_width_mm=finger_width_mm,
                sample_points=sample_points,
            )
        except Exception as exc:
            errors.append(f"area: {exc}")
            if backend == "area":
                raise

    raise RuntimeError("all detector backends failed: " + " | ".join(errors))


def select_side(result: dict[str, object], side: str) -> dict[str, object]:
    side = side.lower().strip()
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    selected = dict(result)
    selected["selected_side"] = side
    selected["selected_meridian_pixel"] = result[f"{side}_meridian_pixel"]
    selected["selected_meridian_camera"] = result[f"{side}_meridian_camera"]
    return selected


def save_detection_artifacts(
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
