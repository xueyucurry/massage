#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from build_bladder_depth_normal_diagnostic import (
    align_axis_rotation,
    axis_vector,
    fit_local_normal_camera,
    matrix_to_rpy,
    normalize,
    project_camera,
    rpy_to_matrix,
    select_line,
    transform_direction,
)
from build_depth_normal_bladder_plan import build_split_axis, pick_evenly_indices, smooth_normals
from rm_demo import rm_json
from rm_demo.rm_bladder import (
    BladderMassageFrame,
    BladderMassagePlan,
    bladder_plan_to_dict,
    build_aligned_contact_preview,
    detect_bladder_lines,
    execute_bladder_hover_path,
    preview_bladder_plan,
    save_bladder_artifacts,
)
from rm_demo.rm_transform import load_transform_matrix, transform_points
from run_external_bladder_live_hover import _intrinsics_from_profile, _read_aligned_frame, _stream_profile


def _load_reference_pose(path: str) -> list[float]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    pose = data.get("tcp_pose_m_rpy")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"reference pose JSON missing tcp_pose_m_rpy: {path}")
    return [float(v) for v in pose[:6]]


def _base_to_pixel(point_base: np.ndarray, camera_from_base: np.ndarray, intrinsics: dict[str, object]) -> list[float] | None:
    point_camera_h = camera_from_base @ np.asarray(
        [float(point_base[0]), float(point_base[1]), float(point_base[2]), 1.0],
        dtype=np.float64,
    )
    return project_camera(point_camera_h[:3], intrinsics)


def _build_tool_axis_hover_pixels(
    *,
    selected: dict[str, object],
    reference_pose: list[float],
    tool_front_axis: str,
    hover_m: float,
    matrix: np.ndarray,
    intrinsics: dict[str, object],
) -> tuple[list[list[float]], list[float]]:
    rot = rpy_to_matrix(*reference_pose[3:6])
    front_axis_base = normalize(rot @ axis_vector(tool_front_axis))
    base_all = np.asarray(selected["selected_meridian_robot"], dtype=np.float64)
    indices = [int(v) for v in selected["selected_plan_indices"]]
    camera_from_base = np.linalg.inv(np.asarray(matrix, dtype=np.float64))
    pixels: list[list[float]] = []
    for idx in indices:
        hover = np.asarray(base_all[idx], dtype=np.float64) + front_axis_base * float(hover_m)
        uv = _base_to_pixel(hover, camera_from_base, intrinsics)
        pixels.append([float("nan"), float("nan")] if uv is None else uv)
    return pixels, [float(v) for v in front_axis_base.tolist()]


def _build_depth_normal_plan(
    *,
    detection: dict[str, object],
    depth_m: np.ndarray,
    intrinsics: dict[str, object],
    matrix: np.ndarray,
    line_selector: str,
    side: str,
    line_type: str,
    plan_points: int,
    hover_m: float,
    dian_jin_depth_m: float,
    fen_jin_lateral_m: float,
    safe_lift_m: float,
    window_px: int,
    stride_px: int,
    depth_band_m: float,
    min_points: int,
    normal_smooth_iterations: int,
    tool_contact_axis: str,
    reference_pose: list[float],
    orientation_mode: str,
) -> tuple[BladderMassagePlan, dict[str, object], list[list[float]], list[list[float]], list[list[float]]]:
    selected_side, selected_type = select_line(detection, line_selector, side, line_type)
    prefix = f"{selected_side}_{selected_type}"
    pixels_all = np.asarray(detection[f"{prefix}_pixel"], dtype=np.float64)
    camera_all = np.asarray(detection[f"{prefix}_camera"], dtype=np.float64)
    if len(pixels_all) < 2 or len(camera_all) < 2:
        raise RuntimeError(f"{prefix} has insufficient detected points")

    base_all = np.asarray(transform_points(camera_all.tolist(), matrix), dtype=np.float64)
    normals_all: list[np.ndarray] = []
    fit_errors_all: list[float] = []
    local_counts_all: list[int] = []
    prev_normal: np.ndarray | None = None
    for pixel in pixels_all:
        normal_camera, _centroid, count, rmse = fit_local_normal_camera(
            depth_m=depth_m,
            intrinsics=intrinsics,
            u=float(pixel[0]),
            v=float(pixel[1]),
            window_px=window_px,
            stride_px=stride_px,
            depth_band_m=depth_band_m,
            min_points=min_points,
        )
        normal_base = transform_direction(matrix, normal_camera)
        if prev_normal is not None and float(np.dot(normal_base, prev_normal)) < 0.0:
            normal_base = -normal_base
        prev_normal = normal_base
        normals_all.append(normal_base)
        fit_errors_all.append(float(rmse))
        local_counts_all.append(int(count))
    normals_all = smooth_normals(normals_all, normal_smooth_iterations)

    selected_indices = pick_evenly_indices(len(base_all), int(plan_points))
    ref_rot = rpy_to_matrix(*reference_pose[3:6])
    ref_axis_world = normalize(ref_rot @ axis_vector(tool_contact_axis))

    frames: list[BladderMassageFrame] = []
    safe_candidates = [float(reference_pose[2]) + float(safe_lift_m)]
    prev_split: np.ndarray | None = None
    for out_idx, src_idx in enumerate(selected_indices, start=1):
        point = np.asarray(base_all[src_idx], dtype=np.float64)
        pixel = np.asarray(pixels_all[src_idx], dtype=np.float64)
        front_normal = normalize(normals_all[src_idx])
        press = -front_normal

        prev_point = np.asarray(base_all[selected_indices[max(0, out_idx - 2)]], dtype=np.float64)
        next_point = np.asarray(base_all[selected_indices[min(len(selected_indices) - 1, out_idx)]], dtype=np.float64)
        tangent = next_point - prev_point
        if float(np.linalg.norm(tangent)) <= 1e-9:
            tangent = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        tangent = normalize(tangent)

        split = build_split_axis(press, tangent)
        if prev_split is not None and float(np.dot(split, prev_split)) < 0.0:
            split = -split
        prev_split = split

        if orientation_mode == "fixed_reference":
            target_rot = ref_rot
        elif orientation_mode == "depth_normal":
            target_rot = align_axis_rotation(ref_axis_world, press) @ ref_rot
        else:
            raise ValueError(f"unsupported orientation_mode: {orientation_mode}")
        rpy = matrix_to_rpy(target_rot)
        hover = point - press * hover_m
        dian = point - press * max(0.0, hover_m - dian_jin_depth_m)
        fen_pos = hover + split * fen_jin_lateral_m
        fen_neg = hover - split * fen_jin_lateral_m
        base_pose = [float(point[0]), float(point[1]), float(point[2]), *[float(v) for v in rpy]]
        safe_candidates.append(float(hover[2]) + 0.01)
        frames.append(
            BladderMassageFrame(
                index=out_idx,
                pixel=[float(pixel[0]), float(pixel[1])],
                robot_point_m=[float(v) for v in point.tolist()],
                hover_pose_m=[float(hover[0]), float(hover[1]), float(hover[2]), *[float(v) for v in rpy]],
                dian_jin_pose_m=[float(dian[0]), float(dian[1]), float(dian[2]), *[float(v) for v in rpy]],
                fen_positive_pose_m=[float(fen_pos[0]), float(fen_pos[1]), float(fen_pos[2]), *[float(v) for v in rpy]],
                fen_negative_pose_m=[float(fen_neg[0]), float(fen_neg[1]), float(fen_neg[2]), *[float(v) for v in rpy]],
                press_direction_m=[float(v) for v in press.tolist()],
                split_axis_m=[float(v) for v in split.tolist()],
                tangent_axis_m=[float(v) for v in tangent.tolist()],
                base_pose_m=base_pose,
                source_pose_quat=[],
            )
        )

    plan = BladderMassagePlan(
        side=selected_side,
        line_type=selected_type,
        point_count=len(frames),
        hover_m=float(hover_m),
        dian_jin_depth_m=float(dian_jin_depth_m),
        fen_jin_lateral_m=float(fen_jin_lateral_m),
        safe_z_m=float(max(safe_candidates)),
        anchor_pose_m=[float(v) for v in reference_pose[:6]],
        frames=frames,
        hover_offset_mode="depth_normal",
    )

    camera_from_base = np.linalg.inv(np.asarray(matrix, dtype=np.float64))
    surface_pixels = [[float(v) for v in pixels_all[idx].tolist()] for idx in selected_indices]
    hover_pixels: list[list[float]] = []
    for frame in frames:
        uv = _base_to_pixel(np.asarray(frame.hover_pose_m[:3], dtype=np.float64), camera_from_base, intrinsics)
        hover_pixels.append([float("nan"), float("nan")] if uv is None else uv)

    selected = {
        "side": selected_side,
        "line_type": selected_type,
        "line_selector": line_selector,
        "selected_prefix": prefix,
        "selected_meridian_pixel": [[float(v) for v in row] for row in pixels_all.tolist()],
        "selected_meridian_camera": [[float(v) for v in row] for row in camera_all.tolist()],
        "selected_meridian_robot": [[float(v) for v in row] for row in base_all.tolist()],
        "selected_plan_indices": [int(v) for v in selected_indices],
        "normal_source": {
            "type": "depth_local_pca",
            "window_px": int(window_px),
            "stride_px": int(stride_px),
            "depth_band_m": float(depth_band_m),
            "normal_smooth_iterations": int(normal_smooth_iterations),
            "mean_front_normal_base": [
                float(v) for v in normalize(np.asarray(normals_all, dtype=np.float64).mean(axis=0)).tolist()
            ],
            "normal_fit_rmse_mean_m": float(np.mean(fit_errors_all)),
            "normal_fit_rmse_max_m": float(np.max(fit_errors_all)),
            "local_depth_point_count_min": int(min(local_counts_all)),
            "local_depth_point_count_max": int(max(local_counts_all)),
        },
        "orientation_source": {
            "type": orientation_mode,
            "reference_rpy": [float(v) for v in reference_pose[3:6]],
            "tool_contact_axis": tool_contact_axis,
        },
    }
    return plan, selected, surface_pixels, hover_pixels, [[float(v) for v in row] for row in pixels_all.tolist()]


def _draw_live_overlay(
    frame_bgr: np.ndarray,
    all_surface_pixels: list[list[float]],
    sample_surface_pixels: list[list[float]],
    hover_pixels: list[list[float]],
    status_lines: list[str],
    *,
    alternate_hover_pixels: list[list[float]] | None = None,
    alternate_label: str = "",
    current_tcp_pixel: list[float] | None = None,
) -> np.ndarray:
    out = frame_bgr.copy()
    all_pts = np.asarray([[int(round(p[0])), int(round(p[1]))] for p in all_surface_pixels], dtype=np.int32)
    if len(all_pts) >= 2:
        cv2.polylines(out, [all_pts.reshape(-1, 1, 2)], False, (255, 0, 255), 3)
    surface_pts = np.asarray([[int(round(p[0])), int(round(p[1]))] for p in sample_surface_pixels], dtype=np.int32)
    hover_pts = np.asarray([[int(round(p[0])), int(round(p[1]))] for p in hover_pixels], dtype=np.int32)
    valid = len(surface_pts) == len(hover_pts) and len(surface_pts) > 0
    if valid:
        if len(hover_pts) >= 2:
            cv2.polylines(out, [hover_pts.reshape(-1, 1, 2)], False, (255, 255, 0), 3)
        for idx, (surface, hover) in enumerate(zip(surface_pts, hover_pts)):
            cv2.circle(out, tuple(int(v) for v in surface), 4, (255, 0, 255), -1)
            cv2.circle(out, tuple(int(v) for v in hover), 4, (0, 255, 255), -1)
            cv2.arrowedLine(
                out,
                tuple(int(v) for v in surface),
                tuple(int(v) for v in hover),
                (0, 255, 255),
                2,
                tipLength=0.25,
            )
            if idx == 0:
                cv2.putText(
                    out,
                    "depth start",
                    (int(hover[0]) + 6, int(hover[1]) + 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
    if alternate_hover_pixels:
        alt_pts = np.asarray(
            [[int(round(p[0])), int(round(p[1]))] for p in alternate_hover_pixels if np.isfinite(p[0]) and np.isfinite(p[1])],
            dtype=np.int32,
        )
        if len(alt_pts) >= 2:
            cv2.polylines(out, [alt_pts.reshape(-1, 1, 2)], False, (0, 165, 255), 3)
            for point in alt_pts:
                cv2.circle(out, tuple(int(v) for v in point), 4, (0, 165, 255), -1)
            label = alternate_label or "tool-axis hover"
            cv2.putText(
                out,
                label,
                (int(alt_pts[0][0]) + 6, int(alt_pts[0][1]) - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
    if current_tcp_pixel and np.isfinite(current_tcp_pixel[0]) and np.isfinite(current_tcp_pixel[1]):
        tcp = (int(round(current_tcp_pixel[0])), int(round(current_tcp_pixel[1])))
        cv2.drawMarker(out, tcp, (255, 255, 255), cv2.MARKER_CROSS, 22, 3, cv2.LINE_AA)
        cv2.circle(out, tcp, 8, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(
            out,
            "current TCP",
            (tcp[0] + 10, tcp[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    y = 28
    for line in status_lines:
        cv2.putText(out, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2, cv2.LINE_AA)
        y += 24
    cv2.putText(
        out,
        "magenta=surface cyan/yellow=depth-normal hover orange=tool-axis hover | q/ESC: stop",
        (14, out.shape[0] - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live side-lying bladder detection with depth-normal hover motion.")
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--matrix-path", default="camera_to_robot.json")
    parser.add_argument("--output-dir", default="rm_demo_output")
    parser.add_argument("--model-path", default="yolo11l-pose.pt")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    parser.add_argument("--line-selector", choices=("semantic", "top_outer", "bottom_outer"), default="top_outer")
    parser.add_argument("--finger-width", type=float, default=45.0)
    parser.add_argument("--sample-points", type=int, default=30)
    parser.add_argument("--plan-points", type=int, default=10)
    parser.add_argument("--frame-start", type=int, default=0, help="1-based first planned hover point to execute")
    parser.add_argument("--frame-count", type=int, default=0, help="number of planned hover points to execute")
    parser.add_argument("--hover-mm", type=float, default=120.0)
    parser.add_argument("--dian-jin-depth-mm", type=float, default=8.0)
    parser.add_argument("--fen-jin-lateral-mm", type=float, default=15.0)
    parser.add_argument("--safe-lift-mm", type=float, default=60.0)
    parser.add_argument("--window-px", type=int, default=31)
    parser.add_argument("--stride-px", type=int, default=2)
    parser.add_argument("--depth-band-m", type=float, default=0.08)
    parser.add_argument("--min-points", type=int, default=40)
    parser.add_argument("--normal-smooth-iterations", type=int, default=1)
    parser.add_argument(
        "--orientation-mode",
        choices=("depth_normal", "fixed_reference"),
        default="depth_normal",
        help="depth_normal aligns the saved tool axis to the fitted back normal; fixed_reference keeps the saved RPY unchanged",
    )
    parser.add_argument(
        "--tool-contact-axis",
        choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"),
        default="neg_z",
        help="tool axis aligned with body-press direction; neg_z keeps tool +Z as outward back normal",
    )
    parser.add_argument("--reference-contact-pose-json", default="rm_demo_output/user_confirmed_side_lying_contact_pose.json")
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--compare-tool-axis-hover", action="store_true")
    parser.add_argument("--show-current-tcp", action="store_true")
    parser.add_argument(
        "--execution-mode",
        choices=("hover_path", "approach_compare", "depth_near_path"),
        default="hover_path",
    )
    parser.add_argument("--approach-mm", type=float, default=40.0)
    parser.add_argument("--approach-source", choices=("depth", "tool", "both"), default="both")
    parser.add_argument("--path-order", choices=("first_to_last", "nearest_sweep"), default="first_to_last")
    parser.add_argument(
        "--allow-near-contact",
        action="store_true",
        help="allow execution with less than 60mm planned remaining hover; unsafe until TCP/tool offset is validated",
    )
    parser.add_argument(
        "--tool-front-axis",
        choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"),
        default="pos_z",
        help="tool-frame axis treated as the massage head outward/front direction for orange comparison line",
    )
    parser.add_argument("--max-step-m", type=float, default=0.03)
    parser.add_argument("--dwell-s", type=float, default=0.0)
    parser.add_argument("--entry-motion", choices=("movel", "movej_p"), default="movel")
    parser.add_argument("--target-point", choices=("all", "closest", "middle", "first", "last"), default="all")
    parser.add_argument("--max-entry-distance-m", type=float, default=0.25)
    parser.add_argument("--max-entry-orientation-delta-rad", type=float, default=0.7)
    parser.add_argument(
        "--allow-uncleared-direct-entry",
        action="store_true",
        help="allow Cartesian entry from the current pose to the first hover point; unsafe unless the corridor is known clear",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--realsense-serial", default="")
    parser.add_argument("--window-name", default="Depth-normal bladder hover")
    parser.add_argument("--save-preview-only", action="store_true")
    parser.add_argument(
        "--keep-window-open-after-motion",
        action="store_true",
        help="keep the live camera/overlay window open after the motion thread finishes; close with q or ESC",
    )
    parser.add_argument("--run", action="store_true")
    return parser.parse_args()


def _select_plan_frames(plan: BladderMassagePlan, frame_start: int, frame_count: int) -> BladderMassagePlan:
    if frame_start <= 0 and frame_count <= 0:
        return plan
    start = max(1, int(frame_start)) if frame_start > 0 else 1
    start_idx = start - 1
    if start_idx >= len(plan.frames):
        raise RuntimeError(f"frame_start {start} outside frame count {len(plan.frames)}")
    frames = plan.frames[start_idx : start_idx + int(frame_count)] if frame_count > 0 else plan.frames[start_idx:]
    if not frames:
        raise RuntimeError("selected plan frame range is empty")
    return BladderMassagePlan(
        side=plan.side,
        line_type=plan.line_type,
        point_count=len(frames),
        hover_m=plan.hover_m,
        dian_jin_depth_m=plan.dian_jin_depth_m,
        fen_jin_lateral_m=plan.fen_jin_lateral_m,
        safe_z_m=plan.safe_z_m,
        anchor_pose_m=plan.anchor_pose_m,
        frames=frames,
        hover_offset_mode=plan.hover_offset_mode,
    )


def _wrap_single_frame(plan: BladderMassagePlan, frame_index: int) -> BladderMassagePlan:
    frame = plan.frames[int(frame_index)]
    return BladderMassagePlan(
        side=plan.side,
        line_type=plan.line_type,
        point_count=1,
        hover_m=plan.hover_m,
        dian_jin_depth_m=plan.dian_jin_depth_m,
        fen_jin_lateral_m=plan.fen_jin_lateral_m,
        safe_z_m=plan.safe_z_m,
        anchor_pose_m=plan.anchor_pose_m,
        frames=[frame],
        hover_offset_mode=plan.hover_offset_mode,
    )


def _angle_delta_rad(start: float, target: float) -> float:
    return float((float(target) - float(start) + np.pi) % (2.0 * np.pi) - np.pi)


def _select_motion_plan(
    plan: BladderMassagePlan,
    *,
    frame_start: int,
    frame_count: int,
    target_point: str,
    current_pose: list[float],
) -> BladderMassagePlan:
    selected = _select_plan_frames(plan, frame_start, frame_count)
    target = str(target_point).strip().lower()
    if target == "all":
        return selected
    if target == "first":
        return _wrap_single_frame(selected, 0)
    if target == "last":
        return _wrap_single_frame(selected, len(selected.frames) - 1)
    if target == "middle":
        return _wrap_single_frame(selected, len(selected.frames) // 2)
    if target == "closest":
        current = np.asarray(current_pose[:3], dtype=np.float64)
        distances = [
            float(np.linalg.norm(np.asarray(frame.hover_pose_m[:3], dtype=np.float64) - current))
            for frame in selected.frames
        ]
        return _wrap_single_frame(selected, int(np.argmin(distances)))
    raise ValueError(f"unsupported target_point: {target_point}")


def _make_pose_from_xyz(xyz: np.ndarray, rpy: list[float]) -> list[float]:
    return [float(xyz[0]), float(xyz[1]), float(xyz[2]), *[float(v) for v in rpy]]


def _make_motion_frame(index: int, template: BladderMassageFrame, pose: list[float], press: np.ndarray) -> BladderMassageFrame:
    return BladderMassageFrame(
        index=int(index),
        pixel=list(template.pixel),
        robot_point_m=list(template.robot_point_m),
        hover_pose_m=list(pose),
        dian_jin_pose_m=list(pose),
        fen_positive_pose_m=list(pose),
        fen_negative_pose_m=list(pose),
        press_direction_m=[float(v) for v in press.tolist()],
        split_axis_m=list(template.split_axis_m),
        tangent_axis_m=list(template.tangent_axis_m),
        base_pose_m=list(template.base_pose_m),
        source_pose_quat=[],
    )


def _build_approach_compare_plan(
    *,
    base_plan: BladderMassagePlan,
    motion_plan: BladderMassagePlan,
    reference_pose: list[float],
    tool_front_axis: str,
    approach_m: float,
    sources: str,
) -> tuple[BladderMassagePlan, list[str]]:
    if not motion_plan.frames:
        raise RuntimeError("empty motion plan")
    frame = motion_plan.frames[0]
    surface = np.asarray(frame.robot_point_m, dtype=np.float64)
    rpy = [float(v) for v in reference_pose[3:6]]
    approach = float(approach_m)
    if approach <= 0.0:
        raise RuntimeError("--approach-mm must be positive")
    if approach >= float(base_plan.hover_m):
        raise RuntimeError("approach distance must be smaller than hover distance")

    ref_rot = rpy_to_matrix(*rpy)
    tool_front = normalize(ref_rot @ axis_vector(tool_front_axis))
    source_list = ["depth", "tool"] if sources == "both" else [sources]
    frames: list[BladderMassageFrame] = []
    labels: list[str] = []
    for source in source_list:
        if source == "depth":
            hover = np.asarray(frame.hover_pose_m[:3], dtype=np.float64)
            to_surface = normalize(surface - hover)
            near = hover + to_surface * approach
            name = "blue/depth"
        elif source == "tool":
            hover = surface + tool_front * float(base_plan.hover_m)
            to_surface = -tool_front
            near = hover + to_surface * approach
            name = f"orange/tool_{tool_front_axis}"
        else:
            raise ValueError(f"unsupported approach source: {source}")
        for suffix, xyz in (("hover", hover), ("near", near), ("hover", hover)):
            pose = _make_pose_from_xyz(xyz, rpy)
            frames.append(_make_motion_frame(len(frames) + 1, frame, pose, to_surface))
            labels.append(f"{name}_{suffix}")

    plan = BladderMassagePlan(
        side=base_plan.side,
        line_type=base_plan.line_type,
        point_count=len(frames),
        hover_m=base_plan.hover_m,
        dian_jin_depth_m=0.0,
        fen_jin_lateral_m=0.0,
        safe_z_m=base_plan.safe_z_m,
        anchor_pose_m=[float(v) for v in reference_pose[:6]],
        frames=frames,
        hover_offset_mode=f"approach_compare_{sources}",
    )
    return plan, labels


def _build_depth_near_path_plan(
    *,
    base_plan: BladderMassagePlan,
    reference_pose: list[float],
    approach_m: float,
) -> tuple[BladderMassagePlan, list[str]]:
    approach = float(approach_m)
    if approach <= 0.0:
        raise RuntimeError("--approach-mm must be positive")
    if approach >= float(base_plan.hover_m):
        raise RuntimeError("approach distance must be smaller than hover distance")
    rpy = [float(v) for v in reference_pose[3:6]]
    frames: list[BladderMassageFrame] = []
    labels: list[str] = []
    for frame in base_plan.frames:
        surface = np.asarray(frame.robot_point_m, dtype=np.float64)
        hover = np.asarray(frame.hover_pose_m[:3], dtype=np.float64)
        to_surface = normalize(surface - hover)
        near = hover + to_surface * approach
        pose = _make_pose_from_xyz(near, rpy)
        frames.append(_make_motion_frame(len(frames) + 1, frame, pose, to_surface))
        labels.append(f"blue/depth_near_path_{frame.index}")
    plan = BladderMassagePlan(
        side=base_plan.side,
        line_type=base_plan.line_type,
        point_count=len(frames),
        hover_m=max(0.0, float(base_plan.hover_m) - approach),
        dian_jin_depth_m=0.0,
        fen_jin_lateral_m=0.0,
        safe_z_m=base_plan.safe_z_m,
        anchor_pose_m=[float(v) for v in reference_pose[:6]],
        frames=frames,
        hover_offset_mode="depth_near_path",
    )
    return plan, labels


def _reorder_plan_nearest_sweep(
    plan: BladderMassagePlan,
    labels: list[str],
    current_pose: list[float],
) -> tuple[BladderMassagePlan, list[str]]:
    if len(plan.frames) <= 1:
        return plan, labels
    current = np.asarray(current_pose[:3], dtype=np.float64)
    distances = [
        float(np.linalg.norm(np.asarray(frame.hover_pose_m[:3], dtype=np.float64) - current))
        for frame in plan.frames
    ]
    nearest = int(np.argmin(distances))
    frames = list(reversed(plan.frames[: nearest + 1])) + plan.frames[nearest + 1 :]
    if labels and len(labels) == len(plan.frames):
        ordered_labels = list(reversed(labels[: nearest + 1])) + labels[nearest + 1 :]
    else:
        ordered_labels = [f"frame_{frame.index}" for frame in frames]
    for idx, frame in enumerate(frames, start=1):
        frame.index = idx
    return (
        BladderMassagePlan(
            side=plan.side,
            line_type=plan.line_type,
            point_count=len(frames),
            hover_m=plan.hover_m,
            dian_jin_depth_m=plan.dian_jin_depth_m,
            fen_jin_lateral_m=plan.fen_jin_lateral_m,
            safe_z_m=plan.safe_z_m,
            anchor_pose_m=plan.anchor_pose_m,
            frames=frames,
            hover_offset_mode=f"{plan.hover_offset_mode}_nearest_sweep",
        ),
        ordered_labels,
    )


def main() -> int:
    args = parse_args()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = f"external_depth_normal_live_{stamp}_{args.line_selector}"

    matrix = load_transform_matrix(str(Path(args.matrix_path).resolve()))
    if matrix is None:
        raise RuntimeError(f"camera->robot matrix not found or invalid: {args.matrix_path}")
    reference_pose = _load_reference_pose(args.reference_contact_pose_json)

    pipeline, profile, color_format = _stream_profile(args)
    align = rs.align(rs.stream.color)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = float(depth_sensor.get_depth_scale())
    intrinsics = _intrinsics_from_profile(profile, depth_scale)
    print(f"realsense_started color_format={color_format} depth_scale={depth_scale}")

    motion_thread: threading.Thread | None = None
    motion_error: list[BaseException] = []
    motion_done = threading.Event()
    stop_requested = False

    try:
        frame_pair = None
        for _ in range(max(1, int(args.warmup_frames))):
            frame_pair = _read_aligned_frame(pipeline, align, color_format, depth_scale)
        if frame_pair is None:
            raise RuntimeError("no RealSense frame")
        color_bgr, depth_m = frame_pair

        detection, raw_overlay = detect_bladder_lines(
            color_bgr=color_bgr,
            depth_m=depth_m,
            intrinsics_data=intrinsics,
            finger_width_mm=args.finger_width,
            model_path=args.model_path,
            sample_points=args.sample_points,
        )
        detection["capture"] = {
            "backend": "local_realsense_first_frame_depth_normal",
            "timestamp": stamp,
            "color_format": str(color_format),
        }

        overlay_path, detect_json_path = save_bladder_artifacts(str(out_dir), detection, raw_overlay, prefix=prefix)
        raw_path = out_dir / f"{prefix}_raw.png"
        depth_path = out_dir / f"{prefix}_depth.npy"
        intrinsics_path = out_dir / f"{prefix}_intrinsics.json"
        cv2.imwrite(str(raw_path), color_bgr)
        np.save(str(depth_path), depth_m)
        intrinsics_path.write_text(json.dumps(intrinsics, ensure_ascii=False, indent=2), encoding="utf-8")

        plan, selected, surface_pixels, hover_pixels, all_surface_pixels = _build_depth_normal_plan(
            detection=detection,
            depth_m=depth_m,
            intrinsics=intrinsics,
            matrix=matrix,
            line_selector=args.line_selector,
            side=args.side,
            line_type=args.line_type,
            plan_points=args.plan_points,
            hover_m=args.hover_mm / 1000.0,
            dian_jin_depth_m=args.dian_jin_depth_mm / 1000.0,
            fen_jin_lateral_m=args.fen_jin_lateral_mm / 1000.0,
            safe_lift_m=args.safe_lift_mm / 1000.0,
            window_px=args.window_px,
            stride_px=args.stride_px,
            depth_band_m=args.depth_band_m,
            min_points=args.min_points,
            normal_smooth_iterations=args.normal_smooth_iterations,
            tool_contact_axis=args.tool_contact_axis,
            reference_pose=reference_pose,
            orientation_mode=args.orientation_mode,
        )

        joints, current_pose, arm_err, sys_err, ik_err = rm_json.get_current_arm_state(args.arm_host)
        motion_plan = _select_motion_plan(
            plan,
            frame_start=args.frame_start,
            frame_count=args.frame_count,
            target_point=args.target_point,
            current_pose=current_pose,
        )
        selected["source_detection_json"] = str(detect_json_path)
        selected["source_matrix_path"] = str(Path(args.matrix_path).resolve())
        selected["current_joint_deg"] = [float(v) for v in joints]
        selected["current_pose_m_rpy"] = [float(v) for v in current_pose]
        selected["arm_error_state"] = {"arm_err": int(arm_err), "sys_err": int(sys_err), "inverse_km_err": int(ik_err)}

        transform_path = out_dir / f"{prefix}_transform.json"
        plan_path = out_dir / f"{prefix}_depth_normal_plan.json"
        transform_path.write_text(json.dumps(selected, ensure_ascii=False, indent=2), encoding="utf-8")
        plan_data = bladder_plan_to_dict(plan)
        plan_data["source_detection_json"] = str(detect_json_path)
        plan_data["source_transform_json"] = str(transform_path)
        plan_data["normal_source"] = selected["normal_source"]
        plan_data["orientation_source"] = selected["orientation_source"]
        plan_data["aligned_contact_preview"] = build_aligned_contact_preview(
            plan,
            tool_contact_axis=args.tool_contact_axis,
            contact_motion_axis=args.tool_contact_axis,
            max_press_m=0.0,
            touch_step_m=0.0,
        )
        plan_path.write_text(json.dumps(plan_data, ensure_ascii=False, indent=2), encoding="utf-8")

        execution_plan = motion_plan
        execution_labels: list[str] = []
        if args.execution_mode == "approach_compare":
            execution_plan, execution_labels = _build_approach_compare_plan(
                base_plan=plan,
                motion_plan=motion_plan,
                reference_pose=reference_pose,
                tool_front_axis=args.tool_front_axis,
                approach_m=args.approach_mm / 1000.0,
                sources=args.approach_source,
            )
        elif args.execution_mode == "depth_near_path":
            execution_plan, execution_labels = _build_depth_near_path_plan(
                base_plan=plan,
                reference_pose=reference_pose,
                approach_m=args.approach_mm / 1000.0,
            )
        if args.path_order == "nearest_sweep":
            execution_plan, execution_labels = _reorder_plan_nearest_sweep(
                execution_plan,
                execution_labels,
                current_pose,
            )

        first_hover = execution_plan.frames[0].hover_pose_m[:3]
        alternate_hover_pixels: list[list[float]] | None = None
        tool_front_axis_base: list[float] | None = None
        if args.compare_tool_axis_hover:
            alternate_hover_pixels, tool_front_axis_base = _build_tool_axis_hover_pixels(
                selected=selected,
                reference_pose=reference_pose,
                tool_front_axis=args.tool_front_axis,
                hover_m=args.hover_mm / 1000.0,
                matrix=matrix,
                intrinsics=intrinsics,
            )
        entry_distance = float(
            np.linalg.norm(np.asarray(first_hover, dtype=np.float64) - np.asarray(current_pose[:3], dtype=np.float64))
        )
        entry_orientation_delta = max(
            abs(_angle_delta_rad(float(current_pose[i]), float(motion_plan.frames[0].hover_pose_m[i])))
            for i in range(3, 6)
        )
        status_base = [
            f"{plan.side}_{plan.line_type} selector={args.line_selector} points={execution_plan.point_count}/{plan.point_count}",
            f"hover={args.hover_mm:.0f}mm speed={args.speed:g} entry={args.entry_motion} target={args.target_point}",
            f"orientation={args.orientation_mode} entry_dist={entry_distance:.3f}m d_rpy={entry_orientation_delta:.3f}",
            f"first_hover=({first_hover[0]:.3f},{first_hover[1]:.3f},{first_hover[2]:.3f})",
        ]
        if args.execution_mode == "approach_compare":
            status_base.append(f"approach_compare={args.approach_source} approach={args.approach_mm:.0f}mm")
        elif args.execution_mode == "depth_near_path":
            status_base.append(f"depth_near_path approach={args.approach_mm:.0f}mm remaining_hover={args.hover_mm - args.approach_mm:.0f}mm")
        if tool_front_axis_base is not None:
            status_base.append(f"orange={args.tool_front_axis} axis ({tool_front_axis_base[0]:.2f},{tool_front_axis_base[1]:.2f},{tool_front_axis_base[2]:.2f})")
        camera_from_base = np.linalg.inv(np.asarray(matrix, dtype=np.float64))
        display_hover_pixels = hover_pixels
        if args.execution_mode == "depth_near_path":
            display_hover_pixels = []
            for exec_frame in execution_plan.frames:
                uv = _base_to_pixel(np.asarray(exec_frame.hover_pose_m[:3], dtype=np.float64), camera_from_base, intrinsics)
                display_hover_pixels.append([float("nan"), float("nan")] if uv is None else uv)
        current_tcp_pixel = (
            _base_to_pixel(np.asarray(current_pose[:3], dtype=np.float64), camera_from_base, intrinsics)
            if args.show_current_tcp
            else None
        )
        preview_img = _draw_live_overlay(
            color_bgr,
            all_surface_pixels,
            surface_pixels,
            display_hover_pixels,
            [*status_base, "motion=preview"],
            alternate_hover_pixels=alternate_hover_pixels,
            alternate_label=f"tool {args.tool_front_axis}",
            current_tcp_pixel=current_tcp_pixel,
        )
        preview_path = out_dir / f"{prefix}_depth_normal_live_preview.png"
        cv2.imwrite(str(preview_path), preview_img)

        print(f"detection_overlay={overlay_path}")
        print(f"detection_json={detect_json_path}")
        print(f"raw_png={raw_path}")
        print(f"depth_npy={depth_path}")
        print(f"intrinsics_json={intrinsics_path}")
        print(f"transform_json={transform_path}")
        print(f"plan_json={plan_path}")
        print(f"live_preview_png={preview_path}")
        print(f"selected={plan.side}_{plan.line_type} selector={args.line_selector}")
        print(f"current_pose={[round(float(v), 6) for v in current_pose]} arm_err={arm_err} sys_err={sys_err} ik_err={ik_err}")
        print(f"motion_target_point={args.target_point} motion_frame_index={motion_plan.frames[0].index}")
        if execution_labels:
            print(f"execution_mode={args.execution_mode} labels={execution_labels}")
        print(f"entry_distance_m={entry_distance:.4f} entry_orientation_delta_rad={entry_orientation_delta:.4f}")
        print(f"mean_front_normal={selected['normal_source']['mean_front_normal_base']}")
        print(
            "normal_fit_rmse_mm="
            f"{float(selected['normal_source']['normal_fit_rmse_mean_m']) * 1000.0:.2f}/"
            f"{float(selected['normal_source']['normal_fit_rmse_max_m']) * 1000.0:.2f}"
        )
        preview_bladder_plan(plan)
        if args.save_preview_only:
            print("save_preview_only=True; no robot motion")
            return 0
        remaining_hover_m = float(args.hover_mm - args.approach_mm) / 1000.0 if args.execution_mode == "depth_near_path" else float(plan.hover_m)
        if args.run and args.execution_mode == "depth_near_path" and remaining_hover_m < 0.06 and not args.allow_near_contact:
            raise RuntimeError(
                f"refusing motion: planned remaining hover {remaining_hover_m:.3f}m is below 0.060m. "
                "Use a larger remaining hover, or pass --allow-near-contact only after TCP/tool offset is validated."
            )
        if args.run and not args.allow_uncleared_direct_entry:
            raise RuntimeError(
                "refusing motion: direct entry from current pose is not cleared. "
                "Use preview output first and provide a validated staging/approach pose, "
                "or pass --allow-uncleared-direct-entry only for a known-clear corridor."
            )
        if args.run and entry_distance > float(args.max_entry_distance_m):
            raise RuntimeError(
                f"refusing motion: entry distance {entry_distance:.3f}m exceeds "
                f"--max-entry-distance-m {float(args.max_entry_distance_m):.3f}m"
            )
        if args.run and entry_orientation_delta > float(args.max_entry_orientation_delta_rad):
            raise RuntimeError(
                f"refusing motion: entry orientation delta {entry_orientation_delta:.3f}rad exceeds "
                f"--max-entry-orientation-delta-rad {float(args.max_entry_orientation_delta_rad):.3f}rad"
            )

        def _run_motion() -> None:
            try:
                execute_bladder_hover_path(
                    host=args.arm_host,
                    plan=execution_plan,
                    speed=args.speed,
                    control_backend="json",
                    dwell_s=args.dwell_s,
                    use_global_safe_z=False,
                    keep_current_orientation=False,
                    max_step_m=args.max_step_m,
                    entry_motion=args.entry_motion,
                )
            except BaseException as exc:
                motion_error.append(exc)
            finally:
                motion_done.set()

        if args.run:
            motion_thread = threading.Thread(target=_run_motion, daemon=True)
            motion_thread.start()
        else:
            motion_done.set()

        cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
        last_tcp_query_s = 0.0
        while True:
            frame_pair = _read_aligned_frame(pipeline, align, color_format, depth_scale)
            if frame_pair is None:
                continue
            live_bgr, _live_depth = frame_pair
            state = "running" if args.run and not motion_done.is_set() else "preview" if not args.run else "done"
            if motion_error:
                state = f"error:{type(motion_error[0]).__name__}"
            if args.show_current_tcp and time.time() - last_tcp_query_s > 0.35:
                last_tcp_query_s = time.time()
                try:
                    _joints, latest_pose, _arm_err, _sys_err, _ik_err = rm_json.get_current_arm_state(args.arm_host)
                    current_tcp_pixel = _base_to_pixel(np.asarray(latest_pose[:3], dtype=np.float64), camera_from_base, intrinsics)
                except Exception:
                    current_tcp_pixel = None
            display = _draw_live_overlay(
                live_bgr,
                all_surface_pixels,
                surface_pixels,
                display_hover_pixels,
                [*status_base, f"motion={state}"],
                alternate_hover_pixels=alternate_hover_pixels,
                alternate_label=f"tool {args.tool_front_axis}",
                current_tcp_pixel=current_tcp_pixel,
            )
            cv2.imshow(args.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                stop_requested = True
                if args.run and not motion_done.is_set():
                    rm_json.stop_motion(args.arm_host)
                break
            if args.run and motion_done.is_set() and not args.keep_window_open_after_motion:
                time.sleep(1.0)
                break
            if not args.run and key == ord("s"):
                cv2.imwrite(str(preview_path), display)
        if motion_thread is not None:
            motion_thread.join(timeout=2.0)
        if motion_error:
            raise motion_error[0]
        if stop_requested:
            print("stop_requested_by_user=True")
        return 0
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
