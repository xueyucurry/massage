#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

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
from rm_demo.rm_bladder import BladderMassageFrame, BladderMassagePlan, bladder_plan_to_dict, preview_bladder_plan
from rm_demo.rm_transform import load_transform_matrix, transform_points


def pick_evenly_indices(length: int, count: int) -> list[int]:
    if length < 1:
        raise RuntimeError("not enough points")
    if count <= 1:
        return [length // 2]
    if count >= length:
        return list(range(length))
    return [int(round(i * (length - 1) / max(1, count - 1))) for i in range(count)]


def build_split_axis(press: np.ndarray, tangent: np.ndarray) -> np.ndarray:
    split = np.cross(press, tangent)
    if float(np.linalg.norm(split)) > 1e-9:
        return normalize(split)
    for candidate in (
        np.asarray([0.0, 0.0, 1.0], dtype=np.float64),
        np.asarray([0.0, 1.0, 0.0], dtype=np.float64),
        np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
    ):
        split = np.cross(press, candidate)
        if float(np.linalg.norm(split)) > 1e-9:
            return normalize(split)
    return np.asarray([0.0, 1.0, 0.0], dtype=np.float64)


def smooth_normals(normals: list[np.ndarray], iterations: int) -> list[np.ndarray]:
    out = [normalize(n) for n in normals]
    for idx in range(1, len(out)):
        if float(np.dot(out[idx], out[idx - 1])) < 0.0:
            out[idx] = -out[idx]
    for _ in range(max(0, int(iterations))):
        smoothed = []
        for idx, normal in enumerate(out):
            acc = np.asarray(normal, dtype=np.float64)
            weight = 1.0
            if idx > 0:
                acc = acc + out[idx - 1] * 0.5
                weight += 0.5
            if idx + 1 < len(out):
                acc = acc + out[idx + 1] * 0.5
                weight += 0.5
            smoothed.append(normalize(acc / weight))
        out = smoothed
        for idx in range(1, len(out)):
            if float(np.dot(out[idx], out[idx - 1])) < 0.0:
                out[idx] = -out[idx]
    return out


def load_reference_pose(path: str) -> list[float]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    pose = data.get("tcp_pose_m_rpy")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"reference pose JSON missing tcp_pose_m_rpy: {path}")
    return [float(v) for v in pose[:6]]


def base_to_pixel(point_base: np.ndarray, camera_from_base: np.ndarray, intrinsics: dict[str, object]) -> list[float] | None:
    p = camera_from_base @ np.asarray([point_base[0], point_base[1], point_base[2], 1.0], dtype=np.float64)
    return project_camera(p[:3], intrinsics)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a side-lying bladder plan from depth-fitted local surface normals.")
    parser.add_argument("--stem", required=True, help="file stem without _detect.json/_depth.npy suffix")
    parser.add_argument("--matrix-path", default="camera_to_robot.json")
    parser.add_argument("--line-selector", choices=("semantic", "top_outer", "bottom_outer"), default="top_outer")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    parser.add_argument("--plan-points", type=int, default=10)
    parser.add_argument("--hover-mm", type=float, default=120.0)
    parser.add_argument("--dian-jin-depth-mm", type=float, default=8.0)
    parser.add_argument("--fen-jin-lateral-mm", type=float, default=15.0)
    parser.add_argument("--safe-lift-mm", type=float, default=60.0)
    parser.add_argument("--window-px", type=int, default=31)
    parser.add_argument("--stride-px", type=int, default=2)
    parser.add_argument("--depth-band-m", type=float, default=0.08)
    parser.add_argument("--min-points", type=int, default=40)
    parser.add_argument("--normal-smooth-iterations", type=int, default=1)
    parser.add_argument("--tool-contact-axis", choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"), default="neg_z")
    parser.add_argument("--reference-contact-pose-json", default="rm_demo_output/user_confirmed_side_lying_contact_pose.json")
    parser.add_argument("--output-plan", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stem = Path(args.stem)
    detect_path = stem.with_name(stem.name + "_detect.json")
    depth_path = stem.with_name(stem.name + "_depth.npy")
    raw_path = stem.with_name(stem.name + "_raw.png")
    intrinsics_path = stem.with_name(stem.name + "_intrinsics.json")
    detection = json.loads(detect_path.read_text(encoding="utf-8"))
    depth_m = np.load(str(depth_path)).astype(np.float32)
    raw = cv2.imread(str(raw_path))
    if raw is None:
        raise RuntimeError(f"failed to read raw image: {raw_path}")
    intrinsics = json.loads(intrinsics_path.read_text(encoding="utf-8")) if intrinsics_path.exists() else dict(detection["intrinsics"])
    matrix = load_transform_matrix(args.matrix_path)
    if matrix is None:
        raise RuntimeError(f"invalid camera->robot matrix: {args.matrix_path}")

    selected_side, selected_type = select_line(detection, args.line_selector, args.side, args.line_type)
    prefix = f"{selected_side}_{selected_type}"
    pixels_all = np.asarray(detection[f"{prefix}_pixel"], dtype=np.float64)
    camera_all = np.asarray(detection[f"{prefix}_camera"], dtype=np.float64)
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
            window_px=args.window_px,
            stride_px=args.stride_px,
            depth_band_m=args.depth_band_m,
            min_points=args.min_points,
        )
        normal_base = transform_direction(matrix, normal_camera)
        if prev_normal is not None and float(np.dot(normal_base, prev_normal)) < 0.0:
            normal_base = -normal_base
        prev_normal = normal_base
        normals_all.append(normal_base)
        fit_errors_all.append(float(rmse))
        local_counts_all.append(int(count))
    normals_all = smooth_normals(normals_all, args.normal_smooth_iterations)

    selected_indices = pick_evenly_indices(len(base_all), int(args.plan_points))
    ref_pose = load_reference_pose(args.reference_contact_pose_json)
    ref_rot = rpy_to_matrix(*ref_pose[3:6])
    ref_axis_world = normalize(ref_rot @ axis_vector(args.tool_contact_axis))
    hover_m = float(args.hover_mm) / 1000.0
    dian_depth_m = float(args.dian_jin_depth_mm) / 1000.0
    fen_m = float(args.fen_jin_lateral_mm) / 1000.0

    frames: list[BladderMassageFrame] = []
    safe_candidates = [float(ref_pose[2]) + float(args.safe_lift_mm) / 1000.0]
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

        target_rot = align_axis_rotation(ref_axis_world, press) @ ref_rot
        rpy = matrix_to_rpy(target_rot)
        hover = point - press * hover_m
        dian = point - press * max(0.0, hover_m - dian_depth_m)
        fen_pos = hover + split * fen_m
        fen_neg = hover - split * fen_m
        base_pose = [float(point[0]), float(point[1]), float(point[2]), *rpy]
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
        hover_m=hover_m,
        dian_jin_depth_m=dian_depth_m,
        fen_jin_lateral_m=fen_m,
        safe_z_m=float(max(safe_candidates)),
        anchor_pose_m=[float(v) for v in ref_pose[:6]],
        frames=frames,
        hover_offset_mode="depth_normal",
    )

    plan_data = bladder_plan_to_dict(plan)
    plan_data["normal_source"] = {
        "type": "depth_local_pca",
        "line_selector": args.line_selector,
        "source_detect_json": str(detect_path.resolve()),
        "source_depth_npy": str(depth_path.resolve()),
        "source_matrix": str(Path(args.matrix_path).resolve()),
        "window_px": int(args.window_px),
        "stride_px": int(args.stride_px),
        "depth_band_m": float(args.depth_band_m),
        "normal_smooth_iterations": int(args.normal_smooth_iterations),
        "mean_front_normal_base": [float(v) for v in normalize(np.asarray(normals_all, dtype=np.float64).mean(axis=0)).tolist()],
        "normal_fit_rmse_mean_m": float(np.mean(fit_errors_all)),
        "normal_fit_rmse_max_m": float(np.max(fit_errors_all)),
        "local_depth_point_count_min": int(min(local_counts_all)),
        "local_depth_point_count_max": int(max(local_counts_all)),
    }
    plan_data["orientation_source"] = {
        "type": "reference_rpy_axis_aligned_to_depth_normal",
        "reference_contact_pose_json": str(Path(args.reference_contact_pose_json).resolve()),
        "reference_rpy": [float(v) for v in ref_pose[3:6]],
        "tool_contact_axis": args.tool_contact_axis,
    }

    out_plan = Path(args.output_plan).resolve() if args.output_plan else stem.with_name(stem.name + "_depth_normal_plan.json")
    out_plan.write_text(json.dumps(plan_data, ensure_ascii=False, indent=2), encoding="utf-8")

    camera_from_base = np.linalg.inv(np.asarray(matrix, dtype=np.float64))
    surface_pixels = [[float(v) for v in pixels_all[idx].tolist()] for idx in selected_indices]
    hover_pixels = []
    for frame in frames:
        uv = base_to_pixel(np.asarray(frame.hover_pose_m[:3], dtype=np.float64), camera_from_base, intrinsics)
        hover_pixels.append([float("nan"), float("nan")] if uv is None else uv)

    img = raw.copy()
    all_line = np.round(pixels_all).astype(np.int32)
    cv2.polylines(img, [all_line.reshape(-1, 1, 2)], False, (255, 0, 255), 3)
    surf_pts = np.round(np.asarray(surface_pixels, dtype=np.float64)).astype(np.int32)
    hover_pts = np.round(np.asarray(hover_pixels, dtype=np.float64)).astype(np.int32)
    cv2.polylines(img, [hover_pts.reshape(-1, 1, 2)], False, (255, 255, 0), 3)
    for p0, p1 in zip(surf_pts, hover_pts):
        cv2.arrowedLine(img, tuple(int(v) for v in p0), tuple(int(v) for v in p1), (0, 255, 255), 2, tipLength=0.25)
    cv2.putText(img, "magenta=surface top_outer cyan/yellow=depth-normal TCP hover", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, f"hover={args.hover_mm:.0f}mm no robot motion", (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)
    out_overlay = out_plan.with_name(out_plan.stem + "_overlay.png")
    cv2.imwrite(str(out_overlay), img)

    surface_arr = np.asarray([frame.robot_point_m for frame in frames], dtype=np.float64)
    hover_arr = np.asarray([frame.hover_pose_m[:3] for frame in frames], dtype=np.float64)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    projections = [("Base X", "Base Y", 0, 1), ("Base X", "Base Z", 0, 2), ("Base Y", "Base Z", 1, 2)]
    for ax, (xlabel, ylabel, ix, iy) in zip(axes, projections):
        ax.plot(surface_arr[:, ix], surface_arr[:, iy], "-o", color="#d100d1", label="surface")
        ax.plot(hover_arr[:, ix], hover_arr[:, iy], "-o", color="#00cccc", label="TCP hover")
        for s, h in zip(surface_arr, hover_arr):
            ax.arrow(s[ix], s[iy], h[ix] - s[ix], h[iy] - s[iy], head_width=0.004, length_includes_head=True, color="#888800")
        ax.set_xlabel(xlabel + " (m)")
        ax.set_ylabel(ylabel + " (m)")
        ax.grid(True, alpha=0.35)
        ax.axis("equal")
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("Depth-normal bladder plan in RM Base frame")
    out_plot = out_plan.with_name(out_plan.stem + "_base_projection.png")
    fig.savefig(str(out_plot), dpi=160)

    print(f"plan_json={out_plan}")
    print(f"overlay_png={out_overlay}")
    print(f"base_projection={out_plot}")
    print(f"selected={selected_side}_{selected_type} points={len(frames)} hover_m={hover_m:.3f}")
    print(f"mean_front_normal={plan_data['normal_source']['mean_front_normal_base']}")
    print(f"normal_fit_rmse_mean_mm={float(np.mean(fit_errors_all))*1000:.2f} max_mm={float(np.max(fit_errors_all))*1000:.2f}")
    preview_bladder_plan(plan)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
