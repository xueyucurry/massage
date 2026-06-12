#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from rm_demo.rm_transform import load_transform_matrix, transform_points


LINE_KEYS = ("left_outer", "left_inner", "right_inner", "right_outer")


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        raise RuntimeError("zero-length vector")
    return vec / norm


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


def axis_vector(name: str) -> np.ndarray:
    axes = {
        "pos_x": [1.0, 0.0, 0.0],
        "neg_x": [-1.0, 0.0, 0.0],
        "pos_y": [0.0, 1.0, 0.0],
        "neg_y": [0.0, -1.0, 0.0],
        "pos_z": [0.0, 0.0, 1.0],
        "neg_z": [0.0, 0.0, -1.0],
    }
    key = str(name).strip().lower()
    if key not in axes:
        raise ValueError(f"unsupported axis: {name}")
    return np.asarray(axes[key], dtype=np.float64)


def align_axis_rotation(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    src = normalize(np.asarray(source, dtype=np.float64))
    dst = normalize(np.asarray(target, dtype=np.float64))
    cross = np.cross(src, dst)
    dot = max(-1.0, min(1.0, float(np.dot(src, dst))))
    sin = float(np.linalg.norm(cross))
    if sin < 1e-9:
        if dot > 0.0:
            return np.eye(3, dtype=np.float64)
        helper = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(src, helper))) > 0.9:
            helper = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        cross = normalize(np.cross(src, helper))
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


def pixel_to_camera(u: float, v: float, z: float, intr: dict[str, object]) -> np.ndarray:
    fx = float(intr["fx"])
    fy = float(intr["fy"])
    ppx = float(intr["ppx"])
    ppy = float(intr["ppy"])
    return np.asarray([(float(u) - ppx) * z / fx, (float(v) - ppy) * z / fy, z], dtype=np.float64)


def project_camera(point_camera: np.ndarray, intr: dict[str, object]) -> list[float] | None:
    z = float(point_camera[2])
    if z <= 1e-6:
        return None
    fx = float(intr["fx"])
    fy = float(intr["fy"])
    ppx = float(intr["ppx"])
    ppy = float(intr["ppy"])
    return [float(fx * point_camera[0] / z + ppx), float(fy * point_camera[1] / z + ppy)]


def transform_direction(matrix: np.ndarray, direction: np.ndarray) -> np.ndarray:
    return normalize(np.asarray(matrix[:3, :3], dtype=np.float64) @ normalize(direction))


def transform_point(matrix: np.ndarray, point_camera: np.ndarray) -> np.ndarray:
    p = np.asarray([float(point_camera[0]), float(point_camera[1]), float(point_camera[2]), 1.0], dtype=np.float64)
    q = np.asarray(matrix, dtype=np.float64) @ p
    return q[:3]


def select_line(detection: dict[str, object], selector: str, side: str, line_type: str) -> tuple[str, str]:
    selector = str(selector).strip().lower()
    if selector == "semantic":
        return side, line_type
    if selector not in ("top_outer", "bottom_outer"):
        raise ValueError(f"unsupported line selector: {selector}")
    left = np.asarray(detection["left_outer_pixel"], dtype=np.float64)
    right = np.asarray(detection["right_outer_pixel"], dtype=np.float64)
    left_y = float(left[:, 1].mean())
    right_y = float(right[:, 1].mean())
    if (selector == "top_outer" and left_y <= right_y) or (selector == "bottom_outer" and left_y > right_y):
        return "left", "outer"
    return "right", "outer"


def fit_local_normal_camera(
    *,
    depth_m: np.ndarray,
    intrinsics: dict[str, object],
    u: float,
    v: float,
    window_px: int,
    stride_px: int,
    depth_band_m: float,
    min_points: int,
) -> tuple[np.ndarray, np.ndarray, int, float]:
    h, w = depth_m.shape[:2]
    cu = int(round(float(u)))
    cv = int(round(float(v)))
    radius = max(1, int(window_px) // 2)
    center_z_values = []
    for yy in range(max(0, cv - 2), min(h, cv + 3)):
        for xx in range(max(0, cu - 2), min(w, cu + 3)):
            z = float(depth_m[yy, xx])
            if np.isfinite(z) and z > 0.1:
                center_z_values.append(z)
    if not center_z_values:
        raise RuntimeError(f"invalid center depth at pixel {(u, v)}")
    center_z = float(np.median(center_z_values))

    points: list[np.ndarray] = []
    for yy in range(max(0, cv - radius), min(h, cv + radius + 1), max(1, int(stride_px))):
        for xx in range(max(0, cu - radius), min(w, cu + radius + 1), max(1, int(stride_px))):
            z = float(depth_m[yy, xx])
            if not np.isfinite(z) or z <= 0.1:
                continue
            if abs(z - center_z) > float(depth_band_m):
                continue
            points.append(pixel_to_camera(xx, yy, z, intrinsics))
    if len(points) < int(min_points):
        raise RuntimeError(f"not enough local depth points at {(u, v)}: {len(points)}")
    pts = np.asarray(points, dtype=np.float64)
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    cov = centered.T @ centered / max(1, len(pts) - 1)
    vals, vecs = np.linalg.eigh(cov)
    normal = normalize(vecs[:, int(np.argmin(vals))])
    # Front/outward side is the side facing the camera.
    if float(np.dot(normal, -centroid)) < 0.0:
        normal = -normal
    residual = np.abs(centered @ normal)
    return normal, centroid, len(points), float(np.sqrt(np.mean(residual * residual)))


def load_reference_rpy(path: str) -> list[float] | None:
    if not path:
        return None
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    pose = data.get("tcp_pose_m_rpy")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"reference pose JSON missing tcp_pose_m_rpy: {path}")
    return [float(v) for v in pose[3:6]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline depth-normal diagnostic for side-lying bladder line hover.")
    parser.add_argument("--stem", required=True, help="file stem without _detect.json/_depth.npy suffix")
    parser.add_argument("--matrix-path", default="camera_to_robot.json")
    parser.add_argument("--line-selector", choices=("semantic", "top_outer", "bottom_outer"), default="top_outer")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    parser.add_argument("--hover-mm", type=float, default=80.0)
    parser.add_argument("--window-px", type=int, default=31)
    parser.add_argument("--stride-px", type=int, default=2)
    parser.add_argument("--depth-band-m", type=float, default=0.08)
    parser.add_argument("--min-points", type=int, default=40)
    parser.add_argument("--tool-contact-axis", choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"), default="neg_z")
    parser.add_argument("--reference-contact-pose-json", default="rm_demo_output/user_confirmed_side_lying_contact_pose.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stem = Path(args.stem)
    detect_path = stem.with_name(stem.name + "_detect.json")
    depth_path = stem.with_name(stem.name + "_depth.npy")
    raw_path = stem.with_name(stem.name + "_raw.png")
    intrinsics_path = stem.with_name(stem.name + "_intrinsics.json")
    if not detect_path.exists() or not depth_path.exists() or not raw_path.exists():
        raise RuntimeError(f"missing input files for stem: {stem}")
    detection = json.loads(detect_path.read_text(encoding="utf-8"))
    depth_m = np.load(str(depth_path)).astype(np.float32)
    raw = cv2.imread(str(raw_path))
    if raw is None:
        raise RuntimeError(f"failed to read raw image: {raw_path}")
    if intrinsics_path.exists():
        intrinsics = json.loads(intrinsics_path.read_text(encoding="utf-8"))
    else:
        intrinsics = dict(detection["intrinsics"])
    matrix = load_transform_matrix(args.matrix_path)
    if matrix is None:
        raise RuntimeError(f"invalid camera->robot matrix: {args.matrix_path}")

    selected_side, selected_type = select_line(detection, args.line_selector, args.side, args.line_type)
    prefix = f"{selected_side}_{selected_type}"
    pixels = np.asarray(detection[f"{prefix}_pixel"], dtype=np.float64)
    selected_camera = np.asarray(detection[f"{prefix}_camera"], dtype=np.float64)
    selected_base = np.asarray(transform_points(selected_camera.tolist(), matrix), dtype=np.float64)
    reference_rpy = load_reference_rpy(args.reference_contact_pose_json)
    reference_rot = None if reference_rpy is None else rpy_to_matrix(*reference_rpy)
    reference_axis = None if reference_rot is None else normalize(reference_rot @ axis_vector(args.tool_contact_axis))

    hover_m = float(args.hover_mm) / 1000.0
    camera_from_base = np.linalg.inv(np.asarray(matrix, dtype=np.float64))
    samples: list[dict[str, object]] = []
    hover_base_points = []
    hover_pixels = []
    normal_base_points = []
    orientation_rpy = []
    normal_fit_errors = []
    axis_dots = []

    prev_normal: np.ndarray | None = None
    for idx, (pixel, surface_camera, surface_base) in enumerate(zip(pixels, selected_camera, selected_base), start=1):
        normal_camera, centroid_camera, used_points, fit_rmse = fit_local_normal_camera(
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
            normal_camera = -normal_camera
        prev_normal = normal_base
        hover_base = np.asarray(surface_base, dtype=np.float64) + normal_base * hover_m
        hover_camera = camera_from_base @ np.asarray([hover_base[0], hover_base[1], hover_base[2], 1.0], dtype=np.float64)
        hover_uv = project_camera(hover_camera[:3], intrinsics)
        if hover_uv is None:
            hover_uv = [float("nan"), float("nan")]

        target_contact_axis = -normal_base
        rpy = None
        dot = None
        if reference_rot is not None and reference_axis is not None:
            dot = float(np.dot(reference_axis, target_contact_axis))
            corrected_rot = align_axis_rotation(reference_axis, target_contact_axis) @ reference_rot
            rpy = matrix_to_rpy(corrected_rot)
            axis_dots.append(dot)
            orientation_rpy.append(rpy)

        hover_base_points.append(hover_base)
        hover_pixels.append(hover_uv)
        normal_base_points.append(normal_base)
        normal_fit_errors.append(fit_rmse)
        samples.append(
            {
                "index": idx,
                "pixel_uv": [float(pixel[0]), float(pixel[1])],
                "surface_camera_m": [float(v) for v in surface_camera.tolist()],
                "surface_base_m": [float(v) for v in surface_base.tolist()],
                "normal_camera_front": [float(v) for v in normal_camera.tolist()],
                "normal_base_front": [float(v) for v in normal_base.tolist()],
                "hover_base_m": [float(v) for v in hover_base.tolist()],
                "hover_pixel_uv": [float(v) for v in hover_uv],
                "local_depth_point_count": int(used_points),
                "normal_fit_rmse_m": float(fit_rmse),
                "reference_axis_dot_contact_axis": dot,
                "recommended_rpy": None if rpy is None else [float(v) for v in rpy],
            }
        )

    hover_base_arr = np.asarray(hover_base_points, dtype=np.float64)
    normals_base_arr = np.asarray(normal_base_points, dtype=np.float64)
    hover_pixels_arr = np.asarray(hover_pixels, dtype=np.float64)
    mean_normal = normalize(normals_base_arr.mean(axis=0))

    out_json = stem.with_name(stem.name + "_depth_normal_diagnostic.json")
    data = {
        "source_detect_json": str(detect_path.resolve()),
        "source_depth_npy": str(depth_path.resolve()),
        "source_matrix": str(Path(args.matrix_path).resolve()),
        "line_selector": args.line_selector,
        "selected_side": selected_side,
        "selected_line_type": selected_type,
        "hover_m": hover_m,
        "normal_rule": "PCA local depth normal, signed from surface toward camera/front side",
        "surface_base_mean_m": [float(v) for v in selected_base.mean(axis=0).tolist()],
        "hover_base_mean_m": [float(v) for v in hover_base_arr.mean(axis=0).tolist()],
        "front_normal_base_mean": [float(v) for v in mean_normal.tolist()],
        "normal_fit_rmse_mean_m": float(np.mean(normal_fit_errors)),
        "normal_fit_rmse_max_m": float(np.max(normal_fit_errors)),
        "reference_contact_axis": args.tool_contact_axis,
        "reference_axis_dot_contact_axis_mean": None if not axis_dots else float(np.mean(axis_dots)),
        "reference_axis_dot_contact_axis_min": None if not axis_dots else float(np.min(axis_dots)),
        "samples": samples,
    }
    out_json.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    # 2D overlay: magenta surface line, cyan normal-derived TCP hover line.
    img = raw.copy()
    surf_pts = np.round(pixels).astype(np.int32)
    hover_pts = np.round(hover_pixels_arr[:, :2]).astype(np.int32)
    cv2.polylines(img, [surf_pts.reshape(-1, 1, 2)], False, (255, 0, 255), 3)
    cv2.polylines(img, [hover_pts.reshape(-1, 1, 2)], False, (255, 255, 0), 3)
    for idx in range(0, len(surf_pts), max(1, len(surf_pts) // 8)):
        p0 = tuple(int(v) for v in surf_pts[idx])
        p1 = tuple(int(v) for v in hover_pts[idx])
        cv2.arrowedLine(img, p0, p1, (0, 255, 255), 2, tipLength=0.25)
    cv2.putText(img, "magenta=surface line cyan=depth-normal hover", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, f"{selected_side}_{selected_type} hover={args.hover_mm:.0f}mm no robot motion", (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2, cv2.LINE_AA)
    out_overlay = stem.with_name(stem.name + "_depth_normal_hover_overlay.png")
    cv2.imwrite(str(out_overlay), img)

    # Base-frame projection.
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    projections = [("Base X", "Base Y", 0, 1), ("Base X", "Base Z", 0, 2), ("Base Y", "Base Z", 1, 2)]
    for ax, (xlabel, ylabel, ix, iy) in zip(axes, projections):
        ax.plot(selected_base[:, ix], selected_base[:, iy], "-o", ms=2.5, lw=2, color="#d100d1", label="surface")
        ax.plot(hover_base_arr[:, ix], hover_base_arr[:, iy], "-o", ms=2.5, lw=2, color="#00cccc", label="normal hover")
        for idx in range(0, len(selected_base), max(1, len(selected_base) // 6)):
            ax.arrow(
                selected_base[idx, ix],
                selected_base[idx, iy],
                hover_base_arr[idx, ix] - selected_base[idx, ix],
                hover_base_arr[idx, iy] - selected_base[idx, iy],
                head_width=0.004,
                length_includes_head=True,
                color="#888800",
                alpha=0.8,
            )
        ax.set_xlabel(xlabel + " (m)")
        ax.set_ylabel(ylabel + " (m)")
        ax.grid(True, alpha=0.35)
        ax.axis("equal")
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("Depth-normal hover in RM Base frame (no robot motion)")
    out_plot = stem.with_name(stem.name + "_depth_normal_base_projection.png")
    fig.savefig(str(out_plot), dpi=160)

    print(f"diagnostic_json={out_json.resolve()}")
    print(f"hover_overlay={out_overlay.resolve()}")
    print(f"base_projection={out_plot.resolve()}")
    print(f"selected={selected_side}_{selected_type} hover_m={hover_m:.3f}")
    print(f"surface_mean={[round(float(v), 6) for v in selected_base.mean(axis=0).tolist()]}")
    print(f"hover_mean={[round(float(v), 6) for v in hover_base_arr.mean(axis=0).tolist()]}")
    print(f"front_normal_mean={[round(float(v), 6) for v in mean_normal.tolist()]}")
    print(f"normal_fit_rmse_mean_mm={float(np.mean(normal_fit_errors))*1000:.2f} max_mm={float(np.max(normal_fit_errors))*1000:.2f}")
    if axis_dots:
        print(f"reference_axis_dot_contact_axis_mean={float(np.mean(axis_dots)):.4f} min={float(np.min(axis_dots)):.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
