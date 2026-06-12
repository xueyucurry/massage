#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from rm_demo import rm_json
from rm_demo.rm_bladder import (
    bladder_plan_to_dict,
    build_aligned_contact_preview,
    build_bladder_massage_plan,
    preview_bladder_plan,
    rebuild_plan_with_fixed_first_normal,
    select_bladder_line,
)
from rm_demo.rm_transform import load_transform_matrix, transform_points


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
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


def _axis_vector(name: str) -> np.ndarray:
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
        raise ValueError(f"unsupported tool axis: {name}")
    return np.asarray(axes[key], dtype=np.float64)


def _normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        raise RuntimeError("zero-length vector")
    return vec / norm


def _choose_press_direction(
    *,
    anchor_pose_m: list[float],
    robot_points_m: list[list[float]],
    camera_to_robot: np.ndarray,
    tool_contact_axis: str,
    press_sign: str,
) -> tuple[np.ndarray, str, np.ndarray]:
    rot = _rpy_to_matrix(*[float(v) for v in anchor_pose_m[3:6]])
    tool_axis_world = _normalize(rot @ _axis_vector(tool_contact_axis))
    sign = str(press_sign).strip().lower()
    if sign == "positive":
        return tool_axis_world, "positive", tool_axis_world
    if sign == "negative":
        return -tool_axis_world, "negative", tool_axis_world
    if sign != "auto":
        raise ValueError(f"unsupported press sign: {press_sign}")

    points = np.asarray(robot_points_m, dtype=np.float64)
    mean_point = points.mean(axis=0)
    camera_origin = np.asarray(camera_to_robot[:3, 3], dtype=np.float64)
    toward_camera = _normalize(camera_origin - mean_point)
    # press moves from hover to body. The outward hover direction is -press.
    positive_score = float(np.dot(-tool_axis_world, toward_camera))
    negative_score = float(np.dot(tool_axis_world, toward_camera))
    if positive_score >= negative_score:
        return tool_axis_world, "positive_auto_camera_side", tool_axis_world
    return -tool_axis_world, "negative_auto_camera_side", tool_axis_world


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a side-lying bladder hover plan from an external camera detection JSON."
    )
    parser.add_argument("--detect-json", required=True)
    parser.add_argument("--matrix-path", default="camera_to_robot.json")
    parser.add_argument("--output", default="")
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    parser.add_argument("--plan-points", type=int, default=6)
    parser.add_argument("--hover-mm", type=float, default=50.0)
    parser.add_argument("--dian-jin-depth-mm", type=float, default=8.0)
    parser.add_argument("--fen-jin-lateral-mm", type=float, default=15.0)
    parser.add_argument("--safe-lift-mm", type=float, default=60.0)
    parser.add_argument(
        "--tool-contact-axis",
        choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"),
        default="pos_z",
    )
    parser.add_argument(
        "--press-sign",
        choices=("positive", "negative", "auto"),
        default="positive",
        help="positive means contact motion follows the chosen tool axis; auto picks the camera-side hover sign",
    )
    parser.add_argument(
        "--start-nearest-anchor",
        action="store_true",
        help="reverse selected points if the last point is nearer to current TCP",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    detect_path = Path(args.detect_json).resolve()
    matrix_path = Path(args.matrix_path).resolve()
    matrix = load_transform_matrix(str(matrix_path))
    if matrix is None:
        raise RuntimeError(f"camera->robot matrix not found or invalid: {matrix_path}")

    detection = json.loads(detect_path.read_text(encoding="utf-8"))
    selected = select_bladder_line(detection, args.side, args.line_type)
    camera_points = list(selected.get("selected_meridian_camera", []))
    pixels = list(selected.get("selected_meridian_pixel", []))
    if len(camera_points) < 2:
        raise RuntimeError("selected_meridian_camera has insufficient points")

    robot_points = transform_points(camera_points, matrix)
    selected["selected_meridian_robot"] = robot_points
    selected["robot_frame_unit"] = "meters"
    selected["transform_backend"] = "external_static_point_calibration"
    selected["matrix_path"] = str(matrix_path)

    joints, anchor_pose, arm_err, sys_err, ik_err = rm_json.get_current_arm_state(args.arm_host)
    if arm_err != 0 or sys_err != 0 or ik_err not in (0, -1):
        raise RuntimeError(f"arm state not clean: arm_err={arm_err} sys_err={sys_err} inverse_km_err={ik_err}")

    press, sign_used, tool_axis_world = _choose_press_direction(
        anchor_pose_m=anchor_pose,
        robot_points_m=robot_points,
        camera_to_robot=matrix,
        tool_contact_axis=args.tool_contact_axis,
        press_sign=args.press_sign,
    )

    base_plan = build_bladder_massage_plan(
        side=args.side,
        line_type=args.line_type,
        meridian_points_robot_m=robot_points,
        meridian_pixels=pixels,
        anchor_pose_m=anchor_pose,
        point_count=args.plan_points,
        hover_m=args.hover_mm / 1000.0,
        dian_jin_depth_m=args.dian_jin_depth_mm / 1000.0,
        fen_jin_lateral_m=args.fen_jin_lateral_mm / 1000.0,
        safe_lift_m=args.safe_lift_mm / 1000.0,
        start_nearest_anchor=bool(args.start_nearest_anchor),
    )
    plan = rebuild_plan_with_fixed_first_normal(
        base_plan,
        source_index=0,
        press_direction_override=[float(v) for v in press.tolist()],
        tool_contact_axis=args.tool_contact_axis,
    )

    data = bladder_plan_to_dict(plan)
    data["source_detection_json"] = str(detect_path)
    data["source_matrix_path"] = str(matrix_path)
    data["anchor_joint_deg"] = [float(v) for v in joints]
    data["normal_source"] = {
        "type": "current_tcp_tool_axis",
        "tool_contact_axis": args.tool_contact_axis,
        "press_sign_requested": args.press_sign,
        "press_sign_used": sign_used,
        "tool_axis_world_m": [float(v) for v in tool_axis_world.tolist()],
        "press_direction_m": [float(v) for v in press.tolist()],
    }
    data["aligned_contact_preview"] = build_aligned_contact_preview(
        plan,
        tool_contact_axis=args.tool_contact_axis,
        contact_motion_axis=args.tool_contact_axis,
        max_press_m=0.0,
        touch_step_m=0.0,
    )

    output = Path(args.output).resolve() if args.output else detect_path.with_name(
        detect_path.stem.replace("_detect", "") + "_external_side_lying_plan.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    points = np.asarray(robot_points, dtype=np.float64)
    hovers = np.asarray([frame.hover_pose_m[:3] for frame in plan.frames], dtype=np.float64)
    print(f"plan_json={output}")
    print(f"anchor_pose={[round(float(v), 6) for v in anchor_pose]}")
    print(f"tool_axis_world={[round(float(v), 6) for v in tool_axis_world.tolist()]}")
    print(f"press_direction={[round(float(v), 6) for v in press.tolist()]} sign={sign_used}")
    print(f"robot_points_bbox_min={[round(float(v), 6) for v in points.min(axis=0).tolist()]}")
    print(f"robot_points_bbox_max={[round(float(v), 6) for v in points.max(axis=0).tolist()]}")
    print(f"hover_bbox_min={[round(float(v), 6) for v in hovers.min(axis=0).tolist()]}")
    print(f"hover_bbox_max={[round(float(v), 6) for v in hovers.max(axis=0).tolist()]}")
    preview_bladder_plan(plan)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
