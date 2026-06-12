#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from rm_demo import rm_json
from rm_demo.rm_bladder import (
    bladder_plan_to_dict,
    build_aligned_contact_preview,
    build_bladder_massage_plan,
    detect_bladder_lines,
    execute_bladder_hover_path,
    preview_bladder_plan,
    rebuild_plan_with_fixed_first_normal,
    save_bladder_artifacts,
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


def _stream_profile(args: argparse.Namespace):
    pipeline = rs.pipeline()
    color_formats = [rs.format.bgr8, rs.format.rgb8]
    last_error: Exception | None = None
    for color_format in color_formats:
        config = rs.config()
        if args.realsense_serial:
            config.enable_device(str(args.realsense_serial))
        config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
        config.enable_stream(rs.stream.color, args.width, args.height, color_format, args.fps)
        try:
            profile = pipeline.start(config)
            return pipeline, profile, color_format
        except Exception as exc:
            last_error = exc
            try:
                pipeline.stop()
            except Exception:
                pass
            pipeline = rs.pipeline()
    raise RuntimeError(f"failed to start RealSense stream: {last_error}")


def _intrinsics_from_profile(profile, depth_scale: float) -> dict[str, object]:
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "coeffs": [float(v) for v in intr.coeffs],
        "model_name": str(intr.model).split(".")[-1],
        "depth_scale": float(depth_scale),
    }


def _read_aligned_frame(pipeline, align, color_format, depth_scale: float):
    frames = align.process(pipeline.wait_for_frames())
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        return None
    color = np.asanyarray(color_frame.get_data())
    if color_format == rs.format.rgb8:
        color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    depth_raw = np.asanyarray(depth_frame.get_data())
    depth_m = depth_raw.astype(np.float32) * float(depth_scale)
    return color, depth_m


def _draw_initial_trajectory(
    frame_bgr: np.ndarray,
    pixels: list[list[float]],
    status_lines: list[str],
    *,
    hover_pixels: list[list[float]] | None = None,
) -> np.ndarray:
    out = frame_bgr.copy()
    pts = np.asarray([[int(round(p[0])), int(round(p[1]))] for p in pixels], dtype=np.int32)
    if len(pts) >= 2:
        cv2.polylines(out, [pts.reshape(-1, 1, 2)], isClosed=False, color=(255, 0, 255), thickness=3)
    for idx, point in enumerate(pts):
        color = (0, 255, 255) if idx == 0 else (255, 0, 255)
        cv2.circle(out, tuple(int(v) for v in point), 4, color, -1)
    if hover_pixels:
        hover_pts = np.asarray([[int(round(p[0])), int(round(p[1]))] for p in hover_pixels], dtype=np.int32)
        if len(hover_pts) >= 2:
            cv2.polylines(out, [hover_pts.reshape(-1, 1, 2)], isClosed=False, color=(0, 255, 255), thickness=3)
            for point in hover_pts:
                cv2.circle(out, tuple(int(v) for v in point), 3, (0, 255, 255), -1)
            first = hover_pts[0]
            cv2.putText(
                out,
                "planned TCP hover",
                (int(first[0]) + 8, int(first[1]) + 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
    y = 28
    for line in status_lines:
        cv2.putText(out, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2, cv2.LINE_AA)
        y += 24
    cv2.putText(
        out,
        "fixed first-frame left_outer trajectory | q/ESC: stop motion and quit",
        (14, out.shape[0] - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return out


def _project_base_points_to_pixels(
    points_base_m: list[list[float]],
    base_from_camera: np.ndarray,
    intrinsics: dict[str, object],
) -> list[list[float]]:
    camera_from_base = np.linalg.inv(np.asarray(base_from_camera, dtype=np.float64))
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])
    out: list[list[float]] = []
    for point_base in points_base_m:
        p = np.asarray([float(point_base[0]), float(point_base[1]), float(point_base[2]), 1.0], dtype=np.float64)
        c = camera_from_base @ p
        if float(c[2]) <= 1e-6:
            continue
        out.append([float(fx * c[0] / c[2] + ppx), float(fy * c[1] / c[2] + ppy)])
    return out


def _load_reference_contact_pose(path: str) -> list[float] | None:
    if not path:
        return None
    src = Path(path)
    if not src.exists():
        raise RuntimeError(f"reference contact pose JSON not found: {src}")
    data = json.loads(src.read_text(encoding="utf-8"))
    pose = data.get("tcp_pose_m_rpy")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"reference contact pose JSON has no tcp_pose_m_rpy: {src}")
    return [float(v) for v in pose[:6]]


def _build_plan(
    *,
    detection: dict[str, object],
    matrix: np.ndarray,
    anchor_pose: list[float],
    side: str,
    line_type: str,
    plan_points: int,
    hover_m: float,
    dian_jin_depth_m: float,
    fen_jin_lateral_m: float,
    safe_lift_m: float,
    tool_contact_axis: str,
    start_nearest_anchor: bool,
) -> tuple[object, dict[str, object], np.ndarray]:
    selected = select_bladder_line(detection, side, line_type)
    camera_points = list(selected.get("selected_meridian_camera", []))
    pixels = list(selected.get("selected_meridian_pixel", []))
    if len(camera_points) < 2:
        raise RuntimeError(f"{side}_{line_type} has insufficient camera points")
    robot_points = transform_points(camera_points, matrix)
    selected["selected_meridian_robot"] = robot_points
    selected["robot_frame_unit"] = "meters"
    selected["transform_backend"] = "external_static_point_calibration"

    rot = _rpy_to_matrix(*anchor_pose[3:6])
    press = _normalize(rot @ _axis_vector(tool_contact_axis))
    base_plan = build_bladder_massage_plan(
        side=side,
        line_type=line_type,
        meridian_points_robot_m=robot_points,
        meridian_pixels=pixels,
        anchor_pose_m=anchor_pose,
        point_count=plan_points,
        hover_m=hover_m,
        dian_jin_depth_m=dian_jin_depth_m,
        fen_jin_lateral_m=fen_jin_lateral_m,
        safe_lift_m=safe_lift_m,
        start_nearest_anchor=start_nearest_anchor,
    )
    plan = rebuild_plan_with_fixed_first_normal(
        base_plan,
        source_index=0,
        press_direction_override=[float(v) for v in press.tolist()],
        tool_contact_axis=tool_contact_axis,
    )
    return plan, selected, press


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect first-frame left_outer and run side-lying hover path with live overlay.")
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--matrix-path", default="camera_to_robot.json")
    parser.add_argument("--output-dir", default="rm_demo_output")
    parser.add_argument("--model-path", default="yolo11l-pose.pt")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    parser.add_argument(
        "--line-selector",
        choices=("semantic", "top_outer", "bottom_outer"),
        default="semantic",
        help="semantic uses --side/--line-type; top_outer selects the outer line with smaller image y",
    )
    parser.add_argument("--finger-width", type=float, default=45.0)
    parser.add_argument("--sample-points", type=int, default=30)
    parser.add_argument("--plan-points", type=int, default=10)
    parser.add_argument("--hover-mm", type=float, default=80.0)
    parser.add_argument("--dian-jin-depth-mm", type=float, default=8.0)
    parser.add_argument("--fen-jin-lateral-mm", type=float, default=15.0)
    parser.add_argument("--safe-lift-mm", type=float, default=60.0)
    parser.add_argument(
        "--tool-contact-axis",
        choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"),
        default="neg_z",
        help="side-lying contact direction. neg_z makes hover move toward current tool +Z/front side.",
    )
    parser.add_argument(
        "--reference-contact-pose-json",
        default="rm_demo_output/user_confirmed_side_lying_contact_pose.json",
        help="fixed user-confirmed contact pose; only its RPY is used for trajectory orientation",
    )
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--max-step-m", type=float, default=0.02)
    parser.add_argument("--dwell-s", type=float, default=0.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--realsense-serial", default="")
    parser.add_argument("--window-name", default="External bladder hover")
    parser.add_argument("--start-nearest-anchor", action="store_true", default=True)
    parser.add_argument("--no-start-nearest-anchor", action="store_false", dest="start_nearest_anchor")
    parser.add_argument(
        "--save-preview-only",
        action="store_true",
        help="save the first-frame surface line and planned TCP hover projection, then exit without moving",
    )
    parser.add_argument("--run", action="store_true", help="execute arm hover motion after first-frame detection")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = f"external_bladder_live_{stamp}_{args.side}_{args.line_type}"

    matrix = load_transform_matrix(str(Path(args.matrix_path).resolve()))
    if matrix is None:
        raise RuntimeError(f"camera->robot matrix not found or invalid: {args.matrix_path}")

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

        detection, overlay = detect_bladder_lines(
            color_bgr=color_bgr,
            depth_m=depth_m,
            intrinsics_data=intrinsics,
            finger_width_mm=args.finger_width,
            model_path=args.model_path,
            sample_points=args.sample_points,
        )
        detection["capture"] = {
            "backend": "local_realsense_first_frame",
            "timestamp": stamp,
            "color_format": str(color_format),
        }
        if args.line_selector == "semantic":
            selected_side = args.side
            selected_line_type = args.line_type
        else:
            left_y = float(np.asarray(detection["left_outer_pixel"], dtype=np.float64)[:, 1].mean())
            right_y = float(np.asarray(detection["right_outer_pixel"], dtype=np.float64)[:, 1].mean())
            if (args.line_selector == "top_outer" and left_y <= right_y) or (
                args.line_selector == "bottom_outer" and left_y > right_y
            ):
                selected_side = "left"
            else:
                selected_side = "right"
            selected_line_type = "outer"
        detection = select_bladder_line(detection, selected_side, selected_line_type)
        overlay_path, detect_json_path = save_bladder_artifacts(str(out_dir), detection, overlay, prefix=prefix)
        raw_path = out_dir / f"{prefix}_raw.png"
        depth_path = out_dir / f"{prefix}_depth.npy"
        intrinsics_path = out_dir / f"{prefix}_intrinsics.json"
        cv2.imwrite(str(raw_path), color_bgr)
        np.save(str(depth_path), depth_m)
        intrinsics_path.write_text(json.dumps(intrinsics, ensure_ascii=False, indent=2), encoding="utf-8")

        joints, anchor_pose, arm_err, sys_err, ik_err = rm_json.get_current_arm_state(args.arm_host)
        if arm_err != 0 or sys_err != 0 or ik_err not in (0, -1):
            raise RuntimeError(f"arm state not clean: arm_err={arm_err} sys_err={sys_err} inverse_km_err={ik_err}")
        reference_contact_pose = _load_reference_contact_pose(args.reference_contact_pose_json)
        if reference_contact_pose is not None:
            anchor_pose = [float(anchor_pose[0]), float(anchor_pose[1]), float(anchor_pose[2]), *reference_contact_pose[3:6]]

        plan, transformed, press = _build_plan(
            detection=detection,
            matrix=matrix,
            anchor_pose=anchor_pose,
            side=selected_side,
            line_type=selected_line_type,
            plan_points=args.plan_points,
            hover_m=args.hover_mm / 1000.0,
            dian_jin_depth_m=args.dian_jin_depth_mm / 1000.0,
            fen_jin_lateral_m=args.fen_jin_lateral_mm / 1000.0,
            safe_lift_m=args.safe_lift_mm / 1000.0,
            tool_contact_axis=args.tool_contact_axis,
            start_nearest_anchor=bool(args.start_nearest_anchor),
        )

        transform_path = out_dir / f"{prefix}_transform.json"
        plan_path = out_dir / f"{prefix}_plan.json"
        trajectory_path = out_dir / f"{prefix}_first_frame_left_outer_trajectory.json"
        transformed["source_detection_json"] = detect_json_path
        transformed["source_matrix_path"] = str(Path(args.matrix_path).resolve())
        transform_path.write_text(json.dumps(transformed, ensure_ascii=False, indent=2), encoding="utf-8")

        plan_data = bladder_plan_to_dict(plan)
        plan_data["source_detection_json"] = detect_json_path
        plan_data["source_transform_json"] = str(transform_path)
        plan_data["anchor_joint_deg"] = [float(v) for v in joints]
        plan_data["normal_source"] = {
            "type": "current_tcp_tool_axis",
            "tool_contact_axis": args.tool_contact_axis,
            "reference_contact_pose_json": str(Path(args.reference_contact_pose_json).resolve())
            if args.reference_contact_pose_json
            else "",
            "reference_contact_rpy": None if reference_contact_pose is None else reference_contact_pose[3:6],
            "press_direction_m": [float(v) for v in press.tolist()],
            "side_lying_hover_rule": "hover = surface - press * hover; with neg_z press this is current tool +Z/front side",
        }
        plan_data["aligned_contact_preview"] = build_aligned_contact_preview(
            plan,
            tool_contact_axis=args.tool_contact_axis,
            contact_motion_axis=args.tool_contact_axis,
            max_press_m=0.0,
            touch_step_m=0.0,
        )
        plan_path.write_text(json.dumps(plan_data, ensure_ascii=False, indent=2), encoding="utf-8")

        trajectory_data = {
            "timestamp": stamp,
            "side": selected_side,
            "line_type": selected_line_type,
            "line_selector": args.line_selector,
            "fixed_first_frame": True,
            "pixel": transformed.get("selected_meridian_pixel", []),
            "camera_points_m": transformed.get("selected_meridian_camera", []),
            "robot_points_m": transformed.get("selected_meridian_robot", []),
            "plan_json": str(plan_path),
            "detect_json": detect_json_path,
            "overlay_png": overlay_path,
            "raw_png": str(raw_path),
            "depth_npy": str(depth_path),
        }
        trajectory_path.write_text(json.dumps(trajectory_data, ensure_ascii=False, indent=2), encoding="utf-8")

        print(f"detection_overlay={overlay_path}")
        print(f"detection_json={detect_json_path}")
        print(f"trajectory_json={trajectory_path}")
        print(f"transform_json={transform_path}")
        print(f"plan_json={plan_path}")
        print(f"line_selector={args.line_selector} selected={selected_side}_{selected_line_type}")
        print(f"anchor_pose={[round(float(v), 6) for v in anchor_pose]}")
        if reference_contact_pose is not None:
            print(f"reference_contact_rpy={[round(float(v), 6) for v in reference_contact_pose[3:6]]}")
        print(f"press_direction={[round(float(v), 6) for v in press.tolist()]} tool_contact_axis={args.tool_contact_axis}")
        preview_bladder_plan(plan)

        pixels = list(transformed.get("selected_meridian_pixel", []))
        hover_pixels = _project_base_points_to_pixels(
            [frame.hover_pose_m[:3] for frame in plan.frames],
            matrix,
            intrinsics,
        )
        first_hover = plan.frames[0].hover_pose_m[:3]
        status_base = [
            f"{selected_side}_{selected_line_type} fixed first frame points={len(pixels)}",
            f"hover={args.hover_mm:.0f}mm axis={args.tool_contact_axis} speed={args.speed}",
            "magenta=surface line yellow=TCP hover path",
            f"first_hover=({first_hover[0]:.3f},{first_hover[1]:.3f},{first_hover[2]:.3f})",
        ]

        preview_img = _draw_initial_trajectory(
            color_bgr,
            pixels,
            [*status_base, "motion=preview"],
            hover_pixels=hover_pixels,
        )
        preview_path = out_dir / f"{prefix}_surface_and_tcp_hover_preview.png"
        cv2.imwrite(str(preview_path), preview_img)
        print(f"surface_and_tcp_hover_preview={preview_path}")
        if args.save_preview_only:
            print("save_preview_only=True; no robot motion")
            return 0

        def _run_motion() -> None:
            try:
                execute_bladder_hover_path(
                    host=args.arm_host,
                    plan=plan,
                    speed=args.speed,
                    control_backend="json",
                    dwell_s=args.dwell_s,
                    use_global_safe_z=False,
                    keep_current_orientation=False,
                    max_step_m=args.max_step_m,
                    entry_motion="movel",
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
        while True:
            frame_pair = _read_aligned_frame(pipeline, align, color_format, depth_scale)
            if frame_pair is None:
                continue
            live_bgr, _ = frame_pair
            state = "running" if args.run and not motion_done.is_set() else "preview" if not args.run else "done"
            if motion_error:
                state = f"error: {type(motion_error[0]).__name__}"
            display = _draw_initial_trajectory(
                live_bgr,
                pixels,
                [*status_base, f"motion={state}"],
                hover_pixels=hover_pixels,
            )
            cv2.imshow(args.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                stop_requested = True
                if args.run and not motion_done.is_set():
                    rm_json.stop_motion(args.arm_host)
                break
            if args.run and motion_done.is_set():
                # Keep the completed trajectory visible for a short moment.
                time.sleep(1.0)
                break
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
