#!/usr/bin/env python3
"""Camera tool for tuning thigh line offsets used by ft.py."""

import argparse
import json
import os
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import torch

from rtmpose_detector import DEFAULT_RTMPOSE_CONFIG, DEFAULT_RTMPOSE_WEIGHTS, ROTATIONS, RTMPoseHipKneeDetector
from thigh_outerline_confirm import (
    DIRECTION_MODES,
    RealSenseReader,
    build_offset_line,
    detect_pose,
    draw_polyline,
    draw_text_box,
    estimate_outward_direction,
    keypoint_indices,
    save_confirmation,
)


SCRIPT_DIR = Path(__file__).resolve().parent
ENV_PATH = SCRIPT_DIR / "thigh_offset_adjust.env"
OUTPUT_DIR = SCRIPT_DIR / "rtmpose_thigh_adjust_output"
WINDOW_NAME = "Left inner thigh offset | OpenCV"


ARROW_KEY_ACTIONS = {
    # Linux/X11 OpenCV key codes.
    65361: "left",
    65362: "up",
    65363: "right",
    65364: "down",
    # Windows OpenCV key codes.
    2424832: "left",
    2490368: "up",
    2555904: "right",
    2621440: "down",
}


def key_action(key: int) -> Optional[str]:
    """Map OpenCV keys to line-edit actions without truncating arrow keys."""
    if key in ARROW_KEY_ACTIONS:
        return ARROW_KEY_ACTIONS[key]
    if key < 0:
        return None
    return {
        ord("a"): "left",
        ord("w"): "up",
        ord("d"): "right",
        ord("x"): "down",
        ord("s"): "save",
        ord("q"): "quit",
        27: "quit",
    }.get(key & 0xFF)


def move_line_in_image(
    selection,
    depth,
    reader,
    outward_3d,
    outward_2d,
    selected_line: np.ndarray,
    offset_mm: float,
    line_shift_mm: float,
    sample_count: int,
    delta_x_px: float,
    delta_y_px: float,
    offset_probe_mm: float,
    shift_probe_mm: float,
):
    """Convert a requested screen-space nudge into the two physical offsets."""
    if len(selected_line) == 0:
        return offset_mm, line_shift_mm, False

    base_center = np.mean(selected_line, axis=0)
    probe_offset = max(1.0, abs(float(offset_probe_mm)))
    probe_shift = max(1.0, abs(float(shift_probe_mm)))
    offset_line, _, _ = build_offset_line(
        selection,
        depth,
        reader,
        outward_3d,
        outward_2d,
        offset_mm + probe_offset,
        sample_count,
        line_shift_mm=line_shift_mm,
    )
    shift_line, _, _ = build_offset_line(
        selection,
        depth,
        reader,
        outward_3d,
        outward_2d,
        offset_mm,
        sample_count,
        line_shift_mm=line_shift_mm + probe_shift,
    )
    if len(offset_line) != len(selected_line) or len(shift_line) != len(selected_line):
        return offset_mm, line_shift_mm, False

    jacobian = np.column_stack(
        (
            (np.mean(offset_line, axis=0) - base_center) / probe_offset,
            (np.mean(shift_line, axis=0) - base_center) / probe_shift,
        )
    )
    if not np.all(np.isfinite(jacobian)) or abs(float(np.linalg.det(jacobian))) < 1e-4:
        return offset_mm, line_shift_mm, False

    delta_mm = np.linalg.solve(
        jacobian,
        np.asarray([delta_x_px, delta_y_px], dtype=np.float32),
    )
    max_delta_mm = max(probe_offset, probe_shift) * 4.0
    delta_mm = np.clip(delta_mm, -max_delta_mm, max_delta_mm)
    return (
        max(0.0, float(offset_mm) + float(delta_mm[0])),
        float(line_shift_mm) + float(delta_mm[1]),
        True,
    )


def _target_env_names(target: str):
    if target == "inner":
        return "THIGH_INNER_OFFSET_MM", "THIGH_INNER_LINE_SHIFT_MM", "leg_inner"
    return "THIGH_OUTER_OFFSET_MM", "THIGH_OUTER_LINE_SHIFT_MM", "leg"


def _target_side_env_name(target: str) -> str:
    return "THIGH_INNER_SIDE" if target == "inner" else "THIGH_OUTER_SIDE"


def _target_side_default(target: str) -> str:
    valid = {"nearest", "auto", "left", "right"}
    specific = os.environ.get(_target_side_env_name(target), "").strip().lower()
    if specific in valid:
        return specific
    legacy = os.environ.get("THIGH_SIDE", "").strip().lower()
    if legacy in valid:
        return legacy
    return "left" if target == "inner" else "right"


def _target_rotation_env_names(target: str):
    prefix = "THIGH_INNER" if target == "inner" else "THIGH_OUTER"
    return f"{prefix}_ROTATION", f"{prefix}_TRY_ROTATIONS"


def _target_rotation_default(target: str) -> str:
    rotation_env, _ = _target_rotation_env_names(target)
    specific = os.environ.get(rotation_env, "").strip().lower()
    if specific in ROTATIONS:
        return specific
    legacy = os.environ.get("THIGH_ROTATION", "").strip().lower()
    if legacy in ROTATIONS:
        return legacy
    return "cw" if target == "inner" else "none"


def _target_try_rotations_default(target: str) -> bool:
    _, try_env = _target_rotation_env_names(target)
    value = os.environ.get(try_env, os.environ.get("THIGH_TRY_ROTATIONS", "0"))
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except Exception:
        return float(default)


def parse_args():
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--target", choices=["outer", "inner"], default=os.environ.get("THIGH_ADJUST_TARGET", "outer"))
    pre_args, _ = pre_parser.parse_known_args()
    offset_env, line_shift_env, _ = _target_env_names(pre_args.target)
    offset_default = _env_float(offset_env, _env_float("THIGH_OFFSET_MM", 25.0))
    line_shift_default = _env_float(line_shift_env, _env_float("THIGH_LINE_SHIFT_MM", 0.0))

    parser = argparse.ArgumentParser(description="Tune thigh outer-line offsets from the RealSense camera.")
    parser.add_argument("--target", choices=["outer", "inner"], default=pre_args.target, help="Which ft.py thigh target to tune.")
    parser.add_argument("--side", choices=["nearest", "auto", "left", "right"], default=_target_side_default(pre_args.target))
    parser.add_argument("--offset-mm", type=float, default=offset_default, help="Lateral outward offset.")
    parser.add_argument("--line-shift-mm", type=float, default=line_shift_default, help="Shift along hip-to-knee line. Positive moves toward knee.")
    parser.add_argument("--offset-step-mm", type=float, default=5.0)
    parser.add_argument("--shift-step-mm", type=float, default=10.0)
    parser.add_argument("--pixel-step", type=float, default=5.0, help="Screen-space movement per arrow/WADX key press.")
    parser.add_argument("--direction", choices=DIRECTION_MODES, default=os.environ.get("THIGH_DIRECTION", "image-down"))
    parser.add_argument("--samples", type=int, default=int(_env_float("THIGH_SAMPLE_POINTS", 10)))
    parser.add_argument("--kpt-thr", type=float, default=_env_float("THIGH_KPT_THR", 0.25))
    parser.add_argument("--pose2d", default=DEFAULT_RTMPOSE_CONFIG)
    parser.add_argument("--pose2d-weights", default=None)
    parser.add_argument("--device", default=os.environ.get("THIGH_DEVICE", "auto"))
    parser.add_argument("--width", type=int, default=int(_env_float("THIGH_CAMERA_WIDTH", 640)))
    parser.add_argument("--height", type=int, default=int(_env_float("THIGH_CAMERA_HEIGHT", 480)))
    parser.add_argument("--fps", type=int, default=int(_env_float("THIGH_CAMERA_FPS", 30)))
    parser.add_argument("--rotation", choices=ROTATIONS, default=_target_rotation_default(pre_args.target))
    rotation_group = parser.add_mutually_exclusive_group()
    rotation_group.add_argument("--try-rotations", dest="try_rotations", action="store_true")
    rotation_group.add_argument("--single-rotation", dest="try_rotations", action="store_false")
    parser.set_defaults(try_rotations=_target_try_rotations_default(pre_args.target))
    parser.add_argument("--no-align-depth", action="store_true")
    return parser.parse_args()


def write_env(
    path: Path,
    target: str,
    offset_mm: float,
    line_shift_mm: float,
    side: str,
    direction: str,
    flip: bool,
    rotation: str,
    try_rotations: bool,
) -> None:
    offset_env, line_shift_env, massage_target = _target_env_names(target)
    side_env = _target_side_env_name(target)
    rotation_env, try_rotations_env = _target_rotation_env_names(target)
    path.write_text(
        "\n".join(
            [
                f'export {offset_env}="{offset_mm:.1f}"',
                f'export {line_shift_env}="{line_shift_mm:.1f}"',
                f'export {side_env}="{side}"',
                f'export {rotation_env}="{rotation}"',
                f'export {try_rotations_env}="{1 if try_rotations else 0}"',
                f'export THIGH_DIRECTION="{direction}"',
                f'export THIGH_FLIP_DIRECTION="{1 if flip else 0}"',
                f'export MASSAGE_TARGET="{massage_target}"',
                "",
            ]
        ),
        encoding="utf-8",
    )


def save_adjustment(
    target: str,
    offset_mm: float,
    line_shift_mm: float,
    side: str,
    direction: str,
    flip: bool,
    valid_ratio: float,
    confirmation_path: Optional[Path],
    rotation: str,
    try_rotations: bool,
) -> Path:
    offset_env, line_shift_env, massage_target = _target_env_names(target)
    side_env = _target_side_env_name(target)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "target": target,
        "massage_target": massage_target,
        offset_env.lower(): float(offset_mm),
        line_shift_env.lower(): float(line_shift_mm),
        "thigh_offset_mm": float(offset_mm),
        "thigh_line_shift_mm": float(line_shift_mm),
        "thigh_side": side,
        side_env.lower(): side,
        "thigh_pose_rotation": rotation,
        "thigh_try_rotations": bool(try_rotations),
        "thigh_direction": direction,
        "thigh_flip_direction": bool(flip),
        "valid_depth_ratio": float(valid_ratio),
        "confirmation_json": None if confirmation_path is None else str(confirmation_path),
        "env_file": str(ENV_PATH),
    }
    path = OUTPUT_DIR / f"thigh_offset_adjust_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def main() -> int:
    args = parse_args()
    device = "cuda:0" if args.device == "auto" and torch.cuda.is_available() else ("cpu" if args.device == "auto" else args.device)
    rotations = ROTATIONS if args.try_rotations else (args.rotation,)
    offset_env, line_shift_env, massage_target = _target_env_names(args.target)
    offset_mm = float(args.offset_mm)
    line_shift_mm = float(args.line_shift_mm)
    side_mode = args.side
    direction_mode = args.direction
    if args.target == "inner" and direction_mode == "image-down":
        offset_mm = abs(offset_mm)
    flip = os.environ.get("THIGH_FLIP_DIRECTION", "0").strip().lower() in {"1", "true", "yes", "on"}

    print(f"[RTMPose] config={args.pose2d}")
    print(f"[RTMPose] weights={args.pose2d_weights or DEFAULT_RTMPOSE_WEIGHTS}")
    print(f"[RTMPose] device={device}, rotations={','.join(rotations)}")
    print(f"[Target] {args.target}: saves {offset_env}/{line_shift_env}, MASSAGE_TARGET={massage_target}")
    print("[Keys] Left/A Right/D Up/W Down/X: move the purple line on screen | S: save | Q/Esc: quit")
    print("[Keys] L/R/N: left/right/nearest leg | V: image-down direction | O: anatomical outer | F: flip")
    print(f"[Meaning] {offset_env} is lateral. {line_shift_env} positive moves toward knee, negative toward hip.")

    detector = RTMPoseHipKneeDetector(
        pose2d=args.pose2d,
        pose2d_weights=args.pose2d_weights,
        device=device,
        side="nearest",
        kpt_thr=args.kpt_thr,
        rotations=rotations,
    )
    reader = RealSenseReader(args.width, args.height, args.fps, align_depth=not args.no_align_depth)
    last_save_msg = ""
    last_wall = time.time()
    fps_smooth = 0.0

    try:
        reader.start()
        while True:
            color, depth, _ = reader.get_frame()
            selection = detect_pose(detector, color, depth, reader.depth_scale, side_mode, args.kpt_thr, rotations)
            vis = color.copy()
            selected_line = np.empty((0, 2), dtype=np.float32)
            selected_depths: List[Optional[float]] = []
            selected_surface_points = []
            direction_source = "none"
            valid_ratio = 0.0

            now = time.time()
            inst_fps = 1.0 / max(now - last_wall, 1e-6)
            fps_smooth = inst_fps if fps_smooth <= 0 else 0.9 * fps_smooth + 0.1 * inst_fps
            last_wall = now

            if selection.valid and selection.keypoints is not None and selection.scores is not None:
                outward_3d, outward_2d, direction_source = estimate_outward_direction(
                    selection,
                    depth,
                    reader,
                    flip,
                    direction_mode=direction_mode,
                )
                hip_i, knee_i, _, opposite_hip_i = keypoint_indices(selection.side)
                hip = selection.keypoints[hip_i]
                knee = selection.keypoints[knee_i]
                opposite_hip = selection.keypoints[opposite_hip_i]
                cv2.line(vis, tuple(np.round(hip).astype(int)), tuple(np.round(knee).astype(int)), (0, 255, 255), 2, cv2.LINE_AA)
                cv2.circle(vis, tuple(np.round(hip).astype(int)), 6, (0, 200, 255), -1, cv2.LINE_AA)
                cv2.circle(vis, tuple(np.round(knee).astype(int)), 6, (0, 255, 0), -1, cv2.LINE_AA)
                cv2.circle(vis, tuple(np.round(opposite_hip).astype(int)), 5, (255, 200, 0), -1, cv2.LINE_AA)

                selected_line, selected_depths, selected_surface_points = build_offset_line(
                    selection,
                    depth,
                    reader,
                    outward_3d,
                    outward_2d,
                    offset_mm,
                    max(2, int(args.samples)),
                    line_shift_mm=line_shift_mm,
                )
                draw_polyline(vis, selected_line, (255, 0, 255), 4)
                for pt, depth_m in zip(selected_line, selected_depths):
                    if not (0 <= pt[0] < vis.shape[1] and 0 <= pt[1] < vis.shape[0]):
                        continue
                    color_dot = (0, 255, 0) if depth_m is not None else (0, 0, 255)
                    cv2.circle(vis, tuple(np.round(pt).astype(int)), 3, color_dot, -1, cv2.LINE_AA)
                valid_ratio = sum(1 for d in selected_depths if d is not None) / max(len(selected_depths), 1)
                status = [
                    f"target={args.target} side={selection.side} mode={side_mode} direction={direction_mode} flip={flip}",
                    f"{offset_env}={offset_mm:.1f}  {line_shift_env}={line_shift_mm:.1f}",
                    f"depth valid={valid_ratio * 100:.0f}% fps={fps_smooth:.1f} dir={direction_source}",
                    "ARROWS / W A X D: move purple line on screen",
                    "S: SAVE | Q/ESC: quit",
                    last_save_msg,
                ]
            else:
                status = [
                    selection.reason,
                    f"target={args.target} mode={side_mode} {offset_env}={offset_mm:.1f} {line_shift_env}={line_shift_mm:.1f}",
                    "ARROWS / W A X D: move purple line on screen",
                    "S: SAVE | Q/ESC: quit",
                    last_save_msg,
                ]

            draw_text_box(vis, status)
            cv2.imshow(WINDOW_NAME, vis)
            key = cv2.waitKeyEx(1)
            action = key_action(key)
            low_key = key & 0xFF if key >= 0 else -1

            if action == "quit":
                break
            if action in {"left", "right", "up", "down"}:
                requested_move = {
                    "left": (-float(args.pixel_step), 0.0),
                    "right": (float(args.pixel_step), 0.0),
                    "up": (0.0, -float(args.pixel_step)),
                    "down": (0.0, float(args.pixel_step)),
                }[action]
                if selection.valid and len(selected_line) > 0:
                    offset_mm, line_shift_mm, moved = move_line_in_image(
                        selection,
                        depth,
                        reader,
                        outward_3d,
                        outward_2d,
                        selected_line,
                        offset_mm,
                        line_shift_mm,
                        max(2, int(args.samples)),
                        requested_move[0],
                        requested_move[1],
                        args.offset_step_mm,
                        args.shift_step_mm,
                    )
                    if not moved:
                        last_save_msg = "MOVE BLOCKED: detected axes are ambiguous"
                else:
                    last_save_msg = "MOVE BLOCKED: wait for a valid left-leg line"
            elif low_key in (ord("["), ord("-"), ord("_")):
                offset_mm = max(0.0, offset_mm - float(args.offset_step_mm))
            elif low_key in (ord("]"), ord("+"), ord("=")):
                offset_mm += float(args.offset_step_mm)
            elif low_key == ord("l"):
                side_mode = "left"
            elif low_key == ord("r"):
                side_mode = "right"
            elif low_key == ord("n"):
                side_mode = "nearest"
            elif low_key == ord("v"):
                direction_mode = "image-down"
                if args.target == "inner":
                    offset_mm = abs(offset_mm)
            elif low_key == ord("o"):
                direction_mode = "outer"
            elif low_key == ord("f"):
                flip = not flip
            elif action == "save" and selection.valid and len(selected_line) > 0:
                confirmation_path = save_confirmation(
                    selection,
                    reader,
                    offset_mm,
                    flip,
                    direction_source,
                    selected_line,
                    selected_depths,
                    selected_surface_points,
                    line_shift_mm=line_shift_mm,
                )
                write_env(
                    ENV_PATH,
                    args.target,
                    offset_mm,
                    line_shift_mm,
                    side_mode,
                    direction_mode,
                    flip,
                    args.rotation,
                    args.try_rotations,
                )
                adjust_path = save_adjustment(
                    args.target,
                    offset_mm,
                    line_shift_mm,
                    side_mode,
                    direction_mode,
                    flip,
                    valid_ratio,
                    confirmation_path,
                    args.rotation,
                    args.try_rotations,
                )
                last_save_msg = f"saved env={ENV_PATH.name} json={adjust_path.name}"
                print(f"[Saved] env: {ENV_PATH}")
                print(f"[Saved] json: {adjust_path}")
                print(f"[Use] source {ENV_PATH} && ./massage start")
            elif action == "save":
                last_save_msg = "SAVE BLOCKED: wait for a valid left-leg line"
    finally:
        reader.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
