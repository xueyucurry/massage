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

from RTMPOSE import DEFAULT_RTMPOSE_CONFIG, DEFAULT_RTMPOSE_WEIGHTS, ROTATIONS, RTMPoseHipKneeDetector
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
WINDOW_NAME = "Thigh offset adjust"


def _target_env_names(target: str):
    if target == "inner":
        return "THIGH_INNER_OFFSET_MM", "THIGH_INNER_LINE_SHIFT_MM", "leg_inner"
    return "THIGH_OUTER_OFFSET_MM", "THIGH_OUTER_LINE_SHIFT_MM", "leg"


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
    parser.add_argument("--side", choices=["nearest", "auto", "left", "right"], default=os.environ.get("THIGH_SIDE", "right"))
    parser.add_argument("--offset-mm", type=float, default=offset_default, help="Lateral outward offset.")
    parser.add_argument("--line-shift-mm", type=float, default=line_shift_default, help="Shift along hip-to-knee line. Positive moves toward knee.")
    parser.add_argument("--offset-step-mm", type=float, default=5.0)
    parser.add_argument("--shift-step-mm", type=float, default=10.0)
    parser.add_argument("--direction", choices=DIRECTION_MODES, default=os.environ.get("THIGH_DIRECTION", "image-down"))
    parser.add_argument("--samples", type=int, default=int(_env_float("THIGH_SAMPLE_POINTS", 10)))
    parser.add_argument("--kpt-thr", type=float, default=_env_float("THIGH_KPT_THR", 0.25))
    parser.add_argument("--pose2d", default=DEFAULT_RTMPOSE_CONFIG)
    parser.add_argument("--pose2d-weights", default=None)
    parser.add_argument("--device", default=os.environ.get("THIGH_DEVICE", "auto"))
    parser.add_argument("--width", type=int, default=int(_env_float("THIGH_CAMERA_WIDTH", 640)))
    parser.add_argument("--height", type=int, default=int(_env_float("THIGH_CAMERA_HEIGHT", 480)))
    parser.add_argument("--fps", type=int, default=int(_env_float("THIGH_CAMERA_FPS", 30)))
    parser.add_argument("--rotation", choices=ROTATIONS, default=os.environ.get("THIGH_ROTATION", "none"))
    parser.add_argument("--try-rotations", action="store_true")
    parser.add_argument("--no-align-depth", action="store_true")
    return parser.parse_args()


def write_env(path: Path, target: str, offset_mm: float, line_shift_mm: float, side: str, direction: str, flip: bool) -> None:
    offset_env, line_shift_env, massage_target = _target_env_names(target)
    path.write_text(
        "\n".join(
            [
                f'export {offset_env}="{offset_mm:.1f}"',
                f'export {line_shift_env}="{line_shift_mm:.1f}"',
                f'export THIGH_SIDE="{side}"',
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
) -> Path:
    offset_env, line_shift_env, massage_target = _target_env_names(target)
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
    flip = os.environ.get("THIGH_FLIP_DIRECTION", "0").strip().lower() in {"1", "true", "yes", "on"}

    print(f"[RTMPose] config={args.pose2d}")
    print(f"[RTMPose] weights={args.pose2d_weights or DEFAULT_RTMPOSE_WEIGHTS}")
    print(f"[RTMPose] device={device}, rotations={','.join(rotations)}")
    print(f"[Target] {args.target}: saves {offset_env}/{line_shift_env}, MASSAGE_TARGET={massage_target}")
    print("[Keys] q quit | s save | [ ] lateral offset | w/x line shift | l/r/n/a side | d image-down | o outer | f flip")
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
                    "[ ] lateral | w hip/up | x knee/down | s save | q quit",
                    last_save_msg,
                ]
            else:
                status = [
                    selection.reason,
                    f"target={args.target} mode={side_mode} {offset_env}={offset_mm:.1f} {line_shift_env}={line_shift_mm:.1f}",
                    "[ ] lateral | w hip/up | x knee/down | s save | q quit",
                    last_save_msg,
                ]

            draw_text_box(vis, status)
            cv2.imshow(WINDOW_NAME, vis)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key in (ord("["), ord("-"), ord("_")):
                offset_mm = max(0.0, offset_mm - float(args.offset_step_mm))
            elif key in (ord("]"), ord("+"), ord("=")):
                offset_mm += float(args.offset_step_mm)
            elif key == ord("w"):
                line_shift_mm -= float(args.shift_step_mm)
            elif key == ord("x"):
                line_shift_mm += float(args.shift_step_mm)
            elif key == ord("l"):
                side_mode = "left"
            elif key == ord("r"):
                side_mode = "right"
            elif key == ord("n"):
                side_mode = "nearest"
            elif key == ord("a"):
                side_mode = "auto"
            elif key == ord("d"):
                direction_mode = "image-down"
            elif key == ord("o"):
                direction_mode = "outer"
            elif key == ord("f"):
                flip = not flip
            elif key == ord("s") and selection.valid and len(selected_line) > 0:
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
                write_env(ENV_PATH, args.target, offset_mm, line_shift_mm, side_mode, direction_mode, flip)
                adjust_path = save_adjustment(args.target, offset_mm, line_shift_mm, side_mode, direction_mode, flip, valid_ratio, confirmation_path)
                last_save_msg = f"saved env={ENV_PATH.name} json={adjust_path.name}"
                print(f"[Saved] env: {ENV_PATH}")
                print(f"[Saved] json: {adjust_path}")
                print(f"[Use] source {ENV_PATH} && ./massage start")
    finally:
        reader.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
