#!/usr/bin/env python3
"""Continuous, vision-only OpenCV viewer for massage target detection."""

import argparse
import os
import sys

import cv2
import numpy as np
import torch

import ft
from lasttime import LinearMeridianDetector, MeridianLineStabilizer, PointSmoother


TARGET_ALIASES = {
    "back": "back",
    "spine": "back",
    "thigh": "leg",
    "outer": "leg",
    "thigh-outer": "leg",
    "outer-thigh": "leg",
    "leg": "leg",
    "leg-outer": "leg",
    "thigh-inner": "leg_inner",
    "inner": "leg_inner",
    "inner-thigh": "leg_inner",
    "leg-inner": "leg_inner",
    "inner-leg": "leg_inner",
}

TARGET_DISPLAY_NAMES = {
    "back": "back",
    "leg": "thigh",
    "leg_inner": "thigh-inner",
}


def normalize_view_target(value):
    text = str(value or "").strip().lower().replace("_", "-")
    target = TARGET_ALIASES.get(text)
    if target is None:
        choices = "back, thigh, thigh-inner"
        raise argparse.ArgumentTypeError(f"unknown target {value!r}; choose one of: {choices}")
    return target


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Continuously show the live OpenCV detection overlay. "
            "This viewer never saves a trajectory or connects to the robot."
        )
    )
    parser.add_argument(
        "target",
        type=normalize_view_target,
        help="back | thigh | thigh-inner (leg/inner-thigh aliases are also accepted)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="exit after N frames; intended for diagnostics",
    )
    return parser


def _open_window(window_name):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.moveWindow(
        window_name,
        int(os.environ.get("FT_DETECTION_WINDOW_X", "60")),
        int(os.environ.get("FT_DETECTION_WINDOW_Y", "80")),
    )


def _show_frame(window_name, frame):
    cv2.imshow(window_name, frame)
    key = cv2.waitKey(1) & 0xFF
    if key in (ord("q"), 27):
        return False
    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1.0
    except cv2.error:
        return True


def _frame_limit_reached(frame_count, max_frames):
    return max_frames is not None and frame_count >= max(1, int(max_frames))


def _create_vision_only_demo(target):
    previous_inner_seed = ft.THIGH_INNER_POSTURE_SEED_ENABLE
    previous_back_seed = ft.BACK_POSTURE_SEED_ENABLE
    try:
        ft.THIGH_INNER_POSTURE_SEED_ENABLE = False
        ft.BACK_POSTURE_SEED_ENABLE = False
        return ft.LastTimeRos2Demo(massage_target=target)
    finally:
        ft.THIGH_INNER_POSTURE_SEED_ENABLE = previous_inner_seed
        ft.BACK_POSTURE_SEED_ENABLE = previous_back_seed


def _init_back_vision_only(demo):
    print("初始化背部纯视觉检测...")
    demo.detector = LinearMeridianDetector(45.0)
    demo.smoother = PointSmoother(alpha=0.25, max_step_px=12.0)
    demo.stabilizer = MeridianLineStabilizer()
    print("背部纯视觉检测初始化完成")


def _init_thigh_vision_only(demo):
    print("初始化腿部纯视觉检测...")
    device = "cuda:0" if ft.THIGH_DEVICE == "auto" and torch.cuda.is_available() else (
        "cpu" if ft.THIGH_DEVICE == "auto" else ft.THIGH_DEVICE
    )
    rotations = ft._thigh_rotations_for_massage_target(demo.massage_target)
    print(f"RTMPose device={device}, rotations={','.join(rotations)}")
    demo.thigh_rotations = rotations
    demo.thigh_pose_detector = ft.RTMPoseHipKneeDetector(
        pose2d=ft.DEFAULT_RTMPOSE_CONFIG,
        pose2d_weights=None,
        device=device,
        side="nearest",
        kpt_thr=ft.THIGH_KPT_THR,
        rotations=rotations,
    )
    print("腿部纯视觉检测初始化完成")


def run_back_viewer(demo, max_frames=None):
    window_name = "Massage Detection - Back"
    _init_back_vision_only(demo)
    _open_window(window_name)
    stable_count = 0
    frame_count = 0

    while True:
        frames = demo.detector.pipeline.wait_for_frames()
        if demo.detector.align is not None:
            frames = demo.detector.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            stable_count = 0
            continue

        image = np.asanyarray(color_frame.get_data())
        demo.frame_idx += 1
        analysis = demo._analyze_visual_frame(image)
        analysis = demo._attach_back_depth_samples(analysis, depth_frame, require_depth=True)
        ready = bool(analysis.get("visual_motion_ready", False))
        stable_count = stable_count + 1 if ready else 0
        status = "READY" if ready else str(analysis.get("visual_status", "search")).upper()

        view = demo._draw_detection_overlay(image, analysis)
        demo._draw_command_banner(
            view,
            [
                f"VIEW ONLY | back | status={status} | stable={stable_count}",
                "q / Esc quit | no save | robot disabled",
            ],
        )
        frame_count += 1
        if not _show_frame(window_name, view) or _frame_limit_reached(frame_count, max_frames):
            break


def run_thigh_viewer(demo, max_frames=None):
    target = demo.massage_target
    target_name = TARGET_DISPLAY_NAMES[target]
    window_name = f"Massage Detection - {target_name}"
    side = ft._thigh_side_for_massage_target(target)
    offset_mm = ft._thigh_offset_for_massage_target(target)
    line_shift_mm = ft._thigh_line_shift_for_massage_target(target)
    stable_count = 0
    frame_count = 0

    _init_thigh_vision_only(demo)
    reader = ft.ThighRealSenseReader(
        ft.THIGH_WIDTH,
        ft.THIGH_HEIGHT,
        ft.THIGH_FPS,
        align_depth=ft.THIGH_ALIGN_DEPTH,
    )
    _open_window(window_name)
    reader_started = False

    try:
        reader.start()
        reader_started = True
        while True:
            color, depth, _ = reader.get_frame()
            view = color.copy()
            selection = ft.detect_thigh_pose(
                demo.thigh_pose_detector,
                color,
                depth,
                reader.depth_scale,
                side,
                ft.THIGH_KPT_THR,
                demo.thigh_rotations,
            )
            valid_ratio = 0.0
            skipped_points = 0

            if selection.valid and selection.keypoints is not None and selection.scores is not None:
                outward_3d, outward_2d, direction_source = ft.estimate_thigh_outward_direction(
                    selection,
                    depth,
                    reader,
                    ft.THIGH_FLIP_DIRECTION,
                    direction_mode=ft.THIGH_DIRECTION,
                )
                line_pixels, depths, surface_points = ft.build_thigh_offset_line(
                    selection,
                    depth,
                    reader,
                    outward_3d,
                    outward_2d,
                    offset_mm,
                    max(2, ft.THIGH_SAMPLE_POINTS),
                    line_shift_mm=line_shift_mm,
                )
                line_pixels, depths, _surface_points, skipped_points = demo._crop_thigh_target_samples(
                    line_pixels,
                    depths,
                    surface_points,
                )
                valid_ratio = sum(depth_m is not None for depth_m in depths) / max(len(depths), 1)
                stable_count = stable_count + 1 if valid_ratio >= ft.THIGH_MIN_DEPTH_RATIO else 0

                ft.draw_thigh_polyline(view, line_pixels, (255, 0, 255), 4)
                for point, depth_m in zip(line_pixels, depths):
                    if not (0 <= point[0] < view.shape[1] and 0 <= point[1] < view.shape[0]):
                        continue
                    dot_color = (0, 255, 0) if depth_m is not None else (0, 0, 255)
                    cv2.circle(
                        view,
                        tuple(np.round(point).astype(int)),
                        3,
                        dot_color,
                        -1,
                        cv2.LINE_AA,
                    )

                ft.draw_thigh_text_box(
                    view,
                    [
                        f"VIEW ONLY | {target_name} | side={selection.side} pose_rot={selection.rotation}",
                        f"offset={offset_mm:.1f}mm shift={line_shift_mm:.1f}mm dir={ft.THIGH_DIRECTION}",
                        f"depth valid={valid_ratio * 100:.0f}% stable={stable_count} skip={skipped_points} source={direction_source}",
                    ],
                )
            else:
                stable_count = 0
                ft.draw_thigh_text_box(
                    view,
                    [
                        f"VIEW ONLY | {target_name} | searching",
                        selection.reason,
                    ],
                )

            demo._draw_command_banner(
                view,
                ["q / Esc quit | no save | robot disabled"],
            )
            frame_count += 1
            if not _show_frame(window_name, view) or _frame_limit_reached(frame_count, max_frames):
                break
    finally:
        if reader_started:
            try:
                reader.stop()
            except Exception as exc:
                print(f"[Viewer] RealSense close warning: {exc}", file=sys.stderr)
        else:
            reader.pipeline = None


def run(args):
    demo = _create_vision_only_demo(args.target)
    try:
        print(f"[Viewer] target={TARGET_DISPLAY_NAMES[args.target]}")
        print("[Viewer] 纯视觉模式：不会保存轨迹，不会连接或移动机械臂。")
        print("[Viewer] 按 q 或 Esc 退出。")
        if args.target == "back":
            run_back_viewer(demo, max_frames=args.max_frames)
        else:
            run_thigh_viewer(demo, max_frames=args.max_frames)
        return 0
    finally:
        pipeline = getattr(getattr(demo, "detector", None), "pipeline", None)
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()


def main():
    args = build_parser().parse_args()
    try:
        return run(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"检测画面启动失败: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
