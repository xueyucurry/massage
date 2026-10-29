import argparse
import json
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import torch

from RTMPOSE import (
    DEFAULT_RTMPOSE_CONFIG,
    DEFAULT_RTMPOSE_WEIGHTS,
    LEFT_ANKLE,
    LEFT_HIP,
    LEFT_KNEE,
    RIGHT_ANKLE,
    RIGHT_HIP,
    RIGHT_KNEE,
    ROTATIONS,
    RTMPoseHipKneeDetector,
    rotate_image,
    unrotate_points,
)


SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "rtmpose_thigh_confirm_output"
WINDOW_NAME = "Thigh outer side line confirm"
DIRECTION_MODES = ("outer", "image-down", "image-up", "image-left", "image-right")


@dataclass
class PoseSelection:
    valid: bool
    reason: str
    keypoints: Optional[np.ndarray] = None
    scores: Optional[np.ndarray] = None
    side: str = "nearest"
    rotation: str = "none"
    score: float = 0.0
    mean_depth_m: Optional[float] = None


class RealSenseReader:
    def __init__(self, width: int, height: int, fps: int, align_depth: bool = True):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.align_depth = bool(align_depth)
        self.pipeline = None
        self.align = None
        self.rs = None
        self.depth_scale = 0.001
        self.color_intrinsics = None

    def start(self):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("当前 venv 未安装 pyrealsense2，无法读取 D435i。") from exc

        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        profile = self.pipeline.start(config)
        self.depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
        self.color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color) if self.align_depth else None
        print(
            f"[RealSense] D435i 已启动 color/depth={self.width}x{self.height}@{self.fps}, "
            f"depth_scale={self.depth_scale:.6f}, align_depth={'on' if self.align_depth else 'off'}"
        )

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
            print("[RealSense] 已关闭")

    def get_frame(self) -> Tuple[np.ndarray, Optional[np.ndarray], float]:
        if self.pipeline is None:
            self.start()
        frames = self.pipeline.wait_for_frames()
        if self.align is not None:
            frames = self.align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame:
            raise RuntimeError("未读取到彩色帧")
        color = np.asanyarray(color_frame.get_data()).copy()
        depth = np.asanyarray(depth_frame.get_data()).copy() if depth_frame else None
        timestamp = float(color_frame.get_timestamp()) * 1e-3
        return color, depth, timestamp

    def deproject(self, pixel: Sequence[float], depth_m: float) -> np.ndarray:
        return np.asarray(self.rs.rs2_deproject_pixel_to_point(self.color_intrinsics, [float(pixel[0]), float(pixel[1])], float(depth_m)))

    def project(self, point_m: Sequence[float]) -> np.ndarray:
        return np.asarray(self.rs.rs2_project_point_to_pixel(self.color_intrinsics, [float(point_m[0]), float(point_m[1]), float(point_m[2])]))


def sample_depth_m(depth: Optional[np.ndarray], pixel: Sequence[float], depth_scale: float, radius: int = 2) -> Optional[float]:
    if depth is None:
        return None
    height, width = depth.shape[:2]
    cx, cy = int(round(float(pixel[0]))), int(round(float(pixel[1])))
    values: List[float] = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = cx + dx, cy + dy
            if not (0 <= x < width and 0 <= y < height):
                continue
            raw = depth[y, x]
            value = float(raw) if np.issubdtype(depth.dtype, np.floating) else float(raw) * depth_scale
            if value > 0.1:
                values.append(value)
    if not values:
        return None
    return float(np.median(values))


def keypoint_indices(side: str) -> Tuple[int, int, int, int]:
    if side == "left":
        return LEFT_HIP, LEFT_KNEE, LEFT_ANKLE, RIGHT_HIP
    return RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE, LEFT_HIP


def score_side(keypoints: np.ndarray, scores: np.ndarray, depth: Optional[np.ndarray], depth_scale: float, side: str) -> Tuple[float, Optional[float]]:
    hip_i, knee_i, ankle_i, _ = keypoint_indices(side)
    depth_values = [
        sample_depth_m(depth, keypoints[hip_i], depth_scale),
        sample_depth_m(depth, keypoints[knee_i], depth_scale),
    ]
    valid_depths = [d for d in depth_values if d is not None]
    mean_depth = float(np.mean(valid_depths)) if valid_depths else None
    score = float(scores[hip_i] + scores[knee_i] + 0.5 * scores[ankle_i])
    if mean_depth is not None:
        score += max(0.0, 4.0 - mean_depth) * 0.15
    return score, mean_depth


def choose_side(keypoints: np.ndarray, scores: np.ndarray, depth: Optional[np.ndarray], depth_scale: float, side_mode: str) -> Tuple[str, float, Optional[float]]:
    left_score, left_depth = score_side(keypoints, scores, depth, depth_scale, "left")
    right_score, right_depth = score_side(keypoints, scores, depth, depth_scale, "right")
    if side_mode in ("left", "right"):
        return (side_mode, left_score, left_depth) if side_mode == "left" else (side_mode, right_score, right_depth)
    if side_mode == "nearest" and left_depth is not None and right_depth is not None and abs(left_depth - right_depth) > 0.03:
        return ("left", left_score, left_depth) if left_depth < right_depth else ("right", right_score, right_depth)
    return ("left", left_score, left_depth) if left_score >= right_score else ("right", right_score, right_depth)


def detect_pose(
    detector: RTMPoseHipKneeDetector,
    image: np.ndarray,
    depth: Optional[np.ndarray],
    depth_scale: float,
    side_mode: str,
    kpt_thr: float,
    rotations: Sequence[str],
) -> PoseSelection:
    best = None
    best_score = -1.0
    orig_shape = image.shape[:2]

    for rotation in rotations:
        infer_image = rotate_image(image, rotation)
        result = detector.infer_frame(infer_image)
        for keypoints, scores in detector.parse_candidates(result):
            mapped = keypoints.copy()
            mapped[:, :2] = unrotate_points(mapped[:, :2], rotation, orig_shape)
            side, score, mean_depth = choose_side(mapped, scores, depth, depth_scale, side_mode)
            if score > best_score:
                best_score = score
                best = (mapped, scores, side, rotation, mean_depth)

    if best is None:
        return PoseSelection(False, "未检测到人体关键点")

    keypoints, scores, side, rotation, mean_depth = best
    hip_i, knee_i, _, _ = keypoint_indices(side)
    hip_score = float(scores[hip_i])
    knee_score = float(scores[knee_i])
    if hip_score < kpt_thr or knee_score < kpt_thr:
        return PoseSelection(
            False,
            f"髋/膝关键点置信度不足 hip={hip_score:.2f}, knee={knee_score:.2f}",
            keypoints=keypoints,
            scores=scores,
            side=side,
            rotation=rotation,
            score=float(best_score),
            mean_depth_m=mean_depth,
        )

    return PoseSelection(
        True,
        "ok",
        keypoints=keypoints,
        scores=scores,
        side=side,
        rotation=rotation,
        score=float(best_score),
        mean_depth_m=mean_depth,
    )


def normalize(vec: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-6:
        return None
    return vec / norm


def estimate_outward_direction(
    selection: PoseSelection,
    depth: Optional[np.ndarray],
    reader: RealSenseReader,
    flip: bool,
    direction_mode: str = "outer",
) -> Tuple[Optional[np.ndarray], np.ndarray, str]:
    keypoints = selection.keypoints
    if keypoints is None:
        return None, np.array([1.0, 0.0], dtype=np.float32), "none"

    image_directions = {
        "image-down": np.array([0.0, 1.0], dtype=np.float32),
        "image-up": np.array([0.0, -1.0], dtype=np.float32),
        "image-left": np.array([-1.0, 0.0], dtype=np.float32),
        "image-right": np.array([1.0, 0.0], dtype=np.float32),
    }
    if direction_mode in image_directions:
        direction = image_directions[direction_mode]
        if flip:
            direction = -direction
        return None, direction, direction_mode

    hip_i, knee_i, _, opposite_hip_i = keypoint_indices(selection.side)
    hip = keypoints[hip_i]
    knee = keypoints[knee_i]
    opposite_hip = keypoints[opposite_hip_i]

    hip_depth = sample_depth_m(depth, hip, reader.depth_scale)
    opposite_depth = sample_depth_m(depth, opposite_hip, reader.depth_scale)
    if hip_depth is not None and opposite_depth is not None:
        outward_3d = reader.deproject(hip, hip_depth) - reader.deproject(opposite_hip, opposite_depth)
        outward_3d = normalize(outward_3d)
        if outward_3d is not None:
            if flip:
                outward_3d = -outward_3d
            outward_2d = normalize(hip - opposite_hip)
            if outward_2d is None:
                outward_2d = np.array([1.0, 0.0], dtype=np.float32)
            if flip:
                outward_2d = -outward_2d
            return outward_3d, outward_2d.astype(np.float32), "3d_hips"

    thigh = knee - hip
    outward_2d = normalize(np.array([-thigh[1], thigh[0]], dtype=np.float32))
    if outward_2d is None:
        outward_2d = np.array([1.0, 0.0], dtype=np.float32)
    hip_to_opposite = normalize(hip - opposite_hip)
    if hip_to_opposite is not None and float(np.dot(outward_2d, hip_to_opposite)) < 0:
        outward_2d = -outward_2d
    if flip:
        outward_2d = -outward_2d
    return None, outward_2d.astype(np.float32), "2d_fallback"


def build_offset_line(
    selection: PoseSelection,
    depth: Optional[np.ndarray],
    reader: RealSenseReader,
    outward_3d: Optional[np.ndarray],
    outward_2d: np.ndarray,
    offset_mm: float,
    sample_count: int,
    line_shift_mm: float = 0.0,
) -> Tuple[np.ndarray, List[Optional[float]], List[Optional[List[float]]]]:
    keypoints = selection.keypoints
    if keypoints is None:
        return np.empty((0, 2), dtype=np.float32), [], []

    hip_i, knee_i, _, _ = keypoint_indices(selection.side)
    hip = keypoints[hip_i].astype(np.float32)
    knee = keypoints[knee_i].astype(np.float32)
    thigh_2d = normalize(knee - hip)
    if thigh_2d is None:
        thigh_2d = np.array([0.0, 1.0], dtype=np.float32)
    thigh_3d = None
    hip_depth = sample_depth_m(depth, hip, reader.depth_scale)
    knee_depth = sample_depth_m(depth, knee, reader.depth_scale)
    if hip_depth is not None and knee_depth is not None:
        thigh_3d = normalize(reader.deproject(knee, knee_depth) - reader.deproject(hip, hip_depth))
    mean_depth = selection.mean_depth_m
    height, width = depth.shape[:2] if depth is not None else (reader.height, reader.width)
    points: List[np.ndarray] = []
    surface_depths: List[Optional[float]] = []
    surface_points: List[Optional[List[float]]] = []

    for t in np.linspace(0.0, 1.0, max(2, sample_count)):
        base_pixel = hip * (1.0 - t) + knee * t
        base_depth = sample_depth_m(depth, base_pixel, reader.depth_scale)
        if base_depth is None:
            base_depth = mean_depth

        if outward_3d is not None and base_depth is not None:
            base_3d = reader.deproject(base_pixel, base_depth)
            target_3d = base_3d + outward_3d * (float(offset_mm) * 0.001)
            if thigh_3d is not None:
                target_3d = target_3d + thigh_3d * (float(line_shift_mm) * 0.001)
            projected = reader.project(target_3d)
        else:
            depth_for_scale = base_depth if base_depth is not None else 1.0
            focal = 0.5 * (float(reader.color_intrinsics.fx) + float(reader.color_intrinsics.fy))
            offset_px = float(offset_mm) * 0.001 * focal / max(depth_for_scale, 1e-3)
            shift_px = float(line_shift_mm) * 0.001 * focal / max(depth_for_scale, 1e-3)
            projected = base_pixel + outward_2d * offset_px + thigh_2d * shift_px

        points.append(projected.astype(np.float32))
        if 0 <= projected[0] < width and 0 <= projected[1] < height:
            surface_depth = sample_depth_m(depth, projected, reader.depth_scale)
            surface_depths.append(surface_depth)
            if surface_depth is not None:
                surface_points.append(reader.deproject(projected, surface_depth).astype(float).tolist())
            else:
                surface_points.append(None)
        else:
            surface_depths.append(None)
            surface_points.append(None)

    return np.asarray(points, dtype=np.float32), surface_depths, surface_points


def draw_polyline(vis: np.ndarray, points: np.ndarray, color: Tuple[int, int, int], thickness: int) -> None:
    if len(points) < 2:
        return
    pts = np.round(points).astype(np.int32).reshape((-1, 1, 2))
    cv2.polylines(vis, [pts], False, color, thickness, cv2.LINE_AA)


def draw_text_box(vis: np.ndarray, lines: Sequence[str], x: int = 12, y: int = 24) -> None:
    line_h = 24
    max_chars = max((len(line) for line in lines), default=0)
    box_w = min(vis.shape[1] - x - 8, max(360, max_chars * 11))
    box_h = line_h * len(lines) + 12
    overlay = vis.copy()
    cv2.rectangle(overlay, (x - 6, y - 20), (x + box_w, y - 20 + box_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.45, vis, 0.55, 0, vis)
    for i, line in enumerate(lines):
        cv2.putText(vis, line, (x, y + i * line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)


def intrinsics_to_dict(intr) -> dict:
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "model": str(intr.model),
        "coeffs": [float(x) for x in intr.coeffs],
    }


def save_confirmation(
    selection: PoseSelection,
    reader: RealSenseReader,
    offset_mm: float,
    flip: bool,
    direction_source: str,
    line_points: np.ndarray,
    surface_depths: List[Optional[float]],
    surface_points: List[Optional[List[float]]],
    line_shift_mm: float = 0.0,
) -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    valid_depths = [d for d in surface_depths if d is not None]
    hip_i, knee_i, _, opposite_hip_i = keypoint_indices(selection.side)
    record = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "side": selection.side,
        "offset_mm": float(offset_mm),
        "line_shift_mm": float(line_shift_mm),
        "flip_outside": bool(flip),
        "direction_source": direction_source,
        "valid_depth_ratio": float(len(valid_depths) / max(len(surface_depths), 1)),
        "depth_scale": float(reader.depth_scale),
        "intrinsics": intrinsics_to_dict(reader.color_intrinsics),
        "keypoints": {
            "hip": selection.keypoints[hip_i].astype(float).tolist(),
            "knee": selection.keypoints[knee_i].astype(float).tolist(),
            "opposite_hip": selection.keypoints[opposite_hip_i].astype(float).tolist(),
            "hip_score": float(selection.scores[hip_i]),
            "knee_score": float(selection.scores[knee_i]),
        },
        "line_pixels": line_points.astype(float).tolist(),
        "line_surface_depth_m": [None if d is None else float(d) for d in surface_depths],
        "line_surface_points_camera_m": surface_points,
    }
    path = OUTPUT_DIR / f"thigh_outerline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
    return path


def parse_offsets(raw: str) -> List[float]:
    values = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    return values or [0.0, 30.0, 50.0, 70.0]


def parse_args():
    parser = argparse.ArgumentParser(description="实时确认平躺大腿外侧侧面中线。")
    parser.add_argument("--side", choices=["nearest", "auto", "left", "right"], default="nearest", help="默认选择离相机更近或置信度更高的一条腿。")
    parser.add_argument("--offset-mm", type=float, default=50.0, help="当前确认线的外侧偏移距离。")
    parser.add_argument("--offset-step-mm", type=float, default=5.0, help="按键调整偏移时的步长。")
    parser.add_argument("--direction", choices=DIRECTION_MODES, default="outer", help="偏移方向。outer 用髋部外侧方向；image-down 按画面向下平移。")
    parser.add_argument("--candidate-offsets-mm", default="0,30,50,70", help="同时显示的候选外侧偏移线，逗号分隔。")
    parser.add_argument("--samples", type=int, default=24, help="每条线采样点数。")
    parser.add_argument("--kpt-thr", type=float, default=0.25)
    parser.add_argument("--pose2d", default=DEFAULT_RTMPOSE_CONFIG)
    parser.add_argument("--pose2d-weights", default=None)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--rotation", choices=ROTATIONS, default="none")
    parser.add_argument("--try-rotations", action="store_true")
    parser.add_argument("--no-align-depth", action="store_true")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--max-frames", type=int, default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    device = "cuda:0" if args.device == "auto" and torch.cuda.is_available() else ("cpu" if args.device == "auto" else args.device)
    rotations = ROTATIONS if args.try_rotations else (args.rotation,)
    candidate_offsets = parse_offsets(args.candidate_offsets_mm)
    current_offset = float(args.offset_mm)
    side_mode = args.side
    direction_mode = args.direction
    flip_outside = False

    print(f"[RTMPose] config={args.pose2d}")
    print(f"[RTMPose] weights={args.pose2d_weights or DEFAULT_RTMPOSE_WEIGHTS}")
    print(f"[RTMPose] device={device}, rotations={','.join(rotations)}")
    print("[操作] q/ESC退出, s保存当前线, [或-减小偏移, ]或+增大偏移, l左腿, r右腿, n最近腿, a自动, d画面向下, o髋部外侧, f翻转方向")

    detector = RTMPoseHipKneeDetector(
        pose2d=args.pose2d,
        pose2d_weights=args.pose2d_weights,
        device=device,
        side="nearest",
        kpt_thr=args.kpt_thr,
        rotations=rotations,
    )
    reader = RealSenseReader(args.width, args.height, args.fps, align_depth=not args.no_align_depth)
    last_wall = time.time()
    fps_smooth = 0.0
    last_save_msg = ""
    frame_count = 0

    try:
        reader.start()
        while True:
            color, depth, _ = reader.get_frame()
            selection = detect_pose(detector, color, depth, reader.depth_scale, side_mode, args.kpt_thr, rotations)
            vis = color.copy()
            selected_line = np.empty((0, 2), dtype=np.float32)
            selected_depths: List[Optional[float]] = []
            selected_surface_points: List[Optional[List[float]]] = []
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
                    flip_outside,
                    direction_mode=direction_mode,
                )
                hip_i, knee_i, _, opposite_hip_i = keypoint_indices(selection.side)
                hip = selection.keypoints[hip_i]
                knee = selection.keypoints[knee_i]
                opposite_hip = selection.keypoints[opposite_hip_i]
                cv2.line(vis, tuple(np.round(hip).astype(int)), tuple(np.round(knee).astype(int)), (0, 255, 255), 3, cv2.LINE_AA)
                cv2.circle(vis, tuple(np.round(hip).astype(int)), 7, (0, 200, 255), -1, cv2.LINE_AA)
                cv2.circle(vis, tuple(np.round(knee).astype(int)), 7, (0, 255, 0), -1, cv2.LINE_AA)
                cv2.circle(vis, tuple(np.round(opposite_hip).astype(int)), 6, (255, 200, 0), -1, cv2.LINE_AA)

                colors = [(0, 255, 255), (255, 180, 0), (0, 200, 255), (120, 120, 255), (180, 255, 80)]
                for idx, offset in enumerate(candidate_offsets):
                    pts, _, _ = build_offset_line(
                        selection, depth, reader, outward_3d, outward_2d, offset, max(2, args.samples)
                    )
                    draw_polyline(vis, pts, colors[idx % len(colors)], 1 if abs(offset) > 1e-6 else 2)
                    if len(pts) > 0:
                        label_pt = np.round(pts[len(pts) // 2]).astype(int)
                        cv2.putText(
                            vis,
                            f"{offset:.0f}mm",
                            (int(label_pt[0]) + 4, int(label_pt[1]) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            colors[idx % len(colors)],
                            1,
                            cv2.LINE_AA,
                        )

                selected_line, selected_depths, selected_surface_points = build_offset_line(
                    selection, depth, reader, outward_3d, outward_2d, current_offset, max(2, args.samples)
                )
                draw_polyline(vis, selected_line, (255, 0, 255), 4)
                for pt, depth_m in zip(selected_line, selected_depths):
                    if not (0 <= pt[0] < vis.shape[1] and 0 <= pt[1] < vis.shape[0]):
                        continue
                    color_dot = (0, 255, 0) if depth_m is not None else (0, 0, 255)
                    cv2.circle(vis, tuple(np.round(pt).astype(int)), 3, color_dot, -1, cv2.LINE_AA)
                valid_ratio = sum(1 for d in selected_depths if d is not None) / max(len(selected_depths), 1)

                draw_text_box(
                    vis,
                    [
                        f"side={selection.side} mode={side_mode} offset={current_offset:.1f}mm direction={direction_mode} flip={flip_outside}",
                        f"score={selection.score:.2f} depth={selection.mean_depth_m if selection.mean_depth_m is not None else -1:.2f}m dir={direction_source}",
                        f"selected depth valid={valid_ratio * 100:.0f}% fps={fps_smooth:.1f}",
                        "[ ] adjust | l/r/n/a side | d down | o outer | f flip | s save | q quit",
                        last_save_msg,
                    ],
                )
            else:
                draw_text_box(
                    vis,
                    [
                        f"{selection.reason}",
                        f"mode={side_mode} offset={current_offset:.1f}mm fps={fps_smooth:.1f}",
                        "[ ] adjust | l/r/n/a side | d down | o outer | f flip | q quit",
                        last_save_msg,
                    ],
                )

            if not args.no_display:
                cv2.imshow(WINDOW_NAME, vis)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = 255

            if key in (ord("q"), 27):
                break
            if key in (ord("["), ord("-"), ord("_")):
                current_offset = max(0.0, current_offset - float(args.offset_step_mm))
            elif key in (ord("]"), ord("+"), ord("=")):
                current_offset += float(args.offset_step_mm)
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
                flip_outside = not flip_outside
            elif key == ord("s") and selection.valid and len(selected_line) > 0:
                path = save_confirmation(
                    selection,
                    reader,
                    current_offset,
                    flip_outside,
                    direction_source,
                    selected_line,
                    selected_depths,
                    selected_surface_points,
                )
                last_save_msg = f"saved: {path.name}"
                print(f"[保存] {path}")

            frame_count += 1
            if args.max_frames is not None and frame_count >= args.max_frames:
                break
    finally:
        reader.stop()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
