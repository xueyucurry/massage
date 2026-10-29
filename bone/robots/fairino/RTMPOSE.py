import argparse
import json
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import torch

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    AnyReader = object


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_BAGS = [
    "d435i_20s_20260420_213033_0.bag",
    "d435i_20s_20260420_213417_0.bag",
]
OUTPUT_DIR = "rtmpose_hip_knee_output"
SYNC_TOLERANCE_SEC = 0.08
DEPTH_SCALE = 0.001
DEFAULT_RTMPOSE_WEIGHT_NAME = "rtmpose-l_simcc-aic-coco_pt-aic-coco_420e-384x288-97d6cb0f_20230228.pth"
DEFAULT_RTMPOSE_WEIGHTS = str(SCRIPT_DIR / "weights" / DEFAULT_RTMPOSE_WEIGHT_NAME)
DEFAULT_RTMPOSE_WEIGHT_URL = (
    "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/"
    +
    DEFAULT_RTMPOSE_WEIGHT_NAME
)

# COCO 17-keypoint order.
LEFT_HIP = 11
RIGHT_HIP = 12
LEFT_KNEE = 13
RIGHT_KNEE = 14
LEFT_ANKLE = 15
RIGHT_ANKLE = 16

ROTATIONS = ("none", "ccw", "cw")


def resolve_default_rtmpose_config() -> str:
    try:
        import mmpose

        config_path = (
            Path(mmpose.__file__).resolve().parent
            / ".mim"
            / "configs"
            / "body_2d_keypoint"
            / "rtmpose"
            / "coco"
            / "rtmpose-l_8xb256-420e_aic-coco-384x288.py"
        )
        if config_path.is_file():
            return str(config_path)
    except Exception:
        pass
    return str(
        SCRIPT_DIR
        / "venv"
        / "Lib"
        / "site-packages"
        / "mmpose"
        / ".mim"
        / "configs"
        / "body_2d_keypoint"
        / "rtmpose"
        / "coco"
        / "rtmpose-l_8xb256-420e_aic-coco-384x288.py"
    )


DEFAULT_RTMPOSE_CONFIG = resolve_default_rtmpose_config()


def ros_time_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if hasattr(stamp, "secs") and hasattr(stamp, "nsecs"):
        return float(stamp.secs) + float(stamp.nsecs) * 1e-9
    return float(stamp)


def image_msg_to_numpy(msg) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding).lower()
    data = np.asarray(msg.data)

    if encoding == "rgb8":
        image = data.reshape(height, width, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return data.reshape(height, width, 3).copy()
    if encoding == "16uc1":
        return data.view(np.uint16).reshape(height, width).copy()
    if encoding == "mono8":
        return data.reshape(height, width).copy()
    raise ValueError(f"暂不支持的图像编码: {msg.encoding}")


def topic_matches(name: str, patterns: Sequence[str]) -> bool:
    return any(name.endswith(pattern) for pattern in patterns)


def pick_topic(topic_names: Sequence[str], explicit: Optional[str], patterns: Sequence[str]) -> Optional[str]:
    if explicit:
        return explicit if explicit in topic_names else None
    for name in topic_names:
        if topic_matches(name, patterns):
            return name
    return None


def rotate_image(image: np.ndarray, mode: str) -> np.ndarray:
    if mode == "none":
        return image
    if mode == "ccw":
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if mode == "cw":
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    raise ValueError(f"未知旋转模式: {mode}")


def unrotate_points(points: np.ndarray, mode: str, orig_shape: Tuple[int, int]) -> np.ndarray:
    if mode == "none":
        return points.astype(np.float32)
    height, width = orig_shape[:2]
    pts = np.asarray(points, dtype=np.float32).copy()
    out = np.zeros_like(pts, dtype=np.float32)
    if mode == "ccw":
        out[:, 0] = width - 1 - pts[:, 1]
        out[:, 1] = pts[:, 0]
        return out
    if mode == "cw":
        out[:, 0] = pts[:, 1]
        out[:, 1] = height - 1 - pts[:, 0]
        return out
    raise ValueError(f"未知旋转模式: {mode}")


def sample_depth_m(depth_image: Optional[np.ndarray], point: Sequence[float], radius: int = 2) -> Optional[float]:
    if depth_image is None:
        return None
    height, width = depth_image.shape[:2]
    cx, cy = int(round(point[0])), int(round(point[1]))
    values: List[float] = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = cx + dx, cy + dy
            if not (0 <= x < width and 0 <= y < height):
                continue
            raw = depth_image[y, x]
            value = float(raw) if np.issubdtype(depth_image.dtype, np.floating) else float(raw) * DEPTH_SCALE
            if value > 0.1:
                values.append(value)
    if not values:
        return None
    return float(np.median(values))


def flatten_pose_predictions(predictions) -> List[dict]:
    out: List[dict] = []
    if predictions is None:
        return out
    if isinstance(predictions, dict):
        if "keypoints" in predictions:
            out.append(predictions)
        for value in predictions.values():
            if isinstance(value, (list, tuple, dict)):
                out.extend(flatten_pose_predictions(value))
        return out
    if isinstance(predictions, (list, tuple)):
        for item in predictions:
            out.extend(flatten_pose_predictions(item))
    return out


@dataclass
class LegLineResult:
    valid: bool
    reason: str
    frame_index: int
    timestamp: float
    side: Optional[str] = None
    rotation: Optional[str] = None
    hip: Optional[List[float]] = None
    knee: Optional[List[float]] = None
    ankle: Optional[List[float]] = None
    hip_score: Optional[float] = None
    knee_score: Optional[float] = None
    ankle_score: Optional[float] = None
    side_score: Optional[float] = None
    mean_depth_m: Optional[float] = None


class RTMPoseHipKneeDetector:
    def __init__(
        self,
        pose2d: str,
        pose2d_weights: Optional[str],
        device: str,
        side: str,
        kpt_thr: float,
        rotations: Sequence[str] = ("none",),
    ):
        try:
            from mmpose.apis import inference_topdown, init_model
        except ImportError as exc:
            raise RuntimeError(
                "当前环境未安装 MMPose/RTMPose。\n"
                "建议在 venv 中安装：\n"
                "  python -m pip install -U openmim rosbags\n"
                "  python -m pip install mmengine mmcv-lite mmdet mmpose\n"
                "安装后再运行本脚本。"
            ) from exc

        self.inference_topdown = inference_topdown
        weights = pose2d_weights
        if not weights:
            weights = DEFAULT_RTMPOSE_WEIGHTS if Path(DEFAULT_RTMPOSE_WEIGHTS).is_file() else DEFAULT_RTMPOSE_WEIGHT_URL
        self.model = init_model(pose2d, weights, device=device)
        self.side = side
        self.kpt_thr = float(kpt_thr)
        self.rotations = tuple(rotations) if rotations else ("none",)

    def infer_frame(self, image: np.ndarray):
        h, w = image.shape[:2]
        bboxes = np.array([[0, 0, w, h]], dtype=np.float32)
        return self.inference_topdown(self.model, image, bboxes=bboxes, bbox_format="xyxy")

    def parse_candidates(self, result) -> List[Tuple[np.ndarray, np.ndarray]]:
        candidates: List[Tuple[np.ndarray, np.ndarray]] = []
        if isinstance(result, list):
            for sample in result:
                pred_instances = getattr(sample, "pred_instances", None)
                if pred_instances is None:
                    continue
                keypoints = np.asarray(getattr(pred_instances, "keypoints", []), dtype=np.float32)
                scores = np.asarray(getattr(pred_instances, "keypoint_scores", []), dtype=np.float32)
                if keypoints.ndim == 3:
                    keypoints = keypoints[0]
                if scores.ndim == 2:
                    scores = scores[0]
                if keypoints.ndim != 2 or keypoints.shape[0] < 17 or keypoints.shape[1] < 2:
                    continue
                if scores.ndim == 0 or len(scores) < 17:
                    scores = np.ones((keypoints.shape[0],), dtype=np.float32)
                candidates.append((keypoints[:, :2], scores))
            return candidates

        for pred in flatten_pose_predictions(result.get("predictions") if isinstance(result, dict) else None):
            keypoints = np.asarray(pred.get("keypoints", []), dtype=np.float32)
            scores = np.asarray(pred.get("keypoint_scores", pred.get("keypoint_score", [])), dtype=np.float32)
            if keypoints.ndim == 3:
                keypoints = keypoints[0]
            if scores.ndim == 2:
                scores = scores[0]
            if keypoints.ndim == 2 and keypoints.shape[0] >= 17 and keypoints.shape[1] >= 2:
                if scores.ndim == 0 or len(scores) < 17:
                    scores = np.ones((keypoints.shape[0],), dtype=np.float32)
                candidates.append((keypoints[:, :2], scores))
        return candidates

    def side_score(
        self,
        keypoints: np.ndarray,
        scores: np.ndarray,
        side: str,
        depth_image: Optional[np.ndarray],
    ) -> Tuple[float, Optional[float]]:
        if side == "left":
            hip_i, knee_i, ankle_i = LEFT_HIP, LEFT_KNEE, LEFT_ANKLE
        else:
            hip_i, knee_i, ankle_i = RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE

        conf_score = float(scores[hip_i] + scores[knee_i] + 0.5 * scores[ankle_i])
        hip_depth = sample_depth_m(depth_image, keypoints[hip_i])
        knee_depth = sample_depth_m(depth_image, keypoints[knee_i])
        valid_depths = [d for d in [hip_depth, knee_depth] if d is not None]
        mean_depth = float(np.mean(valid_depths)) if valid_depths else None

        # 深度越小越靠近相机；只作为轻量加分，避免覆盖置信度。
        if self.side == "nearest" and mean_depth is not None:
            conf_score += max(0.0, 4.0 - mean_depth) * 0.15
        return conf_score, mean_depth

    def select_leg(
        self,
        keypoints: np.ndarray,
        scores: np.ndarray,
        depth_image: Optional[np.ndarray],
    ) -> Tuple[str, float, Optional[float]]:
        if self.side in ("left", "right"):
            score, depth = self.side_score(keypoints, scores, self.side, depth_image)
            return self.side, score, depth
        if self.side == "nearest":
            left_score, left_depth = self.side_score(keypoints, scores, "left", depth_image)
            right_score, right_depth = self.side_score(keypoints, scores, "right", depth_image)
            if left_depth is not None and right_depth is not None and abs(left_depth - right_depth) > 0.03:
                return ("left", left_score, left_depth) if left_depth < right_depth else ("right", right_score, right_depth)
            return ("left", left_score, left_depth) if left_score >= right_score else ("right", right_score, right_depth)

        left_score, left_depth = self.side_score(keypoints, scores, "left", depth_image)
        right_score, right_depth = self.side_score(keypoints, scores, "right", depth_image)
        return ("left", left_score, left_depth) if left_score >= right_score else ("right", right_score, right_depth)

    def detect(self, image: np.ndarray, depth_image: Optional[np.ndarray], frame_index: int, timestamp: float) -> Tuple[np.ndarray, LegLineResult]:
        best = None
        best_score = -1.0
        orig_shape = image.shape[:2]

        for rotation in self.rotations:
            infer_image = rotate_image(image, rotation)
            try:
                result = self.infer_frame(infer_image)
            except Exception as exc:
                vis = image.copy()
                reason = f"RTMPose 推理失败: {exc}"
                cv2.putText(vis, reason[:80], (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                return vis, LegLineResult(False, reason, frame_index, timestamp)

            for keypoints, scores in self.parse_candidates(result):
                mapped = keypoints.copy()
                mapped[:, :2] = unrotate_points(mapped[:, :2], rotation, orig_shape)
                side, score, mean_depth = self.select_leg(mapped, scores, depth_image)
                if score > best_score:
                    best_score = score
                    best = (mapped, scores, side, rotation, mean_depth)

        vis = image.copy()
        if best is None:
            reason = "未检测到人体关键点"
            cv2.putText(vis, reason, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return vis, LegLineResult(False, reason, frame_index, timestamp)

        keypoints, scores, side, rotation, mean_depth = best
        if side == "left":
            hip_i, knee_i, ankle_i = LEFT_HIP, LEFT_KNEE, LEFT_ANKLE
        else:
            hip_i, knee_i, ankle_i = RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE

        hip_score = float(scores[hip_i])
        knee_score = float(scores[knee_i])
        ankle_score = float(scores[ankle_i])
        if hip_score < self.kpt_thr or knee_score < self.kpt_thr:
            reason = f"髋/膝关键点置信度不足 hip={hip_score:.2f}, knee={knee_score:.2f}"
            cv2.putText(vis, reason, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return vis, LegLineResult(
                False,
                reason,
                frame_index,
                timestamp,
                side=side,
                rotation=rotation,
                hip=keypoints[hip_i].tolist(),
                knee=keypoints[knee_i].tolist(),
                ankle=keypoints[ankle_i].tolist(),
                hip_score=hip_score,
                knee_score=knee_score,
                ankle_score=ankle_score,
                side_score=float(best_score),
                mean_depth_m=mean_depth,
            )

        hip = tuple(np.round(keypoints[hip_i]).astype(int).tolist())
        knee = tuple(np.round(keypoints[knee_i]).astype(int).tolist())

        cv2.line(vis, hip, knee, (0, 255, 255), 4)
        cv2.circle(vis, hip, 8, (0, 200, 255), -1)
        cv2.circle(vis, knee, 8, (0, 255, 0), -1)
        cv2.putText(vis, f"{side.upper()} HIP", (hip[0] + 8, hip[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.putText(vis, f"{side.upper()} KNEE", (knee[0] + 8, knee[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        depth_text = f" | depth={mean_depth:.2f}m" if mean_depth is not None else ""
        cv2.putText(
            vis,
            f"RTMPose hip-knee line | side={side} | rot={rotation} | score={best_score:.2f}{depth_text}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
        )

        return vis, LegLineResult(
            True,
            "ok",
            frame_index,
            timestamp,
            side=side,
            rotation=rotation,
            hip=keypoints[hip_i].tolist(),
            knee=keypoints[knee_i].tolist(),
            ankle=keypoints[ankle_i].tolist(),
            hip_score=hip_score,
            knee_score=knee_score,
            ankle_score=ankle_score,
            side_score=float(best_score),
            mean_depth_m=mean_depth,
        )


class RosbagFrameReader:
    def __init__(
        self,
        color_topic: Optional[str] = None,
        depth_topic: Optional[str] = None,
    ):
        self.color_topic_arg = color_topic
        self.depth_topic_arg = depth_topic

    def resolve_topics(self, reader: AnyReader) -> Tuple[str, Optional[str]]:
        if AnyReader is object:
            raise RuntimeError("当前环境未安装 rosbags，无法处理 bag。实时 RealSense 模式不需要 rosbags。")
        topic_names = [c.topic for c in reader.connections]
        color_topic = pick_topic(topic_names, self.color_topic_arg, ["/color/image_raw", "/rgb/image_raw"])
        depth_topic = pick_topic(
            topic_names,
            self.depth_topic_arg,
            ["/aligned_depth_to_color/image_raw", "/depth/image_rect_raw", "/depth/image_raw"],
        )
        if color_topic is None:
            raise RuntimeError(f"bag 中未找到彩色图 topic。可用 topics: {topic_names}")
        return color_topic, depth_topic

    def iter_frames(self, bag_path: str) -> Iterable[Tuple[np.ndarray, Optional[np.ndarray], float]]:
        with AnyReader([Path(bag_path)]) as reader:
            color_topic, depth_topic = self.resolve_topics(reader)
            selected_topics = {color_topic}
            if depth_topic is not None:
                selected_topics.add(depth_topic)
            connections = [c for c in reader.connections if c.topic in selected_topics]
            latest_depth = None
            latest_depth_ts = None

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                topic = connection.topic
                if topic == depth_topic:
                    latest_depth = image_msg_to_numpy(msg)
                    latest_depth_ts = ros_time_to_sec(msg.header.stamp if hasattr(msg, "header") else timestamp * 1e-9)
                    continue
                if topic == color_topic:
                    color = image_msg_to_numpy(msg)
                    ts = ros_time_to_sec(msg.header.stamp if hasattr(msg, "header") else timestamp * 1e-9)
                    depth = None
                    if latest_depth is not None and latest_depth_ts is not None and abs(ts - latest_depth_ts) <= SYNC_TOLERANCE_SEC:
                        depth = latest_depth.copy()
                    yield color, depth, ts


def resolve_bag_paths(raw_bags: Optional[Sequence[str]]) -> List[str]:
    bag_paths = list(raw_bags) if raw_bags else list(DEFAULT_BAGS)
    resolved: List[str] = []
    for bag in bag_paths:
        p = Path(bag)
        if not p.is_file():
            candidate = Path.cwd() / bag
            if candidate.is_file():
                p = candidate
        if not p.is_file():
            raise FileNotFoundError(f"未找到 bag 文件: {bag}")
        resolved.append(str(p))
    return resolved


def process_bag(
    bag_path: str,
    detector: RTMPoseHipKneeDetector,
    reader: RosbagFrameReader,
    output_dir: str,
    frame_step: int,
    display: bool,
    save_video: bool,
) -> Tuple[Optional[str], str]:
    bag_name = Path(bag_path).name
    records: List[LegLineResult] = []
    video_path = None
    timestamps: List[float] = []
    video_frames: List[np.ndarray] = []

    os.makedirs(output_dir, exist_ok=True)
    try:
        for frame_index, (color, depth, ts) in enumerate(reader.iter_frames(bag_path)):
            if frame_index % frame_step != 0:
                continue
            timestamps.append(ts)
            vis, record = detector.detect(color, depth, frame_index, ts)
            records.append(record)

            if save_video:
                video_frames.append(vis.copy())

            if display:
                cv2.imshow(f"RTMPose Hip-Knee | {bag_name}", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        if display:
            cv2.destroyAllWindows()

    if save_video and video_frames:
        if len(timestamps) >= 2:
            duration = max(timestamps[-1] - timestamps[0], 1e-3)
            fps = max(0.05, (len(timestamps) - 1) / duration)
        else:
            fps = 1.0
        video_path = os.path.join(output_dir, f"{Path(bag_name).stem}_rtmpose_hip_knee.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, (video_frames[0].shape[1], video_frames[0].shape[0]))
        try:
            for frame in video_frames:
                video_writer.write(frame)
        finally:
            video_writer.release()

    summary = {
        "bag_name": bag_name,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "video_path": os.path.abspath(video_path) if video_path else None,
        "frames_total": len(records),
        "frames_valid": sum(1 for r in records if r.valid),
        "valid_ratio": sum(1 for r in records if r.valid) / max(len(records), 1),
        "detections": [r.__dict__ for r in records],
    }
    summary_path = os.path.join(output_dir, f"{Path(bag_name).stem}_rtmpose_hip_knee.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    return video_path, summary_path


class RealSenseFrameReader:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        align_depth: bool = True,
    ):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.align_depth = bool(align_depth)
        self.pipeline = None
        self.align = None
        self.depth_scale = DEPTH_SCALE

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
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())
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

    def iter_frames(self) -> Iterable[Tuple[np.ndarray, Optional[np.ndarray], float]]:
        if self.pipeline is None:
            self.start()

        while True:
            frames = self.pipeline.wait_for_frames()
            if self.align is not None:
                frames = self.align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame:
                continue
            color = np.asanyarray(color_frame.get_data()).copy()
            depth = np.asanyarray(depth_frame.get_data()).copy() if depth_frame else None
            timestamp = float(color_frame.get_timestamp()) * 1e-3
            yield color, depth, timestamp


def process_realsense(
    detector: RTMPoseHipKneeDetector,
    reader: RealSenseFrameReader,
    output_dir: str,
    frame_step: int,
    display: bool,
    save_json: bool,
    max_frames: Optional[int],
) -> Optional[str]:
    global DEPTH_SCALE

    records: List[LegLineResult] = []
    last_wall = time.time()
    fps_smooth = 0.0
    summary_path = None

    os.makedirs(output_dir, exist_ok=True)
    try:
        reader.start()
        DEPTH_SCALE = reader.depth_scale
        for frame_index, (color, depth, ts) in enumerate(reader.iter_frames()):
            if frame_index % frame_step != 0:
                continue

            vis, record = detector.detect(color, depth, frame_index, ts)
            records.append(record)

            now = time.time()
            inst_fps = 1.0 / max(now - last_wall, 1e-6)
            fps_smooth = inst_fps if fps_smooth <= 0 else 0.9 * fps_smooth + 0.1 * inst_fps
            last_wall = now
            cv2.putText(
                vis,
                f"D435i realtime | frame={frame_index} | fps={fps_smooth:.1f} | q/ESC quit",
                (20, vis.shape[0] - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )

            if display:
                cv2.imshow("RTMPose D435i realtime", vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            if max_frames is not None and len(records) >= max_frames:
                break
    finally:
        reader.stop()
        if display:
            cv2.destroyAllWindows()

    if save_json:
        summary = {
            "source": "realsense",
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "frames_total": len(records),
            "frames_valid": sum(1 for r in records if r.valid),
            "valid_ratio": sum(1 for r in records if r.valid) / max(len(records), 1),
            "detections": [r.__dict__ for r in records],
        }
        summary_path = os.path.join(
            output_dir,
            f"realsense_rtmpose_hip_knee_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
        )
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print(f"[已导出] 结果: {os.path.abspath(summary_path)}")

    return summary_path


def parse_args():
    parser = argparse.ArgumentParser(description="使用 RTMPose 检测同侧髋部和膝盖，并绘制 hip -> knee 直线。")
    parser.add_argument("--source", choices=["realsense", "bag"], default="realsense", help="默认直接读取当前 D435i 实时画面。")
    parser.add_argument("--bags", nargs="*", default=None, help="仅 --source bag 时使用：要处理的 bag 文件。")
    parser.add_argument("--output-dir", default=OUTPUT_DIR)
    parser.add_argument("--frame-step", type=int, default=1, help="每隔多少帧处理一次。调试可用 30，正式可用 1。")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--no-save-video", action="store_true")
    parser.add_argument("--no-save-json", action="store_true", help="实时模式下不保存检测 JSON。")
    parser.add_argument("--side", choices=["auto", "nearest", "left", "right"], default="nearest", help="选择哪条腿。nearest 会优先选深度更近的一侧。")
    parser.add_argument("--kpt-thr", type=float, default=0.25, help="髋/膝关键点最低置信度。")
    parser.add_argument("--pose2d", default=DEFAULT_RTMPOSE_CONFIG, help="RTMPose 配置路径。")
    parser.add_argument("--pose2d-weights", default=None, help="可选 RTMPose 权重路径；为空则让 MMPose 自动下载。")
    parser.add_argument("--device", default="auto", help="auto/cpu/cuda:0。")
    parser.add_argument("--width", type=int, default=640, help="RealSense 彩色/深度宽度。")
    parser.add_argument("--height", type=int, default=480, help="RealSense 彩色/深度高度。")
    parser.add_argument("--fps", type=int, default=30, help="RealSense 帧率。")
    parser.add_argument("--max-frames", type=int, default=None, help="实时模式调试用：处理 N 帧后自动退出。")
    parser.add_argument("--no-align-depth", action="store_true", help="关闭深度对齐到彩色图。")
    parser.add_argument("--rotation", choices=ROTATIONS, default="none", help="实时推理前旋转图像。默认不旋转以保证速度。")
    parser.add_argument("--try-rotations", action="store_true", help="每帧尝试 none/ccw/cw 三个方向，准确但更慢。")
    parser.add_argument("--color-topic", default=None)
    parser.add_argument("--depth-topic", default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    device = "cuda:0" if args.device == "auto" and torch.cuda.is_available() else ("cpu" if args.device == "auto" else args.device)
    rotations = ROTATIONS if args.try_rotations else (args.rotation,)
    print(f"[RTMPose] config={args.pose2d}")
    print(f"[RTMPose] weights={args.pose2d_weights or DEFAULT_RTMPOSE_WEIGHTS}")
    print(f"[RTMPose] device={device}, rotations={','.join(rotations)}")

    detector = RTMPoseHipKneeDetector(
        pose2d=args.pose2d,
        pose2d_weights=args.pose2d_weights,
        device=device,
        side=args.side,
        kpt_thr=args.kpt_thr,
        rotations=rotations,
    )

    if args.source == "realsense":
        reader = RealSenseFrameReader(
            width=args.width,
            height=args.height,
            fps=args.fps,
            align_depth=not args.no_align_depth,
        )
        process_realsense(
            detector=detector,
            reader=reader,
            output_dir=args.output_dir,
            frame_step=max(1, args.frame_step),
            display=not args.no_display,
            save_json=not args.no_save_json,
            max_frames=args.max_frames,
        )
        return

    bags = resolve_bag_paths(args.bags)
    reader = RosbagFrameReader(color_topic=args.color_topic, depth_topic=args.depth_topic)
    for bag_path in bags:
        print(f"[处理开始] {bag_path}")
        video_path, summary_path = process_bag(
            bag_path=bag_path,
            detector=detector,
            reader=reader,
            output_dir=args.output_dir,
            frame_step=max(1, args.frame_step),
            display=not args.no_display,
            save_video=not args.no_save_video,
        )
        if video_path:
            print(f"[已导出] 视频: {os.path.abspath(video_path)}")
        print(f"[已导出] 结果: {os.path.abspath(summary_path)}")


if __name__ == "__main__":
    main()
