"""
Demo: 膀胱经检测 + 机械臂运动实时展示

工作流程:
  1. 实时检测: RealSense + YOLO + CoTracker 持续输出脊柱与膀胱经轨迹
  2. 基于脊柱左右两侧显示四条膀胱经:
     近侧两条 = 输入偏移量
     外侧两条 = 输入偏移量的两倍
  3. 发生遮挡或人体移动时，优先依靠 CoTracker 脊柱采样点继续稳定绘制
  4. 按 s 可选保存当前检测轨迹，便于离线排查

操作:
  r = 启动单侧固定轨迹力控演示
  t = 启动单侧悬空遮挡测试
  s = 保存当前检测到的膀胱经轨迹
  f = 切换力控模式 (ft_control ↔ impedance)，仅 idle 可切
  +/- = 切换目标力档位 (5~49N)
  q = 退出 (运动中按 q 先停止机器人再退出)
  u/d = 调整手指偏移宽度 +/-5mm

环境变量:
  DEMO_SIDE_LYING=1     侧卧按摩头演示预设：默认关闭 FT_Guard，优先视觉稳定后再允许启动按摩
  FORCE_CONTROL=0       回退到原始无力控模式 (固定 Z)
  FT_COLLISION_GUARD=0  关闭 FT_Guard 碰撞守护 (侧躺大力按压演示用; 仍有软件 |Fz| 上限)
  DEMO_START_POSE      可选固定起始位，格式 x,y,z,rx,ry,rz；不设时按 r/t 会直接读取当前位置
  DEMO_SKIP_HOME=1      启动时不回 p24，仅摄像头膀胱经实时检测 (不按 r/t 则机械臂不动)
  TARGET_FORCE_N        初始目标按压力 (N)，仍可用 +/- 换挡
"""

import json
import math
import os
import sys
import threading
import time
from dataclasses import replace

import cv2
import numpy as np
import torch

from yolo import LinearMeridianDetector, infer_best_pose_with_rotations, get_body_lateral_direction_2d
from dianjing import (
    _load_camera_to_robot_matrix,
    _transform_points,
    _to_mm_points,
    _load_points_prefer_current_calibration,
)
from force_control import (
    ForceControlConfig,
    MOVE_CART_BLEND_BLOCKING,
    _densify_path_xy,
    _tool_z_unit_from_rpy,
    disable_collision_guard,
    find_surface,
    get_force_z,
    init_force_sensor,
    setup_collision_guard,
)

_COTRACKER_DIR = os.path.join(os.path.dirname(__file__), "third_party", "co-tracker")
if _COTRACKER_DIR not in sys.path:
    # 放到末尾，避免 third_party/co-tracker/demo.py 抢占当前项目的 demo.py
    sys.path.append(_COTRACKER_DIR)

try:
    import hubconf as cotracker_hub
except Exception:
    cotracker_hub = None

try:
    from cotracker.utils.visualizer import Visualizer as CoTrackerVisualizer
except Exception:
    CoTrackerVisualizer = None


# ===================== 固定初始位姿（p24） =====================

INIT_POSE_P24 = [104.443, 538.974, 299.820, -178.203, 1.594, -33.748]
INIT_TOOL = 0
INIT_USER = 0
INIT_SAFE_Z_MM = 300.0
DEFAULT_ROBOT_IP = "192.168.58.2"

FORCE_LEVELS_N = [5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 49.0]
ACTIVE_SIDE_NAME = "left"


def _env_flag(name: str, default: bool = False) -> bool:
    v = os.environ.get(name, "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "on")


def _env_first_nonempty(*names: str) -> str:
    for name in names:
        value = os.environ.get(name, "").strip()
        if value:
            return value
    return ""


def _side_lying_demo_enabled() -> bool:
    return _env_flag("DEMO_SIDE_LYING", False)


def _collision_guard_enabled_from_env() -> bool:
    v = os.environ.get("FT_COLLISION_GUARD", "").strip().lower()
    if v:
        return v not in ("0", "false", "off", "no")
    return not _side_lying_demo_enabled()


def _build_force_config_from_env() -> ForceControlConfig:
    """从环境变量构建力控配置；当前 demo 固定使用零负载/零质心补偿。"""
    cfg = ForceControlConfig()
    if _side_lying_demo_enabled():
        cfg.cart_max_xy_step_mm = min(float(cfg.cart_max_xy_step_mm), 6.0)
        cfg.orient_smooth_alpha = max(float(cfg.orient_smooth_alpha), 0.35)
        print("[Demo] DEMO_SIDE_LYING=1：侧卧按摩头演示预设已启用 (视觉稳定优先 / FT_Guard 默认关闭)")
    cfg.enable_collision_guard = _collision_guard_enabled_from_env()
    if not cfg.enable_collision_guard:
        print("[Demo] FT_COLLISION_GUARD=0：已禁用控制器碰撞守护 (按压时勿让人体突然大幅移动)")
    cfg.payload_weight = 0.0
    cfg.payload_cog = [0.0, 0.0, 0.0]
    if _env_first_nonempty("FT_PAYLOAD_WEIGHT_KG", "MASSAGE_HEAD_WEIGHT_KG", "FT_PAYLOAD_COG_MM", "MASSAGE_HEAD_COG_MM"):
        print("[Demo] 当前 demo 已固定为无负载/无质心补偿；已忽略 FT_PAYLOAD_* / MASSAGE_HEAD_*")
    return cfg


def _snap_force_level(value_n: float) -> float:
    value = float(value_n)
    return min(FORCE_LEVELS_N, key=lambda x: abs(x - value))


def _parse_pose6_text(text: str):
    parts = [p.strip() for p in text.replace(";", ",").split(",") if p.strip()]
    if len(parts) != 6:
        return None
    try:
        return [float(v) for v in parts]
    except ValueError:
        return None

# Robot SDK
try:
    from fairino import Robot
except Exception:
    try:
        from src.user_functions.fairino import Robot
    except Exception:
        import sys
        import importlib

        _dir = os.path.dirname(__file__)
        if _dir not in sys.path:
            sys.path.insert(0, _dir)
        Robot = importlib.import_module("fairino").Robot


# ===================== EMA Point Smoother =====================

class PointSmoother:
    """
    指数移动平均平滑器，消除 YOLO 关键点抖动和深度传感器噪声。

    alpha: 新数据权重 (0~1)。越小越平滑但延迟越大。
           30fps 下 0.35 是较好的平衡点。
    miss_tolerance: 连续多少帧丢失检测后重置状态。
    """

    def __init__(self, alpha=0.25, miss_tolerance=5, max_step_px=12.0):
        self.alpha = alpha
        self.miss_tolerance = miss_tolerance
        self.max_step_px = max_step_px
        self._state = {}   # name -> (x_float, y_float)
        self._miss_count = 0

    def smooth(self, name, x, y, round_result=True):
        """平滑单个命名点，支持输出 float 或 int。"""
        self._miss_count = 0
        x = float(x)
        y = float(y)
        if name not in self._state:
            self._state[name] = (x, y)
        else:
            ox, oy = self._state[name]
            # 限制单帧最大跳变，抑制偶发误检导致的闪跳
            dx = x - ox
            dy = y - oy
            dist = (dx * dx + dy * dy) ** 0.5
            if self.max_step_px is not None and dist > self.max_step_px > 0:
                scale = self.max_step_px / dist
                x = ox + dx * scale
                y = oy + dy * scale
            self._state[name] = (
                self.alpha * x + (1 - self.alpha) * ox,
                self.alpha * y + (1 - self.alpha) * oy,
            )
        sx, sy = self._state[name]
        if round_result:
            return int(round(sx)), int(round(sy))
        return sx, sy

    def miss(self):
        """当前帧未检测到人体时调用。"""
        self._miss_count += 1
        if self._miss_count >= self.miss_tolerance:
            self.reset()

    def reset(self):
        self._state.clear()
        self._miss_count = 0


class MeridianLineStabilizer:
    """
    对最终膀胱经线做末级稳定：
    1. 限制单帧端点跳变，抑制 YOLO / CoTracker 切换时的闪跳
    2. 短时遮挡或人体轻微移动时保持上一条稳定线，避免检测线瞬间消失
    3. 只有累计稳定若干帧后才视为 ready，允许启动按摩演示
    """

    def __init__(
        self,
        alpha=0.22,
        reacquire_alpha=0.45,
        max_step_px=18.0,
        hold_frames=18,
        ready_frames=8,
    ):
        self.alpha = float(alpha)
        self.reacquire_alpha = float(reacquire_alpha)
        self.max_step_px = float(max_step_px)
        self.hold_frames = int(hold_frames)
        self.ready_frames = int(ready_frames)
        self.reset()

    def reset(self):
        self._points = None
        self._stable_frames = 0
        self._miss_frames = 0
        self._status = "search"
        self._source = "none"

    @staticmethod
    def _as_point(point):
        return np.asarray(point, dtype=np.float32).reshape(2)

    def _blend_point(self, prev_point, new_point, alpha):
        prev = self._as_point(prev_point)
        cur = self._as_point(new_point)
        delta = cur - prev
        dist = float(np.linalg.norm(delta))
        if self.max_step_px > 0.0 and dist > self.max_step_px:
            cur = prev + delta * (self.max_step_px / max(dist, 1e-6))
        return alpha * cur + (1.0 - alpha) * prev

    def _candidate_from_lines(self, meridian_lines):
        if (
            meridian_lines is None
            or len(meridian_lines) != 2
            or meridian_lines[0] is None
            or meridian_lines[1] is None
        ):
            return None
        (neck_l, tail_l), (neck_r, tail_r) = meridian_lines
        return {
            "neck_l": self._as_point(neck_l),
            "tail_l": self._as_point(tail_l),
            "neck_r": self._as_point(neck_r),
            "tail_r": self._as_point(tail_r),
        }

    def _build_lines(self):
        if self._points is None:
            return None, None
        neck_l = self._points["neck_l"]
        tail_l = self._points["tail_l"]
        neck_r = self._points["neck_r"]
        tail_r = self._points["tail_r"]
        neck_c = (neck_l + neck_r) * 0.5
        tail_c = (tail_l + tail_r) * 0.5
        spine_line = (
            (float(neck_c[0]), float(neck_c[1])),
            (float(tail_c[0]), float(tail_c[1])),
        )
        meridian_lines = (
            ((float(neck_l[0]), float(neck_l[1])), (float(tail_l[0]), float(tail_l[1]))),
            ((float(neck_r[0]), float(neck_r[1])), (float(tail_r[0]), float(tail_r[1]))),
        )
        return spine_line, meridian_lines

    def snapshot(self, quality=0.0):
        spine_line, meridian_lines = self._build_lines()
        return {
            "spine_line": spine_line,
            "meridian_lines": meridian_lines,
            "ready": self._stable_frames >= self.ready_frames and meridian_lines is not None,
            "status": self._status,
            "source": self._source,
            "quality": float(quality),
            "stable_frames": int(self._stable_frames),
            "miss_frames": int(self._miss_frames),
        }

    def update(self, meridian_lines, line_source="none", pose_conf=0.0, tracker_vis_ratio=0.0):
        candidate = self._candidate_from_lines(meridian_lines)
        quality = float(max(0.0, min(1.0, max(float(pose_conf), float(tracker_vis_ratio)))))

        if candidate is None:
            if self._points is None:
                self._status = "search"
                self._source = "none"
                return self.snapshot(quality=quality)
            self._miss_frames += 1
            if self._miss_frames <= self.hold_frames:
                self._status = "hold"
                return self.snapshot(quality=quality)
            self.reset()
            return self.snapshot(quality=quality)

        alpha = self.reacquire_alpha if self._miss_frames > 0 else self.alpha
        if quality >= 0.78:
            alpha = max(alpha, 0.38)

        if self._points is None:
            self._points = candidate
            self._stable_frames = 1
        else:
            self._points = {
                name: self._blend_point(self._points[name], point, alpha)
                for name, point in candidate.items()
            }
            self._stable_frames += 1

        self._miss_frames = 0
        self._source = line_source if line_source != "none" else self._source
        self._status = "stable" if self._stable_frames >= self.ready_frames else "warming"
        return self.snapshot(quality=quality)


# ===================== CoTracker Spine Tracker =====================

class BackRegionCoTracker:
    """
    用四个关键点先定位脊柱，再只在脊柱线上均匀采样点并持续跟踪。
    膀胱经始终由鲁棒脊柱线 + 侧向偏移生成；遮挡/漏检时优先沿脊柱跟踪恢复。
    """

    def __init__(
        self,
        num_points=40,
        reseed_interval=18,
        min_visible_ratio=0.20,
        line_smooth_alpha=0.28,
        occluded_point_weight=0.2,
        outlier_dist_px=24.0,
        min_valid_points=8,
    ):
        self.num_points = num_points
        self.reseed_interval = reseed_interval
        self.min_visible_ratio = min_visible_ratio
        self.line_smooth_alpha = line_smooth_alpha
        self.occluded_point_weight = occluded_point_weight
        self.outlier_dist_px = outlier_dist_px
        self.min_valid_points = min_valid_points
        self.device = (
            "cuda"
            if torch.cuda.is_available()
            else "mps"
            if torch.backends.mps.is_available()
            else "cpu"
        )

        self.model = None
        self.available = False
        self.step = 8
        self.window_frames = []
        self.frames_since_update = 0
        self.last_seed_frame_idx = -10**9
        self.render_seq_id = 0
        self.visible_ratio = 0.0
        self.last_result = None
        self.lateral_direction = np.array([1.0, 0.0], dtype=np.float32)
        self.offset_px = 0.0
        self.reference_ts = np.linspace(0.0, 1.0, int(num_points), dtype=np.float32)

        if cotracker_hub is None:
            print("[CoTracker] 本地 CoTracker 包不可用，已禁用区域跟踪")
            return

        try:
            self.model = cotracker_hub.cotracker3_online(pretrained=True).to(self.device)
            self.model.eval()
            self.step = int(getattr(self.model, "step", 8))
            self.available = True
            print(f"[CoTracker] 在线跟踪已启用, device={self.device}, step={self.step}")
        except Exception as e:
            print(f"[CoTracker] 初始化失败，已禁用区域跟踪: {e}")

    def reset(self):
        self.window_frames = []
        self.frames_since_update = 0
        self.visible_ratio = 0.0
        self.last_result = None

    def should_reseed(self, frame_idx):
        if not self.available:
            return False
        if self.last_result is None:
            return True
        if self.visible_ratio < self.min_visible_ratio:
            return True
        return (frame_idx - self.last_seed_frame_idx) >= self.reseed_interval

    def seed(self, frame_bgr, spine_line, lateral_direction_2d, offset_px, frame_idx):
        if not self.available:
            return None

        queries_xy = self._sample_spine_points(*spine_line)
        queries = np.zeros((1, queries_xy.shape[0], 3), dtype=np.float32)
        queries[0, :, 1:] = queries_xy
        video_rgb = self._frames_to_rgb_array([frame_bgr])
        video_chunk = self._frames_to_tensor_from_rgb(video_rgb)

        with torch.no_grad():
            self.model(
                video_chunk,
                is_first_step=True,
                queries=torch.from_numpy(queries).to(self.device),
                add_support_grid=True,
            )

        self.window_frames = [frame_bgr.copy()]
        self.frames_since_update = 0
        self.last_seed_frame_idx = frame_idx
        self.render_seq_id += 1
        norm = float(np.linalg.norm(lateral_direction_2d))
        if norm > 1e-6:
            self.lateral_direction = np.asarray(lateral_direction_2d, dtype=np.float32) / norm
        self.offset_px = float(max(0.0, offset_px))

        visible = np.ones((self.num_points,), dtype=bool)
        self.visible_ratio = 1.0
        self.last_result = self._build_result(queries_xy, visible)
        if self.last_result is not None:
            seed_tracks = torch.from_numpy(queries_xy[None, None].astype(np.float32))
            seed_visibility = torch.ones((1, 1, queries_xy.shape[0]), dtype=torch.bool)
            self.last_result["video_rgb"] = video_rgb
            self.last_result["tracks_seq"] = seed_tracks
            self.last_result["visibility_seq"] = seed_visibility
            self.last_result["query_frame"] = 0
            self.last_result["render_seq_id"] = int(self.render_seq_id)
        return self.last_result

    def update(self, frame_bgr, frame_idx=None):
        if not self.available or self.last_result is None:
            return self.last_result

        self.window_frames.append(frame_bgr.copy())
        self.frames_since_update += 1

        max_keep = self.step * 3
        if len(self.window_frames) > max_keep:
            self.window_frames = self.window_frames[-max_keep:]

        if len(self.window_frames) < self.step * 2:
            return self.last_result
        if self.frames_since_update < self.step:
            return self.last_result

        frame_chunk = self.window_frames[-self.step * 2 :]
        video_rgb = self._frames_to_rgb_array(frame_chunk)
        video_chunk = self._frames_to_tensor_from_rgb(video_rgb)
        with torch.no_grad():
            tracks, visibility = self.model(
                video_chunk,
                is_first_step=False,
                add_support_grid=True,
            )
        self.frames_since_update = 0
        self.render_seq_id += 1

        if tracks is None or visibility is None:
            return self.last_result

        points = tracks[0, -1].detach().cpu().numpy().reshape(-1, 2)
        visible = visibility[0, -1].detach().cpu().numpy().astype(bool).reshape(-1)
        self.visible_ratio = float(visible.mean()) if len(visible) > 0 else 0.0

        result = self._build_result(points, visible)
        if result is not None:
            result["visible_ratio"] = self.visible_ratio
            result["video_rgb"] = video_rgb
            result["tracks_seq"] = tracks.detach().cpu()
            result["visibility_seq"] = visibility.detach().cpu()
            result["query_frame"] = 0
            result["render_seq_id"] = int(self.render_seq_id if frame_idx is None else frame_idx)
            self.last_result = result
        return self.last_result

    @staticmethod
    def _frames_to_rgb_array(frames_bgr):
        return np.stack([cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) for frame in frames_bgr]).astype(np.uint8)

    def _frames_to_tensor_from_rgb(self, frames_rgb):
        return (
            torch.tensor(frames_rgb, device=self.device)
            .float()
            .permute(0, 3, 1, 2)[None]
        )

    def _sample_spine_points(self, start_pt, end_pt):
        start = np.asarray(start_pt, dtype=np.float32)
        end = np.asarray(end_pt, dtype=np.float32)
        pts = []
        for t in self.reference_ts:
            pts.append(start * (1.0 - t) + end * t)
        return np.asarray(pts, dtype=np.float32)

    @staticmethod
    def _weighted_pca_direction(points_xy, weights=None):
        pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
        if len(pts) < 2:
            return None, None
        if weights is None:
            weights = np.ones((len(pts),), dtype=np.float32)
        weights = np.asarray(weights, dtype=np.float32).reshape(-1)
        wsum = float(np.sum(weights))
        if wsum <= 1e-6:
            return None, None

        mean = np.average(pts, axis=0, weights=weights)
        centered = pts - mean[None, :]
        cov = np.zeros((2, 2), dtype=np.float32)
        for p, w in zip(centered, weights):
            cov += float(w) * np.outer(p, p)
        eigvals, eigvecs = np.linalg.eigh(cov)
        direction = eigvecs[:, int(np.argmax(eigvals))].astype(np.float32)
        if float(np.linalg.norm(direction)) < 1e-6:
            return None, None
        return mean.astype(np.float32), direction.astype(np.float32)

    def _reconstruct_spine_from_visible(self, samples_xy, visible_mask, prev_spine=None):
        pts = np.asarray(samples_xy, dtype=np.float32).reshape(-1, 2)
        vis = np.asarray(visible_mask, dtype=bool).reshape(-1)
        if len(pts) != len(self.reference_ts):
            ref_ts = np.linspace(0.0, 1.0, len(pts), dtype=np.float32)
        else:
            ref_ts = self.reference_ts

        reliable_mask = vis.copy()
        if prev_spine is not None and len(prev_spine) == len(pts):
            deltas = np.linalg.norm(pts - prev_spine, axis=1)
            reliable_mask &= deltas <= float(self.outlier_dist_px)

        reliable_count = int(reliable_mask.sum())
        if reliable_count < max(3, int(self.min_valid_points * 0.45)):
            if prev_spine is None:
                return None, reliable_count, 0.0
            alpha = min(0.12, float(self.line_smooth_alpha))
            blended = alpha * pts + (1.0 - alpha) * prev_spine
            return blended.astype(np.float32), reliable_count, 0.0

        fit_pts = pts[reliable_mask]
        fit_ts = ref_ts[reliable_mask]
        mean, direction = self._weighted_pca_direction(fit_pts)
        if mean is None or direction is None:
            if prev_spine is not None:
                return prev_spine.astype(np.float32), reliable_count, 0.0
            return None, reliable_count, 0.0

        if prev_spine is not None:
            prev_dir = np.asarray(prev_spine[-1], dtype=np.float32) - np.asarray(prev_spine[0], dtype=np.float32)
            if float(np.dot(direction, prev_dir)) < 0.0:
                direction = -direction
        else:
            seed_dir = fit_pts[-1] - fit_pts[0]
            if float(np.dot(direction, seed_dir)) < 0.0:
                direction = -direction

        proj_fit = np.dot(fit_pts - mean[None, :], direction)
        span_ratio = float(np.max(fit_ts) - np.min(fit_ts)) if len(fit_ts) > 0 else 0.0

        if len(fit_ts) >= 2 and span_ratio > 0.10:
            A = np.stack([fit_ts, np.ones_like(fit_ts)], axis=1)
            try:
                slope, intercept = np.linalg.lstsq(A, proj_fit, rcond=None)[0]
            except Exception:
                slope, intercept = 0.0, float(np.mean(proj_fit))
        else:
            slope = 0.0
            intercept = float(np.mean(proj_fit)) if len(proj_fit) > 0 else 0.0

        if prev_spine is not None and len(prev_spine) == len(pts):
            prev_proj = np.dot(prev_spine - mean[None, :], direction)
            prev_slope = float(prev_proj[-1] - prev_proj[0])
            prev_intercept = float(prev_proj[0])
            coverage = max(0.0, min(1.0, span_ratio / 0.65))
            slope = float(coverage * slope + (1.0 - coverage) * prev_slope)
            intercept = float(coverage * intercept + (1.0 - coverage) * prev_intercept)

        proj_all = slope * ref_ts + intercept
        fit = mean[None, :] + proj_all[:, None] * direction[None, :]

        if prev_spine is not None and len(prev_spine) == len(pts):
            alpha = float(self.line_smooth_alpha) * max(0.20, min(1.0, span_ratio / 0.55))
            alpha = max(0.10, min(0.50, alpha))
            fit = alpha * fit + (1.0 - alpha) * prev_spine

        return fit.astype(np.float32), reliable_count, span_ratio

    def _fit_spine_points(self, points, visible):
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        vis = np.asarray(visible, dtype=bool).reshape(-1)
        if len(pts) == 0:
            return None, 0, 0.0

        prev_spine = None
        if self.last_result is not None and self.last_result.get("spine_points") is not None:
            prev_spine = np.asarray(self.last_result["spine_points"], dtype=np.float32).reshape(-1, 2)
            if len(prev_spine) != len(pts):
                prev_spine = None

        fit, reliable_count, span_ratio = self._reconstruct_spine_from_visible(pts, vis, prev_spine=prev_spine)
        if fit is None:
            return prev_spine, reliable_count, span_ratio
        return fit.astype(np.float32), reliable_count, span_ratio

    def _offset_line_from_spine_points(self, spine_points, side_sign):
        if self.offset_px <= 0.0 or spine_points is None or len(spine_points) < 2:
            return None
        offset_vec = np.asarray(self.lateral_direction, dtype=np.float32) * float(self.offset_px) * float(side_sign)
        start = np.asarray(spine_points[0], dtype=np.float32) + offset_vec
        end = np.asarray(spine_points[-1], dtype=np.float32) + offset_vec
        return tuple(start), tuple(end)

    def _build_result(self, points, visible):
        spine_points, valid_count, span_ratio = self._fit_spine_points(points, visible)
        if spine_points is None or len(spine_points) < 2:
            return None

        spine_line = (tuple(spine_points[0]), tuple(spine_points[-1]))
        left_line = self._offset_line_from_spine_points(spine_points, -1.0)
        right_line = self._offset_line_from_spine_points(spine_points, 1.0)

        return {
            "spine_line": spine_line,
            "spine_points": spine_points.reshape(-1, 2),
            "meridian_lines": (left_line, right_line) if left_line is not None and right_line is not None else None,
            "grid_points": np.asarray(points, dtype=np.float32).reshape(-1, 2),
            "grid_visible": np.asarray(visible, dtype=bool).reshape(-1),
            "visible_ratio": float(np.asarray(visible, dtype=np.float32).mean()) if len(visible) > 0 else 0.0,
            "valid_spine_points": int(valid_count),
            "fit_span_ratio": float(span_ratio),
        }


class LiveTrajectoryBuffer:
    """主线程写入实时轨迹，机械臂线程无锁快照读取。"""

    def __init__(
        self,
        stale_timeout_s=0.8,
        default_active_side=ACTIVE_SIDE_NAME,
        side_switch_threshold_mm=8.0,
        side_switch_confirm_frames=3,
    ):
        self.stale_timeout_s = stale_timeout_s
        self._lock = threading.Lock()
        self._seq = 0
        self._latest = None
        self._active_side = str(default_active_side)
        self._active_side_conf_mm = 0.0
        self._active_depths_mm = {"left": None, "right": None}
        self._pending_side = None
        self._pending_side_frames = 0
        self.side_switch_threshold_mm = float(side_switch_threshold_mm)
        self.side_switch_confirm_frames = int(side_switch_confirm_frames)

    @staticmethod
    def _median_depth_mm(points_cam):
        if not points_cam:
            return None
        zs_mm = [float(p[2]) * 1000.0 for p in points_cam if len(p) >= 3]
        if not zs_mm:
            return None
        return float(np.median(zs_mm))

    def _update_active_side(self, left_cam, right_cam):
        left_depth_mm = self._median_depth_mm(left_cam)
        right_depth_mm = self._median_depth_mm(right_cam)
        self._active_depths_mm = {"left": left_depth_mm, "right": right_depth_mm}

        if left_depth_mm is None or right_depth_mm is None:
            return self._active_side, self._active_side_conf_mm

        depth_delta_mm = abs(left_depth_mm - right_depth_mm)
        candidate = "left" if left_depth_mm <= right_depth_mm else "right"
        self._active_side_conf_mm = depth_delta_mm

        if depth_delta_mm < self.side_switch_threshold_mm:
            self._pending_side = None
            self._pending_side_frames = 0
            return self._active_side, self._active_side_conf_mm

        if candidate == self._active_side:
            self._pending_side = None
            self._pending_side_frames = 0
            return self._active_side, self._active_side_conf_mm

        if candidate != self._pending_side:
            self._pending_side = candidate
            self._pending_side_frames = 1
            return self._active_side, self._active_side_conf_mm

        self._pending_side_frames += 1
        if self._pending_side_frames >= self.side_switch_confirm_frames:
            self._active_side = candidate
            self._pending_side = None
            self._pending_side_frames = 0
        return self._active_side, self._active_side_conf_mm

    def update_from_lines(self, detector, meridian_lines, depth_frame):
        if meridian_lines is None or detector.camera_to_robot is None:
            return False

        left_line, right_line = meridian_lines
        left_px = detector.sample_line_pixels(left_line[0], left_line[1])
        right_px = detector.sample_line_pixels(right_line[0], right_line[1])
        left_cam = detector.pixels_to_points3d(left_px, depth_frame)
        right_cam = detector.pixels_to_points3d(right_px, depth_frame)
        if len(left_cam) < 4 or len(right_cam) < 4 or len(left_cam) != len(right_cam):
            return False

        left_robot = detector.transform_points_to_robot(left_cam)
        right_robot = detector.transform_points_to_robot(right_cam)
        if len(left_robot) < 4 or len(right_robot) < 4 or len(left_robot) != len(right_robot):
            return False

        left_mm, _ = _to_mm_points(left_robot)
        right_mm, _ = _to_mm_points(right_robot)
        left_mm = _smooth_surface_trajectory(left_mm)
        right_mm = _smooth_surface_trajectory(right_mm)
        center_mm = [
            [
                (float(lp[0]) + float(rp[0])) * 0.5,
                (float(lp[1]) + float(rp[1])) * 0.5,
                (float(lp[2]) + float(rp[2])) * 0.5,
            ]
            for lp, rp in zip(left_mm, right_mm)
        ]
        center_mm = _smooth_surface_trajectory(center_mm)
        active_side, active_side_conf_mm = self._update_active_side(left_cam, right_cam)
        camera_origin_mm = _camera_origin_mm_from_matrix(detector.camera_to_robot)

        with self._lock:
            self._seq += 1
            self._latest = {
                "seq": self._seq,
                "timestamp": time.time(),
                "left": [list(map(float, p)) for p in left_mm],
                "right": [list(map(float, p)) for p in right_mm],
                "center": [list(map(float, p)) for p in center_mm],
                "active_side": str(active_side),
                "active_side_conf_mm": float(active_side_conf_mm),
                "left_depth_mm": None if self._active_depths_mm["left"] is None else float(self._active_depths_mm["left"]),
                "right_depth_mm": None if self._active_depths_mm["right"] is None else float(self._active_depths_mm["right"]),
                "camera_origin_mm": None if camera_origin_mm is None else [float(v) for v in camera_origin_mm],
            }
        return True

    def get_latest(self, max_age_s=None):
        if max_age_s is None:
            max_age_s = self.stale_timeout_s
        with self._lock:
            data = self._latest
            if data is None:
                return None
            if time.time() - data["timestamp"] > max_age_s:
                return None
            return {
                "seq": data["seq"],
                "timestamp": data["timestamp"],
                "left": [p[:] for p in data["left"]],
                "right": [p[:] for p in data["right"]],
                "center": [p[:] for p in data["center"]],
                "active_side": str(data.get("active_side", self._active_side)),
                "active_side_conf_mm": float(data.get("active_side_conf_mm", self._active_side_conf_mm)),
                "left_depth_mm": data.get("left_depth_mm"),
                "right_depth_mm": data.get("right_depth_mm"),
                "camera_origin_mm": None if data.get("camera_origin_mm") is None else [float(v) for v in data["camera_origin_mm"]],
            }

    def peek_state(self):
        with self._lock:
            if self._latest is None:
                return {
                    "active_side": self._active_side,
                    "active_side_conf_mm": float(self._active_side_conf_mm),
                    "left_depth_mm": self._active_depths_mm["left"],
                    "right_depth_mm": self._active_depths_mm["right"],
                    "seq": 0,
                    "timestamp": 0.0,
                }
            return {
                "active_side": str(self._latest.get("active_side", self._active_side)),
                "active_side_conf_mm": float(self._latest.get("active_side_conf_mm", self._active_side_conf_mm)),
                "left_depth_mm": self._latest.get("left_depth_mm"),
                "right_depth_mm": self._latest.get("right_depth_mm"),
                "seq": int(self._latest.get("seq", 0)),
                "timestamp": float(self._latest.get("timestamp", 0.0)),
            }


def _normalize_vec(vec, eps=1e-6):
    arr = np.asarray(vec, dtype=np.float64)
    norm = float(np.linalg.norm(arr))
    if norm <= eps:
        return None
    return arr / norm


def _camera_origin_mm_from_matrix(matrix4):
    if matrix4 is None:
        return None
    try:
        origin = np.asarray(matrix4, dtype=np.float64) @ np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    except Exception:
        return None
    if origin.shape[0] < 3:
        return None
    return [float(origin[0] * 1000.0), float(origin[1] * 1000.0), float(origin[2] * 1000.0)]


def _smooth_surface_trajectory(points_mm, window=5):
    """
    对 3D 体表轨迹做轻量平滑，保留真实轮廓起伏，不再把整条路径拉成首尾直线。
    """
    if len(points_mm) < 3:
        return [list(map(float, p)) for p in points_mm]

    arr = np.asarray(points_mm, dtype=np.float64)
    out = arr.copy()
    radius = max(1, int(window) // 2)
    for i in range(len(arr)):
        lo = max(0, i - radius)
        hi = min(len(arr), i + radius + 1)
        seg = arr[lo:hi]
        if len(seg) == 0:
            continue
        out[i, 0] = float(np.mean(seg[:, 0]))
        out[i, 1] = float(np.mean(seg[:, 1]))
        out[i, 2] = float(np.median(seg[:, 2]))

    out[0] = arr[0]
    out[-1] = arr[-1]
    return out.tolist()


def _expand_meridian_lines_from_spine(spine_line, inner_lines, scale=2.0):
    """
    以脊柱为中心，将当前内侧膀胱经线按比例放大到更外侧位置。
    scale=2 表示外侧膀胱经距离为当前偏移量的两倍。
    """
    if (
        spine_line is None
        or inner_lines is None
        or len(inner_lines) != 2
        or inner_lines[0] is None
        or inner_lines[1] is None
    ):
        return None

    spine_neck = np.asarray(spine_line[0], dtype=np.float64)
    spine_tail = np.asarray(spine_line[1], dtype=np.float64)
    outer_lines = []

    for inner_line in inner_lines:
        neck_inner = np.asarray(inner_line[0], dtype=np.float64)
        tail_inner = np.asarray(inner_line[1], dtype=np.float64)
        neck_outer = spine_neck + (neck_inner - spine_neck) * float(scale)
        tail_outer = spine_tail + (tail_inner - spine_tail) * float(scale)
        outer_lines.append(
            (
                (float(neck_outer[0]), float(neck_outer[1])),
                (float(tail_outer[0]), float(tail_outer[1])),
            )
        )

    return tuple(outer_lines)


def _build_spine_seed_from_torso(kpts, smoother, finger_width_mm):
    """
    用 shoulders/hips 的整体躯干几何来初始化脊柱，而不是直接拿肩中点到髋中点的短线。
    如果髋点退化到肩部附近，会直接判无效，避免把错误短线交给 CoTracker 稳定跟踪。
    """
    torso_idx = (5, 6, 11, 12)
    if any(float(kpts[idx][2]) < 0.35 for idx in torso_idx):
        return None

    ls = np.asarray(smoother.smooth("ls", kpts[5][0], kpts[5][1], round_result=False), dtype=np.float64)
    rs = np.asarray(smoother.smooth("rs", kpts[6][0], kpts[6][1], round_result=False), dtype=np.float64)
    lh = np.asarray(smoother.smooth("lh", kpts[11][0], kpts[11][1], round_result=False), dtype=np.float64)
    rh = np.asarray(smoother.smooth("rh", kpts[12][0], kpts[12][1], round_result=False), dtype=np.float64)

    shoulder_mid = (ls + rs) * 0.5
    hip_mid = (lh + rh) * 0.5
    shoulder_vec = rs - ls
    hip_vec = rh - lh
    shoulder_px = float(np.linalg.norm(shoulder_vec))
    hip_px = float(np.linalg.norm(hip_vec))
    torso_vec = hip_mid - shoulder_mid
    torso_len_px = float(np.linalg.norm(torso_vec))
    width_px = max(20.0, float(np.median([max(shoulder_px, 1.0), max(hip_px, 1.0)])))

    if torso_len_px < width_px * 0.85:
        return None

    axis_hint = _normalize_vec(torso_vec)
    if axis_hint is None:
        return None

    torso_pts = np.stack([ls, rs, lh, rh], axis=0)
    torso_center = np.mean(torso_pts, axis=0)
    centered = torso_pts - torso_center
    try:
        _, eigvecs = np.linalg.eigh(centered.T @ centered)
        pca_axis = np.asarray(eigvecs[:, -1], dtype=np.float64)
    except Exception:
        pca_axis = axis_hint
    if float(np.dot(pca_axis, axis_hint)) < 0.0:
        pca_axis = -pca_axis
    axis_2d = _normalize_vec(axis_hint * 0.6 + pca_axis * 0.4)
    if axis_2d is None:
        axis_2d = axis_hint

    lateral_guess = (shoulder_vec + hip_vec) * 0.5
    lateral_guess = lateral_guess - axis_2d * float(np.dot(lateral_guess, axis_2d))
    lateral_2d = _normalize_vec(lateral_guess)
    if lateral_2d is None:
        lateral_2d = np.array([-axis_2d[1], axis_2d[0]], dtype=np.float64)

    pixels_per_mm = shoulder_px / 360.0 if shoulder_px > 1e-6 else 1.0
    body_offset_px = max(12.0, float(finger_width_mm) * pixels_per_mm)

    torso_center = np.mean(np.stack([ls, rs, lh, rh], axis=0), axis=0)
    proj_shoulder = float(np.dot(shoulder_mid - torso_center, axis_2d))
    proj_hip = float(np.dot(hip_mid - torso_center, axis_2d))
    if proj_hip < proj_shoulder:
        axis_2d = -axis_2d
        proj_shoulder, proj_hip = -proj_shoulder, -proj_hip
        lateral_2d = -lateral_2d

    torso_span = max(1.0, proj_hip - proj_shoulder)
    neck_extend_px = min(max(width_px * 0.22, body_offset_px * 0.45), torso_span * 0.18)
    tail_extend_px = min(max(width_px * 0.10, body_offset_px * 0.25), torso_span * 0.10)
    neck_raw = torso_center + axis_2d * (proj_shoulder - neck_extend_px)
    tail_raw = torso_center + axis_2d * (proj_hip + tail_extend_px)

    neck_u, neck_v = smoother.smooth("neck", neck_raw[0], neck_raw[1], round_result=False)
    tail_u, tail_v = smoother.smooth("tail", tail_raw[0], tail_raw[1], round_result=False)

    spine_line = ((float(neck_u), float(neck_v)), (float(tail_u), float(tail_v)))
    return {
        "spine_line": spine_line,
        "lateral_direction_2d": lateral_2d,
        "body_offset_px": float(body_offset_px),
        "torso_len_px": float(torso_len_px),
        "shoulder_px": float(shoulder_px),
    }


def _build_cotracker_visualizer():
    if CoTrackerVisualizer is None:
        return None
    try:
        return CoTrackerVisualizer(
            save_dir="./saved_videos",
            pad_value=0,
            linewidth=2,
            mode="optical_flow",
            show_first_frame=0,
            tracks_leave_trace=12,
        )
    except Exception as e:
        print(f"[CoTracker] 官方 Visualizer 初始化失败: {e}")
        return None


def _render_official_cotracker_overlay(tracker_result, visualizer):
    if visualizer is None or tracker_result is None:
        return None
    video_rgb = tracker_result.get("video_rgb")
    tracks_seq = tracker_result.get("tracks_seq")
    visibility_seq = tracker_result.get("visibility_seq")
    query_frame = int(tracker_result.get("query_frame", 0))
    if video_rgb is None or tracks_seq is None or visibility_seq is None:
        return None

    try:
        video_t = torch.from_numpy(np.asarray(video_rgb, dtype=np.uint8)).permute(0, 3, 1, 2)[None].float()
        tracks_t = tracks_seq if torch.is_tensor(tracks_seq) else torch.as_tensor(tracks_seq)
        visibility_t = visibility_seq if torch.is_tensor(visibility_seq) else torch.as_tensor(visibility_seq)
        vis_video = visualizer.visualize(
            video_t,
            tracks_t,
            visibility_t,
            query_frame=query_frame,
            save_video=False,
            opacity=0.95,
        )
        vis_frame = vis_video[0, -1].permute(1, 2, 0).detach().cpu().numpy().astype(np.uint8)
        return cv2.cvtColor(vis_frame, cv2.COLOR_RGB2BGR)
    except Exception as e:
        print(f"[CoTracker] 官方 Visualizer 渲染失败: {e}")
        return None


def _densify_triplet_paths(left_pts, center_pts, right_pts, max_step_mm):
    if not left_pts or not center_pts or not right_pts:
        return None
    if not (len(left_pts) == len(center_pts) == len(right_pts)):
        return None

    out_left = []
    out_center = []
    out_right = []
    for i in range(len(left_pts) - 1):
        l0 = np.asarray(left_pts[i], dtype=np.float64)
        l1 = np.asarray(left_pts[i + 1], dtype=np.float64)
        c0 = np.asarray(center_pts[i], dtype=np.float64)
        c1 = np.asarray(center_pts[i + 1], dtype=np.float64)
        r0 = np.asarray(right_pts[i], dtype=np.float64)
        r1 = np.asarray(right_pts[i + 1], dtype=np.float64)

        out_left.append(l0.tolist())
        out_center.append(c0.tolist())
        out_right.append(r0.tolist())

        d_left = float(np.linalg.norm((l1 - l0)[:2]))
        d_center = float(np.linalg.norm((c1 - c0)[:2]))
        d_right = float(np.linalg.norm((r1 - r0)[:2]))
        n_seg = max(1, int(math.ceil(max(d_left, d_center, d_right) / max_step_mm)))
        for s in range(1, n_seg):
            t = s / n_seg
            out_left.append((l0 * (1.0 - t) + l1 * t).tolist())
            out_center.append((c0 * (1.0 - t) + c1 * t).tolist())
            out_right.append((r0 * (1.0 - t) + r1 * t).tolist())

    out_left.append(list(map(float, left_pts[-1])))
    out_center.append(list(map(float, center_pts[-1])))
    out_right.append(list(map(float, right_pts[-1])))
    return {"left": out_left, "center": out_center, "right": out_right}


def _rpy_from_tool_z_vector(tool_z_unit, reference_rz_deg, reference_rx_deg):
    tool_z = _normalize_vec(tool_z_unit)
    if tool_z is None:
        return reference_rx_deg, 0.0, reference_rz_deg

    rz = math.radians(float(reference_rz_deg))
    cz = math.cos(rz)
    sz = math.sin(rz)
    ux = cz * tool_z[0] + sz * tool_z[1]
    uy = -sz * tool_z[0] + cz * tool_z[1]
    uz = tool_z[2]

    sx = float(max(-1.0, min(1.0, -uy)))
    cx_mag = math.sqrt(max(1e-9, 1.0 - sx * sx))
    base_cx = math.cos(math.radians(float(reference_rx_deg)))
    cx = -cx_mag if base_cx < 0.0 else cx_mag
    rx = math.atan2(sx, cx)

    if abs(cx) < 1e-6:
        ry = 0.0
    else:
        sy = ux / cx
        cy = uz / cx
        scale = math.sqrt(max(1e-9, sy * sy + cy * cy))
        sy /= scale
        cy /= scale
        ry = math.atan2(sy, cy)

    return math.degrees(rx), math.degrees(ry), float(reference_rz_deg)


def _compute_local_pose(
    surface_path,
    center_path,
    idx,
    base_tool_z,
    reference_rz_deg,
    reference_rx_deg,
    camera_origin_mm=None,
):
    n = len(surface_path)
    idx0 = max(0, idx - 1)
    idx1 = min(n - 1, idx + 1)
    p = np.asarray(surface_path[idx], dtype=np.float64)
    c = np.asarray(center_path[idx], dtype=np.float64)
    p0 = np.asarray(surface_path[idx0], dtype=np.float64)
    p1 = np.asarray(surface_path[idx1], dtype=np.float64)

    tangent = _normalize_vec(p1 - p0)
    lateral = _normalize_vec(p - c)
    if lateral is None:
        lateral = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if tangent is None:
        tangent = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    outward = _normalize_vec(np.cross(tangent, lateral))
    camera_vec = None
    if camera_origin_mm is not None:
        camera_vec = _normalize_vec(np.asarray(camera_origin_mm, dtype=np.float64) - p)
    if outward is None:
        outward = camera_vec if camera_vec is not None else _normalize_vec(-np.asarray(base_tool_z, dtype=np.float64))
    if outward is None:
        outward = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    if camera_vec is not None:
        if float(np.dot(outward, camera_vec)) < 0.0:
            outward = -outward
        blended = _normalize_vec(outward * 0.35 + camera_vec * 0.65)
        if blended is not None:
            outward = blended

    tool_axis = -outward
    if float(np.dot(tool_axis, base_tool_z)) < -0.35:
        tool_axis = -tool_axis

    rx_deg, ry_deg, rz_deg = _rpy_from_tool_z_vector(tool_axis, reference_rz_deg, reference_rx_deg)
    tool_z = np.asarray(_tool_z_unit_from_rpy(rx_deg, ry_deg, rz_deg), dtype=np.float64)
    if float(np.dot(tool_z, tool_axis)) < 0.0:
        tool_z = -tool_z
    return p, tool_z, (rx_deg, ry_deg, rz_deg)


def _pose_from_surface_offset(surface_point, tool_z_unit, orientation_deg, offset_mm):
    pos = np.asarray(surface_point, dtype=np.float64) + np.asarray(tool_z_unit, dtype=np.float64) * float(offset_mm)
    rx_deg, ry_deg, rz_deg = orientation_deg
    return [float(pos[0]), float(pos[1]), float(pos[2]), float(rx_deg), float(ry_deg), float(rz_deg)]


# ===================== Motion State =====================

class MotionState:
    """Single-writer (worker thread), multi-reader (main thread)."""

    def __init__(self):
        self.status = "idle"  # idle | running | done | error
        self.progress = 0     # current point index
        self.total = 0        # total points per pass
        self.pass_cur = 0
        self.pass_total = 0
        self.error_msg = ""
        self.stop_event = threading.Event()
        self.force_z = 0.0         # 实时 Fz 读数 (N)
        self.force_mode = "ft_control"  # "ft_control" | "impedance"
        self.target_force = 20.0    # 默认先用低档目标力调试，再逐级升到 49N
        self.follow_side = "left"
        self.live_seq = 0
        self.live_mode_name = ""


def _live_follow_worker(
    state: MotionState,
    live_buffer: LiveTrajectoryBuffer,
    tracker=None,
    static_snapshot=None,
    follow_live_path: bool = False,
    robot_ip: str = "192.168.58.2",
    speed: int = 8,
    tool: int = 0,
    user: int = 0,
    rx: float = -178.190,
    ry: float = 1.724,
    rz: float = -1.187,
    hover_mm: float = 15.0,
    approach_height_mm: float = 35.0,
    occlusion_shift_mm: float = 0.0,
    test_mode_name: str = "live-follow",
    force_mode: str = "ft_control",
    target_force_n: float = 5.0,
    use_force_control: bool = True,
    config: ForceControlConfig = None,
):
    """按单侧膀胱经执行两阶段按摩：先点压分筋，再回头恒力顺筋。"""
    if config is None:
        config = ForceControlConfig()
    if use_force_control and config.enable_collision_guard:
        print("[LiveFollow] 本 demo 默认关闭 FT_Guard，避免按压过程中被碰撞守护打断")
        config.enable_collision_guard = False
    config.target_force_z = target_force_n
    if use_force_control:
        soft_margin = 10.0 if config.enable_collision_guard else 22.0
        config.software_force_limit = max(
            float(config.software_force_limit), abs(float(target_force_n)) + soft_margin
        )
        if config.enable_collision_guard:
            config.guard_force_limit = max(
                float(config.guard_force_limit), abs(float(target_force_n)) + 20.0
            )

    SAFE_Z_MM = 300.0
    TRANSIT_SPEED = 25
    PATH_SPEED = max(2, min(4, int(speed)))
    PRESS_TOL_N = 2.0
    SPLIT_HALF_STROKE_MM = 10.0
    SPLIT_SETTLE_S = 0.18
    SPLIT_POINT_COUNT = 6
    GLIDE_POINT_COUNT = 12
    TRACK_MIN_VISIBLE_RATIO = 0.45
    TRACK_MIN_VALID_POINTS = 10
    FORCE_KP = 0.10
    FORCE_KI = 0.008
    FORCE_IMAX = 40.0
    FORCE_MAX_STEP = 0.5
    PRESS_OFFSET_LIMIT_MM = 18.0
    CONTACT_INSIDE_LIMIT_MM = 6.0
    CONTACT_LATERAL_ERR_LIMIT_MM = 18.0

    robot = None

    try:
        state.status = "running"
        state.error_msg = ""
        state.live_mode_name = f"{test_mode_name}-single-side"
        state.pass_total = 2
        state.pass_cur = 0
        state.progress = 0
        state.total = 0
        state.follow_side = ACTIVE_SIDE_NAME
        state.live_seq = 0

        robot = Robot.RPC(robot_ip)
        robot.SetSpeed(TRANSIT_SPEED)

        live_rx, live_ry, live_rz = rx, ry, rz
        cur_pose = robot.GetActualTCPPose()
        if isinstance(cur_pose, tuple) and len(cur_pose) == 2 and cur_pose[0] == 0:
            pose_vals = cur_pose[1]
            live_rx, live_ry, live_rz = float(pose_vals[3]), float(pose_vals[4]), float(pose_vals[5])

        base_tool_z = np.asarray(_tool_z_unit_from_rpy(live_rx, live_ry, live_rz), dtype=np.float64)

        def _stopped():
            return state.stop_event.is_set()

        def _prepare_paths(snapshot):
            packed = _densify_triplet_paths(
                snapshot.get("left", []),
                snapshot.get("center", []),
                snapshot.get("right", []),
                max(3.0, float(config.cart_max_xy_step_mm)),
            )
            if packed is None:
                return None
            for side_name in ("left", "right", "center"):
                if not packed.get(side_name) or len(packed[side_name]) < 2:
                    return None
            return packed

        static_packed = _prepare_paths(static_snapshot) if static_snapshot is not None else None
        if static_snapshot is not None and static_packed is None:
            raise RuntimeError("初始膀胱经轨迹无效，无法启动固定轨迹按摩")

        def _get_packed_paths():
            if static_packed is not None and not follow_live_path:
                state.live_seq = int(static_snapshot.get("seq", 0))
                return static_packed, static_snapshot
            snapshot = live_buffer.get_latest()
            if snapshot is None:
                return None, None
            packed = _prepare_paths(snapshot)
            if packed is None:
                return None, None
            state.live_seq = snapshot["seq"]
            return packed, snapshot

        def _resolve_active_side(snapshot=None):
            candidate = ""
            if snapshot is not None:
                candidate = str(snapshot.get("active_side", "")).strip().lower()
            if candidate not in ("left", "right"):
                candidate = str(live_buffer.peek_state().get("active_side", "")).strip().lower()
            return candidate if candidate in ("left", "right") else ACTIVE_SIDE_NAME

        def _current_camera_origin_mm():
            if static_snapshot is not None and not follow_live_path:
                snapshot = static_snapshot
            else:
                snapshot = live_buffer.get_latest(max_age_s=1.5)
            if snapshot is None:
                return None
            camera_origin_mm = snapshot.get("camera_origin_mm")
            if camera_origin_mm is None:
                return None
            return [float(v) for v in camera_origin_mm]

        def _tracking_is_reliable():
            if static_packed is not None and not follow_live_path:
                return True
            result = tracker.last_result if tracker is not None else None
            if result is None:
                return False
            vis_ratio = float(result.get("visible_ratio", 0.0))
            valid_points = int(result.get("valid_spine_points", 0))
            return vis_ratio >= TRACK_MIN_VISIBLE_RATIO and valid_points >= TRACK_MIN_VALID_POINTS

        def _move_pose(target6, max_step_mm=5.0, max_steps=120, blend_ms=-1.0, segmented=False):
            def _segmented_move():
                last_err = None
                for _ in range(max_steps):
                    ret = robot.GetActualTCPPose()
                    if not (isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0):
                        return False
                    ap = ret[1]
                    dx = float(target6[0]) - float(ap[0])
                    dy = float(target6[1]) - float(ap[1])
                    dz = float(target6[2]) - float(ap[2])
                    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if dist < 0.5:
                        last_err = robot.MoveCart(desc_pos=list(target6), tool=tool, user=user, blendT=blend_ms)
                        return last_err == 0
                    scale = min(1.0, max_step_mm / max(dist, 1e-6))
                    wp = [
                        float(ap[0]) + dx * scale,
                        float(ap[1]) + dy * scale,
                        float(ap[2]) + dz * scale,
                        float(target6[3]),
                        float(target6[4]),
                        float(target6[5]),
                    ]
                    last_err = robot.MoveCart(desc_pos=wp, tool=tool, user=user, blendT=blend_ms)
                    if last_err != 0:
                        return False
                return False

            if segmented:
                return _segmented_move()

            rtn = robot.MoveCart(desc_pos=list(target6), tool=tool, user=user, blendT=blend_ms)
            if rtn == 0:
                return True
            if rtn in (14, 112):
                return _segmented_move()
            return False

        def _read_tcp_pose():
            ret = robot.GetActualTCPPose()
            if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
                return [float(v) for v in ret[1][:6]]
            return None

        def _point_ratios(count):
            if count <= 1:
                return [0.0]
            return [i / (count - 1) for i in range(count)]

        def _idx_for_ratio(path_len, ratio):
            if path_len <= 1:
                return 0
            return min(path_len - 1, max(0, int(round(float(ratio) * (path_len - 1)))))

        def _get_side_paths(side_name, require_fresh=False, fallback_surface=None, fallback_center=None):
            packed, _ = _get_packed_paths()
            if packed is None:
                if require_fresh:
                    raise RuntimeError("当前没有新鲜的实时视觉轨迹")
                return fallback_surface, fallback_center
            return packed[side_name], packed["center"]

        def _surface_target(surface_path, center_path, idx, lateral_shift_mm=0.0):
            camera_origin_mm = _current_camera_origin_mm()
            surface_point, tool_z_unit, orient = _compute_local_pose(
                surface_path,
                center_path,
                idx,
                base_tool_z,
                live_rz,
                live_rx,
                camera_origin_mm=camera_origin_mm,
            )
            target_surface = np.asarray(surface_point, dtype=np.float64).copy()
            target_surface[1] += occlusion_shift_mm
            if abs(float(lateral_shift_mm)) > 1e-6:
                lateral = _normalize_vec(
                    np.asarray(surface_path[idx], dtype=np.float64) - np.asarray(center_path[idx], dtype=np.float64)
                )
                if lateral is not None:
                    target_surface = target_surface + lateral * float(lateral_shift_mm)
            return target_surface, np.asarray(tool_z_unit, dtype=np.float64), orient

        def _trim_force(target_surface, tool_z_unit, orient, contact_offset_mm, target_n, max_iter=35, tol_n=PRESS_TOL_N):
            ctrl_err_i = 0.0
            offset = float(contact_offset_mm)
            last_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, offset)
            for _ in range(max_iter):
                if _stopped():
                    raise InterruptedError
                fz = get_force_z(robot)
                if fz is not None:
                    state.force_z = fz
                    if abs(abs(fz) - abs(target_n)) <= tol_n and abs(fz) >= abs(target_n) * 0.82:
                        return offset, last_pose, True
                    target_fz = -abs(float(target_n))
                    err = target_fz - float(fz)
                    ctrl_err_i = max(-FORCE_IMAX, min(FORCE_IMAX, ctrl_err_i + err))
                    delta = -(FORCE_KP * err + FORCE_KI * ctrl_err_i)
                    delta = max(-FORCE_MAX_STEP, min(FORCE_MAX_STEP, delta))
                    offset = max(-approach_height_mm, min(PRESS_OFFSET_LIMIT_MM, offset + delta))
                last_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, offset)
                if not _move_pose(last_pose, max_step_mm=1.2, segmented=True):
                    return offset, last_pose, False
                time.sleep(SPLIT_SETTLE_S)
            return offset, last_pose, False

        def _check_force_limit():
            fz = get_force_z(robot)
            if fz is not None:
                state.force_z = fz
                if abs(fz) > config.software_force_limit:
                    robot.StopMotion()
                    raise RuntimeError(f"力超限 |Fz|={abs(fz):.1f}N")

        def _build_force_stages(final_target_n: float):
            target_abs = abs(float(final_target_n))
            if target_abs <= 1e-6:
                return [0.0]
            stages = []
            for step in config.force_ramp_steps:
                step = abs(float(step))
                if 0.0 < step < target_abs - 0.5:
                    stages.append(step)
            stages.append(target_abs)
            out = []
            for step in stages:
                if not out or abs(step - out[-1]) > 1e-6:
                    out.append(step)
            return out

        def _approach_target_force(target_surface, tool_z_unit, orient, contact_offset_mm, final_target_n):
            offset = float(contact_offset_mm)
            last_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, offset)
            ok = False
            for stage_force in _build_force_stages(final_target_n):
                offset, last_pose, ok = _trim_force(
                    target_surface,
                    tool_z_unit,
                    orient,
                    offset,
                    stage_force,
                    max_iter=18 if stage_force < abs(float(final_target_n)) else 28,
                    tol_n=3.5 if stage_force < abs(float(final_target_n)) else PRESS_TOL_N,
                )
                if _stopped():
                    raise InterruptedError
            return offset, last_pose, ok

        def _probe_surface_contact(target_surface, tool_z_unit, orient, hover_offset_mm, phase_name):
            if _stopped():
                raise InterruptedError

            probe_cfg = replace(config)
            probe_cfg.find_surface_max_dis = min(
                float(config.find_surface_max_dis),
                max(8.0, abs(float(hover_offset_mm)) + 8.0),
            )
            probe_cfg.find_surface_force = max(
                2.0,
                min(float(config.find_surface_force), max(2.5, abs(float(target_force_n)) * 0.20)),
            )
            probe_cfg.find_surface_speed = min(float(config.find_surface_speed), 1.5 if _side_lying_demo_enabled() else 2.0)

            if not find_surface(robot, probe_cfg):
                raise RuntimeError(
                    f"{phase_name}首次接触失败：在 {probe_cfg.find_surface_max_dis:.1f}mm 内未探到人体表面"
                )

            actual_pose = _read_tcp_pose()
            if actual_pose is None:
                raise RuntimeError(f"{phase_name}首次接触后无法读取 TCP 位姿")

            axis = _normalize_vec(tool_z_unit)
            if axis is None:
                raise RuntimeError(f"{phase_name}接触法向无效")

            actual_point = np.asarray(actual_pose[:3], dtype=np.float64)
            delta = actual_point - np.asarray(target_surface, dtype=np.float64)
            signed_offset = float(np.dot(delta, axis))
            lateral_error_mm = float(np.linalg.norm(delta - axis * signed_offset))

            if signed_offset > CONTACT_INSIDE_LIMIT_MM:
                robot.StopMotion()
                raise RuntimeError(
                    f"{phase_name}首次接触已超出预计体表 {signed_offset:.1f}mm，已中止"
                )
            if lateral_error_mm > CONTACT_LATERAL_ERR_LIMIT_MM:
                robot.StopMotion()
                raise RuntimeError(
                    f"{phase_name}首次接触偏离轨迹 {lateral_error_mm:.1f}mm，已中止"
                )

            planned_offset = min(0.0, signed_offset)
            contact_pose = [
                float(actual_point[0]),
                float(actual_point[1]),
                float(actual_point[2]),
                float(orient[0]),
                float(orient[1]),
                float(orient[2]),
            ]
            print(
                f"[LiveFollow] {phase_name} 探面接触: offset={signed_offset:+.1f}mm, "
                f"use_offset={planned_offset:+.1f}mm, lateral_err={lateral_error_mm:.1f}mm"
            )
            return planned_offset, contact_pose, True

        def _safe_hover_exit(surface_path, center_path, side_name):
            if surface_path is None or center_path is None or len(surface_path) == 0:
                raise RuntimeError(f"跟踪不稳定，已退出接触 ({side_name})")
            idx = _idx_for_ratio(len(surface_path), 0.5)
            target_surface, tool_z_unit, orient = _surface_target(surface_path, center_path, idx, 0.0)
            hover_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, -float(hover_mm))
            robot.SetSpeed(10 if use_force_control else TRANSIT_SPEED)
            _move_pose_soft(hover_pose, fast_step_mm=4.0, fine_step_mm=1.5, blend_ms=15.0)
            safe_pose = [float(hover_pose[0]), float(hover_pose[1]), SAFE_Z_MM, float(orient[0]), float(orient[1]), float(orient[2])]
            _move_pose(safe_pose)
            raise RuntimeError(f"跟踪不稳定，已退出接触 ({side_name})")

        def _move_pose_soft(target6, fast_step_mm=6.0, fine_step_mm=2.5, blend_ms=20.0):
            if _move_pose(target6, max_step_mm=fast_step_mm, blend_ms=blend_ms):
                return True
            return _move_pose(target6, max_step_mm=fine_step_mm, blend_ms=10.0, segmented=True)

        def _move_linear_soft(target6, blend_r=30.0, fine_step_mm=2.0):
            # 当前 SDK 的 MoveL 参数封装与本项目调用不兼容，实时演示统一走受控 MoveCart。
            return _move_pose_soft(target6, fast_step_mm=4.0, fine_step_mm=fine_step_mm, blend_ms=10.0)

        def _run_point_press_split(side_name, surface_path, center_path):
            state.follow_side = side_name
            ratios = _point_ratios(SPLIT_POINT_COUNT)
            state.total = len(ratios)
            state.progress = 0
            print(f"[LiveFollow] {side_name} 点压分筋: {len(ratios)} points, target={target_force_n:.1f}N")

            surface_path, center_path = _get_side_paths(side_name, require_fresh=True, fallback_surface=surface_path, fallback_center=center_path)
            first_idx = _idx_for_ratio(len(surface_path), ratios[0])
            target_surface, tool_z_unit, orient = _surface_target(surface_path, center_path, first_idx, 0.0)
            hover_offset = -float(hover_mm)
            hover_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, hover_offset)
            safe_pose = [float(hover_pose[0]), float(hover_pose[1]), SAFE_Z_MM, float(orient[0]), float(orient[1]), float(orient[2])]
            if not _move_pose(safe_pose):
                raise RuntimeError(f"MoveCart safe err at idx={first_idx}")
            robot.SetSpeed(10 if use_force_control else TRANSIT_SPEED)
            if not _move_linear_soft(hover_pose, blend_r=20.0, fine_step_mm=1.5):
                raise RuntimeError(f"MoveCart hover err at idx={first_idx}")

            contact_offset = hover_offset
            if use_force_control:
                contact_offset, _, _ = _probe_surface_contact(
                    target_surface, tool_z_unit, orient, hover_offset, f"{side_name} 点压分筋"
                )
                contact_offset, _, ok = _approach_target_force(
                    target_surface, tool_z_unit, orient, contact_offset, target_force_n
                )
                fz_now = get_force_z(robot)
                fz_s = f"{fz_now:.1f}" if fz_now is not None else "?"
                print(f"[LiveFollow] {side_name} 点 1/{len(ratios)} press: fz={fz_s}N {'OK' if ok else 'soft'}")

            for step_idx, ratio in enumerate(ratios, start=1):
                if _stopped():
                    raise InterruptedError
                state.progress = step_idx
                surface_path, center_path = _get_side_paths(side_name, fallback_surface=surface_path, fallback_center=center_path)
                if not _tracking_is_reliable():
                    _safe_hover_exit(surface_path, center_path, side_name)
                idx = _idx_for_ratio(len(surface_path), ratio)
                target_surface, tool_z_unit, orient = _surface_target(surface_path, center_path, idx, 0.0)
                center_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, contact_offset)
                if not _move_linear_soft(center_pose, blend_r=35.0, fine_step_mm=2.0):
                    raise RuntimeError(f"MoveCart center err at idx={idx}")
                if use_force_control:
                    contact_offset, _, ok = _trim_force(target_surface, tool_z_unit, orient, contact_offset, target_force_n, max_iter=12, tol_n=2.5)
                    fz_now = get_force_z(robot)
                    fz_s = f"{fz_now:.1f}" if fz_now is not None else "?"
                    print(f"[LiveFollow] {side_name} 点 {step_idx}/{len(ratios)} press: fz={fz_s}N {'OK' if ok else 'soft'}")

                for lateral_shift in (SPLIT_HALF_STROKE_MM, -SPLIT_HALF_STROKE_MM, 0.0):
                    if _stopped():
                        raise InterruptedError
                    surface_path, center_path = _get_side_paths(side_name, fallback_surface=surface_path, fallback_center=center_path)
                    if not _tracking_is_reliable():
                        _safe_hover_exit(surface_path, center_path, side_name)
                    idx = _idx_for_ratio(len(surface_path), ratio)
                    target_surface_shift, tool_z_shift, orient_shift = _surface_target(surface_path, center_path, idx, lateral_shift)
                    pose = _pose_from_surface_offset(target_surface_shift, tool_z_shift, orient_shift, contact_offset)
                    if not _move_linear_soft(pose, blend_r=25.0, fine_step_mm=2.0):
                        raise RuntimeError(f"MoveCart split err at idx={idx}")
                    if use_force_control:
                        contact_offset, _, _ = _trim_force(
                            target_surface_shift, tool_z_shift, orient_shift, contact_offset, target_force_n, max_iter=4, tol_n=4.0
                        )
                    time.sleep(SPLIT_SETTLE_S)
                    _check_force_limit()

            robot.SetSpeed(TRANSIT_SPEED)
            end_hover_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, hover_offset)
            if not _move_linear_soft(end_hover_pose, blend_r=25.0, fine_step_mm=2.0):
                raise RuntimeError(f"MoveCart retract err ({side_name})")
            end_safe_pose = [float(end_hover_pose[0]), float(end_hover_pose[1]), SAFE_Z_MM, float(orient[0]), float(orient[1]), float(orient[2])]
            if not _move_pose(end_safe_pose):
                raise RuntimeError(f"MoveCart lift err ({side_name})")

        def _run_glide(side_name, surface_path, center_path):
            state.follow_side = side_name
            ratios = _point_ratios(GLIDE_POINT_COUNT)
            state.total = len(ratios)
            state.progress = 0
            print(f"[LiveFollow] {side_name} 顺筋: {len(ratios)} points, target={target_force_n:.1f}N")

            surface_path, center_path = _get_side_paths(side_name, require_fresh=True, fallback_surface=surface_path, fallback_center=center_path)
            start_idx = _idx_for_ratio(len(surface_path), ratios[0])
            target_surface, tool_z_unit, orient = _surface_target(surface_path, center_path, start_idx, 0.0)
            hover_offset = -float(hover_mm)
            hover_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, hover_offset)
            safe_pose = [float(hover_pose[0]), float(hover_pose[1]), SAFE_Z_MM, float(orient[0]), float(orient[1]), float(orient[2])]
            if not _move_pose(safe_pose):
                raise RuntimeError(f"MoveCart safe start err ({side_name})")
            robot.SetSpeed(10 if use_force_control else TRANSIT_SPEED)
            if not _move_linear_soft(hover_pose, blend_r=20.0, fine_step_mm=1.5):
                raise RuntimeError(f"MoveCart hover start err ({side_name})")

            contact_offset = hover_offset
            if use_force_control:
                contact_offset, _, _ = _probe_surface_contact(
                    target_surface, tool_z_unit, orient, hover_offset, f"{side_name} 顺筋"
                )
                contact_offset, _, ok = _approach_target_force(
                    target_surface, tool_z_unit, orient, contact_offset, target_force_n
                )
                fz_now = get_force_z(robot)
                fz_s = f"{fz_now:.1f}" if fz_now is not None else "?"
                print(f"[LiveFollow] {side_name} 顺筋起点: fz={fz_s}N {'OK' if ok else 'soft'}")

            robot.SetSpeed(PATH_SPEED)
            try:
                for step_idx, ratio in enumerate(ratios, start=1):
                    if _stopped():
                        raise InterruptedError
                    state.progress = step_idx
                    surface_path, center_path = _get_side_paths(side_name, fallback_surface=surface_path, fallback_center=center_path)
                    if not _tracking_is_reliable():
                        _safe_hover_exit(surface_path, center_path, side_name)
                    idx = _idx_for_ratio(len(surface_path), ratio)
                    target_surface, tool_z_unit, orient = _surface_target(surface_path, center_path, idx, 0.0)
                    pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, contact_offset)
                    moved = _move_linear_soft(pose, blend_r=40.0, fine_step_mm=2.5)
                    if not moved:
                        raise RuntimeError(f"MoveCart glide err at step={step_idx}/{len(ratios)}, src_idx={idx}")
                    if use_force_control and (step_idx == 1 or step_idx == len(ratios) or step_idx % 3 == 0):
                        contact_offset, _, _ = _trim_force(target_surface, tool_z_unit, orient, contact_offset, target_force_n, max_iter=4, tol_n=4.0)
                    _check_force_limit()
                    time.sleep(0.02)
            finally:
                robot.SetSpeed(TRANSIT_SPEED)

            end_hover_pose = _pose_from_surface_offset(target_surface, tool_z_unit, orient, hover_offset)
            if not _move_linear_soft(end_hover_pose, blend_r=25.0, fine_step_mm=2.0):
                raise RuntimeError(f"MoveCart glide retract err ({side_name})")
            end_safe_pose = [float(end_hover_pose[0]), float(end_hover_pose[1]), SAFE_Z_MM, float(orient[0]), float(orient[1]), float(orient[2])]
            if not _move_pose(end_safe_pose):
                raise RuntimeError(f"MoveCart glide lift err ({side_name})")

        if use_force_control:
            ret = robot.GetActualTCPPose()
            if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
                pose = ret[1]
                cur_z = float(pose[2])
                if cur_z < SAFE_Z_MM:
                    safe_pose = [
                        float(pose[0]),
                        float(pose[1]),
                        SAFE_Z_MM,
                        float(pose[3]),
                        float(pose[4]),
                        float(pose[5]),
                    ]
                    if not _move_pose(safe_pose):
                        raise RuntimeError("力控初始化前抬升失败")
            time.sleep(0.8)
            if not init_force_sensor(robot, config):
                raise RuntimeError("力传感器初始化失败")
            if config.enable_collision_guard:
                if not setup_collision_guard(robot, config):
                    raise RuntimeError("碰撞守护设置失败")
            else:
                print("[LiveFollow] 碰撞守护已关闭，按目标力恒压演示")

        _, start_snapshot = _get_packed_paths()
        side_order = [_resolve_active_side(start_snapshot)]
        state.follow_side = side_order[0]
        pass_idx = 0
        for side_name in side_order:
            packed, _ = _get_packed_paths()
            if packed is None:
                raise RuntimeError("当前没有新鲜的实时视觉轨迹")
            surface_path = packed[side_name]
            center_path = packed["center"]

            pass_idx += 1
            state.pass_cur = pass_idx
            state.live_mode_name = f"{test_mode_name}-press-split"
            _run_point_press_split(side_name, surface_path, center_path)

            packed, _ = _get_packed_paths()
            if packed is None:
                raise RuntimeError("顺筋阶段缺少实时视觉轨迹")
            surface_path = packed[side_name]
            center_path = packed["center"]

            pass_idx += 1
            state.pass_cur = pass_idx
            state.live_mode_name = f"{test_mode_name}-glide"
            _run_glide(side_name, surface_path, center_path)

        state.status = "done"
    except InterruptedError:
        state.status = "idle"
    except Exception as e:
        state.error_msg = str(e)
        state.status = "error"
        print(f"[LiveFollow] ERROR: {e}")
    finally:
        if robot:
            try:
                robot.StopMotion()
            except Exception:
                pass
            try:
                ret = robot.GetActualTCPPose()
                if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
                    cur = ret[1]
                    safe_pose = [float(cur[0]), float(cur[1]), SAFE_Z_MM, float(cur[3]), float(cur[4]), float(cur[5])]
                    _move_pose(safe_pose)
            except Exception:
                pass
            try:
                if use_force_control:
                    disable_collision_guard(robot)  # flag=0，未开启时亦为安全复位
            except Exception:
                pass
            try:
                if use_force_control:
                    robot.FT_Activate(0)
            except Exception:
                pass
            try:
                robot.CloseRPC()
            except Exception:
                pass


def _write_live_snapshot_json(snapshot, output_dir="robot_meridian_output"):
    os.makedirs(output_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    json_path = os.path.join(output_dir, f"live_snapshot_{ts}.json")
    data = {
        "source": "live_snapshot",
        "seq": int(snapshot.get("seq", 0)),
        "timestamp": float(snapshot.get("timestamp", time.time())),
        "left_meridian_robot": [[float(v) for v in p] for p in snapshot.get("left", [])],
        "right_meridian_robot": [[float(v) for v in p] for p in snapshot.get("right", [])],
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    return json_path


# ===================== Trajectory Linearization =====================

def _linearize_trajectory(points_mm):
    """
    将噪声3D轨迹线性化为干净的直线。SS

    膀胱经在人体背部是一条直线（颈→尾），但深度传感器的逐像素噪声
    导致转换后的3D轨迹在Z轴上下跳动（本例中波动达21mm）。
    线性插值消除中间噪声，仅保留首尾方向。
    """
    if len(points_mm) < 2:
        return points_mm
    n = len(points_mm)
    first = points_mm[0]
    last = points_mm[-1]
    return [
        [
            first[0] + (last[0] - first[0]) * i / (n - 1),
            first[1] + (last[1] - first[1]) * i / (n - 1),
            first[2] + (last[2] - first[2]) * i / (n - 1),
        ]
        for i in range(n)
    ]


# ===================== Robot Worker =====================

def _robot_worker(
    state: MotionState,
    json_path: str,
    sides: list = None,
    robot_ip: str = "192.168.58.2",
    speed: int = 8,
    tool: int = 0,
    user: int = 0,
    rx: float = -178.190,
    ry: float = 1.724,
    rz: float = -1.187,
    approach_height_mm: float = 25.0,
    press_depth_mm: float = 0.0,
    hover_mm: float = 15.0,
    sample_step: int = 2,
    passes: int = 2,
    spline_avg_time_ms: int = 2000,
):
    """
    hover_mm: 在轨迹最高点上方额外抬高的距离，整条轨迹保持此固定高度。
              设为 0 表示贴着体表最高点运行，>0 表示悬浮。
    sides:    要执行的膀胱经侧别列表，默认 ["left", "right"]。
    """
    if sides is None:
        sides = ["left", "right"]

    SAFE_Z_MM = 300.0  # 安全转场高度，远高于人体

    robot = None
    try:
        state.status = "running"
        state.error_msg = ""

        # --- Connect once, reuse for all sides ---
        robot = Robot.RPC(robot_ip)
        robot.SetSpeed(speed)

        def _stopped():
            return state.stop_event.is_set()

        total_pass_count = len(sides) * passes
        state.pass_total = total_pass_count
        global_pass = 0

        for side_idx, side in enumerate(sides):
            if _stopped():
                raise InterruptedError

            print(f"\n[Robot] ========== Side {side_idx+1}/{len(sides)}: {side} ==========")

            # --- Load & transform trajectory for this side ---
            try:
                points, frame = _load_points_prefer_current_calibration(
                    json_path, side=side, prefer_camera_retransform=True
                )
            except Exception as e:
                print(f"[Robot] Skip {side}: {e}")
                global_pass += passes
                continue

            print(f"[Robot] Loaded {len(points)} pts (frame={frame})")

            if frame == "camera":
                T4 = _load_camera_to_robot_matrix()
                if T4 is None:
                    raise RuntimeError("camera_to_robot.json not found")
                points = _transform_points(points, T4)

            points_mm, scale = _to_mm_points(points)

            raw_zs = [p[2] for p in points_mm]
            print(f"[Robot] Raw Z range: [{min(raw_zs):.1f}, {max(raw_zs):.1f}] mm "
                  f"(delta={max(raw_zs)-min(raw_zs):.1f}mm)")

            points_mm = _linearize_trajectory(points_mm)

            # --- Flatten Z: 取最大 Z + hover，整条轨迹保持固定高度 ---
            max_z = max(p[2] for p in points_mm)
            flat_z = max_z + hover_mm
            for p in points_mm:
                p[2] = flat_z
            print(f"[Robot] Fixed Z = {flat_z:.1f} mm (max_surface={max_z:.1f} + hover={hover_mm})")

            sampled = points_mm[:: max(1, int(sample_step))]
            if sampled[-1] != points_mm[-1]:
                sampled.append(points_mm[-1])

            xs = [p[0] for p in sampled]
            ys = [p[1] for p in sampled]
            print(f"[Robot] Trajectory: {len(sampled)} pts")
            print(f"[Robot]   X: [{min(xs):.1f}, {max(xs):.1f}] mm")
            print(f"[Robot]   Y: [{min(ys):.1f}, {max(ys):.1f}] mm")
            print(f"[Robot]   Z: {flat_z:.1f} mm (constant)")

            state.total = len(sampled)

            # --- Safe transit to start ---
            first = sampled[0]

            # Step 0: 垂直抬起到安全高度
            if _stopped():
                raise InterruptedError
            ret = robot.GetActualTCPPose()
            if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
                cur = ret[1]
                cur_z = float(cur[2])
                if cur_z < SAFE_Z_MM:
                    up_pose = [float(cur[0]), float(cur[1]), SAFE_Z_MM,
                               float(cur[3]), float(cur[4]), float(cur[5])]
                    print(f"[Robot] Lifting from Z={cur_z:.1f} to Z={SAFE_Z_MM}")
                    rtn = robot.MoveCart(desc_pos=up_pose, tool=tool, user=user, blendT=0.0)
                    if rtn != 0:
                        print(f"[Robot] MoveCart lift err={rtn}")

            # Step 1: 水平移到轨迹起点上方
            if _stopped():
                raise InterruptedError
            safe_pose = [first[0], first[1], SAFE_Z_MM, rx, ry, rz]
            print(f"[Robot] Transit to {side} start: {safe_pose[:3]}")
            rtn = robot.MoveCart(desc_pos=safe_pose, tool=tool, user=user, blendT=0.0)
            if rtn != 0:
                raise RuntimeError(f"MoveCart transit err={rtn}")

            # Step 2: 下降到运动高度
            if _stopped():
                raise InterruptedError
            approach_pose = [first[0], first[1], flat_z, rx, ry, rz]
            print(f"[Robot] Descend to working Z={flat_z:.1f}")
            rtn = robot.MoveCart(desc_pos=approach_pose, tool=tool, user=user, blendT=0.0)
            if rtn != 0:
                raise RuntimeError(f"MoveCart descend err={rtn}")

            # --- Spline passes (固定高度运行) ---
            for p_idx in range(passes):
                if _stopped():
                    raise InterruptedError
                global_pass += 1
                state.pass_cur = global_pass
                state.progress = 0
                path = sampled if p_idx % 2 == 0 else list(reversed(sampled))
                direction = "fwd(neck->tail)" if p_idx % 2 == 0 else "rev(tail->neck)"
                print(f"[Robot] Pass {global_pass}/{total_pass_count} "
                      f"({side} {direction}), {len(path)} pts, Z={flat_z:.1f}")

                rtn = robot.NewSplineStart(type=0, averageTime=spline_avg_time_ms)
                use_spline = rtn == 0
                if not use_spline:
                    print(f"[Robot] SplineStart failed err={rtn}, fallback to MoveL")

                for i, p in enumerate(path):
                    if _stopped():
                        if use_spline:
                            robot.NewSplineEnd()
                        raise InterruptedError
                    state.progress = i + 1
                    pose = [p[0], p[1], flat_z, rx, ry, rz]
                    if use_spline:
                        last_flag = 1 if i == len(path) - 1 else 0
                        rtn = robot.NewSplinePoint(
                            desc_pos=pose, tool=tool, user=user, lastFlag=last_flag,
                        )
                        if rtn != 0:
                            print(f"[Robot] SplinePoint err={rtn} at idx={i}")
                    else:
                        rtn = robot.MoveL(desc_pos=pose, tool=tool, user=user, blendR=50.0)
                        if rtn != 0:
                            print(f"[Robot] MoveL err={rtn} at idx={i}")

                if use_spline:
                    rtn = robot.NewSplineEnd()
                    if rtn != 0:
                        print(f"[Robot] SplineEnd err={rtn}")

                print(f"[Robot] Pass {global_pass} done ({side})")

            # --- 该侧完成，抬起 ---
            if _stopped():
                raise InterruptedError
            last = sampled[-1] if passes % 2 != 0 else sampled[0]
            lift_pose = [last[0], last[1], SAFE_Z_MM, rx, ry, rz]
            print(f"[Robot] {side} done, lifting to Z={SAFE_Z_MM}")
            robot.MoveCart(desc_pos=lift_pose, tool=tool, user=user, blendT=0.0)

        # --- All sides complete ---
        robot.CloseRPC()
        state.status = "done"
        print(f"\n[Robot] All {len(sides)} sides complete ({total_pass_count} passes total)")

    except InterruptedError:
        state.status = "idle"
        print("[Robot] Stopped by user")
        if robot:
            try:
                robot.CloseRPC()
            except Exception:
                pass
    except Exception as e:
        state.error_msg = str(e)
        state.status = "error"
        print(f"[Robot] ERROR: {e}")
        if robot:
            try:
                robot.CloseRPC()
            except Exception:
                pass


def _read_current_tcp_pose(robot_ip: str = DEFAULT_ROBOT_IP):
    """读取当前 TCP 位姿，返回 [x, y, z, rx, ry, rz] 或 None。"""
    robot = None
    try:
        robot = Robot.RPC(robot_ip)
        ret = robot.GetActualTCPPose()
        if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
            pose = ret[1]
            return [float(v) for v in pose[:6]]
        print(f"[Demo] 读取当前 TCP 位姿失败: {ret}")
        return None
    except Exception as e:
        print(f"[Demo] 读取当前 TCP 位姿异常: {e}")
        return None
    finally:
        if robot:
            try:
                robot.CloseRPC()
            except Exception:
                pass


def _resolve_demo_start_pose(robot_ip: str = DEFAULT_ROBOT_IP):
    """
    决定本次 demo 起始位：
    1. 若设置 DEMO_START_POSE，则使用固定 pose
    2. 否则读取机器人当前位置
    """
    pose_text = os.environ.get("DEMO_START_POSE", "").strip()
    if pose_text:
        pose = _parse_pose6_text(pose_text)
        if pose is not None:
            return pose, "env"
        print(f"[Demo] 忽略无效 DEMO_START_POSE={pose_text!r}，改为读取当前位置")

    pose = _read_current_tcp_pose(robot_ip=robot_ip)
    if pose is not None:
        return pose, "current"
    return None, "unavailable"


def _move_to_initial_pose(start_pose=None, start_label: str = "p24", robot_ip: str = DEFAULT_ROBOT_IP, speed: int = 10):
    """
    每次任务开始前回到指定起始位姿。
    返回 True/False。
    """
    robot = None
    try:
        target_pose = [float(v) for v in (start_pose if start_pose is not None else INIT_POSE_P24)]
        robot = Robot.RPC(robot_ip)
        robot.SetSpeed(speed)

        # 先垂直抬到安全高度，再平移到初始点，避免低位横移碰撞
        ret = robot.GetActualTCPPose()
        if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
            cur = ret[1]
            cur_z = float(cur[2])
            if cur_z < INIT_SAFE_Z_MM:
                up_pose = [float(cur[0]), float(cur[1]), INIT_SAFE_Z_MM,
                           float(cur[3]), float(cur[4]), float(cur[5])]
                rtn = robot.MoveCart(desc_pos=up_pose, tool=INIT_TOOL, user=INIT_USER, blendT=0.0)
                if rtn != 0:
                    print(f"[Demo] 回初始位前抬升失败, err={rtn}")
                    return False

        rtn = robot.MoveCart(desc_pos=target_pose, tool=INIT_TOOL, user=INIT_USER, blendT=0.0)
        if rtn != 0:
            print(f"[Demo] 回到起始位({start_label})失败, err={rtn}")
            return False

        print(f"[Demo] 已回到起始位 {start_label}: {target_pose}")
        return True
    except Exception as e:
        print(f"[Demo] 回初始位异常: {e}")
        return False
    finally:
        if robot:
            try:
                robot.CloseRPC()
            except Exception:
                pass


# ===================== Main Loop =====================

def main():
    try:
        finger_mm = float(input("手指宽度(mm, 默认45): ").strip() or "45")
    except Exception:
        finger_mm = 45.0

    # 力控开关：FORCE_CONTROL=0 回退到无力控模式
    use_force_control = os.environ.get("FORCE_CONTROL", "1") != "0"
    force_cfg_base = _build_force_config_from_env()
    skip_home = _env_flag("DEMO_SKIP_HOME")
    robot_ip = _env_first_nonempty("ROBOT_IP") or DEFAULT_ROBOT_IP

    detector = LinearMeridianDetector(finger_mm)
    smoother = PointSmoother(alpha=0.25, max_step_px=12.0)
    line_stabilizer = MeridianLineStabilizer()
    tracker = BackRegionCoTracker()
    live_buffer = LiveTrajectoryBuffer()
    tracker_visualizer = _build_cotracker_visualizer()
    tracker_overlay_cache = {"render_seq_id": None, "frame_bgr": None}
    frame_idx = 0

    def _pt(p):
        return int(round(p[0])), int(round(p[1]))

    if os.environ.get("DEMO_START_POSE", "").strip():
        print("[Demo] 启动：不再自动回 p24；本次 demo 将在按 r/t 时使用 DEMO_START_POSE")
    else:
        print("[Demo] 启动：不再自动回 p24；按 r/t 时将直接读取机器人当前位置作为起始位")
    if skip_home:
        print("[Demo] DEMO_SKIP_HOME=1 已兼容保留；当前版本默认启动即不自动回位")

    saved_json = None
    saved_pixels = None  # ((neck_l, tail_l), (neck_r, tail_r))
    state = MotionState()
    tf0 = os.environ.get("TARGET_FORCE_N", "").strip()
    if tf0:
        try:
            v = float(tf0)
            state.target_force = max(3.0, min(55.0, v))
            print(f"[Demo] TARGET_FORCE_N 初始目标力: {state.target_force:.1f}N")
        except ValueError:
            print(f"[Demo] 忽略无效 TARGET_FORCE_N={tf0!r}")
    worker = None
    visual_ready = False
    visual_motion_ready = False
    visual_status = "search"
    show_debug_overlay = _env_flag("DEMO_DEBUG_OVERLAY", True)
    active_side_name = ACTIVE_SIDE_NAME
    active_side_conf_mm = 0.0
    visual_only_preview = True

    try:
        while True:
            # ---- Camera capture (always running for display) ----
            frames = detector.pipeline.wait_for_frames()
            frames = detector.align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            cur_status = state.status
            frame_idx += 1

            # ---- Detection phase ----
            meridian_lines = None
            outer_meridian_lines = None
            spine_line = None
            tracker_result = tracker.last_result
            line_source = "none"

            # ---- Run YOLO + CoTracker continuously ----
            pose_info = infer_best_pose_with_rotations(detector.model, img, conf=0.5)

            detected = False
            pose_conf = 0.0
            yolo_spine_seed = None
            if pose_info["kpts"] is not None:
                kpts = pose_info["kpts"]
                pose_conf = float(
                    min(kpts[5][2], kpts[6][2], kpts[11][2], kpts[12][2])
                )

                if (
                    kpts[5][2] > 0.5
                    and kpts[6][2] > 0.5
                    and kpts[11][2] > 0.3
                    and kpts[12][2] > 0.3
                ):
                    spine_seed = _build_spine_seed_from_torso(kpts, smoother, detector.user_finger_width_mm)
                    if spine_seed is not None:
                        detected = True
                    if spine_seed is not None:
                        (neck_u, neck_v), (tail_u, tail_v) = spine_seed["spine_line"]
                        lateral_direction_2d = np.asarray(
                            spine_seed["lateral_direction_2d"],
                            dtype=np.float64,
                        )
                        body_offset_px = float(spine_seed["body_offset_px"])
                        shoulder_px = float(spine_seed["shoulder_px"])

                        raw_neck_l = (
                            neck_u - lateral_direction_2d[0] * body_offset_px,
                            neck_v - lateral_direction_2d[1] * body_offset_px,
                        )
                        raw_neck_r = (
                            neck_u + lateral_direction_2d[0] * body_offset_px,
                            neck_v + lateral_direction_2d[1] * body_offset_px,
                        )
                        raw_tail_l = (
                            tail_u - lateral_direction_2d[0] * body_offset_px,
                            tail_v - lateral_direction_2d[1] * body_offset_px,
                        )
                        raw_tail_r = (
                            tail_u + lateral_direction_2d[0] * body_offset_px,
                            tail_v + lateral_direction_2d[1] * body_offset_px,
                        )

                        neck_l = smoother.smooth("neck_l", *raw_neck_l, round_result=False)
                        neck_r = smoother.smooth("neck_r", *raw_neck_r, round_result=False)
                        tail_l = smoother.smooth("tail_l", *raw_tail_l, round_result=False)
                        tail_r = smoother.smooth("tail_r", *raw_tail_r, round_result=False)

                        spine_line = ((neck_u, neck_v), (tail_u, tail_v))
                        if neck_l and tail_l and neck_r and tail_r:
                            meridian_lines = ((neck_l, tail_l), (neck_r, tail_r))
                            yolo_spine_seed = spine_line
                            allow_reseed = (cur_status != "running")
                            if allow_reseed and tracker.should_reseed(frame_idx):
                                tracker_result = tracker.seed(
                                    img,
                                    yolo_spine_seed,
                                    lateral_direction_2d,
                                    body_offset_px,
                                    frame_idx,
                                )
                            else:
                                tracker_result = tracker.update(img, frame_idx=frame_idx)
                            line_source = "yolo"

            if not detected:
                smoother.miss()
                tracker_result = tracker.update(img, frame_idx=frame_idx)
            elif yolo_spine_seed is None:
                tracker_result = tracker.update(img, frame_idx=frame_idx)

            tracker_vis_ratio = 0.0
            tracker_fit_span = 0.0
            tracker_valid_points = 0
            if tracker_result is not None:
                tracker_vis_ratio = float(tracker_result.get("visible_ratio", tracker.visible_ratio))
                tracker_fit_span = float(tracker_result.get("fit_span_ratio", 0.0))
                tracker_valid_points = int(tracker_result.get("valid_spine_points", 0))

            # 只要脊柱跟踪仍然可用，就优先使用脊柱采样点的跟踪结果；
            # YOLO 主要负责初始化和周期性重置脊柱位置。
            tracker_reliable = (
                tracker_result is not None
                and tracker_fit_span >= 0.30
                and tracker_valid_points >= max(4, int(tracker.min_valid_points * 0.5))
            )
            prefer_tracker = (
                tracker_reliable
                and tracker_vis_ratio >= 0.18
            )

            if prefer_tracker:
                if tracker_result.get("spine_line") is not None:
                    spine_line = tracker_result["spine_line"]
                if tracker_result.get("meridian_lines") is not None:
                    meridian_lines = tracker_result["meridian_lines"]
                    line_source = "cotracker"
            elif meridian_lines is None and tracker_reliable:
                if tracker_result.get("spine_line") is not None:
                    spine_line = tracker_result["spine_line"]
                if tracker_result.get("meridian_lines") is not None:
                    meridian_lines = tracker_result["meridian_lines"]
                    line_source = "cotracker"

            stable_track = line_stabilizer.update(
                meridian_lines=meridian_lines,
                line_source=line_source,
                pose_conf=pose_conf,
                tracker_vis_ratio=tracker_vis_ratio,
            )
            spine_line = stable_track["spine_line"]
            meridian_lines = stable_track["meridian_lines"]
            visual_ready = bool(stable_track["ready"])
            visual_status = str(stable_track["status"])
            visual_motion_ready = visual_ready and visual_status == "stable"
            line_source = str(stable_track["source"])
            outer_meridian_lines = _expand_meridian_lines_from_spine(
                spine_line,
                meridian_lines,
                scale=2.0,
            )

            live_ok = live_buffer.update_from_lines(detector, meridian_lines, depth_frame)
            live_meta = live_buffer.peek_state()
            active_side_name = str(live_meta.get("active_side", ACTIVE_SIDE_NAME))
            active_side_conf_mm = float(live_meta.get("active_side_conf_mm", 0.0))

            official_tracker_overlay = None
            if tracker_result is not None and show_debug_overlay and tracker_visualizer is not None:
                render_seq_id = tracker_result.get("render_seq_id")
                if tracker_overlay_cache["render_seq_id"] != render_seq_id:
                    tracker_overlay_cache["render_seq_id"] = render_seq_id
                    tracker_overlay_cache["frame_bgr"] = _render_official_cotracker_overlay(
                        tracker_result,
                        tracker_visualizer,
                    )
                official_tracker_overlay = tracker_overlay_cache.get("frame_bgr")
                if official_tracker_overlay is not None and official_tracker_overlay.shape == img.shape:
                    img = official_tracker_overlay.copy()

            if tracker_result is not None and show_debug_overlay and official_tracker_overlay is None:
                tracker_pts = tracker_result.get("grid_points", [])
                tracker_vis = tracker_result.get("grid_visible", [])
                tracker_spine_pts = tracker_result.get("spine_points", [])

                prev_fit_pt = None
                for fit_pt in tracker_spine_pts:
                    fit_xy = _pt(fit_pt)
                    if prev_fit_pt is not None:
                        cv2.line(img, prev_fit_pt, fit_xy, (255, 255, 255), 1)
                    cv2.circle(img, fit_xy, 2, (255, 255, 255), -1)
                    prev_fit_pt = fit_xy

                for pt, vis in zip(tracker_pts, tracker_vis):
                    xy = _pt(pt)
                    if vis:
                        cv2.circle(img, xy, 3, (255, 0, 255), -1)
                    else:
                        cv2.circle(img, xy, 2, (90, 90, 90), 1)
                        cv2.line(img, (xy[0] - 2, xy[1] - 2), (xy[0] + 2, xy[1] + 2), (90, 90, 90), 1)
                        cv2.line(img, (xy[0] - 2, xy[1] + 2), (xy[0] + 2, xy[1] - 2), (90, 90, 90), 1)

            if spine_line is not None:
                cv2.line(img, _pt(spine_line[0]), _pt(spine_line[1]), (0, 0, 255), 2)
                if show_debug_overlay:
                    cv2.circle(img, _pt(spine_line[0]), 4, (0, 0, 255), -1)
                    cv2.circle(img, _pt(spine_line[1]), 4, (0, 0, 255), -1)
            if outer_meridian_lines is not None:
                outer_colors = ((255, 220, 0), (255, 220, 0))
                for outer_line, color in zip(outer_meridian_lines, outer_colors):
                    if outer_line is None:
                        continue
                    cv2.line(img, _pt(outer_line[0]), _pt(outer_line[1]), color, 2)
                    cv2.circle(img, _pt(outer_line[0]), 3, color, -1)
                    cv2.circle(img, _pt(outer_line[1]), 3, color, -1)
            if meridian_lines is not None:
                inner_colors = ((0, 255, 0), (0, 255, 0))
                for inner_line, color in zip(meridian_lines, inner_colors):
                    if inner_line is None:
                        continue
                    cv2.line(img, _pt(inner_line[0]), _pt(inner_line[1]), color, 2)
                    cv2.circle(img, _pt(inner_line[0]), 4, color, -1)
                    cv2.circle(img, _pt(inner_line[1]), 4, color, -1)

            # ---- Saved trajectory overlay (cyan thick) ----
            if saved_pixels:
                (nl, tl), (nr, tr) = saved_pixels
                cv2.line(img, _pt(nl), _pt(tl), (255, 255, 0), 3)
                cv2.line(img, _pt(nr), _pt(tr), (255, 255, 0), 3)

            # ---- HUD ----
            yt = 30
            vision_label = "LOCKED" if visual_motion_ready else visual_status.upper()
            vision_color = (0, 220, 0) if visual_motion_ready else (0, 215, 255) if visual_status in ("warming", "hold") else (180, 180, 180)
            cv2.putText(
                img,
                f"Preview  inner={detector.user_finger_width_mm:.0f}mm  outer={detector.user_finger_width_mm * 2.0:.0f}mm  vision={vision_label}",
                (10, yt), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
            )
            yt += 28

            if cur_status == "idle":
                hint = (
                    "Preview only | s:save  u/d:offset  p:tracker  q:quit"
                )
            elif cur_status == "running":
                hint = "Robot running | q:stop"
            elif cur_status == "done":
                hint = "Done | s:save  p:tracker  q:quit"
            else:
                hint = f"ERR: {state.error_msg[:50]}"
            cv2.putText(
                img, hint, (10, yt),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1,
            )
            yt += 28

            if tracker.available:
                vis_ratio = 100.0 * tracker.visible_ratio
                if not show_debug_overlay:
                    tracker_vis_mode = "off"
                elif tracker_visualizer is not None:
                    tracker_vis_mode = "official"
                else:
                    tracker_vis_mode = "fallback"
                cv2.putText(
                    img,
                    f"Track={line_source}  vis={vis_ratio:.0f}%  valid={tracker_valid_points:02d}/40  span={tracker_fit_span:.2f}  visible_side={active_side_name}  side_conf={active_side_conf_mm:.0f}mm  ready={'Y' if visual_motion_ready else 'N'}  live={'ok' if live_ok else 'stale'}  tracker={tracker_vis_mode}",
                    (10, yt),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, vision_color, 1,
                )
                yt += 24

            guard_on = force_cfg_base.enable_collision_guard
            cv2.putText(
                img,
                f"FT_Guard={'on' if guard_on else 'OFF'}  payload={force_cfg_base.payload_weight:.2f}kg  start={'env' if os.environ.get('DEMO_START_POSE', '').strip() else 'current'}",
                (10, yt),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 220, 180) if guard_on else (0, 165, 255),
                1,
            )
            yt += 22

            if cur_status == "running":
                prog = state.progress
                total = state.total
                pc = state.pass_cur
                pt = state.pass_total
                live_mode = state.live_mode_name if state.live_mode_name else ("live-single-demo" if state.live_seq > 0 else "single demo")
                cv2.putText(
                    img,
                    f"{live_mode}  Pass {pc}/{pt}  Point {prog}/{total}  seq={state.live_seq}",
                    (10, yt), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2,
                )
                yt += 28
                # 力控信息
                if use_force_control:
                    fz_val = state.force_z
                    fz_color = (0, 200, 0) if abs(fz_val) < 20 else (0, 0, 255)
                    cv2.putText(
                        img,
                        f"Fz={fz_val:.1f}N  target={state.target_force:.0f}N  mode={state.force_mode}",
                        (10, yt), cv2.FONT_HERSHEY_SIMPLEX, 0.55, fz_color, 2,
                    )
                    yt += 28
                bar_w = 300
                overall = pc - 1 + (prog / max(total, 1))
                if pt > 0:
                    filled = int(bar_w * overall / pt)
                    cv2.rectangle(img, (10, yt), (10 + bar_w, yt + 18), (80, 80, 80), -1)
                    cv2.rectangle(img, (10, yt), (10 + filled, yt + 18), (0, 200, 0), -1)
                    cv2.rectangle(img, (10, yt), (10 + bar_w, yt + 18), (255, 255, 255), 1)
            elif cur_status == "done":
                cv2.putText(
                    img, "Motion Complete!", (10, yt),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                )
            elif cur_status == "error":
                cv2.putText(
                    img, "Motion Failed", (10, yt),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                )

            if saved_json:
                cv2.putText(
                    img,
                    f"Saved: {os.path.basename(saved_json)}",
                    (10, img.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1,
                )
            if use_force_control and cur_status in ("idle", "done"):
                cv2.putText(
                    img,
                    f"Force: {state.force_mode}  target={state.target_force:.0f}N",
                    (10, img.shape[0] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 255), 1,
                )

            cv2.imshow("Meridian Demo", img)
            key = cv2.waitKey(1) & 0xFF

            # ---- Key handling ----
            if key == ord("q"):
                if state.status == "running":
                    state.stop_event.set()
                    if worker and worker.is_alive():
                        worker.join(timeout=5)
                break

            # Block save/offset during robot motion
            if key == ord("s") and cur_status != "running":
                if meridian_lines is None:
                    print("当前帧膀胱经线无效，未保存")
                else:
                    _, jpath = detector.save_meridian_result(
                        img.copy(),
                        meridian_lines[0],
                        meridian_lines[1],
                        depth_frame,
                    )
                    saved_json = jpath
                    saved_pixels = meridian_lines
                    print(f"[已保存] {jpath}")

            if key == ord("g") and cur_status != "running":
                print("[Demo] 固定轨迹演示已下线；请按 r 启动单侧实时力控演示")

            if key == ord("r"):
                if visual_only_preview:
                    print("[Demo] 当前版本仅做视觉预览：展示脊柱两侧四条膀胱经，已禁用机器人动作")
                    continue
                if state.status == "running":
                    print("机器人运动中，请等待完成")
                elif live_buffer.get_latest() is None or not visual_motion_ready:
                    print("当前视觉轨迹还未稳定；请先保持侧卧人体在画面中约 0.5~1 秒，等待 state=stable / ready=Y")
                else:
                    motion_snapshot = live_buffer.get_latest()
                    if motion_snapshot is None:
                        print("[Demo] 启动失败：未取到初始膀胱经轨迹快照")
                        continue
                    motion_side = str(motion_snapshot.get("active_side", active_side_name))
                    start_pose, start_source = _resolve_demo_start_pose(robot_ip=robot_ip)
                    if start_pose is None:
                        print("[Demo] 无法读取当前位置；如需固定起点，请设置 DEMO_START_POSE=x,y,z,rx,ry,rz")
                        continue
                    if start_source == "env":
                        if not _move_to_initial_pose(start_pose=start_pose, start_label="env", robot_ip=robot_ip):
                            print("[Demo] 未能回到 DEMO_START_POSE，已取消本次实时演示")
                            continue
                    else:
                        print(f"[Demo] 本次实时演示使用当前位置作为起始位: {start_pose}")

                    state.status = "idle"
                    state.progress = 0
                    state.total = 0
                    state.pass_cur = 0
                    state.pass_total = 0
                    state.error_msg = ""
                    state.force_z = 0.0
                    state.live_seq = 0
                    state.live_mode_name = "live-single-demo"
                    state.stop_event.clear()

                    worker = threading.Thread(
                        target=_live_follow_worker,
                        kwargs=dict(
                            state=state,
                            live_buffer=live_buffer,
                            tracker=tracker,
                            static_snapshot=motion_snapshot,
                            follow_live_path=False,
                            robot_ip=robot_ip,
                            use_force_control=use_force_control,
                            hover_mm=12.0,
                            approach_height_mm=28.0,
                            occlusion_shift_mm=0.0,
                            speed=6,
                            force_mode=state.force_mode,
                            target_force_n=state.target_force,
                            test_mode_name="live-demo",
                            config=replace(force_cfg_base),
                        ),
                        daemon=True,
                    )
                    worker.start()
                    mode_str = (
                        f"单侧固定轨迹力控({state.force_mode}, side={motion_side}, target={state.target_force:.0f}N, 点压分筋+顺筋)"
                        if use_force_control else
                        f"单侧固定轨迹跟踪(side={motion_side}, 无力控)"
                    )
                    print(f"[Demo] 已启动单侧按摩演示 [{mode_str}]，按摩使用启动瞬间的初始膀胱经轨迹")

            if key == ord("t"):
                if visual_only_preview:
                    print("[Demo] 当前版本仅做视觉预览：展示脊柱两侧四条膀胱经，已禁用机器人动作")
                    continue
                if state.status == "running":
                    print("机器人运动中，请等待完成")
                elif live_buffer.get_latest() is None or not visual_motion_ready:
                    print("当前视觉轨迹还未稳定；请先保持侧卧人体在画面中约 0.5~1 秒，等待 state=stable / ready=Y")
                else:
                    motion_snapshot = live_buffer.get_latest()
                    if motion_snapshot is None:
                        print("[Demo] 启动失败：未取到初始膀胱经轨迹快照")
                        continue
                    motion_side = str(motion_snapshot.get("active_side", active_side_name))
                    start_pose, start_source = _resolve_demo_start_pose(robot_ip=robot_ip)
                    if start_pose is None:
                        print("[Demo] 无法读取当前位置；如需固定起点，请设置 DEMO_START_POSE=x,y,z,rx,ry,rz")
                        continue
                    if start_source == "env":
                        if not _move_to_initial_pose(start_pose=start_pose, start_label="env", robot_ip=robot_ip):
                            print("[Demo] 未能回到 DEMO_START_POSE，已取消本次悬空测试")
                            continue
                    else:
                        print(f"[Demo] 本次悬空测试使用当前位置作为起始位: {start_pose}")

                    state.status = "idle"
                    state.progress = 0
                    state.total = 0
                    state.pass_cur = 0
                    state.pass_total = 0
                    state.error_msg = ""
                    state.force_z = 0.0
                    state.live_seq = 0
                    state.live_mode_name = ""
                    state.stop_event.clear()

                    worker = threading.Thread(
                        target=_live_follow_worker,
                        kwargs=dict(
                            state=state,
                            live_buffer=live_buffer,
                            tracker=tracker,
                            static_snapshot=motion_snapshot,
                            follow_live_path=False,
                            robot_ip=robot_ip,
                            use_force_control=False,
                            hover_mm=40.0,
                            occlusion_shift_mm=35.0,
                            speed=6,
                            test_mode_name="hover-test",
                            config=replace(force_cfg_base),
                        ),
                        daemon=True,
                    )
                    worker.start()
                    print(
                        f"[Demo] 悬空遮挡测试已启动 [hover-test, side={motion_side}, hover=40mm, shift=35mm]，"
                        "机器人将更靠近相机视线以验证遮挡跟踪"
                    )

            if key == ord("u") and cur_status != "running":
                detector.user_finger_width_mm += 5
            if key == ord("d") and cur_status != "running":
                detector.user_finger_width_mm -= 5
            if key == ord("p") and cur_status != "running":
                show_debug_overlay = not show_debug_overlay
                print(f"[Demo] CoTracker overlay: {'on' if show_debug_overlay else 'off'}")

            # 力控模式切换 (仅 idle/done 时)
            if key == ord("f") and cur_status != "running":
                if state.force_mode == "ft_control":
                    state.force_mode = "impedance"
                else:
                    state.force_mode = "ft_control"
                print(f"[Demo] 力控模式切换为: {state.force_mode}")

            # 目标力档位调整
            if key == ord("+") or key == ord("="):
                if cur_status != "running":
                    cur_force = _snap_force_level(state.target_force)
                    idx = FORCE_LEVELS_N.index(cur_force)
                    state.target_force = FORCE_LEVELS_N[min(idx + 1, len(FORCE_LEVELS_N) - 1)]
                    print(f"[Demo] 目标力档位: {state.target_force:.0f}N")
            if key == ord("-") or key == ord("_"):
                if cur_status != "running":
                    cur_force = _snap_force_level(state.target_force)
                    idx = FORCE_LEVELS_N.index(cur_force)
                    state.target_force = FORCE_LEVELS_N[max(idx - 1, 0)]
                    print(f"[Demo] 目标力档位: {state.target_force:.0f}N")

    finally:
        detector.pipeline.stop()
        cv2.destroyAllWindows()
        if state.status == "running":
            state.stop_event.set()
            if worker and worker.is_alive():
                worker.join(timeout=5)


if __name__ == "__main__":
    main()
