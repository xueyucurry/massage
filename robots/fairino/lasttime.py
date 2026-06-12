"""
lasttime.py - 膀胱经按摩动作演示程序

完整复用demo.py的视觉检测流程（YOLO + CoTracker + MeridianLineStabilizer）
在左外侧膀胱经上执行点筋、分筋、顺筋动作
"""

import os
import sys
import time
import threading
import math
import numpy as np
import cv2

# 导入demo.py中的所有检测模块
from demo import (
    LinearMeridianDetector,
    BackRegionCoTracker,
    MeridianLineStabilizer,
    PointSmoother,
    _build_spine_seed_from_torso,
    _camera_origin_mm_from_matrix,
    _expand_meridian_lines_from_spine,
    _normalize_vec,
    _rpy_from_tool_z_vector,
    infer_best_pose_with_rotations,
    DEFAULT_ROBOT_IP,
    INIT_POSE_P24,
    INIT_SAFE_Z_MM,
)

# 导入机械臂控制（与demo.py相同的方式）
from fairino import Robot, SDK_ROOT as FAIRINO_SDK_ROOT, SDK_MODULE as FAIRINO_SDK_MODULE
from force_control import ForceControlConfig, get_force_z, init_force_sensor

# ===================== 配置常量 =====================

SAMPLE_POINTS = 10
STABLE_FRAMES = 8

HOVER_HEIGHT_MM = float(os.environ.get("HOVER_HEIGHT_MM", "25.0"))
DIAN_JIN_DEPTH_MM = float(os.environ.get("DIAN_JIN_DEPTH_MM", "20.0"))
FEN_JIN_LATERAL_MM = float(os.environ.get("FEN_JIN_LATERAL_MM", "20.0"))

MOVE_VEL_FAST = 30
MOVE_VEL_SLOW = 10
BLEND_BLOCKING = -1.0
ROBOT_CONNECT_RETRIES = int(os.environ.get("ROBOT_CONNECT_RETRIES", "5"))
ROBOT_CONNECT_RETRY_DELAY_S = float(os.environ.get("ROBOT_CONNECT_RETRY_DELAY_S", "1.0"))

ROBOT_IP = os.environ.get("ROBOT_IP", DEFAULT_ROBOT_IP)
USE_REALTIME_STATE = os.environ.get("LASTTIME_USE_REALTIME", "1").strip().lower() in {
    "1", "true", "yes", "on",
}
PLANE_FIT_RADIUS_PX = int(os.environ.get("LASTTIME_PLANE_FIT_RADIUS_PX", "6"))
PLANE_FIT_STEP_PX = int(os.environ.get("LASTTIME_PLANE_FIT_STEP_PX", "2"))
PLANE_FIT_MIN_POINTS = int(os.environ.get("LASTTIME_PLANE_FIT_MIN_POINTS", "12"))
PLANE_NORMAL_SMOOTH_WINDOW = int(os.environ.get("LASTTIME_PLANE_NORMAL_SMOOTH_WINDOW", "3"))
ENABLE_LIVE_PREVIEW_WINDOW = os.environ.get("LASTTIME_LIVE_PREVIEW", "1").strip().lower() in {
    "1", "true", "yes", "on",
}
LIVE_PREVIEW_WINDOW_NAME = os.environ.get("LASTTIME_LIVE_PREVIEW_WINDOW", "Live Preview")
LASTTIME_FORCE_CONTROL = os.environ.get("LASTTIME_FORCE_CONTROL", "1").strip().lower() in {
    "1", "true", "yes", "on",
}
LASTTIME_FORCE_N = float(os.environ.get("LASTTIME_FORCE_N", os.environ.get("TARGET_FORCE_N", "10.0")))
LASTTIME_FORCE_SOFTWARE_LIMIT_N = float(os.environ.get("LASTTIME_FORCE_SOFTWARE_LIMIT_N", "20.0"))
LASTTIME_FORCE_CONTACT_OFFSET_MM = float(os.environ.get("LASTTIME_FORCE_CONTACT_OFFSET_MM", "0.0"))
LASTTIME_FORCE_PRESS_LIMIT_MM = float(os.environ.get("LASTTIME_FORCE_PRESS_LIMIT_MM", "18.0"))
LASTTIME_FORCE_KP = float(os.environ.get("LASTTIME_FORCE_KP", "0.10"))
LASTTIME_FORCE_KI = float(os.environ.get("LASTTIME_FORCE_KI", "0.008"))
LASTTIME_FORCE_IMAX = float(os.environ.get("LASTTIME_FORCE_IMAX", "40.0"))
LASTTIME_FORCE_MAX_STEP_MM = float(os.environ.get("LASTTIME_FORCE_MAX_STEP_MM", "0.5"))
LASTTIME_FORCE_SETTLE_S = float(os.environ.get("LASTTIME_FORCE_SETTLE_S", "0.18"))
LASTTIME_FORCE_TOL_N = float(os.environ.get("LASTTIME_FORCE_TOL_N", "2.0"))
LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION = os.environ.get(
    "LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION", "1"
).strip().lower() in {
    "1", "true", "yes", "on",
}
LASTTIME_SAFE_Z_MM = float(os.environ.get("LASTTIME_SAFE_Z_MM", str(INIT_SAFE_Z_MM)))


# ===================== 辅助函数 =====================

def _pt(p):
    return (int(round(p[0])), int(round(p[1])))


def sample_line_pixels(start, end, num_points=SAMPLE_POINTS):
    """在线段上均匀采样点"""
    sx, sy = start
    ex, ey = end
    points = []
    for i in range(num_points):
        t = i / max(1, num_points - 1)
        x = sx + (ex - sx) * t
        y = sy + (ey - sy) * t
        points.append((x, y))
    return points


def calculate_lateral_vector_3d(spine_points_mm):
    """计算脊柱的3D侧向向量"""
    if len(spine_points_mm) < 2:
        return [1.0, 0.0, 0.0]

    start = np.array(spine_points_mm[0])
    end = np.array(spine_points_mm[-1])
    spine_vec = end - start
    spine_vec_2d = np.array([spine_vec[0], spine_vec[1], 0.0])
    spine_len = np.linalg.norm(spine_vec_2d)

    if spine_len < 1e-6:
        return [1.0, 0.0, 0.0]

    spine_unit = spine_vec_2d / spine_len
    lateral = np.array([-spine_unit[1], spine_unit[0], 0.0])
    return lateral.tolist()


def _transform_vector_camera_to_robot(matrix4, vec3):
    if matrix4 is None:
        return None
    try:
        rot = np.asarray(matrix4, dtype=np.float64)[:3, :3]
        transformed = rot @ np.asarray(vec3, dtype=np.float64)
    except Exception:
        return None
    return _normalize_vec(transformed)


def _estimate_patch_plane_normal_camera(
    detector,
    depth_frame,
    pixel_u,
    pixel_v,
    radius_px=PLANE_FIT_RADIUS_PX,
    step_px=PLANE_FIT_STEP_PX,
    min_points=PLANE_FIT_MIN_POINTS,
):
    points = []
    for du in range(-int(radius_px), int(radius_px) + 1, max(1, int(step_px))):
        for dv in range(-int(radius_px), int(radius_px) + 1, max(1, int(step_px))):
            p3 = detector.get_point3d_from_depth(float(pixel_u + du), float(pixel_v + dv), depth_frame)
            if p3 is None:
                continue
            points.append(np.asarray(p3, dtype=np.float64))

    if len(points) < max(3, int(min_points)):
        return None

    arr = np.asarray(points, dtype=np.float64)
    center = np.mean(arr, axis=0)
    try:
        _, _, vh = np.linalg.svd(arr - center, full_matrices=False)
    except Exception:
        return None
    return _normalize_vec(vh[-1])


def _fallback_tool_z_axis(point_mm, camera_origin_mm):
    if camera_origin_mm is not None:
        outward = _normalize_vec(np.asarray(camera_origin_mm, dtype=np.float64) - np.asarray(point_mm, dtype=np.float64))
        if outward is not None:
            return -outward
    return np.array([0.0, 0.0, -1.0], dtype=np.float64)


def _smooth_unit_vectors(vectors, window=PLANE_NORMAL_SMOOTH_WINDOW):
    if not vectors:
        return []
    radius = max(0, int(window) // 2)
    out = []
    for idx, vec in enumerate(vectors):
        seg = []
        ref = np.asarray(vec, dtype=np.float64)
        for j in range(max(0, idx - radius), min(len(vectors), idx + radius + 1)):
            cur = np.asarray(vectors[j], dtype=np.float64)
            if float(np.dot(cur, ref)) < 0.0:
                cur = -cur
            seg.append(cur)
        avg = _normalize_vec(np.mean(seg, axis=0))
        if avg is None:
            avg = ref
        if out and float(np.dot(avg, out[-1])) < 0.0:
            avg = -avg
        out.append(avg)
    return out


def _build_split_axis(tool_z_unit, tangent_unit):
    split_axis = _normalize_vec(np.cross(np.asarray(tool_z_unit, dtype=np.float64), np.asarray(tangent_unit, dtype=np.float64)))
    if split_axis is not None:
        return split_axis
    for fallback in (
        np.array([0.0, 0.0, 1.0], dtype=np.float64),
        np.array([0.0, 1.0, 0.0], dtype=np.float64),
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
    ):
        split_axis = _normalize_vec(np.cross(np.asarray(tool_z_unit, dtype=np.float64), fallback))
        if split_axis is not None:
            return split_axis
    return np.array([0.0, 1.0, 0.0], dtype=np.float64)


def build_pose_from_frame(frame, tool_offset_mm=0.0, split_offset_mm=0.0):
    point_mm = np.asarray(frame["point_mm"], dtype=np.float64)
    tool_z_unit = np.asarray(frame["tool_z_unit"], dtype=np.float64)
    split_axis_unit = np.asarray(frame["split_axis_unit"], dtype=np.float64)
    pos = point_mm + tool_z_unit * float(tool_offset_mm) + split_axis_unit * float(split_offset_mm)
    base_pose = frame["base_pose"]
    return [
        float(pos[0]),
        float(pos[1]),
        float(pos[2]),
        float(base_pose[0]),
        float(base_pose[1]),
        float(base_pose[2]),
    ]


# ===================== 主流程类 =====================

class LastTimeDemo:
    """lasttime演示程序主类"""

    def __init__(self):
        self.detector = None
        self.tracker = None
        self.stabilizer = None
        self.smoother = None
        self.robot = None

        self.stable_depth_frame = None
        self.locked_color_frame = None
        self.spine_line = None
        self.meridian_lines = None
        self.outer_meridian_lines = None
        self.camera_origin_mm = None
        self.massage_pixels = []
        self.massage_frames = []
        self.massage_points_mm = []
        self.preview_thread = None
        self.preview_stop_event = threading.Event()
        self.preview_state_lock = threading.Lock()
        self.preview_status_text = "等待检测"
        self.preview_active_point_idx = None
        self.preview_spine_line = None
        self.preview_meridian_lines = None
        self.preview_outer_meridian_lines = None
        self.preview_tracking_label = "SEARCH"
        self.motion_thread = None
        self.motion_success = False
        self.motion_error = None
        self.force_enabled = bool(LASTTIME_FORCE_CONTROL)
        self.force_ready = False
        self.force_config = None
        self.motion_orientation = None
        self.force_contact_offset_mm = LASTTIME_FORCE_CONTACT_OFFSET_MM

        self.frame_idx = 0

    def init_vision(self):
        """初始化视觉系统（完整复用demo.py）"""
        print("初始化视觉系统...")

        finger_mm = 45.0
        self.detector = LinearMeridianDetector(finger_mm)
        self.smoother = PointSmoother(alpha=0.25, max_step_px=12.0)
        self.stabilizer = MeridianLineStabilizer()
        self.tracker = BackRegionCoTracker()

        if self.detector.camera_to_robot is None:
            raise RuntimeError("无法加载标定矩阵")

        print("视觉系统初始化完成")

    def update_preview_status(self, status_text, active_point_idx=None):
        with self.preview_state_lock:
            self.preview_status_text = str(status_text)
            self.preview_active_point_idx = active_point_idx

    def _set_preview_tracking_state(
        self,
        spine_line=None,
        meridian_lines=None,
        outer_meridian_lines=None,
        tracking_label="SEARCH",
    ):
        with self.preview_state_lock:
            self.preview_spine_line = spine_line
            self.preview_meridian_lines = meridian_lines
            self.preview_outer_meridian_lines = outer_meridian_lines
            self.preview_tracking_label = str(tracking_label)

    def _analyze_visual_frame(self, img):
        meridian_lines = None
        outer_meridian_lines = None
        spine_line = None
        tracker_result = self.tracker.last_result
        line_source = "none"

        pose_info = infer_best_pose_with_rotations(self.detector.model, img, conf=0.5)

        detected = False
        pose_conf = 0.0
        tracker_vis_ratio = 0.0

        if pose_info["kpts"] is not None:
            kpts = pose_info["kpts"]
            pose_conf = float(min(kpts[5][2], kpts[6][2], kpts[11][2], kpts[12][2]))

            if (kpts[5][2] > 0.5 and kpts[6][2] > 0.5 and
                kpts[11][2] > 0.3 and kpts[12][2] > 0.3):

                spine_seed = _build_spine_seed_from_torso(
                    kpts, self.smoother, self.detector.user_finger_width_mm
                )

                if spine_seed is not None:
                    detected = True
                    (neck_u, neck_v), (tail_u, tail_v) = spine_seed["spine_line"]
                    lateral_direction_2d = np.asarray(
                        spine_seed["lateral_direction_2d"], dtype=np.float32
                    )
                    body_offset_px = float(spine_seed["body_offset_px"])

                    if self.tracker.available:
                        if (self.frame_idx - self.tracker.last_seed_frame_idx
                            >= self.tracker.reseed_interval):
                            self.tracker.seed(
                                img, spine_seed["spine_line"],
                                lateral_direction_2d, body_offset_px, self.frame_idx
                            )
                        else:
                            self.tracker.update(img, self.frame_idx)

                    neck_l = (neck_u - lateral_direction_2d[0] * body_offset_px,
                             neck_v - lateral_direction_2d[1] * body_offset_px)
                    tail_l = (tail_u - lateral_direction_2d[0] * body_offset_px,
                             tail_v - lateral_direction_2d[1] * body_offset_px)
                    neck_r = (neck_u + lateral_direction_2d[0] * body_offset_px,
                             neck_v + lateral_direction_2d[1] * body_offset_px)
                    tail_r = (tail_u + lateral_direction_2d[0] * body_offset_px,
                             tail_v + lateral_direction_2d[1] * body_offset_px)

                    spine_line = ((neck_u, neck_v), (tail_u, tail_v))
                    meridian_lines = ((neck_l, tail_l), (neck_r, tail_r))
                    line_source = "pose"

        tracker_reliable = False
        if tracker_result is not None:
            tracker_vis_ratio = float(tracker_result.get("visible_ratio", 0.0))
            tracker_reliable = tracker_vis_ratio >= 0.35

            if tracker_result.get("spine_line") is not None:
                spine_line = tracker_result["spine_line"]

            if not detected and tracker_reliable:
                if tracker_result.get("meridian_lines") is not None:
                    meridian_lines = tracker_result["meridian_lines"]
                    line_source = "tracker"
            elif meridian_lines is None and tracker_reliable:
                if tracker_result.get("meridian_lines") is not None:
                    meridian_lines = tracker_result["meridian_lines"]
                    line_source = "tracker_fallback"

        stable_track = self.stabilizer.update(
            meridian_lines=meridian_lines,
            line_source=line_source,
            pose_conf=pose_conf,
            tracker_vis_ratio=tracker_vis_ratio,
        )

        meridian_lines = stable_track["meridian_lines"]
        visual_ready = bool(stable_track["ready"])
        visual_status = str(stable_track["status"])
        visual_motion_ready = visual_ready and visual_status == "stable"

        outer_meridian_lines = _expand_meridian_lines_from_spine(
            spine_line, meridian_lines, scale=2.0
        )

        return {
            "spine_line": spine_line,
            "meridian_lines": meridian_lines,
            "outer_meridian_lines": outer_meridian_lines,
            "visual_ready": visual_ready,
            "visual_status": visual_status,
            "visual_motion_ready": visual_motion_ready,
        }

    def _locked_analysis(self):
        return {
            "spine_line": self.spine_line,
            "meridian_lines": self.meridian_lines,
            "outer_meridian_lines": self.outer_meridian_lines,
            "visual_ready": True,
            "visual_status": "stable",
            "visual_motion_ready": True,
        }

    def _draw_preview_overlay(self, frame):
        img = frame.copy()
        active_idx = None
        status_text = ""
        tracking_label = "SEARCH"
        preview_meridian_lines = None
        preview_outer_meridian_lines = None
        with self.preview_state_lock:
            active_idx = self.preview_active_point_idx
            status_text = self.preview_status_text
            tracking_label = self.preview_tracking_label
            preview_meridian_lines = self.preview_meridian_lines
            preview_outer_meridian_lines = self.preview_outer_meridian_lines

        if preview_outer_meridian_lines:
            for line in preview_outer_meridian_lines:
                cv2.line(img, _pt(line[0]), _pt(line[1]), (255, 0, 255), 2)
        if preview_meridian_lines:
            for line in preview_meridian_lines:
                cv2.line(img, _pt(line[0]), _pt(line[1]), (0, 255, 0), 2)

        for idx, pixel in enumerate(self.massage_pixels):
            color = (0, 255, 255) if idx == active_idx else (0, 180, 255)
            radius = 6 if idx == active_idx else 4
            cv2.circle(img, _pt(pixel), radius, color, -1)
            cv2.putText(
                img,
                str(idx + 1),
                (int(pixel[0]) + 6, int(pixel[1]) - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )

        cv2.putText(
            img,
            f"Track: {tracking_label}",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 220, 0) if tracking_label == "LOCKED" else (0, 220, 255),
            2,
        )
        cv2.putText(
            img,
            f"Motion: {status_text}",
            (10, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 220, 255),
            2,
        )
        cv2.putText(
            img,
            "Robot path locked to first stable frame",
            (10, 86),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        return img

    def _draw_detection_overlay(self, img, analysis):
        img_display = img.copy()
        outer_meridian_lines = analysis["outer_meridian_lines"]
        meridian_lines = analysis["meridian_lines"]
        visual_status = analysis["visual_status"]
        visual_motion_ready = analysis["visual_motion_ready"]

        if outer_meridian_lines:
            for line in outer_meridian_lines:
                cv2.line(img_display, _pt(line[0]), _pt(line[1]), (255, 0, 255), 2)
        if meridian_lines:
            for line in meridian_lines:
                cv2.line(img_display, _pt(line[0]), _pt(line[1]), (0, 255, 0), 2)

        vision_label = "LOCKED" if visual_motion_ready else visual_status.upper()
        vision_color = (0, 220, 0) if visual_motion_ready else (0, 215, 255)
        cv2.putText(
            img_display,
            f"Vision: {vision_label}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            vision_color,
            2,
        )
        return img_display

    def _live_preview_loop(self):
        try:
            while not self.preview_stop_event.is_set():
                frames = self.detector.pipeline.wait_for_frames()
                frames = self.detector.align.process(frames)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                img = np.asanyarray(color_frame.get_data())
                self.frame_idx += 1
                analysis = self._analyze_visual_frame(img)
                tracking_label = (
                    "LOCKED" if analysis["visual_motion_ready"]
                    else str(analysis["visual_status"]).upper()
                )
                self._set_preview_tracking_state(
                    spine_line=analysis["spine_line"],
                    meridian_lines=analysis["meridian_lines"],
                    outer_meridian_lines=analysis["outer_meridian_lines"],
                    tracking_label=tracking_label,
                )
                preview = self._draw_preview_overlay(img)
                cv2.imshow(LIVE_PREVIEW_WINDOW_NAME, preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.preview_stop_event.set()
                    break
        except Exception as e:
            print(f"预览窗口线程退出: {e}")

    def _motion_worker(self):
        try:
            self.update_preview_status("连接机械臂")
            self.init_robot()
            self.motion_success = bool(self.execute_massage_sequence())
        except Exception as e:
            self.motion_error = e
            self.motion_success = False

    def run_live_preview_until_motion_done(self):
        preview_enabled = bool(ENABLE_LIVE_PREVIEW_WINDOW)
        fallback_frame = None
        if self.locked_color_frame is not None:
            fallback_frame = self.locked_color_frame.copy()
        done_status_set = False

        while True:
            try:
                frames = self.detector.pipeline.wait_for_frames()
                frames = self.detector.align.process(frames)
                color_frame = frames.get_color_frame()
                if color_frame:
                    img = np.asanyarray(color_frame.get_data())
                elif fallback_frame is not None:
                    img = fallback_frame.copy()
                else:
                    continue

                motion_alive = (
                    self.motion_thread is not None
                    and self.motion_thread.is_alive()
                )
                if not motion_alive and not done_status_set:
                    if self.motion_error is not None:
                        self.update_preview_status("动作异常，按q退出")
                    elif self.motion_success:
                        self.update_preview_status("动作完成，按q退出")
                    else:
                        self.update_preview_status("动作结束，按q退出")
                    done_status_set = True

                # Keep the camera image live, but keep meridian/trajectory overlay
                # locked to the first stable detection frame.
                analysis = self._locked_analysis()

                detection = self._draw_detection_overlay(img, analysis)
                cv2.imshow("Detection", detection)
                if preview_enabled:
                    preview = self._draw_preview_overlay(img)
                    cv2.imshow(LIVE_PREVIEW_WINDOW_NAME, preview)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    if preview_enabled:
                        try:
                            cv2.destroyWindow(LIVE_PREVIEW_WINDOW_NAME)
                        except Exception:
                            pass
                    try:
                        cv2.destroyWindow("Detection")
                    except Exception:
                        pass
                    break
                time.sleep(0.03)
            except Exception as e:
                print(f"预览刷新错误: {e}")
                time.sleep(0.1)

    def start_live_preview(self):
        if not ENABLE_LIVE_PREVIEW_WINDOW or self.detector is None:
            return
        if self.preview_thread is not None and self.preview_thread.is_alive():
            return
        self.preview_stop_event.clear()
        self.preview_thread = threading.Thread(
            target=self._live_preview_loop,
            name="lasttime_live_preview",
            daemon=True,
        )
        self.preview_thread.start()
        print(f"演示窗口已启动: {LIVE_PREVIEW_WINDOW_NAME}")

    def stop_live_preview(self):
        self.preview_stop_event.set()
        if self.preview_thread is not None:
            self.preview_thread.join(timeout=2.0)
            self.preview_thread = None
        try:
            cv2.destroyWindow(LIVE_PREVIEW_WINDOW_NAME)
        except Exception:
            pass

    def wait_for_stable_detection(self):
        """等待检测稳定（完整复用demo.py的检测流程）"""
        print(f"等待检测稳定（需要连续{STABLE_FRAMES}帧）...")
        timeout = 30
        start_time = time.time()

        visual_ready = False
        visual_status = "search"

        while time.time() - start_time < timeout:
            try:
                frames = self.detector.pipeline.wait_for_frames()
                frames = self.detector.align.process(frames)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                img = np.asanyarray(color_frame.get_data())
                self.frame_idx += 1

                analysis = self._analyze_visual_frame(img)
                spine_line = analysis["spine_line"]
                meridian_lines = analysis["meridian_lines"]
                outer_meridian_lines = analysis["outer_meridian_lines"]
                visual_status = analysis["visual_status"]
                visual_motion_ready = analysis["visual_motion_ready"]

                img_display = self._draw_detection_overlay(img, analysis)
                cv2.imshow("Detection", img_display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return False

                if visual_motion_ready:
                    print("检测稳定！")
                    try:
                        depth_frame.keep()
                    except Exception:
                        pass
                    self.stable_depth_frame = depth_frame
                    self.locked_color_frame = img.copy()
                    self.spine_line = spine_line
                    self.meridian_lines = meridian_lines
                    self.outer_meridian_lines = outer_meridian_lines
                    self._set_preview_tracking_state(
                        spine_line=spine_line,
                        meridian_lines=meridian_lines,
                        outer_meridian_lines=outer_meridian_lines,
                        tracking_label="LOCKED",
                    )
                    return True

            except Exception as e:
                print(f"检测错误: {e}")
                continue

        print("检测超时")
        return False

    def capture_trajectory(self):
        """捕获左外侧膀胱经轨迹"""
        print("捕获轨迹...")

        if self.outer_meridian_lines is None:
            raise RuntimeError("外侧膀胱经未检测到")

        # 获取左外侧膀胱经
        left_outer_line = self.outer_meridian_lines[0]

        # 采样10个点
        pixels = sample_line_pixels(left_outer_line[0], left_outer_line[1], SAMPLE_POINTS)
        print(f"采样了{len(pixels)}个像素点")

        valid_pixels = []
        points_cam = []
        for u, v in pixels:
            p3 = self.detector.get_point3d_from_depth(float(u), float(v), self.stable_depth_frame)
            if p3 is None:
                continue
            valid_pixels.append((float(u), float(v)))
            points_cam.append(np.asarray(p3, dtype=np.float64))

        print(f"转换为{len(points_cam)}个3D点")

        if len(points_cam) < SAMPLE_POINTS // 2:
            raise RuntimeError(f"深度数据不足，只获取到{len(points_cam)}个有效点")

        # 转换为机械臂坐标
        points_robot = self.detector.transform_points_to_robot([p.tolist() for p in points_cam])
        print(f"转换为机械臂坐标：{len(points_robot)}个点")

        points_robot_mm = [
            np.asarray([float(p[0] * 1000.0), float(p[1] * 1000.0), float(p[2] * 1000.0)], dtype=np.float64)
            for p in points_robot
        ]
        self.camera_origin_mm = _camera_origin_mm_from_matrix(self.detector.camera_to_robot)

        tool_z_units = []
        normal_ok_count = 0
        for idx, (pixel, point_robot_mm) in enumerate(zip(valid_pixels, points_robot_mm)):
            normal_cam = _estimate_patch_plane_normal_camera(
                self.detector,
                self.stable_depth_frame,
                pixel[0],
                pixel[1],
            )
            if normal_cam is None:
                tool_z_units.append(_fallback_tool_z_axis(point_robot_mm, self.camera_origin_mm))
                continue

            outward_robot = _transform_vector_camera_to_robot(self.detector.camera_to_robot, normal_cam)
            if outward_robot is None:
                tool_z_units.append(_fallback_tool_z_axis(point_robot_mm, self.camera_origin_mm))
                continue

            if self.camera_origin_mm is not None:
                to_camera = np.asarray(self.camera_origin_mm, dtype=np.float64) - point_robot_mm
                if float(np.dot(outward_robot, to_camera)) < 0.0:
                    outward_robot = -outward_robot

            tool_z_units.append(-outward_robot)
            normal_ok_count += 1

        tool_z_units = _smooth_unit_vectors(tool_z_units)

        frames = []
        for idx, point_robot_mm in enumerate(points_robot_mm):
            if len(points_robot_mm) == 1:
                tangent = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            else:
                prev_pt = points_robot_mm[max(0, idx - 1)]
                next_pt = points_robot_mm[min(len(points_robot_mm) - 1, idx + 1)]
                tangent = _normalize_vec(next_pt - prev_pt)
                if tangent is None:
                    tangent = np.array([0.0, 1.0, 0.0], dtype=np.float64)

            tool_z_unit = np.asarray(tool_z_units[idx], dtype=np.float64)
            split_axis_unit = _build_split_axis(tool_z_unit, tangent)
            if frames and float(np.dot(split_axis_unit, np.asarray(frames[-1]["split_axis_unit"], dtype=np.float64))) < 0.0:
                split_axis_unit = -split_axis_unit

            base_pose = tuple(
                float(v)
                for v in _rpy_from_tool_z_vector(tool_z_unit, INIT_POSE_P24[5], INIT_POSE_P24[3])
            )
            frames.append(
                {
                    "index": idx,
                    "pixel": valid_pixels[idx],
                    "point_mm": point_robot_mm.tolist(),
                    "tool_z_unit": tool_z_unit.tolist(),
                    "split_axis_unit": split_axis_unit.tolist(),
                    "base_pose": base_pose,
                }
            )

        self.massage_pixels = valid_pixels
        self.massage_frames = frames
        self.massage_points_mm = [frame["point_mm"] for frame in frames]

        print(f"生成了{len(self.massage_points_mm)}个按摩点")
        print(f"局部平面法向成功估计：{normal_ok_count}/{len(self.massage_points_mm)}")
        return True

    def init_robot(self):
        """初始化机械臂（简化版，参考dianjing.py）"""
        print(f"连接机械臂 {ROBOT_IP}...")
        self.robot = Robot.RPC(ROBOT_IP)
        self._recover_robot_ready("初始化")
        self.robot.SetSpeed(MOVE_VEL_FAST)
        cur = self._read_current_tcp_pose()
        if cur is not None:
            self.motion_orientation = [float(cur[3]), float(cur[4]), float(cur[5])]
        print("机械臂初始化完成")

    def _recover_robot_ready(self, reason=""):
        if self.robot is None:
            return False
        prefix = f"[Robot] 恢复准备状态({reason})" if reason else "[Robot] 恢复准备状态"
        try:
            err_before = self.robot.GetRobotErrorCode()
            print(f"{prefix}: error_before={err_before}")
        except Exception as e:
            print(f"{prefix}: 读取错误码失败: {e}")
            err_before = None

        for label, fn in (
            ("Mode(0)", lambda: self.robot.Mode(0)),
            ("StopMotion()", lambda: self.robot.StopMotion()),
            ("ResetAllError()", lambda: self.robot.ResetAllError()),
            ("RobotEnable(1)", lambda: self.robot.RobotEnable(1)),
            ("SetSpeed", lambda: self.robot.SetSpeed(MOVE_VEL_FAST)),
        ):
            try:
                ret = fn()
                if ret != 0:
                    print(f"{prefix}: {label} 返回 {ret}")
            except Exception as e:
                print(f"{prefix}: {label} 异常: {e}")

        try:
            err_after = self.robot.GetRobotErrorCode()
            print(f"{prefix}: error_after={err_after}")
            if isinstance(err_after, tuple) and err_after[0] == 0:
                codes = err_after[1]
                return len(codes) >= 2 and int(codes[0]) == 0 and int(codes[1]) == 0
        except Exception as e:
            print(f"{prefix}: 复查错误码失败: {e}")
        return True

    def _read_current_tcp_pose(self):
        if self.robot is None:
            return None
        ret = self.robot.GetActualTCPPose()
        if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
            return [float(v) for v in ret[1][:6]]
        return None

    def _fmt_pose(self, pose):
        return "[" + ", ".join(f"{float(v):.3f}" for v in pose) + "]"

    def _move_pose(self, target_pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=False):
        if not segmented:
            ret = self.robot.MoveCart(desc_pos=list(target_pose), tool=0, user=0, vel=vel, blendT=blendT)
            if ret == 0:
                return True
            if ret == 14:
                self._recover_robot_ready("MoveCart err=14")
                ret = self.robot.MoveCart(desc_pos=list(target_pose), tool=0, user=0, vel=vel, blendT=blendT)
                if ret == 0:
                    return True
            if ret not in (14, 112):
                print(f"    MoveCart失败 err={ret}, target={self._fmt_pose(target_pose)}")
                return False

        cur = self._read_current_tcp_pose()
        if cur is None:
            return False
        for _ in range(120):
            dx = float(target_pose[0]) - cur[0]
            dy = float(target_pose[1]) - cur[1]
            dz = float(target_pose[2]) - cur[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < 0.5:
                ret = self.robot.MoveCart(desc_pos=list(target_pose), tool=0, user=0, vel=vel, blendT=blendT)
                if ret != 0:
                    print(f"    MoveCart分段末点失败 err={ret}, target={self._fmt_pose(target_pose)}")
                return ret == 0
            scale = min(1.0, 5.0 / max(dist, 1e-6))
            wp = [
                cur[0] + dx * scale,
                cur[1] + dy * scale,
                cur[2] + dz * scale,
                float(target_pose[3]),
                float(target_pose[4]),
                float(target_pose[5]),
            ]
            ret = self.robot.MoveCart(desc_pos=wp, tool=0, user=0, vel=vel, blendT=blendT)
            if ret != 0:
                if ret == 14:
                    self._recover_robot_ready("MoveCart分段 err=14")
                    ret = self.robot.MoveCart(desc_pos=wp, tool=0, user=0, vel=vel, blendT=blendT)
                    if ret == 0:
                        cur = self._read_current_tcp_pose()
                        if cur is None:
                            return False
                        continue
                print(f"    MoveCart分段失败 err={ret}, target={self._fmt_pose(wp)}")
                return False
            cur = self._read_current_tcp_pose()
            if cur is None:
                return False
        return False

    def _build_session_safe_pose(self):
        cur = self._read_current_tcp_pose()
        if cur is None:
            safe_pose = INIT_POSE_P24.copy()
            safe_pose[2] = LASTTIME_SAFE_Z_MM
            return safe_pose, True
        safe_pose = [float(v) for v in cur]
        self.motion_orientation = [safe_pose[3], safe_pose[4], safe_pose[5]]
        if safe_pose[2] < LASTTIME_SAFE_Z_MM:
            safe_pose[2] = LASTTIME_SAFE_Z_MM
            print(f"当前 TCP Z={cur[2]:.1f}mm，先竖直抬升到 Z={LASTTIME_SAFE_Z_MM:.1f}mm")
            return safe_pose, True
        print(f"当前 TCP 已在安全高度 Z={safe_pose[2]:.1f}mm，跳过旧 P24 固定安全位")
        return safe_pose, False

    def init_force_control(self):
        if not self.force_enabled or self.force_ready:
            return True
        if self.robot is None:
            raise RuntimeError("机械臂未连接，无法初始化力传感器")

        cfg = ForceControlConfig()
        cfg.target_force_z = abs(float(LASTTIME_FORCE_N))
        cfg.enable_collision_guard = False
        cfg.software_force_limit = max(
            abs(float(LASTTIME_FORCE_SOFTWARE_LIMIT_N)),
            abs(float(LASTTIME_FORCE_N)) + 8.0,
        )
        self.force_config = cfg

        print("\n[Force] 初始化 SDK 软件PI恒力控制...")
        print("[Force] 校零要求：末端必须悬空、无外部接触。")
        if not init_force_sensor(self.robot, cfg):
            raise RuntimeError("力传感器初始化失败")
        print(
            f"[Force] 软件PI恒力: target={cfg.target_force_z:.1f}N, "
            f"limit={cfg.software_force_limit:.1f}N, guard=off"
        )
        self._recover_robot_ready("力传感器初始化后")
        self.force_ready = True
        return True

    def _pose_from_frame_offset(self, frame, offset_mm, split_offset_mm=0.0):
        pose = build_pose_from_frame(
            frame,
            tool_offset_mm=float(offset_mm),
            split_offset_mm=float(split_offset_mm),
        )
        if LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION and self.motion_orientation is not None:
            pose[3], pose[4], pose[5] = self.motion_orientation
        return pose

    def _read_force_z(self):
        if not self.force_ready:
            return None
        fz = get_force_z(self.robot)
        return None if fz is None else float(fz)

    def _check_force_limit(self, context):
        if not self.force_ready:
            return
        fz = self._read_force_z()
        if fz is None:
            raise RuntimeError(f"{context}: 读取 Fz 失败")
        if abs(fz) > self.force_config.software_force_limit:
            try:
                self.robot.StopMotion()
            except Exception:
                pass
            raise RuntimeError(f"{context}: 力超限 |Fz|={abs(fz):.1f}N")

    def _trim_force_at(self, frame, split_offset_mm=0.0, start_offset=None, max_iter=35, tol_n=LASTTIME_FORCE_TOL_N):
        if not self.force_ready:
            return start_offset if start_offset is not None else LASTTIME_FORCE_CONTACT_OFFSET_MM, False

        offset = self.force_contact_offset_mm if start_offset is None else float(start_offset)
        ctrl_err_i = 0.0
        target_abs = abs(float(LASTTIME_FORCE_N))
        target_fz = -target_abs
        ok = False

        for _ in range(max_iter):
            fz = self._read_force_z()
            if fz is not None:
                if abs(abs(fz) - target_abs) <= float(tol_n) and abs(fz) >= target_abs * 0.75:
                    ok = True
                    break
                err = target_fz - fz
                ctrl_err_i = max(-LASTTIME_FORCE_IMAX, min(LASTTIME_FORCE_IMAX, ctrl_err_i + err))
                delta = -(LASTTIME_FORCE_KP * err + LASTTIME_FORCE_KI * ctrl_err_i)
                delta = max(-LASTTIME_FORCE_MAX_STEP_MM, min(LASTTIME_FORCE_MAX_STEP_MM, delta))
                offset = max(
                    -float(HOVER_HEIGHT_MM),
                    min(float(LASTTIME_FORCE_PRESS_LIMIT_MM), offset + delta),
                )

            pose = self._pose_from_frame_offset(frame, offset, split_offset_mm)
            if not self._move_pose(pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
                return offset, False
            time.sleep(LASTTIME_FORCE_SETTLE_S)
            self._check_force_limit("软件PI恒力")

        fz = self._read_force_z()
        fz_text = "?" if fz is None else f"{fz:.1f}"
        print(f"[Force] offset={offset:+.1f}mm Fz={fz_text}N {'OK' if ok else 'soft'}")
        self.force_contact_offset_mm = offset
        return offset, ok

    def execute_dian_jin(self, frame):
        """执行点筋动作"""
        self.update_preview_status("点筋", frame.get("index"))
        hover_pose = build_pose_from_frame(frame, tool_offset_mm=-HOVER_HEIGHT_MM)
        if LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION and self.motion_orientation is not None:
            hover_pose[3], hover_pose[4], hover_pose[5] = self.motion_orientation
        dian_jin_pose = build_pose_from_frame(
            frame,
            tool_offset_mm=-(HOVER_HEIGHT_MM - DIAN_JIN_DEPTH_MM),
        )

        if self.force_ready:
            if not self._move_pose(hover_pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
                print("    警告：点筋悬空位失败")
                return False
            _, ok = self._trim_force_at(frame, 0.0, self.force_contact_offset_mm, max_iter=38)
            time.sleep(0.35)
            self._check_force_limit("点筋")
            if not ok:
                print("    警告：点筋未完全收敛到目标力，继续按软件限幅执行")
        else:
            ret = self.robot.MoveCart(desc_pos=dian_jin_pose, tool=0, user=0,
                                       vel=MOVE_VEL_SLOW, blendT=0.0)
            if ret != 0:
                print(f"    警告：点筋失败 (err={ret})")
                return False
            time.sleep(0.5)

        if not self._move_pose(hover_pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
            print("    警告：回到悬空位失败")
            return False
        return True

    def execute_fen_jin(self, frame):
        """执行分筋动作"""
        self.update_preview_status("分筋", frame.get("index"))
        hover_pose = build_pose_from_frame(frame, tool_offset_mm=-HOVER_HEIGHT_MM)
        if LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION and self.motion_orientation is not None:
            hover_pose[3], hover_pose[4], hover_pose[5] = self.motion_orientation
        positive_pose = build_pose_from_frame(
            frame,
            tool_offset_mm=-HOVER_HEIGHT_MM,
            split_offset_mm=FEN_JIN_LATERAL_MM,
        )
        negative_pose = build_pose_from_frame(
            frame,
            tool_offset_mm=-HOVER_HEIGHT_MM,
            split_offset_mm=-FEN_JIN_LATERAL_MM,
        )

        if self.force_ready:
            if not self._move_pose(hover_pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
                print("    警告：分筋悬空位失败")
                return False
            offset, _ = self._trim_force_at(frame, 0.0, self.force_contact_offset_mm, max_iter=24)
            for label, lateral_shift in (("分筋偏移+", FEN_JIN_LATERAL_MM), ("分筋偏移-", -FEN_JIN_LATERAL_MM), ("分筋回中心", 0.0)):
                pose = self._pose_from_frame_offset(frame, offset, lateral_shift)
                if not self._move_pose(pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
                    print(f"    警告：{label}失败")
                    return False
                offset, _ = self._trim_force_at(frame, lateral_shift, offset, max_iter=8, tol_n=4.0)
                time.sleep(0.15)
            self.force_contact_offset_mm = offset
            if not self._move_pose(hover_pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
                print("    警告：分筋回悬空位失败")
                return False
            return True

        ret = self.robot.MoveCart(desc_pos=positive_pose, tool=0, user=0,
                                  vel=MOVE_VEL_SLOW, blendT=0.0)
        if ret != 0:
            print(f"    警告：分筋偏移+失败 (err={ret})")
            return False
        time.sleep(0.3)
        ret = self.robot.MoveCart(desc_pos=negative_pose, tool=0, user=0,
                                  vel=MOVE_VEL_SLOW, blendT=0.0)
        if ret != 0:
            print(f"    警告：分筋偏移-失败 (err={ret})")
            return False
        time.sleep(0.3)
        ret = self.robot.MoveCart(desc_pos=hover_pose, tool=0, user=0,
                                  vel=MOVE_VEL_SLOW, blendT=0.0)
        if ret != 0:
            print(f"    警告：分筋回悬空位失败 (err={ret})")
            return False
        time.sleep(0.3)
        return True

    def execute_shun_jin(self):
        """执行顺筋动作"""
        print("顺筋动作...")
        if self.force_ready:
            if not self.massage_frames:
                return False
            start_frame = self.massage_frames[0]
            hover_pose = self._pose_from_frame_offset(start_frame, -HOVER_HEIGHT_MM)
            if not self._move_pose(hover_pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
                print("    警告：顺筋起始悬空位失败")
                return False
            offset, _ = self._trim_force_at(start_frame, 0.0, self.force_contact_offset_mm, max_iter=35)
            for i, frame in enumerate(self.massage_frames):
                self.update_preview_status("顺筋", i)
                print(f"  移动到点{i}...")
                pose = self._pose_from_frame_offset(frame, offset)
                if not self._move_pose(pose, vel=max(2, MOVE_VEL_SLOW // 2), blendT=BLEND_BLOCKING, segmented=True):
                    print(f"    警告：移动失败 (idx={i})")
                    return False
                if i == 0 or i == len(self.massage_frames) - 1 or i % 3 == 0:
                    offset, _ = self._trim_force_at(frame, 0.0, offset, max_iter=8, tol_n=4.0)
                self._check_force_limit("顺筋")
                time.sleep(0.03)
            end_hover = self._pose_from_frame_offset(self.massage_frames[-1], -HOVER_HEIGHT_MM)
            if not self._move_pose(end_hover, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
                print("    警告：顺筋结束回悬空位失败")
                return False
            return True

        for i, frame in enumerate(self.massage_frames):
            self.update_preview_status("顺筋", i)
            print(f"  移动到点{i}...")
            pose = build_pose_from_frame(frame, tool_offset_mm=-HOVER_HEIGHT_MM)
            ret = self.robot.MoveCart(desc_pos=pose, tool=0, user=0,
                                      vel=MOVE_VEL_SLOW, blendT=0.0)
            if ret != 0:
                print(f"    警告：移动失败 (err={ret})")
                return False
        return True

    def execute_massage_sequence(self):
        """执行完整按摩序列"""
        print("\n开始执行按摩序列...")

        try:
            # 移动到安全高度
            self.update_preview_status("移动到安全高度")
            print("移动到安全高度...")
            safe_pose, should_move_to_safe = self._build_session_safe_pose()
            if should_move_to_safe and not self._move_pose(safe_pose, vel=MOVE_VEL_FAST, blendT=BLEND_BLOCKING, segmented=True):
                print("警告：移动到安全高度失败")

            if self.force_enabled:
                self.init_force_control()

            # 移动到第一个点
            self.update_preview_status("移动到起始位置", 0)
            print("移动到起始位置...")
            first_frame = self.massage_frames[0]
            first_pose = build_pose_from_frame(first_frame, tool_offset_mm=-HOVER_HEIGHT_MM)
            if LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION and self.motion_orientation is not None:
                first_pose[3], first_pose[4], first_pose[5] = self.motion_orientation
            if not self._move_pose(first_pose, vel=MOVE_VEL_FAST, blendT=BLEND_BLOCKING, segmented=True):
                print("警告：移动到起始位置失败")

            # 执行点筋+分筋
            print("\n执行点筋+分筋动作...")
            for i, frame in enumerate(self.massage_frames):
                self.update_preview_status("到达悬空位", i)
                print(f"\n处理点 {i+1}/{len(self.massage_points_mm)}...")

                hover_pose = build_pose_from_frame(frame, tool_offset_mm=-HOVER_HEIGHT_MM)
                if LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION and self.motion_orientation is not None:
                    hover_pose[3], hover_pose[4], hover_pose[5] = self.motion_orientation
                if not self._move_pose(hover_pose, vel=MOVE_VEL_SLOW, blendT=BLEND_BLOCKING, segmented=True):
                    print("  警告：移动到悬空位失败")
                    continue

                print(f"  点筋...")
                self.execute_dian_jin(frame)

                print(f"  分筋...")
                self.execute_fen_jin(frame)

            # 回到起点
            self.update_preview_status("回到起点", 0)
            print("\n回到起点...")
            if not self._move_pose(first_pose, vel=MOVE_VEL_FAST, blendT=BLEND_BLOCKING, segmented=True):
                print("警告：回到起点失败")

            # 顺筋
            print("\n执行顺筋动作...")
            self.execute_shun_jin()

            # 返回安全位置
            self.update_preview_status("返回安全位置")
            print("\n返回安全位置...")
            if not self._move_pose(safe_pose, vel=MOVE_VEL_FAST, blendT=BLEND_BLOCKING, segmented=True):
                print("警告：返回安全位置失败")

            print("\n按摩序列执行完成！")
            return True

        except Exception as e:
            print(f"\n错误：{e}")
            return False

    def run(self):
        """主循环"""
        try:
            self.init_vision()

            if not self.wait_for_stable_detection():
                return False

            if not self.capture_trajectory():
                return False
            self.update_preview_status("首帧轨迹已锁定", 0)
            self.motion_success = False
            self.motion_error = None

            if ENABLE_LIVE_PREVIEW_WINDOW:
                self.motion_thread = threading.Thread(
                    target=self._motion_worker,
                    name="lasttime_motion_worker",
                    daemon=True,
                )
                self.motion_thread.start()
                self.run_live_preview_until_motion_done()
                self.motion_thread.join()
                self.motion_thread = None
                if self.motion_error is not None:
                    raise self.motion_error
                return bool(self.motion_success)

            self.init_robot()
            return self.execute_massage_sequence()

        except KeyboardInterrupt:
            print("\n用户中断")
            return False
        except Exception as e:
            print(f"\n错误：{e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            print("\n清理资源...")
            if self.motion_thread is not None and self.motion_thread.is_alive():
                self.motion_thread.join()
                self.motion_thread = None
            self.stop_live_preview()
            if self.detector and self.detector.pipeline:
                try:
                    self.detector.pipeline.stop()
                    print("相机已关闭")
                except Exception as e:
                    print(f"关闭相机失败: {e}")

            if self.robot:
                try:
                    if self.force_ready:
                        try:
                            self.robot.StopMotion()
                        except Exception:
                            pass
                        try:
                            self.robot.FT_Activate(0)
                            print("力传感器已复位")
                        except Exception as e:
                            print(f"力传感器复位失败: {e}")
                    self.robot.CloseRPC()
                    print("机械臂连接已关闭")
                except Exception as e:
                    print(f"关闭机械臂连接失败: {e}")

            cv2.destroyAllWindows()
            print("清理完成")


# ===================== 主入口 =====================

def main():
    print("=" * 60)
    print("lasttime.py - 膀胱经按摩动作演示")
    print("=" * 60)
    print()
    print("配置参数：")
    print(f"  机械臂IP: {ROBOT_IP}")
    print(f"  悬空高度: {HOVER_HEIGHT_MM}mm")
    print(f"  点筋深度: {DIAN_JIN_DEPTH_MM}mm")
    print(f"  分筋偏移: {FEN_JIN_LATERAL_MM}mm")
    print(f"  采样点数: {SAMPLE_POINTS}")
    print("  末端姿态: 局部深度平面法向")
    print(
        f"  平面拟合: radius={PLANE_FIT_RADIUS_PX}px "
        f"step={PLANE_FIT_STEP_PX}px min_pts={PLANE_FIT_MIN_POINTS}"
    )
    print(f"  演示预览窗口: {'开启（实时跟踪，仅展示）' if ENABLE_LIVE_PREVIEW_WINDOW else '关闭'}")
    print(f"  实时状态监控: {'开启' if USE_REALTIME_STATE else '关闭（RPC-only）'}")
    print(
        f"  SDK软件PI恒力: {'开启' if LASTTIME_FORCE_CONTROL else '关闭'} "
        f"target={LASTTIME_FORCE_N:.1f}N limit={LASTTIME_FORCE_SOFTWARE_LIMIT_N:.1f}N"
    )
    if LASTTIME_FORCE_CONTROL:
        print(
            f"  恒力参数: contact_offset={LASTTIME_FORCE_CONTACT_OFFSET_MM:.1f}mm "
            f"press_limit={LASTTIME_FORCE_PRESS_LIMIT_MM:.1f}mm "
            f"KP={LASTTIME_FORCE_KP:.3f} KI={LASTTIME_FORCE_KI:.3f} "
            f"姿态={'保持当前TCP' if LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION else '局部法向'}"
        )
    print(f"  机械臂SDK: {FAIRINO_SDK_ROOT}")
    print(f"  SDK模块文件: {FAIRINO_SDK_MODULE}")
    print()

    demo = LastTimeDemo()
    success = demo.run()

    if success:
        print("\n演示完成！")
        return 0
    else:
        print("\n演示失败")
        return 1


if __name__ == "__main__":
    sys.exit(main())
