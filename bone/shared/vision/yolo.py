import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import torch
import json
import os
from datetime import datetime

# ================= 配置区域 =================
# 用户手指宽度默认值（物理偏移量，单位：毫米，可运行时输入覆盖）
USER_FINGER_WIDTH_MM = 45.0
# 使用更高精度的姿态权重（比 s 版更准但更重）
MODEL_PATH = 'yolo11l-pose.pt'

# 深度单位换算（RealSense z16 常用比例：原始值 * 0.001 = 米）
DEPTH_SCALE = 0.001

# 默认肩宽（毫米），用于无深度的静态图片模式估算尺度
ESTIMATED_SHOULDER_MM = 360.0
# 检测结果导出目录（给机械臂输入）
ROBOT_OUTPUT_DIR = "robot_meridian_output"
# 膀胱经采样点数量
MERIDIAN_SAMPLE_POINTS = 30
# 相机到机械臂基座的标定矩阵文件（可选）
CAMERA_TO_ROBOT_FILE = "camera_to_robot.json"
# 深度取样窗口半径：2 表示 5x5 邻域中值
DEPTH_MEDIAN_RADIUS = 2
# 尝试多种朝向做姿态推理，提升侧躺/倒置人体检测率
POSE_ROTATION_MODES = ("none", "cw90", "ccw90", "180")
REQUIRED_KEYPOINTS = (5, 6, 11, 12)
EMA_ALPHA = 0.25
EMA_MAX_STEP_PX = 12.0
EMA_MISS_TOLERANCE = 5
MIN_VISUAL_OFFSET_PX = 12.0
# ===========================================


def default_intrinsics(width, height):
    """
    若无真实内参，用 D435i 640x480 典型内参按分辨率线性缩放兜底。
    """
    base_w, base_h = 640.0, 480.0
    scale_w, scale_h = width / base_w, height / base_h
    intr = rs.intrinsics()
    intr.width, intr.height = width, height
    intr.ppx, intr.ppy = 318.9 * scale_w, 241.0 * scale_h
    intr.fx, intr.fy = 616.3 * scale_w, 616.5 * scale_h
    intr.model = rs.distortion.none
    intr.coeffs = [0, 0, 0, 0, 0]
    return intr


def load_camera_to_robot_matrix(path=CAMERA_TO_ROBOT_FILE):
    """
    读取 4x4 标定矩阵（相机坐标 -> 机械臂基座坐标）。
    文件格式:
    {
      "matrix": [[...4列], [...], [...], [0,0,0,1]]
    }
    """
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        m = np.array(data.get("matrix", []), dtype=np.float64)
        if m.shape != (4, 4):
            return None
        return m
    except Exception:
        return None


def rotate_image_for_pose(img, mode):
    if mode == "none":
        return img
    if mode == "cw90":
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if mode == "ccw90":
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if mode == "180":
        return cv2.rotate(img, cv2.ROTATE_180)
    raise ValueError(f"Unsupported rotation mode: {mode}")


def map_keypoints_to_original(kpts, orig_w, orig_h, mode):
    mapped = kpts.copy()
    if mode == "none":
        return mapped

    for point in mapped:
        x, y = float(point[0]), float(point[1])
        if mode == "cw90":
            point[0] = y
            point[1] = orig_h - 1 - x
        elif mode == "ccw90":
            point[0] = orig_w - 1 - y
            point[1] = x
        elif mode == "180":
            point[0] = orig_w - 1 - x
            point[1] = orig_h - 1 - y
        else:
            raise ValueError(f"Unsupported rotation mode: {mode}")
    return mapped


def pose_candidate_score(kpts):
    if kpts is None or len(kpts) <= max(REQUIRED_KEYPOINTS):
        return -1.0
    return float(sum(float(kpts[idx][2]) for idx in REQUIRED_KEYPOINTS))


def extract_best_pose_keypoints(result):
    if result.keypoints is None or len(result.keypoints.data) == 0:
        return None, -1.0

    best_kpts = None
    best_score = -1.0
    for person_kpts in result.keypoints.data.cpu().numpy():
        score = pose_candidate_score(person_kpts)
        if score > best_score:
            best_score = score
            best_kpts = person_kpts
    return best_kpts, best_score


def infer_best_pose_with_rotations(model, img, conf=0.5):
    """
    对 0/90/180/270 四个朝向做姿态推理，返回映射回原图坐标系后的最佳关键点。
    这样侧躺、倒置的人体也能被检出。
    """
    orig_h, orig_w = img.shape[:2]
    best = {
        "kpts": None,
        "score": -1.0,
        "rotation": "none",
    }

    for mode in POSE_ROTATION_MODES:
        rotated_img = rotate_image_for_pose(img, mode)
        results = model(rotated_img, verbose=False, conf=conf)
        kpts, score = extract_best_pose_keypoints(results[0])
        if kpts is None:
            continue

        mapped_kpts = map_keypoints_to_original(kpts, orig_w, orig_h, mode)
        if score > best["score"]:
            best["kpts"] = mapped_kpts
            best["score"] = score
            best["rotation"] = mode

    return best



def normalize_vector(vec, eps=1e-6):
    arr = np.asarray(vec, dtype=np.float64)
    norm = float(np.linalg.norm(arr))
    if norm <= eps:
        return None
    return arr / norm


def get_body_lateral_direction_2d(kpts):
    for left_idx, right_idx in ((5, 6), (11, 12)):
        if kpts[left_idx][2] > 0.3 and kpts[right_idx][2] > 0.3:
            direction = normalize_vector([
                float(kpts[right_idx][0] - kpts[left_idx][0]),
                float(kpts[right_idx][1] - kpts[left_idx][1]),
            ])
            if direction is not None:
                return direction
    return np.array([1.0, 0.0], dtype=np.float64)



class PointSmoother:
    def __init__(self, alpha=EMA_ALPHA, miss_tolerance=EMA_MISS_TOLERANCE, max_step_px=EMA_MAX_STEP_PX):
        self.alpha = alpha
        self.miss_tolerance = miss_tolerance
        self.max_step_px = max_step_px
        self._state = {}
        self._miss_count = 0

    def smooth(self, name, x, y, round_result=False):
        self._miss_count = 0
        x = float(x)
        y = float(y)
        if name not in self._state:
            self._state[name] = (x, y)
        else:
            ox, oy = self._state[name]
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
        return float(sx), float(sy)

    def miss(self):
        self._miss_count += 1
        if self._miss_count >= self.miss_tolerance:
            self.reset()

    def reset(self):
        self._state.clear()
        self._miss_count = 0

class LinearMeridianDetector:
    def __init__(self, finger_width_mm=USER_FINGER_WIDTH_MM):
        # 1. 初始化 RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color) # 深度对齐
        # 深度已对齐到彩色相机空间，必须使用彩色流内参做反投影
        # （D435i 深度/彩色 FOV 不同，用错内参会导致 3D 坐标系统性偏差）
        self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        # 2. 初始化 YOLO
        # 优先使用 GPU（如可用），否则回落 CPU
        self.model = YOLO(MODEL_PATH)
        if torch.cuda.is_available():
            self.model.to('cuda')
        self.user_finger_width_mm = finger_width_mm  # 运行时输入的手指宽度（毫米）
        self.depth_warned = False  # 深度可用性提示开关
        self.smoother = PointSmoother()
        device = self.model.device if hasattr(self.model, "device") else "unknown"
        print(f"模式：直线检测 | 当前偏移: {self.user_finger_width_mm}mm | 推理设备: {device}")
        self.camera_to_robot = load_camera_to_robot_matrix()
        if self.camera_to_robot is not None:
            print(f"已加载标定矩阵: {CAMERA_TO_ROBOT_FILE}")
        else:
            print(f"未检测到标定矩阵 {CAMERA_TO_ROBOT_FILE}，将仅导出相机坐标轨迹")

    def get_stable_depth(self, depth_frame, pixel_u, pixel_v, radius=DEPTH_MEDIAN_RADIUS):
        """
        在像素邻域内取中值深度，减少单像素噪声和空洞导致的抖动。
        返回米，失败返回 0.0。
        """
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        cu = int(round(pixel_u))
        cv = int(round(pixel_v))
        if cu < 0 or cu >= width or cv < 0 or cv >= height:
            return 0.0

        depths = []
        for du in range(-radius, radius + 1):
            for dv in range(-radius, radius + 1):
                pu = cu + du
                pv = cv + dv
                if 0 <= pu < width and 0 <= pv < height:
                    d = depth_frame.get_distance(pu, pv)
                    if d > 0.1:
                        depths.append(float(d))
        if not depths:
            return 0.0
        return float(np.median(depths))

    def get_point3d_from_depth(self, pixel_u, pixel_v, depth_frame):
        if pixel_u < 0 or pixel_u >= 640 or pixel_v < 0 or pixel_v >= 480:
            return None
        dist = self.get_stable_depth(depth_frame, pixel_u, pixel_v)
        if dist <= 0:
            return None
        p_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [pixel_u, pixel_v], dist)
        return np.array(p_3d, dtype=np.float64)

    def get_body_lateral_direction_3d(self, kpts, depth_frame):
        directions = []
        for left_idx, right_idx in ((5, 6), (11, 12)):
            left_pt = self.get_point3d_from_depth(float(kpts[left_idx][0]), float(kpts[left_idx][1]), depth_frame)
            right_pt = self.get_point3d_from_depth(float(kpts[right_idx][0]), float(kpts[right_idx][1]), depth_frame)
            if left_pt is None or right_pt is None:
                continue
            direction = normalize_vector(right_pt - left_pt)
            if direction is not None:
                directions.append(direction)

        if not directions:
            return np.array([1.0, 0.0, 0.0], dtype=np.float64)

        merged = normalize_vector(np.mean(directions, axis=0))
        if merged is None:
            return np.array([1.0, 0.0, 0.0], dtype=np.float64)
        return merged

    def get_offset_points_2d(self, pixel_u, pixel_v, depth_frame, offset_mm, lateral_direction_3d=None):
        """
        输入一个点的像素坐标，按人体左右方向做物理偏移，返回左右偏移后的两个点的像素坐标。
        """
        p_3d = self.get_point3d_from_depth(pixel_u, pixel_v, depth_frame)
        if p_3d is None:
            return None, None

        offset_m = offset_mm / 1000.0
        direction = normalize_vector(lateral_direction_3d if lateral_direction_3d is not None else [1.0, 0.0, 0.0])
        if direction is None:
            direction = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        p_left_3d = (p_3d - direction * offset_m).tolist()
        p_right_3d = (p_3d + direction * offset_m).tolist()

        p_left_2d = rs.rs2_project_point_to_pixel(self.intrinsics, p_left_3d)
        p_right_2d = rs.rs2_project_point_to_pixel(self.intrinsics, p_right_3d)

        return (float(p_left_2d[0]), float(p_left_2d[1])), (float(p_right_2d[0]), float(p_right_2d[1]))

    @staticmethod
    def to_pixel_point(point):
        if point is None:
            return None
        return int(round(point[0])), int(round(point[1]))

    def sample_line_pixels(self, p1, p2, num_points=MERIDIAN_SAMPLE_POINTS):
        pts = []
        for i in range(num_points + 1):
            t = i / num_points
            u = int(round(p1[0] * (1 - t) + p2[0] * t))
            v = int(round(p1[1] * (1 - t) + p2[1] * t))
            pts.append([u, v])
        return pts

    def pixels_to_points3d(self, pixels, depth_frame):
        pts_3d = []
        width, height = depth_frame.get_width(), depth_frame.get_height()
        for u, v in pixels:
            if not (0 <= u < width and 0 <= v < height):
                continue
            dist = self.get_stable_depth(depth_frame, u, v)
            if dist <= 0:
                continue
            p3 = rs.rs2_deproject_pixel_to_point(self.intrinsics, [float(u), float(v)], dist)
            pts_3d.append([float(p3[0]), float(p3[1]), float(p3[2])])  # meters
        return pts_3d

    def transform_points_to_robot(self, points_cam):
        if self.camera_to_robot is None:
            return []
        out = []
        for p in points_cam:
            p4 = np.array([p[0], p[1], p[2], 1.0], dtype=np.float64)
            q = self.camera_to_robot @ p4
            out.append([float(q[0]), float(q[1]), float(q[2])])
        return out

    def enforce_visual_offset(self, center_pt, candidate_pt, lateral_direction_2d, min_offset_px, side_sign):
        center = np.array(center_pt, dtype=np.float64)
        direction = normalize_vector(lateral_direction_2d)
        if direction is None:
            direction = np.array([1.0, 0.0], dtype=np.float64)

        if candidate_pt is not None:
            candidate = np.array(candidate_pt, dtype=np.float64)
            if float(np.linalg.norm(candidate - center)) >= max(1.0, min_offset_px * 0.7):
                return float(candidate[0]), float(candidate[1])

        fallback = center + side_sign * direction * float(min_offset_px)
        return float(fallback[0]), float(fallback[1])

    def smooth_pose_keypoints(self, kpts):
        smoothed = kpts.copy()
        for idx, name in ((5, 'ls'), (6, 'rs'), (11, 'lh'), (12, 'rh')):
            sx, sy = self.smoother.smooth(name, kpts[idx][0], kpts[idx][1], round_result=False)
            smoothed[idx][0] = sx
            smoothed[idx][1] = sy
        return smoothed

    def save_meridian_result(self, frame, left_line, right_line, depth_frame):
        os.makedirs(ROBOT_OUTPUT_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        img_path = os.path.join(ROBOT_OUTPUT_DIR, f"meridian_frame_{ts}.png")
        json_path = os.path.join(ROBOT_OUTPUT_DIR, f"meridian_data_{ts}.json")

        cv2.imwrite(img_path, frame)

        left_px = self.sample_line_pixels(left_line[0], left_line[1])
        right_px = self.sample_line_pixels(right_line[0], right_line[1])
        left_cam = self.pixels_to_points3d(left_px, depth_frame)
        right_cam = self.pixels_to_points3d(right_px, depth_frame)
        left_robot = self.transform_points_to_robot(left_cam)
        right_robot = self.transform_points_to_robot(right_cam)

        data = {
            "timestamp": ts,
            "image_path": os.path.abspath(img_path),
            "sample_points": MERIDIAN_SAMPLE_POINTS,
            "camera_frame_unit": "meters",
            "left_meridian_pixel": left_px,
            "right_meridian_pixel": right_px,
            "left_meridian_camera": left_cam,
            "right_meridian_camera": right_cam,
        }
        if left_robot and right_robot:
            data["robot_frame_unit"] = "meters"
            data["left_meridian_robot"] = left_robot
            data["right_meridian_robot"] = right_robot

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return img_path, json_path

    def run(self):
        """
        实时相机模式。
        """
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                frames = self.align.process(frames)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame: continue
                
                img = np.asanyarray(color_frame.get_data())

                # 简单检测深度可用性：取中心点距离（米），打印一次提示
                center_dist = depth_frame.get_distance(depth_frame.get_width() // 2, depth_frame.get_height() // 2)
                if center_dist <= 0 and not self.depth_warned:
                    print("深度无效：当前帧中心点距离为 0，可能未对齐或超出量程")
                    self.depth_warned = True
                if center_dist > 0 and self.depth_warned:
                    print(f"深度恢复：中心点距离约 {center_dist:.3f} m")
                    self.depth_warned = False
                
                pose_info = infer_best_pose_with_rotations(self.model, img, conf=0.5)

                meridian_lines = None
                if pose_info["kpts"] is not None:
                    kpts = self.smooth_pose_keypoints(pose_info["kpts"])

                    # 检查肩部(5,6)和髋部(11,12)的置信度
                    if kpts[5][2] > 0.5 and kpts[6][2] > 0.5 and kpts[11][2] > 0.3 and kpts[12][2] > 0.3:
                        ls = np.array([float(kpts[5][0]), float(kpts[5][1])], dtype=np.float64)
                        rs_pt = np.array([float(kpts[6][0]), float(kpts[6][1])], dtype=np.float64)
                        lh = np.array([float(kpts[11][0]), float(kpts[11][1])], dtype=np.float64)
                        rh = np.array([float(kpts[12][0]), float(kpts[12][1])], dtype=np.float64)

                        raw_neck = (ls + rs_pt) / 2.0
                        raw_tail = (lh + rh) / 2.0
                        neck_u, neck_v = self.smoother.smooth('neck', raw_neck[0], raw_neck[1], round_result=False)
                        tail_u, tail_v = self.smoother.smooth('tail', raw_tail[0], raw_tail[1], round_result=False)

                        lateral_direction_2d = get_body_lateral_direction_2d(kpts)
                        shoulder_px = float(np.linalg.norm(rs_pt - ls))
                        pixels_per_mm = shoulder_px / ESTIMATED_SHOULDER_MM if ESTIMATED_SHOULDER_MM > 0 else 1.0
                        body_offset_px = max(MIN_VISUAL_OFFSET_PX, self.user_finger_width_mm * pixels_per_mm)

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

                        neck_l = self.smoother.smooth('neck_l', raw_neck_l[0], raw_neck_l[1], round_result=True)
                        neck_r = self.smoother.smooth('neck_r', raw_neck_r[0], raw_neck_r[1], round_result=True)
                        tail_l = self.smoother.smooth('tail_l', raw_tail_l[0], raw_tail_l[1], round_result=True)
                        tail_r = self.smoother.smooth('tail_r', raw_tail_r[0], raw_tail_r[1], round_result=True)
                        neck_center = self.smoother.smooth('neck_center', neck_u, neck_v, round_result=True)
                        tail_center = self.smoother.smooth('tail_center', tail_u, tail_v, round_result=True)

                        shoulder_cm_real = None
                        depth_l = depth_frame.get_distance(int(kpts[5][0]), int(kpts[5][1]))
                        depth_r = depth_frame.get_distance(int(kpts[6][0]), int(kpts[6][1]))
                        if depth_l > 0 and depth_r > 0:
                            p3d_l = rs.rs2_deproject_pixel_to_point(self.intrinsics, [kpts[5][0], kpts[5][1]], depth_l)
                            p3d_r = rs.rs2_deproject_pixel_to_point(self.intrinsics, [kpts[6][0], kpts[6][1]], depth_r)
                            shoulder_cm_real = float(np.linalg.norm(np.array(p3d_r) - np.array(p3d_l)) * 100.0)

                        cv2.line(img, neck_center, tail_center, (0, 0, 255), 2)
                        cv2.line(img, neck_l, tail_l, (0, 255, 0), 2)
                        cv2.line(img, neck_r, tail_r, (0, 255, 0), 2)
                        meridian_lines = ((neck_l, tail_l), (neck_r, tail_r))

                        for idx, name, color in [
                            (5, "LS", (0, 200, 255)),
                            (6, "RS", (0, 200, 255)),
                            (11, "LH", (200, 255, 0)),
                            (12, "RH", (200, 255, 0)),
                        ]:
                            x, y = int(kpts[idx][0]), int(kpts[idx][1])
                            cv2.circle(img, (x, y), 5, color, -1)
                            cv2.putText(img, name, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                        shoulder_txt = f"{shoulder_cm_real:.1f}cm" if shoulder_cm_real and shoulder_cm_real > 0 else "N/A"
                        cv2.putText(img, f"Offset: {self.user_finger_width_mm}mm | Shoulder: {shoulder_txt}", (20, 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                        cv2.putText(img, f"Pose rot: {pose_info['rotation']} | Body offset: {int(round(body_offset_px))} px", (20, 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                        cv2.putText(img, "s: save meridian for robot | q: quit | u/d: offset", (20, 110),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
                else:
                    self.smoother.miss()

                cv2.imshow("Linear Meridian Locator", img)
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'): break
                if key & 0xFF == ord('s'):
                    if meridian_lines is None:
                        print("当前帧膀胱经线无效，未保存")
                    else:
                        img_path, json_path = self.save_meridian_result(
                            img.copy(),
                            meridian_lines[0],
                            meridian_lines[1],
                            depth_frame,
                        )
                        print(f"[已保存] 图片: {img_path}")
                        print(f"[已保存] 轨迹: {json_path}")
                
                # 动态调整宽度（更新实例属性）
                if key & 0xFF == ord('u'): self.user_finger_width_mm += 5
                if key & 0xFF == ord('d'): self.user_finger_width_mm -= 5

        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    # 运行时输入手指宽度（毫米），默认 45
    try:
        global_finger_width_mm = float(input("请输入手指宽度(毫米，默认45): ").strip() or "45")
    except Exception:
        global_finger_width_mm = USER_FINGER_WIDTH_MM
        print(f"输入无效，使用默认 {global_finger_width_mm} mm")

    # 简单的模式选择：
    # 1) 输入图片路径 -> 静态图片模式（失败不退出，可继续输入路径）
    # 2) 输入 "webcam"  -> 笔记本/USB 摄像头模式（无深度，仅2D）
    # 3) 留空直接回车   -> RealSense 实时模式
    while True:
        img_path = input('输入本地图像路径，或输入 "webcam" 使用笔记本摄像头（留空则使用 RealSense 实时，输入 quit 退出）: ').strip()

        if img_path.lower() == "quit":
            break

        if not img_path:
            # 实时相机模式（需要 RealSense）
            detector = LinearMeridianDetector(global_finger_width_mm)
            detector.run()
            break

        if img_path.lower() == "webcam":
            # ----------- 普通摄像头实时模式（无深度） -----------
            try:
                cam_index = int(input("输入摄像头索引(默认0，可尝试1/2以避开RealSense): ").strip() or "0")
            except Exception:
                cam_index = 0
            cap = cv2.VideoCapture(cam_index)
            if not cap.isOpened():
                print("无法打开摄像头")
                continue

            model = YOLO(MODEL_PATH)
            if torch.cuda.is_available():
                model.to('cuda')
            finger_width_mm = global_finger_width_mm
            print(f'摄像头索引 {cam_index} 模式启动，按 q 退出，u/d 调整手指宽度（当前 {finger_width_mm:.1f}mm）')

            try:
                while True:
                    ret, img = cap.read()
                    if not ret:
                        print("无法读取摄像头画面")
                        break

                    pose_info = infer_best_pose_with_rotations(model, img, conf=0.5)
                    if pose_info["kpts"] is not None:
                        kpts = pose_info["kpts"]
                        h, w, _ = img.shape
                        if kpts[5][2] > 0.5 and kpts[6][2] > 0.5 and kpts[11][2] > 0.5 and kpts[12][2] > 0.5:
                            neck_u = int((kpts[5][0] + kpts[6][0]) / 2)
                            neck_v = int((kpts[5][1] + kpts[6][1]) / 2)
                            tail_u = int((kpts[11][0] + kpts[12][0]) / 2)
                            tail_v = int((kpts[11][1] + kpts[12][1]) / 2)

                            dx_px = kpts[6][0] - kpts[5][0]
                            dy_px = kpts[6][1] - kpts[5][1]
                            shoulder_px = float(np.hypot(dx_px, dy_px))

                            pixels_per_mm = shoulder_px / ESTIMATED_SHOULDER_MM if ESTIMATED_SHOULDER_MM > 0 else 1.0
                            offset_px = int(max(1, finger_width_mm * pixels_per_mm))
                            lateral_direction_2d = get_body_lateral_direction_2d(kpts)

                            def clamp_point(x, y):
                                return int(max(0, min(w - 1, x))), int(max(0, min(h - 1, y)))

                            neck_l = clamp_point(neck_u - lateral_direction_2d[0] * offset_px,
                                                 neck_v - lateral_direction_2d[1] * offset_px)
                            neck_r = clamp_point(neck_u + lateral_direction_2d[0] * offset_px,
                                                 neck_v + lateral_direction_2d[1] * offset_px)
                            tail_l = clamp_point(tail_u - lateral_direction_2d[0] * offset_px,
                                                 tail_v - lateral_direction_2d[1] * offset_px)
                            tail_r = clamp_point(tail_u + lateral_direction_2d[0] * offset_px,
                                                 tail_v + lateral_direction_2d[1] * offset_px)

                            cv2.line(img, (neck_u, neck_v), (tail_u, tail_v), (0, 0, 255), 2)
                            cv2.line(img, neck_l, tail_l, (0, 255, 0), 2)
                            cv2.line(img, neck_r, tail_r, (0, 255, 0), 2)

                            cv2.putText(img, f"Offset(px): {offset_px} (est. shoulder {ESTIMATED_SHOULDER_MM}mm)",
                                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            cv2.putText(img, f"Pose rot: {pose_info['rotation']}",
                                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

                            for idx, name in [(5, "LS"), (6, "RS"), (11, "LH"), (12, "RH")]:
                                x, y = int(kpts[idx][0]), int(kpts[idx][1])
                                cv2.circle(img, (x, y), 5, (0, 200, 255), -1)
                                cv2.putText(img, name, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

                    # 左下角显示分辨率
                    if 'w' in locals() and 'h' in locals():
                        cv2.putText(img, f"{w}x{h}", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    cv2.imshow("Linear Meridian Locator (Webcam)", img)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    if key == ord('u'):
                        finger_width_mm += 5
                    if key == ord('d'):
                        finger_width_mm -= 5
            finally:
                cap.release()
                cv2.destroyAllWindows()
            # 返回主菜单可继续输入路径
            continue

        # ---------------- 静态图片模式（失败不退出，可继续输入） ----------------
        import os
        if not os.path.exists(img_path):
            print("文件不存在，请检查路径")
            continue

        img = cv2.imread(img_path)
        if img is None:
            print("无法读取图片，请检查文件格式与路径")
            continue

        depth_path = input("输入对应的深度图路径(16-bit png，对齐彩色，留空则按2D估算): ").strip()
        depth_img = None
        depth_intr = None
        if depth_path:
            if not os.path.exists(depth_path):
                print("深度图不存在，改用2D估算")
            else:
                depth_img = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH)
                if depth_img is None:
                    print("无法读取深度图，改用2D估算")
                    depth_img = None
                else:
                    depth_intr = default_intrinsics(depth_img.shape[1], depth_img.shape[0])

        model = YOLO(MODEL_PATH)
        if torch.cuda.is_available():
            model.to('cuda')
        pose_info = infer_best_pose_with_rotations(model, img, conf=0.5)

        if pose_info["kpts"] is None:
            print("未检测到人体关键点，请更换图片再试")
            continue

        kpts = pose_info["kpts"]
        h, w, _ = img.shape

        if kpts[5][2] > 0.5 and kpts[6][2] > 0.5 and kpts[11][2] > 0.5:
            neck_u = int((kpts[5][0] + kpts[6][0]) / 2)
            neck_v = int((kpts[5][1] + kpts[6][1]) / 2)
            tail_u = int((kpts[11][0] + kpts[12][0]) / 2)
            tail_v = int((kpts[11][1] + kpts[12][1]) / 2)

            def clamp_point(x, y):
                return int(max(0, min(w - 1, x))), int(max(0, min(h - 1, y)))

            shoulder_cm_real = None

            if depth_img is not None and depth_intr is not None:
                # 使用深度做 3D 偏移，再投影回 2D
                def project_offset(u, v, offset_mm):
                    if u < 0 or u >= depth_img.shape[1] or v < 0 or v >= depth_img.shape[0]:
                        return None, None
                    depth_m = float(depth_img[int(v), int(u)]) * DEPTH_SCALE
                    if depth_m <= 0:
                        return None, None
                    p3d = rs.rs2_deproject_pixel_to_point(depth_intr, [u, v], depth_m)
                    offset_m = offset_mm / 1000.0
                    left_3d = [p3d[0] - offset_m, p3d[1], p3d[2]]
                    right_3d = [p3d[0] + offset_m, p3d[1], p3d[2]]
                    left_2d = rs.rs2_project_point_to_pixel(depth_intr, left_3d)
                    right_2d = rs.rs2_project_point_to_pixel(depth_intr, right_3d)
                    return clamp_point(int(left_2d[0]), int(left_2d[1])), clamp_point(int(right_2d[0]), int(right_2d[1]))

                neck_l, neck_r = project_offset(neck_u, neck_v, USER_FINGER_WIDTH_MM)
                tail_l, tail_r = project_offset(tail_u, tail_v, USER_FINGER_WIDTH_MM)
                # 计算真实肩宽（厘米）
                depth_l = float(depth_img[int(kpts[5][1]), int(kpts[5][0])]) * DEPTH_SCALE
                depth_r = float(depth_img[int(kpts[6][1]), int(kpts[6][0])]) * DEPTH_SCALE
                if depth_l > 0 and depth_r > 0:
                    p3d_l = rs.rs2_deproject_pixel_to_point(depth_intr, [kpts[5][0], kpts[5][1]], depth_l)
                    p3d_r = rs.rs2_deproject_pixel_to_point(depth_intr, [kpts[6][0], kpts[6][1]], depth_r)
                    shoulder_cm_real = float(np.linalg.norm(np.array(p3d_r) - np.array(p3d_l)) * 100.0)
                # 用投影后的横向像素差作为展示偏移
                offset_px = abs(neck_u - neck_l[0]) if neck_l else 0
            else:
                # 无深度，用估计肩宽换算像素偏移
                dx_px = kpts[6][0] - kpts[5][0]
                dy_px = kpts[6][1] - kpts[5][1]
                shoulder_px = float(np.hypot(dx_px, dy_px))
                pixels_per_mm = shoulder_px / ESTIMATED_SHOULDER_MM if ESTIMATED_SHOULDER_MM > 0 else 1.0
                offset_px = int(max(1, USER_FINGER_WIDTH_MM * pixels_per_mm))
                lateral_direction_2d = get_body_lateral_direction_2d(kpts)
                neck_l = clamp_point(neck_u - lateral_direction_2d[0] * offset_px,
                                     neck_v - lateral_direction_2d[1] * offset_px)
                neck_r = clamp_point(neck_u + lateral_direction_2d[0] * offset_px,
                                     neck_v + lateral_direction_2d[1] * offset_px)
                tail_l = clamp_point(tail_u - lateral_direction_2d[0] * offset_px,
                                     tail_v - lateral_direction_2d[1] * offset_px)
                tail_r = clamp_point(tail_u + lateral_direction_2d[0] * offset_px,
                                     tail_v + lateral_direction_2d[1] * offset_px)

            # 绘制
            cv2.line(img, (neck_u, neck_v), (tail_u, tail_v), (0, 0, 255), 2)
            if neck_l and tail_l:
                cv2.line(img, neck_l, tail_l, (0, 255, 0), 2)
            if neck_r and tail_r:
                cv2.line(img, neck_r, tail_r, (0, 255, 0), 2)

            shoulder_txt = f"{shoulder_cm_real:.1f}cm" if shoulder_cm_real and shoulder_cm_real > 0 else f"est {ESTIMATED_SHOULDER_MM/10:.1f}cm"
            cv2.putText(img, f"Offset(px): {offset_px} | Shoulder: {shoulder_txt}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(img, f"Pose rot: {pose_info['rotation']}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

            for idx, name in [(5, "LS"), (6, "RS"), (11, "LH"), (12, "RH")]:
                x, y = int(kpts[idx][0]), int(kpts[idx][1])
                cv2.circle(img, (x, y), 5, (0, 200, 255), -1)
                cv2.putText(img, name, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

            cv2.imshow("Linear Meridian Locator (Image)", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("肩/髋关键点置信度不足，未绘制，请换一张图再试。")
        # 处理完一张图后，继续回到输入提示