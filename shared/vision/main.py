import pyrealsense2 as rs  # RealSense SDK，用于深度/彩色流与几何计算
import numpy as np  # 数值与几何计算
import cv2  # OpenCV 图像处理与展示
import mediapipe as mp  # MediaPipe Pose 关键点检测

# =========================
# 常量配置
# =========================
BLADDER_OFFSET_MULTIPLIER = 1.5  # 膀胱经偏移 = 手指宽度 * 1.5
DEPTH_SCALE = 0.001  # RealSense 深度单位换算：16bit 原始值 * 0.001 = 米（D435i 常见标定）
ESTIMATED_SHOULDER_WIDTH_CM = 36.0  # 深度不可用时的肩宽兜底（厘米）


# =========================
# 内参与几何
# =========================
def default_intrinsics(width, height):
    """
    若无真实内参，用 640x480 典型值按分辨率线性缩放兜底。
    解释（相机内参）：
    - fx, fy：焦距（以像素为单位），决定 3D 点到像素的投影缩放
    - ppx, ppy：主点（光心）在像素平面的坐标
    - model/coeffs：畸变模型与系数（这里设为无畸变）
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


# =========================
# 深度与肩宽计算
# =========================
def sample_depth_value(x, y, depth_frame, depth_image, intr):
    """
    根据来源（实时帧或离线深度图）读取深度，单位米。
    - 实时：depth_frame.get_distance(x, y) 直接返回米
    - 离线：16bit 深度值 * DEPTH_SCALE（0.001）转换成米
    """
    if depth_frame is not None:
        if 0 <= x < intr.width and 0 <= y < intr.height:
            return depth_frame.get_distance(int(x), int(y))
        return 0.0
    if depth_image is not None and 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
        return float(depth_image[int(y), int(x)]) * DEPTH_SCALE
    return 0.0


def compute_shoulder_pixels_per_cm(landmarks, depth_frame, depth_image, depth_intrinsics, image_shape):
    """
    返回 (pixels_per_cm, shoulder_used_cm)。
    步骤：肩像素距离 -> 采样深度 -> 反投影成 3D -> 计算真实肩宽(cm) -> 像素/厘米。
    关键公式：
    1) 深度采样：depth_m = depth_raw * DEPTH_SCALE（若为离线图）
    2) 反投影：P3D = rs.rs2_deproject_pixel_to_point(intr, [u, v], depth_m)
       说明：用相机内参把像素坐标 + 深度还原成相机坐标系下的 (X, Y, Z)（单位米）
    3) 肩宽（厘米）= ||P3D_right - P3D_left|| * 100
    4) 像素/厘米 = shoulder_px / shoulder_cm_real
       若深度无效，则用 ESTIMATED_SHOULDER_WIDTH_CM 兜底
    """
    h, w, _ = image_shape
    l_shoulder, r_shoulder = landmarks[11], landmarks[12]
    # 像素肩宽：使用欧氏距离，既考虑左右差也考虑上下差
    dx_px = (r_shoulder.x - l_shoulder.x) * w
    dy_px = (r_shoulder.y - l_shoulder.y) * h
    shoulder_px = float(np.hypot(dx_px, dy_px))

    shoulder_cm_real = None
    intr = depth_intrinsics
    if intr is not None:
        max_w, max_h = max(1, intr.width), max(1, intr.height)

        def clamp_xy(x, y):
            return int(max(0, min(max_w - 1, x))), int(max(0, min(max_h - 1, y)))

        lx, ly = clamp_xy(l_shoulder.x * w, l_shoulder.y * h)
        rx, ry = clamp_xy(r_shoulder.x * w, r_shoulder.y * h)

        dist_l = sample_depth_value(lx, ly, depth_frame, depth_image, intr)
        dist_r = sample_depth_value(rx, ry, depth_frame, depth_image, intr)

        if dist_l > 0 and dist_r > 0:
            p3d_l = rs.rs2_deproject_pixel_to_point(intr, [lx, ly], dist_l)
            p3d_r = rs.rs2_deproject_pixel_to_point(intr, [rx, ry], dist_r)
            shoulder_cm_real = np.linalg.norm(np.array(p3d_r) - np.array(p3d_l)) * 100.0

    if shoulder_cm_real and shoulder_cm_real > 0:
        pixels_per_cm = shoulder_px / shoulder_cm_real
        shoulder_used_cm = shoulder_cm_real
    else:
        pixels_per_cm = shoulder_px / ESTIMATED_SHOULDER_WIDTH_CM if ESTIMATED_SHOULDER_WIDTH_CM > 0 else 10.0
        shoulder_used_cm = ESTIMATED_SHOULDER_WIDTH_CM

    if pixels_per_cm <= 0:
        pixels_per_cm = 10.0  # 再兜底，避免除零

    return pixels_per_cm, shoulder_used_cm


# =========================
# 绘制
# =========================
def draw_spine_and_bladder(color_image, landmarks, pixels_per_cm, finger_width_cm):
    """绘制脊柱中线和左右膀胱经，返回偏移像素。"""
    h, w, _ = color_image.shape
    l_shoulder, r_shoulder = landmarks[11], landmarks[12]
    l_hip, r_hip = landmarks[23], landmarks[24]

    cx_top = int((l_shoulder.x + r_shoulder.x) / 2 * w)
    cy_top = int((l_shoulder.y + r_shoulder.y) / 2 * h)
    cx_bottom = int((l_hip.x + r_hip.x) / 2 * w)
    cy_bottom = int((l_hip.y + r_hip.y) / 2 * h)

    offset_px = int(max(1, finger_width_cm * BLADDER_OFFSET_MULTIPLIER * pixels_per_cm))
    offset_px = min(offset_px, max(1, w // 2 - 1))  # 不超过半幅

    def clamp_point(x, y):
        return int(max(0, min(w - 1, x))), int(max(0, min(h - 1, y)))

    cx_top_left, cy_top_left = clamp_point(cx_top - offset_px, cy_top)
    cx_bottom_left, cy_bottom_left = clamp_point(cx_bottom - offset_px, cy_bottom)
    cx_top_right, cy_top_right = clamp_point(cx_top + offset_px, cy_top)
    cx_bottom_right, cy_bottom_right = clamp_point(cx_bottom + offset_px, cy_bottom)
    cx_top, cy_top = clamp_point(cx_top, cy_top)
    cx_bottom, cy_bottom = clamp_point(cx_bottom, cy_bottom)

    # 标注四个关键点：左肩、右肩、左髋、右髋（圆点 + 简短文字）
    keypoints = [
        ("LS", clamp_point(int(l_shoulder.x * w), int(l_shoulder.y * h)), (0, 200, 255)),
        ("RS", clamp_point(int(r_shoulder.x * w), int(r_shoulder.y * h)), (0, 200, 255)),
        ("LH", clamp_point(int(l_hip.x * w), int(l_hip.y * h)), (200, 255, 0)),
        ("RH", clamp_point(int(r_hip.x * w), int(r_hip.y * h)), (200, 255, 0)),
    ]
    for label, (px, py), color in keypoints:
        cv2.circle(color_image, (px, py), 5, color, -1)
        cv2.putText(color_image, label, (px + 6, py - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    cv2.line(color_image, (cx_top, cy_top), (cx_bottom, cy_bottom), (0, 255, 0), 2)          # 脊柱中心线
    cv2.line(color_image, (cx_top_left, cy_top_left), (cx_bottom_left, cy_bottom_left), (255, 0, 0), 2)  # 左膀胱经
    cv2.line(color_image, (cx_top_right, cy_top_right), (cx_bottom_right, cy_bottom_right), (0, 0, 255), 2)  # 右膀胱经
    return offset_px


# =========================
# 单帧处理

# =========================
def process_one(color_image, depth_frame, depth_image, depth_intrinsics, finger_width_cm, pose):
    """处理一帧彩色图（可含深度），返回绘制后图像与信息字符串。"""
    image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = pose.process(image_rgb)
    if not results.pose_landmarks:
        return color_image, "未检测到人体关键点"

    landmarks = results.pose_landmarks.landmark
    pixels_per_cm, shoulder_used_cm = compute_shoulder_pixels_per_cm(
        landmarks, depth_frame, depth_image, depth_intrinsics, color_image.shape
    )
    offset_px = draw_spine_and_bladder(color_image, landmarks, pixels_per_cm, finger_width_cm)

    info = f"Offset: {offset_px}px (~{finger_width_cm*BLADDER_OFFSET_MULTIPLIER:.2f} finger widths), Shoulder: {shoulder_used_cm:.1f}cm"
    cv2.putText(color_image, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return color_image, info


# =========================
# 模式：静态图片
# =========================
def run_static(color_path, depth_path, finger_width_cm):
    color_image = cv2.imread(color_path)
    if color_image is None:
        print("无法读取彩色图，请检查路径")
        return

    depth_image = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH)
    if depth_image is None:
        print("无法读取深度图，请检查路径")
        return

    depth_intrinsics = default_intrinsics(depth_image.shape[1], depth_image.shape[0])
    pose = mp.solutions.pose.Pose(
        static_image_mode=True,
        model_complexity=1,
        enable_segmentation=False,
        min_detection_confidence=0.3,
        min_tracking_confidence=0.3,
    )

    result_img, info = process_one(color_image, None, depth_image, depth_intrinsics, finger_width_cm, pose)
    print(info)
    cv2.imshow("Bladder Meridian Detection", result_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# =========================
# 模式：实时相机
# =========================
def run_realtime(finger_width_cm):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    align = rs.align(rs.stream.color)
    print("相机已启动，按 'q' 退出...")

    pose = mp.solutions.pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        enable_segmentation=False,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                aligned = align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                result_img, _ = process_one(color_image, depth_frame, None, depth_intrinsics, finger_width_cm, pose)
                cv2.imshow("Bladder Meridian Detection", result_img)
            except Exception as e:
                print(f"实时循环出错：{e}")
                continue

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


# =========================
# 入口
# =========================
def main():
    try:
        finger_width_cm = float(input("请输入手指宽度(厘米，默认2.0): ") or "2.0")
    except Exception:
        finger_width_cm = 2.0

    color_path = input("输入彩色图路径(留空则使用相机实时检测): ").strip()
    if color_path:
        depth_path = input("输入对应的深度图路径(16-bit png，对齐彩色，必填以计算肩宽): ").strip()
        if not depth_path:
            print("需要深度图来计算真实肩宽，请提供 depth_raw_*.png")
            return
        run_static(color_path, depth_path, finger_width_cm)
    else:
        run_realtime(finger_width_cm)


if __name__ == "__main__":
    main()