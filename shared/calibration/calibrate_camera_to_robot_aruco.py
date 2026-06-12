import json
import os
import sys
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FAIRINO_DIR = os.path.join(REPO_ROOT, "robots", "fairino")
if FAIRINO_DIR not in sys.path:
    sys.path.insert(0, FAIRINO_DIR)

# 兼容 fairino 导入
try:
    from fairino import Robot
except Exception:
    try:
        from src.user_functions.fairino import Robot
    except Exception:
        import importlib

        _dir = os.path.dirname(__file__)
        if _dir not in sys.path:
            sys.path.insert(0, _dir)
        Robot = importlib.import_module("fairino").Robot


# ---------------- 配置 ----------------
ROBOT_IP = "192.168.58.2"
OUTPUT_FILE = "camera_to_robot.json"
REPORT_FILE = "camera_to_robot_aruco_report.json"
PAIR_LOG_FILE = "camera_robot_aruco_pairs.json"

# ArUco 参数
ARUCO_DICT_NAME = "DICT_5X5_250"
ARUCO_ID = 0
ARUCO_MARKER_SIZE_M = 0.09  # 打印标记边长（米），例如 5cm -> 0.05

# 法奥位姿单位（通常 mm）
ROBOT_POS_IN_MM = True
SET_ROBOT_SPEED_ON_START = os.environ.get("CALIB_SET_ROBOT_SPEED", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}

# 采样约束
MIN_NEW_POINT_DIST_M = 0.01   # 与历史最近点至少 1cm
MIN_SPAN_M = 0.08             # 最小空间跨度 8cm

# 法兰 -> ArUco 中心 偏置（工具坐标系，单位：米）
# 你给的数据是 137.356mm，且说明“沿末端水平向前”：
# 这里按工具 +X 方向处理。若方向相反改为 -0.137356。
FLANGE_TO_ARUCO_TOOL_M = np.array([0.137356, 0.0, 0.0], dtype=np.float64)


def estimate_rigid_transform(cam_pts, robot_pts):
    """
    求解 p_robot = R * p_cam + t
    返回 T(4x4), rmse
    """
    cam = np.asarray(cam_pts, dtype=np.float64)
    rob = np.asarray(robot_pts, dtype=np.float64)
    if cam.shape[0] < 4:
        raise ValueError("至少需要4组点对")
    if cam.shape != rob.shape:
        raise ValueError("点对数量不一致")

    cam_mean = cam.mean(axis=0)
    rob_mean = rob.mean(axis=0)
    cam_c = cam - cam_mean
    rob_c = rob - rob_mean

    H = cam_c.T @ rob_c
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = rob_mean - R @ cam_mean

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t

    cam_h = np.hstack([cam, np.ones((cam.shape[0], 1))])
    rob_fit = (T @ cam_h.T).T[:, :3]
    rmse = float(np.sqrt(np.mean(np.sum((rob_fit - rob) ** 2, axis=1))))
    return T, rmse, rob_fit


def compute_errors(cam_pts, robot_pts, T):
    cam = np.asarray(cam_pts, dtype=np.float64)
    rob = np.asarray(robot_pts, dtype=np.float64)
    cam_h = np.hstack([cam, np.ones((cam.shape[0], 1))])
    rob_fit = (T @ cam_h.T).T[:, :3]
    errs = np.linalg.norm(rob_fit - rob, axis=1)
    return errs, rob_fit


def robust_fit(cam_pts, robot_pts, min_points=6):
    T0, rmse0, _ = estimate_rigid_transform(cam_pts, robot_pts)
    errs0, _ = compute_errors(cam_pts, robot_pts, T0)
    med = float(np.median(errs0))
    mad = float(np.median(np.abs(errs0 - med)))
    sigma = 1.4826 * mad
    th = max(0.005, med + 3.0 * sigma)
    inliers = [i for i, e in enumerate(errs0) if e <= th]
    outliers = [i for i, e in enumerate(errs0) if e > th]

    if len(inliers) >= min_points:
        cam_in = [cam_pts[i] for i in inliers]
        rob_in = [robot_pts[i] for i in inliers]
        T1, rmse1, _ = estimate_rigid_transform(cam_in, rob_in)
        if rmse1 <= rmse0:
            return T1, rmse1, inliers, outliers, th
    return T0, rmse0, list(range(len(cam_pts))), [], th


def axis_span(points_xyz):
    p = np.asarray(points_xyz, dtype=np.float64)
    mins = p.min(axis=0)
    maxs = p.max(axis=0)
    span = maxs - mins
    return mins.tolist(), maxs.tolist(), span.tolist()


def euler_deg_xyz_to_R(rx_deg, ry_deg, rz_deg):
    """
    将 rx, ry, rz(度) 转旋转矩阵。
    默认按 R = Rz * Ry * Rx 组合（常见机械臂姿态表示）。
    """
    rx = np.deg2rad(rx_deg)
    ry = np.deg2rad(ry_deg)
    rz = np.deg2rad(rz_deg)

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def get_aruco_dict(name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("当前 OpenCV 未包含 aruco 模块，请安装 opencv-contrib-python")
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"不支持的字典: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def estimate_single_marker_pose(marker_corners, marker_size_m, camera_matrix, dist_coeffs):
    """
    兼容不同 OpenCV 版本的单个 ArUco 姿态估计。
    新版部分环境没有 cv2.aruco.estimatePoseSingleMarkers，用 solvePnP 代替。
    """
    marker_corners = np.asarray(marker_corners, dtype=np.float64)
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners, marker_size_m, camera_matrix, dist_coeffs
        )
        return rvecs[0][0], tvecs[0][0]

    half = float(marker_size_m) / 2.0
    obj_pts = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    img_pts = marker_corners.reshape(-1, 2).astype(np.float64)

    flags = []
    if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
        flags.append(cv2.SOLVEPNP_IPPE_SQUARE)
    flags.append(cv2.SOLVEPNP_ITERATIVE)

    for flag in flags:
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, camera_matrix, dist_coeffs, flags=flag
        )
        if ok:
            return rvec.reshape(3), tvec.reshape(3)
    raise RuntimeError("solvePnP failed for ArUco marker")


def normalize_robot_pose_response(ret):
    """
    兼容 FAIRINO SDK 包装层和底层 XMLRPC 的位姿返回格式。
    SDK: (0, [x, y, z, rx, ry, rz])
    XMLRPC: [0, x, y, z, rx, ry, rz]
    """
    if not isinstance(ret, (tuple, list)):
        raise RuntimeError(f"位姿返回类型异常: {type(ret).__name__}: {ret}")

    if len(ret) == 2 and int(ret[0]) == 0:
        pose = ret[1]
        if isinstance(pose, (tuple, list)) and len(pose) >= 6:
            return [float(v) for v in pose[:6]]

    if len(ret) >= 7 and int(ret[0]) == 0:
        return [float(v) for v in ret[1:7]]

    if len(ret) >= 6:
        try:
            return [float(v) for v in ret[:6]]
        except Exception:
            pass

    raise RuntimeError(f"无法解析位姿返回: {ret}")


class ArucoCalibrator:
    def __init__(self):
        print("[Init] 启动 RealSense...", flush=True)
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)  # 深度对齐到彩色
        print("[Init] RealSense OK", flush=True)

        # 使用彩色流内参（深度对齐后与彩色共享坐标系）
        color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        self.rs_intr = intr  # pyrealsense2 intrinsics object for deproject
        self.K = np.array(
            [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
            dtype=np.float64,
        )
        self.D = np.array(intr.coeffs, dtype=np.float64)

        print(f"[Init] 连接机械臂 {ROBOT_IP}...", flush=True)
        self.robot = Robot.RPC(ROBOT_IP)
        print("[Init] 机械臂 RPC OK", flush=True)
        if SET_ROBOT_SPEED_ON_START:
            print("[Init] SetSpeed(10)...", flush=True)
            ret = self.robot.SetSpeed(10)
            print(f"[Init] SetSpeed(10) -> {ret}", flush=True)
        else:
            print("[Init] 跳过 SetSpeed；标定只读取当前 TCP 位姿", flush=True)

        self.aruco_dict = get_aruco_dict(ARUCO_DICT_NAME)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        self.cam_pts = []
        self.rob_pts = []
        self.pair_log = []
        self.pose_source = None

    def read_robot_pose_mm(self):
        errors = []

        # 当前 C 扩展 SDK 的 GetActualTCPPose 包装层可能因 ctypes 字段未实例化而失败；
        # 底层 XMLRPC 仍可正常返回 [0, x, y, z, rx, ry, rz]。
        direct_rpc = getattr(self.robot, "robot", None)
        candidates = []
        if direct_rpc is not None:
            candidates.extend(
                [
                    ("xmlrpc GetActualToolFlangePose", lambda: direct_rpc.GetActualToolFlangePose(1)),
                    ("xmlrpc GetActualTCPPose", lambda: direct_rpc.GetActualTCPPose(1)),
                ]
            )
        candidates.extend(
            [
                ("sdk GetActualToolFlangePose", lambda: self.robot.GetActualToolFlangePose(1)),
                ("sdk GetActualTCPPose", lambda: self.robot.GetActualTCPPose(1)),
            ]
        )

        for source, fn in candidates:
            try:
                pose = normalize_robot_pose_response(fn())
                if self.pose_source != source:
                    print(f"[Robot] 位姿读取方式: {source}")
                    self.pose_source = source
                return pose
            except Exception as e:
                errors.append(f"{source}: {e}")

        raise RuntimeError("所有位姿读取方式均失败: " + " | ".join(errors))

    def get_robot_tcp_xyz_m(self):
        pose = self.read_robot_pose_mm()
        x, y, z = float(pose[0]), float(pose[1]), float(pose[2])
        if ROBOT_POS_IN_MM:
            x, y, z = x / 1000.0, y / 1000.0, z / 1000.0
        rx, ry, rz = float(pose[3]), float(pose[4]), float(pose[5])
        R = euler_deg_xyz_to_R(rx, ry, rz)
        # 将“法兰点”补偿到“Aruco中心点”
        p_flange = np.array([x, y, z], dtype=np.float64)
        p_aruco = p_flange + R @ FLANGE_TO_ARUCO_TOOL_M
        return p_aruco.tolist(), [float(v) for v in pose]

    def detect_marker_pose(self, img, depth_frame=None):
        """
        检测 ArUco 标记。
        返回 (aruco_pose, depth_xyz, corners, ids)
        - aruco_pose: (rvec, tvec) 来自 estimatePoseSingleMarkers（仅用于画坐标轴）
        - depth_xyz: [x, y, z] 来自深度传感器反投影（用于标定）
        """
        corners, ids, _ = self.detector.detectMarkers(img)
        if ids is None or len(ids) == 0:
            return None, None, corners, ids
        ids_flat = ids.flatten().tolist()
        if ARUCO_ID not in ids_flat:
            return None, None, corners, ids

        idx = ids_flat.index(ARUCO_ID)
        marker_corners = np.array(corners[idx], dtype=np.float64)
        rvec, tvec = estimate_single_marker_pose(
            marker_corners, ARUCO_MARKER_SIZE_M, self.K, self.D
        )
        aruco_pose = (rvec, tvec)

        # --- 深度传感器反投影：更准确的 3D 定位 ---
        depth_xyz = None
        if depth_frame is not None:
            # ArUco 四角中心像素
            center_u = float(np.mean(corners[idx][0][:, 0]))
            center_v = float(np.mean(corners[idx][0][:, 1]))
            cu, cv = int(round(center_u)), int(round(center_v))

            # 在 5x5 窗口内取中值深度（降噪）
            depths = []
            w = depth_frame.get_width()
            h = depth_frame.get_height()
            for du in range(-2, 3):
                for dv in range(-2, 3):
                    pu, pv = cu + du, cv + dv
                    if 0 <= pu < w and 0 <= pv < h:
                        d = depth_frame.get_distance(pu, pv)
                        if d > 0.1:  # 过滤无效值
                            depths.append(d)
            if depths:
                depth_m = float(np.median(depths))
                p3d = rs.rs2_deproject_pixel_to_point(
                    self.rs_intr, [center_u, center_v], depth_m
                )
                depth_xyz = [float(p3d[0]), float(p3d[1]), float(p3d[2])]

        return aruco_pose, depth_xyz, corners, ids

    def run(self):
        print("=== FR5 + RealSense D435i ArUco 标定 (深度增强版) ===")
        print(f"机器人IP: {ROBOT_IP}")
        print(f"Aruco字典: {ARUCO_DICT_NAME}, ID: {ARUCO_ID}, 边长: {ARUCO_MARKER_SIZE_M} m")
        print(f"法兰->Aruco 偏置(工具系,m): {FLANGE_TO_ARUCO_TOOL_M.tolist()}")
        print(f"3D定位方式: 深度传感器反投影 (与运行时一致)")
        print("操作：")
        print("1) 将 ArUco 纸贴在法兰或临时夹具上，保持平整")
        print("2) 移动机械臂到不同空间位置（建议姿态尽量一致）")
        print("3) 画面识别到目标ID后按 c 采样")
        print("4) 采样 >= 12 组，空间分布尽量大")
        print("5) 按 f 拟合并保存 camera_to_robot.json")
        print("按 x 撤销最后一组，按 q 退出")

        cv2.namedWindow("Aruco Camera->Robot Calibration", cv2.WINDOW_NORMAL)
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                frames = self.align.process(frames)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not color_frame or not depth_frame:
                    continue
                img = np.asanyarray(color_frame.get_data())

                aruco_pose, depth_xyz, corners, ids = self.detect_marker_pose(
                    img, depth_frame
                )
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(img, corners, ids)

                # 用于采样的 3D 坐标（优先深度传感器）
                cam_xyz = depth_xyz
                if aruco_pose is not None:
                    rvec, tvec = aruco_pose
                    aruco_xyz = [float(tvec[0]), float(tvec[1]), float(tvec[2])]
                    cv2.drawFrameAxes(img, self.K, self.D, rvec, tvec, ARUCO_MARKER_SIZE_M * 0.6, 2)

                    if depth_xyz is not None:
                        # 显示两种方法的对比
                        diff = np.linalg.norm(np.array(depth_xyz) - np.array(aruco_xyz)) * 1000
                        cv2.putText(
                            img,
                            f"Depth(m): ({depth_xyz[0]:.3f},{depth_xyz[1]:.3f},{depth_xyz[2]:.3f})",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                        )
                        cv2.putText(
                            img,
                            f"ArUco(m): ({aruco_xyz[0]:.3f},{aruco_xyz[1]:.3f},{aruco_xyz[2]:.3f}) diff={diff:.1f}mm",
                            (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1,
                        )
                    else:
                        # 深度不可用时回退到 ArUco 估计
                        cam_xyz = aruco_xyz
                        cv2.putText(
                            img,
                            f"ArUco(m): ({aruco_xyz[0]:.3f},{aruco_xyz[1]:.3f},{aruco_xyz[2]:.3f}) [no depth]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2,
                        )
                else:
                    cv2.putText(
                        img,
                        f"marker id={ARUCO_ID} not found",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                    )

                cv2.putText(
                    img,
                    "click window first | c/space:capture  x:undo  f:fit+save  q:quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2,
                )
                cv2.putText(
                    img,
                    f"pairs: {len(self.cam_pts)}",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2,
                )
                cv2.imshow("Aruco Camera->Robot Calibration", img)

                key_raw = cv2.waitKeyEx(30)
                key = key_raw & 0xFF if key_raw != -1 else -1
                if key in (ord("q"), ord("Q"), 27):
                    break

                if key in (ord("x"), ord("X")):
                    if not self.cam_pts:
                        print("当前没有可撤销采样")
                    else:
                        self.cam_pts.pop()
                        self.rob_pts.pop()
                        self.pair_log.pop()
                        print(f"[撤销成功] 剩余采样: {len(self.cam_pts)}")

                if key in (ord("c"), ord("C"), ord(" ")):
                    if cam_xyz is None:
                        print("未识别到目标 ArUco 或深度无效，无法采样")
                        continue
                    try:
                        rob_xyz_m, raw_pose = self.get_robot_tcp_xyz_m()
                    except Exception as e:
                        print(f"读取机器人位姿失败: {e}")
                        continue

                    if self.cam_pts:
                        cam_d = min(np.linalg.norm(np.asarray(cam_xyz) - np.asarray(p)) for p in self.cam_pts)
                        rob_d = min(np.linalg.norm(np.asarray(rob_xyz_m) - np.asarray(p)) for p in self.rob_pts)
                        if cam_d < MIN_NEW_POINT_DIST_M and rob_d < MIN_NEW_POINT_DIST_M:
                            print(
                                f"与历史点过近(cam:{cam_d:.4f}m, robot:{rob_d:.4f}m)，"
                                "请换新位置再采样"
                            )
                            continue

                    method = "depth" if depth_xyz is not None else "aruco_fallback"
                    self.cam_pts.append(cam_xyz)
                    self.rob_pts.append(rob_xyz_m)
                    self.pair_log.append(
                        {
                            "camera_xyz_m": cam_xyz,
                            "robot_tcp_pose_raw": raw_pose,
                            "robot_xyz_m": rob_xyz_m,
                            "method": method,
                        }
                    )
                    print(f"[采样成功] #{len(self.cam_pts)} cam={[round(v,4) for v in cam_xyz]} "
                          f"robot={[round(v,4) for v in rob_xyz_m]} ({method})")

                if key in (ord("f"), ord("F")):
                    if len(self.cam_pts) < 4:
                        print("点对不足，至少4组，建议12~20组")
                        continue
                    cam_min, cam_max, cam_span = axis_span(self.cam_pts)
                    rob_min, rob_max, rob_span = axis_span(self.rob_pts)
                    if max(cam_span) < MIN_SPAN_M or max(rob_span) < MIN_SPAN_M:
                        print("点对空间分布过小，拟合不稳定。")
                        print(f"相机点跨度(m): {cam_span}")
                        print(f"机器人点跨度(m): {rob_span}")
                        print("请扩大采样范围后再拟合")
                        continue

                    try:
                        T, rmse, inliers, outliers, th = robust_fit(self.cam_pts, self.rob_pts)
                        errs, rob_fit = compute_errors(self.cam_pts, self.rob_pts, T)
                    except Exception as e:
                        print(f"拟合失败: {e}")
                        continue

                    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    out_main = {
                        "timestamp": ts,
                        "method": "aruco_center_point_based",
                        "robot_ip": ROBOT_IP,
                        "aruco_dict": ARUCO_DICT_NAME,
                        "aruco_id": ARUCO_ID,
                        "aruco_size_m": ARUCO_MARKER_SIZE_M,
                        "point_pairs": len(self.cam_pts),
                        "rmse_m": rmse,
                        "inlier_count": len(inliers),
                        "outlier_count": len(outliers),
                        "outlier_threshold_m": th,
                        "matrix": T.tolist(),
                        "equation": "p_robot = T @ [p_cam_x, p_cam_y, p_cam_z, 1]^T",
                    }
                    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
                        json.dump(out_main, f, ensure_ascii=False, indent=2)
                    with open(PAIR_LOG_FILE, "w", encoding="utf-8") as f:
                        json.dump({"pairs": self.pair_log}, f, ensure_ascii=False, indent=2)

                    report = {
                        "timestamp": ts,
                        "rmse_m": rmse,
                        "outlier_threshold_m": th,
                        "inlier_indices": inliers,
                        "outlier_indices": outliers,
                        "per_point_error_m": [float(e) for e in errs.tolist()],
                        "camera_min_xyz_m": cam_min,
                        "camera_max_xyz_m": cam_max,
                        "camera_span_xyz_m": cam_span,
                        "robot_min_xyz_m": rob_min,
                        "robot_max_xyz_m": rob_max,
                        "robot_span_xyz_m": rob_span,
                        "pairs_detailed": [],
                    }
                    for i in range(len(self.cam_pts)):
                        report["pairs_detailed"].append(
                            {
                                "index": i,
                                "camera_xyz_m": [float(v) for v in self.cam_pts[i]],
                                "robot_xyz_m": [float(v) for v in self.rob_pts[i]],
                                "robot_fit_xyz_m": [float(v) for v in rob_fit[i].tolist()],
                                "error_m": float(errs[i]),
                                "is_inlier": i in inliers,
                            }
                        )
                    with open(REPORT_FILE, "w", encoding="utf-8") as f:
                        json.dump(report, f, ensure_ascii=False, indent=2)

                    print(f"[保存完成] {OUTPUT_FILE}")
                    print(f"[保存完成] {PAIR_LOG_FILE}")
                    print(f"[保存完成] {REPORT_FILE}")
                    print(f"RMSE: {rmse:.6f} m | 内点/外点: {len(inliers)}/{len(outliers)} | 阈值: {th:.6f} m")
                    if rmse > 0.01:
                        print("RMSE 偏大，建议重新采样（扩大空间分布，减少抖动）")

        finally:
            try:
                self.robot.CloseRPC()
            except Exception:
                pass
            self.pipeline.stop()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    ArucoCalibrator().run()
