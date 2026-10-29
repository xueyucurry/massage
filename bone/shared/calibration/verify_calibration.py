"""
手眼标定验证脚本（深度增强版）：

使用深度传感器测量 ArUco 3D 坐标（与标定和运行时一致），
将预测的机器人坐标与实际位姿对比。

操作:
  m = 移动到标记 XY 上方(安全高度)
  g = 移动到标记位置(含 Z，安全限制)
  r = 读取机械臂当前位姿，与预测对比
  q = 退出
"""
import json
import os

import cv2
import numpy as np
import pyrealsense2 as rs

try:
    from fairino import Robot
except Exception:
    try:
        from src.user_functions.fairino import Robot
    except Exception:
        import importlib, sys
        _dir = os.path.dirname(__file__)
        if _dir not in sys.path:
            sys.path.insert(0, _dir)
        Robot = importlib.import_module("fairino").Robot

# ---------- 配置 ----------
ROBOT_IP = "192.168.58.2"
CALIBRATION_FILE = "camera_to_robot.json"
SAFE_Z_MM = 300.0           # 安全转场高度
APPROACH_HEIGHT_MM = 50.0    # 标记上方的额外抬高
DEFAULT_RX, DEFAULT_RY, DEFAULT_RZ = -178.190, 1.724, -1.187


def load_calibration(path=CALIBRATION_FILE):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    T = np.array(data["matrix"], dtype=np.float64)
    aruco_dict_name = data.get("aruco_dict", "DICT_5X5_250")
    aruco_id = data.get("aruco_id", 0)
    aruco_size = data.get("aruco_size_m", 0.09)
    rmse = data.get("rmse_m", -1)
    return T, aruco_dict_name, aruco_id, aruco_size, rmse


def cam_to_robot(T, p_cam):
    p = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0], dtype=np.float64)
    q = T @ p
    return q[:3]


def get_current_orientation(robot):
    ret = robot.GetActualTCPPose()
    if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
        pose = ret[1]
        return [float(pose[3]), float(pose[4]), float(pose[5])]
    return [DEFAULT_RX, DEFAULT_RY, DEFAULT_RZ]


def safe_move(robot, target_pose, tool=0, user=0):
    """安全移动：先垂直抬起到安全高度，再水平移动，再下降。"""
    ret = robot.GetActualTCPPose()
    if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
        cur = ret[1]
        cur_z = float(cur[2])
        # Step 1: 垂直抬起
        if cur_z < SAFE_Z_MM:
            up_pose = [float(cur[0]), float(cur[1]), SAFE_Z_MM,
                       float(cur[3]), float(cur[4]), float(cur[5])]
            rtn = robot.MoveCart(desc_pos=up_pose, tool=tool, user=user, blendT=0.0)
            if rtn != 0:
                print(f"  抬起失败 err={rtn}")
                return rtn

    # Step 2: 水平移到目标 XY (保持安全高度)
    transit_pose = [target_pose[0], target_pose[1], SAFE_Z_MM,
                    target_pose[3], target_pose[4], target_pose[5]]
    rtn = robot.MoveCart(desc_pos=transit_pose, tool=tool, user=user, blendT=0.0)
    if rtn != 0:
        print(f"  水平移动失败 err={rtn}")
        return rtn

    # Step 3: 下降到目标 Z
    if target_pose[2] < SAFE_Z_MM:
        rtn = robot.MoveCart(desc_pos=target_pose, tool=tool, user=user, blendT=0.0)
        if rtn != 0:
            print(f"  下降失败 err={rtn}")
            return rtn

    return 0


def main():
    T, dict_name, target_id, marker_size, rmse = load_calibration()
    print(f"标定矩阵已加载: RMSE={rmse * 1000:.1f}mm, ArUco={dict_name} ID={target_id}")

    R = T[:3, :3]
    det_R = np.linalg.det(R)
    orth_err = np.max(np.abs(R @ R.T - np.eye(3)))
    print(f"旋转矩阵: det={det_R:.6f}, 正交误差={orth_err:.2e}")
    t = T[:3, 3]
    print(f"相机位置(m): X={t[0]:.3f} Y={t[1]:.3f} Z={t[2]:.3f}")

    # RealSense (深度 + 彩色 + 对齐)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    # 使用彩色流内参（深度对齐后共享坐标系）
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()
    rs_intr = intr
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
    D = np.array(intr.coeffs, dtype=np.float64)

    # ArUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    # Robot
    robot = Robot.RPC(ROBOT_IP)
    robot.SetSpeed(10)

    predicted_robot_mm = None
    last_compare = None  # (predicted, actual, dist) for on-screen display
    last_move_target = None  # 实际发送给机器人的目标坐标

    print("\n操作:")
    print("  m = 安全移动到标记上方")
    print("  g = 安全移动到标记位置")
    print("  r = 对比预测 vs 实际位姿")
    print("  q = 退出\n")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not color_frame or not depth_frame:
                continue
            img = np.asanyarray(color_frame.get_data())

            corners, ids, _ = detector.detectMarkers(img)
            cam_xyz = None

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
                ids_flat = ids.flatten().tolist()
                if target_id in ids_flat:
                    idx = ids_flat.index(target_id)

                    # ArUco pose (仅用于画坐标轴)
                    mc = np.array(corners[idx], dtype=np.float64)
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        mc, marker_size, K, D
                    )
                    rvec, tvec = rvecs[0][0], tvecs[0][0]
                    cv2.drawFrameAxes(img, K, D, rvec, tvec, marker_size * 0.5, 2)

                    # 深度传感器测量 3D（与标定和运行时一致）
                    center_u = float(np.mean(corners[idx][0][:, 0]))
                    center_v = float(np.mean(corners[idx][0][:, 1]))
                    cu, cv_ = int(round(center_u)), int(round(center_v))

                    depths = []
                    w, h = depth_frame.get_width(), depth_frame.get_height()
                    for du in range(-2, 3):
                        for dv in range(-2, 3):
                            pu, pv = cu + du, cv_ + dv
                            if 0 <= pu < w and 0 <= pv < h:
                                d = depth_frame.get_distance(pu, pv)
                                if d > 0.1:
                                    depths.append(d)

                    if depths:
                        depth_m = float(np.median(depths))
                        p3d = rs.rs2_deproject_pixel_to_point(
                            rs_intr, [center_u, center_v], depth_m
                        )
                        cam_xyz = [float(p3d[0]), float(p3d[1]), float(p3d[2])]

                        rob_m = cam_to_robot(T, cam_xyz)
                        rob_mm = rob_m * 1000.0
                        predicted_robot_mm = rob_mm

                        cv2.putText(img,
                            f"Cam(m): ({cam_xyz[0]:.3f}, {cam_xyz[1]:.3f}, {cam_xyz[2]:.3f})",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                        cv2.putText(img,
                            f"Robot(mm): ({rob_mm[0]:.1f}, {rob_mm[1]:.1f}, {rob_mm[2]:.1f})",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
                    else:
                        cv2.putText(img, "Depth invalid at marker",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            if cam_xyz is None:
                # 不清空 predicted_robot_mm，保留最后一次有效预测用于对比
                cv2.putText(img, f"ArUco ID={target_id} not found", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                if predicted_robot_mm is not None:
                    p = predicted_robot_mm
                    cv2.putText(img,
                        f"Last predict(mm): ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

            # 显示上次对比结果
            if last_compare is not None:
                pred, actual, dist = last_compare
                color = (0, 255, 0) if dist < 10 else (0, 165, 255) if dist < 20 else (0, 0, 255)
                quality = "Good" if dist < 10 else "OK" if dist < 20 else "Bad"
                cv2.putText(img,
                    f"Last test: dist={dist:.1f}mm [{quality}]",
                    (10, img.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.putText(img,
                    f"  dX={actual[0]-pred[0]:.1f} dY={actual[1]-pred[1]:.1f} dZ={actual[2]-pred[2]:.1f}",
                    (10, img.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            cv2.putText(img, "m:above  g:go  r:compare  q:quit", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.imshow("Calibration Verify", img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            if key == ord("m") and predicted_robot_mm is not None:
                x, y, z = predicted_robot_mm
                target_z = max(z + APPROACH_HEIGHT_MM, SAFE_Z_MM)
                cur_rx, cur_ry, cur_rz = get_current_orientation(robot)
                pose = [x, y, target_z, cur_rx, cur_ry, cur_rz]
                last_move_target = [x, y, target_z]
                print(f"[安全移动到上方] target=X={x:.1f} Y={y:.1f} Z={target_z:.1f} (原始Z={z:.1f})")
                rtn = safe_move(robot, pose)
                print(f"  结果: err={rtn}")

            if key == ord("g") and predicted_robot_mm is not None:
                x, y, z = predicted_robot_mm
                target_z = max(z, SAFE_Z_MM)
                cur_rx, cur_ry, cur_rz = get_current_orientation(robot)
                pose = [x, y, target_z, cur_rx, cur_ry, cur_rz]
                last_move_target = [x, y, target_z]
                print(f"[安全移动到位置] target=X={x:.1f} Y={y:.1f} Z={target_z:.1f} (原始Z={z:.1f})")
                rtn = safe_move(robot, pose)
                print(f"  结果: err={rtn}")

            if key == ord("r"):
                ret = robot.GetActualTCPPose()
                if isinstance(ret, tuple) and len(ret) == 2 and ret[0] == 0:
                    actual = ret[1]
                    print(f"\n[机械臂实际位姿] X={actual[0]:.1f} Y={actual[1]:.1f} Z={actual[2]:.1f}")

                    # 优先和实际发送的目标坐标对比（最准确）
                    if last_move_target is not None:
                        tgt = last_move_target
                        dx = actual[0] - tgt[0]
                        dy = actual[1] - tgt[1]
                        dz = actual[2] - tgt[2]
                        dist_xy = float(np.sqrt(dx**2 + dy**2))
                        dist_3d = float(np.sqrt(dx**2 + dy**2 + dz**2))
                        last_compare = (tgt, [actual[0], actual[1], actual[2]], dist_xy)
                        print(f"  目标(mm): X={tgt[0]:.1f} Y={tgt[1]:.1f} Z={tgt[2]:.1f}")
                        print(f"  偏差: dX={dx:.1f} dY={dy:.1f} dZ={dz:.1f}")
                        print(f"  XY距离={dist_xy:.1f}mm | 3D距离={dist_3d:.1f}mm")
                        if dist_xy < 10:
                            print("  >> XY偏差 < 10mm，标定良好!")
                        elif dist_xy < 20:
                            print("  >> XY偏差 10-20mm，标定可用")
                        else:
                            print("  >> XY偏差 > 20mm，建议重新标定")
                    elif predicted_robot_mm is not None:
                        pred = predicted_robot_mm
                        dx = actual[0] - pred[0]
                        dy = actual[1] - pred[1]
                        dist_xy = float(np.sqrt(dx**2 + dy**2))
                        print(f"  预测(mm): X={pred[0]:.1f} Y={pred[1]:.1f} Z={pred[2]:.1f}")
                        print(f"  XY偏差: dX={dx:.1f} dY={dy:.1f} | XY距离={dist_xy:.1f}mm")
                        print(f"  (Z不可比: 机器人在安全高度，预测Z={pred[2]:.1f})")
                    else:
                        print("  无预测数据，请先让相机看到 ArUco 标记")
                else:
                    print(f"读取位姿失败: {ret}")

    finally:
        robot.CloseRPC()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
