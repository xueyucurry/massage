#!/home/franka/anaconda3/envs/llamauav/bin/python
import datetime
import os

import cv2
import numpy as np
import pyrealsense2 as rs


def colorize_depth(depth_image: np.ndarray, max_depth_mm: float = 3000.0) -> np.ndarray:
    depth_mm = depth_image.astype(np.float32)
    valid = depth_mm > 0
    vis = np.zeros_like(depth_mm, dtype=np.uint8)
    if np.any(valid):
        clipped = np.clip(depth_mm, 0.0, max_depth_mm)
        vis[valid] = np.round(clipped[valid] / max_depth_mm * 255.0).astype(np.uint8)
    colored = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    colored[~valid] = 0
    return colored


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    align = rs.align(rs.stream.color)

    try:
        profile = pipeline.start(config)
    except Exception as e:
        raise RuntimeError(
            "RealSense 启动失败，可能是设备未连接或已被其他程序占用。"
        ) from e

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    save_dir = os.path.join(os.path.dirname(__file__), "realsense_snapshots")
    os.makedirs(save_dir, exist_ok=True)

    print("RealSense 实时预览已启动")
    print("按 q / ESC 退出，按 s 保存当前彩色图和深度图")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = colorize_depth(depth_image)

            center_y = color_image.shape[0] // 2
            center_x = color_image.shape[1] // 2
            center_depth_m = float(depth_image[center_y, center_x]) * depth_scale

            cv2.circle(color_image, (center_x, center_y), 4, (0, 255, 255), -1)
            cv2.circle(depth_colormap, (center_x, center_y), 4, (255, 255, 255), -1)
            cv2.putText(
                color_image,
                f"Center depth: {center_depth_m:.3f} m",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            combined = np.hstack([color_image, depth_colormap])
            cv2.imshow("RealSense Live | Color + Depth", combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s"):
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                color_path = os.path.join(save_dir, f"color_{ts}.png")
                depth_path = os.path.join(save_dir, f"depth_{ts}.png")
                depth_vis_path = os.path.join(save_dir, f"depth_vis_{ts}.png")
                cv2.imwrite(color_path, color_image)
                cv2.imwrite(depth_path, depth_image)
                cv2.imwrite(depth_vis_path, depth_colormap)
                print(f"[Saved] {color_path}")
                print(f"[Saved] {depth_path}")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
