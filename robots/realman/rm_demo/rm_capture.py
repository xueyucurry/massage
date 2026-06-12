from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np

from .config import (
    DEFAULT_CAPTURE_FPS,
    DEFAULT_CAPTURE_HEIGHT,
    DEFAULT_CAPTURE_WARMUP_FRAMES,
    DEFAULT_CAPTURE_WIDTH,
)


@dataclass
class CapturedFrame:
    timestamp: str
    color_bgr: np.ndarray
    depth_m: np.ndarray
    intrinsics: dict[str, object]
    color_path: str
    depth_path: str
    intrinsics_path: str


def _import_rs():
    import pyrealsense2 as rs  # type: ignore

    return rs


def _import_ros_capture_modules():
    candidates = ["/home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/lib/python3/dist-packages"]
    for root in ("/opt/ros",):
        if os.path.isdir(root):
            for name in os.listdir(root):
                candidates.append(os.path.join(root, name, "lib", "python3", "dist-packages"))
    for candidate in candidates:
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.append(candidate)

    import rospy  # type: ignore
    from cv_bridge import CvBridge  # type: ignore
    from sensor_msgs.msg import CameraInfo, Image  # type: ignore

    return rospy, CvBridge, CameraInfo, Image


def has_realsense_device() -> bool:
    try:
        rs = _import_rs()
        ctx = rs.context()
        return len(ctx.devices) > 0
    except Exception:
        return False


def rs_intrinsics_to_dict(intrinsics, depth_scale: float) -> dict[str, object]:
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "ppx": float(intrinsics.ppx),
        "ppy": float(intrinsics.ppy),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "model_name": str(intrinsics.model).split(".")[-1],
        "coeffs": [float(v) for v in intrinsics.coeffs],
        "depth_scale": float(depth_scale),
    }


def dict_to_rs_intrinsics(data: dict[str, object]):
    rs = _import_rs()
    intr = rs.intrinsics()
    intr.width = int(data["width"])
    intr.height = int(data["height"])
    intr.ppx = float(data["ppx"])
    intr.ppy = float(data["ppy"])
    intr.fx = float(data["fx"])
    intr.fy = float(data["fy"])
    model_name = str(data.get("model_name", "none"))
    intr.model = getattr(rs.distortion, model_name, rs.distortion.none)
    coeffs = list(data.get("coeffs", [0, 0, 0, 0, 0]))
    intr.coeffs = [float(v) for v in coeffs[:5]]
    return intr


def ensure_output_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


def _capture_single_frame_via_ros(output_dir: str) -> CapturedFrame:
    rospy, CvBridge, CameraInfo, Image = _import_ros_capture_modules()
    if not rospy.core.is_initialized():
        rospy.init_node("rm_demo_capture", anonymous=True, disable_signals=True)

    ensure_output_dir(output_dir)
    bridge = CvBridge()
    cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=8.0)
    color_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=8.0)
    depth_msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=8.0)

    color_bgr = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
    depth_raw = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    if depth_raw.dtype == np.uint16:
        depth_scale = 0.001
        depth_m = depth_raw.astype(np.float32) * depth_scale
    else:
        depth_scale = 1.0
        depth_m = depth_raw.astype(np.float32)

    distortion_model = str(getattr(cam_info, "distortion_model", "none")).lower().strip()
    if distortion_model in ("plumb_bob", "brown_conrady"):
        model_name = "brown_conrady"
    elif distortion_model in ("inverse_brown_conrady",):
        model_name = "inverse_brown_conrady"
    else:
        model_name = "none"
    intrinsics = {
        "width": int(cam_info.width),
        "height": int(cam_info.height),
        "ppx": float(cam_info.K[2]),
        "ppy": float(cam_info.K[5]),
        "fx": float(cam_info.K[0]),
        "fy": float(cam_info.K[4]),
        "model_name": model_name,
        "coeffs": [float(v) for v in cam_info.D[:5]] + [0.0] * max(0, 5 - len(cam_info.D[:5])),
        "depth_scale": float(depth_scale),
    }
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = f"capture_{timestamp}"
    color_path, depth_path, intrinsics_path = save_capture_artifacts(
        output_dir=output_dir,
        color_bgr=color_bgr,
        depth_m=depth_m,
        intrinsics=intrinsics,
        prefix=prefix,
    )
    return CapturedFrame(
        timestamp=timestamp,
        color_bgr=color_bgr,
        depth_m=depth_m,
        intrinsics=intrinsics,
        color_path=color_path,
        depth_path=depth_path,
        intrinsics_path=intrinsics_path,
    )


def save_capture_artifacts(
    output_dir: str,
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: dict[str, object],
    prefix: str,
) -> tuple[str, str, str]:
    ensure_output_dir(output_dir)
    color_path = os.path.join(output_dir, f"{prefix}_rgb.png")
    depth_path = os.path.join(output_dir, f"{prefix}_depth.npy")
    intrinsics_path = os.path.join(output_dir, f"{prefix}_intrinsics.json")
    cv2.imwrite(color_path, color_bgr)
    np.save(depth_path, depth_m)
    with open(intrinsics_path, "w", encoding="utf-8") as f:
        json.dump(intrinsics, f, ensure_ascii=False, indent=2)
    return color_path, depth_path, intrinsics_path


def capture_single_frame(
    output_dir: str,
    width: int = DEFAULT_CAPTURE_WIDTH,
    height: int = DEFAULT_CAPTURE_HEIGHT,
    fps: int = DEFAULT_CAPTURE_FPS,
    warmup_frames: int = DEFAULT_CAPTURE_WARMUP_FRAMES,
) -> CapturedFrame:
    try:
        rs = _import_rs()
    except Exception:
        return _capture_single_frame_via_ros(output_dir)

    ensure_output_dir(output_dir)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, int(width), int(height), rs.format.z16, int(fps))
    config.enable_stream(rs.stream.color, int(width), int(height), rs.format.bgr8, int(fps))
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = float(depth_sensor.get_depth_scale())
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    try:
        for _ in range(max(1, int(warmup_frames))):
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()
        frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("failed to capture aligned color/depth frame")

        intrinsics = rs_intrinsics_to_dict(
            profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics(),
            depth_scale=depth_scale,
        )
        color_bgr = np.asanyarray(color_frame.get_data()).copy()
        depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_m = depth_raw * depth_scale
        prefix = f"capture_{timestamp}"
        color_path, depth_path, intrinsics_path = save_capture_artifacts(
            output_dir=output_dir,
            color_bgr=color_bgr,
            depth_m=depth_m,
            intrinsics=intrinsics,
            prefix=prefix,
        )
        return CapturedFrame(
            timestamp=timestamp,
            color_bgr=color_bgr,
            depth_m=depth_m,
            intrinsics=intrinsics,
            color_path=color_path,
            depth_path=depth_path,
            intrinsics_path=intrinsics_path,
        )
    finally:
        pipeline.stop()


def load_captured_frame(
    color_path: str,
    depth_path: str,
    intrinsics_path: str,
) -> CapturedFrame:
    color_bgr = cv2.imread(color_path, cv2.IMREAD_COLOR)
    if color_bgr is None:
        raise FileNotFoundError(f"failed to read color image: {color_path}")
    depth_m = np.load(depth_path)
    with open(intrinsics_path, "r", encoding="utf-8") as f:
        intrinsics = json.load(f)
    base = os.path.splitext(os.path.basename(color_path))[0]
    timestamp = base.replace("_rgb", "")
    return CapturedFrame(
        timestamp=timestamp,
        color_bgr=color_bgr,
        depth_m=depth_m,
        intrinsics=intrinsics,
        color_path=os.path.abspath(color_path),
        depth_path=os.path.abspath(depth_path),
        intrinsics_path=os.path.abspath(intrinsics_path),
    )
