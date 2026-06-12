#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from rm_demo.config import DEFAULT_HOST, ROS_VENDOR_PYTHON_DIR  # noqa: E402


def _append_ros_python_candidates() -> None:
    candidates = [ROS_VENDOR_PYTHON_DIR]
    for root in ("/opt/ros",):
        if os.path.isdir(root):
            for name in os.listdir(root):
                candidates.append(os.path.join(root, name, "lib", "python3", "dist-packages"))
    extra = os.environ.get("PYTHONPATH", "")
    if extra:
        candidates.extend(path for path in extra.split(":") if path)
    for candidate in candidates:
        if candidate and os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.append(candidate)


@dataclass
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    coeffs: list[float]
    model_name: str = "none"
    depth_scale: float = 1.0

    @property
    def k(self) -> np.ndarray:
        return np.asarray(
            [[self.fx, 0.0, self.ppx], [0.0, self.fy, self.ppy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @property
    def d(self) -> np.ndarray:
        coeffs = list(self.coeffs[:5]) + [0.0] * max(0, 5 - len(self.coeffs))
        return np.asarray(coeffs[:5], dtype=np.float64)


@dataclass
class RgbdFrame:
    color_bgr: np.ndarray
    depth_m: np.ndarray
    intrinsics: CameraIntrinsics
    stamp: float


@dataclass
class ArucoObservation:
    marker_id: int
    dictionary: str
    center_px: list[float]
    corners_px: list[list[float]]
    camera_xyz_m: list[float]
    camera_xyz_method: str
    pnp_tvec_m: list[float] | None
    pnp_rvec: list[float] | None
    camera_from_marker_matrix: list[list[float]] | None


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def matrix_to_rpy(rot: np.ndarray) -> list[float]:
    m = np.asarray(rot, dtype=np.float64)
    sy = max(-1.0, min(1.0, -float(m[2, 0])))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) > 1e-6:
        roll = math.atan2(float(m[2, 1]), float(m[2, 2]))
        yaw = math.atan2(float(m[1, 0]), float(m[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(m[0, 1]), float(m[1, 1]))
    return [float(roll), float(pitch), float(yaw)]


def pose_to_matrix(pose_xyzrpy: list[float]) -> np.ndarray:
    if len(pose_xyzrpy) < 6:
        raise RuntimeError(f"pose must contain 6 values, got {len(pose_xyzrpy)}")
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rpy_to_matrix(float(pose_xyzrpy[3]), float(pose_xyzrpy[4]), float(pose_xyzrpy[5]))
    out[:3, 3] = [float(v) for v in pose_xyzrpy[:3]]
    return out


def make_hmat(rotation: np.ndarray, translation: np.ndarray | list[float]) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    out[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return out


def invert_hmat(mat: np.ndarray) -> np.ndarray:
    m = np.asarray(mat, dtype=np.float64).reshape(4, 4)
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = m[:3, :3].T
    out[:3, 3] = -(m[:3, :3].T @ m[:3, 3])
    return out


def rotation_angle_error_rad(a: np.ndarray, b: np.ndarray) -> float:
    rel = np.asarray(a, dtype=np.float64).T @ np.asarray(b, dtype=np.float64)
    cos_angle = (float(np.trace(rel)) - 1.0) * 0.5
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return float(math.acos(cos_angle))


def average_rotations(rotations: list[np.ndarray]) -> np.ndarray:
    if not rotations:
        return np.eye(3, dtype=np.float64)
    accum = np.zeros((3, 3), dtype=np.float64)
    for rot in rotations:
        accum += np.asarray(rot, dtype=np.float64).reshape(3, 3)
    u, _s, vt = np.linalg.svd(accum)
    avg = u @ vt
    if np.linalg.det(avg) < 0.0:
        u[:, -1] *= -1.0
        avg = u @ vt
    return avg


def pixel_to_camera(u: float, v: float, z_m: float, intr: CameraIntrinsics) -> np.ndarray:
    return np.asarray(
        [
            (float(u) - intr.ppx) * float(z_m) / intr.fx,
            (float(v) - intr.ppy) * float(z_m) / intr.fy,
            float(z_m),
        ],
        dtype=np.float64,
    )


def sample_depth_m(depth_m: np.ndarray, u: float, v: float, radius: int, min_m: float, max_m: float) -> float | None:
    h, w = depth_m.shape[:2]
    cu, cv = int(round(float(u))), int(round(float(v)))
    x0, x1 = max(0, cu - radius), min(w, cu + radius + 1)
    y0, y1 = max(0, cv - radius), min(h, cv + radius + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    vals = depth_m[y0:y1, x0:x1]
    vals = vals[np.isfinite(vals) & (vals >= float(min_m)) & (vals <= float(max_m))]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def estimate_rigid_transform(camera_pts: list[list[float]], base_pts: list[list[float]]) -> tuple[np.ndarray, float, np.ndarray]:
    cam = np.asarray(camera_pts, dtype=np.float64)
    base = np.asarray(base_pts, dtype=np.float64)
    if cam.shape != base.shape:
        raise RuntimeError(f"point shape mismatch: camera={cam.shape}, base={base.shape}")
    if cam.shape[0] < 4:
        raise RuntimeError("at least 4 point pairs are required")

    cam_mean = cam.mean(axis=0)
    base_mean = base.mean(axis=0)
    cam_centered = cam - cam_mean
    base_centered = base - base_mean
    h = cam_centered.T @ base_centered
    u, _s, vt = np.linalg.svd(h)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0.0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    trans = base_mean - rot @ cam_mean

    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[:3, 3] = trans
    cam_h = np.hstack([cam, np.ones((cam.shape[0], 1), dtype=np.float64)])
    fitted = (mat @ cam_h.T).T[:, :3]
    rmse = float(np.sqrt(np.mean(np.sum((fitted - base) ** 2, axis=1))))
    return mat, rmse, fitted


def compute_errors(camera_pts: list[list[float]], base_pts: list[list[float]], mat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    cam = np.asarray(camera_pts, dtype=np.float64)
    base = np.asarray(base_pts, dtype=np.float64)
    cam_h = np.hstack([cam, np.ones((cam.shape[0], 1), dtype=np.float64)])
    fitted = (np.asarray(mat, dtype=np.float64) @ cam_h.T).T[:, :3]
    return np.linalg.norm(fitted - base, axis=1), fitted


def robust_fit(camera_pts: list[list[float]], base_pts: list[list[float]], min_inliers: int) -> tuple[np.ndarray, float, list[int], list[int], float]:
    mat0, rmse0, _ = estimate_rigid_transform(camera_pts, base_pts)
    errs0, _ = compute_errors(camera_pts, base_pts, mat0)
    median = float(np.median(errs0))
    mad = float(np.median(np.abs(errs0 - median)))
    sigma = 1.4826 * mad
    threshold = max(0.005, median + 3.0 * sigma)
    inliers = [idx for idx, err in enumerate(errs0.tolist()) if float(err) <= threshold]
    outliers = [idx for idx, err in enumerate(errs0.tolist()) if float(err) > threshold]
    if len(inliers) >= int(min_inliers):
        cam_in = [camera_pts[i] for i in inliers]
        base_in = [base_pts[i] for i in inliers]
        mat1, rmse1, _ = estimate_rigid_transform(cam_in, base_in)
        if rmse1 <= rmse0:
            return mat1, rmse1, inliers, outliers, threshold
    return mat0, rmse0, list(range(len(camera_pts))), [], threshold


def axis_span(points: list[list[float]]) -> tuple[list[float], list[float], list[float]]:
    arr = np.asarray(points, dtype=np.float64)
    mins = arr.min(axis=0)
    maxs = arr.max(axis=0)
    return mins.tolist(), maxs.tolist(), (maxs - mins).tolist()


def create_aruco_detector(dictionary_name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV does not include aruco; install opencv-contrib-python")
    if not hasattr(cv2.aruco, dictionary_name):
        raise RuntimeError(f"unsupported aruco dictionary: {dictionary_name}")
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    else:
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)

    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)

        def detect(gray: np.ndarray):
            return detector.detectMarkers(gray)

    else:

        def detect(gray: np.ndarray):
            return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    return detect


def choose_marker(corners: Any, ids: Any, marker_id: int) -> tuple[int, np.ndarray] | None:
    if ids is None or len(ids) == 0:
        return None
    ids_flat = [int(v) for v in np.asarray(ids).reshape(-1).tolist()]
    if marker_id >= 0:
        if marker_id not in ids_flat:
            return None
        idx = ids_flat.index(marker_id)
    else:
        areas = []
        for item in corners:
            pts = np.asarray(item, dtype=np.float64).reshape(4, 2)
            areas.append(abs(float(cv2.contourArea(pts.astype(np.float32)))))
        idx = int(np.argmax(areas))
    return ids_flat[idx], np.asarray(corners[idx], dtype=np.float64).reshape(4, 2)


class ArucoLocator:
    def __init__(
        self,
        *,
        dictionary_name: str,
        marker_id: int,
        marker_size_m: float,
        depth_radius_px: int,
        min_depth_m: float,
        max_depth_m: float,
        allow_pnp_fallback: bool,
    ) -> None:
        self.dictionary_name = str(dictionary_name)
        self.marker_id = int(marker_id)
        self.marker_size_m = float(marker_size_m)
        self.depth_radius_px = int(depth_radius_px)
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.allow_pnp_fallback = bool(allow_pnp_fallback)
        self._detect = create_aruco_detector(self.dictionary_name)

    def locate(self, frame: RgbdFrame) -> tuple[ArucoObservation | None, Any, Any]:
        gray = cv2.cvtColor(frame.color_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detect(gray)
        chosen = choose_marker(corners, ids, self.marker_id)
        if chosen is None:
            return None, corners, ids

        chosen_id, marker_corners = chosen
        center_u = float(np.mean(marker_corners[:, 0]))
        center_v = float(np.mean(marker_corners[:, 1]))
        pnp_rvec: list[float] | None = None
        pnp_tvec: list[float] | None = None
        camera_from_marker: np.ndarray | None = None
        if self.marker_size_m > 0.0:
            try:
                rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                    marker_corners.reshape(1, 4, 2),
                    self.marker_size_m,
                    frame.intrinsics.k,
                    frame.intrinsics.d,
                )
                pnp_rvec = [float(v) for v in rvecs[0][0].tolist()]
                pnp_tvec = [float(v) for v in tvecs[0][0].tolist()]
                rot, _jac = cv2.Rodrigues(np.asarray(pnp_rvec, dtype=np.float64))
                camera_from_marker = make_hmat(rot, np.asarray(pnp_tvec, dtype=np.float64))
            except Exception:
                pnp_rvec = None
                pnp_tvec = None
                camera_from_marker = None

        z_m = sample_depth_m(
            frame.depth_m,
            center_u,
            center_v,
            radius=self.depth_radius_px,
            min_m=self.min_depth_m,
            max_m=self.max_depth_m,
        )
        if z_m is not None:
            cam_xyz = pixel_to_camera(center_u, center_v, z_m, frame.intrinsics)
            method = "aligned_depth_center"
        elif self.allow_pnp_fallback and pnp_tvec is not None:
            cam_xyz = np.asarray(pnp_tvec, dtype=np.float64)
            method = "aruco_pnp_fallback"
        else:
            return None, corners, ids

        return (
            ArucoObservation(
                marker_id=int(chosen_id),
                dictionary=self.dictionary_name,
                center_px=[center_u, center_v],
                corners_px=[[float(x), float(y)] for x, y in marker_corners.tolist()],
                camera_xyz_m=[float(v) for v in cam_xyz.tolist()],
                camera_xyz_method=method,
                pnp_tvec_m=pnp_tvec,
                pnp_rvec=pnp_rvec,
                camera_from_marker_matrix=None if camera_from_marker is None else [[float(v) for v in row] for row in camera_from_marker.tolist()],
            ),
            corners,
            ids,
        )

    def draw(self, frame: RgbdFrame, observation: ArucoObservation | None, corners: Any, ids: Any) -> np.ndarray:
        canvas = frame.color_bgr.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(canvas, corners, ids)
        if observation is not None and observation.pnp_rvec is not None and observation.pnp_tvec_m is not None:
            try:
                cv2.drawFrameAxes(
                    canvas,
                    frame.intrinsics.k,
                    frame.intrinsics.d,
                    np.asarray(observation.pnp_rvec, dtype=np.float64),
                    np.asarray(observation.pnp_tvec_m, dtype=np.float64),
                    max(0.01, self.marker_size_m * 0.6),
                    2,
                )
            except Exception:
                pass
        return canvas


class OpenCvSource:
    def __init__(
        self,
        *,
        device: int,
        width: int,
        height: int,
        fps: int,
        fx: float,
        fy: float,
        ppx: float,
        ppy: float,
        fov_deg: float,
        intrinsics_json: str,
        intrinsics_from_realsense: bool,
        realsense_serial: str,
    ) -> None:
        self.device = int(device)
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open /dev/video{self.device}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self.cap.set(cv2.CAP_PROP_FPS, int(fps))

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.cap.release()
            raise RuntimeError(f"opened /dev/video{self.device}, but failed to read a frame")
        self.height, self.width = int(frame.shape[0]), int(frame.shape[1])
        self._last_frame = frame

        if bool(intrinsics_from_realsense):
            self.intrinsics = resolve_realsense_color_intrinsics(
                width=self.width,
                height=self.height,
                fps=int(fps),
                serial=str(realsense_serial or "").strip(),
            )
            print(
                "[camera] loaded RealSense color intrinsics for OpenCV frames "
                f"{self.intrinsics.width}x{self.intrinsics.height}",
                flush=True,
            )
        elif str(intrinsics_json or "").strip():
            with open(str(intrinsics_json), "r", encoding="utf-8") as f:
                data = json.load(f)
            coeffs = [float(v) for v in list(data.get("coeffs", [0, 0, 0, 0, 0]))[:5]]
            coeffs += [0.0] * max(0, 5 - len(coeffs))
            self.intrinsics = CameraIntrinsics(
                width=int(data.get("width", self.width)),
                height=int(data.get("height", self.height)),
                fx=float(data["fx"]),
                fy=float(data["fy"]),
                ppx=float(data.get("ppx", data.get("cx", self.width * 0.5))),
                ppy=float(data.get("ppy", data.get("cy", self.height * 0.5))),
                coeffs=coeffs,
                model_name=str(data.get("model_name", "none")),
                depth_scale=1.0,
            )
            print(f"[camera] loaded OpenCV intrinsics from {intrinsics_json}", flush=True)
        else:
            if fx > 0.0 and fy > 0.0:
                fx_used = float(fx)
                fy_used = float(fy)
            else:
                fov_rad = math.radians(float(fov_deg))
                fx_used = float(self.width) / (2.0 * math.tan(fov_rad * 0.5))
                fy_used = fx_used
                print(
                    "[warn] no camera intrinsics supplied; using approximate "
                    f"fx=fy={fx_used:.1f} from fov={fov_deg:.1f} deg. "
                    "Use real intrinsics for final calibration.",
                    flush=True,
                )
            self.intrinsics = CameraIntrinsics(
                width=self.width,
                height=self.height,
                fx=fx_used,
                fy=fy_used,
                ppx=float(ppx) if ppx > 0.0 else float(self.width) * 0.5,
                ppy=float(ppy) if ppy > 0.0 else float(self.height) * 0.5,
                coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
                model_name="none",
                depth_scale=1.0,
            )

        print(f"[camera] opened /dev/video{self.device} frame={self.width}x{self.height}", flush=True)

    def read(self, timeout_s: float = 2.0) -> RgbdFrame:
        del timeout_s
        ok, frame = self.cap.read()
        if not ok or frame is None:
            frame = self._last_frame
        else:
            self._last_frame = frame
        depth_m = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.float32)
        return RgbdFrame(color_bgr=frame.copy(), depth_m=depth_m, intrinsics=self.intrinsics, stamp=time.time())

    def close(self) -> None:
        self.cap.release()


def resolve_realsense_color_intrinsics(*, width: int, height: int, fps: int, serial: str = "") -> CameraIntrinsics:
    import pyrealsense2 as rs  # type: ignore

    pipeline = rs.pipeline()
    formats = [rs.format.bgr8, rs.format.rgb8, rs.format.yuyv, rs.format.bgra8, rs.format.rgba8]
    fps_candidates = []
    for candidate in (int(fps), 30, 15, 6):
        if candidate > 0 and candidate not in fps_candidates:
            fps_candidates.append(candidate)

    last_error: Exception | None = None
    for fmt in formats:
        for fps_candidate in fps_candidates:
            config = rs.config()
            if serial:
                config.enable_device(serial)
            try:
                config.enable_stream(rs.stream.color, int(width), int(height), fmt, int(fps_candidate))
                profile = config.resolve(pipeline)
                stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
                intr = stream.get_intrinsics()
                return CameraIntrinsics(
                    width=int(intr.width),
                    height=int(intr.height),
                    fx=float(intr.fx),
                    fy=float(intr.fy),
                    ppx=float(intr.ppx),
                    ppy=float(intr.ppy),
                    coeffs=[float(v) for v in intr.coeffs],
                    model_name=str(intr.model).split(".")[-1],
                    depth_scale=1.0,
                )
            except Exception as exc:
                last_error = exc
    detail = f": {last_error}" if last_error else ""
    raise RuntimeError(f"failed to resolve RealSense color intrinsics for {width}x{height}{detail}")


class RealsenseSource:
    def __init__(self, width: int, height: int, fps: int, warmup_frames: int) -> None:
        import pyrealsense2 as rs  # type: ignore

        self.rs = rs
        self.pipeline = None
        self.color_format = None
        last_error: Exception | None = None
        color_formats = [rs.format.rgb8, rs.format.bgr8]
        for color_format in color_formats:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, int(width), int(height), rs.format.z16, int(fps))
            config.enable_stream(rs.stream.color, int(width), int(height), color_format, int(fps))
            try:
                profile = pipeline.start(config)
                # Prove the selected profile actually delivers synchronized frames.
                for _ in range(max(1, int(warmup_frames))):
                    frames = pipeline.wait_for_frames(timeout_ms=8000)
                    if frames.get_color_frame() and frames.get_depth_frame():
                        break
                else:
                    raise RuntimeError("RealSense started but did not deliver RGBD frames")
                self.pipeline = pipeline
                self.profile = profile
                self.color_format = color_format
                break
            except Exception as exc:
                last_error = exc
                try:
                    pipeline.stop()
                except Exception:
                    pass
        if self.pipeline is None:
            detail = f": {last_error}" if last_error else ""
            raise RuntimeError(f"failed to start RealSense RGBD stream{detail}")
        self.align = rs.align(rs.stream.color)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())
        color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        self.intrinsics = CameraIntrinsics(
            width=int(intr.width),
            height=int(intr.height),
            fx=float(intr.fx),
            fy=float(intr.fy),
            ppx=float(intr.ppx),
            ppy=float(intr.ppy),
            coeffs=[float(v) for v in intr.coeffs],
            model_name=str(intr.model).split(".")[-1],
            depth_scale=self.depth_scale,
        )
        print(
            "[camera] RealSense RGBD frame=%dx%d fps=%d color_format=%s"
            % (int(intr.width), int(intr.height), int(fps), str(self.color_format).split(".")[-1]),
            flush=True,
        )

    def read(self, timeout_s: float = 2.0) -> RgbdFrame:
        frames = self.pipeline.wait_for_frames(timeout_ms=int(float(timeout_s) * 1000.0))
        frames = self.align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("failed to capture aligned RealSense RGBD frame")
        color_raw = np.asanyarray(color_frame.get_data())
        if self.color_format == self.rs.format.rgb8:
            color = cv2.cvtColor(color_raw, cv2.COLOR_RGB2BGR)
        else:
            color = color_raw.copy()
        depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
        return RgbdFrame(color_bgr=color, depth_m=depth, intrinsics=self.intrinsics, stamp=time.time())

    def close(self) -> None:
        if self.pipeline is not None:
            self.pipeline.stop()


class RosRgbdSource:
    def __init__(
        self,
        *,
        color_topic: str,
        depth_topic: str,
        camera_info_topic: str,
    ) -> None:
        _append_ros_python_candidates()
        import rospy  # type: ignore
        from cv_bridge import CvBridge  # type: ignore
        from sensor_msgs.msg import CameraInfo, Image  # type: ignore

        if not rospy.core.is_initialized():
            rospy.init_node("rm_aruco_external_camera_calib", anonymous=True, disable_signals=True)
        self.rospy = rospy
        self.bridge = CvBridge()
        self.CameraInfo = CameraInfo
        self.Image = Image
        self.color_topic = str(color_topic)
        self.depth_topic = str(depth_topic)
        self.camera_info_topic = str(camera_info_topic)
        self.intrinsics: CameraIntrinsics | None = None

    def _intrinsics_from_msg(self, msg: Any) -> CameraIntrinsics:
        distortion_model = str(getattr(msg, "distortion_model", "none")).lower().strip()
        if distortion_model in ("plumb_bob", "brown_conrady"):
            model_name = "brown_conrady"
        elif distortion_model == "inverse_brown_conrady":
            model_name = "inverse_brown_conrady"
        else:
            model_name = "none"
        coeffs = [float(v) for v in list(getattr(msg, "D", []))[:5]]
        coeffs += [0.0] * max(0, 5 - len(coeffs))
        return CameraIntrinsics(
            width=int(msg.width),
            height=int(msg.height),
            fx=float(msg.K[0]),
            fy=float(msg.K[4]),
            ppx=float(msg.K[2]),
            ppy=float(msg.K[5]),
            coeffs=coeffs,
            model_name=model_name,
            depth_scale=1.0,
        )

    def read(self, timeout_s: float = 3.0) -> RgbdFrame:
        if self.intrinsics is None:
            info = self.rospy.wait_for_message(self.camera_info_topic, self.CameraInfo, timeout=float(timeout_s))
            self.intrinsics = self._intrinsics_from_msg(info)
        color_msg = self.rospy.wait_for_message(self.color_topic, self.Image, timeout=float(timeout_s))
        depth_msg = self.rospy.wait_for_message(self.depth_topic, self.Image, timeout=float(timeout_s))
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) * 0.001
        else:
            depth_m = depth_raw.astype(np.float32)
        return RgbdFrame(color_bgr=color, depth_m=depth_m, intrinsics=self.intrinsics, stamp=time.time())

    def close(self) -> None:
        return None


class RobotPoseReader:
    def __init__(self, *, backend: str, host: str, tool_name: str, allow_arm_error: bool) -> None:
        self.backend = str(backend).strip().lower()
        self.host = str(host)
        self.tool_name = str(tool_name).strip()
        self.allow_arm_error = bool(allow_arm_error)
        self.bridge = None
        if self.backend == "ros":
            from rm_demo.rm_ros import RosArmBridge

            self.bridge = RosArmBridge()
            if self.tool_name:
                self.bridge.change_tool(self.tool_name)
        elif self.backend == "json":
            from rm_demo import rm_json

            self.rm_json = rm_json
        else:
            raise RuntimeError(f"unsupported arm backend: {backend}")

    def read(self) -> dict[str, Any]:
        if self.backend == "ros":
            assert self.bridge is not None
            joints, pose, arm_err, sys_err, inverse_km_err = self.bridge.get_current_arm_state(self.host, timeout=2.0)
        else:
            joints, pose, arm_err, sys_err, inverse_km_err = self.rm_json.get_current_arm_state(self.host)
        if not self.allow_arm_error and (int(arm_err) != 0 or int(sys_err) != 0):
            raise RuntimeError(f"arm state has errors: arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}")
        return {
            "joints_deg": [float(v) for v in joints[:6]],
            "pose_m_rad": [float(v) for v in pose[:6]],
            "arm_err": int(arm_err),
            "sys_err": int(sys_err),
            "inverse_km_err": int(inverse_km_err),
        }


def backup_existing_file(path: Path) -> Path | None:
    if not path.exists():
        return None
    backup = path.with_name(path.name + "." + datetime.now().strftime("%Y%m%d_%H%M%S") + ".bak")
    shutil.copy2(path, backup)
    return backup


def save_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def handeye_method_value(name: str) -> int:
    key = str(name or "park").strip().lower()
    mapping = {
        "tsai": getattr(cv2, "CALIB_HAND_EYE_TSAI"),
        "park": getattr(cv2, "CALIB_HAND_EYE_PARK"),
        "horaud": getattr(cv2, "CALIB_HAND_EYE_HORAUD"),
        "andreff": getattr(cv2, "CALIB_HAND_EYE_ANDREFF"),
        "daniilidis": getattr(cv2, "CALIB_HAND_EYE_DANIILIDIS"),
    }
    if key not in mapping:
        raise RuntimeError(f"unsupported handeye method: {name}")
    return int(mapping[key])


def fit_handeye_and_save(samples: list[dict[str, Any]], args: argparse.Namespace, *, force: bool = False) -> bool:
    if len(samples) < int(args.min_pairs):
        print(f"[fit] samples={len(samples)} < min_pairs={args.min_pairs}; continue sampling")
        return False

    valid_samples = [
        sample for sample in samples
        if sample.get("base_from_tool_matrix") is not None and sample.get("camera_from_marker_matrix") is not None
    ]
    if len(valid_samples) < int(args.min_pairs):
        print(
            f"[fit] handeye-valid samples={len(valid_samples)} < min_pairs={args.min_pairs}; "
            "check marker size and ArUco PnP detection"
        )
        return False

    base_tool_mats = [np.asarray(sample["base_from_tool_matrix"], dtype=np.float64).reshape(4, 4) for sample in valid_samples]
    camera_marker_mats = [
        np.asarray(sample["camera_from_marker_matrix"], dtype=np.float64).reshape(4, 4)
        for sample in valid_samples
    ]

    base_positions = [mat[:3, 3].tolist() for mat in base_tool_mats]
    cam_positions = [mat[:3, 3].tolist() for mat in camera_marker_mats]
    base_min, base_max, base_span = axis_span(base_positions)
    cam_min, cam_max, cam_span = axis_span(cam_positions)
    if max(base_span) < float(args.min_span_m) or max(cam_span) < float(args.min_span_m):
        print(f"[fit] sample span too small; camera_span={cam_span}, base_span={base_span}")
        if not force:
            return False

    # OpenCV's eye-to-hand form expects ^gT_b as the first input. Our robot
    # state gives ^bT_g, so each robot pose is inverted. With ^cT_marker from
    # ArUco PnP, calibrateHandEye returns ^bT_c for this configuration.
    tool_base_mats = [invert_hmat(mat) for mat in base_tool_mats]
    r_gripper2base = [mat[:3, :3] for mat in tool_base_mats]
    t_gripper2base = [mat[:3, 3].reshape(3, 1) for mat in tool_base_mats]
    r_target2cam = [mat[:3, :3] for mat in camera_marker_mats]
    t_target2cam = [mat[:3, 3].reshape(3, 1) for mat in camera_marker_mats]

    rot, trans = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=handeye_method_value(args.handeye_method),
    )
    base_from_camera = make_hmat(np.asarray(rot, dtype=np.float64), np.asarray(trans, dtype=np.float64).reshape(3))

    tool_from_marker_mats = [
        invert_hmat(base_from_tool) @ base_from_camera @ camera_from_marker
        for base_from_tool, camera_from_marker in zip(base_tool_mats, camera_marker_mats)
    ]
    avg_tool_from_marker = make_hmat(
        average_rotations([mat[:3, :3] for mat in tool_from_marker_mats]),
        np.mean(np.asarray([mat[:3, 3] for mat in tool_from_marker_mats], dtype=np.float64), axis=0),
    )
    translation_errors = [
        float(np.linalg.norm(mat[:3, 3] - avg_tool_from_marker[:3, 3]))
        for mat in tool_from_marker_mats
    ]
    rotation_errors = [
        rotation_angle_error_rad(avg_tool_from_marker[:3, :3], mat[:3, :3])
        for mat in tool_from_marker_mats
    ]

    rot_rpy = matrix_to_rpy(base_from_camera[:3, :3])
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    output = Path(args.output).resolve()
    backup = backup_existing_file(output)
    result = {
        "timestamp": now,
        "method": "rm_external_camera_handeye_eye_to_hand",
        "equation": "p_base = matrix @ [p_camera_x, p_camera_y, p_camera_z, 1]^T",
        "matrix": [[float(v) for v in row] for row in base_from_camera.tolist()],
        "translation_m": [float(v) for v in base_from_camera[:3, 3].tolist()],
        "rotation_rpy_rad": [float(v) for v in rot_rpy],
        "rotation_rpy_deg": [float(math.degrees(v)) for v in rot_rpy],
        "handeye_method": str(args.handeye_method),
        "point_pairs": len(valid_samples),
        "tool_from_marker_estimated_matrix": [[float(v) for v in row] for row in avg_tool_from_marker.tolist()],
        "tool_from_marker_translation_m": [float(v) for v in avg_tool_from_marker[:3, 3].tolist()],
        "tool_from_marker_rotation_rpy_rad": matrix_to_rpy(avg_tool_from_marker[:3, :3]),
        "residual_translation_rmse_m": float(np.sqrt(np.mean(np.square(translation_errors)))),
        "residual_translation_max_m": float(np.max(translation_errors)),
        "residual_rotation_rmse_rad": float(np.sqrt(np.mean(np.square(rotation_errors)))),
        "residual_rotation_max_rad": float(np.max(rotation_errors)),
        "camera_source": str(args.camera_source),
        "arm_backend": str(args.arm_backend),
        "arm_host": str(args.host),
        "tool_name": str(args.tool_name),
        "aruco_dictionary": str(args.aruco_dict),
        "aruco_id": int(args.marker_id),
        "aruco_size_m": float(args.marker_size_m),
        "camera_span_xyz_m": [float(v) for v in cam_span],
        "base_span_xyz_m": [float(v) for v in base_span],
        "backup_of_previous_output": None if backup is None else str(backup),
        "note": "ArUco marker-to-tool/TCP offset is not required as input; it is estimated as tool_from_marker.",
    }
    save_json(output, result)

    pairs_output = Path(args.pairs_output).resolve()
    report_output = Path(args.report_output).resolve()
    save_json(
        pairs_output,
        {
            "timestamp": now,
            "output_matrix": str(output),
            "solve_mode": "handeye",
            "samples": samples,
        },
    )
    save_json(
        report_output,
        {
            "timestamp": now,
            "output_matrix": str(output),
            "solve_mode": "handeye",
            "handeye_method": str(args.handeye_method),
            "residual_translation_error_m": translation_errors,
            "residual_rotation_error_rad": rotation_errors,
            "camera_min_xyz_m": cam_min,
            "camera_max_xyz_m": cam_max,
            "camera_span_xyz_m": cam_span,
            "base_min_xyz_m": base_min,
            "base_max_xyz_m": base_max,
            "base_span_xyz_m": base_span,
            "tool_from_marker_per_sample": [
                [[float(v) for v in row] for row in mat.tolist()]
                for mat in tool_from_marker_mats
            ],
        },
    )
    print(f"[fit] saved matrix: {output}")
    print(f"[fit] saved pairs:  {pairs_output}")
    print(f"[fit] saved report: {report_output}")
    if backup is not None:
        print(f"[fit] previous output backed up: {backup}")
    print(
        "[fit] handeye=%s residual_trans_rmse=%.4f m residual_rot_rmse=%.3f deg translation=%s rpy_deg=%s"
        % (
            args.handeye_method,
            result["residual_translation_rmse_m"],
            math.degrees(result["residual_rotation_rmse_rad"]),
            [round(float(v), 5) for v in base_from_camera[:3, 3].tolist()],
            [round(math.degrees(float(v)), 3) for v in rot_rpy],
        )
    )
    if result["residual_translation_rmse_m"] > float(args.warn_rmse_m):
        print(f"[warn] residual translation RMSE is above {args.warn_rmse_m:.3f} m")
    return True


def fit_point_pairs_and_save(samples: list[dict[str, Any]], args: argparse.Namespace, *, force: bool = False) -> bool:
    if len(samples) < int(args.min_pairs):
        print(f"[fit] samples={len(samples)} < min_pairs={args.min_pairs}; continue sampling")
        return False
    camera_pts = [[float(v) for v in sample["camera_xyz_m"]] for sample in samples]
    base_pts = [[float(v) for v in sample["base_marker_xyz_m"]] for sample in samples]
    cam_min, cam_max, cam_span = axis_span(camera_pts)
    base_min, base_max, base_span = axis_span(base_pts)
    if max(cam_span) < float(args.min_span_m) or max(base_span) < float(args.min_span_m):
        print(f"[fit] sample span too small; camera_span={cam_span}, base_span={base_span}")
        if not force:
            return False

    mat, rmse, inliers, outliers, threshold = robust_fit(camera_pts, base_pts, min_inliers=max(4, int(args.min_pairs) // 2))
    errs, fitted = compute_errors(camera_pts, base_pts, mat)
    rot_rpy = matrix_to_rpy(mat[:3, :3])
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    output = Path(args.output).resolve()
    backup = backup_existing_file(output)
    result = {
        "timestamp": now,
        "method": "rm_external_depth_camera_aruco_point_pairs",
        "equation": "p_base = matrix @ [p_camera_x, p_camera_y, p_camera_z, 1]^T",
        "matrix": [[float(v) for v in row] for row in mat.tolist()],
        "translation_m": [float(v) for v in mat[:3, 3].tolist()],
        "rotation_rpy_rad": [float(v) for v in rot_rpy],
        "rotation_rpy_deg": [float(math.degrees(v)) for v in rot_rpy],
        "rmse_m": float(rmse),
        "max_error_m": float(np.max(errs)),
        "mean_error_m": float(np.mean(errs)),
        "point_pairs": len(samples),
        "inlier_count": len(inliers),
        "outlier_count": len(outliers),
        "outlier_threshold_m": float(threshold),
        "camera_source": str(args.camera_source),
        "arm_backend": str(args.arm_backend),
        "arm_host": str(args.host),
        "tool_name": str(args.tool_name),
        "marker_offset_in_tool_m": [float(v) for v in args.marker_offset_m],
        "aruco_dictionary": str(args.aruco_dict),
        "aruco_id": int(args.marker_id),
        "aruco_size_m": float(args.marker_size_m),
        "camera_span_xyz_m": [float(v) for v in cam_span],
        "base_span_xyz_m": [float(v) for v in base_span],
        "backup_of_previous_output": None if backup is None else str(backup),
    }
    save_json(output, result)

    pairs_output = Path(args.pairs_output).resolve()
    report_output = Path(args.report_output).resolve()
    save_json(
        pairs_output,
        {
            "timestamp": now,
            "output_matrix": str(output),
            "samples": samples,
        },
    )
    save_json(
        report_output,
        {
            "timestamp": now,
            "output_matrix": str(output),
            "rmse_m": float(rmse),
            "per_point_error_m": [float(v) for v in errs.tolist()],
            "inlier_indices": inliers,
            "outlier_indices": outliers,
            "camera_min_xyz_m": cam_min,
            "camera_max_xyz_m": cam_max,
            "camera_span_xyz_m": cam_span,
            "base_min_xyz_m": base_min,
            "base_max_xyz_m": base_max,
            "base_span_xyz_m": base_span,
            "samples_fit": [
                {
                    "index": idx,
                    "camera_xyz_m": camera_pts[idx],
                    "base_marker_xyz_m": base_pts[idx],
                    "base_fit_xyz_m": [float(v) for v in fitted[idx].tolist()],
                    "error_m": float(errs[idx]),
                    "is_inlier": idx in inliers,
                }
                for idx in range(len(samples))
            ],
        },
    )
    print(f"[fit] saved matrix: {output}")
    print(f"[fit] saved pairs:  {pairs_output}")
    print(f"[fit] saved report: {report_output}")
    if backup is not None:
        print(f"[fit] previous output backed up: {backup}")
    print(
        "[fit] rmse=%.4f m max=%.4f m inliers=%d outliers=%d translation=%s rpy_deg=%s"
        % (
            rmse,
            float(np.max(errs)),
            len(inliers),
            len(outliers),
            [round(float(v), 5) for v in mat[:3, 3].tolist()],
            [round(math.degrees(float(v)), 3) for v in rot_rpy],
        )
    )
    if rmse > float(args.warn_rmse_m):
        print(f"[warn] RMSE is above {args.warn_rmse_m:.3f} m; sample more widely or check marker offset")
    return True


def fit_and_save(samples: list[dict[str, Any]], args: argparse.Namespace, *, force: bool = False) -> bool:
    if str(args.solve_mode).strip().lower() == "handeye":
        return fit_handeye_and_save(samples, args, force=force)
    return fit_point_pairs_and_save(samples, args, force=force)


def make_source(args: argparse.Namespace):
    source = str(args.camera_source).strip().lower()
    if source == "opencv":
        return OpenCvSource(
            device=args.opencv_device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            fx=args.fx,
            fy=args.fy,
            ppx=args.ppx,
            ppy=args.ppy,
            fov_deg=args.assumed_fov_deg,
            intrinsics_json=args.intrinsics_json,
            intrinsics_from_realsense=args.opencv_realsense_intrinsics,
            realsense_serial=args.realsense_serial,
        )
    if source in ("auto", "realsense"):
        try:
            return RealsenseSource(
                width=args.width,
                height=args.height,
                fps=args.fps,
                warmup_frames=args.warmup_frames,
            )
        except Exception as exc:
            if source == "realsense":
                raise
            print(f"[camera] local RealSense unavailable, falling back to ROS topics: {type(exc).__name__}: {exc}")
    if source in ("auto", "ros"):
        return RosRgbdSource(
            color_topic=args.color_topic,
            depth_topic=args.depth_topic,
            camera_info_topic=args.camera_info_topic,
        )
    raise RuntimeError(f"unsupported camera source: {args.camera_source}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate fixed external RGBD camera to RM robot Base with an ArUco marker. "
            "Attach the marker to the current tool/TCP, manually move the arm to several positions, "
            "press c to sample, then f to fit and save camera_to_robot.json. "
            "The default handeye mode does not need the ArUco center-to-TCP offset."
        )
    )
    parser.add_argument(
        "--solve-mode",
        choices=("handeye", "point"),
        default="handeye",
        help="handeye uses full 6D poses and estimates marker-to-tool offset; point mode needs --marker-offset-m",
    )
    parser.add_argument(
        "--handeye-method",
        choices=("tsai", "park", "horaud", "andreff", "daniilidis"),
        default="park",
        help="OpenCV hand-eye solver used by --solve-mode handeye",
    )
    parser.add_argument("--camera-source", choices=("auto", "realsense", "ros", "opencv"), default="auto")
    parser.add_argument("--opencv-device", type=int, default=4, help="OpenCV/V4L2 device index for --camera-source opencv")
    parser.add_argument("--realsense-serial", default="", help="optional RealSense serial number")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--intrinsics-json", default="", help="camera intrinsics JSON for --camera-source opencv")
    parser.add_argument(
        "--opencv-realsense-intrinsics",
        action="store_true",
        help="for --camera-source opencv, read RealSense color intrinsics without streaming through librealsense",
    )
    parser.add_argument("--fx", type=float, default=0.0, help="manual camera fx for --camera-source opencv")
    parser.add_argument("--fy", type=float, default=0.0, help="manual camera fy for --camera-source opencv")
    parser.add_argument("--ppx", type=float, default=0.0, help="manual camera principal point x for --camera-source opencv")
    parser.add_argument("--ppy", type=float, default=0.0, help="manual camera principal point y for --camera-source opencv")
    parser.add_argument(
        "--assumed-fov-deg",
        type=float,
        default=60.0,
        help="temporary horizontal FOV used to approximate OpenCV camera intrinsics when fx/fy are not supplied",
    )
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--ros-master-uri", default=os.environ.get("ROS_MASTER_URI", "http://192.168.1.11:11311"))
    parser.add_argument("--ros-ip", default=os.environ.get("ROS_IP", ""))
    parser.add_argument("--host", default=DEFAULT_HOST, help="RM JSON host, used by JSON backend and ROS fallback")
    parser.add_argument("--arm-backend", choices=("ros", "json"), default="ros")
    parser.add_argument(
        "--tool-name",
        default="mas_rub",
        help="tool frame whose TCP pose is read from the controller; pass an empty string to avoid switching tools",
    )
    parser.add_argument(
        "--marker-offset-m",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.0],
        metavar=("X", "Y", "Z"),
        help="Only for --solve-mode point: ArUco center offset relative to the active tool/TCP frame, in meters",
    )
    parser.add_argument("--allow-arm-error", action="store_true")
    parser.add_argument("--aruco-dict", default="DICT_5X5_250")
    parser.add_argument("--marker-id", type=int, default=0, help="target ArUco id; -1 selects the largest detected marker")
    parser.add_argument("--marker-size-m", type=float, default=0.05, help="printed marker side length; needed for axis drawing/PnP fallback")
    parser.add_argument("--allow-pnp-fallback", action="store_true", help="use ArUco PnP tvec when aligned depth is unavailable")
    parser.add_argument("--depth-radius-px", type=int, default=4)
    parser.add_argument("--min-depth-m", type=float, default=0.15)
    parser.add_argument("--max-depth-m", type=float, default=2.5)
    parser.add_argument("--min-new-point-dist-m", type=float, default=0.01)
    parser.add_argument("--min-pairs", type=int, default=8)
    parser.add_argument("--min-span-m", type=float, default=0.08)
    parser.add_argument("--warn-rmse-m", type=float, default=0.01)
    parser.add_argument("--output", default=str(PROJECT_DIR / "camera_to_robot.json"))
    parser.add_argument("--sample-dir", default=str(PROJECT_DIR / "rm_demo_output" / "aruco_calibration"))
    parser.add_argument("--pairs-output", default="")
    parser.add_argument("--report-output", default="")
    parser.add_argument("--save-images", action="store_true")
    parser.add_argument("--force-fit-small-span", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    os.environ.setdefault("ROS_MASTER_URI", str(args.ros_master_uri))
    if str(args.ros_ip).strip():
        os.environ.setdefault("ROS_IP", str(args.ros_ip).strip())

    sample_dir = Path(args.sample_dir).resolve()
    sample_dir.mkdir(parents=True, exist_ok=True)
    if not args.pairs_output:
        args.pairs_output = str(sample_dir / "camera_robot_aruco_pairs.json")
    if not args.report_output:
        args.report_output = str(sample_dir / "camera_to_robot_aruco_report.json")

    print("=== RM external RGBD camera -> robot Base ArUco calibration ===")
    print(f"camera_source={args.camera_source} ROS_MASTER_URI={os.environ.get('ROS_MASTER_URI', '')}")
    print(f"arm_backend={args.arm_backend} host={args.host} tool_name={args.tool_name!r}")
    print(f"solve_mode={args.solve_mode} handeye_method={args.handeye_method}")
    print(f"aruco_dict={args.aruco_dict} marker_id={args.marker_id} marker_size_m={args.marker_size_m}")
    if args.solve_mode == "point":
        print(f"marker_offset_in_tool_m={args.marker_offset_m}")
    else:
        print("marker_offset_in_tool_m is not required in handeye mode")
    print("This script does not move the robot. Move/teach the arm manually, then press c to sample.")
    print("Keys: c=capture point, x=undo, f=fit+save, q=quit")

    source = make_source(args)
    robot = RobotPoseReader(
        backend=args.arm_backend,
        host=args.host,
        tool_name=args.tool_name,
        allow_arm_error=args.allow_arm_error,
    )
    locator = ArucoLocator(
        dictionary_name=args.aruco_dict,
        marker_id=args.marker_id,
        marker_size_m=args.marker_size_m,
        depth_radius_px=args.depth_radius_px,
        min_depth_m=args.min_depth_m,
        max_depth_m=args.max_depth_m,
        allow_pnp_fallback=bool(args.allow_pnp_fallback or args.solve_mode == "handeye"),
    )
    samples: list[dict[str, Any]] = []
    window_name = "RM ArUco camera->base calibration"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while True:
            try:
                frame = source.read(timeout_s=3.0)
            except Exception as exc:
                print(f"[camera] read failed: {type(exc).__name__}: {exc}")
                time.sleep(0.3)
                continue

            observation, corners, ids = locator.locate(frame)
            canvas = locator.draw(frame, observation, corners, ids)
            cv2.putText(canvas, "c:capture  x:undo  f:fit+save  q:quit", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.putText(canvas, f"samples: {len(samples)}", (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            if observation is None:
                cv2.putText(canvas, "target marker not found or depth invalid", (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 255), 2)
            else:
                xyz = observation.camera_xyz_m
                cv2.putText(
                    canvas,
                    "id=%d %s cam=(%.3f, %.3f, %.3f)m"
                    % (observation.marker_id, observation.camera_xyz_method, xyz[0], xyz[1], xyz[2]),
                    (10, 88),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )
            cv2.imshow(window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("x"):
                if samples:
                    removed = samples.pop()
                    print(f"[undo] removed sample #{removed.get('index')}; remaining={len(samples)}")
                    save_json(Path(args.pairs_output).resolve(), {"samples": samples})
                else:
                    print("[undo] no samples")
                continue
            if key == ord("f"):
                fit_and_save(samples, args, force=bool(args.force_fit_small_span))
                continue
            if key != ord("c"):
                continue
            if observation is None:
                print("[capture] marker/depth not ready")
                continue

            try:
                state = robot.read()
            except Exception as exc:
                print(f"[capture] failed to read robot pose: {type(exc).__name__}: {exc}")
                continue

            base_from_tool = pose_to_matrix(state["pose_m_rad"])
            base_marker: np.ndarray | None = None
            if args.solve_mode == "handeye":
                if observation.camera_from_marker_matrix is None:
                    print("[capture] handeye mode requires ArUco PnP pose; check --marker-size-m and camera intrinsics")
                    continue
                camera_from_marker = np.asarray(observation.camera_from_marker_matrix, dtype=np.float64).reshape(4, 4)
                camera_marker = camera_from_marker[:3, 3]
                base_sample_position = base_from_tool[:3, 3]
            else:
                marker_offset = np.asarray([float(v) for v in args.marker_offset_m] + [1.0], dtype=np.float64)
                base_marker = (base_from_tool @ marker_offset)[:3]
                camera_marker = np.asarray(observation.camera_xyz_m, dtype=np.float64)
                base_sample_position = base_marker
            if samples:
                cam_nearest = min(float(np.linalg.norm(camera_marker - np.asarray(s["camera_sample_xyz_m"], dtype=np.float64))) for s in samples)
                base_nearest = min(float(np.linalg.norm(base_sample_position - np.asarray(s["base_sample_xyz_m"], dtype=np.float64))) for s in samples)
                if cam_nearest < float(args.min_new_point_dist_m) and base_nearest < float(args.min_new_point_dist_m):
                    print(
                        "[capture] point is too close to existing samples "
                        f"(camera={cam_nearest:.4f}m base={base_nearest:.4f}m); move to another pose"
                    )
                    continue

            sample_index = len(samples) + 1
            sample = {
                "index": sample_index,
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "camera_xyz_m": [float(v) for v in observation.camera_xyz_m],
                "camera_sample_xyz_m": [float(v) for v in camera_marker.tolist()],
                "base_sample_xyz_m": [float(v) for v in base_sample_position.tolist()],
                "base_marker_xyz_m": None if base_marker is None else [float(v) for v in base_marker.tolist()],
                "base_from_tool_matrix": [[float(v) for v in row] for row in base_from_tool.tolist()],
                "camera_from_marker_matrix": observation.camera_from_marker_matrix,
                "robot_joints_deg": state["joints_deg"],
                "robot_tool_pose_m_rad": state["pose_m_rad"],
                "arm_err": state["arm_err"],
                "sys_err": state["sys_err"],
                "inverse_km_err": state["inverse_km_err"],
                "marker": asdict(observation),
            }
            samples.append(sample)
            save_json(Path(args.pairs_output).resolve(), {"samples": samples})
            if args.save_images:
                image_path = sample_dir / f"sample_{sample_index:03d}.png"
                cv2.imwrite(str(image_path), canvas)
                sample["image_path"] = str(image_path)
                save_json(Path(args.pairs_output).resolve(), {"samples": samples})
            print(
                "[capture] #%d camera=%s base=%s method=%s"
                % (
                    sample_index,
                    [round(float(v), 5) for v in camera_marker.tolist()],
                    [round(float(v), 5) for v in base_sample_position.tolist()],
                    observation.camera_xyz_method,
                )
            )
    finally:
        try:
            source.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
