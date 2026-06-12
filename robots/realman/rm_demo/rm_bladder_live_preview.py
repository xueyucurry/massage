#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from http import server
from socketserver import ThreadingMixIn
from typing import Any

os.environ.setdefault("MPLBACKEND", "Agg")

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from rm_demo.config import DEFAULT_FINGER_WIDTH_MM, DEFAULT_MODEL_PATH
    from rm_demo.rm_bladder import (
        POSE_ROTATION_MODES,
        REQUIRED_KEYPOINTS,
        _build_overlay,
        _build_spine_seed_from_torso,
        _extract_best_pose_keypoints,
        _expand_meridian_lines_from_spine,
        _infer_best_pose_with_rotations,
        _load_model,
        _map_keypoints_to_original,
        _rotate_image_for_pose,
    )
else:
    from .config import DEFAULT_FINGER_WIDTH_MM, DEFAULT_MODEL_PATH
    from .rm_bladder import (
        POSE_ROTATION_MODES,
        REQUIRED_KEYPOINTS,
        _build_overlay,
        _build_spine_seed_from_torso,
        _extract_best_pose_keypoints,
        _expand_meridian_lines_from_spine,
        _infer_best_pose_with_rotations,
        _load_model,
        _map_keypoints_to_original,
        _rotate_image_for_pose,
    )

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live bladder meridian preview over HTTP")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8765, help="HTTP bind port")
    parser.add_argument("--model-path", default=DEFAULT_MODEL_PATH, help="pose model path")
    parser.add_argument("--finger-width", type=float, default=DEFAULT_FINGER_WIDTH_MM, help="base lateral offset in mm")
    parser.add_argument("--conf", type=float, default=0.5, help="pose confidence threshold")
    parser.add_argument("--preview-hz", type=float, default=4.0, help="max overlay refresh rate")
    parser.add_argument("--jpeg-quality", type=int, default=85, help="JPEG quality 1..100")
    return parser.parse_args()


def _import_ros_modules():
    import rospy  # type: ignore
    from cv_bridge import CvBridge  # type: ignore
    from sensor_msgs.msg import Image  # type: ignore

    return rospy, CvBridge, Image


def _clamp_tuple(point: tuple[float, float]) -> tuple[int, int]:
    return int(round(point[0])), int(round(point[1]))


def _pose_confidence(kpts: np.ndarray | None) -> float:
    if kpts is None:
        return 0.0
    return float(min(float(kpts[idx][2]) for idx in REQUIRED_KEYPOINTS))


def _draw_keypoints(image: np.ndarray, kpts: np.ndarray | None) -> None:
    if kpts is None:
        return
    labels = {5: "LS", 6: "RS", 11: "LH", 12: "RH"}
    for idx, label in labels.items():
        x, y, conf = float(kpts[idx][0]), float(kpts[idx][1]), float(kpts[idx][2])
        color = (0, 220, 0) if conf >= 0.35 else (0, 160, 255)
        cv2.circle(image, (int(round(x)), int(round(y))), 5, color, -1)
        cv2.putText(
            image,
            f"{label}:{conf:.2f}",
            (int(round(x)) + 6, int(round(y)) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )


def _normalize_2d(vec: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-6:
        return None
    return np.asarray(vec, dtype=np.float64) / norm


def _weighted_midpoint(kpts: np.ndarray, indices: tuple[int, int], min_conf: float = 0.005) -> np.ndarray | None:
    pts: list[np.ndarray] = []
    weights: list[float] = []
    for idx in indices:
        x, y, conf = float(kpts[idx][0]), float(kpts[idx][1]), float(kpts[idx][2])
        if not np.isfinite(x) or not np.isfinite(y) or conf < min_conf:
            continue
        pts.append(np.asarray([x, y], dtype=np.float64))
        weights.append(max(0.05, conf))
    if not pts:
        return None
    arr = np.stack(pts, axis=0)
    w = np.asarray(weights, dtype=np.float64)
    return np.sum(arr * w[:, None], axis=0) / max(1e-6, float(np.sum(w)))


def _pair_width(kpts: np.ndarray, indices: tuple[int, int], min_conf: float = 0.005) -> float | None:
    a, b = indices
    if float(kpts[a][2]) < min_conf or float(kpts[b][2]) < min_conf:
        return None
    pa = np.asarray([float(kpts[a][0]), float(kpts[a][1])], dtype=np.float64)
    pb = np.asarray([float(kpts[b][0]), float(kpts[b][1])], dtype=np.float64)
    if not np.all(np.isfinite(pa)) or not np.all(np.isfinite(pb)):
        return None
    width = float(np.linalg.norm(pb - pa))
    return width if width > 1.0 else None


def _build_relaxed_side_spine_seed(
    kpts: np.ndarray,
    frame_shape: tuple[int, int, int],
    finger_width_mm: float,
) -> dict[str, object] | None:
    shoulder_mid = _weighted_midpoint(kpts, (5, 6))
    hip_mid = _weighted_midpoint(kpts, (11, 12))
    if shoulder_mid is None or hip_mid is None:
        return None

    torso_vec = hip_mid - shoulder_mid
    torso_len_px = float(np.linalg.norm(torso_vec))
    if torso_len_px < 40.0:
        return None
    axis_2d = _normalize_2d(torso_vec)
    if axis_2d is None:
        return None
    lateral_2d = _normalize_2d(np.asarray([-axis_2d[1], axis_2d[0]], dtype=np.float64))
    if lateral_2d is None:
        return None

    widths = [
        value
        for value in (
            _pair_width(kpts, (5, 6)),
            _pair_width(kpts, (11, 12)),
        )
        if value is not None
    ]
    width_px = float(np.median(widths)) if widths else max(80.0, min(220.0, torso_len_px * 0.35))
    pixels_per_mm = width_px / 380.0
    body_offset_px = max(18.0, float(finger_width_mm) * pixels_per_mm)

    h, w = frame_shape[:2]
    margin = 0.04 * max(w, h)
    neck_pt = shoulder_mid - axis_2d * min(0.10 * torso_len_px, margin)
    tail_pt = hip_mid + axis_2d * min(0.12 * torso_len_px, margin * 1.2)

    return {
        "spine_line": (
            (float(neck_pt[0]), float(neck_pt[1])),
            (float(tail_pt[0]), float(tail_pt[1])),
        ),
        "lateral_direction_2d": lateral_2d.tolist(),
        "body_offset_px": float(body_offset_px),
        "shoulder_px": float(width_px),
    }


def _annotate_status(image: np.ndarray, lines: list[str], color: tuple[int, int, int]) -> None:
    y = 28
    for line in lines:
        cv2.putText(image, line, (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        y += 28


def _infer_pose_with_rotation_mode(model, img: np.ndarray, conf: float, rotation_mode: str) -> dict[str, object]:
    if rotation_mode == "auto":
        return _infer_best_pose_with_rotations(model, img, conf=conf)
    if rotation_mode not in POSE_ROTATION_MODES:
        raise ValueError(f"unsupported rotation mode: {rotation_mode}")

    orig_h, orig_w = img.shape[:2]
    rotated = _rotate_image_for_pose(img, rotation_mode)
    results = model(rotated, verbose=False, conf=conf)
    kpts, score = _extract_best_pose_keypoints(results[0])
    if kpts is None:
        return {"kpts": None, "score": -1.0, "rotation": rotation_mode}
    mapped = _map_keypoints_to_original(kpts, orig_w, orig_h, rotation_mode)
    return {"kpts": mapped, "score": score, "rotation": rotation_mode}


def _build_preview_overlay(
    frame_bgr: np.ndarray,
    model,
    finger_width_mm: float,
    conf: float,
    rotation_mode: str = "auto",
) -> tuple[np.ndarray, dict[str, Any]]:
    pose_info = _infer_pose_with_rotation_mode(model, frame_bgr, conf=conf, rotation_mode=rotation_mode)
    return _build_preview_overlay_from_pose(
        frame_bgr=frame_bgr,
        kpts=pose_info.get("kpts"),
        pose_rotation=str(pose_info.get("rotation", "none")),
        finger_width_mm=finger_width_mm,
        rotation_mode=rotation_mode,
    )


def _build_preview_overlay_from_pose(
    frame_bgr: np.ndarray,
    kpts: np.ndarray | None,
    pose_rotation: str,
    finger_width_mm: float,
    rotation_mode: str = "auto",
) -> tuple[np.ndarray, dict[str, Any]]:
    overlay = frame_bgr.copy()
    pose_conf = _pose_confidence(kpts)
    status: dict[str, Any] = {
        "tracking": "SEARCH",
        "pose_conf": round(pose_conf, 3),
        "rotation": str(pose_rotation),
        "rotation_mode": str(rotation_mode),
        "finger_width_mm": float(finger_width_mm),
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "kpts": None if kpts is None else np.asarray(kpts, dtype=np.float32).copy(),
    }
    _draw_keypoints(overlay, kpts)
    if kpts is None:
        _annotate_status(
            overlay,
            [
                "SEARCH: no valid body pose",
                "Adjust camera so shoulders and hips are fully visible",
            ],
            (0, 0, 255),
        )
        return overlay, status

    seed = _build_spine_seed_from_torso(kpts, finger_width_mm=float(finger_width_mm))
    if seed is None:
        seed = _build_relaxed_side_spine_seed(kpts, overlay.shape, finger_width_mm=float(finger_width_mm))
        if seed is None:
            status["tracking"] = "LOW_CONF"
            _annotate_status(
                overlay,
                [
                    f"LOW_CONF: torso confidence {pose_conf:.2f}",
                    "Keep full back, shoulders, hips in frame",
                ],
                (0, 165, 255),
            )
            return overlay, status
        status["tracking"] = "RELAXED"
        status["relaxed_reason"] = f"torso_conf={pose_conf:.2f}"

    spine_line = seed["spine_line"]
    lateral_direction_2d = np.asarray(seed["lateral_direction_2d"], dtype=np.float64)
    body_offset_px = float(seed["body_offset_px"])

    neck = np.asarray(spine_line[0], dtype=np.float64)
    tail = np.asarray(spine_line[1], dtype=np.float64)
    neck_l = neck - lateral_direction_2d * body_offset_px
    neck_r = neck + lateral_direction_2d * body_offset_px
    tail_l = tail - lateral_direction_2d * body_offset_px
    tail_r = tail + lateral_direction_2d * body_offset_px
    inner_lines = (
        ((float(neck_l[0]), float(neck_l[1])), (float(tail_l[0]), float(tail_l[1]))),
        ((float(neck_r[0]), float(neck_r[1])), (float(tail_r[0]), float(tail_r[1]))),
    )
    outer_lines = _expand_meridian_lines_from_spine(spine_line, inner_lines, scale=2.0)
    overlay = _build_overlay(
        overlay,
        spine_line=spine_line,
        inner_lines=inner_lines,
        outer_lines=outer_lines,
        pose_rotation=str(pose_rotation),
        finger_width_mm=float(finger_width_mm),
        shoulder_cm_real=None,
    )
    cv2.circle(overlay, _clamp_tuple(spine_line[0]), 6, (0, 0, 255), -1)
    cv2.circle(overlay, _clamp_tuple(spine_line[1]), 6, (0, 0, 255), -1)
    tracking_state = str(status.get("tracking") or "LOCKED")
    if tracking_state not in ("RELAXED",):
        status["tracking"] = "LOCKED"
    status["body_offset_px"] = round(body_offset_px, 1)
    status_line = "RELAXED" if status["tracking"] == "RELAXED" else "LOCKED"
    _annotate_status(
        overlay,
        [
            f"{status_line}: pose {pose_conf:.2f} rot={status['rotation']}",
            f"Offset={body_offset_px:.1f}px",
        ],
        (0, 220, 255) if status["tracking"] == "RELAXED" else (0, 255, 0),
    )
    return overlay, status


class PreviewState:
    def __init__(self, jpeg_quality: int) -> None:
        self.lock = threading.Lock()
        self.jpeg_quality = max(50, min(100, int(jpeg_quality)))
        self.latest_frame: np.ndarray | None = None
        self.latest_frame_seq = 0
        self.latest_jpeg = self._placeholder_jpeg("waiting for camera...")
        self.latest_status: dict[str, Any] = {
            "tracking": "INIT",
            "message": "waiting for camera...",
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.stop_event = threading.Event()

    def _placeholder_jpeg(self, text: str) -> bytes:
        canvas = np.zeros((480, 640, 3), dtype=np.uint8)
        canvas[:] = (24, 24, 24)
        cv2.putText(canvas, text, (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise RuntimeError("failed to encode placeholder frame")
        return bytes(buf)

    def update_raw_frame(self, frame_bgr: np.ndarray) -> None:
        with self.lock:
            self.latest_frame = frame_bgr.copy()
            self.latest_frame_seq += 1

    def take_latest_frame(self) -> tuple[np.ndarray | None, int]:
        with self.lock:
            if self.latest_frame is None:
                return None, self.latest_frame_seq
            return self.latest_frame.copy(), self.latest_frame_seq

    def publish_overlay(self, overlay_bgr: np.ndarray, status: dict[str, Any]) -> None:
        ok, buf = cv2.imencode(".jpg", overlay_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        with self.lock:
            self.latest_jpeg = bytes(buf)
            self.latest_status = dict(status)

    def publish_status(self, status: dict[str, Any]) -> None:
        with self.lock:
            self.latest_status = dict(status)

    def get_snapshot(self) -> tuple[bytes, dict[str, Any]]:
        with self.lock:
            return self.latest_jpeg, dict(self.latest_status)


class PreviewProcessor:
    def __init__(self, args: argparse.Namespace, state: PreviewState) -> None:
        self.args = args
        self.state = state
        self.model = _load_model(args.model_path)
        self.thread = threading.Thread(target=self._loop, name="rm_bladder_live_preview", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _loop(self) -> None:
        last_seq = -1
        interval = 1.0 / max(0.2, float(self.args.preview_hz))
        while not self.state.stop_event.is_set():
            frame_bgr, seq = self.state.take_latest_frame()
            if frame_bgr is None or seq == last_seq:
                time.sleep(0.03)
                continue
            last_seq = seq
            try:
                overlay, status = _build_preview_overlay(
                    frame_bgr,
                    model=self.model,
                    finger_width_mm=float(self.args.finger_width),
                    conf=float(self.args.conf),
                )
            except Exception as exc:
                overlay = frame_bgr.copy()
                status = {
                    "tracking": "ERROR",
                    "message": str(exc),
                    "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                }
                _annotate_status(overlay, [f"ERROR: {exc}"], (0, 0, 255))
            self.state.publish_overlay(overlay, status)
            time.sleep(interval)


class CameraPoller:
    def __init__(self, rospy, bridge, Image, state: PreviewState) -> None:
        self.rospy = rospy
        self.bridge = bridge
        self.Image = Image
        self.state = state
        self.thread = threading.Thread(target=self._loop, name="rm_bladder_camera_poller", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _loop(self) -> None:
        while not self.state.stop_event.is_set():
            try:
                msg = self.rospy.wait_for_message("/camera/color/image_raw", self.Image, timeout=3.0)
                frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.state.update_raw_frame(frame_bgr)
            except Exception as exc:
                self.state.publish_status(
                    {
                        "tracking": "WAIT_CAMERA",
                        "message": f"camera wait failed: {exc}",
                        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    }
                )
                time.sleep(0.2)


class PreviewRequestHandler(server.BaseHTTPRequestHandler):
    server_version = "RMBladderPreview/1.0"

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            self._serve_index()
            return
        if self.path == "/stream.mjpg":
            self._serve_stream()
            return
        if self.path == "/snapshot.jpg":
            self._serve_snapshot()
            return
        if self.path == "/status.json":
            self._serve_status()
            return
        self.send_error(404, "not found")

    def log_message(self, format: str, *args) -> None:
        return

    @property
    def preview_state(self) -> PreviewState:
        return self.server.preview_state  # type: ignore[attr-defined]

    def _serve_index(self) -> None:
        html = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Bladder Preview</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: sans-serif; }
    header { padding: 12px 16px; background: #1d1d1d; border-bottom: 1px solid #333; }
    main { display: flex; gap: 16px; padding: 16px; align-items: flex-start; }
    img { max-width: min(70vw, 960px); width: 100%; border: 1px solid #333; background: #000; }
    pre { margin: 0; padding: 12px; background: #181818; border: 1px solid #333; min-width: 300px; white-space: pre-wrap; }
    a { color: #7dd3fc; }
  </style>
</head>
<body>
  <header>
    <strong>RealMan Bladder Live Preview</strong>
    <span style="margin-left:12px">Adjust camera until status becomes LOCKED.</span>
    <span style="margin-left:12px"><a href="/snapshot.jpg" target="_blank">Snapshot</a></span>
  </header>
  <main>
    <img src="/stream.mjpg" alt="live preview">
    <pre id="status">loading...</pre>
  </main>
  <script>
    async function refreshStatus() {
      try {
        const res = await fetch('/status.json', {cache: 'no-store'});
        const data = await res.json();
        document.getElementById('status').textContent = JSON.stringify(data, null, 2);
      } catch (err) {
        document.getElementById('status').textContent = String(err);
      }
    }
    refreshStatus();
    setInterval(refreshStatus, 700);
  </script>
</body>
</html>"""
        data = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_snapshot(self) -> None:
        jpg, _ = self.preview_state.get_snapshot()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpg)))
        self.end_headers()
        self.wfile.write(jpg)

    def _serve_status(self) -> None:
        _, status = self.preview_state.get_snapshot()
        data = json.dumps(status, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_stream(self) -> None:
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while not self.preview_state.stop_event.is_set():
                jpg, _ = self.preview_state.get_snapshot()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("utf-8"))
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(0.15)
        except (BrokenPipeError, ConnectionResetError):
            return


class ThreadedHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True

    def __init__(self, server_address, RequestHandlerClass, preview_state: PreviewState):
        super().__init__(server_address, RequestHandlerClass)
        self.preview_state = preview_state


def main() -> None:
    args = parse_args()
    rospy, CvBridge, Image = _import_ros_modules()
    if not rospy.core.is_initialized():
        rospy.init_node("rm_bladder_live_preview", anonymous=True, disable_signals=True)

    bridge = CvBridge()
    state = PreviewState(jpeg_quality=args.jpeg_quality)
    processor = PreviewProcessor(args=args, state=state)
    camera_poller = CameraPoller(rospy=rospy, bridge=bridge, Image=Image, state=state)
    camera_poller.start()
    processor.start()

    httpd = ThreadedHTTPServer((args.host, int(args.port)), PreviewRequestHandler, preview_state=state)
    print(f"live_preview_url=http://{args.host}:{args.port}/", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.stop_event.set()
        httpd.server_close()


if __name__ == "__main__":
    main()
