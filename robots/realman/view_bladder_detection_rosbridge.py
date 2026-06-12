#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import datetime
import errno
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import cv2
import numpy as np
import roslibpy

from rm_demo.rm_bladder import _load_model
from rm_demo.rm_bladder_live_preview import (
    _build_preview_overlay,
    _build_preview_overlay_from_pose,
    _infer_pose_with_rotation_mode,
)
from view_remote_camera_over_ssh import can_use_opencv_viewer

DEFAULT_BOARD_HOST = os.environ.get("RM_BOARD_HOST", "192.168.1.11")
DEFAULT_ROSBRIDGE_PORT = int(os.environ.get("RM_ROSBRIDGE_PORT", "9090"))
DEFAULT_TOPIC = "/camera/color/image_raw/compressed"
DEFAULT_MODEL = os.path.abspath("yolo11n-pose.pt")
ROTATION_MODE_CHOICES = ("auto_lock", "auto", "none", "cw90", "ccw90", "180")
TOPIC_TYPE_CHOICES = ("compressed", "raw")
VIEWER_CHOICES = ("auto", "opencv", "http")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Realtime bladder detection using direct rosbridge subscription."
    )
    parser.add_argument("--board-host", default=DEFAULT_BOARD_HOST, help="board IP running rosbridge")
    parser.add_argument("--rosbridge-port", type=int, default=DEFAULT_ROSBRIDGE_PORT, help="rosbridge websocket port")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="compressed image topic")
    parser.add_argument(
        "--viewer",
        choices=VIEWER_CHOICES,
        default="auto",
        help="preview backend: OpenCV window or browser HTTP stream",
    )
    parser.add_argument(
        "--topic-type",
        choices=TOPIC_TYPE_CHOICES,
        default="compressed",
        help="rosbridge image message type",
    )
    parser.add_argument("--model-path", default=DEFAULT_MODEL, help="pose model path")
    parser.add_argument("--finger-width-mm", type=float, default=45.0, help="visual offset for bladder lines")
    parser.add_argument("--conf", type=float, default=0.5, help="pose confidence threshold")
    parser.add_argument("--detect-hz", type=float, default=3.0, help="overlay inference rate")
    parser.add_argument(
        "--rotation-mode",
        choices=ROTATION_MODE_CHOICES,
        default="auto_lock",
        help="pose rotation strategy: auto_lock learns a fixed rotation then switches to single-pass inference",
    )
    parser.add_argument("--auto-lock-frames", type=int, default=4, help="successful auto frames needed before locking rotation")
    parser.add_argument(
        "--smooth-alpha",
        type=float,
        default=0.75,
        help="EMA smoothing factor for keypoints after lock; higher is steadier but lags more",
    )
    parser.add_argument(
        "--deadband-px",
        type=float,
        default=2.5,
        help="ignore tiny keypoint movements below this pixel distance after smoothing",
    )
    parser.add_argument(
        "--hold-motion-threshold",
        type=float,
        default=2.0,
        help="if frame-to-frame motion is below this mean gray difference, reuse previous pose without rerunning inference",
    )
    parser.add_argument("--width", type=int, default=1280, help="display width")
    parser.add_argument("--http-host", default="127.0.0.1", help="HTTP bind host for browser preview")
    parser.add_argument("--http-port", type=int, default=8766, help="HTTP port for browser preview")
    parser.add_argument("--jpeg-quality", type=int, default=85, help="HTTP preview JPEG quality")
    parser.add_argument("--jpeg-save-dir", default="rosbridge_bladder_preview", help="snapshot directory")
    return parser.parse_args()


def _decode_compressed_bgr(msg: dict[str, Any]) -> np.ndarray | None:
    data = msg.get("data")
    if isinstance(data, str):
        raw = base64.b64decode(data)
    elif isinstance(data, list):
        raw = bytes(int(v) & 0xFF for v in data)
    else:
        return None
    img = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    return None if img is None else img


def _decode_uint8_array(data_field) -> bytes:
    if isinstance(data_field, str):
        try:
            return base64.b64decode(data_field)
        except Exception:
            return bytes()
    if isinstance(data_field, list):
        return bytes(int(v) & 0xFF for v in data_field)
    return bytes()


def _decode_raw_bgr(msg: dict[str, Any]) -> np.ndarray | None:
    try:
        height = int(msg["height"])
        width = int(msg["width"])
        encoding = str(msg["encoding"])
        raw = _decode_uint8_array(msg["data"])
        if encoding == "bgr8":
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)
            return arr.copy()
        if encoding == "rgb8":
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    except Exception:
        return None
    return None


def _fit_width(image: np.ndarray, width: int) -> np.ndarray:
    h, w = image.shape[:2]
    target_w = max(320, int(width))
    target_h = max(180, int(round(target_w * h / max(1, w))))
    if target_w == w:
        return image
    return cv2.resize(image, (target_w, target_h))


def _draw_lines(image: np.ndarray, lines: list[str], color: tuple[int, int, int] = (0, 255, 255)) -> np.ndarray:
    canvas = image.copy()
    y = 28
    for line in lines:
        cv2.putText(canvas, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        y += 28
    return canvas


def _stack_raw_and_overlay(raw_bgr: np.ndarray, overlay_bgr: np.ndarray) -> np.ndarray:
    if raw_bgr.shape[:2] != overlay_bgr.shape[:2]:
        overlay_bgr = cv2.resize(overlay_bgr, (raw_bgr.shape[1], raw_bgr.shape[0]))
    raw = raw_bgr.copy()
    overlay = overlay_bgr.copy()
    cv2.putText(raw, "raw", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(overlay, "overlay", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2, cv2.LINE_AA)
    return np.hstack([raw, overlay])


def _save_snapshot(output_dir: str, raw_bgr: np.ndarray, overlay_bgr: np.ndarray | None) -> None:
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = os.path.join(output_dir, f"rosbridge_raw_{ts}.png")
    cv2.imwrite(raw_path, raw_bgr)
    print(f"[Saved] {raw_path}")
    if overlay_bgr is not None:
        overlay_path = os.path.join(output_dir, f"rosbridge_overlay_{ts}.png")
        cv2.imwrite(overlay_path, overlay_bgr)
        print(f"[Saved] {overlay_path}")


def _compose_preview(
    raw_bgr: np.ndarray | None,
    stamp: float,
    stream_fps: float,
    stream_err: str,
    overlay_bgr: np.ndarray | None,
    detect_status: str,
    detect_err: str,
    detect_fps: float,
    detect_cost: float,
    topic: str,
    width: int,
) -> np.ndarray:
    if raw_bgr is None:
        canvas = np.zeros((360, max(640, width), 3), dtype=np.uint8)
        preview = _draw_lines(
            canvas,
            [
                "waiting for rosbridge frames...",
                stream_err[:120] if stream_err else "connecting...",
            ],
        )
        return _fit_width(preview, width)

    base = raw_bgr if overlay_bgr is None else _stack_raw_and_overlay(raw_bgr, overlay_bgr)
    lines = [
        f"stream_fps={stream_fps:.1f} detect_fps={detect_fps:.1f} detect_cost={detect_cost:.2f}s",
        f"status={detect_status} topic={topic}",
        detect_err[:120] if detect_err else f"stamp={datetime.datetime.fromtimestamp(stamp).strftime('%H:%M:%S')}",
    ]
    color = (0, 0, 255) if detect_err else (0, 255, 255)
    return _fit_width(_draw_lines(base, lines, color=color), width)


class RosbridgeCompressedStream:
    def __init__(self, ros: roslibpy.Ros, topic_name: str, topic_type: str = "compressed") -> None:
        self.ros = ros
        self.topic_name = topic_name
        self.topic_type = str(topic_type)
        self.lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.latest_stamp = 0.0
        self.frame_count = 0
        self.first_stamp = 0.0
        self.last_error = ""
        msg_type = "sensor_msgs/CompressedImage" if self.topic_type == "compressed" else "sensor_msgs/Image"
        self.topic = roslibpy.Topic(ros, topic_name, msg_type, queue_length=1)
        self.topic.subscribe(self._on_msg)

    def _on_msg(self, msg: dict[str, Any]) -> None:
        try:
            if self.topic_type == "compressed":
                img = _decode_compressed_bgr(msg)
            else:
                img = _decode_raw_bgr(msg)
            if img is None:
                raise RuntimeError(f"{self.topic_type} image decode failed")
            now = time.time()
            with self.lock:
                self.latest_frame = img
                self.latest_stamp = now
                self.frame_count += 1
                if self.first_stamp == 0.0:
                    self.first_stamp = now
                self.last_error = ""
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)

    def read(self) -> tuple[np.ndarray | None, float, float, str, int]:
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
            stamp = self.latest_stamp
            err = self.last_error
            count = self.frame_count
            if self.frame_count <= 1 or self.latest_stamp <= self.first_stamp:
                fps = 0.0
            else:
                fps = (self.frame_count - 1) / (self.latest_stamp - self.first_stamp)
        return frame, stamp, fps, err, count

    def close(self) -> None:
        self.topic.unsubscribe()


class DetectionWorker:
    def __init__(
        self,
        stream: RosbridgeCompressedStream,
        model_path: str,
        finger_width_mm: float,
        conf: float,
        detect_hz: float,
        rotation_mode: str,
        auto_lock_frames: int,
        smooth_alpha: float,
        deadband_px: float,
        hold_motion_threshold: float,
    ) -> None:
        self.stream = stream
        self.model = _load_model(model_path)
        self.finger_width_mm = float(finger_width_mm)
        self.conf = float(conf)
        self.detect_hz = max(0.5, float(detect_hz))
        self.rotation_mode = str(rotation_mode)
        self.auto_lock_frames = max(1, int(auto_lock_frames))
        self.smooth_alpha = min(0.95, max(0.0, float(smooth_alpha)))
        self.deadband_px = max(0.0, float(deadband_px))
        self.hold_motion_threshold = max(0.0, float(hold_motion_threshold))
        self.lock = threading.Lock()
        self.latest_overlay: np.ndarray | None = None
        self.latest_status = "waiting for frames..."
        self.latest_error = ""
        self.last_processed_stamp = -1.0
        self.detect_count = 0
        self.detect_fps = 0.0
        self.first_detect_stamp = 0.0
        self.last_detect_cost_s = 0.0
        self.locked_rotation = ""
        self.rotation_votes = {mode: 0 for mode in ("none", "cw90", "ccw90", "180")}
        self.prev_kpts: np.ndarray | None = None
        self.prev_frame_small: np.ndarray | None = None
        self.last_rotation = "none"
        self.running = False
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="rosbridge-bladder-detect", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def read(self) -> tuple[np.ndarray | None, str, str, float, float]:
        with self.lock:
            overlay = None if self.latest_overlay is None else self.latest_overlay.copy()
            return overlay, self.latest_status, self.latest_error, self.detect_fps, self.last_detect_cost_s

    def _effective_rotation_mode(self) -> str:
        if self.rotation_mode == "auto_lock":
            return self.locked_rotation or "auto"
        return self.rotation_mode

    def _maybe_update_rotation_lock(self, status: dict[str, Any]) -> None:
        if self.rotation_mode != "auto_lock" or self.locked_rotation:
            return
        pose_conf = float(status.get("pose_conf", 0.0) or 0.0)
        rot = str(status.get("rotation", ""))
        if pose_conf < 0.35 or rot not in self.rotation_votes:
            return
        self.rotation_votes[rot] += 1
        vote_total = sum(self.rotation_votes.values())
        if vote_total >= self.auto_lock_frames:
            self.locked_rotation = max(self.rotation_votes, key=self.rotation_votes.get)

    def _frame_motion_score(self, frame_bgr: np.ndarray) -> float:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, (96, 72))
        if self.prev_frame_small is None:
            self.prev_frame_small = small
            return 999.0
        diff = cv2.absdiff(small, self.prev_frame_small)
        score = float(np.mean(diff))
        self.prev_frame_small = small
        return score

    def _loop(self) -> None:
        interval = 1.0 / self.detect_hz
        while self.running:
            frame, stamp, _, err, _ = self.stream.read()
            if frame is None:
                with self.lock:
                    if err:
                        self.latest_error = err
                        self.latest_status = "waiting for frames..."
                time.sleep(0.03)
                continue
            if stamp <= self.last_processed_stamp:
                time.sleep(0.01)
                continue
            self.last_processed_stamp = stamp
            t0 = time.time()
            effective_rotation = self._effective_rotation_mode()
            motion_score = self._frame_motion_score(frame)
            try:
                can_hold = (
                    motion_score <= self.hold_motion_threshold
                    and self.prev_kpts is not None
                    and (self.locked_rotation or self.rotation_mode in ("none", "cw90", "ccw90", "180"))
                )
                if can_hold:
                    current_rotation = self.last_rotation
                    kpts_to_draw = self.prev_kpts.copy()
                    status = {
                        "tracking": "HOLD",
                        "rotation": current_rotation,
                        "held": True,
                        "motion_score": round(motion_score, 3),
                    }
                else:
                    pose_info = _infer_pose_with_rotation_mode(
                        self.model,
                        frame,
                        conf=self.conf,
                        rotation_mode=effective_rotation,
                    )
                    raw_kpts = pose_info.get("kpts")
                    current_rotation = str(pose_info.get("rotation", "n/a"))
                    if raw_kpts is not None:
                        raw_kpts = np.asarray(raw_kpts, dtype=np.float32)
                    kpts_to_draw = raw_kpts
                    if raw_kpts is None:
                        self.prev_kpts = None
                    elif (
                        self.prev_kpts is not None
                        and self.prev_kpts.shape == raw_kpts.shape
                        and (self.locked_rotation or self.rotation_mode in ("none", "cw90", "ccw90", "180"))
                    ):
                        alpha = self.smooth_alpha
                        smoothed = raw_kpts.copy()
                        smoothed[:, :2] = alpha * self.prev_kpts[:, :2] + (1.0 - alpha) * raw_kpts[:, :2]
                        deltas = np.linalg.norm(smoothed[:, :2] - self.prev_kpts[:, :2], axis=1)
                        keep_mask = deltas < self.deadband_px
                        smoothed[keep_mask, :2] = self.prev_kpts[keep_mask, :2]
                        smoothed[:, 2] = raw_kpts[:, 2]
                        kpts_to_draw = smoothed
                        self.prev_kpts = smoothed
                    else:
                        self.prev_kpts = raw_kpts.copy() if raw_kpts is not None else None

                    self.last_rotation = current_rotation

                overlay, preview_status = _build_preview_overlay_from_pose(
                    frame_bgr=frame,
                    kpts=kpts_to_draw,
                    pose_rotation=current_rotation,
                    finger_width_mm=self.finger_width_mm,
                    rotation_mode=effective_rotation,
                )
                if not can_hold:
                    preview_status["smoothed"] = bool(kpts_to_draw is not None and self.prev_kpts is not None)
                    self._maybe_update_rotation_lock(preview_status)
                else:
                    held_tracking = str(preview_status.get("tracking", ""))
                    preview_status.update(status)
                    preview_status["tracking"] = f"HOLD_{held_tracking}" if held_tracking else "HOLD"
                    preview_status["held_tracking"] = held_tracking
                    preview_status["smoothed"] = True
                preview_status["motion_score"] = round(motion_score, 3)
                cost = time.time() - t0
                with self.lock:
                    self.latest_overlay = overlay
                    lock_suffix = f" lock={self.locked_rotation}" if self.locked_rotation else f" mode={effective_rotation}"
                    smooth_suffix = f" smooth={self.smooth_alpha:.2f}" if preview_status.get("smoothed") else " smooth=off"
                    hold_suffix = f" hold<{self.hold_motion_threshold:.1f}" if preview_status.get("held") else ""
                    self.latest_status = (
                        f"{preview_status.get('tracking', 'ok')} rot={current_rotation}{lock_suffix}"
                        f"{smooth_suffix}{hold_suffix} motion={motion_score:.2f}"
                    )
                    self.latest_error = ""
                    self.last_detect_cost_s = cost
                    self.detect_count += 1
                    now = time.time()
                    if self.first_detect_stamp == 0.0:
                        self.first_detect_stamp = now
                    elif now > self.first_detect_stamp:
                        self.detect_fps = (self.detect_count - 1) / (now - self.first_detect_stamp)
            except Exception as exc:
                self.prev_kpts = None
                cost = time.time() - t0
                bad = _draw_lines(
                    frame,
                    [
                        f"detect failed: {exc}",
                        "keep full back / shoulders / hips in frame",
                    ],
                    color=(0, 0, 255),
                )
                with self.lock:
                    self.latest_overlay = bad
                    self.latest_status = f"detect_failed mode={effective_rotation}"
                    self.latest_error = str(exc)
                    self.last_detect_cost_s = cost
            sleep_left = interval - (time.time() - t0)
            if sleep_left > 0:
                time.sleep(sleep_left)


class _BladderPreviewHttpHandler(BaseHTTPRequestHandler):
    server_version = "RosbridgeBladderHTTP/1.0"

    def do_GET(self) -> None:
        server: "BladderPreviewHttpServer" = self.server  # type: ignore[assignment]
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Rosbridge Bladder Detection</title>
  <style>
    body {{ background:#101214; color:#e8eaed; font-family:Arial,sans-serif; margin:0; padding:16px; }}
    img {{ max-width:100%; height:auto; border:1px solid #2f3439; }}
    .bar {{ display:flex; gap:12px; align-items:center; margin-bottom:10px; color:#b8c0c8; }}
    code {{ color:#d7e6ff; }}
  </style>
</head>
<body>
  <div class="bar">
    <strong>Rosbridge Bladder Detection</strong>
    <span id="status">connecting</span>
    <code>{server.topic}</code>
  </div>
  <img src="/stream.mjpg" alt="stream" />
  <script>
    async function refreshStatus() {{
      try {{
        const res = await fetch('/status.json', {{cache: 'no-store'}});
        const data = await res.json();
        document.getElementById('status').textContent =
          `frames=${{data.frame_count}} stream=${{data.stream_fps}}fps detect=${{data.detect_fps}}fps ${{data.status}}`;
      }} catch (err) {{
        document.getElementById('status').textContent = String(err);
      }}
    }}
    setInterval(refreshStatus, 1000);
    refreshStatus();
  </script>
</body>
</html>"""
            body = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/status.json":
            body = json.dumps(server.status_data(), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/snapshot.jpg":
            jpg_bytes, _ = server.get_jpeg_frame()
            if jpg_bytes is None:
                self.send_error(503)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpg_bytes)))
            self.end_headers()
            self.wfile.write(jpg_bytes)
            return

        if path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_stamp = None
            while True:
                jpg_bytes, stamp = server.get_jpeg_frame()
                if jpg_bytes is None:
                    time.sleep(0.1)
                    continue
                if last_stamp is not None and stamp == last_stamp:
                    time.sleep(0.03)
                    continue
                last_stamp = stamp
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg_bytes)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpg_bytes)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
            return

        self.send_error(404)

    def log_message(self, format: str, *args) -> None:
        return


class BladderPreviewHttpServer(ThreadingHTTPServer):
    def __init__(
        self,
        host: str,
        port: int,
        stream: RosbridgeCompressedStream,
        detector: DetectionWorker,
        topic: str,
        width: int,
        jpeg_quality: int,
    ) -> None:
        super().__init__((host, port), _BladderPreviewHttpHandler)
        self.stream = stream
        self.detector = detector
        self.topic = topic
        self.width = int(width)
        self.jpeg_quality = max(30, min(95, int(jpeg_quality)))
        self._http_thread: threading.Thread | None = None

    def start(self) -> None:
        self._http_thread = threading.Thread(target=self.serve_forever, name="bladder-preview-http", daemon=True)
        self._http_thread.start()

    def stop(self) -> None:
        self.shutdown()
        self.server_close()
        if self._http_thread is not None:
            self._http_thread.join(timeout=1.0)

    def _read_preview(self) -> tuple[np.ndarray, float]:
        raw_bgr, stamp, stream_fps, stream_err, _ = self.stream.read()
        overlay_bgr, detect_status, detect_err, detect_fps, detect_cost = self.detector.read()
        preview = _compose_preview(
            raw_bgr,
            stamp,
            stream_fps,
            stream_err,
            overlay_bgr,
            detect_status,
            detect_err,
            detect_fps,
            detect_cost,
            self.topic,
            self.width,
        )
        return preview, stamp if raw_bgr is not None else time.time()

    def get_jpeg_frame(self) -> tuple[bytes | None, float | None]:
        preview, stamp = self._read_preview()
        ok, buf = cv2.imencode(".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        return (buf.tobytes(), stamp) if ok else (None, None)

    def status_data(self) -> dict[str, object]:
        _, stamp, stream_fps, stream_err, count = self.stream.read()
        _, detect_status, detect_err, detect_fps, detect_cost = self.detector.read()
        return {
            "topic": self.topic,
            "frame_count": count,
            "stamp": stamp,
            "stream_fps": round(stream_fps, 1),
            "detect_fps": round(detect_fps, 1),
            "detect_cost_s": round(detect_cost, 3),
            "status": detect_status,
            "stream_error": stream_err,
            "detect_error": detect_err,
        }


def create_http_server_with_fallback(
    host: str,
    preferred_port: int,
    stream: RosbridgeCompressedStream,
    detector: DetectionWorker,
    topic: str,
    width: int,
    jpeg_quality: int,
    max_tries: int = 20,
) -> BladderPreviewHttpServer:
    if int(preferred_port) == 0:
        ports_to_try = [0]
    else:
        ports_to_try = [int(preferred_port) + i for i in range(max(1, int(max_tries)))]

    last_exc: Exception | None = None
    for port in ports_to_try:
        try:
            return BladderPreviewHttpServer(host, port, stream, detector, topic, width, jpeg_quality)
        except OSError as exc:
            last_exc = exc
            if exc.errno != errno.EADDRINUSE:
                raise
            continue
    if last_exc is not None:
        raise last_exc
    raise RuntimeError("failed to create HTTP preview server")


def main() -> None:
    args = parse_args()
    viewer = args.viewer
    if viewer == "auto":
        viewer = "opencv" if can_use_opencv_viewer() else "http"
    if viewer == "opencv" and not can_use_opencv_viewer():
        raise RuntimeError("current shell cannot access a local OpenCV display; use --viewer http instead")
    if not os.path.isfile(args.model_path):
        raise FileNotFoundError(f"model not found: {args.model_path}")

    ros = roslibpy.Ros(host=args.board_host, port=int(args.rosbridge_port))
    ros.run()
    if not ros.is_connected:
        raise RuntimeError(f"failed to connect rosbridge ws://{args.board_host}:{args.rosbridge_port}")

    stream = RosbridgeCompressedStream(ros, args.topic, topic_type=args.topic_type)
    detector = DetectionWorker(
        stream,
        model_path=os.path.abspath(args.model_path),
        finger_width_mm=args.finger_width_mm,
        conf=args.conf,
        detect_hz=args.detect_hz,
        rotation_mode=args.rotation_mode,
        auto_lock_frames=args.auto_lock_frames,
        smooth_alpha=args.smooth_alpha,
        deadband_px=args.deadband_px,
        hold_motion_threshold=args.hold_motion_threshold,
    )
    detector.start()

    window_name = "Realtime Bladder Detection (rosbridge)"
    print(f"rosbridge=ws://{args.board_host}:{args.rosbridge_port}")
    print(f"topic={args.topic} topic_type={args.topic_type}")
    print(
        f"viewer={viewer} model={os.path.abspath(args.model_path)} detect_hz={args.detect_hz} "
        f"rotation_mode={args.rotation_mode} auto_lock_frames={args.auto_lock_frames} "
        f"smooth_alpha={args.smooth_alpha} deadband_px={args.deadband_px} "
        f"hold_motion_threshold={args.hold_motion_threshold}"
    )
    if args.viewer == "auto" and viewer == "http" and os.environ.get("DISPLAY"):
        print("检测到 DISPLAY 存在但当前会话无法访问图形界面，已自动回退到浏览器预览")

    http_server: BladderPreviewHttpServer | None = None

    try:
        if viewer == "http":
            http_server = create_http_server_with_fallback(
                args.http_host,
                args.http_port,
                stream,
                detector,
                args.topic,
                args.width,
                args.jpeg_quality,
            )
            http_server.start()
            actual_port = int(http_server.server_address[1])
            if actual_port != int(args.http_port):
                print(f"端口 {args.http_port} 已占用，已自动切换到 {actual_port}")
            print(f"浏览器打开: http://{args.http_host}:{actual_port}")
            print("按 Ctrl+C 退出，浏览器中实时查看")
            while True:
                time.sleep(1.0)

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        print("按 q / ESC 退出，按 s 保存当前原图与叠加图")
        while True:
            raw_bgr, stamp, stream_fps, stream_err, _ = stream.read()
            overlay_bgr, detect_status, detect_err, detect_fps, detect_cost = detector.read()
            preview = _compose_preview(
                raw_bgr,
                stamp,
                stream_fps,
                stream_err,
                overlay_bgr,
                detect_status,
                detect_err,
                detect_fps,
                detect_cost,
                args.topic,
                args.width,
            )
            preview = _draw_lines(preview, ["keys: q/ESC quit | s save snapshot"], color=(200, 200, 200))
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and raw_bgr is not None:
                _save_snapshot(args.jpeg_save_dir, raw_bgr, overlay_bgr)
    finally:
        if http_server is not None:
            http_server.stop()
        detector.stop()
        stream.close()
        ros.terminate()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
