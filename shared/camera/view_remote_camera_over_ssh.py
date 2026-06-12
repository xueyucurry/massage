#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import datetime
import errno
import io
import json
import os
import subprocess
import textwrap
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


DEFAULT_SSH_TARGET = os.environ.get("RM_BOARD_SSH", "rm@192.168.1.11")
DEFAULT_REMOTE_SETUP = (
    "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
    "source /home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/setup.bash >/dev/null 2>&1"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview remote camera frames over SSH without writing files on the remote host."
    )
    parser.add_argument("--ssh-target", default=DEFAULT_SSH_TARGET, help="SSH target, e.g. rm@192.168.1.11")
    parser.add_argument(
        "--remote-setup",
        default=DEFAULT_REMOTE_SETUP,
        help="commands executed on the remote host before running the capture snippet",
    )
    parser.add_argument("--mode", choices=("color", "rgbd"), default="color", help="preview color only or color+depth")
    parser.add_argument(
        "--viewer",
        choices=("auto", "opencv", "http"),
        default="auto",
        help="preview backend: OpenCV window or browser HTTP stream",
    )
    parser.add_argument("--width", type=int, default=960, help="preview width for the color panel")
    parser.add_argument("--max-depth-mm", type=float, default=3000.0, help="depth colormap upper bound")
    parser.add_argument("--refresh-period-s", type=float, default=0.25, help="delay between remote snapshots")
    parser.add_argument("--jpeg-quality", type=int, default=75, help="remote JPEG quality for color frames")
    parser.add_argument("--output-dir", default="remote_camera_snapshots", help="local snapshot directory")
    parser.add_argument("--http-host", default="127.0.0.1", help="HTTP bind host for browser preview")
    parser.add_argument("--http-port", type=int, default=8765, help="HTTP port for browser preview")
    return parser.parse_args()


def colorize_depth(depth_m: np.ndarray, max_depth_mm: float = 3000.0) -> np.ndarray:
    depth_mm = depth_m.astype(np.float32) * 1000.0
    valid = depth_mm > 0
    vis = np.zeros_like(depth_mm, dtype=np.uint8)
    if np.any(valid):
        clipped = np.clip(depth_mm, 0.0, max_depth_mm)
        vis[valid] = np.round(clipped[valid] / max_depth_mm * 255.0).astype(np.uint8)
    colored = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    colored[~valid] = 0
    return colored


def can_use_opencv_viewer() -> bool:
    if not os.environ.get("DISPLAY"):
        return False
    checks = [
        ["xdpyinfo"],
        ["xset", "q"],
    ]
    for cmd in checks:
        try:
            proc = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0, check=False)
        except Exception:
            continue
        if proc.returncode == 0:
            return True
    return False


class SshRgbdGrabber:
    def __init__(
        self,
        ssh_target: str,
        remote_setup: str,
        refresh_period_s: float = 0.8,
        include_depth: bool = False,
        jpeg_quality: int = 75,
    ) -> None:
        self.ssh_target = ssh_target
        self.remote_setup = remote_setup.strip()
        self.refresh_period_s = max(0.2, float(refresh_period_s))
        self.include_depth = bool(include_depth)
        self.jpeg_quality = max(30, min(95, int(jpeg_quality)))
        self._last_fetch_time = 0.0
        self._latest_frame: dict[str, object] | None = None
        self._last_error: str | None = None

    def get_latest(self, force: bool = False) -> tuple[dict[str, object] | None, str | None]:
        now = time.time()
        if (
            not force
            and self._latest_frame is not None
            and (now - self._last_fetch_time) < self.refresh_period_s
        ):
            return self._latest_frame, None

        try:
            payload = self._capture_once()
            color_bgr = cv2.imdecode(
                np.frombuffer(base64.b64decode(payload["color_jpg_b64"]), dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if color_bgr is None:
                raise RuntimeError("failed to decode remote color image")
            intrinsics = dict(payload.get("intrinsics", {}))
            frame: dict[str, object] = {
                "stamp": float(payload.get("stamp", time.time())),
                "color_bgr": color_bgr,
                "intrinsics": intrinsics,
            }
            if "depth_npy_b64" in payload:
                depth_m = np.load(io.BytesIO(base64.b64decode(payload["depth_npy_b64"]))).astype(np.float32)
                frame["depth_m"] = depth_m
            self._latest_frame = frame
            self._last_fetch_time = now
            self._last_error = None
            return self._latest_frame, None
        except Exception as exc:
            self._last_error = str(exc)
            self._last_fetch_time = now
            return self._latest_frame, self._last_error

    def _capture_once(self) -> dict[str, object]:
        remote_cmd = f"{self.remote_setup}; python3 -"
        depth_import = "import io\nimport numpy as np" if self.include_depth else ""
        depth_wait = 'depth_msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=8.0)' if self.include_depth else ""
        depth_decode = textwrap.dedent(
            """
            depth_raw = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth_raw.dtype == np.uint16:
                depth_m = depth_raw.astype(np.float32) * 0.001
            else:
                depth_m = depth_raw.astype(np.float32)
            """
        ).strip() if self.include_depth else ""
        depth_encode = textwrap.dedent(
            """
            depth_io = io.BytesIO()
            np.save(depth_io, depth_m)
            payload["depth_npy_b64"] = base64.b64encode(depth_io.getvalue()).decode("ascii")
            """
        ).strip() if self.include_depth else ""
        remote_py = textwrap.dedent(
            """
            import base64
            import json
            import time

            import cv2
            import rospy
            from cv_bridge import CvBridge
            from sensor_msgs.msg import CameraInfo, Image
            __DEPTH_IMPORT__

            if not rospy.core.is_initialized():
                rospy.init_node("view_remote_camera_over_ssh", anonymous=True, disable_signals=True)

            bridge = CvBridge()
            default_intrinsics = {
                "width": 640,
                "height": 480,
                "ppx": 318.4939270019531,
                "ppy": 245.07725524902344,
                "fx": 609.5076904296875,
                "fy": 609.6961059570312,
                "model_name": "brown_conrady",
                "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
                "depth_scale": 0.001,
            }

            intrinsics = dict(default_intrinsics)
            try:
                cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=2.0)
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
                    "depth_scale": 0.001,
                }
            except Exception:
                pass

            color_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=8.0)
            __DEPTH_WAIT__
            color_bgr = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            __DEPTH_DECODE__

            ok, color_buf = cv2.imencode(".jpg", color_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), __JPEG_QUALITY__])
            if not ok:
                raise RuntimeError("jpg encode failed")
            payload = {
                "stamp": time.time(),
                "intrinsics": intrinsics,
                "color_jpg_b64": base64.b64encode(color_buf.tobytes()).decode("ascii"),
            }
            __DEPTH_ENCODE__
            print(json.dumps(payload, ensure_ascii=False))
            """
        )
        remote_py = (
            remote_py.replace("__DEPTH_IMPORT__", depth_import)
            .replace("__DEPTH_WAIT__", depth_wait)
            .replace("__DEPTH_DECODE__", depth_decode)
            .replace("__JPEG_QUALITY__", str(self.jpeg_quality))
            .replace("__DEPTH_ENCODE__", depth_encode)
        )
        proc = subprocess.run(
            ["ssh", self.ssh_target, "bash", "-lc", remote_cmd],
            input=remote_py,
            text=True,
            capture_output=True,
            timeout=20.0,
            check=False,
        )
        if proc.returncode != 0:
            raise RuntimeError(proc.stderr.strip() or proc.stdout.strip() or "remote capture failed")
        lines = (proc.stdout or "").strip().splitlines()
        if not lines:
            raise RuntimeError("remote capture returned empty stdout")
        return json.loads(lines[-1])


class SshFrameWorker:
    def __init__(self, grabber: SshRgbdGrabber) -> None:
        self.grabber = grabber
        self.lock = threading.Lock()
        self.latest_frame: dict[str, object] | None = None
        self.latest_error: str = ""
        self.running = False
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="ssh-camera-worker", daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        while self.running:
            frame, err = self.grabber.get_latest(force=True)
            with self.lock:
                if frame is not None:
                    self.latest_frame = frame
                if err:
                    self.latest_error = err
                elif frame is not None:
                    self.latest_error = ""

    def read(self) -> tuple[dict[str, object] | None, str]:
        with self.lock:
            frame = self.latest_frame
            err = self.latest_error
        return frame, err

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)


class _PreviewHttpHandler(BaseHTTPRequestHandler):
    server_version = "RemoteCameraHTTP/1.0"

    def do_GET(self) -> None:
        server: "PreviewHttpServer" = self.server  # type: ignore[assignment]
        if self.path in ("/", "/index.html"):
            html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Remote Camera Preview</title>
  <style>
    body {{ background:#111; color:#eee; font-family:sans-serif; margin:0; padding:16px; }}
    img {{ max-width:100%; height:auto; border:1px solid #333; }}
    .hint {{ color:#aaa; margin-top:8px; }}
  </style>
</head>
<body>
  <h3>Remote Camera Preview</h3>
  <img src="/stream.mjpg" alt="stream" />
  <div class="hint">source={server.status_text()}</div>
</body>
</html>"""
            body = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/stream.mjpg":
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


class PreviewHttpServer(ThreadingHTTPServer):
    def __init__(self, host: str, port: int, worker: SshFrameWorker, width: int, max_depth_mm: float) -> None:
        super().__init__((host, port), _PreviewHttpHandler)
        self.worker = worker
        self.width = width
        self.max_depth_mm = max_depth_mm
        self._http_thread: threading.Thread | None = None

    def start(self) -> None:
        self._http_thread = threading.Thread(target=self.serve_forever, name="preview-http", daemon=True)
        self._http_thread.start()

    def stop(self) -> None:
        self.shutdown()
        self.server_close()
        if self._http_thread is not None:
            self._http_thread.join(timeout=1.0)

    def get_jpeg_frame(self) -> tuple[bytes | None, float | None]:
        frame, err = self.worker.read()
        if frame is None:
            canvas = np.zeros((360, max(640, self.width), 3), dtype=np.uint8)
            cv2.putText(canvas, "waiting for remote camera frame...", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
            if err:
                cv2.putText(canvas, err[:120], (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
            ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            return (buf.tobytes(), None) if ok else (None, None)
        preview = build_preview(frame, self.width, self.max_depth_mm, f"ssh stream | {self.status_text()}")
        ok, buf = cv2.imencode(".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        return (buf.tobytes(), float(frame["stamp"])) if ok else (None, None)

    def status_text(self) -> str:
        frame, err = self.worker.read()
        if frame is None:
            return err or "waiting"
        if err:
            return err[:120]
        return "ok"


def create_http_server_with_fallback(
    host: str,
    preferred_port: int,
    worker: SshFrameWorker,
    width: int,
    max_depth_mm: float,
    max_tries: int = 20,
) -> PreviewHttpServer:
    ports_to_try: list[int]
    if int(preferred_port) == 0:
        ports_to_try = [0]
    else:
        ports_to_try = [int(preferred_port) + i for i in range(max(1, int(max_tries)))]

    last_exc: Exception | None = None
    for port in ports_to_try:
        try:
            return PreviewHttpServer(host, port, worker, width, max_depth_mm)
        except OSError as exc:
            last_exc = exc
            if exc.errno != errno.EADDRINUSE:
                raise
            continue
    if last_exc is not None:
        raise last_exc
    raise RuntimeError("failed to create HTTP preview server")


def build_preview(frame: dict[str, object], width: int, max_depth_mm: float, status_line: str) -> np.ndarray:
    color_bgr = np.asarray(frame["color_bgr"])
    h, w = color_bgr.shape[:2]
    panel_w = max(160, int(width))
    panel_h = max(120, int(round(panel_w * h / max(1, w))))
    color_show = cv2.resize(color_bgr, (panel_w, panel_h))
    has_depth = "depth_m" in frame
    if has_depth:
        depth_m = np.asarray(frame["depth_m"])
        depth_vis = colorize_depth(depth_m, max_depth_mm=max_depth_mm)
        depth_show = cv2.resize(depth_vis, (panel_w, panel_h))
        combined = np.hstack([color_show, depth_show])
        center_y = depth_m.shape[0] // 2
        center_x = depth_m.shape[1] // 2
        center_depth_m = float(depth_m[center_y, center_x]) if depth_m.size else 0.0
        extra_line = f"remote_stamp={datetime.datetime.fromtimestamp(float(frame['stamp'])).strftime('%H:%M:%S')} center_depth={center_depth_m:.3f}m"
    else:
        combined = color_show
        extra_line = f"remote_stamp={datetime.datetime.fromtimestamp(float(frame['stamp'])).strftime('%H:%M:%S')}"

    lines = [
        status_line,
        extra_line,
        "keys: q/ESC quit | s save snapshot",
    ]
    y = 28
    for line in lines:
        cv2.putText(combined, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
        y += 28
    return combined


def save_snapshot(output_dir: str, frame: dict[str, object]) -> None:
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    color_path = os.path.join(output_dir, f"remote_color_{ts}.png")
    color_bgr = np.asarray(frame["color_bgr"])
    cv2.imwrite(color_path, color_bgr)
    print(f"[Saved] {color_path}")
    if "depth_m" in frame:
        depth_path = os.path.join(output_dir, f"remote_depth_{ts}.npy")
        depth_vis_path = os.path.join(output_dir, f"remote_depth_vis_{ts}.png")
        depth_m = np.asarray(frame["depth_m"])
        np.save(depth_path, depth_m)
        cv2.imwrite(depth_vis_path, colorize_depth(depth_m))
        print(f"[Saved] {depth_path}")
        print(f"[Saved] {depth_vis_path}")


def main() -> None:
    args = parse_args()
    viewer = args.viewer
    if viewer == "auto":
        viewer = "opencv" if can_use_opencv_viewer() else "http"
    grabber = SshRgbdGrabber(
        ssh_target=args.ssh_target,
        remote_setup=args.remote_setup,
        refresh_period_s=args.refresh_period_s,
        include_depth=args.mode == "rgbd",
        jpeg_quality=args.jpeg_quality,
    )
    worker = SshFrameWorker(grabber)
    print(f"SSH remote camera viewer -> {args.ssh_target}")
    print(
        f"mode={args.mode} viewer={viewer} "
        f"refresh_period={args.refresh_period_s:.2f}s jpeg_quality={args.jpeg_quality}"
    )
    if args.viewer == "auto" and viewer == "http" and os.environ.get("DISPLAY"):
        print("检测到 DISPLAY 存在但当前会话无法访问图形界面，已自动回退到浏览器预览")

    frame: dict[str, object] | None = None
    last_error = ""
    window_name = "Remote Camera Over SSH"
    worker.start()
    http_server: PreviewHttpServer | None = None

    try:
        if viewer == "http":
            http_server = create_http_server_with_fallback(
                args.http_host,
                args.http_port,
                worker,
                args.width,
                args.max_depth_mm,
            )
            http_server.start()
            actual_port = int(http_server.server_address[1])
            if actual_port != int(args.http_port):
                print(f"端口 {args.http_port} 已占用，已自动切换到 {actual_port}")
            print(f"浏览器打开: http://{args.http_host}:{actual_port}")
            print("按 Ctrl+C 退出，浏览器中实时查看")
            while True:
                time.sleep(1.0)

        print("按 q / ESC 退出，按 s 保存当前画面")
        while True:
            frame, err = worker.read()
            if err:
                last_error = err

            if frame is not None:
                status = f"ssh={args.ssh_target}"
                if last_error:
                    status += f" | last_error={last_error[:80]}"
                preview = build_preview(frame, args.width, args.max_depth_mm, status)
            else:
                preview = np.zeros((360, max(640, args.width * 2), 3), dtype=np.uint8)
                cv2.putText(preview, "waiting for remote camera frame...", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
                if last_error:
                    cv2.putText(preview, last_error[:120], (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(preview, "keys: q/ESC quit", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2, cv2.LINE_AA)

            cv2.imshow(window_name, preview)
            key = cv2.waitKey(15) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s") and frame is not None:
                save_snapshot(args.output_dir, frame)
    finally:
        if http_server is not None:
            http_server.stop()
        worker.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
