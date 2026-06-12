#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime
import os
import threading
import time

import cv2
import numpy as np

from rm_demo.rm_bladder import detect_bladder_lines
from view_remote_camera_over_ssh import (
    DEFAULT_REMOTE_SETUP,
    DEFAULT_SSH_TARGET,
    SshFrameWorker,
    SshRgbdGrabber,
    can_use_opencv_viewer,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Realtime bladder meridian detection preview over the remote camera."
    )
    parser.add_argument("--ssh-target", default=DEFAULT_SSH_TARGET, help="SSH target, e.g. rm@192.168.1.11")
    parser.add_argument(
        "--remote-setup",
        default=DEFAULT_REMOTE_SETUP,
        help="commands executed on the remote host before running the capture snippet",
    )
    parser.add_argument("--refresh-period-s", type=float, default=0.35, help="delay between remote snapshots")
    parser.add_argument("--jpeg-quality", type=int, default=80, help="remote JPEG quality")
    parser.add_argument("--width", type=int, default=1100, help="display width")
    parser.add_argument("--model-path", default=os.path.abspath("yolo11l-pose.pt"), help="pose model path")
    parser.add_argument("--finger-width-mm", type=float, default=45.0, help="visual offset for bladder lines")
    parser.add_argument("--sample-points", type=int, default=30, help="line sampling count")
    parser.add_argument("--conf", type=float, default=0.5, help="YOLO confidence threshold")
    parser.add_argument("--output-dir", default="remote_bladder_preview", help="snapshot directory")
    return parser.parse_args()


def _fit_width(image: np.ndarray, width: int) -> np.ndarray:
    h, w = image.shape[:2]
    target_w = max(320, int(width))
    target_h = max(180, int(round(target_w * h / max(1, w))))
    if target_w == w:
        return image
    return cv2.resize(image, (target_w, target_h))


def _stack_raw_and_overlay(raw_bgr: np.ndarray, overlay_bgr: np.ndarray) -> np.ndarray:
    if raw_bgr.shape[:2] != overlay_bgr.shape[:2]:
        overlay_bgr = cv2.resize(overlay_bgr, (raw_bgr.shape[1], raw_bgr.shape[0]))
    raw = raw_bgr.copy()
    cv2.putText(raw, "raw", (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2, cv2.LINE_AA)
    overlay = overlay_bgr.copy()
    cv2.putText(overlay, "overlay", (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2, cv2.LINE_AA)
    return np.hstack([raw, overlay])


def _draw_lines(image: np.ndarray, lines: list[str], color: tuple[int, int, int] = (0, 255, 255)) -> np.ndarray:
    canvas = image.copy()
    y = 28
    for line in lines:
        cv2.putText(canvas, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        y += 28
    return canvas


def _save_snapshot(output_dir: str, overlay_bgr: np.ndarray, raw_bgr: np.ndarray, depth_m: np.ndarray | None) -> None:
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    overlay_path = os.path.join(output_dir, f"bladder_overlay_{ts}.png")
    raw_path = os.path.join(output_dir, f"bladder_raw_{ts}.png")
    cv2.imwrite(overlay_path, overlay_bgr)
    cv2.imwrite(raw_path, raw_bgr)
    print(f"[Saved] {overlay_path}")
    print(f"[Saved] {raw_path}")
    if depth_m is not None:
        depth_path = os.path.join(output_dir, f"bladder_depth_{ts}.npy")
        np.save(depth_path, depth_m)
        print(f"[Saved] {depth_path}")


class DetectionWorker:
    def __init__(
        self,
        frame_worker: SshFrameWorker,
        *,
        model_path: str,
        finger_width_mm: float,
        sample_points: int,
        conf: float,
    ) -> None:
        self.frame_worker = frame_worker
        self.model_path = model_path
        self.finger_width_mm = float(finger_width_mm)
        self.sample_points = int(sample_points)
        self.conf = float(conf)
        self.lock = threading.Lock()
        self.running = False
        self.thread: threading.Thread | None = None
        self.last_stamp = -1.0
        self.latest_overlay: np.ndarray | None = None
        self.latest_color: np.ndarray | None = None
        self.latest_depth: np.ndarray | None = None
        self.latest_status = "waiting for remote frames..."
        self.latest_error = ""
        self.detect_count = 0
        self.last_detect_dt = 0.0

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="bladder-detect-worker", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def read(self) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, str, str]:
        with self.lock:
            overlay = None if self.latest_overlay is None else self.latest_overlay.copy()
            color = None if self.latest_color is None else self.latest_color.copy()
            depth = None if self.latest_depth is None else self.latest_depth.copy()
            status = self.latest_status
            error = self.latest_error
        return overlay, color, depth, status, error

    def _loop(self) -> None:
        while self.running:
            frame, frame_err = self.frame_worker.read()
            if frame is None:
                if frame_err:
                    with self.lock:
                        self.latest_error = frame_err
                        self.latest_status = "waiting for remote frames..."
                time.sleep(0.05)
                continue

            stamp = float(frame.get("stamp", 0.0))
            if stamp <= self.last_stamp:
                time.sleep(0.01)
                continue

            color_bgr = np.asarray(frame["color_bgr"]).copy()
            depth_m = np.asarray(frame["depth_m"], dtype=np.float32).copy() if "depth_m" in frame else None
            intrinsics = dict(frame.get("intrinsics", {}))
            t0 = time.time()
            try:
                if depth_m is None:
                    raise RuntimeError("remote frame does not include depth")
                detect_result, overlay = detect_bladder_lines(
                    color_bgr=color_bgr,
                    depth_m=depth_m,
                    intrinsics_data=intrinsics,
                    finger_width_mm=self.finger_width_mm,
                    model_path=self.model_path,
                    sample_points=self.sample_points,
                    conf=self.conf,
                )
                shoulder_cm = "N/A"
                left_inner = list(detect_result.get("left_inner_camera", []))
                right_inner = list(detect_result.get("right_inner_camera", []))
                if left_inner and right_inner:
                    shoulder_cm = str(round(float(np.linalg.norm(np.asarray(right_inner[0]) - np.asarray(left_inner[0])) * 100.0), 1))
                self.last_detect_dt = time.time() - t0
                self.detect_count += 1
                status = (
                    f"ok | pose_rot={detect_result.get('pose_rotation', 'n/a')} "
                    f"| detect={self.last_detect_dt:.2f}s | shoulder_est={shoulder_cm}cm"
                )
                error = frame_err or ""
            except Exception as exc:
                self.last_detect_dt = time.time() - t0
                overlay = _draw_lines(
                    color_bgr,
                    [
                        "detect failed",
                        str(exc)[:120],
                        f"detect={self.last_detect_dt:.2f}s | adjust camera until both shoulders and hips are visible",
                    ],
                    color=(0, 0, 255),
                )
                status = f"detect_failed | detect={self.last_detect_dt:.2f}s"
                error = str(exc)

            with self.lock:
                self.last_stamp = stamp
                self.latest_color = color_bgr
                self.latest_depth = depth_m
                self.latest_overlay = overlay
                self.latest_status = status
                self.latest_error = error


def main() -> None:
    args = parse_args()
    if not can_use_opencv_viewer():
        raise RuntimeError("current shell cannot access a local OpenCV display; use a desktop terminal first")

    grabber = SshRgbdGrabber(
        ssh_target=args.ssh_target,
        remote_setup=args.remote_setup,
        refresh_period_s=args.refresh_period_s,
        include_depth=True,
        jpeg_quality=args.jpeg_quality,
    )
    frame_worker = SshFrameWorker(grabber)
    detect_worker = DetectionWorker(
        frame_worker,
        model_path=os.path.abspath(args.model_path),
        finger_width_mm=args.finger_width_mm,
        sample_points=args.sample_points,
        conf=args.conf,
    )

    window_name = "Realtime Bladder Detection"
    frame_worker.start()
    detect_worker.start()
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print(f"实时膀胱经检测 -> {args.ssh_target}")
    print(
        f"refresh_period={args.refresh_period_s:.2f}s jpeg_quality={args.jpeg_quality} "
        f"model={os.path.abspath(args.model_path)}"
    )
    print("按 q / ESC 退出，按 s 保存当前叠加图")

    try:
        while True:
            overlay, raw_bgr, depth_m, status, error = detect_worker.read()
            if overlay is None:
                if raw_bgr is not None:
                    preview = _draw_lines(
                        raw_bgr,
                        [
                            "raw camera only | detection pending",
                            error[:120] if error else "first model load may take a moment",
                        ],
                        color=(0, 255, 255),
                    )
                else:
                    canvas = np.zeros((360, max(640, args.width), 3), dtype=np.uint8)
                    lines = [
                        "waiting for remote frames...",
                        error[:120] if error else "first model load may take a moment",
                    ]
                    preview = _draw_lines(canvas, lines, color=(0, 255, 255))
            else:
                if raw_bgr is not None:
                    preview_base = _stack_raw_and_overlay(raw_bgr, overlay)
                else:
                    preview_base = overlay
                lines = [
                    f"ssh={args.ssh_target}",
                    status,
                    f"stamp={datetime.datetime.now().strftime('%H:%M:%S')} | q/ESC quit | s save snapshot",
                ]
                preview = _draw_lines(preview_base, lines, color=(0, 255, 255))

            preview = _fit_width(preview, args.width)
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and overlay is not None and raw_bgr is not None:
                _save_snapshot(args.output_dir, overlay, raw_bgr, depth_m)
    finally:
        detect_worker.stop()
        frame_worker.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
