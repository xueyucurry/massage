#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import threading
import time
from typing import Any

import cv2
import numpy as np

from view_bladder_detection_rosbridge import (
    ROTATION_MODE_CHOICES,
    DetectionWorker,
    create_http_server_with_fallback,
)

DEFAULT_CONTAINER = os.environ.get("RM_ROS_CONTAINER", "noetic")
DEFAULT_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://192.168.1.11:11311")
DEFAULT_ROS_IP = os.environ.get("ROS_IP", "192.168.1.250")
DEFAULT_TOPIC = os.environ.get("CAMERA_TOPIC", "/camera/color/image_raw")
DEFAULT_MODEL = os.path.abspath("yolo11l-pose.pt")

TOPIC_TYPE_CHOICES = ("raw", "compressed")


RELAY_PY = r"""
import os
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

topic = os.environ.get("CAMERA_TOPIC", "/camera/color/image_raw")
topic_type = os.environ.get("CAMERA_TOPIC_TYPE", "raw")
jpeg_quality = max(30, min(95, int(os.environ.get("JPEG_QUALITY", "82"))))
relay_hz = max(0.1, float(os.environ.get("RELAY_HZ", "12.0")))
bridge = CvBridge()
last_emit = 0.0


def _stamp(msg):
    try:
        value = float(msg.header.stamp.to_sec())
        return value if value > 0.0 else time.time()
    except Exception:
        return time.time()


def _write_frame(frame_bgr, stamp):
    ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    if not ok:
        return
    data = bytes(buf)
    sys.stdout.buffer.write(f"FRAME {stamp:.6f} {len(data)}\n".encode("ascii"))
    sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()


def _on_raw(msg):
    global last_emit
    now = time.time()
    if now - last_emit < 1.0 / relay_hz:
        return
    last_emit = now
    try:
        frame_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        _write_frame(frame_bgr, _stamp(msg))
    except Exception as exc:
        print(f"raw frame decode failed: {exc}", file=sys.stderr, flush=True)


def _on_compressed(msg):
    global last_emit
    now = time.time()
    if now - last_emit < 1.0 / relay_hz:
        return
    last_emit = now
    try:
        raw = bytes(msg.data)
        frame_bgr = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame_bgr is None:
            raise RuntimeError("cv2.imdecode returned None")
        _write_frame(frame_bgr, _stamp(msg))
    except Exception as exc:
        print(f"compressed frame decode failed: {exc}", file=sys.stderr, flush=True)


rospy.init_node("rm_docker_camera_jpeg_relay", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
msg_type = CompressedImage if topic_type == "compressed" else Image
callback = _on_compressed if topic_type == "compressed" else _on_raw
rospy.Subscriber(topic, msg_type, callback, queue_size=1, buff_size=2 ** 24)
print(
    f"relay subscribed topic={topic} type={topic_type} master={os.environ.get('ROS_MASTER_URI', '')}",
    file=sys.stderr,
    flush=True,
)
rospy.spin()
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Browser bladder detection preview with ROS1 subscription delegated to a local docker container."
    )
    parser.add_argument("--container", default=DEFAULT_CONTAINER, help="ROS1 docker container name")
    parser.add_argument("--master-uri", default=DEFAULT_MASTER_URI, help="ROS master URI on the board")
    parser.add_argument("--ros-ip", default=DEFAULT_ROS_IP, help="host ROS_IP reachable by the board")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="camera topic subscribed inside docker")
    parser.add_argument("--topic-type", choices=TOPIC_TYPE_CHOICES, default="raw", help="camera topic message type")
    parser.add_argument("--model-path", default=DEFAULT_MODEL, help="pose model path on host")
    parser.add_argument("--finger-width-mm", type=float, default=45.0, help="visual offset for bladder lines")
    parser.add_argument("--conf", type=float, default=0.5, help="pose confidence threshold")
    parser.add_argument("--detect-hz", type=float, default=1.5, help="host overlay inference rate")
    parser.add_argument("--relay-hz", type=float, default=12.0, help="docker camera frame relay rate")
    parser.add_argument("--rotation-mode", choices=ROTATION_MODE_CHOICES, default="cw90", help="pose rotation strategy")
    parser.add_argument("--auto-lock-frames", type=int, default=4, help="successful auto frames needed before locking")
    parser.add_argument("--smooth-alpha", type=float, default=0.75, help="EMA smoothing factor for keypoints")
    parser.add_argument("--deadband-px", type=float, default=2.5, help="ignore tiny keypoint movements below this")
    parser.add_argument("--hold-motion-threshold", type=float, default=2.0, help="motion threshold for holding pose")
    parser.add_argument("--width", type=int, default=1280, help="HTTP preview width")
    parser.add_argument("--http-host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--http-port", type=int, default=8766, help="HTTP bind port")
    parser.add_argument("--jpeg-quality", type=int, default=85, help="HTTP and relay JPEG quality")
    return parser.parse_args()


def _docker_command(argv: list[str]) -> list[str]:
    try:
        subprocess.run(["docker", "version"], check=True, capture_output=True, timeout=3)
        return ["docker", *argv]
    except Exception:
        return ["sg", "docker", "-c", shlex.join(["docker", *argv])]


class DockerRosImageStream:
    def __init__(
        self,
        container: str,
        master_uri: str,
        ros_ip: str,
        topic: str,
        topic_type: str,
        relay_hz: float,
        jpeg_quality: int,
    ) -> None:
        self.container = container
        self.master_uri = master_uri
        self.ros_ip = ros_ip
        self.topic = topic
        self.topic_type = topic_type
        self.relay_hz = max(0.1, float(relay_hz))
        self.jpeg_quality = max(30, min(95, int(jpeg_quality)))
        self.lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.latest_stamp = 0.0
        self.frame_count = 0
        self.first_stamp = 0.0
        self.last_error = "docker relay not started"
        self.proc: subprocess.Popen[bytes] | None = None
        self.stdout_thread: threading.Thread | None = None
        self.stderr_thread: threading.Thread | None = None

    def start(self) -> None:
        subprocess.run(_docker_command(["start", self.container]), check=False, capture_output=True, timeout=10)
        env_args = [
            "-e",
            f"ROS_MASTER_URI={self.master_uri}",
            "-e",
            f"ROS_IP={self.ros_ip}",
            "-e",
            f"ROS_HOSTNAME={self.ros_ip}",
            "-e",
            f"CAMERA_TOPIC={self.topic}",
            "-e",
            f"CAMERA_TOPIC_TYPE={self.topic_type}",
            "-e",
            f"RELAY_HZ={self.relay_hz}",
            "-e",
            f"JPEG_QUALITY={self.jpeg_quality}",
        ]
        inner_cmd = "source /opt/ros/noetic/setup.bash; python3 -u -"
        cmd = _docker_command(["exec", "-i", *env_args, self.container, "bash", "-lc", inner_cmd])
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if self.proc.stdin is None:
            raise RuntimeError("failed to open docker relay stdin")
        self.proc.stdin.write(RELAY_PY.encode("utf-8"))
        self.proc.stdin.close()
        self.stdout_thread = threading.Thread(target=self._stdout_loop, name="docker-ros-image-stdout", daemon=True)
        self.stderr_thread = threading.Thread(target=self._stderr_loop, name="docker-ros-image-stderr", daemon=True)
        self.stdout_thread.start()
        self.stderr_thread.start()

    def _set_error(self, text: str) -> None:
        with self.lock:
            self.last_error = text

    def _stderr_loop(self) -> None:
        assert self.proc is not None and self.proc.stderr is not None
        while True:
            line = self.proc.stderr.readline()
            if not line:
                break
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                self._set_error(text[-180:])

    def _stdout_loop(self) -> None:
        assert self.proc is not None and self.proc.stdout is not None
        stdout = self.proc.stdout
        while True:
            line = stdout.readline()
            if not line:
                break
            if not line.startswith(b"FRAME "):
                self._set_error(line.decode("utf-8", errors="replace").strip()[-180:])
                continue
            try:
                _, stamp_text, length_text = line.decode("ascii").strip().split()
                stamp = float(stamp_text)
                length = int(length_text)
                data = stdout.read(length)
                if len(data) != length:
                    raise RuntimeError(f"short jpeg frame: {len(data)} < {length}")
                frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError("host jpeg decode failed")
                now = time.time()
                with self.lock:
                    self.latest_frame = frame
                    self.latest_stamp = stamp if stamp > 0.0 else now
                    self.frame_count += 1
                    if self.first_stamp == 0.0:
                        self.first_stamp = now
                    self.last_error = ""
            except Exception as exc:
                self._set_error(str(exc))
        if self.proc is not None and self.proc.poll() is not None:
            self._set_error(f"docker relay exited rc={self.proc.returncode}")

    def read(self) -> tuple[np.ndarray | None, float, float, str, int]:
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
            stamp = self.latest_stamp
            err = self.last_error
            count = self.frame_count
            if self.frame_count <= 1 or self.first_stamp <= 0.0:
                fps = 0.0
            else:
                elapsed = max(0.001, time.time() - self.first_stamp)
                fps = (self.frame_count - 1) / elapsed
        return frame, stamp, fps, err, count

    def close(self) -> None:
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.stdout_thread is not None:
            self.stdout_thread.join(timeout=1.0)
        if self.stderr_thread is not None:
            self.stderr_thread.join(timeout=1.0)


def main() -> None:
    args = parse_args()
    if not os.path.isfile(args.model_path):
        raise FileNotFoundError(f"model not found: {args.model_path}")

    stream = DockerRosImageStream(
        container=args.container,
        master_uri=args.master_uri,
        ros_ip=args.ros_ip,
        topic=args.topic,
        topic_type=args.topic_type,
        relay_hz=args.relay_hz,
        jpeg_quality=args.jpeg_quality,
    )
    stream.start()
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
    http_server = None
    print(f"docker_container={args.container}", flush=True)
    print(f"ros_master={args.master_uri} ros_ip={args.ros_ip}", flush=True)
    print(f"topic={args.topic} topic_type={args.topic_type} relay_hz={args.relay_hz}", flush=True)
    print(
        f"model={os.path.abspath(args.model_path)} rotation_mode={args.rotation_mode} detect_hz={args.detect_hz}",
        flush=True,
    )
    try:
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
            print(f"端口 {args.http_port} 已占用，已自动切换到 {actual_port}", flush=True)
        print(f"浏览器打开: http://{args.http_host}:{actual_port}", flush=True)
        print("按 Ctrl+C 退出，浏览器中实时查看", flush=True)
        while True:
            time.sleep(1.0)
    finally:
        if http_server is not None:
            http_server.stop()
        detector.stop()
        stream.close()


if __name__ == "__main__":
    main()
