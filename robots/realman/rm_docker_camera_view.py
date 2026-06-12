"""Subscribe /camera/color/image_raw from the board's ROS master and show with OpenCV.

Run inside the local `noetic` docker (host network) against the board's rosmaster.
"""
from __future__ import annotations

import os
import signal
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

_WINDOW = "remote_camera_rgb (board@192.168.1.11)"


class ColorViewer:
    def __init__(self, topic: str = "/camera/color/image_raw") -> None:
        self._bridge = CvBridge()
        self._latest: np.ndarray | None = None
        self._last_stamp: float = 0.0
        self._count = 0
        self._first_stamp: float = 0.0
        self._topic = topic
        self._sub = rospy.Subscriber(topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24)

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"cv_bridge decode failed: {exc}")
            return
        now = time.time()
        if self._first_stamp == 0.0:
            self._first_stamp = now
        self._last_stamp = now
        self._count += 1

    @property
    def fps(self) -> float:
        if self._count <= 1 or self._last_stamp <= self._first_stamp:
            return 0.0
        return (self._count - 1) / (self._last_stamp - self._first_stamp)

    @property
    def frame(self) -> np.ndarray | None:
        return self._latest


def main() -> int:
    topic = os.environ.get("CAMERA_TOPIC", "/camera/color/image_raw")

    rospy.init_node("rm_docker_camera_view", anonymous=True, disable_signals=True)
    master = rospy.get_master().getUri()[2]  # type: ignore[attr-defined]
    print(f"[info] ROS master: {master}", flush=True)
    print(f"[info] subscribe: {topic}", flush=True)

    viewer = ColorViewer(topic)

    cv2.namedWindow(_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WINDOW, 960, 720)

    stop = {"flag": False}

    def _handle(signum, _frame):  # type: ignore[no-untyped-def]
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    t_wait0 = time.time()
    while not stop["flag"] and viewer.frame is None:
        if time.time() - t_wait0 > 10.0:
            print("[error] waited 10s, still no frames on topic", file=sys.stderr)
            return 2
        time.sleep(0.05)

    print("[info] first frame received, entering display loop (press q or ESC to quit)", flush=True)

    last_log = 0.0
    while not stop["flag"]:
        frame = viewer.frame
        if frame is None:
            time.sleep(0.01)
            continue

        overlay = frame.copy()
        fps = viewer.fps
        cv2.putText(
            overlay,
            f"fps={fps:5.1f}  {frame.shape[1]}x{frame.shape[0]}",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(_WINDOW, overlay)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

        now = time.time()
        if now - last_log > 2.0:
            last_log = now
            print(f"[stat] fps={fps:.1f} frames={viewer._count}", flush=True)

    cv2.destroyAllWindows()
    print("[info] exited cleanly", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
