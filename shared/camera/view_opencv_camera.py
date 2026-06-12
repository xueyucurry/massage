#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime
import os
import time

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open a local V4L2 camera with OpenCV.")
    parser.add_argument("--device", type=int, default=4, help="V4L2 device index, e.g. 4 for /dev/video4")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--save-dir",
        default="/home/franka/massage/robots/realman/rm_demo_output/opencv_camera_snapshots",
        help="directory for snapshots saved with the s key",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    os.makedirs(args.save_dir, exist_ok=True)

    cap = cv2.VideoCapture(int(args.device), cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open /dev/video{int(args.device)}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(args.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(args.height))
    cap.set(cv2.CAP_PROP_FPS, int(args.fps))

    window = f"OpenCV Camera /dev/video{int(args.device)}"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"[camera] opened /dev/video{int(args.device)}", flush=True)
    frame_count = 0
    last_report = time.time()
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[camera] failed to read frame", flush=True)
                time.sleep(0.05)
                continue
            frame_count += 1
            cv2.putText(
                frame,
                f"/dev/video{int(args.device)}  q/ESC quit  s save",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
            cv2.imshow(window, frame)
            now = time.time()
            if now - last_report > 2.0:
                print(f"[camera] streaming frames={frame_count} shape={frame.shape}", flush=True)
                last_report = now
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                print("[camera] quit key pressed", flush=True)
                break
            if key == ord("s"):
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                path = os.path.join(args.save_dir, f"camera_{ts}.png")
                cv2.imwrite(path, frame)
                print(f"[camera] saved {path}", flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[camera] closed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
