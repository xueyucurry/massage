"""MediaPipe hand-gesture input for an active FAIRINO massage session.

This module deliberately has no dependency on the robot SDK.  It turns stable
MediaPipe gesture results into a small command vocabulary and leaves all robot
motion to :mod:`runtime`, which already owns the safe pause/resume/stop path.
"""

import argparse
import os
import threading
import time
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple


GESTURE_TO_COMMAND = {
    "Closed_Fist": "pause",
    "Thumb_Up": "resume",
    "Open_Palm": "stop",
}
ACTIVE_STATUSES = {"running", "pausing", "paused", "stopping"}
REALSENSE_CAMERA_NAMES = {"realsense", "rs", "d435", "d435i"}


def _env_enabled(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _camera_source(value: str):
    """Accept either an OpenCV device number or a device/file path."""
    value = str(value).strip()
    try:
        return int(value)
    except ValueError:
        return value


class _OpenCVFrameSource:
    def __init__(self, cv2, camera):
        backend = cv2.CAP_V4L2 if isinstance(camera, int) and os.name != "nt" else cv2.CAP_ANY
        self.capture = cv2.VideoCapture(camera, backend)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f"无法打开 OpenCV 摄像头 {camera}")
        self.label = str(camera)

    def read(self):
        return self.capture.read()

    def close(self):
        self.capture.release()


class _RealSenseFrameSource:
    """A color-only RealSense stream opened after massage detection has closed."""

    def __init__(self):
        try:
            import numpy as np
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("当前 GUI Python 环境未安装 pyrealsense2") from exc

        self.np = np
        self.pipeline = rs.pipeline()
        config = rs.config()
        width = int(os.environ.get("FAIRINO_GESTURE_REALSENSE_WIDTH", "640"))
        height = int(os.environ.get("FAIRINO_GESTURE_REALSENSE_HEIGHT", "480"))
        fps = int(os.environ.get("FAIRINO_GESTURE_REALSENSE_FPS", "30"))
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(config)
        self.label = f"RealSense {width}x{height}@{fps}"

    def read(self):
        frames = self.pipeline.wait_for_frames(1500)
        color = frames.get_color_frame()
        if not color:
            return False, None
        return True, self.np.asanyarray(color.get_data()).copy()

    def close(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


class GestureCommandGate:
    """Require a stable, state-valid gesture and a neutral re-arm period."""

    def __init__(self, stable_frames: int = 5, neutral_frames: int = 6):
        self.stable_frames = max(1, int(stable_frames))
        self.neutral_frames = max(1, int(neutral_frames))
        self._candidate: Optional[str] = None
        self._candidate_count = 0
        self._neutral_count = 0
        self._armed = True

    @staticmethod
    def _is_allowed(command: str, state: Dict[str, object]) -> bool:
        status = str(state.get("status") or "").strip().lower()
        worker_alive = bool(state.get("worker_alive"))
        if command == "pause":
            return status == "running" and worker_alive
        if command == "resume":
            # A stopped task is deliberately not resumable.
            return status == "paused" and not worker_alive
        if command == "stop":
            return status in {"running", "pausing", "paused"}
        return False

    def observe(self, gesture: Optional[str], state: Dict[str, object]) -> Optional[str]:
        """Return one command, or ``None`` when no safe new command exists."""
        command = GESTURE_TO_COMMAND.get(str(gesture or ""))
        if command is None or not self._is_allowed(command, state):
            self._candidate = None
            self._candidate_count = 0
            self._neutral_count += 1
            if self._neutral_count >= self.neutral_frames:
                self._armed = True
            return None

        self._neutral_count = 0
        if command != self._candidate:
            self._candidate = command
            self._candidate_count = 1
            if self._armed and self._candidate_count >= self.stable_frames:
                self._armed = False
                return command
            return None

        self._candidate_count += 1
        if self._armed and self._candidate_count >= self.stable_frames:
            self._armed = False
            return command
        return None


class GestureControlMonitor:
    """Read a camera, recognize gestures, and dispatch safe session commands.

    Heavy imports are intentionally delayed until the worker thread starts so a
    missing optional MediaPipe package never prevents the GUI or robot service
    from starting.  The status callback reports that condition to the session.
    """

    def __init__(
        self,
        state_provider: Callable[[], Dict[str, object]],
        command_callback: Callable[[str], bool],
        status_callback: Callable[[str, str], None],
        observation_callback: Optional[Callable[[Optional[str], float, bool], None]] = None,
        *,
        model_path: Optional[Path] = None,
        camera: Optional[str] = None,
        display: Optional[bool] = None,
    ):
        project_root = Path(__file__).resolve().parents[5]
        self.state_provider = state_provider
        self.command_callback = command_callback
        self.status_callback = status_callback
        self.observation_callback = observation_callback
        self.model_path = Path(
            model_path
            or os.environ.get("FAIRINO_GESTURE_MODEL_PATH")
            or project_root / "gesture_recognizer.task"
        )
        raw_camera = camera if camera is not None else os.environ.get("FAIRINO_GESTURE_CAMERA", "realsense")
        self.camera = _camera_source(raw_camera)
        self.display = _env_enabled("FAIRINO_GESTURE_DISPLAY", False) if display is None else bool(display)
        self.min_score = float(os.environ.get("FAIRINO_GESTURE_MIN_SCORE", "0.60"))
        self.gate = GestureCommandGate(
            stable_frames=int(os.environ.get("FAIRINO_GESTURE_STABLE_FRAMES", "3")),
            neutral_frames=int(os.environ.get("FAIRINO_GESTURE_NEUTRAL_FRAMES", "4")),
        )
        self._stop_event = threading.Event()
        self._last_observation_at = 0.0
        self._last_observation_key: Optional[Tuple[Optional[str], int, bool]] = None

    def stop(self):
        self._stop_event.set()

    @staticmethod
    def _top_gesture(result) -> Tuple[Optional[str], float]:
        gestures = getattr(result, "gestures", None) or []
        if not gestures or not gestures[0]:
            return None, 0.0
        top = gestures[0][0]
        return getattr(top, "category_name", None), float(getattr(top, "score", 0.0) or 0.0)

    @staticmethod
    def _landmark_fallback(result) -> Optional[str]:
        """Recognize only the two fail-safe hand shapes from landmarks.

        The canned classifier is the authority when it reports one of the
        requested labels. Some camera angles make it report ``None`` for a
        clear fist or palm, though the same Gesture Recognizer still returns
        reliable landmarks. This fallback only pauses or stops; it never
        resumes a task.
        """
        hands = getattr(result, "hand_landmarks", None) or []
        if not hands or len(hands[0]) < 21:
            return None
        landmarks = hands[0]

        def angle_degrees(a, b, c) -> float:
            # Angle at b, using xyz so hand orientation does not matter.
            ab = (
                float(a.x) - float(b.x),
                float(a.y) - float(b.y),
                float(getattr(a, "z", 0.0)) - float(getattr(b, "z", 0.0)),
            )
            cb = (
                float(c.x) - float(b.x),
                float(c.y) - float(b.y),
                float(getattr(c, "z", 0.0)) - float(getattr(b, "z", 0.0)),
            )
            ab_norm = sum(component * component for component in ab) ** 0.5
            cb_norm = sum(component * component for component in cb) ** 0.5
            if ab_norm < 1e-6 or cb_norm < 1e-6:
                return 0.0
            cosine = sum(left * right for left, right in zip(ab, cb)) / (ab_norm * cb_norm)
            cosine = max(-1.0, min(1.0, cosine))
            import math

            return math.degrees(math.acos(cosine))

        # (MCP, PIP, tip) for index, middle, ring, and little fingers.
        joint_angles = [
            angle_degrees(landmarks[mcp], landmarks[pip], landmarks[tip])
            for mcp, pip, tip in ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20))
        ]
        extended = sum(value >= 150.0 for value in joint_angles)
        folded = sum(value <= 95.0 for value in joint_angles)
        if extended >= 3:
            return "Open_Palm"
        if folded >= 3:
            return "Closed_Fist"
        return None

    def _keep_running(self) -> bool:
        state = self.state_provider()
        return str(state.get("status") or "").lower() in ACTIVE_STATUSES

    def _open_frame_source(self, cv2):
        if str(self.camera).strip().lower() in REALSENSE_CAMERA_NAMES:
            return _RealSenseFrameSource()
        return _OpenCVFrameSource(cv2, self.camera)

    def _report_observation(self, gesture: Optional[str], score: float, accepted: bool):
        """Persist enough input evidence to diagnose a missed hand gesture.

        The recognizer can run at 30 Hz.  Reporting only when the category
        changes (or once per second) avoids competing with the massage worker
        for session-state writes.
        """
        if self.observation_callback is None:
            return
        key = (gesture, int(score * 100), bool(accepted))
        now = time.monotonic()
        if key == self._last_observation_key and now - self._last_observation_at < 1.0:
            return
        self._last_observation_key = key
        self._last_observation_at = now
        self.observation_callback(gesture, score, accepted)

    @staticmethod
    def _display_label(
        gesture: Optional[str], raw_gesture: Optional[str], score: float, used_landmark_fallback: bool
    ) -> str:
        """Build a diagnostic label without touching the camera frame source."""
        detected = gesture or raw_gesture or "No hand"
        suffix = " landmarks" if used_landmark_fallback else ""
        return f"{detected} {score:.2f}{suffix}"

    def run(self):
        try:
            import cv2
            import mediapipe as mp
        except Exception as exc:
            self.status_callback("unavailable", f"手势控制不可用：缺少 mediapipe 或 OpenCV（{exc}）")
            return

        if not self.model_path.is_file():
            self.status_callback("unavailable", f"手势控制不可用：找不到模型 {self.model_path}")
            return

        try:
            source = self._open_frame_source(cv2)
        except Exception as exc:
            self.status_callback("unavailable", f"手势控制不可用：无法打开摄像头 {self.camera}（{exc}）")
            return

        self.status_callback("running", f"手势控制已启动（摄像头 {source.label}）")
        base_options = mp.tasks.BaseOptions(model_asset_path=str(self.model_path))
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        timestamp_ms = 0
        window_name = "FAIRINO Gesture Control"
        terminal_status_reported = False
        try:
            with mp.tasks.vision.GestureRecognizer.create_from_options(options) as recognizer:
                while not self._stop_event.is_set() and self._keep_running():
                    ok, frame = source.read()
                    if not ok:
                        self.status_callback("warning", "手势摄像头读取失败，已停止手势控制")
                        terminal_status_reported = True
                        return
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    timestamp_ms = max(timestamp_ms + 1, int(time.monotonic() * 1000))
                    image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                    result = recognizer.recognize_for_video(image, timestamp_ms)
                    raw_gesture, score = self._top_gesture(result)
                    gesture = raw_gesture if raw_gesture in GESTURE_TO_COMMAND and score >= self.min_score else None
                    used_landmark_fallback = False
                    if gesture is None:
                        landmark_gesture = self._landmark_fallback(result)
                        if landmark_gesture is not None:
                            gesture = landmark_gesture
                            used_landmark_fallback = True
                    self._report_observation(
                        gesture or raw_gesture,
                        score if not used_landmark_fallback else max(score, self.min_score),
                        gesture is not None,
                    )
                    state = self.state_provider()
                    command = self.gate.observe(gesture, state)
                    if command and self.command_callback(command):
                        action_label = {"pause": "暂停", "resume": "继续", "stop": "停止"}[command]
                        self.status_callback("command", f"已识别手势 {gesture}，执行{action_label}请求")

                    if self.display:
                        # Keep ``source`` as the camera object.  Reassigning it
                        # here used to turn it into a string after a landmark
                        # fallback, so the preview crashed on the next frame.
                        label = self._display_label(gesture, raw_gesture, score, used_landmark_fallback)
                        cv2.putText(frame, label, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.putText(frame, "Fist=pause  ThumbUp=resume  Palm=stop", (16, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                        cv2.imshow(window_name, frame)
                        if (cv2.waitKey(1) & 0xFF) in {27, ord("q")}:
                            self.status_callback("stopped", "手势预览窗口已关闭，手势控制已停止")
                            terminal_status_reported = True
                            return
        except Exception as exc:
            self.status_callback("error", f"手势控制异常退出：{exc}")
            terminal_status_reported = True
        finally:
            source.close()
            if self.display:
                try:
                    cv2.destroyWindow(window_name)
                except Exception:
                    pass
            if not self._stop_event.is_set() and not terminal_status_reported:
                self.status_callback("stopped", "本次按摩结束，手势控制已停止")


def _diagnostic_main() -> int:
    """Run a camera-only recognizer preview; never sends a robot command."""
    parser = argparse.ArgumentParser(description="FAIRINO MediaPipe hand-gesture preview")
    parser.add_argument("--camera", default=None, help="realsense (default), OpenCV index, or video device path")
    args = parser.parse_args()

    def status(status_name: str, message: str):
        print(f"[{status_name}] {message}", flush=True)

    def command(command_name: str) -> bool:
        print(f"[recognized] {command_name}（仅预览，未发送机械臂指令）", flush=True)
        return True

    monitor = GestureControlMonitor(
        state_provider=lambda: {"status": "running", "worker_alive": True},
        command_callback=command,
        status_callback=status,
        camera=args.camera,
        display=True,
    )
    print("手势预览已启动：握拳=暂停，竖拇指=继续，张开手掌=停止；按 q 或 Esc 退出。", flush=True)
    monitor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(_diagnostic_main())
