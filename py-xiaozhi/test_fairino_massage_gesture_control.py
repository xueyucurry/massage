import asyncio
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from src.mcp.tools.fairino_massage.gesture_control import (
    GestureCommandGate,
    GestureControlMonitor,
)
from src.mcp.tools.fairino_massage.runtime import FairinoMassageRuntime


class GestureCommandGateTests(unittest.TestCase):
    @staticmethod
    def _gesture_result(folded: bool):
        landmarks = [SimpleNamespace(x=0.0, y=0.0, z=0.0) for _ in range(21)]
        for mcp, pip, tip in ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)):
            landmarks[mcp] = SimpleNamespace(x=0.0, y=1.0, z=0.0)
            landmarks[pip] = SimpleNamespace(x=0.0, y=0.6, z=0.0)
            landmarks[tip] = SimpleNamespace(
                x=0.1 if folded else 0.0,
                y=0.7 if folded else 0.1,
                z=0.0,
            )
        return SimpleNamespace(hand_landmarks=[landmarks])

    def test_default_camera_is_realsense(self):
        with mock.patch.dict("os.environ", {}, clear=True):
            monitor = GestureControlMonitor(lambda: {}, lambda _command: True, lambda _status, _message: None)
        self.assertEqual(monitor.camera, "realsense")

    def test_requires_stable_gesture_and_neutral_rearm(self):
        gate = GestureCommandGate(stable_frames=3, neutral_frames=2)
        running = {"status": "running", "worker_alive": True}

        self.assertIsNone(gate.observe("Closed_Fist", running))
        self.assertIsNone(gate.observe("Closed_Fist", running))
        self.assertEqual(gate.observe("Closed_Fist", running), "pause")
        self.assertIsNone(gate.observe("Closed_Fist", running))

        self.assertIsNone(gate.observe(None, running))
        self.assertIsNone(gate.observe(None, running))
        self.assertIsNone(gate.observe("Closed_Fist", running))
        self.assertIsNone(gate.observe("Closed_Fist", running))
        self.assertEqual(gate.observe("Closed_Fist", running), "pause")

    def test_thumb_up_never_restarts_stopped_task(self):
        gate = GestureCommandGate(stable_frames=1, neutral_frames=1)
        self.assertIsNone(gate.observe("Thumb_Up", {"status": "stopped", "worker_alive": False}))
        self.assertIsNone(gate.observe("Thumb_Up", {"status": "running", "worker_alive": True}))
        self.assertEqual(gate.observe("Thumb_Up", {"status": "paused", "worker_alive": False}), "resume")

    def test_landmarks_fall_back_only_for_open_palm_and_closed_fist(self):
        self.assertEqual(GestureControlMonitor._landmark_fallback(self._gesture_result(folded=False)), "Open_Palm")
        self.assertEqual(GestureControlMonitor._landmark_fallback(self._gesture_result(folded=True)), "Closed_Fist")

    def test_preview_label_reports_fallback_without_replacing_camera_source(self):
        self.assertEqual(
            GestureControlMonitor._display_label("Open_Palm", None, 0.6, True),
            "Open_Palm 0.60 landmarks",
        )


class GestureRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        root = Path(self.temp_dir.name)
        self.state_path = root / "state.json"
        self.runtime = FairinoMassageRuntime(self.state_path)
        self.runtime.control_path = root / "control.json"
        self.runtime._state.update(session_id="gesture-session", status="running", worker_pid=12345)
        self.runtime._save_state_unlocked()

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_closed_fist_uses_existing_safe_pause_control_file(self):
        with mock.patch.object(self.runtime, "_is_worker_alive_unlocked", return_value=True):
            self.assertTrue(self.runtime._handle_gesture_command("pause", "gesture-session"))

        control = json.loads(self.runtime.control_path.read_text(encoding="utf-8"))
        self.assertEqual(control["request"], "pause")
        self.assertEqual(control["source"], "gesture:Closed_Fist")
        self.assertEqual(self.runtime._load_state()["status"], "pausing")

    def test_thumb_up_resumes_only_saved_paused_session(self):
        self.runtime._state.update(status="paused")
        self.runtime._save_state_unlocked()
        with mock.patch.object(self.runtime, "_is_worker_alive_unlocked", return_value=False), mock.patch.object(
            self.runtime, "_start_worker", return_value={"success": True}
        ) as start_worker:
            self.assertTrue(self.runtime._handle_gesture_command("resume", "gesture-session"))

        self.assertEqual(start_worker.call_args.kwargs["resume"], True)

    def test_open_palm_delegates_to_normal_stop_flow(self):
        with mock.patch.object(self.runtime, "stop", new=mock.AsyncMock(return_value={"success": True})) as stop:
            self.assertTrue(self.runtime._handle_gesture_command("stop", "gesture-session"))

        stop.assert_awaited_once_with(control_source="gesture:Open_Palm", gesture_command="stop")


if __name__ == "__main__":
    unittest.main()
