"""Checks that XiaoZhi and the GUI share the same fail-closed data selector."""

import json
import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import AsyncMock, Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "health"))

import data_source
import demo_alerts
from gui import model as gui_model
from PyQt5.QtCore import QCoreApplication
from src.health_band.reader import HealthBandReader
from src.display.gui_display import GuiDisplay


class HealthDemoIntegrationTests(unittest.TestCase):
    def test_gui_health_event_reaches_xiaozhi_voice_request(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        gui = GuiDisplay()
        voice = Mock(running=True, announce_health_alert=AsyncMock(), spawn=Mock())
        try:
            with patch("src.application.Application.get_instance", return_value=voice):
                gui._on_health_alert("检测到血氧读数持续很低，请复测。", 3)
            voice.announce_health_alert.assert_called_once()
            voice.spawn.assert_called_once()
            voice.spawn.call_args.args[0].close()
        finally:
            gui.health_panel_model._timer.stop()

    def test_idle_gui_raises_real_heart_warning_from_one_database(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            demo, control = root / "demo.sqlite3", root / "control.json"
            demo_alerts.write_all_cases(demo, datetime.now(timezone.utc) - timedelta(seconds=32))
            app = QCoreApplication.instance() or QCoreApplication([])
            self.assertIsNotNone(app)
            with patch.object(data_source, "DEMO_DATABASE", demo), patch.object(data_source, "CONTROL_FILE", control), \
                 patch.object(gui_model, "DATABASE_PATH", root / "real.sqlite3"), patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), \
                 patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"), patch.object(gui_model.HealthGuiModel, "_receiver_status", return_value={"running": False, "text": "测试"}):
                gui = gui_model.HealthGuiModel()
                announcements = []
                gui.alertRaised.connect(lambda speech, priority: announcements.append((speech, priority)))
                try:
                    control.write_text(json.dumps({"mode": "demo", "run_id": "all", "scenario": "all"}))
                    gui.refresh()
                    self.assertEqual(gui._state["physiological_alert"]["key"], "hr_rise")
                    self.assertIn("心率", gui.alertMessage)
                    self.assertEqual(len(announcements), 1)
                    self.assertNotIn("按摩力度", announcements[0][0])
                finally:
                    gui._timer.stop()

    def test_future_test_rows_stay_hidden_until_their_time(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            demo, control = root / "demo.sqlite3", root / "control.json"
            rows = demo_alerts.write_all_cases(demo, datetime.now(timezone.utc))
            control.write_text(json.dumps({"mode": "demo", "run_id": "all", "scenario": "all"}))
            with patch.object(data_source, "DEMO_DATABASE", demo), patch.object(data_source, "CONTROL_FILE", control):
                reader = HealthBandReader()
                self.assertEqual(reader.latest()["spo2_percent"], 98)
                self.assertLess(len(reader.history()), len(rows))
                self.assertEqual(len(reader.status()), 1)
                app = QCoreApplication.instance() or QCoreApplication([])
                self.assertIsNotNone(app)
                with patch.object(gui_model.HealthGuiModel, "_receiver_status", return_value={"running": False, "text": "测试"}), patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"):
                    gui = gui_model.HealthGuiModel()
                    try:
                        self.assertTrue(gui.simulatedSource)
                        self.assertLess(len(gui._samples()), len(rows))
                        self.assertEqual(gui.spo2Text, "98.0 %")
                    finally:
                        gui._timer.stop()
            self.assertEqual(len(rows), 48)

    def test_switch_uses_fixed_separate_database_and_stop_restores_real(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            real, demo = root / "real.sqlite3", root / "demo.sqlite3"
            control = root / "control.json"
            real.write_bytes(b"real database sentinel")
            with patch.object(data_source, "REAL_DATABASE", real), patch.object(data_source, "DEMO_DATABASE", demo), patch.object(data_source, "CONTROL_FILE", control), patch.object(demo_alerts, "LIVE_DATABASE", real):
                self.assertEqual(data_source.resolve_active_source().path, real)
                demo_alerts.write_scenario(demo, "spo2_urgent")
                control.write_text(json.dumps({"mode": "demo", "run_id": "test", "scenario": "spo2_urgent", "path": str(real)}))
                self.assertEqual(data_source.resolve_active_source().path, demo)
                self.assertEqual(HealthBandReader().database_path, demo.resolve())
                self.assertEqual(HealthBandReader().latest()["spo2_percent"], 88)
                app = QCoreApplication.instance() or QCoreApplication([])
                self.assertIsNotNone(app)
                with patch.object(gui_model.HealthGuiModel, "_receiver_status", return_value={"running": False, "text": "测试"}), patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"):
                    gui = gui_model.HealthGuiModel()
                    try:
                        self.assertTrue(gui.simulatedSource)
                        self.assertEqual(gui.spo2Text, "88.0 %")
                        gui.set_massage_session("test-session", True)
                        self.assertEqual(gui._state["physiological_alert"]["key"], "spo2_urgent")
                        control.write_text(json.dumps({"mode": "real"}))
                        gui.refresh()
                        self.assertFalse(gui.simulatedSource)
                        self.assertIsNone(gui._state["physiological_alert"])
                    finally:
                        gui._timer.stop()
                self.assertEqual(HealthBandReader(database_path=real).database_path, real.resolve())
                self.assertEqual(real.read_bytes(), b"real database sentinel")
                self.assertEqual(data_source.resolve_active_source().path, real)
                self.assertEqual(HealthBandReader().database_path, real.resolve())


if __name__ == "__main__":
    unittest.main()
