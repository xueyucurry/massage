import tempfile
import unittest
import sqlite3
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import patch

from PyQt5.QtCore import QCoreApplication

from gui import model as gui_model
from gui.model import BodyMetricsStore, HealthGuiModel
from massage_profile import MassageRecommendationStore, recommendation_for_bmi
from data_source import ActiveSource


class BodyMetricsStoreTests(unittest.TestCase):
    def test_manual_entry_calculates_bmi(self):
        with tempfile.TemporaryDirectory() as directory:
            store = BodyMetricsStore(Path(directory) / "body_metrics.json")
            summary = store.set_manual(170, 65)
            self.assertAlmostEqual(summary["bmi"], 22.49, places=2)
            self.assertEqual(summary["source"], "手动录入")

    def test_future_device_adapter_uses_same_store(self):
        with tempfile.TemporaryDirectory() as directory:
            store = BodyMetricsStore(Path(directory) / "body_metrics.json")
            summary = store.update_from_device(180, 81, "智能秤")
            self.assertEqual(summary["source"], "智能秤")
            self.assertEqual(summary["bmi"], 25.0)

    def test_bmi_massage_profile_uses_confirmed_single_use_recommendation(self):
        self.assertEqual(recommendation_for_bmi(18.4)["factor"], 0.90)
        self.assertEqual(recommendation_for_bmi(23.9)["factor"], 1.00)
        self.assertEqual(recommendation_for_bmi(24.0)["factor"], 1.05)
        self.assertEqual(recommendation_for_bmi(28.0)["factor"], 1.10)
        with tempfile.TemporaryDirectory() as directory:
            store = MassageRecommendationStore(Path(directory) / "recommendation.json")
            confirmed = store.confirm(28.1, "手动录入")
            self.assertEqual(confirmed["status"], "confirmed")
            applied = store.consume(10)
            self.assertEqual(applied["requested_force_n"], 11.0)
            self.assertFalse(store.pending()["available"])


class HealthGuiModelTests(unittest.TestCase):
    def setUp(self):
        self.source_patch = patch.object(gui_model, "resolve_active_source", return_value=ActiveSource("real", Path("unused"), "test-real"))
        self.source_patch.start()

    def tearDown(self):
        self.source_patch.stop()
    def test_critical_readings_warn_even_without_a_massage_session(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / "health_band.sqlite3"
            self._database(database)
            now = datetime.now(timezone.utc)
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"), patch.object(HealthGuiModel, "_receiver_status", return_value={"running": False, "text": "测试"}):
                model = HealthGuiModel()
                model._session_started = now - timedelta(minutes=1)
                model._alert_started_at = model._session_started
                announcements = []
                model.alertRaised.connect(lambda speech, priority: announcements.append((speech, priority)))
                with sqlite3.connect(database) as connection:
                    for seconds in (10, 5, 0):
                        connection.execute("INSERT INTO health_band_samples VALUES (?, 82, 88, 36.5, 80)",
                                           [(now - timedelta(seconds=seconds)).isoformat()])
                model.refresh()
                self.assertEqual(model._state["physiological_alert"]["key"], "spo2_urgent")
                self.assertEqual(model.badgeLabel, "紧急预警")
                self.assertEqual(model.badgeColor, "#B42323")
                self.assertIn("立即复测", model.alertMessage)
                self.assertNotIn("停止按摩", model.alertMessage)
                self.assertEqual(len(announcements), 1)
                self.assertNotIn("按摩", announcements[0][0])
                model._timer.stop()

    def test_idle_heart_trend_does_not_offer_robot_force_control(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / "health_band.sqlite3"
            self._database(database)
            now = datetime.now(timezone.utc)
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"), patch.object(HealthGuiModel, "_receiver_status", return_value={"running": False, "text": "测试"}):
                model = HealthGuiModel()
                model._session_started = now - timedelta(minutes=1)
                model._alert_started_at = model._session_started
                announcements = []
                model.alertRaised.connect(lambda speech, priority: announcements.append((speech, priority)))
                with sqlite3.connect(database) as connection:
                    for index in range(12):
                        connection.execute("INSERT INTO health_band_samples VALUES (?, ?, 98, 36.5, 80)",
                                           [(now - timedelta(seconds=(11 - index) * 5)).isoformat(), 82 if index < 6 else 105])
                model.refresh()
                self.assertEqual(model._state["physiological_alert"]["key"], "hr_rise")
                self.assertEqual(model.badgeLabel, "体征提醒")
                self.assertEqual(model.badgeColor, "#B76E00")
                self.assertEqual(len(announcements), 1)
                self.assertNotIn("按摩力度", announcements[0][0])
                model._timer.stop()

    @staticmethod
    def _database(path: Path) -> None:
        with sqlite3.connect(path) as connection:
            connection.execute(
                """CREATE TABLE health_band_samples (
                    received_at TEXT, heart_rate_bpm REAL, spo2_percent REAL,
                    temperature_c REAL, battery_percent REAL)"""
            )

    def test_saving_body_metrics_schedules_bmi_force_without_confirmation(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recommendation_store = MassageRecommendationStore(root / "recommendation.json")
            with patch.object(gui_model, "DATABASE_PATH", root / "missing.sqlite3"), \
                 patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), \
                 patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"), \
                 patch.object(gui_model, "MassageRecommendationStore", return_value=recommendation_store):
                model = HealthGuiModel()
                model.saveManualMetrics("180", "81")
                self.assertEqual(recommendation_store.pending()["factor"], 1.05)
                self.assertIn("超重体型", model.massageRecommendationText)
                self.assertIn("1.05 倍", model.massageRecommendationText)
                self.assertEqual(recommendation_store.consume(10)["requested_force_n"], 10.5)
                model.refresh()
                self.assertIn("下次按摩请重新录入资料", model.massageRecommendationText)
                model.updateBodyMetricsFromDevice(170, 65, "智能秤")
                self.assertEqual(recommendation_store.pending()["factor"], 1.0)
                self.assertIn("正常体型", model.massageRecommendationText)
                model._timer.stop()

    def test_active_massage_alert_is_separate_from_receiver_status(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / "health_band.sqlite3"
            self._database(database)
            now = datetime.now(timezone.utc)
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"), patch.object(HealthGuiModel, "_receiver_status", return_value={"running": True, "text": "接收器运行中"}):
                model = HealthGuiModel()
                model._session_started = now - timedelta(minutes=1)
                with sqlite3.connect(database) as connection:
                    for seconds in (10, 5, 0):
                        connection.execute(
                            "INSERT INTO health_band_samples VALUES (?, 75, 94, 36.5, 80)",
                            [(now - timedelta(seconds=seconds)).isoformat()],
                        )
                announcements = []
                model.alertRaised.connect(lambda speech, priority: announcements.append((speech, priority)))
                model.set_massage_session("session-1", True)
                self.assertEqual(model.statusLabel, "数据正常")
                self.assertIn("血氧持续低于95%", model.alertMessage)
                self.assertEqual(len(announcements), 1)
                model.refresh()
                self.assertEqual(len(announcements), 1)
                model._timer.stop()

    def test_invalid_later_sample_stays_visible_without_clearing_trend(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "health_band.sqlite3"
            self._database(database)
            now = datetime.now(timezone.utc)
            with sqlite3.connect(database) as connection:
                for offset, heart_rate, spo2 in ((1, 75, 98), (35, 76, 98), (50, 0, None)):
                    connection.execute(
                        "INSERT INTO health_band_samples VALUES (?, ?, ?, 36.5, 80)",
                        [(now + timedelta(seconds=offset)).isoformat(), heart_rate, spo2],
                    )
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", Path(directory) / "body.json"), patch.object(gui_model, "SETTINGS_PATH", Path(directory) / "settings.json"):
                model = HealthGuiModel()
                self.assertEqual(model.statusLabel, "数据待确认")
                self.assertEqual(model.heartRateText, "--")
                self.assertEqual(model.spo2Text, "--")
                self.assertEqual(len(model.heartRateTrend), 2)
                model._timer.stop()

    def test_first_invalid_sample_does_not_show_vitals_or_trends(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "health_band.sqlite3"
            self._database(database)
            with sqlite3.connect(database) as connection:
                connection.execute(
                    "INSERT INTO health_band_samples VALUES (?, 0, 0, 0, 80)",
                    [datetime.now(timezone.utc).isoformat()],
                )
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", Path(directory) / "body.json"), patch.object(gui_model, "SETTINGS_PATH", Path(directory) / "settings.json"):
                model = HealthGuiModel()
                self.assertEqual(model.statusLabel, "等待接收")
                self.assertEqual(model.heartRateText, "--")
                self.assertEqual(model.heartRateTrend, [])
                model._timer.stop()

    def test_new_gui_session_hides_saved_body_profile_until_entered_again(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "health_band.sqlite3"
            self._database(database)
            body_path = Path(directory) / "body.json"
            BodyMetricsStore(body_path).set_manual(173, 80)
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", body_path), patch.object(gui_model, "SETTINGS_PATH", Path(directory) / "settings.json"):
                model = HealthGuiModel()
                self.assertEqual(model.heightText, "--")
                self.assertEqual(model.weightText, "--")
                self.assertEqual(model.bmiText, "--")
                model.saveManualMetrics("173", "80")
                self.assertEqual(model.heightText, "173 cm")
                self.assertEqual(model.weightText, "80.0 kg")
                model._timer.stop()

    def test_fresh_valid_sample_with_low_battery_is_flagged(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "health_band.sqlite3"
            self._database(database)
            with sqlite3.connect(database) as connection:
                connection.execute(
                    "INSERT INTO health_band_samples VALUES (?, 75, 98, 36.5, 20)",
                    [(datetime.now(timezone.utc) + timedelta(seconds=1)).isoformat()],
                )
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", Path(directory) / "body.json"), patch.object(gui_model, "SETTINGS_PATH", Path(directory) / "settings.json"):
                model = HealthGuiModel()
                self.assertEqual(model.statusLabel, "电量偏低")
                self.assertEqual(model.badgeLabel, "电量偏低")
                self.assertIn("20%", model.message)
                model._timer.stop()

    def test_trend_displays_only_latest_twelve_buckets(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / "health_band.sqlite3"
            self._database(database)
            now = datetime.now(timezone.utc)
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"), patch.object(HealthGuiModel, "_receiver_status", return_value={"running": True, "text": "测试"}):
                model = HealthGuiModel()
                model._session_started = now - timedelta(minutes=2)
                model._bucket_seconds = 5
                with sqlite3.connect(database) as connection:
                    for index in range(20):
                        connection.execute(
                            "INSERT INTO health_band_samples VALUES (?, ?, 98, 36.5, 80)",
                            [(now - timedelta(seconds=(19 - index) * 5)).isoformat(), 70 + index],
                        )
                model.refresh()
                self.assertEqual(model.heartRateTrend, list(range(78, 90)))
                self.assertEqual(len(model.spo2Trend), 12)
                self.assertEqual(model.badgeLabel, "数据正常")
                model._timer.stop()


if __name__ == "__main__":
    unittest.main()
