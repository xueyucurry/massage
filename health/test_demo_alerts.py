import sqlite3
import json
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import patch

import demo_alerts
from gui import model as gui_model
from PyQt5.QtCore import QCoreApplication
from vital_alerts import VitalAlertEngine
from storage import HealthStore
from data_source import ActiveSource


class DemoAlertTests(unittest.TestCase):
    def setUp(self):
        self.source_patch = patch.object(gui_model, "resolve_active_source", return_value=ActiveSource("real", Path("unused"), "test-real"))
        self.source_patch.start()

    def tearDown(self):
        self.source_patch.stop()

    def test_one_database_contains_all_cases_in_time_order(self):
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "demo.sqlite3"
            start = datetime.now(timezone.utc)
            rows = demo_alerts.write_all_cases(database, start)
            cases = {row["scenario"] for row in rows}
            self.assertEqual(len(rows), 48)
            self.assertEqual((datetime.fromisoformat(rows[-1]["received_at"]) - start).total_seconds(), 210)
            self.assertEqual(cases - {"normal"}, {"hr_rise", "hr_fall", "spo2_watch", "spo2_urgent", "temp_urgent", "temp_high"})
            self.assertGreater(datetime.fromisoformat(rows[-1]["received_at"]), start)
            with sqlite3.connect(database) as connection:
                self.assertEqual(connection.execute("SELECT COUNT(*) FROM health_band_samples").fetchone()[0], len(rows))
                self.assertTrue(connection.execute("SELECT value FROM demo_metadata WHERE key='marker'").fetchone()[0])
            engine, due, observed, announcements, alert_times = VitalAlertEngine(cooldown_seconds=0), [], set(), [], []
            for row in rows:
                due.append(row)
                moment = datetime.fromisoformat(row["received_at"])
                result = engine.evaluate(due, session_key="combined-test", now=moment)
                if result["announcement"]:
                    announcements.append(result["announcement"]["key"])
                    alert_times.append((moment - start).total_seconds())
                if result["alert"] and row["scenario"] == result["alert"]["key"]:
                    observed.add(row["scenario"])
            self.assertEqual(observed, {"spo2_watch", "spo2_urgent", "temp_high", "temp_urgent", "hr_rise", "hr_fall"})
            self.assertEqual(announcements, ["hr_rise", "hr_fall", "spo2_watch", "spo2_urgent", "temp_urgent", "temp_high"])
            self.assertEqual(alert_times, [30, 80, 125, 150, 175, 200])

    def test_test_database_matches_real_sample_schema_and_has_full_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            real, demo = root / "real.sqlite3", root / "demo.sqlite3"
            HealthStore(real)
            rows = demo_alerts.write_scenario(demo, "spo2_urgent")
            self.assertEqual(len(rows), 60)
            with sqlite3.connect(real) as connection:
                real_columns = connection.execute("PRAGMA table_info(health_band_samples)").fetchall()
            with sqlite3.connect(demo) as connection:
                demo_columns = connection.execute("PRAGMA table_info(health_band_samples)").fetchall()
                latest = connection.execute(
                    "SELECT device_mac,device_time_epoch,device_time_utc,raw_json FROM health_band_samples ORDER BY id DESC LIMIT 1"
                ).fetchone()
            self.assertEqual(real_columns, demo_columns)
            self.assertEqual(latest[0], demo_alerts.DEMO_MAC)
            self.assertIsInstance(latest[1], int)
            self.assertTrue(latest[2])
            self.assertEqual(json.loads(latest[3])["spo2"], 88)
            with sqlite3.connect(real) as connection:
                self.assertEqual(connection.execute("SELECT COUNT(*) FROM health_band_samples").fetchone()[0], 0)

    def test_all_scenarios_match_the_real_alert_engine(self):
        now = datetime.now(timezone.utc)
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "demo.sqlite3"
            for key, scenario in demo_alerts.SCENARIOS.items():
                written = demo_alerts.write_scenario(database, key, now)
                with sqlite3.connect(database) as connection:
                    count = connection.execute("SELECT COUNT(*) FROM health_band_samples").fetchone()[0]
                self.assertEqual(count, len(written), key)
                result = VitalAlertEngine().evaluate(written, session_key=key, now=now)
                actual = (result["alert"] or {}).get("key")
                self.assertEqual(actual, scenario.alert_key, key)

    def test_never_overwrites_unmarked_or_live_database(self):
        with tempfile.TemporaryDirectory() as directory:
            other = Path(directory) / "other.sqlite3"
            with sqlite3.connect(other) as connection:
                connection.execute("CREATE TABLE private_data (value TEXT)")
                connection.execute("INSERT INTO private_data VALUES ('keep')")
            with self.assertRaises(ValueError):
                demo_alerts.write_scenario(other, "normal")
            with sqlite3.connect(other) as connection:
                self.assertEqual(connection.execute("SELECT value FROM private_data").fetchone()[0], "keep")
            with patch.object(demo_alerts, "LIVE_DATABASE", other):
                with self.assertRaises(ValueError):
                    demo_alerts.write_scenario(other, "normal")

    def test_scenario_timestamps_can_be_refreshed_without_accumulation(self):
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "demo.sqlite3"
            now = datetime.now(timezone.utc)
            first = demo_alerts.write_scenario(database, "hr_rise", now)
            second = demo_alerts.write_scenario(database, "hr_rise", now + timedelta(seconds=5))
            self.assertEqual(len(first), len(second))
            self.assertNotEqual(first[-1]["received_at"], second[-1]["received_at"])
            with sqlite3.connect(database) as connection:
                self.assertEqual(connection.execute("SELECT COUNT(*) FROM health_band_samples").fetchone()[0], len(second))

    def test_all_scenarios_match_the_real_gui_state(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / "demo.sqlite3"
            with patch.object(gui_model, "DATABASE_PATH", database), patch.object(gui_model, "BODY_METRICS_PATH", root / "body.json"), patch.object(gui_model, "SETTINGS_PATH", root / "settings.json"), patch.object(gui_model.HealthGuiModel, "_receiver_status", return_value={"running": True, "text": "演示数据"}):
                model = gui_model.HealthGuiModel()
                model._session_started = datetime.now(timezone.utc) - timedelta(minutes=2)
                try:
                    for index, (key, scenario) in enumerate(demo_alerts.SCENARIOS.items()):
                        demo_alerts.write_scenario(database, key)
                        model._has_valid_measurement = False
                        model.set_massage_session(f"demo-test-{index}", True)
                        model.refresh()
                        self.assertEqual(model.statusLabel, scenario.quality, key)
                        actual = (model._state.get("physiological_alert") or {}).get("key")
                        self.assertEqual(actual, scenario.alert_key, key)
                finally:
                    model._timer.stop()


if __name__ == "__main__":
    unittest.main()
