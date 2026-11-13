import sqlite3
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path

from src.health_band import BodyMetricsStore, HealthBandReader


class HealthBandReaderTests(unittest.TestCase):
    def _database(self, received_at: str, heart_rate=72.0, spo2=98.0, battery=80.0):
        directory = tempfile.TemporaryDirectory()
        path = Path(directory.name) / "health_band.sqlite3"
        with sqlite3.connect(path) as connection:
            connection.executescript(
                """
                CREATE TABLE health_band_samples (
                    id INTEGER PRIMARY KEY,
                    received_at TEXT NOT NULL,
                    device_mac TEXT NOT NULL,
                    device_time_raw TEXT,
                    device_time_utc TEXT,
                    heart_rate_bpm REAL,
                    spo2_percent REAL,
                    temperature_c REAL,
                    battery_percent REAL
                );
                """
            )
            connection.execute(
                """INSERT INTO health_band_samples
                (received_at, device_mac, heart_rate_bpm, spo2_percent, temperature_c, battery_percent)
                VALUES (?, 'AABBCCDDEEFF', ?, ?, 36.6, ?)""",
                (received_at, heart_rate, spo2, battery),
            )
        return directory, path

    def test_latest_and_fresh_summary(self):
        directory, path = self._database(datetime.now(timezone.utc).isoformat())
        self.addCleanup(directory.cleanup)
        reader = HealthBandReader(path)
        self.assertEqual(reader.latest()["heart_rate_bpm"], 72.0)
        self.assertEqual(reader.summary()["status"], "normal")

    def test_stale_summary(self):
        old = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
        directory, path = self._database(old)
        self.addCleanup(directory.cleanup)
        self.assertEqual(HealthBandReader(path).summary(stale_after_seconds=120)["status"], "stale")

    def test_low_battery_and_mac_masking(self):
        directory, path = self._database(datetime.now(timezone.utc).isoformat(), battery=10)
        self.addCleanup(directory.cleanup)
        summary = HealthBandReader(path).summary()
        self.assertEqual(summary["status"], "alert")
        self.assertIn("手环电量低", summary["message"])
        self.assertEqual(HealthBandReader.public_sample(summary)["device_mac"], "AA******EEFF")

    def test_zero_vitals_keep_the_gui_in_waiting_state(self):
        directory, path = self._database(datetime.now(timezone.utc).isoformat(), heart_rate=0, spo2=0)
        self.addCleanup(directory.cleanup)
        summary = HealthBandReader(path).summary()
        self.assertEqual(summary["status"], "awaiting")
        self.assertEqual(summary["status_label"], "等待接收")
        self.assertNotIn("heart_rate_bpm", summary)
        self.assertEqual(summary["trends"], {})
        self.assertEqual(summary["trend_minutes"], 1)

    def test_first_complete_nonzero_sample_enables_live_view(self):
        directory, path = self._database(datetime.now(timezone.utc).isoformat(), heart_rate=0, spo2=98)
        self.addCleanup(directory.cleanup)
        with sqlite3.connect(path) as connection:
            connection.execute(
                """INSERT INTO health_band_samples
                (received_at, device_mac, heart_rate_bpm, spo2_percent, temperature_c, battery_percent)
                VALUES (?, 'AABBCCDDEEFF', 73, 99, 36.7, 80)""",
                [(datetime.now(timezone.utc) + timedelta(seconds=1)).isoformat()],
            )
        summary = HealthBandReader(path).summary()
        self.assertEqual(summary["status"], "normal")
        self.assertEqual(summary["heart_rate_bpm"], 73.0)

    def test_missing_vital_sign_keeps_the_gui_in_waiting_state(self):
        directory, path = self._database(datetime.now(timezone.utc).isoformat(), heart_rate=None, spo2=98)
        self.addCleanup(directory.cleanup)
        summary = HealthBandReader(path).summary()
        self.assertEqual(summary["status"], "awaiting")
        self.assertEqual(summary["status_label"], "等待接收")

    def test_repeated_invalid_samples_keep_the_gui_in_waiting_state(self):
        directory, path = self._database((datetime.now(timezone.utc) - timedelta(minutes=2)).isoformat())
        self.addCleanup(directory.cleanup)
        with sqlite3.connect(path) as connection:
            for seconds_ago in (60, 30, 0):
                connection.execute(
                    """INSERT INTO health_band_samples
                    (received_at, device_mac, heart_rate_bpm, spo2_percent, temperature_c, battery_percent)
                    VALUES (?, 'AABBCCDDEEFF', 0, 98, 36.6, 80)""",
                    [(datetime.now(timezone.utc) - timedelta(seconds=seconds_ago)).isoformat()],
                )
        summary = HealthBandReader(path).summary()
        self.assertEqual(summary["status"], "awaiting")
        self.assertEqual(summary["status_label"], "等待接收")

    def test_session_waits_for_a_new_sample_instead_of_showing_history(self):
        received_at = datetime.now(timezone.utc) - timedelta(minutes=1)
        directory, path = self._database(received_at.isoformat())
        self.addCleanup(directory.cleanup)
        summary = HealthBandReader(path).summary(since=datetime.now(timezone.utc))
        self.assertEqual(summary["status"], "awaiting")
        self.assertEqual(summary["heart_rate_bpm"] if "heart_rate_bpm" in summary else None, None)
        self.assertIn("等待本次会话的首条", summary["message"])

    def test_body_metrics_manual_entry_calculates_and_persists_bmi(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        store = BodyMetricsStore(Path(directory.name) / "body_metrics.json")
        summary = store.set_manual(170, 65)
        self.assertEqual(summary["height_cm"], 170.0)
        self.assertEqual(summary["weight_kg"], 65.0)
        self.assertAlmostEqual(summary["bmi"], 22.49, places=2)
        self.assertEqual(summary["source"], "手动录入")

    def test_body_metrics_device_entry_uses_same_future_integration_path(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        store = BodyMetricsStore(Path(directory.name) / "body_metrics.json")
        self.assertEqual(store.update_from_device(180, 81, "智能秤")["source"], "智能秤")
        self.assertAlmostEqual(store.summary()["bmi"], 25.0)


if __name__ == "__main__":
    unittest.main()
