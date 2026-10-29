import tempfile
import unittest
from pathlib import Path

from storage import HealthStore


class HealthStoreTests(unittest.TestCase):
    def test_record_latest_and_deduplication(self):
        with tempfile.TemporaryDirectory() as directory:
            store = HealthStore(Path(directory) / "health.sqlite3")
            sample = {
                "mac": "47:54:12:00:02:D6",
                "time": "6549F84E",
                "hr": 72,
                "spo2": 98,
                "temperature": 36.5,
                "battery": 80,
            }
            self.assertTrue(store.record_sample(sample, "b/vitals"))
            self.assertFalse(store.record_sample(sample, "b/vitals"))
            latest = store.latest("4754120002D6")
            self.assertEqual(latest["heart_rate_bpm"], 72.0)
            self.assertEqual(latest["device_mac"], "4754120002D6")


if __name__ == "__main__":
    unittest.main()
