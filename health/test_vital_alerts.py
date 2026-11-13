import unittest
from datetime import datetime, timedelta, timezone

from vital_alerts import VitalAlertEngine


START = datetime(2026, 1, 1, tzinfo=timezone.utc)


def rows(hearts, oxygen=98, temp=36.5):
    return [
        {"received_at": (START + timedelta(seconds=i * 5)).isoformat(),
         "heart_rate_bpm": heart, "spo2_percent": oxygen, "temperature_c": temp}
        for i, heart in enumerate(hearts)
    ]


class VitalAlertTests(unittest.TestCase):
    def test_spo2_escalates_once_and_invalid_interrupts_streak(self):
        engine = VitalAlertEngine()
        samples = rows([75] * 6, 94)
        first = engine.evaluate(samples, session_key="one", now=START + timedelta(seconds=25))
        self.assertEqual(first["announcement"]["key"], "spo2_watch")
        self.assertIsNone(engine.evaluate(samples, session_key="one", now=START + timedelta(seconds=26))["announcement"])
        samples += rows([75] * 3, 88)[-3:]
        for i, sample in enumerate(samples[-3:]):
            sample["received_at"] = (START + timedelta(seconds=30 + i * 5)).isoformat()
        self.assertEqual(engine.evaluate(samples, session_key="one", now=START + timedelta(seconds=40))["alert"]["key"], "spo2_urgent")
        samples[-2]["spo2_percent"] = 0
        self.assertIsNone(engine.evaluate(samples, session_key="two", now=START + timedelta(seconds=40))["alert"])

    def test_urgent_oxygen_requires_three_consecutive_fresh_readings(self):
        engine = VitalAlertEngine()
        samples = rows([75, 75], 88)
        self.assertIsNone(engine.evaluate(samples, session_key="a", now=START + timedelta(seconds=5))["alert"])
        samples = rows([75] * 3, 88)
        self.assertEqual(engine.evaluate(samples, session_key="a", now=START + timedelta(seconds=10))["announcement"]["key"], "spo2_urgent")
        samples[-1]["spo2_percent"] = 0
        self.assertIsNone(engine.evaluate(samples, session_key="b", now=START + timedelta(seconds=10))["alert"])

    def test_inclusive_boundaries_and_invalid_reading_restart_streak(self):
        self.assertEqual(VitalAlertEngine().evaluate(rows([75] * 3, 88), session_key="oxygen", now=START + timedelta(seconds=10))["alert"]["key"], "spo2_urgent")
        self.assertEqual(VitalAlertEngine().evaluate(rows([75] * 3, 98, 35), session_key="cold", now=START + timedelta(seconds=10))["alert"]["key"], "temp_urgent")
        self.assertEqual(VitalAlertEngine().evaluate(rows([75] * 3, 98, 38), session_key="hot", now=START + timedelta(seconds=10))["alert"]["key"], "temp_high")
        interrupted = rows([75] * 4, 94)
        interrupted[2]["spo2_percent"] = 0
        self.assertIsNone(VitalAlertEngine().evaluate(interrupted[:3], session_key="broken", now=START + timedelta(seconds=10))["alert"])
        self.assertIsNone(VitalAlertEngine().evaluate(interrupted, session_key="broken", now=START + timedelta(seconds=15))["alert"])

    def test_wrist_temperature_bounds_require_recheck_not_diagnosis(self):
        engine = VitalAlertEngine()
        self.assertIsNone(engine.evaluate(rows([75] * 3, 98, 35.8), session_key="low", now=START + timedelta(seconds=10))["alert"])
        self.assertEqual(engine.evaluate(rows([75] * 3, 98, 38.1), session_key="high", now=START + timedelta(seconds=10))["alert"]["key"], "temp_high")
        urgent = engine.evaluate(rows([75] * 3, 98, 34.8), session_key="urgent", now=START + timedelta(seconds=10))["alert"]
        self.assertEqual(urgent["key"], "temp_urgent")
        self.assertIn("体温计复测", urgent["message"])

    def test_hr_rise_and_fall_need_stable_baseline(self):
        engine = VitalAlertEngine()
        rising = rows([80] * 6 + [105] * 6)
        self.assertEqual(engine.evaluate(rising, session_key="a", now=START + timedelta(seconds=55))["alert"]["key"], "hr_rise")
        falling = rows([80] * 6 + [58] * 6)
        self.assertEqual(engine.evaluate(falling, session_key="b", now=START + timedelta(seconds=55))["alert"]["key"], "hr_fall")
        normal = rows([80] * 6 + [72] * 6)
        self.assertIsNone(engine.evaluate(normal, session_key="c", now=START + timedelta(seconds=55))["alert"])

    def test_hr_baseline_waits_for_a_later_stable_window(self):
        engine = VitalAlertEngine()
        unstable = rows([70, 90, 70, 90, 70, 90])
        self.assertIsNone(engine.evaluate(unstable, session_key="later", now=START + timedelta(seconds=25))["alert"])
        self.assertIsNone(engine.heart_baseline)
        stable = rows([82] * 6)
        for index, sample in enumerate(stable):
            sample["received_at"] = (START + timedelta(seconds=30 + index * 5)).isoformat()
        engine.evaluate(unstable + stable, session_key="later", now=START + timedelta(seconds=55))
        self.assertEqual(engine.heart_baseline, 82)
        elevated = rows([105] * 6)
        for index, sample in enumerate(elevated):
            sample["received_at"] = (START + timedelta(seconds=60 + index * 5)).isoformat()
        self.assertEqual(engine.evaluate(unstable + stable + elevated, session_key="later", now=START + timedelta(seconds=85))["alert"]["key"], "hr_rise")

    def test_stale_and_bad_values_never_trigger(self):
        engine = VitalAlertEngine()
        samples = rows([75] * 6, 88, 34.5)
        self.assertIsNone(engine.evaluate(samples, session_key="x", now=START + timedelta(minutes=2))["alert"])
        samples[-1]["temperature_c"] = 0
        samples[-1]["spo2_percent"] = 0
        self.assertIsNone(engine.evaluate(samples, session_key="y", now=START + timedelta(seconds=25))["alert"])

    def test_new_low_priority_alert_waits_for_cooldown_then_speaks(self):
        engine = VitalAlertEngine(cooldown_seconds=60)
        samples = rows([75] * 3, 94)
        self.assertIsNotNone(engine.evaluate(samples, session_key="a", now=START + timedelta(seconds=10))["announcement"])
        for i in range(3, 6):
            sample = rows([75] * 1, 98, 38.1)[0]
            sample["received_at"] = (START + timedelta(seconds=i * 5)).isoformat()
            samples.append(sample)
        self.assertIsNone(engine.evaluate(samples, session_key="a", now=START + timedelta(seconds=25))["announcement"])
        # Keep the new condition active until the cooling window has elapsed.
        for i in range(6, 16):
            sample = rows([75] * 1, 98, 38.1)[0]
            sample["received_at"] = (START + timedelta(seconds=i * 5)).isoformat()
            samples.append(sample)
        self.assertEqual(engine.evaluate(samples, session_key="a", now=START + timedelta(seconds=75))["announcement"]["key"], "temp_high")


if __name__ == "__main__":
    unittest.main()
