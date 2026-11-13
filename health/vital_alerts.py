"""Session-scoped wristband screening alerts for the massage GUI.

These are prompts to recheck a consumer wristband, never diagnoses.  The
engine consumes received timestamps (rather than device clocks), ignores bad
readings, and emits only changes to the highest-priority active alert.
"""

from __future__ import annotations

import math
from datetime import datetime, timedelta, timezone
from statistics import median
from typing import Any


def _time(value: Any) -> datetime | None:
    try:
        result = datetime.fromisoformat(str(value))
        return result.replace(tzinfo=timezone.utc) if result.tzinfo is None else result.astimezone(timezone.utc)
    except (TypeError, ValueError):
        return None


def valid_vital_value(sample: dict, key: str) -> float | None:
    try:
        value = float(sample.get(key))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    if key == "spo2_percent":
        return value if 0 < value <= 100 else None
    if key == "temperature_c":
        return value if 25 <= value <= 45 else None
    return value if 20 <= value <= 250 else None


class VitalAlertEngine:
    """Evaluate each fresh upload once; latch alerts until readings recover."""

    def __init__(self, *, stale_seconds: int = 20, cooldown_seconds: int = 180):
        self.stale_seconds = stale_seconds
        self.cooldown_seconds = cooldown_seconds
        self.session_key: str | None = None
        self.last_sample_at: datetime | None = None
        self.last_alert_key: str | None = None
        self.last_announcement_at: datetime | None = None
        self.heart_baseline: float | None = None

    def reset(self, session_key: str) -> None:
        self.session_key = session_key
        self.last_sample_at = None
        self.last_alert_key = None
        self.last_announcement_at = None
        self.heart_baseline = None

    def evaluate(self, samples: list[dict], *, session_key: str, now: datetime | None = None) -> dict:
        if self.session_key != session_key:
            self.reset(session_key)
        now = now or datetime.now(timezone.utc)
        ordered = sorted(((stamp, sample) for sample in samples if (stamp := _time(sample.get("received_at")))), key=lambda pair: pair[0])
        if not ordered:
            self.last_alert_key = None
            return {"alert": None, "announcement": None}
        latest_at, latest = ordered[-1]
        if (now - latest_at).total_seconds() > self.stale_seconds or latest_at > now + timedelta(seconds=5):
            self.last_alert_key = None
            return {"alert": None, "announcement": None}

        candidates: list[dict] = []
        # Consecutive means consecutive uploads; an invalid value interrupts a
        # streak instead of allowing a later reading to complete it.
        recent = ordered[-3:]
        continuous = len(recent) == 3 and (recent[-1][0] - recent[0][0]).total_seconds() <= 30
        oxygen = [valid_vital_value(row, "spo2_percent") for _, row in recent]
        temp = [valid_vital_value(row, "temperature_c") for _, row in recent]
        if continuous and all(value is not None and value <= 88 for value in oxygen):
            candidates.append(self._alert("spo2_urgent", 3, "血氧持续低于或等于88%，请立即复测并尽快就医。", "血氧持续低于88%，请立即复测并尽快就医。"))
        elif continuous and all(value is not None and value <= 94 for value in oxygen):
            candidates.append(self._alert("spo2_watch", 1, "血氧持续低于95%，请检查佩戴并复测。", "血氧持续低于95%，请检查手环佩戴，并确认是否不适。"))
        if continuous and all(value is not None and value <= 35 for value in temp):
            candidates.append(self._alert("temp_urgent", 3, "手环体温读数持续低于或等于35℃，请立即用体温计复测；如复测仍低或有不适，请及时就医。", "手环体温读数持续低于35℃，请立即用体温计复测；如仍偏低或有不适，请及时就医。"))
        elif continuous and all(value is not None and value >= 38 for value in temp):
            candidates.append(self._alert("temp_high", 1, "手环体温读数持续达到或超过38℃，请用体温计复测；如确认发热或有不适，请咨询医生。", "手环体温读数持续高于38℃，请立即用体温计复测；如确认发热或有不适，请及时就医。"))

        # Keep searching until six consecutive valid readings are stable, then
        # lock the reference for this monitoring session.
        if self.heart_baseline is None:
            for index in range(5, len(ordered)):
                window = ordered[index - 5:index + 1]
                values = [valid_vital_value(row, "heart_rate_bpm") for _, row in window]
                if (all(value is not None for value in values)
                        and (window[-1][0] - window[0][0]).total_seconds() <= 35
                        and max(values) - min(values) <= 15):
                    self.heart_baseline = median(values)
                    break
        if self.heart_baseline is not None and len(ordered) >= 12:
            baseline = self.heart_baseline
            heart_recent = ordered[-6:]
            earlier = [(stamp, row) for stamp, row in ordered if latest_at - timedelta(seconds=65) <= stamp <= latest_at - timedelta(seconds=30)][-6:]
            hearts = [valid_vital_value(row, "heart_rate_bpm") for _, row in heart_recent]
            earlier_hearts = [valid_vital_value(row, "heart_rate_bpm") for _, row in earlier]
            if (len(earlier_hearts) == 6 and all(value is not None for value in hearts + earlier_hearts)
                    and (heart_recent[-1][0] - heart_recent[0][0]).total_seconds() <= 35):
                current = median(hearts)
                difference = current - baseline
                local_difference = current - median(earlier_hearts)
                if difference >= 20 and local_difference >= 20:
                    candidates.append(self._alert("hr_rise", 1, "心率较个人基线明显升高，请确认是否不适并检查手环佩戴。", "心率较个人基线明显升高，请检查手环佩戴，并确认是否不适。"))
                elif difference <= -20 and local_difference <= -20:
                    candidates.append(self._alert("hr_fall", 1, "心率较个人基线明显下降，请确认是否头晕或不适并检查手环佩戴。", "心率较个人基线明显下降，请检查手环佩戴，并确认是否头晕或不适。"))

        alert = max(candidates, key=lambda item: item["priority"], default=None)
        if latest_at == self.last_sample_at:
            return {"alert": alert, "announcement": None}
        self.last_sample_at = latest_at
        if alert is None:
            self.last_alert_key = None
            return {"alert": None, "announcement": None}
        changed = alert["key"] != self.last_alert_key
        elapsed = (now - self.last_announcement_at).total_seconds() if self.last_announcement_at else float("inf")
        announcement = alert if changed and (alert["priority"] >= 2 or elapsed >= self.cooldown_seconds) else None
        if announcement:
            self.last_announcement_at = now
            self.last_alert_key = alert["key"]
        return {"alert": alert, "announcement": announcement}

    @staticmethod
    def _alert(key: str, priority: int, message: str, speech: str) -> dict:
        return {"key": key, "priority": priority, "message": message, "speech": speech}
