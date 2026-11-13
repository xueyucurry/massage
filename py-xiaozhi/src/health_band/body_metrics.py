"""Local height, weight and BMI storage for the GUI.

This is deliberately separate from the wristband SQLite data: height and weight
are user profile data, while the wristband database only contains reports sent by
the device.  A future measuring-device adapter can call ``update_from_device``.
"""

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_METRICS_PATH = PROJECT_ROOT / "health" / "data" / "body_metrics.json"


def _number(value: Any) -> Optional[float]:
    try:
        return None if value in (None, "") else float(value)
    except (TypeError, ValueError):
        return None


class BodyMetricsStore:
    """Small JSON-backed profile store with a device-update integration point."""

    def __init__(self, path: Optional[Path | str] = None) -> None:
        self.path = Path(path or os.getenv("BODY_METRICS_FILE") or DEFAULT_METRICS_PATH).expanduser()

    @staticmethod
    def _validate(height_cm: Any, weight_kg: Any) -> tuple[float, float]:
        height, weight = _number(height_cm), _number(weight_kg)
        if height is None or not 50 <= height <= 250:
            raise ValueError("身高应在 50–250 cm 之间")
        if weight is None or not 10 <= weight <= 500:
            raise ValueError("体重应在 10–500 kg 之间")
        return height, weight

    def summary(self) -> Dict[str, Any]:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            payload = {}
        if not isinstance(payload, dict):
            payload = {}

        height, weight = _number(payload.get("height_cm")), _number(payload.get("weight_kg"))
        bmi = None if height is None or weight is None or height <= 0 else weight / ((height / 100) ** 2)
        return {
            "height_cm": height,
            "weight_kg": weight,
            "bmi": bmi,
            "source": str(payload.get("source") or "未录入"),
            "updated_at": str(payload.get("updated_at") or ""),
        }

    def _save(self, height_cm: Any, weight_kg: Any, source: str) -> Dict[str, Any]:
        height, weight = self._validate(height_cm, weight_kg)
        payload = {
            "height_cm": height,
            "weight_kg": weight,
            "source": str(source or "设备"),
            "updated_at": datetime.now(timezone.utc).isoformat(),
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(self.path)
        return self.summary()

    def set_manual(self, height_cm: Any, weight_kg: Any) -> Dict[str, Any]:
        return self._save(height_cm, weight_kg, "手动录入")

    def update_from_device(self, height_cm: Any, weight_kg: Any, source: str = "测量设备") -> Dict[str, Any]:
        """Future scales/body-measurement adapters use this public entry point."""
        return self._save(height_cm, weight_kg, source)
