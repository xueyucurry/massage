"""BMI-based massage recommendation policy.

This module never controls the robot.  It stores an automatically scheduled,
single-use recommendation for the massage runtime to consume through its small
adapter.  The runtime's own absolute force limits remain authoritative.
"""

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


HEALTH_DIR = Path(__file__).resolve().parent
RECOMMENDATION_PATH = HEALTH_DIR / "data" / "bmi_massage_recommendation.json"
MIN_FACTOR = 0.90
MAX_FACTOR = 1.10


def _number(value: Any) -> Optional[float]:
    try:
        return None if value in (None, "") else float(value)
    except (TypeError, ValueError):
        return None


def recommendation_for_bmi(bmi: Any) -> Dict[str, Any]:
    """Return the conservative Chinese-adult BMI recommendation, if valid."""
    value = _number(bmi)
    if value is None or not 10 <= value <= 80:
        return {"available": False, "reason": "需要有效的成人 BMI 才能生成按摩建议"}
    if value < 18.5:
        category, factor = "偏瘦", 0.90
    elif value < 24:
        category, factor = "正常", 1.00
    elif value < 28:
        category, factor = "超重", 1.05
    else:
        category, factor = "肥胖", 1.10
    return {
        "available": True,
        "bmi": round(value, 1),
        "category": category,
        "factor": factor,
        "message": f"BMI {value:.1f}（{category}），建议下一次按摩使用基准力度的 {factor:.2f}×",
    }


class MassageRecommendationStore:
    """Private, single-use BMI recommendation store."""

    def __init__(self, path: Optional[Path | str] = None) -> None:
        self.path = Path(path or RECOMMENDATION_PATH)

    def confirm(self, bmi: Any, source: str = "手动录入") -> Dict[str, Any]:
        recommendation = recommendation_for_bmi(bmi)
        if not recommendation["available"]:
            raise ValueError(recommendation["reason"])
        payload = {
            **recommendation,
            "source": str(source or "未知来源"),
            "status": "confirmed",
            "confirmed_at": datetime.now(timezone.utc).isoformat(),
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        temporary.replace(self.path)
        return payload

    def pending(self) -> Dict[str, Any]:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {"available": False}
        return payload if payload.get("status") == "confirmed" else {"available": False}

    def consume(self, baseline_force_n: Any) -> Optional[Dict[str, Any]]:
        """Consume one scheduled recommendation and calculate its requested force."""
        payload = self.pending()
        baseline = _number(baseline_force_n)
        factor = _number(payload.get("factor"))
        if not payload.get("available") or baseline is None or factor is None:
            return None
        factor = max(MIN_FACTOR, min(MAX_FACTOR, factor))
        payload.update(
            status="consumed",
            consumed_at=datetime.now(timezone.utc).isoformat(),
            baseline_force_n=baseline,
            requested_force_n=baseline * factor,
        )
        self.path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        return payload
