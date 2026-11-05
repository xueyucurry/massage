"""Read health-band data without changing the receiver or its SQLite database."""

import json
import os
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DATABASE_PATH = PROJECT_ROOT / "health" / "data" / "health_band.sqlite3"
DEFAULT_POLICY_PATH = PROJECT_ROOT / "health" / "health_alert_policy.json"
DEFAULT_POLICY = {
    "stale_after_seconds": 90,
    "expected_upload_interval_seconds": 30,
    "upload_interval_factor": 2.0,
    "low_battery_percent": 20,
    "consecutive_samples": 3,
    "trend_minutes": 10,
    "consecutive_invalid_samples": 3,
    "invalid_measurements": {
        "heart_rate_bpm": {"min_exclusive": 0},
        "spo2_percent": {"min_exclusive": 0, "max": 100},
        "temperature_c": {"min_exclusive": 0},
    },
    "thresholds": {
        "heart_rate_bpm": {"low": None, "high": None},
        "spo2_percent": {"low": None, "high": None},
        "temperature_c": {"low": None, "high": None},
    },
}
MEASUREMENT_LABELS = {
    "heart_rate_bpm": "心率",
    "spo2_percent": "血氧",
    "temperature_c": "体温",
}


def _normalise_mac(value: Any) -> str:
    return "".join(char for char in str(value or "").upper() if char in "0123456789ABCDEF")


def _as_utc(value: str) -> Optional[datetime]:
    try:
        parsed = datetime.fromisoformat(value)
        return parsed.replace(tzinfo=timezone.utc) if parsed.tzinfo is None else parsed.astimezone(timezone.utc)
    except (TypeError, ValueError):
        return None


def _number(value: Any) -> Optional[float]:
    try:
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None


def mask_mac(value: Any) -> str:
    """Keep enough of an identifier for support without exposing the full MAC."""
    mac = _normalise_mac(value)
    return f"{mac[:2]}******{mac[-4:]}" if len(mac) == 12 else ""


class HealthBandReader:
    """Small read-only view over ``health/data/health_band.sqlite3``."""

    def __init__(self, database_path: Optional[Path | str] = None, policy_path: Optional[Path | str] = None) -> None:
        configured = database_path or os.getenv("HEALTH_BAND_DB") or DEFAULT_DATABASE_PATH
        self.database_path = Path(configured).expanduser().resolve()
        self.policy_path = Path(policy_path or os.getenv("HEALTH_BAND_ALERT_POLICY") or DEFAULT_POLICY_PATH).expanduser()

    def _connect(self) -> sqlite3.Connection:
        if not self.database_path.is_file():
            raise FileNotFoundError(f"未找到手环数据库：{self.database_path}")
        connection = sqlite3.connect(f"file:{self.database_path}?mode=ro", uri=True, timeout=3)
        connection.row_factory = sqlite3.Row
        return connection

    def policy(self) -> Dict[str, Any]:
        policy = json.loads(json.dumps(DEFAULT_POLICY))
        if not self.policy_path.is_file():
            return policy
        try:
            loaded = json.loads(self.policy_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return policy
        if not isinstance(loaded, dict):
            return policy
        for key in ("stale_after_seconds", "expected_upload_interval_seconds", "upload_interval_factor", "low_battery_percent", "consecutive_samples", "trend_minutes", "consecutive_invalid_samples"):
            if key in loaded:
                policy[key] = loaded[key]
        if isinstance(loaded.get("thresholds"), dict):
            for metric in policy["thresholds"]:
                if isinstance(loaded["thresholds"].get(metric), dict):
                    policy["thresholds"][metric].update(loaded["thresholds"][metric])
        if isinstance(loaded.get("invalid_measurements"), dict):
            for metric in policy["invalid_measurements"]:
                if isinstance(loaded["invalid_measurements"].get(metric), dict):
                    policy["invalid_measurements"][metric].update(loaded["invalid_measurements"][metric])
        return policy

    @staticmethod
    def _invalid_measurement(value: Any, rule: Dict[str, Any]) -> bool:
        numeric = _number(value)
        if numeric is None:
            return True
        minimum = _number(rule.get("min_exclusive"))
        maximum = _number(rule.get("max"))
        return (minimum is not None and numeric <= minimum) or (maximum is not None and numeric > maximum)

    def quality_sample(self, sample: Optional[Dict[str, Any]], policy: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Mask obviously invalid sensor values for display while retaining raw DB rows."""
        result = dict(sample or {})
        invalid = []
        for metric, label in MEASUREMENT_LABELS.items():
            if self._invalid_measurement(result.get(metric), (policy or self.policy())["invalid_measurements"][metric]):
                result[metric] = None
                invalid.append(label)
        result["invalid_measurements"] = invalid
        return result

    @staticmethod
    def _sample(row: sqlite3.Row) -> Dict[str, Any]:
        return {
            "received_at": row["received_at"], "device_mac": row["device_mac"],
            "device_time": row["device_time_utc"] or row["device_time_raw"],
            "heart_rate_bpm": row["heart_rate_bpm"], "spo2_percent": row["spo2_percent"],
            "temperature_c": row["temperature_c"], "battery_percent": row["battery_percent"],
        }

    def _samples(self, sql: str, params: List[Any]) -> List[Dict[str, Any]]:
        with self._connect() as connection:
            rows = connection.execute(sql, params).fetchall()
        return [self._sample(row) for row in rows]

    def latest(self, device_mac: str = "") -> Optional[Dict[str, Any]]:
        mac = _normalise_mac(device_mac)
        sql, params = "SELECT * FROM health_band_samples", []
        if mac:
            sql += " WHERE device_mac = ?"
            params.append(mac)
        return next(iter(self._samples(sql + " ORDER BY received_at DESC LIMIT 1", params)), None)

    def history(self, hours: int = 1, device_mac: str = "") -> List[Dict[str, Any]]:
        hours = max(1, min(int(hours), 24 * 31))
        mac = _normalise_mac(device_mac)
        sql = "SELECT * FROM health_band_samples WHERE julianday(received_at) >= julianday('now', ?)"
        params: List[Any] = [f"-{hours} hours"]
        if mac:
            sql += " AND device_mac = ?"
            params.append(mac)
        return self._samples(sql + " ORDER BY received_at ASC", params)

    def recent(self, minutes: int = 30, device_mac: str = "", limit: int = 120, since: Optional[datetime] = None) -> List[Dict[str, Any]]:
        minutes = max(1, min(int(minutes), 24 * 31 * 60))
        mac = _normalise_mac(device_mac)
        sql = "SELECT * FROM health_band_samples WHERE julianday(received_at) >= julianday('now', ?)"
        params: List[Any] = [f"-{minutes} minutes"]
        if mac:
            sql += " AND device_mac = ?"
            params.append(mac)
        if since:
            sql += " AND julianday(received_at) >= julianday(?)"
            params.append(since.astimezone(timezone.utc).isoformat())
        sql += " ORDER BY received_at ASC LIMIT ?"
        params.append(max(1, min(int(limit), 500)))
        return self._samples(sql, params)

    def _latest_samples(self, device_mac: str, limit: int, since: Optional[datetime] = None) -> List[Dict[str, Any]]:
        sql = "SELECT * FROM health_band_samples WHERE device_mac = ?"
        params: List[Any] = [_normalise_mac(device_mac)]
        if since:
            sql += " AND julianday(received_at) >= julianday(?)"
            params.append(since.astimezone(timezone.utc).isoformat())
        sql += " ORDER BY received_at DESC LIMIT ?"
        params.append(max(1, min(int(limit), 500)))
        return list(reversed(self._samples(sql, params)))

    def status(self, stale_after_seconds: Optional[int] = None) -> List[Dict[str, Any]]:
        samples = self._samples(
            """SELECT sample.* FROM health_band_samples AS sample
            INNER JOIN (SELECT device_mac, MAX(received_at) AS last_received_at
                        FROM health_band_samples GROUP BY device_mac) AS latest
            ON sample.device_mac = latest.device_mac AND sample.received_at = latest.last_received_at
            ORDER BY sample.received_at DESC""", [],
        )
        return [self.assess(sample, stale_after_seconds) for sample in samples]

    def assess(self, sample: Optional[Dict[str, Any]], stale_after_seconds: Optional[int] = None, since: Optional[datetime] = None) -> Dict[str, Any]:
        policy = self.policy()
        stale_after = int(stale_after_seconds or policy["stale_after_seconds"])
        result: Dict[str, Any] = self.quality_sample(sample, policy)

        def awaiting(message: str) -> Dict[str, Any]:
            """Do not expose a partial/zero-valued measurement as live vitals."""
            return {
                "status": "awaiting",
                "status_label": "等待接收",
                "color": "#8A94A6",
                "message": message,
                "age_seconds": None,
                "alerts": [],
                "trends": {},
                "trend_minutes": int(policy["trend_minutes"]),
            }

        if not sample:
            if since:
                return awaiting("正在接收数据，等待本次会话首条非零、非空的有效手环上报")
            return {"status": "unavailable", "status_label": "未收到数据", "color": "#8A94A6", "message": "尚未收到手环数据", "age_seconds": None, "alerts": ["尚未收到手环数据"], "trends": {}}

        received_at = _as_utc(str(sample.get("received_at") or ""))
        if since and (not received_at or received_at < since.astimezone(timezone.utc)):
            return awaiting("正在接收数据，等待本次会话首条非零、非空的有效手环上报")

        # The GUI only enters its live-data view after a complete vital-sign
        # sample arrives.  A zero or missing vital sign is normally a sensor
        # acquisition failure, not a value to present as the current reading.
        if result.get("invalid_measurements"):
            return awaiting("正在接收数据，等待下一条非零、非空的有效手环上报")

        age_seconds = None if not received_at else max(0, int((datetime.now(timezone.utc) - received_at).total_seconds()))
        alerts: List[str] = []
        invalid = result.get("invalid_measurements") or []
        if age_seconds is None:
            alerts.append("接收时间无法识别")
        elif age_seconds > stale_after:
            alerts.append(f"最后一条数据已超过 {stale_after} 秒")
        if invalid:
            alerts.append("本次" + "、".join(invalid) + "未获得有效测量")

        battery, battery_limit = _number(sample.get("battery_percent")), _number(policy.get("low_battery_percent"))
        if battery is not None and battery_limit is not None and battery <= battery_limit:
            alerts.append(f"手环电量低（{battery:g}%）")

        samples = self._latest_samples(str(sample.get("device_mac") or ""), max(8, int(policy["consecutive_samples"]), int(policy["consecutive_invalid_samples"])), since)
        if len(samples) >= 2 and age_seconds is not None and age_seconds <= stale_after:
            previous = _as_utc(str(samples[-2].get("received_at") or ""))
            if previous and received_at:
                interval = int((received_at - previous).total_seconds())
                expected = _number(policy.get("expected_upload_interval_seconds")) or 30
                factor = _number(policy.get("upload_interval_factor")) or 2
                result["last_upload_interval_seconds"] = interval
                if interval > expected * factor:
                    alerts.append(f"最近上传间隔偏长（{interval} 秒）")

        invalid_count = max(1, int(policy["consecutive_invalid_samples"]))
        for metric, label in MEASUREMENT_LABELS.items():
            invalid_streak = 0
            for item in reversed(samples):
                if self._invalid_measurement(item.get(metric), policy["invalid_measurements"][metric]):
                    invalid_streak += 1
                else:
                    break
            if invalid_streak >= invalid_count:
                alerts.insert(0, f"{label}连续 {invalid_streak} 条未获得有效测量，请检查佩戴或手环状态")

        count = max(1, int(policy["consecutive_samples"]))
        for metric, label in MEASUREMENT_LABELS.items():
            bounds = policy["thresholds"].get(metric, {})
            low, high = _number(bounds.get("low")), _number(bounds.get("high"))
            values = [
                _number(item.get(metric)) for item in samples
                if not self._invalid_measurement(item.get(metric), policy["invalid_measurements"][metric])
            ][-count:]
            if len(values) == count and low is not None and all(value < low for value in values):
                alerts.append(f"{label}连续 {count} 条低于已配置阈值")
            if len(values) == count and high is not None and all(value > high for value in values):
                alerts.append(f"{label}连续 {count} 条高于已配置阈值")

        trend_minutes = max(1, min(int(policy["trend_minutes"]), 60))
        trend_samples = [self.quality_sample(item, policy) for item in self.recent(trend_minutes, str(sample.get("device_mac") or ""), 120, since)]
        trends = {metric: [item[metric] for item in trend_samples if item.get(metric) is not None] for metric in MEASUREMENT_LABELS}
        result["trend_minutes"] = trend_minutes
        if age_seconds is None:
            status, label, color = "alert", "时间异常", "#D64545"
        elif age_seconds > stale_after:
            status, label, color = "stale", "数据超时", "#D9822B"
        elif alerts:
            status, label, color = "alert", "需要关注", "#D9822B"
        else:
            status, label, color = "normal", "数据正常", "#12A87A"
        result.update({"status": status, "status_label": label, "color": color, "message": alerts[0] if alerts else "手环数据已连接", "age_seconds": age_seconds, "alerts": alerts, "trends": trends})
        return result

    def summary(self, stale_after_seconds: Optional[int] = None, since: Optional[datetime] = None) -> Dict[str, Any]:
        try:
            return self.assess(self.latest(), stale_after_seconds, since)
        except (sqlite3.Error, OSError) as exc:
            return {"status": "unavailable", "status_label": "读取失败", "color": "#D64545", "message": f"健康数据不可用：{exc}", "age_seconds": None, "alerts": ["健康数据不可用"], "trends": {}}

    @staticmethod
    def public_sample(sample: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if not sample:
            return sample
        result = dict(sample)
        if "device_mac" in result:
            result["device_mac"] = mask_mac(result["device_mac"])
        return result
