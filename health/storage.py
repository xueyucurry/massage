"""SQLite persistence for T10-4G health-band reports."""

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Union


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_DATABASE_PATH = PROJECT_ROOT / "data" / "health_band.sqlite3"


def normalise_mac(value: Any) -> str:
    return "".join(char for char in str(value or "").upper() if char in "0123456789ABCDEF")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _optional_number(value: Any) -> Optional[float]:
    if value in (None, ""):
        return None
    if isinstance(value, bool):
        raise ValueError("布尔值不是有效测量值")
    return float(value)


def _device_time(value: Any) -> tuple[Optional[str], Optional[int], Optional[str]]:
    if value in (None, ""):
        return None, None, None
    raw = str(value).strip()
    try:
        epoch = int(raw, 16)
        return raw.upper(), epoch, datetime.fromtimestamp(epoch, timezone.utc).isoformat()
    except (OverflowError, OSError, ValueError):
        return raw, None, None


class HealthStore:
    def __init__(self, database_path: Optional[Union[Path, str]] = None) -> None:
        self.database_path = Path(database_path or DEFAULT_DATABASE_PATH).expanduser()
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        self._ensure_schema()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.database_path, timeout=10)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA journal_mode=WAL")
        return connection

    def _ensure_schema(self) -> None:
        with self._connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS health_band_samples (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    received_at TEXT NOT NULL,
                    mqtt_topic TEXT NOT NULL,
                    device_mac TEXT NOT NULL,
                    sim_iccid TEXT,
                    firmware_version TEXT,
                    device_time_raw TEXT,
                    device_time_epoch INTEGER,
                    device_time_utc TEXT,
                    heart_rate_bpm REAL,
                    spo2_percent REAL,
                    temperature_c REAL,
                    battery_percent REAL,
                    raw_json TEXT NOT NULL,
                    UNIQUE(device_mac, device_time_raw, raw_json)
                );
                CREATE INDEX IF NOT EXISTS idx_health_band_samples_device_received
                    ON health_band_samples(device_mac, received_at DESC);
                """
            )

    def record_sample(self, payload: Dict[str, Any], topic: str) -> bool:
        if not isinstance(payload, dict):
            raise ValueError("上报内容必须是 JSON 对象")
        device_mac = normalise_mac(payload.get("mac"))
        if len(device_mac) != 12:
            raise ValueError("payload.mac 必须是 12 位十六进制 MAC")

        raw_time, epoch, utc_time = _device_time(payload.get("time"))
        raw_json = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        values = (
            _utc_now(), topic, device_mac, str(payload.get("qccid") or "") or None,
            str(payload.get("v") or "") or None, raw_time, epoch, utc_time,
            _optional_number(payload.get("hr")), _optional_number(payload.get("spo2")),
            _optional_number(payload.get("temperature")), _optional_number(payload.get("battery")), raw_json,
        )
        with self._connect() as connection:
            cursor = connection.execute(
                """
                INSERT OR IGNORE INTO health_band_samples (
                    received_at, mqtt_topic, device_mac, sim_iccid, firmware_version,
                    device_time_raw, device_time_epoch, device_time_utc,
                    heart_rate_bpm, spo2_percent, temperature_c, battery_percent, raw_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                values,
            )
            return cursor.rowcount == 1

    @staticmethod
    def _sample(row: sqlite3.Row) -> Dict[str, Any]:
        return {
            "received_at": row["received_at"],
            "device_mac": row["device_mac"],
            "device_time": row["device_time_utc"] or row["device_time_raw"],
            "heart_rate_bpm": row["heart_rate_bpm"],
            "spo2_percent": row["spo2_percent"],
            "temperature_c": row["temperature_c"],
            "battery_percent": row["battery_percent"],
        }

    def latest(self, device_mac: str = "") -> Optional[Dict[str, Any]]:
        sql, params = "SELECT * FROM health_band_samples", []
        device_mac = normalise_mac(device_mac)
        if device_mac:
            sql += " WHERE device_mac = ?"
            params.append(device_mac)
        sql += " ORDER BY received_at DESC LIMIT 1"
        with self._connect() as connection:
            row = connection.execute(sql, params).fetchone()
        return self._sample(row) if row else None

    def history(self, hours: int = 1, device_mac: str = "") -> List[Dict[str, Any]]:
        hours = max(1, min(int(hours), 24 * 31))
        sql = "SELECT * FROM health_band_samples WHERE julianday(received_at) >= julianday('now', ?)"
        params: List[Any] = [f"-{hours} hours"]
        device_mac = normalise_mac(device_mac)
        if device_mac:
            sql += " AND device_mac = ?"
            params.append(device_mac)
        sql += " ORDER BY received_at ASC"
        with self._connect() as connection:
            rows = connection.execute(sql, params).fetchall()
        return [self._sample(row) for row in rows]

    def status(self, device_macs: Optional[List[str]] = None) -> List[Dict[str, Any]]:
        with self._connect() as connection:
            rows = connection.execute(
                """
                SELECT sample.* FROM health_band_samples AS sample
                INNER JOIN (
                    SELECT device_mac, MAX(received_at) AS last_received_at
                    FROM health_band_samples GROUP BY device_mac
                ) AS latest ON sample.device_mac = latest.device_mac
                    AND sample.received_at = latest.last_received_at
                ORDER BY sample.received_at DESC
                """
            ).fetchall()
        samples = [self._sample(row) for row in rows]
        allowed = {normalise_mac(item) for item in device_macs or []}
        return [sample for sample in samples if not allowed or sample["device_mac"] in allowed]
