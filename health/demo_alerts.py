#!/usr/bin/env python3
"""Synthetic wristband sample recipes and marked demo database writer."""

from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from data_source import DEMO_DATABASE, REAL_DATABASE as LIVE_DATABASE, DEMO_MARKER


HEALTH_DIR = Path(__file__).resolve().parent
DEMO_MAC = "D0D0D0D0D0D0"
@dataclass(frozen=True)
class Scenario:
    title: str
    explanation: str
    quality: str
    alert_key: str | None = None


SCENARIOS: dict[str, Scenario] = {
    "waiting": Scenario("等待首条数据", "数据库没有本次会话数据；体征和趋势为空。", "等待接收"),
    "normal": Scenario("正常数据", "三项体征有效，未达到任何预警条件。", "数据正常"),
    "battery_low": Scenario("手环电量低", "电量 15%；这是接收质量提示，不是生理预警。", "电量偏低"),
    "invalid_first": Scenario("首批数据无效", "心率和体温为 0，血氧为空；尚未建立有效展示。", "等待接收"),
    "invalid_latest": Scenario("最新数据无效", "先有有效数据，最新上传的三个体征无效；显示 --，保留有效趋势。", "数据待确认"),
    "stale": Scenario("数据超时", "先正常上报，随后停止新增记录；超过 20 秒提示超时。", "数据超时"),
    "single_spo2_dip": Scenario("血氧单点 88%", "只出现一次 88%，不足连续确认；不发出生理预警。", "数据正常"),
    "spo2_trend_only": Scenario("血氧波动仅留趋势", "血氧有变化，但最近读数恢复；曲线可见，趋势本身不报警。", "数据正常"),
    "spo2_watch": Scenario("血氧持续 94%", "连续 3 条低于 95%，提示检查佩戴并复测。", "数据正常", "spo2_watch"),
    "spo2_urgent": Scenario("血氧持续 88%", "连续 3 条不高于 88%，提示复测并尽快就医。", "数据正常", "spo2_urgent"),
    "temp_high": Scenario("体温持续 38.2℃", "连续 3 条达到 38℃，提示体温计复测。", "数据正常", "temp_high"),
    "temp_urgent": Scenario("体温持续 34.5℃", "连续 3 条不高于 35℃，提示体温计复测并就医。", "数据正常", "temp_urgent"),
    "hr_rise": Scenario("心率快速升高", "稳定基线 82；最近约 30 秒持续到 105。", "数据正常", "hr_rise"),
    "hr_fall": Scenario("心率快速下降", "稳定基线 82；最近约 30 秒持续到 60，提示关注不适。", "数据正常", "hr_fall"),
    "combined": Scenario("多项同时异常", "心率升高、血氧 88%、体温 34.5℃ 同时出现；只显示最高优先级血氧预警。", "数据正常", "spo2_urgent"),
}


def scenario_rows(key: str, now: datetime) -> list[dict[str, Any]]:
    """Build five minutes of realistic five-second readings ending at now."""
    if key not in SCENARIOS:
        raise KeyError(key)
    if key == "waiting":
        return []
    count = 60
    hearts = [76 + (index % 5 - 2) for index in range(count)]
    oxygen = [97 if index % 4 == 0 else 98 for index in range(count)]
    temperature = [36.5 + (index % 3 - 1) * 0.1 for index in range(count)]
    battery = [15 if key == "battery_low" else 80] * count
    if key == "invalid_first":
        hearts, oxygen, temperature = [0] * count, [None] * count, [0] * count
    elif key == "invalid_latest":
        hearts[-1], oxygen[-1], temperature[-1] = 0, None, 0
    elif key == "single_spo2_dip":
        oxygen[-1] = 88
    elif key == "spo2_trend_only":
        oxygen[-9:] = [98, 97, 96, 95, 94, 95, 96, 97, 98]
    elif key in {"spo2_watch", "spo2_urgent"}:
        oxygen[-3:] = [{"spo2_watch": 94, "spo2_urgent": 88}[key]] * 3
    elif key in {"temp_high", "temp_urgent"}:
        temperature[-3:] = [{"temp_high": 38.2, "temp_urgent": 34.5}[key]] * 3
    elif key in {"hr_rise", "hr_fall", "combined"}:
        baseline = 82
        hearts = [baseline + (index % 3 - 1) for index in range(count)]
        hearts[-6:] = [60 if key == "hr_fall" else 105] * 6
        if key == "combined":
            oxygen[-3:] = [88] * 3
            temperature[-3:] = [34.5] * 3
    end = now - timedelta(seconds=35 if key == "stale" else 0)
    return [
        {"received_at": (end - timedelta(seconds=(count - index - 1) * 5)).isoformat(),
         "heart_rate_bpm": hearts[index], "spo2_percent": oxygen[index],
         "temperature_c": temperature[index], "battery_percent": battery[index],
         "device_mac": DEMO_MAC}
        for index in range(count)
    ]


def all_cases_rows(now: datetime) -> list[dict[str, Any]]:
    """One prewritten timeline; the existing readers reveal only due rows."""
    cursor = now - timedelta(seconds=30)
    rows: list[dict[str, Any]] = []

    def normal(index: int = 0, **changes: Any) -> dict[str, Any]:
        value = {"heart_rate_bpm": 82 + (index % 3 - 1), "spo2_percent": 98,
                 "temperature_c": 36.5, "battery_percent": 80}
        value.update(changes)
        return value

    def add(case: str, values: list[dict[str, Any]]) -> None:
        nonlocal cursor
        for value in values:
            cursor += timedelta(seconds=5)
            rows.append({"received_at": cursor.isoformat(), "device_mac": DEMO_MAC,
                         "scenario": case, **value})

    def recover(count: int = 2) -> None:
        add("normal", [normal(index) for index in range(count)])

    # Six stable baseline readings end at switch time. Each case is followed
    # by normal readings so the next alert is a separate, audible event.
    recover(6)
    add("hr_rise", [normal(heart_rate_bpm=105)] * 6)
    recover(6)
    add("hr_fall", [normal(heart_rate_bpm=60)] * 6)
    recover(4)
    add("spo2_watch", [normal(spo2_percent=94)] * 3)
    recover(2)
    add("spo2_urgent", [normal(spo2_percent=88)] * 3)
    recover(2)
    add("temp_urgent", [normal(temperature_c=34.5)] * 3)
    recover(2)
    add("temp_high", [normal(temperature_c=38.2)] * 3)
    recover(2)
    return rows


def _ensure_demo_database(path: Path) -> None:
    path = Path(path)
    if path.is_symlink() or path.resolve() == LIVE_DATABASE.resolve() or (path.exists() and LIVE_DATABASE.exists() and path.samefile(LIVE_DATABASE)):
        raise ValueError("拒绝写入真实手环数据库或符号链接")
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        with sqlite3.connect(path) as connection:
            try:
                marker = connection.execute("SELECT value FROM demo_metadata WHERE key='marker'").fetchone()
            except sqlite3.Error as exc:
                raise ValueError("目标文件不是本演示程序创建的数据库，拒绝覆盖") from exc
            if not marker or marker[0] != DEMO_MARKER:
                raise ValueError("目标文件不是本演示程序创建的数据库，拒绝覆盖")
        return
    with sqlite3.connect(path) as connection:
        connection.executescript("""
            CREATE TABLE demo_metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL);
            CREATE TABLE health_band_samples (
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
            CREATE INDEX idx_health_band_samples_device_received
                ON health_band_samples(device_mac, received_at DESC);
        """)
        connection.execute("INSERT INTO demo_metadata VALUES ('marker', ?)", [DEMO_MARKER])


def _replace_rows(path: Path, rows: list[dict[str, Any]], default_case: str) -> None:
    _ensure_demo_database(path)
    with sqlite3.connect(path) as connection:
        connection.execute("DELETE FROM health_band_samples")
        for index, row in enumerate(rows):
            stamp = datetime.fromisoformat(row["received_at"])
            epoch = int(stamp.timestamp())
            device_time_raw = f"{epoch:08X}"
            payload = {
                "mac": DEMO_MAC, "qccid": "8986000000000000000", "v": "demo-1.0",
                "time": device_time_raw, "hr": row["heart_rate_bpm"],
                "spo2": row["spo2_percent"], "temperature": row["temperature_c"],
                "battery": row["battery_percent"], "demo": True,
                "scenario": row.get("scenario", default_case), "index": index,
            }
            connection.execute(
                """INSERT INTO health_band_samples
                   (received_at, mqtt_topic, device_mac, sim_iccid,
                    firmware_version, device_time_raw, device_time_epoch,
                    device_time_utc, heart_rate_bpm, spo2_percent,
                    temperature_c, battery_percent, raw_json)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                [row["received_at"], "demo/b/vitals", DEMO_MAC, payload["qccid"],
                 payload["v"], device_time_raw, epoch, stamp.astimezone(timezone.utc).isoformat(),
                 row["heart_rate_bpm"], row["spo2_percent"], row["temperature_c"],
                 row["battery_percent"], json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))],
            )


def write_scenario(path: Path, key: str, now: datetime | None = None) -> list[dict[str, Any]]:
    """Replace samples in the marked test database; used by offline tests."""
    rows = scenario_rows(key, now or datetime.now(timezone.utc))
    _replace_rows(path, rows, key)
    return rows


def write_all_cases(path: Path = DEMO_DATABASE, now: datetime | None = None) -> list[dict[str, Any]]:
    """Write one complete time-ordered test database and then leave it alone."""
    rows = all_cases_rows(now or datetime.now(timezone.utc))
    _replace_rows(path, rows, "all")
    return rows
