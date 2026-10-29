#!/usr/bin/env python3
"""Human-friendly local queries for saved health-band measurements."""

import argparse
import json
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from storage import HealthStore


PROJECT_ROOT = Path(__file__).resolve().parent
CHINA_TIMEZONE = ZoneInfo("Asia/Shanghai")


def _value(value, suffix: str = "") -> str:
    return "无" if value is None else f"{value}{suffix}"


def _time(value: str | None) -> str:
    if not value:
        return "无"
    try:
        moment = datetime.fromisoformat(value)
        return moment.astimezone(CHINA_TIMEZONE).strftime("%Y-%m-%d %H:%M:%S 北京时间")
    except ValueError:
        return value


def _config() -> dict:
    path = PROJECT_ROOT / "health.json"
    return json.loads(path.read_text(encoding="utf-8"))


def _store(config: dict) -> HealthStore:
    path = config.get("database_path") or "data/health_band.sqlite3"
    database_path = Path(path)
    if not database_path.is_absolute():
        database_path = PROJECT_ROOT / database_path
    return HealthStore(database_path)


def _print_sample(sample: dict | None) -> None:
    if not sample:
        print("尚未收到手环数据")
        return
    print(f"接收时间：{_time(sample['received_at'])}")
    print(f"手环 MAC：{sample['device_mac']}")
    print(f"测量时间：{_time(sample['device_time'])}")
    print(f"心率：{_value(sample['heart_rate_bpm'], ' bpm')}")
    print(f"血氧：{_value(sample['spo2_percent'], ' %')}")
    print(f"体温：{_value(sample['temperature_c'], ' ℃')}")
    print(f"电量：{_value(sample['battery_percent'], ' %')}")


def main() -> None:
    parser = argparse.ArgumentParser(description="查询本地 4G 手环数据")
    parser.add_argument("command", nargs="?", choices=("latest", "status", "history"), default="latest")
    parser.add_argument("--hours", type=int, default=1, help="history 查询小时数")
    args = parser.parse_args()
    config = _config()
    store = _store(config)
    allowed = config.get("allowed_devices") or []

    if args.command == "latest":
        _print_sample(store.latest(allowed[0] if len(allowed) == 1 else ""))
    elif args.command == "status":
        samples = store.status(allowed)
        print("尚未收到手环数据" if not samples else "\n".join(
            f"{sample['device_mac']}：最后接收于 {_time(sample['received_at'])}" for sample in samples
        ))
    else:
        samples = store.history(hours=max(1, min(args.hours, 744)), device_mac=allowed[0] if len(allowed) == 1 else "")
        recent = samples[-10:]
        print(f"最近 {args.hours} 小时共 {len(samples)} 条；显示最后 {len(recent)} 条：")
        for sample in recent:
            print(
                f"{_time(sample['received_at'])} | HR {_value(sample['heart_rate_bpm'])} | "
                f"SpO2 {_value(sample['spo2_percent'])} | T {_value(sample['temperature_c'])} ℃ | "
                f"电量 {_value(sample['battery_percent'])}%"
            )


if __name__ == "__main__":
    main()
