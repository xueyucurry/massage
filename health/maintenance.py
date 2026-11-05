#!/usr/bin/env python3
"""Explicit local maintenance commands for health-band data."""

import argparse
import json
from pathlib import Path

from storage import HealthStore


PROJECT_ROOT = Path(__file__).resolve().parent


def database_path() -> Path:
    config = json.loads((PROJECT_ROOT / "health.json").read_text(encoding="utf-8"))
    value = Path(config.get("database_path") or "data/health_band.sqlite3")
    return value if value.is_absolute() else PROJECT_ROOT / value


def main() -> None:
    parser = argparse.ArgumentParser(description="4G 手环本地数据维护")
    parser.add_argument("command", choices=("prune",))
    parser.add_argument("--days", type=int, required=True, help="保留最近多少天的数据（1-3650）")
    args = parser.parse_args()
    deleted = HealthStore(database_path()).prune_older_than_days(args.days)
    print(f"已删除 {deleted} 条早于 {args.days} 天的本地手环记录")


if __name__ == "__main__":
    main()
