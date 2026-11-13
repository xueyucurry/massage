#!/usr/bin/env python3
"""Prepare a marked test database and switch only wristband readers to it."""

from __future__ import annotations

import argparse
import json
import os
import uuid
from pathlib import Path

from data_source import CONTROL_FILE, DEMO_DATABASE, resolve_active_source
from demo_alerts import write_all_cases


def atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("start", "stop", "status"):
        commands.add_parser(name)
    args = parser.parse_args(argv)
    if args.command == "status":
        source = resolve_active_source()
        print("当前读取：" + ("模拟数据库" if source.mode == "demo" else "真实手环数据库"))
        return 0
    if args.command == "stop":
        atomic_json(CONTROL_FILE, {"mode": "real"})
        print("已恢复读取真实手环数据库。")
        return 0
    rows = write_all_cases(DEMO_DATABASE)
    atomic_json(CONTROL_FILE, {"mode": "demo", "run_id": uuid.uuid4().hex, "scenario": "all"})
    print(f"已切换到包含全部测试情况的模拟数据库，共 {len(rows)} 条记录。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
