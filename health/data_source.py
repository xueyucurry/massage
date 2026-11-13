"""Fail-closed selection of the wristband read database.

The MQTT receiver is deliberately independent of this module and always writes
its configured real database.  Only readers use this selector.
"""

from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parent / "data"
REAL_DATABASE = DATA_DIR / "health_band.sqlite3"
DEMO_DATABASE = DATA_DIR / "demo_health_band.sqlite3"
CONTROL_FILE = DATA_DIR / "band_demo_control.json"
DEMO_MARKER = "massage-health-alert-demo-v1"


@dataclass(frozen=True)
class ActiveSource:
    mode: str
    path: Path
    token: str
    scenario: str = ""


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def marked_demo_database(path: Path | None = None) -> bool:
    path = path or DEMO_DATABASE
    try:
        if path.is_symlink() or path.resolve() == REAL_DATABASE.resolve() or not path.is_file() or (REAL_DATABASE.exists() and path.samefile(REAL_DATABASE)):
            return False
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True, timeout=1) as db:
            row = db.execute("SELECT value FROM demo_metadata WHERE key='marker'").fetchone()
            return bool(row and row[0] == DEMO_MARKER)
    except (OSError, sqlite3.Error):
        return False


def resolve_active_source() -> ActiveSource:
    real = ActiveSource("real", REAL_DATABASE, "real")
    control = read_json(CONTROL_FILE)
    run_id = control.get("run_id")
    if control.get("mode") != "demo" or not isinstance(run_id, str) or not run_id:
        return real
    if not isinstance(control.get("scenario"), str) or not marked_demo_database():
        return real
    return ActiveSource("demo", DEMO_DATABASE, f"demo:{run_id}", control["scenario"])
