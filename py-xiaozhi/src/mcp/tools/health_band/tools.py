"""MCP callbacks for read-only health-band queries."""

import json
from typing import Any, Dict

from src.health_band import HealthBandReader


def _reader() -> HealthBandReader:
    return HealthBandReader()


def latest(args: Dict[str, Any]) -> str:
    reader = _reader()
    sample = reader.latest(args.get("device_mac") or "")
    return json.dumps(reader.public_sample(reader.assess(sample)), ensure_ascii=False)


def history(args: Dict[str, Any]) -> str:
    reader = _reader()
    samples = reader.history(args.get("hours", 1), args.get("device_mac") or "")
    policy = reader.policy()
    return json.dumps({"hours": args.get("hours", 1), "count": len(samples), "samples": [reader.public_sample(reader.quality_sample(sample, policy)) for sample in samples]}, ensure_ascii=False)


def status(args: Dict[str, Any]) -> str:
    reader = _reader()
    return json.dumps({"devices": [reader.public_sample(sample) for sample in reader.status(args.get("stale_after_seconds") or None)]}, ensure_ascii=False)


def assess(args: Dict[str, Any]) -> str:
    """Return freshness/completeness warnings; never controls the robot."""
    reader = _reader()
    return json.dumps(reader.public_sample(reader.summary(args.get("stale_after_seconds") or None)), ensure_ascii=False)
