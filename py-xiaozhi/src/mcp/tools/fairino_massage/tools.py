"""MCP callbacks for FAIRINO trajectory-based massage."""

import json
from typing import Any, Dict, Optional

from .runtime import get_runtime


def _optional_float(value: Any) -> Optional[float]:
    if value in (None, "", 0, "0"):
        return None
    return float(value)


def _optional_int(value: Any) -> Optional[int]:
    if value in (None, "", 0, "0"):
        return None
    return int(value)


async def detect_meridian(args: Dict[str, Any]) -> str:
    runtime = get_runtime()
    display_arg = args.get("display")
    result = await runtime.detect(
        target=args.get("target") or "back",
        display=True if display_arg is None else bool(display_arg),
        timeout_s=_optional_float(args.get("timeout_s")),
        stable_frames=_optional_int(args.get("stable_frames")),
    )
    return json.dumps(result, ensure_ascii=False)


async def start_massage(args: Dict[str, Any]) -> str:
    runtime = get_runtime()
    result = await runtime.start(
        actions=args.get("actions") or "all",
        target=args.get("target") or "auto",
        trajectory_path=args.get("trajectory_path") or "",
    )
    return json.dumps(result, ensure_ascii=False)


async def adjust_force(args: Dict[str, Any]) -> str:
    runtime = get_runtime()
    result = await runtime.adjust_force(
        direction=args.get("direction") or "stronger",
        delta_n=_optional_float(args.get("delta_n")),
    )
    return json.dumps(result, ensure_ascii=False)


async def pause_massage(args: Dict[str, Any]) -> str:
    result = await get_runtime().pause()
    return json.dumps(result, ensure_ascii=False)


async def resume_massage(args: Dict[str, Any]) -> str:
    result = await get_runtime().resume()
    return json.dumps(result, ensure_ascii=False)


async def stop_massage(args: Dict[str, Any]) -> str:
    result = await get_runtime().stop()
    return json.dumps(result, ensure_ascii=False)


async def get_status(args: Dict[str, Any]) -> str:
    result = await get_runtime().status()
    return json.dumps(result, ensure_ascii=False)
