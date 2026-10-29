"""MCP callbacks for FAIRINO trajectory-based massage."""

import json
from typing import Any, Dict, Optional

from .runtime import _speech_safe_label, get_runtime


def _optional_float(value: Any) -> Optional[float]:
    if value in (None, "", 0, "0"):
        return None
    return float(value)


def _optional_int(value: Any) -> Optional[int]:
    if value in (None, "", 0, "0"):
        return None
    return int(value)


def _tts_safe_payload(value: Any) -> Any:
    if isinstance(value, str):
        return _speech_safe_label(value)
    if isinstance(value, dict):
        return {key: _tts_safe_payload(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_tts_safe_payload(item) for item in value]
    if isinstance(value, tuple):
        return [_tts_safe_payload(item) for item in value]
    return value


def _dumps_result(result: Dict[str, Any]) -> str:
    return json.dumps(_tts_safe_payload(result), ensure_ascii=False)


async def detect_meridian(args: Dict[str, Any]) -> str:
    runtime = get_runtime()
    display_arg = args.get("display")
    result = await runtime.detect(
        target=args.get("target") or "back",
        display=True if display_arg is None else bool(display_arg),
        timeout_s=_optional_float(args.get("timeout_s")),
        stable_frames=_optional_int(args.get("stable_frames")),
    )
    return _dumps_result(result)


async def start_massage(args: Dict[str, Any]) -> str:
    runtime = get_runtime()
    result = await runtime.start(
        actions=args.get("actions") or "all",
        target=args.get("target") or "auto",
        trajectory_path=args.get("trajectory_path") or "",
    )
    return _dumps_result(result)


async def start_shun_jin(args: Dict[str, Any]) -> str:
    runtime = get_runtime()
    result = await runtime.start(
        actions="shun_jin",
        target=args.get("target") or "auto",
        trajectory_path=args.get("trajectory_path") or "",
    )
    return _dumps_result(result)


async def adjust_force(args: Dict[str, Any]) -> str:
    runtime = get_runtime()
    result = await runtime.adjust_force(
        direction=args.get("direction") or "",
        delta_n=_optional_float(args.get("delta_n")),
    )
    return _dumps_result(result)


async def pause_massage(args: Dict[str, Any]) -> str:
    result = await get_runtime().pause()
    return _dumps_result(result)


async def resume_massage(args: Dict[str, Any]) -> str:
    result = await get_runtime().resume()
    return _dumps_result(result)


async def stop_massage(args: Dict[str, Any]) -> str:
    result = await get_runtime().stop()
    return _dumps_result(result)


async def get_status(args: Dict[str, Any]) -> str:
    result = await get_runtime().status()
    return _dumps_result(result)
