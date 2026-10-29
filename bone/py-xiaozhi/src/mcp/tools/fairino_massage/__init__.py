"""FAIRINO trajectory-based massage MCP tools."""

from typing import Optional

from .manager import FairinoMassageToolsManager

_fairino_massage_manager: Optional[FairinoMassageToolsManager] = None


def get_fairino_massage_manager() -> FairinoMassageToolsManager:
    global _fairino_massage_manager
    if _fairino_massage_manager is None:
        _fairino_massage_manager = FairinoMassageToolsManager()
    return _fairino_massage_manager


__all__ = ["FairinoMassageToolsManager", "get_fairino_massage_manager"]
