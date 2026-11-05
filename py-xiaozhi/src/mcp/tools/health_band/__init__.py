"""Health-band MCP tools."""

from .manager import HealthBandToolsManager

_manager = None


def get_health_band_manager() -> HealthBandToolsManager:
    global _manager
    if _manager is None:
        _manager = HealthBandToolsManager()
    return _manager


__all__ = ["HealthBandToolsManager", "get_health_band_manager"]
