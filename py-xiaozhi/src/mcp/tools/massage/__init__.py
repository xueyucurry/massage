"""按摩工具模块入口.

提供获取按摩工具管理器的接口。
"""

from typing import Optional

from .manager import MassageToolsManager

_massage_manager: Optional[MassageToolsManager] = None


def get_massage_manager() -> MassageToolsManager:
    global _massage_manager
    if _massage_manager is None:
        _massage_manager = MassageToolsManager()
    return _massage_manager


