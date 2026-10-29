"""Runner 工具模块入口.

提供获得 Runner 工具管理器的接口。
"""

from typing import Optional

from .manager import RunnerToolsManager

_runner_manager: Optional[RunnerToolsManager] = None


def get_runner_manager() -> RunnerToolsManager:
    global _runner_manager
    if _runner_manager is None:
        _runner_manager = RunnerToolsManager()
    return _runner_manager


