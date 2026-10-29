"""Runner 工具管理器.

负责注册用于运行自定义函数/脚本的 MCP 工具。
"""

from typing import Any, Dict, Callable

from src.utils.logging_config import get_logger

from .tools import run_function

logger = get_logger(__name__)


class RunnerToolsManager:
    """Runner 工具管理器."""

    def __init__(self) -> None:
        self._initialized = False

    def init_tools(self, add_tool, PropertyList, Property, PropertyType) -> None:
        """注册 Runner 相关工具。"""
        try:
            # 注册运行自定义函数工具
            props = PropertyList(
                [
                    Property("module", PropertyType.STRING),
                    Property("function", PropertyType.STRING),
                    Property("args", PropertyType.STRING, default_value="[]"),
                    Property("kwargs", PropertyType.STRING, default_value="{}"),
                ]
            )

            add_tool(
                (
                    "self.runner.run_function",
                    "运行自定义函数。参数：module 模块名(如 src.user_functions.my_funcs)，"
                    "function 函数名，args JSON数组字符串，kwargs JSON对象字符串。",
                    props,
                    run_function,
                )
            )

            self._initialized = True
            logger.info("[RunnerManager] Runner 工具注册完成")
        except Exception as e:
            logger.error(f"[RunnerManager] 注册失败: {e}", exc_info=True)
            raise


