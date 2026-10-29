"""按摩工具管理器.

注册语义化“点筋/分筋/顺筋”触发对应的用户函数模块。
"""

from src.utils.logging_config import get_logger
from .tools import start_massage
from .tools import start_massage as _start

logger = get_logger(__name__)


class MassageToolsManager:
    def __init__(self) -> None:
        self._initialized = False

    def init_tools(self, add_tool, PropertyList, Property, PropertyType) -> None:
        try:
            props = PropertyList(
                [
                    # 手法: dianjing | fenjing | shunjing
                    Property("method", PropertyType.STRING),
                    # 可选：时长（分钟）
                    Property("duration_min", PropertyType.INTEGER, default_value=0),
                ]
            )

            add_tool(
                (
                    "self.massage.start",
                    "【按摩工具】当用户说：开始点筋法按摩/开始点筋、开始分筋法按摩/开始分筋、开始顺筋法按摩/开始顺筋 时调用。"
                    "功能：触发对应的用户脚本模块执行（dianjing/fenjing/shunjing）。"
                    "参数：method 取 'dianjing'|'fenjing'|'shunjing'，可选 duration_min（分钟）。",
                    props,
                    start_massage,
                )
            )

            # 零参数直达工具，提升路由准确率
            dj_props = PropertyList([])
            async def _dj_async(_args):
                return await _start({"method": "dianjing", "duration_min": 0})
            add_tool((
                "self.massage.dianjing",
                "【按摩】点筋法。触发 'src/user_functions/dianjing.py'。触发词：点筋、点筋法、开始点筋。",
                dj_props,
                _dj_async,
            ))

            fj_props = PropertyList([])
            async def _fj_async(_args):
                return await _start({"method": "fenjing", "duration_min": 0})
            add_tool((
                "self.massage.fenjing",
                "【按摩】分筋法。触发 'src/user_functions/fenjing.py'。触发词：分筋、分筋法、开始分筋。",
                fj_props,
                _fj_async,
            ))

            sj_props = PropertyList([])
            async def _sj_async(_args):
                return await _start({"method": "shunjing", "duration_min": 0})
            add_tool((
                "self.massage.shunjing",
                "【按摩】顺筋法。触发 'src/user_functions/shunjing.py'。触发词：顺筋、顺筋法、开始顺筋。",
                sj_props,
                _sj_async,
            ))

            self._initialized = True
            logger.info("[MassageManager] 按摩工具注册完成")
        except Exception as e:
            logger.error(f"[MassageManager] 注册失败: {e}", exc_info=True)
            raise


