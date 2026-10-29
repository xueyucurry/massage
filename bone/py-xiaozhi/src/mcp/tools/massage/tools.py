"""按摩工具实现.

将语义化手法映射到用户函数模块，并执行。
"""

import asyncio
import importlib
import json
import os
import sys
from typing import Any, Dict

from src.utils.logging_config import get_logger

logger = get_logger(__name__)


METHOD_TO_MODULE = {
    "dianjing": "src.user_functions.dianjing",
    "fenjing": "src.user_functions.fenjing",
    "shunjing": "src.user_functions.shunjing",
}


async def start_massage(args: Dict[str, Any]) -> str:
    method = (args.get("method") or "").strip().lower()
    duration_min = int(args.get("duration_min") or 0)
    logger.info(f"[MASSAGE] start_massage called: method={method}, duration_min={duration_min}")

    if method not in METHOD_TO_MODULE:
        return json.dumps(
            {
                "success": False,
                "message": "method 必须是 'dianjing' | 'fenjing' | 'shunjing'",
            },
            ensure_ascii=False,
        )

    module_name = METHOD_TO_MODULE[method]
    logger.info(f"[MASSAGE] resolved module: {module_name}")

    # 确保语音调用场景下，用户脚本可按 "from fairino import ..." 找到同目录 SDK
    try:
        # 定位到 src 目录，再拼出 src/user_functions
        src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        user_funcs_dir = os.path.join(src_dir, "user_functions")
        if os.path.isdir(user_funcs_dir) and user_funcs_dir not in sys.path:
            sys.path.insert(0, user_funcs_dir)
            logger.info(f"[MASSAGE] inserted to sys.path: {user_funcs_dir}")
        else:
            logger.info(f"[MASSAGE] path already present or not found: {user_funcs_dir}")
    except Exception as e:
        logger.info(f"[MASSAGE] sys.path insert failed: {e}")

    try:
        logger.info(f"[MASSAGE] importing module: {module_name}")
        module = importlib.import_module(module_name)
        # 约定用户模块里导出 `main` 入口；如无则尝试与模块同名函数
        func = getattr(module, "main", None)
        if func is None:
            fname = method  # 如 dianjing.py 里有 dianjing() 则也可
            func = getattr(module, fname, None)
        if func is None:
            logger.info("[MASSAGE] entry function not found: main() or same-named function")
            return json.dumps(
                {"success": False, "message": "未找到 main 或同名函数"}, ensure_ascii=False
            )
        logger.info(f"[MASSAGE] resolved function: {func.__name__}")
    except Exception as e:
        logger.info(f"[MASSAGE] import failed: {e}")
        return json.dumps({"success": False, "message": str(e)}, ensure_ascii=False)

    try:
        if asyncio.iscoroutinefunction(func):
            logger.info("[MASSAGE] calling coroutine function")
            result = await func(duration_min=duration_min)
        else:
            # 允许函数签名不接收参数
            logger.info("[MASSAGE] calling sync function in thread")
            try:
                result = await asyncio.to_thread(func, duration_min=duration_min)
            except TypeError:
                logger.info("[MASSAGE] function has no duration_min parameter, calling without args")
                result = await asyncio.to_thread(func)

        logger.info(f"[MASSAGE] function returned: {result}")
        return json.dumps({"success": True, "result": result}, ensure_ascii=False)
    except Exception as e:
        logger.error(f"[MASSAGE] 执行失败: {e}", exc_info=True)
        return json.dumps({"success": False, "message": str(e)}, ensure_ascii=False)


