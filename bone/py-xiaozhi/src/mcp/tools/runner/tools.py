"""Runner 工具实现.

提供运行自定义函数的具体逻辑。
"""

import asyncio
import importlib
import json
from typing import Any, Dict

from src.utils.logging_config import get_logger

logger = get_logger(__name__)


async def run_function(args: Dict[str, Any]) -> str:
    """运行用户提供的函数。

    参数：
    - module: 字符串，模块路径，如 'src.user_functions.my_funcs'
    - function: 字符串，函数名，如 'foo'
    - args: 字符串，JSON 数组，如 '[1, 2, "x"]'（可选）
    - kwargs: 字符串，JSON 对象，如 '{"a":1}'（可选）
    返回：函数的字符串化结果（JSON.dumps 后的文本）。
    """
    module_name = args.get("module")
    function_name = args.get("function")
    args_json = args.get("args", "[]")
    kwargs_json = args.get("kwargs", "{}")

    if not module_name or not function_name:
        return json.dumps(
            {"success": False, "message": "缺少 module 或 function 参数"}, ensure_ascii=False
        )

    try:
        call_args = json.loads(args_json) if isinstance(args_json, str) else (args_json or [])
        call_kwargs = json.loads(kwargs_json) if isinstance(kwargs_json, str) else (kwargs_json or {})
        if not isinstance(call_args, list) or not isinstance(call_kwargs, dict):
            raise ValueError("args 必须是 JSON 数组，kwargs 必须是 JSON 对象")
    except Exception as e:
        return json.dumps({"success": False, "message": f"参数解析失败: {e}"}, ensure_ascii=False)

    try:
        module = importlib.import_module(module_name)
        func = getattr(module, function_name)
    except Exception as e:
        return json.dumps({"success": False, "message": f"加载失败: {e}"}, ensure_ascii=False)

    try:
        if asyncio.iscoroutinefunction(func):
            result = await func(*call_args, **call_kwargs)
        else:
            result = await asyncio.to_thread(func, *call_args, **call_kwargs)
        return json.dumps({"success": True, "result": result}, ensure_ascii=False)
    except Exception as e:
        logger.error(f"[Runner] 执行失败: {e}", exc_info=True)
        return json.dumps({"success": False, "message": str(e)}, ensure_ascii=False)


