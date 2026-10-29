"""文本纠正工具.

用于纠正语音识别中的专业术语错误，特别是按摩相关的术语。
"""

import re
from typing import Optional, Tuple

from src.utils.logging_config import get_logger

logger = get_logger(__name__)

# 术语纠正映射：将常见的语音识别错误映射到正确的术语
TERM_CORRECTION_MAP = {
    # 点筋法的常见误识别
    "点金": "点筋",
    "点经": "点筋",
    "点精": "点筋",
    "点进": "点筋",
    "点津": "点筋",
    "点劲": "点筋",
    "电筋": "点筋",
    "典筋": "点筋",
    "垫筋": "点筋",
    "点金法": "点筋法",
    "点经法": "点筋法",
    "点精法": "点筋法",
    "点进法": "点筋法",
    "点津法": "点筋法",
    "点劲法": "点筋法",
    "电筋法": "点筋法",
    "典筋法": "点筋法",
    "垫筋法": "点筋法",
    "点金房": "点筋法",
    "点经房": "点筋法",
    "点精房": "点筋法",
    "点金法按摩": "点筋法按摩",
    "点经法按摩": "点筋法按摩",
    "点精法按摩": "点筋法按摩",
    
    # 分筋法的常见误识别
    "分金": "分筋",
    "分经": "分筋",
    "分精": "分筋",
    "分进": "分筋",
    "分津": "分筋",
    "分劲": "分筋",
    "粉筋": "分筋",
    "分金法": "分筋法",
    "分经法": "分筋法",
    "分精法": "分筋法",
    "分进法": "分筋法",
    "分津法": "分筋法",
    "分劲法": "分筋法",
    "粉筋法": "分筋法",
    "分金房": "分筋法",
    "分经房": "分筋法",
    "分精房": "分筋法",
    "分金法按摩": "分筋法按摩",
    "分经法按摩": "分筋法按摩",
    "分精法按摩": "分筋法按摩",
    
    # 顺筋法的常见误识别
    "顺金": "顺筋",
    "顺经": "顺筋",
    "顺精": "顺筋",
    "顺进": "顺筋",
    "顺津": "顺筋",
    "顺劲": "顺筋",
    "瞬筋": "顺筋",
    "顺金法": "顺筋法",
    "顺经法": "顺筋法",
    "顺精法": "顺筋法",
    "顺进法": "顺筋法",
    "顺津法": "顺筋法",
    "顺劲法": "顺筋法",
    "瞬筋法": "顺筋法",
    "顺金房": "顺筋法",
    "顺经房": "顺筋法",
    "顺精房": "顺筋法",
    "顺金法按摩": "顺筋法按摩",
    "顺经法按摩": "顺筋法按摩",
    "顺精法按摩": "顺筋法按摩",
}


def correct_text(text: str) -> Tuple[str, bool]:
    """
    纠正文本中的专业术语错误.
    
    参数:
        text: 原始文本（可能包含识别错误）
    
    返回:
        Tuple[纠正后的文本, 是否进行了纠正]
    """
    if not text:
        return text, False
    
    original_text = text
    corrected_text = text
    
    # 尝试完全匹配替换
    for wrong_term, correct_term in TERM_CORRECTION_MAP.items():
        if wrong_term in corrected_text:
            corrected_text = corrected_text.replace(wrong_term, correct_term)
    
    # 如果完全匹配没有找到，尝试模糊匹配
    if corrected_text == original_text:
        # 使用正则表达式进行模糊匹配
        # 匹配"点/分/顺" + "筋/金/经/精/进/津/劲" + 可能的"法"/"房"/"按摩"等
        patterns = [
            (r'点([金经精进津劲筋电典垫])([法房]|按摩)?', r'点筋\2'),
            (r'分([金经精进津劲筋粉])([法房]|按摩)?', r'分筋\2'),
            (r'顺([金经精进津劲筋瞬])([法房]|按摩)?', r'顺筋\2'),
        ]
        
        for pattern, replacement in patterns:
            if re.search(pattern, corrected_text):
                corrected_text = re.sub(pattern, replacement, corrected_text)
                break
    
    # 进一步处理"房"误识别为"法"的情况（处理没有在模糊匹配中捕获的情况）
    if "房" in corrected_text:
        # 如果包含"点筋房"、"分筋房"、"顺筋房"等，替换为对应的"法"
        corrected_text = re.sub(r'点筋房', '点筋法', corrected_text)
        corrected_text = re.sub(r'分筋房', '分筋法', corrected_text)
        corrected_text = re.sub(r'顺筋房', '顺筋法', corrected_text)
    
    was_corrected = corrected_text != original_text
    
    if was_corrected:
        logger.info(f"[TEXT_CORRECTOR] 文本纠正: '{original_text}' -> '{corrected_text}'")
    
    return corrected_text, was_corrected


def should_hide_user_input(config_manager) -> bool:
    """
    检查是否应该隐藏用户语音输入.
    
    参数:
        config_manager: 配置管理器实例
    
    返回:
        是否应该隐藏用户输入
    """
    try:
        return config_manager.get_config(
            "UI_OPTIONS.HIDE_USER_SPEECH_INPUT",
            False
        )
    except Exception:
        return False

