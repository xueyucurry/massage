#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
唤醒词自动生成工具

功能：
1. 输入中文自动转换为带声调拼音
2. 按字母分隔拼音（声母+韵母）
3. 验证token是否在tokens.txt中
4. 自动生成keywords.txt格式
"""

import sys
from pathlib import Path

try:
    from pypinyin import lazy_pinyin, Style
except ImportError:
    print("❌ 缺少依赖: pypinyin")
    print("请安装: pip install pypinyin")
    sys.exit(1)


class KeywordGenerator:
    def __init__(self, model_dir: Path):
        """
        初始化唤醒词生成器

        Args:
            model_dir: 模型目录路径（包含tokens.txt和keywords.txt）
        """
        self.model_dir = Path(model_dir)
        self.tokens_file = self.model_dir / "tokens.txt"
        self.keywords_file = self.model_dir / "keywords.txt"

        # 加载已有的tokens
        self.available_tokens = self._load_tokens()

        # 声母表（需要分离的）
        self.initials = [
            'b', 'p', 'm', 'f', 'd', 't', 'n', 'l',
            'g', 'k', 'h', 'j', 'q', 'x',
            'zh', 'ch', 'sh', 'r', 'z', 'c', 's', 'y', 'w'
        ]

    def _load_tokens(self) -> set:
        """加载tokens.txt中的所有可用token"""
        if not self.tokens_file.exists():
            print(f"⚠️  警告: tokens文件不存在: {self.tokens_file}")
            return set()

        tokens = set()
        with open(self.tokens_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    # 格式: "token id" 或 "token"
                    parts = line.split()
                    if parts:
                        tokens.add(parts[0])

        print(f"✅ 加载了 {len(tokens)} 个可用tokens")
        return tokens

    def _split_pinyin(self, pinyin: str) -> list:
        """
        将拼音按声母韵母分隔

        例如: "xiǎo" -> ["x", "iǎo"]
              "mǐ" -> ["m", "ǐ"]
              "ài" -> ["ài"]  (零声母)
        """
        if not pinyin:
            return []

        # 按长度优先尝试匹配声母（zh, ch, sh优先）
        for initial in sorted(self.initials, key=len, reverse=True):
            if pinyin.startswith(initial):
                final = pinyin[len(initial):]
                if final:
                    return [initial, final]
                else:
                    return [initial]

        # 没有声母（零声母）
        return [pinyin]

    def chinese_to_keyword_format(self, chinese_text: str) -> str:
        """
        将中文转换为keyword格式

        Args:
            chinese_text: 中文文本，如"小米小米"

        Returns:
            keyword格式，如"x iǎo m ǐ x iǎo m ǐ @小米小米"
        """
        # 转换为带声调拼音
        pinyin_list = lazy_pinyin(chinese_text, style=Style.TONE)

        # 分割每个拼音
        split_parts = []
        missing_tokens = []

        for pinyin in pinyin_list:
            parts = self._split_pinyin(pinyin)

            # 验证每个part是否在tokens中
            for part in parts:
                if part not in self.available_tokens:
                    missing_tokens.append(part)
                split_parts.append(part)

        # 拼接结果
        pinyin_str = " ".join(split_parts)
        keyword_line = f"{pinyin_str} @{chinese_text}"

        # 如果有缺失的token，给出警告
        if missing_tokens:
            print(f"⚠️  警告: 以下token不在tokens.txt中: {', '.join(set(missing_tokens))}")
            print(f"   生成的关键词可能无法正常工作")

        return keyword_line

    def add_keyword(self, chinese_text: str, append: bool = True) -> bool:
        """
        添加唤醒词到keywords.txt

        Args:
            chinese_text: 中文唤醒词
            append: 是否追加（True）或覆盖（False）

        Returns:
            是否成功
        """
        try:
            # 生成keyword格式
            keyword_line = self.chinese_to_keyword_format(chinese_text)

            # 检查是否已存在
            if self.keywords_file.exists():
                with open(self.keywords_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if f"@{chinese_text}" in content:
                        print(f"⚠️  关键词 '{chinese_text}' 已存在")
                        return False

            # 写入文件
            mode = 'a' if append else 'w'
            with open(self.keywords_file, mode, encoding='utf-8') as f:
                f.write(keyword_line + '\n')

            print(f"✅ 成功添加: {keyword_line}")
            return True

        except Exception as e:
            print(f"❌ 添加失败: {e}")
            return False

    def batch_add_keywords(self, chinese_texts: list, overwrite: bool = False):
        """
        批量添加唤醒词

        Args:
            chinese_texts: 中文列表
            overwrite: 是否覆盖原文件
        """
        if overwrite:
            print("⚠️  将覆盖现有keywords.txt")

        success_count = 0
        for text in chinese_texts:
            text = text.strip()
            if not text:
                continue

            if self.add_keyword(text, append=not overwrite):
                success_count += 1

            # 第一个后都追加
            overwrite = False

        print(f"\n📊 完成: 成功添加 {success_count}/{len(chinese_texts)} 个关键词")

    def list_keywords(self):
        """列出当前所有关键词"""
        if not self.keywords_file.exists():
            print("⚠️  keywords.txt 不存在")
            return

        print(f"\n📄 当前关键词列表 ({self.keywords_file}):")
        print("-" * 60)

        with open(self.keywords_file, 'r', encoding='utf-8') as f:
            for i, line in enumerate(f, 1):
                line = line.strip()
                if line and not line.startswith('#'):
                    # 提取中文部分显示
                    if '@' in line:
                        pinyin_part, chinese_part = line.split('@', 1)
                        print(f"{i}. {chinese_part.strip():15s} -> {pinyin_part.strip()}")
                    else:
                        print(f"{i}. {line}")

        print("-" * 60)


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(
        description="唤醒词自动生成工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 添加单个关键词
  python keyword_generator.py -a "小米小米"

  # 批量添加关键词
  python keyword_generator.py -b "小米小米" "你好小智" "贾维斯"

  # 从文件批量导入（每行一个中文）
  python keyword_generator.py -f keywords_input.txt

  # 列出当前关键词
  python keyword_generator.py -l

  # 测试转换（不写入文件）
  python keyword_generator.py -t "小米小米"
        """
    )

    parser.add_argument(
        '-m', '--model-dir',
        default='models',
        help='模型目录路径（默认: models）'
    )

    parser.add_argument(
        '-a', '--add',
        help='添加单个关键词（中文）'
    )

    parser.add_argument(
        '-b', '--batch',
        nargs='+',
        help='批量添加关键词（多个中文，空格分隔）'
    )

    parser.add_argument(
        '-f', '--file',
        help='从文件批量导入（每行一个中文）'
    )

    parser.add_argument(
        '-l', '--list',
        action='store_true',
        help='列出当前所有关键词'
    )

    parser.add_argument(
        '-t', '--test',
        help='测试转换（不写入文件）'
    )

    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='覆盖模式（清空现有关键词）'
    )

    args = parser.parse_args()

    # 确定模型目录
    if Path(args.model_dir).is_absolute():
        model_dir = Path(args.model_dir)
    else:
        # 相对路径：相对于项目根目录
        script_dir = Path(__file__).parent
        project_root = script_dir.parent
        model_dir = project_root / args.model_dir

    if not model_dir.exists():
        print(f"❌ 模型目录不存在: {model_dir}")
        sys.exit(1)

    print(f"🔧 使用模型目录: {model_dir}")

    # 创建生成器
    generator = KeywordGenerator(model_dir)

    # 执行操作
    if args.test:
        # 测试模式
        print(f"\n🧪 测试转换:")
        keyword_line = generator.chinese_to_keyword_format(args.test)
        print(f"   输入: {args.test}")
        print(f"   输出: {keyword_line}")

    elif args.add:
        # 添加单个
        generator.add_keyword(args.add)

    elif args.batch:
        # 批量添加
        generator.batch_add_keywords(args.batch, overwrite=args.overwrite)

    elif args.file:
        # 从文件导入
        input_file = Path(args.file)
        if not input_file.exists():
            print(f"❌ 文件不存在: {input_file}")
            sys.exit(1)

        with open(input_file, 'r', encoding='utf-8') as f:
            keywords = [line.strip() for line in f if line.strip()]

        print(f"📥 从文件导入 {len(keywords)} 个关键词")
        generator.batch_add_keywords(keywords, overwrite=args.overwrite)

    elif args.list:
        # 列出关键词
        generator.list_keywords()

    else:
        # 交互模式
        print("\n🎤 唤醒词生成工具（交互模式）")
        print("输入中文唤醒词，按 Ctrl+C 或输入 'q' 退出\n")

        try:
            while True:
                chinese = input("请输入中文唤醒词: ").strip()

                if not chinese or chinese.lower() == 'q':
                    break

                generator.add_keyword(chinese)
                print()

        except KeyboardInterrupt:
            print("\n\n👋 已退出")

    # 最后列出所有关键词
    if not args.list and (args.add or args.batch or args.file):
        generator.list_keywords()


if __name__ == "__main__":
    main()
