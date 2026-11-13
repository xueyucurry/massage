#!/usr/bin/env python3
"""Capture raw bytes from the 3760262997 scale serial link without sending data.

The vendor document describes a different frame format. This probe shows
observed frames and decoded fields without writing BMI or personal records.
"""

from __future__ import annotations

import argparse
import os
import select
import termios
import time
from collections import Counter
from datetime import datetime

from scale_port import configure_9600_8n1
from scale_protocol import result_frames


def summarize(captured: bytes) -> None:
    print(f"总计 {len(captured)} 字节；单字节 55 出现 {captured.count(0x55)} 次")
    short_frames = Counter()
    for index in range(len(captured) - 6):
        if captured[index:index + 2] != b"\xa5\x5a":
            continue
        frame = captured[index:index + 7]
        checksum = 0
        for value in frame[:-1]:
            checksum ^= value
        short_frames[(frame, checksum == frame[-1])] += 1
    for (frame, checksum_ok), count in short_frames.most_common(20):
        print(f"A5 5A 短帧 ×{count}，异或校验{'通过' if checksum_ok else '未通过'}：{frame.hex(' ')}")

    long_frames = Counter()
    start = 0
    while (index := captured.find(b"\xab\xba", start)) >= 0:
        next_short = captured.find(b"\xa5\x5a", index + 2)
        next_long = captured.find(b"\xab\xba", index + 2)
        ends = [position for position in (next_short, next_long) if position >= 0]
        end = min(ends) if ends else len(captured)
        long_frames[captured[index:min(end, index + 64)]] += 1
        start = index + 2
    for frame, count in long_frames.most_common(10):
        print(f"AB BA 候选帧 ×{count}，已显示前 {len(frame)} 字节：{frame.hex(' ')}")
    for measurement in result_frames(captured):
        print(
            "待核对的实测字段："
            f"年龄 {measurement.age_years} 岁、身高 {measurement.height_cm} cm、"
            f"体重 {measurement.weight_kg:.2f} kg（尚未写入 BMI）"
        )
    if not short_frames and not long_frames and captured:
        print(f"前 64 字节：{captured[:64].hex(' ')}")
    if captured:
        print(f"末 32 字节：{captured[-32:].hex(' ')}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="例如 /dev/serial/by-id/usb-Prolific_...-port0")
    parser.add_argument("--seconds", type=float, default=45.0)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error("--seconds 必须在 1–120 之间")

    try:
        fd = os.open(args.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except PermissionError:
        parser.exit(1, "无串口读取权限：当前用户需加入 dialout 组并重新登录。\n")
    try:
        previous = configure_9600_8n1(fd)
        captured = bytearray()
        deadline = time.monotonic() + args.seconds
        next_progress = time.monotonic() + 5
        print(f"正在只读采集 {args.port}，9600 8N1，最多 {args.seconds:g} 秒；请完成一次称重。结束后只显示去重摘要。", flush=True)
        try:
            while len(captured) < 65536:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                readable, _, _ = select.select([fd], [], [], min(remaining, 0.5))
                if readable:
                    chunk = os.read(fd, min(4096, 65536 - len(captured)))
                    if chunk:
                        captured.extend(chunk)
                if time.monotonic() >= next_progress:
                    print(f"{datetime.now().astimezone().isoformat(timespec='seconds')}  已接收 {len(captured)} 字节", flush=True)
                    next_progress += 5
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSANOW, previous)
        summarize(bytes(captured))
        return 0 if captured else 2
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())
