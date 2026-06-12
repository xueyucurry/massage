#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
六维力传感器数据读取程序
通过USB转RS485模块，使用Modbus协议通信
站号: 0x09, 波特率: 115200, CRC校验: CRC16_MODBUS
"""

import serial
import serial.tools.list_ports
import struct
import time
from datetime import datetime

# ============ 配置参数 ============
BAUD_RATE = 115200
STATION = 0x09
FRAME_HEADER = bytes([0x20, 0x4E])
FRAME_LENGTH = 16  # 帧头2 + 数据12 + CRC2

# 频率 -> 寄存器值
FREQ_MAP = {
    100: 0x00,   # 100Hz
    250: 0x01,   # 250Hz
    500: 0x02,   # 500Hz
    1000: 0x03,  # 1000Hz (浮点数模式)
}


def crc16_modbus(data: bytes) -> int:
    """CRC16 MODBUS 校验计算"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_start_command(station: int, freq: int) -> bytes:
    """
    构建启动数据回传命令
    Modbus功能码 0x10 写多个寄存器, com地址 0x019A, 数量 1, 字节数 2
    """
    freq_code = FREQ_MAP[freq]
    cmd = bytes([station, 0x10, 0x01, 0x9A, 0x00, 0x01, 0x02, 0x00, freq_code])
    crc = crc16_modbus(cmd)
    return cmd + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_stop_command() -> bytes:
    """构建停止数据传输命令 (50个0xFF)"""
    return bytes([0xFF] * 50)


def parse_frame(frame: bytes) -> dict | None:
    """
    解析传感器数据帧
    格式: 20 4E Fx_L Fx_H Fy_L Fy_H Fz_L Fz_H Mx_L Mx_H My_L My_H Mz_L Mz_H CRC_L CRC_H
    数据为16位有符号整数, 力/100(N), 力矩/1000(Nm)
    """
    if len(frame) != FRAME_LENGTH or frame[:2] != FRAME_HEADER:
        return None

    # CRC校验
    crc_received = frame[14] | (frame[15] << 8)
    crc_calculated = crc16_modbus(frame[:14])
    if crc_received != crc_calculated:
        return None

    # 解析6通道16位有符号整数 (小端序: 低字节在前)
    fx, fy, fz, mx, my, mz = struct.unpack_from('<6h', frame, 2)

    return {
        'Fx': fx / 100.0,    # N
        'Fy': fy / 100.0,    # N
        'Fz': fz / 100.0,    # N
        'Mx': mx / 1000.0,   # Nm
        'My': my / 1000.0,   # Nm
        'Mz': mz / 1000.0,   # Nm
    }


def list_serial_ports():
    """列出可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到可用串口!")
        return []
    print("可用串口:")
    for p in ports:
        print(f"  {p.device} - {p.description}")
    return [p.device for p in ports]


def read_frames(ser, buffer: bytearray):
    """从缓冲区中查找并解析所有完整帧, 返回结果列表和更新后的缓冲区"""
    if ser.in_waiting > 0:
        buffer.extend(ser.read(ser.in_waiting))

    results = []
    while len(buffer) >= FRAME_LENGTH:
        # 查找帧头 0x20 0x4E
        idx = buffer.find(FRAME_HEADER)
        if idx == -1:
            buffer = buffer[-1:]  # 保留最后1字节(可能是不完整帧头)
            break
        if idx > 0:
            buffer = buffer[idx:]  # 丢弃帧头前的数据
        if len(buffer) < FRAME_LENGTH:
            break

        frame = bytes(buffer[:FRAME_LENGTH])
        result = parse_frame(frame)
        if result:
            results.append(result)
            buffer = buffer[FRAME_LENGTH:]
        else:
            buffer = buffer[1:]  # CRC失败, 跳过1字节继续搜索

    return results, buffer


def main():
    print("=" * 60)
    print("        六维力传感器数据读取程序")
    print(f"        站号: 0x{STATION:02X}  波特率: {BAUD_RATE}")
    print("=" * 60)

    # 列出可用串口
    available = list_serial_ports()

    # 选择串口
    default_port = available[0] if available else 'COM3'
    port = input(f"\n请输入串口号 (默认 {default_port}): ").strip()
    if not port:
        port = default_port
    elif port.isdigit():
        port = f"COM{port}"

    # 选择频率
    print("\n回传频率选项: 100Hz / 250Hz / 500Hz / 1000Hz")
    freq_str = input("请输入回传频率 (默认 100): ").strip()
    freq = int(freq_str) if freq_str else 100
    if freq not in FREQ_MAP:
        print(f"不支持的频率 {freq}Hz, 可选: {list(FREQ_MAP.keys())}")
        return

    # 打开串口
    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
    except serial.SerialException as e:
        print(f"打开串口失败: {e}")
        return

    print(f"\n串口 {port} 已打开")

    # 先发送停止命令, 确保传感器处于空闲状态
    ser.write(build_stop_command())
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 构建启动命令
    cmd = build_start_command(STATION, freq)
    cmd_hex = ' '.join(f'{b:02X}' for b in cmd)

    # 发送第一次启动命令
    send_count = 1
    print(f"[第{send_count}次] 发送启动命令 ({freq}Hz): {cmd_hex}")
    ser.write(cmd)

    print(f"\n等待传感器响应, 无数据时每1s重发, Ctrl+C 停止\n")
    header = f"{'时间':>15} {'Fx(N)':>10} {'Fy(N)':>10} {'Fz(N)':>10} {'Mx(Nm)':>10} {'My(Nm)':>10} {'Mz(Nm)':>10}"
    print(header)
    print("-" * len(header))

    buffer = bytearray()
    count = 0
    t_start = time.time()
    last_send_time = time.time()
    data_started = False  # 是否已收到过有效数据

    try:
        while True:
            results, buffer = read_frames(ser, buffer)
            if results:
                if not data_started:
                    elapsed_to_first = time.time() - t_start
                    print(f"[已连接] 共发送 {send_count} 次命令, "
                          f"耗时 {elapsed_to_first:.1f}s 收到首帧数据\n")
                    print(header)
                    print("-" * len(header))
                    data_started = True

                for r in results:
                    count += 1
                    if freq <= 100 or count % (freq // 100) == 0:
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        print(f"{ts:>15} {r['Fx']:>10.2f} {r['Fy']:>10.2f} {r['Fz']:>10.2f} "
                              f"{r['Mx']:>10.3f} {r['My']:>10.3f} {r['Mz']:>10.3f}")
            else:
                # 未收到有效数据, 每1s重发启动命令
                now = time.time()
                if not data_started and (now - last_send_time) >= 1.0:
                    send_count += 1
                    total_elapsed = now - t_start
                    print(f"[第{send_count}次] 重发启动命令 (已等待 {total_elapsed:.1f}s)")
                    ser.write(cmd)
                    last_send_time = now
                time.sleep(0.001)

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        print(f"\n\n已停止, 共发送 {send_count} 次命令, 总用时 {elapsed:.1f}s")
        if count > 0:
            print(f"共接收 {count} 帧, 平均 {count / elapsed:.1f} 帧/s")
        else:
            print("未收到有效数据")

        # 发送停止命令
        stop_cmd = build_stop_command()
        ser.write(stop_cmd)
        time.sleep(0.1)
        ser.reset_input_buffer()
        print("已发送停止命令")

    finally:
        ser.close()
        print("串口已关闭")


if __name__ == '__main__':
    main()
