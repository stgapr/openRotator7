#!/usr/bin/env python3
"""
专为 WitMotion WT9011DCL-BT50 定制的 BLE 转 Easycomm 网关
基于官方示例代码专注于角度输出
"""

from __future__ import annotations

import asyncio
import argparse
import time
from dataclasses import dataclass
from typing import Callable, Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.exc import BleakError

# ==================== BLE 常量 ====================
SERVICE_UUID = "0000ffe5-0000-1000-8000-00805f9a34fb"
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"
WRITE_UUID = "0000ffe9-0000-1000-8000-00805f9a34fb"

# ==================== Easycomm 常量 ====================
AZIMUTH_MIN, AZIMUTH_MAX = 0.0, 360.0
ELEVATION_MIN, ELEVATION_MAX = 0.0, 180.0


REGISTER_ROLL = 0x3A  # X轴角度 (roll) 寄存器地址，根据文档确认
REGISTER_YAW = 0x3C   # Z轴角度 (yaw) 寄存器地址，根据文档确认

def read_register_command(register: int) -> bytes:
    """生成读取指定寄存器的指令 (0xFF 0xAA 0x27 寄存器地址 0x00)"""
    return bytes((0xFF, 0xAA, 0x27, register & 0xFF, 0x00))
    
@dataclass
class AngleData:
    """仅包含我们需要的角度数据"""
    azimuth: Optional[float] = None   # 对应传感器 Z 轴
    elevation: Optional[float] = None # 对应传感器 X 轴

class WitMotionAngleParser:
    """专门解析角度数据包 (0x55 0x61) 的解析器"""
    
    def __init__(self, on_angle: Callable[[AngleData], None]):
        self._buf = bytearray()
        self._on_angle = on_angle

    def feed(self, data: bytes) -> None:
        for value in data:
            self._buf.append(value)
            # 帧同步：查找 0x55 作为帧头
            if len(self._buf) == 1 and self._buf[0] != 0x55:
                self._buf.clear()
                continue
            # 检查第二个字节是否为角度数据包标识 (0x61)
            if len(self._buf) == 2 and self._buf[1] != 0x61:
                del self._buf[0]  # 移除错误的帧头，继续搜索
                continue
            # 角度数据包总长为 20 字节 (头2字节 + 数据18字节? 实际解析时使用前20字节)
            if len(self._buf) == 20:
                self._process_angle_frame(bytes(self._buf))
                self._buf.clear()

    def _process_angle_frame(self, frame: bytes) -> None:
        """解析角度数据帧，提取 X (俯仰) 和 Z (方位) 角度"""
        # 根据示例代码：角度偏移量为 14, 16, 18
        angle_x_raw = self._i16(frame, 14)  # 绕X轴角度 -> 俯仰角 (Elevation)
        angle_y_raw = self._i16(frame, 16)  # 绕Y轴角度 -> 偏航方位角（Azimuth)
        
        # 转换为度，并保留一位小数
        elevation = round((angle_x_raw / 32768.0) * 180.0, 1)
        azimuth = round((angle_y_raw / 32768.0) * 180.0, 1)
        
        # 将角度映射到 Easycomm 规范范围
        elevation = self._map_to_range(elevation, ELEVATION_MIN, ELEVATION_MAX)
        azimuth = self._map_to_range(azimuth, AZIMUTH_MIN, AZIMUTH_MAX)
        
        self._on_angle(AngleData(azimuth=azimuth, elevation=elevation))

    @staticmethod
    def _i16(frame: bytes, offset: int) -> int:
        """将高低字节转换为有符号16位整数 (低字节在前)"""
        value = frame[offset] | (frame[offset + 1] << 8)
        return value - 0x10000 if value >= 0x8000 else value
    
    @staticmethod
    def _map_to_range(value: float, min_val: float, max_val: float) -> float:
        """将角度映射到指定范围"""
        if max_val == 360.0:
            # 方位角：处理负值和超过360度的情况
            value = value % 360.0
            return round(value, 1)
        else:  # 俯仰角 0-180
            return round(max(min_val, min(value, max_val)), 1)

class EasycommGateway:
    """Easycomm 协议网关，将角度数据转换为控制指令"""
    
    @staticmethod
    def format_command(azimuth: float, elevation: float) -> str:
        """格式化 Easycomm 指令：P AZIMUTH ELEVATION\n"""
        return f"AZ{azimuth:.1f} EL{elevation:.1f}\n"
    
    @staticmethod
    def output_command(command: str):
        """
        输出指令到目标 (目前打印到屏幕，可替换为串口/网络发送)
        """
        print(f"[Easycomm] {command.strip()}")

# ==================== 主程序 ====================

async def select_device(args: argparse.Namespace) -> BLEDevice | None:
    """扫描并选择目标设备"""
    print(f"正在扫描 BLE 设备 (超时 {args.timeout} 秒)...")
    devices = await BleakScanner.discover(timeout=args.timeout, return_adv=True)
    
    for addr, (dev, adv) in devices.items():
        name = dev.name or adv.local_name or ""
        if args.name.lower() in name.lower() or SERVICE_UUID in adv.service_uuids:
            print(f"[发现] {name} ({addr}), RSSI: {adv.rssi}")
            return dev
    print(f"未找到包含 '{args.name}' 的设备")
    return None

async def run(args: argparse.Namespace) -> int:
    # 1. 选择设备
    device = await select_device(args)
    if not device:
        return 1
    
    # 2. 定义角度回调：将数据转换为 Easycomm 指令并输出
    def on_angle(angle: AngleData):
        if angle.azimuth is not None and angle.elevation is not None:
            cmd = EasycommGateway.format_command(angle.azimuth, angle.elevation)
            EasycommGateway.output_command(cmd)
    
    parser = WitMotionAngleParser(on_angle)
    
    # 3. 连接并订阅通知
    print(f"正在连接 {device.name} ({device.address})...")
    try:
        async with BleakClient(device, timeout=15.0) as client:
            print(f"已连接，开始接收角度数据... (按 Ctrl+C 停止)")
            await client.start_notify(NOTIFY_UUID, lambda _, data: parser.feed(bytes(data)))
            
            # 保持运行，直到用户中断
            try:
                if args.duration > 0:
                    await asyncio.sleep(args.duration)
                else:
                    while True:
                        await asyncio.sleep(1)
            except asyncio.CancelledError:
                pass
            finally:
                await client.stop_notify(NOTIFY_UUID)
                
    except BleakError as e:
        print(f"[错误] 连接失败: {e}")
        return 1
    
    return 0

def main():
    parser = argparse.ArgumentParser(description="WitMotion WT9011DCL-BT50 BLE 转 Easycomm 网关")
    parser.add_argument("--name", default="WT901BLE67", help="设备名称关键字")
    parser.add_argument("--timeout", type=float, default=10.0, help="扫描超时 (秒)")
    parser.add_argument("--duration", type=float, default=0, help="运行时长 (秒)，0 表示持续运行")
    args = parser.parse_args()
    
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n用户中断，程序退出")
    except Exception as e:
        print(f"程序异常: {e}")
        return 1
    return 0

if __name__ == "__main__":
    main()