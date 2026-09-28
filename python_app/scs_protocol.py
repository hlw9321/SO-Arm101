# Copyright 2026 SO-ARM101 Control Suite contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SCS 总线舵机串口协议层
支持 STS3215 / SCSCL 系列舵机, 半双工 UART 通信
"""

import struct
import time
import serial
from dataclasses import dataclass
from typing import Optional

# ============================
# 协议常量
# ============================
HEADER1 = 0xFF
HEADER2 = 0xFF
BROADCAST_ID = 0xFE

CMD_PING       = 0x01
CMD_READ       = 0x02
CMD_WRITE      = 0x03
CMD_REG_WRITE  = 0x04
CMD_ACTION     = 0x05
CMD_RESET      = 0x06
CMD_SYNC_WRITE = 0x83

# STS3215 寄存器地址
REG_ID             = 5
REG_BAUD           = 6
REG_RETURN_DELAY   = 7
REG_MIN_ANGLE_L    = 9
REG_MAX_ANGLE_L    = 11
REG_HOMING_OFFSET  = 31     # 零点偏移校准值 (int12 带符号)
REG_MAX_TEMP       = 13
REG_MAX_VOLT       = 14
REG_MIN_VOLT       = 16
REG_EEPROM_LOCK    = 55     # 写 0 解锁 EEPROM
REG_TORQUE_LIMIT_L = 38
REG_TORQUE_ENABLE  = 40
REG_LED            = 41
REG_GOAL_POS_L     = 42
REG_GOAL_TIME_L    = 44
REG_GOAL_SPEED_L   = 46
REG_PRESENT_POS_L  = 56
REG_PRESENT_SPEED_L = 58
REG_PRESENT_LOAD_L = 60
REG_PRESENT_VOLT   = 62
REG_PRESENT_TEMP   = 63
REG_MOVING         = 66
REG_PRESENT_CURRENT_L = 69

# STS3215 参数
RAW_RESOLUTION = 4096      # 12-bit 分辨率
RAW_DEG_MAX    = 300.0     # STS3215 可用角度范围 0-300°

# ============================
# 工具函数
# ============================

def checksum(data: bytes) -> int:
    """计算 SCS 协议校验和: 对除帧头外的所有字节取累加和低8位取反"""
    s = sum(data) & 0xFF
    return (~s) & 0xFF


def deg_to_raw(deg: float) -> int:
    """角度(度) → 原始位置值(0-4095)"""
    val = int(deg * RAW_RESOLUTION / RAW_DEG_MAX)
    return max(0, min(RAW_RESOLUTION - 1, val))


def raw_to_deg(raw: int) -> float:
    """原始位置值 → 角度(度)"""
    return raw * RAW_DEG_MAX / RAW_RESOLUTION


# ============================
# 数据类
# ============================

@dataclass
class ServoState:
    """单舵机实时状态"""
    position: int = 0       # 原始位置 0-4095
    speed: int = 0          # 当前速度
    load: int = 0           # 当前负载
    voltage: int = 0        # 电压 (0.1V 单位)
    temperature: int = 0    # 温度 (℃)
    moving: bool = False    # 是否运动中

    @property
    def angle_deg(self) -> float:
        return raw_to_deg(self.position)


# ============================
# SCS 总线管理类
# ============================

class SCServoBus:
    """
    半双工 SCS 总线舵机驱动器
    通过 USB 转 TTL 模块 (如 CH340/CP2102) 的 TX 接舵机数据线,
    需要注意硬件电平匹配 (3.3V / 5V TTL)
    """

    def __init__(self, port: str, baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self._ser: Optional["serial.Serial"] = None

    # ---- 连接管理 ----

    def open(self):
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,           # 50ms 超时
            )
        except serial.SerialException as e:
            raise IOError(
                f"串口 {self.port} 打开失败: {e}\n"
                f"→ 请检查设备是否已连接，或在设备管理器中禁用/启用以重置驱动。"
            ) from e
        # 禁用 DTR/RTS, 防止脉冲复位控制板
        self._ser.dtr = False
        self._ser.rts = False
        # CH343/CH340 等需要短暂初始化时间
        time.sleep(0.3)
        # 清空开机垃圾数据
        self._ser.reset_input_buffer()
        if not self._ser.is_open:
            raise IOError(f"无法打开串口 {self.port}")

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ---- 底层帧收发 ----

    def _send_packet(self, servo_id: int, cmd: int, params: bytes = b""):
        """组装并发送一帧指令包
        SCS 协议帧格式:
          0xFF 0xFF ID LENGTH CMD [PARAMS...] CHECKSUM
          其中 LENGTH = CMD(1) + len(PARAMS) + 1(checksum位置)
          例如 PING(无参数): LENGTH = 1 + 0 + 1 = 2
        """
        length = len(params) + 2   # CMD(1) + PARAMS + 1
        packet = bytearray()
        packet.append(HEADER1)
        packet.append(HEADER2)
        packet.append(servo_id)
        packet.append(length)
        packet.append(cmd)
        packet.extend(params)
        packet.append(checksum(packet[2:]))
        self._ser.write(bytes(packet))
        self._ser.flush()
        # 等待舵机处理 (STM32 内核处理约 200~500us)
        time.sleep(0.0005)

    def _read_response(self, servo_id: int, cmd: int, param_len: int = 0) -> Optional[bytes]:
        """读取并解析应答包, 返回 params 部分; 失败返回 None
        应答帧: FF FF | ID | LEN | ERR | [PARAMS...] | CHK
               0  1    2    3     4     5...          ?
        LEN 从 ERR 开始计数, 包含 ERR + PARAMS + CHK
        最小帧: FF FF ID 02 ERR CHK = 6 字节
        """
        buf = bytearray()

        deadline = time.time() + 0.10
        min_len = 6
        while time.time() < deadline:
            if self._ser.in_waiting:
                buf.extend(self._ser.read(self._ser.in_waiting))
                if len(buf) >= min_len:
                    break
            else:
                time.sleep(0.0001)

        if len(buf) < min_len:
            return None

        for i in range(len(buf) - 5):
            if buf[i] != HEADER1 or buf[i + 1] != HEADER2:
                continue
            rid = buf[i + 2]
            rlen = buf[i + 3]         # LEN = ERR(1) + params + CHK(1)
            if rid != servo_id or rlen < 2:
                continue
            # 整帧长度: FF FF(2) + ID(1) + LEN(1) + rlen 字节
            total_frame = 4 + rlen    # 2 + 1 + 1 + rlen
            if i + total_frame > len(buf):
                continue
            # CHK 是 rlen 区域的最后一个字节
            chk_pos = i + 3 + rlen    # FF FF ID LEN + rlen个字节, 最后一字节就是CHK
            rerr = buf[i + 4]         # rlen 区域的第0字节 = ERR
            calc = checksum(buf[i + 2 : chk_pos])  # 从ID到CHK之前
            if buf[chk_pos] != calc:
                continue
            # params = ERR之后, CHK之前
            params_start = i + 5      # FF FF ID LEN ERR = 5 字节后
            params_end = chk_pos      # CHK 位置
            return bytes(buf[params_start : params_end])
        return None

    # ---- 高层命令 ----

    def ping(self, servo_id: int) -> bool:
        """检测舵机是否在线, 失败时重试一次"""
        for attempt in range(2):
            self._send_packet(servo_id, CMD_PING)
            time.sleep(0.005)
            resp = self._read_response(servo_id, CMD_PING)
            if resp is not None:
                return True
            if attempt == 0:
                time.sleep(0.01)  # 第一次失败等 10ms 再试
        return False

    def read_bytes(self, servo_id: int, reg_addr: int, length: int,
                   wait_ms: float = 1.0) -> Optional[bytes]:
        """读取连续寄存器, wait_ms 控制发收间隔 (默认 1ms)"""
        params = bytes([reg_addr, length])
        self._send_packet(servo_id, CMD_READ, params)
        time.sleep(wait_ms / 1000.0)
        resp = self._read_response(servo_id, CMD_READ, length)
        if resp is None or len(resp) < length:
            return None
        return resp[:length]

    def read_position(self, servo_id: int, wait_ms: float = 1.0) -> Optional[int]:
        return self.read_u16(servo_id, REG_PRESENT_POS_L, wait_ms=wait_ms)

    def read_homing_offset(self, servo_id: int, wait_ms: float = 1.0) -> Optional[int]:
        """读取舵机位置校正值 (地址31, sign-magnitude 编码, 与 lerobot 一致)

        lerobot 用 sign-magnitude (encode_sign_magnitude, sign_bit=11):
            - BIT11 为方向位 (1=负, 0=正)
            - 低 11 位为幅值 (0~2047)
        注意: 不是二进制补码, 不能用 &0x0800 判断负数后直接减 0x1000.
        高 4 位为无效位 (可能是残留值), 忽略."""
        val = self.read_u16(servo_id, REG_HOMING_OFFSET, wait_ms=wait_ms)
        if val is None:
            return None
        # 只取低 12 位, 按 sign-magnitude 解码
        val &= 0x0FFF
        direction = (val >> 11) & 1
        magnitude = val & 0x7FF
        return -magnitude if direction else magnitude

    def read_position_limits(self, servo_id: int,
                             wait_ms: float = 1.0) -> Optional[tuple[int, int]]:
        """读取舵机位置限位 (min, max), 地址9/11 (uint16)"""
        min_val = self.read_u16(servo_id, REG_MIN_ANGLE_L, wait_ms=wait_ms)
        max_val = self.read_u16(servo_id, REG_MAX_ANGLE_L, wait_ms=wait_ms)
        if min_val is None or max_val is None:
            return None
        return (min_val, max_val)

    def write_position_limits(self, servo_id: int, min_val: int, max_val: int,
                              unlock_need: bool = True):
        """
        写入舵机位置限位 (min, max), 地址9/11 (uint16).
        写 EEPROM 前需关扭矩 + 解锁, 写后锁定. 调用后扭矩将关断.
        """
        if unlock_need:
            self.write_u8(servo_id, REG_TORQUE_ENABLE, 0)
            time.sleep(0.02)
            self.write_u8(servo_id, REG_EEPROM_LOCK, 0)      # 解锁 EEPROM
            time.sleep(0.02)
        # 保证 min <= max
        lo = min(min_val, max_val)
        hi = max(min_val, max_val)
        lo = max(0, min(4095, lo))
        hi = max(0, min(4095, hi))
        self.write_u16(servo_id, REG_MIN_ANGLE_L, lo)
        time.sleep(0.02)
        self.write_u16(servo_id, REG_MAX_ANGLE_L, hi)
        time.sleep(0.02)
        if unlock_need:
            self.write_u8(servo_id, REG_EEPROM_LOCK, 1)       # 锁定 EEPROM
            time.sleep(0.02)

    def read_u16(self, servo_id: int, reg_addr: int, wait_ms: float = 1.0) -> Optional[int]:
        data = self.read_bytes(servo_id, reg_addr, 2, wait_ms=wait_ms)
        return struct.unpack("<H", data)[0] if data and len(data) == 2 else None

    def read_u8(self, servo_id: int, reg_addr: int, wait_ms: float = 1.0) -> Optional[int]:
        data = self.read_bytes(servo_id, reg_addr, 1, wait_ms=wait_ms)
        return data[0] if data else None

    def write_u8(self, servo_id: int, reg_addr: int, value: int):
        params = bytes([reg_addr, value])
        self._send_packet(servo_id, CMD_WRITE, params)

    def write_u16(self, servo_id: int, reg_addr: int, value: int):
        lo = value & 0xFF
        hi = (value >> 8) & 0xFF
        params = bytes([reg_addr, lo, hi])
        self._send_packet(servo_id, CMD_WRITE, params)

    # ---- 舵机控制快捷方法 ----

    def set_torque(self, servo_id: int, enable: bool):
        """使能 / 释放扭矩"""
        self.write_u8(servo_id, REG_TORQUE_ENABLE, 1 if enable else 0)

    def calibrate_center(self, servo_id: int):
        """官方一键中位校准: 写扭矩开关寄存器 40 = 128,
        舵机内部自动把当前物理位置校正为 2048 (中位).
        无需手动计算 Homing 偏移, 最可靠. 调用后需重新使能扭矩."""
        self.write_u8(servo_id, REG_TORQUE_ENABLE, 128)

    def set_position(self, servo_id: int, position_raw: int, speed: int = 0):
        """设置目标位置 (原始值 0-4095)"""
        if speed > 0:
            self.write_u16(servo_id, REG_GOAL_SPEED_L, speed)
        self.write_u16(servo_id, REG_GOAL_POS_L, position_raw)

    def configure_full_angle_range(self, servo_ids: list[int]):
        """
        将舵机硬件角度限制设为全范围 (0-4095),
        先解锁 EEPROM → 写 MIN/MAX → 锁定.
        调用后扭矩将关断, 需要后续重新使能.
        """
        for sid in servo_ids:
            self.write_u8(sid, REG_TORQUE_ENABLE, 0)
            time.sleep(0.02)
            self.write_u8(sid, REG_EEPROM_LOCK, 0)      # 解锁 EEPROM
            time.sleep(0.02)
            self.write_u16(sid, REG_MIN_ANGLE_L, 0)      # 最小 = 0
            time.sleep(0.02)
            self.write_u16(sid, REG_MAX_ANGLE_L, 4095)   # 最大 = 4095
            time.sleep(0.02)
            self.write_u8(sid, REG_EEPROM_LOCK, 1)       # 锁定 EEPROM
            time.sleep(0.02)

    def write_homing_offset(self, servo_id: int, offset: int,
                            unlock_need: bool = True):
        """
        写入舵机位置校正值 Homing_Offset (地址31, sign-magnitude 编码, 与 lerobot 一致)
        写 NVS 前需关扭矩 + 解锁, 写后锁定.
        调用后扭矩将关断, 需要后续重新使能.
        """
        if unlock_need:
            self.write_u8(servo_id, REG_TORQUE_ENABLE, 0)
            time.sleep(0.02)
            self.write_u8(servo_id, REG_EEPROM_LOCK, 0)      # 解锁
            time.sleep(0.02)
        # sign-magnitude 编码 (与 lerobot encode_sign_magnitude 一致):
        #   BIT11 方向位 (1=负), 低 11 位幅值, 范围 -2047~2047.
        # 高 4 位无效, 不写入 (保持为 0).
        offset = max(-2047, min(2047, offset))
        if offset < 0:
            val = (1 << 11) | (-offset)   # 方向位1 + 幅值
        else:
            val = offset
        self.write_u16(servo_id, REG_HOMING_OFFSET, val)
        time.sleep(0.02)
        if unlock_need:
            self.write_u8(servo_id, REG_EEPROM_LOCK, 1)       # 锁定 EEPROM
            time.sleep(0.02)

    def read_state(self, servo_id: int) -> Optional[ServoState]:
        """批量读取舵机完整状态 (9字节)"""
        data = self.read_bytes(servo_id, REG_PRESENT_POS_L, 9)
        if data is None or len(data) < 9:
            return None
        pos = data[0] | (data[1] << 8)
        spd = data[2] | (data[3] << 8)
        # STS3215 有符号速度
        if spd > 32767:
            spd -= 65536
        load = data[4] | (data[5] << 8)
        if load > 32767:
            load -= 65536
        volt = data[6]
        temp = data[7]
        moving = data[8] != 0
        return ServoState(
            position=pos, speed=spd, load=load,
            voltage=volt, temperature=temp, moving=moving
        )

    def set_torque_limit(self, servo_id: int, limit: int):
        """设置扭矩限制百分比 (0-1000, 对应 0-100.0%)"""
        lim = max(0, min(1000, limit))
        self.write_u16(servo_id, REG_TORQUE_LIMIT_L, lim)

    # ---- 同步写 (多舵机同时执行) ----

    def sync_write_positions(self, id_pos_list: list[tuple[int, int]],
                              move_time_ms: int = 0, speed: int = 0):
        """
        同步写入多舵机目标位置 (时间同步或速度同步)
        id_pos_list: [(servo_id, position_raw), ...]
        move_time_ms: >0 时使用时间控制 (所有舵机同时到达), 0 时使用速度控制
        speed: 仅在 move_time_ms=0 时生效, 写入速度寄存器
        """
        if not id_pos_list:
            return

        if move_time_ms > 0:
            # 时间同步: 从 REG_GOAL_POS_L(42) 连续写 4 字节: pos(2) + time(2)
            move_time = max(10, min(10000, move_time_ms))
            params = bytearray()
            params.append(REG_GOAL_POS_L)   # 起始地址 = 位置寄存器 (42)
            params.append(4)                 # 每舵机 4 字节: pos_lo, pos_hi, time_lo, time_hi
            for sid, pos in id_pos_list:
                params.append(sid)
                params.append(pos & 0xFF)
                params.append((pos >> 8) & 0xFF)
                params.append(move_time & 0xFF)
                params.append((move_time >> 8) & 0xFF)
            self._send_packet(BROADCAST_ID, CMD_SYNC_WRITE, bytes(params))
        else:
            # 速度同步模式: 可同时写入速度 + 位置
            if speed > 0:
                # 先同步写速度
                sp_params = bytearray()
                sp_params.append(REG_GOAL_SPEED_L)
                sp_params.append(2)
                for sid, _pos in id_pos_list:
                    sp_params.append(sid)
                    sp_params.append(speed & 0xFF)
                    sp_params.append((speed >> 8) & 0xFF)
                self._send_packet(BROADCAST_ID, CMD_SYNC_WRITE, bytes(sp_params))
                time.sleep(0.001)
            # 再同步写位置
            params = bytearray()
            params.append(REG_GOAL_POS_L)
            params.append(2)
            for sid, pos in id_pos_list:
                params.append(sid)
                params.append(pos & 0xFF)
                params.append((pos >> 8) & 0xFF)
            self._send_packet(BROADCAST_ID, CMD_SYNC_WRITE, bytes(params))

        time.sleep(0.001)

    def sync_write_torque(self, id_enable_list: list[tuple[int, bool]]):
        """同步使能/释放多舵机扭矩"""
        if not id_enable_list:
            return
        params = bytearray()
        params.append(REG_TORQUE_ENABLE)
        params.append(1)
        for sid, en in id_enable_list:
            params.append(sid)
            params.append(1 if en else 0)
        self._send_packet(BROADCAST_ID, CMD_SYNC_WRITE, bytes(params))
        time.sleep(0.001)


# ============================
# 自检
# ============================

if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("用法: python scs_protocol.py <COM端口>")
        print("示例: python scs_protocol.py COM3")
        sys.exit(1)

    bus = SCServoBus(sys.argv[1])
    bus.open()
    print(f"[OK] 串口 {sys.argv[1]} 已打开")

    print("\n扫描舵机 ID 1-12 ...")
    for sid in range(1, 13):
        if bus.ping(sid):
            pos = bus.read_position(sid)
            deg = raw_to_deg(pos) if pos else -1
            print(f"  ID {sid:2d}: 在线, 当前位置 = {pos} raw ({deg:.1f}°)")
        else:
            print(f"  ID {sid:2d}: 离线")

    bus.close()
    print("\n[OK] 测试完成")
