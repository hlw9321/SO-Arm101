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
SCS 舵机总线诊断工具
用于排查串口连接问题: 自动扫描 COM 口, 多波特率尝试 ping 舵机
"""

import serial
import serial.tools.list_ports
import time
import struct

HEADER1, HEADER2 = 0xFF, 0xFF
CMD_PING = 0x01


def checksum(data: bytes) -> int:
    s = sum(data) & 0xFF
    return (~s) & 0xFF


def ping(ser, servo_id: int, timeout: float = 0.05) -> bool:
    """发送 ping 包并等待应答"""
    # 构造 ping 包: 0xFF 0xFF ID 0x02 0x01 checksum
    packet = bytes([HEADER1, HEADER2, servo_id, 0x02, CMD_PING])
    chk = checksum(packet[2:])
    packet += bytes([chk])

    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    # 等待应答: 0xFF 0xFF ID 0x02 0x00? check (至少6字节)
    deadline = time.time() + timeout
    buf = bytearray()
    while len(buf) < 6 and time.time() < deadline:
        waiting = ser.in_waiting
        if waiting:
            buf.extend(ser.read(waiting))
        else:
            time.sleep(0.001)

    if len(buf) >= 6:
        for i in range(len(buf) - 5):
            if buf[i] == 0xFF and buf[i+1] == 0xFF:
                rid = buf[i+2]
                if rid == servo_id:
                    return True
        # 有返回但不匹配, 打印原始数据
        print(f"     原始返回: {buf.hex(' ')}")
    return False


def scan_servos(port: str, baud: int):
    """对指定端口/波特率扫描舵机"""
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.5)  # 等控制板初始化

        print(f"\n  ── 扫描舵机 ID 1~12 ──")
        found = []
        for sid in range(1, 13):
            if ping(ser, sid):
                found.append(sid)
                print(f"     ✓ ID {sid} 在线")
            else:
                print(f"     ✗ ID {sid} 无响应")

        ser.close()
        if found:
            print(f"\n  ★ 找到 {len(found)} 个舵机: {found}")
            return True, found
        else:
            print(f"\n    未找到舵机")
            return False, []

    except serial.SerialException as e:
        print(f"    串口错误: {e}")
        return False, []
    except Exception as e:
        print(f"    异常: {e}")
        return False, []


def main():
    print("╔══════════════════════════════════════════╗")
    print("║   SCS 舵机总线诊断工具                    ║")
    print("╚══════════════════════════════════════════╝")

    # 列出所有串口
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("\n❌ 未检测到任何串口!")
        print("   请确保 USB 控制板已插入, 驱动已安装")
        input("\n按 Enter 退出...")
        return

    print(f"\n🔍 检测到 {len(ports)} 个串口:")
    for i, p in enumerate(ports):
        print(f"   [{i}] {p.device} - {p.description}")
        if p.hwid:
            print(f"        HWID: {p.hwid}")
        if p.manufacturer:
            print(f"        制造商: {p.manufacturer}")

    # 用户选择端口 (或自动)
    print(f"\n📌 将自动对所有端口进行扫描...")

    baud_rates = [1000000, 500000, 115200, 921600]

    all_found = {}
    for port_info in ports:
        port = port_info.device
        print(f"\n{'─'*50}")
        print(f"端口: {port} ({port_info.description})")

        for baud in baud_rates:
            print(f"\n  波特率: {baud} bps")
            ok, ids = scan_servos(port, baud)
            if ok:
                all_found[port] = (baud, ids)
                break  # 找到就下一个端口

    # 汇总
    print(f"\n{'═'*50}")
    print(f"📋 扫描结果汇总:")
    if all_found:
        for port, (baud, ids) in all_found.items():
            print(f"  ✅ {port} @ {baud} bps → 舵机: {ids}")
        print(f"\n💡 请在上位机中选择对应的串口和波特率")
        print(f"   主臂通常 ID 1-6, 从臂 ID 7-12")
    else:
        print(f"  ❌ 所有端口/波特率均未找到舵机")
        print(f"\n  可能原因:")
        print(f"  1. USB 控制板需要先给舵机供电 (接电池或电源)")
        print(f"  2. 舵机 ID 不在 1-12 范围内")
        print(f"  3. 控制板协议不是透传 SCS (可能需要专用驱动)")
        print(f"  4. 舵机数据线接触不良")
        print(f"\n  💡 你的控制板具体是什么型号? (比如 Waveshare/众灵/...)")

    input(f"\n按 Enter 退出...")


if __name__ == "__main__":
    main()
