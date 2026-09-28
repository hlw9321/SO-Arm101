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
增量控制核心引擎
- 读取主臂当前位置, 计算相对于初始值的增量
- 将从臂驱动到 "从臂初始值 + 主臂增量"
- 支持软限位 / 回零 / 紧急停止
"""

import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable
from enum import Enum, auto

from scs_protocol import SCServoBus, raw_to_deg, REG_GOAL_SPEED_L


# ============================
# 关节参数
# ============================
NUM_JOINTS = 6


@dataclass
class JointConfig:
    """单关节配置"""
    name: str = ""                          # 关节名称
    master_id: int = 0                      # 主臂舵机 ID
    slave_id: int = 0                       # 从臂舵机 ID
    soft_min_deg: float = -2048.0           # 软限位最小偏移量 (舵机原始值 raw, 相对零点)
    soft_max_deg: float = 2048.0            # 软限位最大偏移量 (舵机原始值 raw, 相对零点)
    speed: int = 500                        # 运动速度 (0速控)
    reversed: bool = False                  # 从臂是否反向


# 默认 SO-ARM101 6-DOF 关节配置
# 双总线模式: 主臂和从臂各接一个控制板, ID 独立 (都是 1-6)
# 单总线模式: 两臂共用一个控制板时, 改从臂 ID 为 7-12
DEFAULT_JOINTS: list[JointConfig] = [
    JointConfig("1", 1, 1,  -2048, 2048, 500, reversed=False),
    JointConfig("2", 2, 2,  -2048, 2048, 500, reversed=False),
    JointConfig("3", 3, 3,  -2048, 2048, 500, reversed=False),
    JointConfig("4", 4, 4,  -2048, 2048, 500, reversed=False),
    JointConfig("5", 5, 5,  -2048, 2048, 500, reversed=False),
    JointConfig("6", 6, 6,  -2048, 2048, 500, reversed=False),
]


# ============================
# 控制器状态枚举
# ============================
class CtrlState(Enum):
    IDLE        = auto()   # 空闲, 未连接
    ARMED       = auto()   # 已获取初始值, 待运行
    RUNNING     = auto()   # 运行中, 增量映射激活
    PAUSED      = auto()   # 暂停 (扭矩保持, 但不更新位置)
    ESTOP       = auto()   # 紧急停止 (释放扭矩)


# ============================
# 增量控制器
# ============================

class DeltaController:
    """
    增量映射控制器

    工作流程:
      1. 上电 → 连接串口 → 使能扭矩
      2. 用户手动将两臂摆到相同姿态 → 点击 "捕获零点"
      3. 控制器读取主臂和从臂当前位置作为 θ_master_init, θ_slave_init
      4. 运行循环: θ_slave_target = θ_slave_init + (θ_master_current - θ_master_init)
      5. 点击 "回零" 回到初始姿态, 点击 "急停" 释放扭矩
    """

    def __init__(self,
                 master_bus: SCServoBus,
                 slave_bus: Optional[SCServoBus] = None,
                 joints: Optional[list[JointConfig]] = None,
                 loop_hz: float = 80.0):
        """
        Args:
            master_bus: 主臂舵机总线
            slave_bus:  从臂舵机总线 (None 表示共用同一总线, 仅 ID 不同)
            joints:     关节配置列表
            loop_hz:    控制循环频率 (默认 80Hz, 12.5ms 周期)
        """
        self.master_bus = master_bus
        self.slave_bus = slave_bus if slave_bus else master_bus  # 单总线模式
        self.joints = joints or DEFAULT_JOINTS
        self.loop_hz = loop_hz
        self.period = 1.0 / loop_hz

        # 初始基准值 (上电时读取)
        self.master_init: list[int] = [0] * NUM_JOINTS    # 主臂初始值 (raw)
        self.slave_init: list[int] = [0] * NUM_JOINTS     # 从臂初始值 (raw)

        # 当前位置缓存
        self.master_current: list[int] = [0] * NUM_JOINTS
        self.slave_current: list[int] = [0] * NUM_JOINTS
        self.slave_target: list[int] = [0] * NUM_JOINTS   # 从臂目标位置

        # 偏移校正量 (手动微调): 叠加到从臂目标位置, 供"偏移校正"面板在
        # RUNNING 状态下实时微调从臂. 控制循环每帧重算目标时加入, 不会被覆盖.
        self.slave_trim_offset: list[int] = [0] * NUM_JOINTS

        # 状态
        self._state = CtrlState.IDLE
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        # 是否向从臂写位置 (False=仅实时读取显示, True=控制从臂跟随)
        self._write_slave = False

        # 回调
        self.on_state_change: Optional[Callable[[CtrlState], None]] = None
        self.on_pose_update: Optional[Callable[[list[float], list[float]], None]] = None

        # 统计信息
        self.loop_count = 0
        self.last_loop_time = 0.0

    # ============ 属性 ============

    @property
    def state(self) -> CtrlState:
        return self._state

    @state.setter
    def state(self, new_state: CtrlState):
        if new_state != self._state:
            old = self._state
            self._state = new_state
            if self.on_state_change:
                self.on_state_change(new_state)

    @property
    def is_running(self) -> bool:
        return self._state == CtrlState.RUNNING

    # ============ 连接管理 ============

    def connect(self):
        """打开串口并尝试 ping 所有舵机"""
        if not self.master_bus.is_open:
            self.master_bus.open()

        if self.slave_bus is not self.master_bus and not self.slave_bus.is_open:
            self.slave_bus.open()

        # 扫描主臂舵机
        offline = []
        for j in self.joints:
            if not self.master_bus.ping(j.master_id):
                offline.append(f"主臂 ID{j.master_id}")
        # 扫描从臂舵机
        for j in self.joints:
            if not self.slave_bus.ping(j.slave_id):
                offline.append(f"从臂 ID{j.slave_id}")

        if offline:
            raise ConnectionError(f"舵机未响应: {', '.join(offline)}")

        self.state = CtrlState.IDLE
        return True

    def disconnect(self):
        """断开连接, 释放扭矩"""
        self.stop()
        self._release_all_torque()
        self.master_bus.close()
        if self.slave_bus is not self.master_bus:
            self.slave_bus.close()
        self.state = CtrlState.IDLE

    # ============ 核心操作 ============

    def enable_all_torque(self):
        """使能从臂扭矩, 主臂保持自由 (便于手动操作)"""
        # 从臂: 预设跟随速度 + 使能扭矩
        for j in self.joints:
            self.slave_bus.write_u16(j.slave_id, REG_GOAL_SPEED_L, 1500)
        if self.slave_bus is not self.master_bus:
            id_list = [(j.slave_id, True) for j in self.joints]
            self.slave_bus.sync_write_torque(id_list)
        else:
            id_list = [(j.slave_id, True) for j in self.joints]
            self.master_bus.sync_write_torque(id_list)

        time.sleep(0.1)

    def _release_all_torque(self):
        """释放所有舵机扭矩"""
        try:
            id_list = [(j.master_id, False) for j in self.joints]
            self.master_bus.sync_write_torque(id_list)
            id_list = [(j.slave_id, False) for j in self.joints]
            self.slave_bus.sync_write_torque(id_list)
        except Exception:
            pass  # 已断连时忽略

    def disable_all_torque(self):
        """失能所有舵机扭矩 (不改变运行状态, 仅释放扭矩)"""
        try:
            self._release_all_torque()
        except Exception:
            pass

    def set_homing_zero(self, servo_id: int, arm: str = "master") -> bool:
        """
        设置中位(homing值): 校准指定 servo_id 的主臂或从臂舵机.
        arm: "master"(主臂) 或 "slave"(从臂).
        舵机须已掰到机械中位.
        读取当前 raw 位置与当前 Homing, 计算偏差并写入校准值,
        使机械中位对应的 raw 值 = 2048.
        写 EEPROM 会关断扭矩, 需要后续重新使能.
        """
        try:
            # 找到该 ID 对应的关节, 只校准指定的主臂或从臂
            for j in self.joints:
                if arm == "master" and j.master_id == servo_id:
                    self._calibrate_one(self.master_bus, j.master_id)
                elif arm == "slave" and j.slave_id == servo_id:
                    self._calibrate_one(self.slave_bus, j.slave_id)
            self.last_error = ""
            return True
        except Exception as e:
            self.last_error = str(e)
            print(f"[错误] 设置中位校准失败: {e}")
            return False

    def _calibrate_one(self, bus, sid: int):
        """
        官方一键中位校准: 写扭矩开关寄存器 40 = 128,
        舵机内部自动把当前物理位置校正为 2048 (中位).
        内部自动处理 Homing 与位置换算, 无需手动计算, 最可靠.
        校准后重新使能扭矩并验证位置是否回到 2048.
        """
        def _read_pos_with_retry(retries: int = 5, wait: float = 0.2) -> Optional[int]:
            """读取位置, 失败时重试 (写 128 校准后舵机可能短暂无响应)"""
            for _ in range(retries):
                val = bus.read_position(sid)
                if val is not None:
                    return val
                time.sleep(wait)
            return None

        # 1. 读校准前位置 (舵机须已掰到机械中位)
        before = _read_pos_with_retry()
        # 2. 官方一键校准: 当前物理位置 → 2048
        bus.calibrate_center(sid)
        time.sleep(0.5)
        # 3. 验证: 校准后位置应接近 2048 (带重试)
        #    注: 不自动使能扭矩, 让用户后续手动使能, 避免校准操作影响运行状态
        check = _read_pos_with_retry()
        if check is None:
            raise IOError(f"校准后读取 ID{sid} 位置失败")
        if abs(check - 2048) > 100:
            print(f"[警告] 校准 ID{sid} 后位置={check}, 未对齐到 2048 (校准前={before})")

    def capture_zero(self) -> bool:
        """
        捕获零点基准
        读取主臂和从臂所有舵机的当前位置, 作为初始值
        必须在两臂摆到完全相同的物理姿态后调用
        """
        self.last_error = ""
        try:
            for i, j in enumerate(self.joints):
                # 读主臂当前位置, 带重试: 一键限位等写 EEPROM 后舵机会短暂
                # 不响应读指令, 直接读容易返回 None 误报失败.
                pos = None
                for _ in range(8):
                    pos = self.master_bus.read_position(j.master_id)
                    if pos is not None:
                        break
                    time.sleep(0.1)
                if pos is None:
                    self.last_error = f"读取主臂关节 {j.name} (ID{j.master_id}) 失败"
                    raise IOError(self.last_error)
                self.master_init[i] = pos

                pos = None
                for _ in range(8):
                    pos = self.slave_bus.read_position(j.slave_id)
                    if pos is not None:
                        break
                    time.sleep(0.1)
                if pos is None:
                    self.last_error = f"读取从臂关节 {j.name} (ID{j.slave_id}) 失败"
                    raise IOError(self.last_error)
                self.slave_init[i] = pos

            self.master_current = list(self.master_init)
            self.slave_current = list(self.slave_init)
            self.slave_target = list(self.slave_init)

            self.state = CtrlState.ARMED
            return True
        except Exception as e:
            print(f"[错误] 捕获零点失败: {e}")
            return False

    def go_home(self):
        """
        回零: 将从臂恢复到初始姿态
        在 RUNNING 状态直接设置目标; 在 PAUSED 状态也会执行
        """
        if self.state not in (CtrlState.ARMED, CtrlState.RUNNING, CtrlState.PAUSED):
            return

        id_pos = []
        for i, j in enumerate(self.joints):
            home_pos = self.slave_init[i]
            self.slave_target[i] = home_pos
            id_pos.append((j.slave_id, home_pos))
        # 同步写入回零位置 (速度已在 enable_all_torque 中预置)
        self.slave_bus.sync_write_positions(id_pos)

    # ============ 控制循环 ============

    def start(self):
        """启动后台循环: 捕获零点后即可运行, 仅实时读取显示 (不控制从臂)"""
        if self.state != CtrlState.ARMED:
            raise RuntimeError("请先捕获零点 (capture_zero)")

        self._stop_event.clear()
        self._write_slave = False
        self.state = CtrlState.ARMED
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def enable_control(self):
        """开始控制: 让从臂跟随主臂 (进入 RUNNING)"""
        if self.state != CtrlState.ARMED:
            return
        self._write_slave = True
        self.state = CtrlState.RUNNING

    def stop(self):
        """停止控制循环"""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.state not in (CtrlState.IDLE, CtrlState.ESTOP):
            self.state = CtrlState.IDLE

    def restart_loop(self, was_running: bool):
        """
        校准 Homing(会写 EEPROM 暂停循环)后恢复控制循环.
        was_running=True 则恢复 RUNNING(继续控制从臂), 否则仅恢复 ARMED(实时显示).
        """
        if self.state not in (CtrlState.IDLE, CtrlState.ESTOP):
            return
        self.state = CtrlState.ARMED
        self._stop_event.clear()
        self._write_slave = bool(was_running)
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        if was_running:
            self.state = CtrlState.RUNNING

    def pause(self):
        """暂停 (保持扭矩, 停止更新位置)"""
        if self.state == CtrlState.RUNNING:
            self._stop_event.set()
            self.state = CtrlState.PAUSED

    def resume(self):
        """恢复运行"""
        if self.state == CtrlState.PAUSED:
            self._stop_event.clear()
            self.state = CtrlState.RUNNING
            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()

    def emergency_stop(self):
        """紧急停止: 停止循环并释放所有扭矩"""
        self._stop_event.set()
        self._release_all_torque()
        self.state = CtrlState.ESTOP

    def _control_loop(self):
        """后台控制循环: 快速读取主臂 → 计算增量 → 同步写入从臂"""

        while not self._stop_event.is_set():
            t_start = time.perf_counter()

            try:
                # 1. 快速批量读取主臂所有舵机当前位置 (每个 ~1.5ms)
                for i, j in enumerate(self.joints):
                    pos = self.master_bus.read_position(j.master_id, wait_ms=1.0)
                    if pos is not None:
                        self.master_current[i] = pos

                # 2. 读取从臂当前实际位置 (用于实时显示)
                for i, j in enumerate(self.joints):
                    pos = self.slave_bus.read_position(j.slave_id, wait_ms=1.0)
                    if pos is not None:
                        self.slave_current[i] = pos

                # 3. 若已开启控制, 计算增量并映射到从臂目标, 同步写入从臂
                if self._write_slave:
                    id_pos = []
                    for i, j in enumerate(self.joints):
                        delta_raw = self.master_current[i] - self.master_init[i]
                        if j.reversed:
                            delta_raw = -delta_raw

                        target = self.slave_init[i] + delta_raw
                        # 叠加偏移校正量 (手动微调), 使 RUNNING 状态下也能实时校正从臂
                        target += self.slave_trim_offset[i]
                        # 硬件保护: 不超出舵机物理范围 0~4095
                        # (软限位已取消, 从臂可全范围跟随主臂)
                        target = max(0, min(4095, target))

                        self.slave_target[i] = target
                        id_pos.append((j.slave_id, target))

                    # 同步写入从臂 (速度已预设在 enable_all_torque 中)
                    self.slave_bus.sync_write_positions(id_pos)

                self.loop_count += 1
                self.last_loop_time = time.perf_counter() - t_start

                # 4. 回调更新 UI (原始值 raw): 主臂当前 + 从臂当前
                if self.on_pose_update:
                    self.on_pose_update(list(self.master_current), list(self.slave_current))

            except serial.SerialException as e:
                print(f"[错误] 串口通信失败: {e}")
                self.state = CtrlState.ESTOP
                self._stop_event.set()
                break
            except Exception as e:
                print(f"[警告] 控制循环异常: {e}")

            # 频率控制
            elapsed = time.perf_counter() - t_start
            sleep_time = self.period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ============ 调试信息 ============

    def get_status_dict(self) -> dict:
        """获取当前状态摘要 (供 UI 展示)"""
        return {
            "state": self._state.name,
            "loop_count": self.loop_count,
            "last_loop_ms": round(self.last_loop_time * 1000, 1),
            "master_raw": list(self.master_current),
            "slave_raw": list(self.slave_target),
            "master_deg": [round(raw_to_deg(p), 1) for p in self.master_current],
            "slave_deg": [round(raw_to_deg(p), 1) for p in self.slave_target],
            "init_master_deg": [round(raw_to_deg(p), 1) for p in self.master_init],
            "init_slave_deg": [round(raw_to_deg(p), 1) for p in self.slave_init],
        }


# ============================
# 独立测试
# ============================

if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("用法: python delta_controller.py <COM端口> [从臂COM端口]")
        print("单总线模式:    python delta_controller.py COM3")
        print("双总线模式:    python delta_controller.py COM3 COM4")
        sys.exit(1)

    master_port = sys.argv[1]
    slave_port = sys.argv[2] if len(sys.argv) > 2 else None

    print(f"主臂串口: {master_port}")
    master_bus = SCServoBus(master_port)

    slave_bus = None
    if slave_port:
        print(f"从臂串口: {slave_port}")
        slave_bus = SCServoBus(slave_port)

    ctrl = DeltaController(master_bus, slave_bus)

    def log_status(s):
        print(f"[状态变更] → {s.name}")

    def log_pose(m_deg, s_deg):
        print(f"\r主臂: {[f'{v:6.1f}' for v in m_deg]} | 从臂: {[f'{v:6.1f}' for v in s_deg]}", end="")

    ctrl.on_state_change = log_status
    ctrl.on_pose_update = log_pose

    try:
        print("\n连接舵机总线...")
        ctrl.connect()

        print("\n使能扭矩...")
        ctrl.enable_all_torque()

        print("\n请将两臂摆到相同姿态, 然后按 Enter 捕获零点...")
        input()

        print("正在读取初始位置...")
        if not ctrl.capture_zero():
            print("捕获零点失败, 退出")
            sys.exit(1)

        print(f"\n零点已捕获 (原始值 raw):")
        for i, j in enumerate(ctrl.joints):
            md = ctrl.master_init[i]
            sd = ctrl.slave_init[i]
            print(f"  {j.name}: 主={md}  从={sd}  偏差={sd - md}")

        print("\n启动增量控制 (Ctrl+C 停止)...")
        ctrl.start()

        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        ctrl.disconnect()
        print("已断开连接")
