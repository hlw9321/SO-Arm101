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
SO-ARM101 增量控制测试上位机
基于 PySide6, 直接通过 USB 转串口控制 STS3215 总线舵机

工作流程:
  1. 选择串口 → 连接
  2. 使能扭矩 → 手动将两臂摆到相同姿态
  3. 点击"捕获零点"记录基准
  4. 点击"运行"开始增量映射
  5. 主臂运动实时同步到从臂
"""

from __future__ import annotations

import sys
import time
import re
import subprocess
import json
import os
import threading
from pathlib import Path

# 将脚本所在目录加入 sys.path, 保证 scs_protocol / delta_controller 这两个
# 本地模块能被可靠导入. 否则当 main_app.py 被作为模块导入或经 PyInstaller
# 打包 (运行目录与脚本目录不同) 时, 隐式相对导入会报 ModuleNotFoundError,
# 导致在任意电脑上启动失败.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QSpinBox, QDoubleSpinBox,
    QDialog, QDialogButtonBox, QCheckBox, QStatusBar, QMessageBox,
    QFrame, QSplitter,
    QLineEdit, QProgressBar, QTextEdit, QGridLayout, QSizePolicy,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QSlider, QRadioButton,
)
from PySide6.QtCore import (
    Qt, QTimer, Signal, Slot, QThread, QObject,
    QPoint,
)
from PySide6.QtGui import QColor, QFont, QPalette, QIcon

from scs_protocol import SCServoBus
from delta_controller import (
    DeltaController, CtrlState,
    DEFAULT_JOINTS, NUM_JOINTS
)

# ============================
# 串口工具函数
# ============================

def list_serial_ports() -> list[str]:
    """列出可用串口"""
    import serial.tools.list_ports as lp
    ports = lp.comports()
    return [p.device for p in ports]


def get_port_descriptions() -> dict[str, str]:
    """获取串口描述 {port: description}"""
    import serial.tools.list_ports as lp
    return {p.device: p.description for p in lp.comports()}


# ============================
# 虚拟环境全盘搜索 (后台线程执行, 不卡 UI)
# ============================

# 全盘活扫描时剪枝的目录名 (小写): 这些目录几乎不可能含 conda/venv 环境, 且
# 体积极大/极慢, 直接跳过可大幅缩短扫描时间并避免扫描到一半卡死.
# 注: Application Data / Local Settings 是用户目录下的联接点 (junction), 在部分
# Python 版本下 is_symlink() 返回 False, 会"绕回" AppData 造成重复/循环遍历, 故显式跳过.
_ENV_SCAN_SKIP = {
    "$recycle.bin", "system volume information", "windows", "winsxs",
    "node_modules", "$windows.~bt", "$windows.~ws", "recovery",
    "boot", "efi", "documents and settings",
    "application data", "local settings",
}


def _local_fixed_drives() -> list[str]:
    """返回所有本地固定盘根路径 (如 ['C:\\', 'D:\\']).

    仅 DRIVE_FIXED (类型 3), 跳过可移动/网络/光驱/RAM, 避免在 U 盘或网络盘上
    全盘漫游导致长时间卡顿.
    """
    drives = []
    try:
        import ctypes
        buf = ctypes.create_unicode_buffer(256)
        if ctypes.windll.kernel32.GetLogicalDriveStringsW(256, buf):
            for d in buf.value.split("\x00"):
                d = d.strip()
                if not d:
                    continue
                try:
                    t = ctypes.windll.kernel32.GetDriveTypeW(d)
                except Exception:
                    t = 0
                if t == 3:  # DRIVE_FIXED
                    drives.append(d)
        if drives:
            return drives
    except Exception:
        pass
    return ["C:\\"]


def _is_lerobot_env(env: str, log=None) -> bool:
    """判断 env 目录是否为可用的 lerobot 虚拟环境 (conda/venv 均可, 兼容多种安装方式).

    判定: 含 python 可执行, 且该 python 能真正 import lerobot.
    兼容以下安装形态, 不再要求 site-packages/lerobot 必须是目录:
      - PyPI 常规安装: site-packages/lerobot 为目录;
      - 可编辑安装 (pip install -e .): site-packages 下仅 lerobot.egg-link / .pth
        指向源码, 无 lerobot 目录 —— 这是"手动安装"常见的漏判点;
      - 装在 base 环境 / 自定义 envs_dirs / -p 前缀等非常规布局。
    流程: 先快速确认有 python.exe (无则必为源码克隆, 直接 False, 不进 subprocess);
    再快速查 site-packages/lerobot 目录命中即返回; 否则用该环境 python 真正 import
    兜底验证 (覆盖可编辑/非常规布局, 且能区分源码克隆与真实环境)。
    log(msg, level): 可选的日志回调, 用于输出"为什么不是环境"的诊断信息。
    """
    def _log(msg, level="info"):
        if log:
            try:
                log(msg, level)
            except Exception:
                pass

    if not os.path.isdir(env):
        return False
    py = (os.path.join(env, "Scripts", "python.exe")
          if os.name == "nt" else os.path.join(env, "bin", "python"))
    if not os.path.exists(py):
        # 没有独立 python: 几乎必为源码克隆 (如把仓库直接 git clone 到 envs/lerobot)
        _log(f"[venv诊断] {env}: 无 {py}, 视为非 python 环境(疑似源码克隆)", "warn")
        return False  # 没有独立 python -> 必为源码克隆, 直接排除

    # 1) 快速命中: site-packages/lerobot 为目录 (覆盖绝大多数常规安装)
    sp = _site_packages_of(env)
    if sp and os.path.isdir(os.path.join(sp, "lerobot")):
        return True

    # 2) 兜底: 用该环境 python 真实 import 验证 (覆盖可编辑安装 / 非常规布局)
    try:
        r = subprocess.run(
            [py, "-c", "import lerobot"],
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=30,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        if r.returncode == 0:
            return True
        # import 失败: 打印原因, 便于判断是"装了但缺依赖"还是"根本没装"
        err = (r.stderr or r.stdout or "").strip().splitlines()
        err = " | ".join(err[-6:])[:400]
        _log(f"[venv诊断] {env}: python 能跑但 import lerobot 失败 -> {err}", "warn")
        return False
    except Exception as ex:
        _log(f"[venv诊断] {env}: 执行 import 验证异常 -> {ex}", "warn")
        return False


def _site_packages_of(env: str) -> str:
    """返回 env 的 site-packages 目录路径 (Windows/Linux/macOS), 不存在返回空串."""
    if os.name == "nt":
        p = os.path.join(env, "Lib", "site-packages")
        return p if os.path.isdir(p) else ""
    sp_root = os.path.join(env, "lib")
    if not os.path.isdir(sp_root):
        return ""
    try:
        for sub in os.listdir(sp_root):
            if sub.startswith("python") and os.path.isdir(
                    os.path.join(sp_root, sub, "site-packages")):
                return os.path.join(sp_root, sub, "site-packages")
    except Exception:
        pass
    return ""


def _lerobot_base_python(root: str) -> str:
    """若 conda 根目录的 base 环境已安装 lerobot, 返回其 python 路径; 否则返回空串.

    手动安装最常见的坑: 忘了新建/激活 lerobot 环境, 直接把 lerobot 装进了 base.
    此函数让扫描也能识别这种"装在 base"的情况 (base 的 python 在 <根>/python.exe,
    而非 <根>/Scripts/python.exe, 故单独处理).
    """
    if not os.path.isdir(root):
        return ""
    py = (os.path.join(root, "python.exe")
          if os.name == "nt" else os.path.join(root, "bin", "python"))
    if not os.path.exists(py):
        return ""
    sp = _site_packages_of(root)
    if sp and os.path.isdir(os.path.join(sp, "lerobot")):
        return py
    try:
        r = subprocess.run(
            [py, "-c", "import lerobot"],
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=30,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        return py if r.returncode == 0 else ""
    except Exception:
        return ""


def _scan_for_lerobot_env(candidate_dirs: list[str], on_progress=None, on_log=None,
                          timeout: int = 180, max_depth: int = 9,
                          max_dirs: int = 400000) -> str:
    """全盘搜索名为 lerobot 的虚拟环境, 返回完整路径, 未找到返回空串.

    主线程勿直接调用 (耗时), 应放在 _EnvSearchWorker 后台线程. 分三层, 越靠前越准越快:
      0) 始终把 C:\\Users\\<用户>\\miniconda3 等用户显式扫描地址并入候选;
      1) 快速查候选 conda 根下 <根>/envs/lerobot (秒级, 覆盖绝大多数情况, 含
         AppData\\Local\\miniconda3 等);
      2) 浅优先: 仅扫各本地固定盘 (C:/D:/...) 深度<=2、且名为 miniconda*/anaconda*/
         miniforge* 的安装目录, 极快命中 C:\\miniconda3、C:\\Users\\<用户>\\miniconda3、
         D:\\miniconda3 等"浅地址";
      3) 全深兜底: 对固定盘做有界深度 DFS, 找任意名为 lerobot 且含 lerobot 包的目录
         (覆盖自定义 envs_dirs / -p 前缀等非常规位置).
    全程受 timeout / max_depth / max_dirs 三重保护, 不会无限卡。
    on_progress(msg) 用于回报进度 (如扫描了多少目录), 可为 None。
    on_log(msg, level) 用于输出扫描过程/失败原因到日志, 可为 None。
    """
    name = "lerobot"
    name_low = name.lower()
    prof = os.environ.get("USERPROFILE") or os.path.expanduser("~")
    local = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~/AppData/Local")

    def _log(msg, level="info"):
        if on_log:
            try:
                on_log(msg, level)
            except Exception:
                pass

    # 统计 (用于扫描失败时给出原因)
    n_cand = 0          # 检查的候选 conda 根数
    n_conda = 0         # 浅扫检查的 conda 安装目录数
    n_lerobot_dir = 0   # 深扫发现名为 lerobot 的目录数
    n_lerobot_bad = 0   # 其中非 python 环境 (疑似源码克隆) 的数量

    def _fail():
        bad = (f", 其中 {n_lerobot_bad} 个非 python 环境(疑似源码克隆)"
               if n_lerobot_bad else "")
        _log(
            "扫描失败: 全盘未找到 lerobot 虚拟环境。\n"
            f"  已检查候选 conda 根 {n_cand} 个 / 浅扫 conda 安装 {n_conda} 个 / "
            f"深扫目录 {scanned} 个; 发现疑似 lerobot 目录 {n_lerobot_dir} 个{bad}。\n"
            "  请确认已运行 env_setup 安装程序, 或环境不在扫描范围内。",
            "error")
        return ""

    # 0) 候选 conda 根 + 用户显式扫描地址 (C:\Users\<用户>\miniconda3 等)
    extra = []
    for v in (prof, local):
        for nm in ("miniconda3", "Miniconda3", "miniconda2", "Miniconda2",
                   "anaconda3", "Anaconda3", "miniforge3", "Miniforge3"):
            extra.append(os.path.join(v, nm))
    cand = list(candidate_dirs)
    seen = set(os.path.normcase(os.path.normpath(c)) for c in cand)
    for c in extra:
        nc = os.path.normpath(c)
        if os.path.normcase(nc) not in seen:
            seen.add(os.path.normcase(nc))
            cand.append(nc)

    # 1) 候选 conda 根快速查 (秒级)
    for root in cand:
        if not os.path.isdir(root):
            continue
        n_cand += 1
        _log(f"[venv扫描] 检查候选 conda 根: {root}")
        env = os.path.join(root, "envs", name)
        if _is_lerobot_env(env, log=_log):
            _log(f"[venv扫描] 命中 lerobot 环境: {env}", "ok")
            return os.path.normpath(env)
        # 1b) base 环境兜底 (手动安装常把 lerobot 装进 base)
        base_py = _lerobot_base_python(root)
        if base_py:
            _log(f"[venv扫描] 命中 lerobot 环境(base): {root}", "ok")
            return os.path.normpath(root)

    # 2) 浅优先: 仅扫各本地固定盘 (C:/D:/...) 深度<=2 的 conda 系安装目录.
    #    只匹配 miniconda*/anaconda*/miniforge* 目录名, 速度极快, 优先于全深扫描,
    #    能迅速命中 C:\miniconda3、C:\Users\<用户>\miniconda3、D:\miniconda3 等浅地址.
    deadline = time.time() + timeout
    scanned = 0
    conda_hint = ("conda", "forge", "anaconda")

    def _try_conda_root(r):
        env = os.path.join(r, "envs", name)
        if _is_lerobot_env(env, log=_log):
            _log(f"[venv扫描] 命中 lerobot 环境: {env}", "ok")
            return os.path.normpath(env)
        # base 环境兜底
        if _lerobot_base_python(r):
            _log(f"[venv扫描] 命中 lerobot 环境(base): {r}", "ok")
            return os.path.normpath(r)
        return ""

    for drive in _local_fixed_drives():
        if time.time() > deadline:
            return _fail()
        try:
            with os.scandir(drive) as it:
                l1 = [e.path for e in it
                      if not e.is_symlink() and e.is_dir(follow_symlinks=False)]
        except Exception:
            l1 = []
        # 深度1: 盘根直接子目录中含 conda 字样 (如 C:\miniconda3)
        for r in l1:
            bn = os.path.basename(r).lower()
            if any(h in bn for h in conda_hint):
                n_conda += 1
                _log(f"[venv扫描] 检查 conda 安装: {r}")
                hit = _try_conda_root(r)
                if hit:
                    return hit
        # 深度2: 盘根子目录下再一层 (如 C:\Users\Administrator\miniconda3)
        for r in l1:
            if time.time() > deadline:
                return _fail()
            try:
                with os.scandir(r) as it:
                    l2 = [e.path for e in it
                          if not e.is_symlink() and e.is_dir(follow_symlinks=False)]
            except Exception:
                l2 = []
            for s in l2:
                bn = os.path.basename(s).lower()
                if any(h in bn for h in conda_hint):
                    n_conda += 1
                    _log(f"[venv扫描] 检查 conda 安装: {s}")
                    hit = _try_conda_root(s)
                    if hit:
                        return hit
            scanned += 1

    # 3) 全深兜底扫描 (有界): 覆盖自定前缀 / -p 等非常规位置
    last_report = 0

    def roots() -> list[str]:
        r = []
        prog = os.environ.get("ProgramData") or "C:/ProgramData"
        for d in (prof, local, prog):
            if d and os.path.isdir(d):
                r.append(d)
        for d in _local_fixed_drives():
            r.append(d)
        seen2, out = set(), []
        for x in r:
            x = os.path.normpath(x)
            if x not in seen2:
                seen2.add(x)
                out.append(x)
        return out

    for base in roots():
        if not os.path.isdir(base):
            continue
        stack = [(base, 0)]
        while stack:
            if time.time() > deadline or scanned > max_dirs:
                return _fail()
            cur, depth = stack.pop()
            try:
                with os.scandir(cur) as it:
                    entries = list(it)
            except Exception:
                continue
            for e in entries:
                try:
                    if e.is_symlink():
                        continue
                    is_dir = e.is_dir(follow_symlinks=False)
                except Exception:
                    continue
                if not is_dir:
                    continue
                bn = e.name.lower()
                if bn == name_low:
                    # 名为 lerobot: 校验是否为环境; 是则命中, 否则记录并跳过
                    n_lerobot_dir += 1
                    if _is_lerobot_env(e.path, log=_log):
                        _log(f"[venv扫描] 命中 lerobot 环境: {e.path}", "ok")
                        return os.path.normpath(e.path)
                    n_lerobot_bad += 1
                    _log(f"[venv扫描] 发现 lerobot 目录但非 python 环境(疑似源码克隆): {e.path}", "warn")
                    continue
                if depth >= max_depth:
                    continue
                if bn in _ENV_SCAN_SKIP or bn.startswith("$"):
                    continue
                stack.append((e.path, depth + 1))
                scanned += 1
                if on_progress and (scanned - last_report) >= 8000:
                    last_report = scanned
                    on_progress(f"正在全盘搜索 lerobot 环境... 已扫描 {scanned} 个目录")
    return _fail()


class _EnvSearchWorker(QThread):
    """后台线程: 全盘搜索 lerobot 虚拟环境, 完成后发回路径 (空串=未找到)."""
    progress = Signal(str)
    log = Signal(str, str)      # (msg, level) -> 主线程日志框
    finished = Signal(str)

    def __init__(self, candidate_dirs: list[str]):
        super().__init__()
        self._candidate_dirs = candidate_dirs

    def run(self):
        try:
            path = _scan_for_lerobot_env(
                self._candidate_dirs,
                on_progress=self.progress.emit,
                on_log=self.log.emit)
        except Exception as e:  # 兜底: 任何异常都不应让线程静默卡死
            path = ""
            print("[EnvSearchWorker] 异常:", e)
        self.finished.emit(path or "")


def restart_all_com_devices() -> tuple[bool, str]:
    """通过 pnputil 重置所有 COM 端口硬件驱动
    
    用于修复 Windows 下 COM 端口 ERROR_GEN_FAILURE(31) 驱动卡死问题。
    返回 (成功标志, 消息)。
    """
    # Step 1: 通过 PowerShell 查找所有 COM 端口设备实例 ID
    ps_cmd = (
        'Get-PnpDevice -PresentOnly | '
        'Where-Object { $_.Class -eq "Ports" -and $_.FriendlyName -match "COM\\d+" } | '
        'ForEach-Object { "{0}|{1}" -f $_.FriendlyName, $_.InstanceId }'
    )
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps_cmd],
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=15
        )
    except FileNotFoundError:
        return False, "找不到 PowerShell，请确认系统环境正常"
    except subprocess.TimeoutExpired:
        return False, "查找设备超时，请重试"

    stdout = (result.stdout or "").strip()
    if result.returncode != 0 or not stdout:
        return False, "无法找到任何 COM 端口设备"

    # 解析: "USB-SERIAL CH340 (COM20)|USB\VID_1A86..."
    devices: list[tuple[str, str]] = []
    for line in stdout.split('\n'):
        line = line.strip()
        if '|' in line:
            name, inst_id = line.split('|', 1)
            devices.append((name.strip(), inst_id.strip()))

    if not devices:
        return False, "未发现任何 COM 端口设备"

    # Step 2: 逐一重置
    ok_list: list[str] = []
    fail_list: list[str] = []

    for name, inst_id in devices:
        try:
            pnp_result = subprocess.run(
                ["pnputil", "/restart-device", inst_id],
                capture_output=True, text=True, encoding="utf-8", errors="replace",
                timeout=30
            )
            if pnp_result.returncode == 0:
                ok_list.append(name)
            else:
                err = (pnp_result.stderr or pnp_result.stdout or "未知错误").strip()
                fail_list.append(f"{name}: {err}")
        except FileNotFoundError:
            return False, "找不到 pnputil，请确认系统环境正常"
        except subprocess.TimeoutExpired:
            fail_list.append(f"{name}: 超时")

    # 组装结果
    parts = []
    if ok_list:
        parts.append(f"成功 ({len(ok_list)}): {', '.join(ok_list)}")
    if fail_list:
        parts.append(f"失败 ({len(fail_list)}):\n" + "\n".join(f"  - {f}" for f in fail_list))

    msg = "\n\n".join(parts)
    if not fail_list:
        return True, msg + "\n\n请重新连接"
    elif ok_list:
        # 部分成功也算成功
        msg += "\n\n提示: 部分设备可能需要以管理员身份运行此程序"
        return True, msg
    else:
        msg += "\n\n提示: 可能需要以管理员身份运行此程序"
        return False, msg


# ============================
# 关节角度可视化组件
# ============================

class JointBar(QWidget):
    """单关节旋转值指示条 — 右侧面板专用"""

    def __init__(self, label: str = "", parent=None):
        super().__init__(parent)
        self._master_deg = 0.0
        self._slave_deg = 0.0
        self._label = label
        self._min_deg = 0.0
        self._max_deg = 4095.0
        # 移动记录: 主臂/从臂的 max/min (默认开启, 重新连接时自动清空)
        self._record_enabled = True
        self._m_max = None
        self._m_min = None
        self._s_max = None
        self._s_min = None
        self.setMinimumHeight(56)
        self.setFixedHeight(56)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_limits(self, min_deg: float, max_deg: float):
        self._min_deg = min_deg
        self._max_deg = max_deg
        self.update()

    def set_record_enabled(self, enabled: bool):
        self._record_enabled = enabled
        if enabled:
            self.reset_record()
        else:
            self._m_max = self._m_min = self._s_max = self._s_min = None
        self.update()

    def reset_record(self):
        self._m_max = None
        self._m_min = None
        self._s_max = None
        self._s_min = None
        self.update()

    def set_angles(self, master_deg: float, slave_deg: float):
        self._master_deg = master_deg
        self._slave_deg = slave_deg
        # 记录移动最大/最小值
        if self._record_enabled:
            if self._m_max is None or master_deg > self._m_max:
                self._m_max = master_deg
            if self._m_min is None or master_deg < self._m_min:
                self._m_min = master_deg
            if self._s_max is None or slave_deg > self._s_max:
                self._s_max = slave_deg
            if self._s_min is None or slave_deg < self._s_min:
                self._s_min = slave_deg
        self.update()

    def _deg_to_x(self, deg: float, left: int, right: int) -> int:
        span = self._max_deg - self._min_deg
        if span == 0:
            return (left + right) // 2
        ratio = (deg - self._min_deg) / span
        return left + int(ratio * (right - left))

    def paintEvent(self, event):
        import PySide6.QtGui as QtGui

        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)

        w, h = self.width(), self.height()

        # 字体
        font_label = p.font()
        font_label.setPointSize(10)
        font_val = p.font()
        font_val.setPointSize(9)

        # ---------- 关节名 (左对齐) ----------
        p.setPen(QColor(180, 180, 180))
        p.setFont(font_label)
        name_w = QtGui.QFontMetrics(font_label).horizontalAdvance(self._label)
        p.drawText(6, 16, self._label)

        # ---------- 角度条区域 ----------
        bar_left = name_w + 16
        bar_right = w - 10
        bar_y = 30
        bar_h = 8
        bar_mid = bar_y + bar_h // 2
        bar_w = bar_right - bar_left

        if bar_w < 40:
            p.end()
            return

        # 背景条
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(55, 55, 55))
        p.drawRoundedRect(bar_left, bar_y, bar_w, bar_h, 4, 4)

        # 中位刻度线 (范围中心, raw面板=2048 / 偏移面板=0)
        mid_x = self._deg_to_x((self._min_deg + self._max_deg) / 2.0,
                               bar_left, bar_right)
        mid_x = max(bar_left, min(bar_right, mid_x))
        p.setPen(QtGui.QPen(QColor(90, 90, 90), 1))
        p.drawLine(mid_x, bar_y - 4, mid_x, bar_y + bar_h + 4)

        # 零点刻度 (值 0 处)
        zero_x = self._deg_to_x(0.0, bar_left, bar_right)
        zero_x = max(bar_left, min(bar_right, zero_x))
        p.drawLine(zero_x, bar_y - 6, zero_x, bar_y + bar_h + 6)

        # 主臂指示 (蓝色圆点)
        mx = self._deg_to_x(self._master_deg, bar_left, bar_right)
        mx = max(bar_left, min(bar_right, mx))
        p.setBrush(QColor(52, 152, 219))
        p.setPen(QtGui.QPen(QColor(255, 255, 255), 1.5))
        p.drawEllipse(int(mx - 6), bar_mid - 6, 12, 12)

        # 从臂指示 (绿色钻石)
        sx = self._deg_to_x(self._slave_deg, bar_left, bar_right)
        sx = max(bar_left, min(bar_right, sx))
        diamond = QtGui.QPolygon([
            QPoint(int(sx), bar_mid - 7),
            QPoint(int(sx) + 7, bar_mid),
            QPoint(int(sx), bar_mid + 7),
            QPoint(int(sx) - 7, bar_mid),
        ])
        p.setBrush(QColor(46, 204, 113))
        p.setPen(QtGui.QPen(QColor(255, 255, 255), 1))
        p.drawPolygon(diamond)

        # ---------- 移动记录 max/min 标记 (仅 raw 面板) ----------
        if self._record_enabled:
            # 主臂 max/min (条上方蓝色刻度)
            if self._m_max is not None:
                mxx = self._deg_to_x(self._m_max, bar_left, bar_right)
                mxx = max(bar_left, min(bar_right, mxx))
                p.setPen(QtGui.QPen(QColor(52, 152, 219), 1))
                p.drawLine(mxx, bar_y - 10, mxx, bar_y - 2)
            if self._m_min is not None:
                mnx = self._deg_to_x(self._m_min, bar_left, bar_right)
                mnx = max(bar_left, min(bar_right, mnx))
                p.setPen(QtGui.QPen(QColor(52, 152, 219), 1))
                p.drawLine(mnx, bar_y - 10, mnx, bar_y - 2)

            # 从臂 max/min (条下方绿色刻度)
            if self._s_max is not None:
                sxx = self._deg_to_x(self._s_max, bar_left, bar_right)
                sxx = max(bar_left, min(bar_right, sxx))
                p.setPen(QtGui.QPen(QColor(46, 204, 113), 1))
                p.drawLine(sxx, bar_y + bar_h + 2, sxx, bar_y + bar_h + 10)
            if self._s_min is not None:
                snx = self._deg_to_x(self._s_min, bar_left, bar_right)
                snx = max(bar_left, min(bar_right, snx))
                p.setPen(QtGui.QPen(QColor(46, 204, 113), 1))
                p.drawLine(snx, bar_y + bar_h + 2, snx, bar_y + bar_h + 10)

        # ---------- 数值刻度 ----------
        p.setPen(QColor(120, 120, 120))
        p.setFont(font_val)
        p.drawText(bar_left + 1, bar_y + bar_h + 18, f"{self._min_deg:.0f}")

        fm_range = QtGui.QFontMetrics(font_val)
        # 动态显示主臂当前值 (raw面板=绝对位置 / 偏移面板=偏移量)
        master_text = f"{self._master_deg:.0f}"
        slave_text = f"{self._slave_deg:.0f}"
        master_w = fm_range.horizontalAdvance(master_text)
        slave_w = fm_range.horizontalAdvance(slave_text)
        # 主臂值(蓝色) + 从臂值(绿色) 并排显示在中位刻度右侧
        p.setPen(QColor(52, 152, 219))
        p.drawText(mid_x + 4, bar_y + bar_h + 18, master_text)
        p.setPen(QColor(46, 204, 113))
        p.drawText(mid_x + 4 + master_w + 4, bar_y + bar_h + 18, slave_text)

        max_text = f"{self._max_deg:.0f}"
        max_w = fm_range.horizontalAdvance(max_text)
        p.setPen(QColor(120, 120, 120))
        p.drawText(bar_right - max_w, bar_y + bar_h + 18, max_text)

        p.end()


# ============================
# 配置持久化辅助
# ============================

def _resource_path(relative: str) -> Path:
    """获取资源路径, 兼容 PyInstaller 打包和源码运行"""
    base = getattr(sys, "_MEIPASS", str(Path(__file__).parent))
    return Path(base) / relative


CONFIG_PATH = _resource_path("delta_arm_config.json")
ICON_PATH = _resource_path("app_icon.ico")

# 关节名映射 (id -> 标准关节名, 与 lerobot 校准格式一致)
JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}




# 导出校准值的默认目标路径 (lerobot 校准目录)
# 解析规则与 lerobot 0.3.4 源码 (lerobot/constants.py) 保持一致:
#   1) 若设置了 HF_LEROBOT_HOME 环境变量 -> 直接用
#   2) 否则取 HF_HOME (huggingface_hub 默认 ~/.cache/huggingface) / "lerobot"
#   3) 校准目录 = 上述 / "calibration"
# 这样在用户自定义了这些环境变量的电脑上, 也能和 lerobot 读写同一份校准文件.
# 用 expanduser 解析 ~, 兼容 PyInstaller exe 环境.
def _lerobot_calib_base():
    env = os.environ
    if env.get("HF_LEROBOT_HOME"):
        base_home = Path(env["HF_LEROBOT_HOME"]).expanduser()
    else:
        # HF_HOME 来自 huggingface_hub.constants, 默认 ~/.cache/huggingface
        hf_home = env.get("HF_HOME") or str(Path.home() / ".cache" / "huggingface")
        base_home = Path(hf_home).expanduser() / "lerobot"
    return base_home / "calibration"


def _lerobot_calib_candidates():
    """返回 (leader_dirs, follower_dirs) 两组候选目录 (均已 normpath).

    仅使用 lerobot 0.3.4 的标准目录结构, 不再兼容旧版本/其他命名,
    避免一次导出写出多份冗余文件.
    """
    base = _lerobot_calib_base()
    leader_dirs = [base / "teleoperators" / "so101_leader"]
    follower_dirs = [base / "robots" / "so101_follower"]
    return leader_dirs, follower_dirs


def _lerobot_calib_dirs():
    """主候选 (0.3.3 / 0.3.4 标准目录). 供保存默认使用."""
    base = _lerobot_calib_base()
    return (
        base / "teleoperators" / "so101_leader",
        base / "robots" / "so101_follower",
    )


CALIB_EXPORT_LEADER_DIR, CALIB_EXPORT_FOLLOWER_DIR = _lerobot_calib_dirs()

def export_calibration_file(joints: list, id_to_homing: dict,
                            id_to_range: dict) -> dict:
    """
    按 lerobot 校准格式生成关节 JSON 数据.
    joints: JointConfig 列表
    id_to_homing: {id: homing_offset}
    id_to_range: {id: (range_min, range_max)}
    """
    data = {}
    for j in joints:
        name = JOINT_NAMES.get(j.master_id, f"joint_{j.master_id}")
        homing = id_to_homing.get(j.master_id, 0)
        rmin, rmax = id_to_range.get(j.master_id, (0, 4095))
        data[name] = {
            "id": j.master_id,
            "drive_mode": 0,
            "homing_offset": homing,
            "range_min": rmin,
            "range_max": rmax,
        }
    return data

def save_calib_json(path: Path, data: dict):
    """保存校准 JSON 到指定路径"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)

def save_config(loop_hz: float):
    """保存配置到文件 (仅保存控制频率, 不保存串口以去除记忆功能)"""
    data = {
        "loop_hz": loop_hz,
    }
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def load_config() -> dict:
    """从文件加载配置, 不存在则返回 None"""
    if not CONFIG_PATH.exists():
        return None
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


# ============================
# 主窗口
# ============================

class MainWindow(QMainWindow):
    """SO-ARM101 增量控制上位机主窗口"""

    # 信号 (从后台线程更新 UI)
    pose_signal = Signal(list, list)    # (master_deg, slave_deg)
    state_signal = Signal(int)           # CtrlState 枚举值
    status_signal = Signal(str)          # 状态栏文本
    loop_stat_signal = Signal(int, float)  # (count, ms)
    read_limits_signal = Signal(dict)    # 后台读取的限位值回填表格: {id:(min,max)}

    def __init__(self):
        super().__init__()
        self.setWindowTitle("SO-ARM101")
        if ICON_PATH.exists():
            self.setWindowIcon(QIcon(str(ICON_PATH)))
        self.setMinimumSize(1000, 680)
        self.resize(1100, 720)

        # 核心控制器
        self._ctrl: DeltaController | None = None
        self._connected = False
        self._torque_enabled = False   # 扭矩是否已使能
        # 捕获零点时缓存的 Homing 校准值 (供导出校准值使用)
        self._calib_homing_master = {}
        self._calib_homing_slave = {}
        # 零点原始值 (捕获零点时记录, 用于计算实时偏移量)
        self._zero_m_deg: list[float] = []
        self._zero_s_deg: list[float] = []

        # 定时器
        self._port_timer = QTimer(self)     # 串口刷新
        self._port_timer.timeout.connect(self._refresh_ports)
        self._port_timer.start(2000)
        # 窗口显示后立即刷新一次串口 (singleShot(0) 在 UI 构建完成后下一次事件循环执行)
        QTimer.singleShot(0, self._refresh_ports)

        # 连接信号
        self.pose_signal.connect(self._on_pose_update)
        self.state_signal.connect(self._on_state_change)
        self.status_signal.connect(self._on_status_msg)
        self.loop_stat_signal.connect(self._on_loop_stat)
        self.read_limits_signal.connect(self._on_read_limits_done)

        self._init_ui()
        self._load_config()

    # ============ UI 构建 ============

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # --- 顶部: 串口配置 ---
        top_group = self._build_serial_panel()
        root.addWidget(top_group)

        # --- 中部: 限位面板 + 实时 raw 值 + 实时偏移量 (宽度比 2:1:1) ---
        mid_split = QSplitter(Qt.Horizontal)
        mid_split.addWidget(self._build_limit_panel())
        mid_split.addWidget(self._build_angle_panel())
        mid_split.addWidget(self._build_offset_panel())
        # 默认宽度比例: 限位 : raw : 偏移 = 2 : 1 : 1
        mid_split.setSizes([400, 200, 200])
        mid_split.setStretchFactor(0, 2)
        mid_split.setStretchFactor(1, 1)
        mid_split.setStretchFactor(2, 1)
        root.addWidget(mid_split, 1)

        # --- 底部: 控制按钮 & 状态 ---
        bottom_layout = QHBoxLayout()

        # 左侧控制按钮
        ctrl_group = self._build_control_panel()
        bottom_layout.addWidget(ctrl_group, 1)

        # 右侧日志
        log_group = self._build_log_panel()
        bottom_layout.addWidget(log_group, 1)

        root.addLayout(bottom_layout)

        # --- 状态栏 ---
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("就绪 - 请选择串口并连接")

        # 循环频率显示
        self._loop_label = QLabel("循环: -- Hz | 耗时: -- ms")
        self._loop_label.setStyleSheet("color: #888; padding-right: 10px;")
        self._status_bar.addPermanentWidget(self._loop_label)

    def _build_serial_panel(self) -> QGroupBox:
        g = QGroupBox("串口连接")
        layout = QHBoxLayout(g)
        layout.setSpacing(8)

        # 主臂串口 (可编辑, 禁止聚焦隐藏光标)
        layout.addWidget(QLabel("主臂:"))
        self._master_port_cb = QComboBox()
        self._master_port_cb.setEditable(True)
        self._master_port_cb.setFocusPolicy(Qt.NoFocus)
        self._master_port_cb.setMinimumWidth(100)
        layout.addWidget(self._master_port_cb)

        # 从臂串口 (可编辑, 禁止聚焦隐藏光标)
        layout.addWidget(QLabel("从臂:"))
        self._slave_port_cb = QComboBox()
        self._slave_port_cb.setEditable(True)
        self._slave_port_cb.setFocusPolicy(Qt.NoFocus)
        self._slave_port_cb.setMinimumWidth(100)
        layout.addWidget(self._slave_port_cb)

        # 波特率
        layout.addWidget(QLabel("波特率:"))
        self._baud_cb = QComboBox()
        self._baud_cb.addItems(["1000000", "500000", "115200"])
        self._baud_cb.setCurrentText("1000000")
        self._baud_cb.setFixedWidth(90)
        layout.addWidget(self._baud_cb)

        # 循环频率
        layout.addWidget(QLabel("频率(Hz):"))
        self._loop_spin = QDoubleSpinBox()
        self._loop_spin.setRange(10, 100)
        self._loop_spin.setValue(80)
        self._loop_spin.setDecimals(0)
        self._loop_spin.setFixedWidth(60)
        layout.addWidget(self._loop_spin)

        # 一键选择相邻 COM 口
        self._auto_select_asc = True  # True=正向, False=反向
        self._auto_select_btn = QPushButton("一键选择")
        self._auto_select_btn.setFixedWidth(80)
        self._auto_select_btn.setToolTip("自动选择两个编号最相近的 COM 口填入主臂/从臂\n点击切换正/反向排序")
        self._auto_select_btn.clicked.connect(self._on_auto_select)
        layout.addWidget(self._auto_select_btn)

        # 连接按钮
        self._connect_btn = QPushButton("连接")
        self._connect_btn.setFixedWidth(70)
        self._connect_btn.setStyleSheet(
            "QPushButton { background-color: #2ecc71; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 5px; }"
            "QPushButton:hover { background-color: #3498db; }"
            "QPushButton:disabled { background-color: #555; }"
        )
        self._connect_btn.clicked.connect(self._on_connect)
        layout.addWidget(self._connect_btn)

        # 断开按钮
        self._disconnect_btn = QPushButton("断开")
        self._disconnect_btn.setFixedWidth(70)
        self._disconnect_btn.setEnabled(False)
        self._disconnect_btn.clicked.connect(self._on_disconnect)
        layout.addWidget(self._disconnect_btn)

        # 重置 COM 端口按钮 (调试用)
        self._restart_port_btn = QPushButton("重置COM")
        self._restart_port_btn.setFixedWidth(80)
        self._restart_port_btn.setToolTip("使用 pnputil 重启所有 COM 端口硬件驱动\n适用于 ERROR_GEN_FAILURE(31) 等驱动卡死问题")
        self._restart_port_btn.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; "
            "border-radius: 4px; padding: 5px; }"
            "QPushButton:hover { background-color: #2980b9; }"
            "QPushButton:disabled { background-color: #555; }"
        )
        self._restart_port_btn.clicked.connect(self._on_restart_port)
        layout.addWidget(self._restart_port_btn)

        # 导出校准值 (leader/follower)
        self._export_btn = QPushButton("导出校准")
        self._export_btn.setFixedWidth(80)
        self._export_btn.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 6px; }"
            "QPushButton:hover { background-color: #2980b9; }"
            "QPushButton:disabled { background-color: #555; }"
        )
        self._export_btn.setToolTip(
            "导出主臂(leader)和从臂(follower)的校准值 JSON\n"
            "保存到 lerobot 校准目录, 用于机械臂遥控")
        self._export_btn.setEnabled(False)
        self._export_btn.clicked.connect(self._on_export_calib)
        layout.addWidget(self._export_btn)

        # 虚拟环境 (断开串口并启动 lerobot 遥操作)
        self._venv_btn = QPushButton("虚拟环境")
        self._venv_btn.setFixedWidth(80)
        self._venv_btn.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 6px; }"
            "QPushButton:hover { background-color: #2980b9; }"
            "QPushButton:disabled { background-color: #555; }"
        )
        self._venv_btn.setToolTip(
            "断开当前串口连接, 并在 conda lerobot 环境中启动遥操作:\n"
            "python -m lerobot.teleoperate --robot.type=so101_follower ...\n"
            "COM 口自动使用软件当前选择的主臂/从臂端口")
        self._venv_btn.clicked.connect(self._on_launch_venv)
        layout.addWidget(self._venv_btn)

        layout.addStretch()
        return g

    def _build_limit_panel(self) -> QGroupBox:
        g = QGroupBox("舵机限位值")
        layout = QVBoxLayout(g)
        layout.setSpacing(6)

        self._limit_table = QTableWidget(NUM_JOINTS, 5)
        self._limit_table.setHorizontalHeaderLabels([
            "id", "主臂min", "主臂max", "从臂min", "从臂max"
        ])
        self._limit_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        # min/max 列可编辑, id 列只读
        self._limit_table.setEditTriggers(
            QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self._limit_table.verticalHeader().setVisible(False)
        # 表格含表头总高约260px: 每行高度 = (260 - 表头) / 行数
        # 表头约32px, 每行 ≈ (260 - 32) / NUM_JOINTS
        row_h = max(1, (260 - 32) // NUM_JOINTS)
        self._limit_table.verticalHeader().setDefaultSectionSize(row_h)
        for r in range(NUM_JOINTS):
            self._limit_table.setRowHeight(r, row_h)

        for i, j in enumerate(DEFAULT_JOINTS):
            name_item = QTableWidgetItem(j.name)
            name_item.setTextAlignment(Qt.AlignCenter)
            name_item.setFlags(
                Qt.ItemIsEnabled | Qt.ItemIsSelectable)   # id 列不可编辑
            self._limit_table.setItem(i, 0, name_item)
            for col in range(1, 5):
                item = QTableWidgetItem("--")
                item.setTextAlignment(Qt.AlignCenter)
                self._limit_table.setItem(i, col, item)

        layout.addWidget(self._limit_table)

        # (0,4095) 与 实时raw限位值 两个按钮并排 (各占一半)
        fill_row = QHBoxLayout()
        fill_row.setSpacing(4)
        self._fill_04095_btn = QPushButton("（0,4095）")
        self._fill_04095_btn.setEnabled(False)
        self._fill_04095_btn.setToolTip(
            "将表格中所有舵机的 min/max 填写为 (0, 4095)")
        self._fill_04095_btn.clicked.connect(self._on_fill_04095)
        fill_row.addWidget(self._fill_04095_btn, 1)
        self._fill_raw_btn = QPushButton("实时raw限位值")
        self._fill_raw_btn.setEnabled(False)
        self._fill_raw_btn.setToolTip(
            "按实时 raw 值(min/max)填写表格, 需先运行且 raw 值有变化")
        self._fill_raw_btn.clicked.connect(self._on_fill_raw_limits)
        fill_row.addWidget(self._fill_raw_btn, 1)
        layout.addLayout(fill_row)

        # 一键限位: 把表格中的 min/max 值写入各舵机 EEPROM
        self._apply_limit_btn = QPushButton("一键限位 (写入EEPROM)")
        self._apply_limit_btn.setEnabled(False)
        self._apply_limit_btn.setToolTip(
            "将表格中每个舵机的 min/max 值写入主臂+从臂舵机 EEPROM\n"
            "(写 EEPROM 会关断扭矩, 需要重新使能)")
        self._apply_limit_btn.clicked.connect(self._on_apply_limits)
        layout.addWidget(self._apply_limit_btn)

        return g

    def _build_angle_panel(self) -> QGroupBox:
        g = QGroupBox("实时 raw 值  (●主臂  ◆从臂)")
        layout = QVBoxLayout(g)
        layout.setContentsMargins(9, 0, 9, 9)
        layout.setSpacing(0)

        self._joint_bars: list[JointBar] = []
        for i in range(NUM_JOINTS):
            bar = JointBar(DEFAULT_JOINTS[i].name)
            bar.set_limits(0.0, 4095.0)   # 固定显示舵机原始值范围 0~4095
            layout.addWidget(bar)
            self._joint_bars.append(bar)

        return g

    def _build_offset_panel(self) -> QGroupBox:
        g = QGroupBox("实时偏移量  (●主臂  ◆从臂)")
        layout = QVBoxLayout(g)
        layout.setContentsMargins(9, 0, 9, 9)
        layout.setSpacing(0)

        self._offset_bars: list[JointBar] = []
        for i in range(NUM_JOINTS):
            bar = JointBar(DEFAULT_JOINTS[i].name)
            # 偏移量范围: 相对零点的偏移量, 刻度范围 ±2048
            bar.set_limits(-2048.0, 2048.0)
            layout.addWidget(bar)
            self._offset_bars.append(bar)

        return g


    def _read_back_limits_to_table(self):
        """写入 EEPROM 后读回舵机限位值并刷新限位表格 (主线程同步).
        读取前已暂停控制循环, 总线空闲; EEPROM 烧写后舵机短暂不响应读,
        因此对每次读取加重试."""
        if not (self._ctrl and hasattr(self, "_limit_table")):
            return
        for i, j in enumerate(self._ctrl.joints):
            for bus, sid, cols in (
                    (self._ctrl.master_bus, j.master_id, (1, 2)),
                    (self._ctrl.slave_bus, j.slave_id, (3, 4))):
                r = None
                for _ in range(5):
                    try:
                        r = bus.read_position_limits(sid)
                        if r is not None:
                            break
                    except Exception:
                        pass
                    time.sleep(0.1)
                if r is not None:
                    self._limit_table.item(i, cols[0]).setText(str(int(r[0])))
                    self._limit_table.item(i, cols[1]).setText(str(int(r[1])))
                else:
                    self._limit_table.item(i, cols[0]).setText("--")
                    self._limit_table.item(i, cols[1]).setText("--")
        self._log("已读回舵机限位值并刷新表格", "info")

    def _build_control_panel(self) -> QGroupBox:
        g = QGroupBox("控制")
        layout = QVBoxLayout(g)
        layout.setSpacing(8)

        # 扭矩使能 / 失能 / 设置目标 raw
        torque_row = QHBoxLayout()
        self._torque_btn = QPushButton("使能扭矩")
        self._torque_btn.setEnabled(False)
        self._torque_btn.clicked.connect(self._on_enable_torque)
        torque_row.addWidget(self._torque_btn)
        self._disable_torque_btn = QPushButton("失能扭矩")
        self._disable_torque_btn.setEnabled(False)
        self._disable_torque_btn.clicked.connect(self._on_disable_torque)
        torque_row.addWidget(self._disable_torque_btn)

        self._center_target_btn = QPushButton("设置目标raw")
        self._center_target_btn.setEnabled(False)
        self._center_target_btn.setToolTip("设置 1~6 号舵机软件中位的目标 raw 值")
        self._center_target_btn.clicked.connect(self._on_edit_center_targets)
        torque_row.addWidget(self._center_target_btn)

        layout.addLayout(torque_row)

        # 1~6 号舵机软件中位的目标 raw 值 (由"设置目标raw"按钮弹窗编辑)
        self._center_target_values = [2048, 850, 3100, 2800, 2048, 880]

        # 一键中位
        self._center_btn = QPushButton("一键中位")
        self._center_btn.setEnabled(False)
        self._center_btn.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 6px; }"
            "QPushButton:hover { background-color: #2980b9; }"
            "QPushButton:disabled { background-color: #555; }"
        )
        self._center_btn.setToolTip(
            "读取 1~6 号舵机(主臂+从臂)当前位置作基准, 写入 Homing_Offset 使当前姿态对应各目标 raw 值\n"
            "(lerobot 标准做法: 不依赖舵机 40=128 指令, 仅改 homing 寄存器; 写 EEPROM 会关断扭矩)")
        self._center_btn.clicked.connect(self._on_software_center_all)
        layout.addWidget(self._center_btn)

        # 捕获零点
        self._capture_btn = QPushButton("捕获零点")
        self._capture_btn.setEnabled(False)
        self._capture_btn.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 6px; }"
            "QPushButton:hover { background-color: #2980b9; }"
            "QPushButton:disabled { background-color: #555; }"
        )
        self._capture_btn.clicked.connect(self._on_capture_zero)
        layout.addWidget(self._capture_btn)

        # 运行
        self._run_btn = QPushButton("▶ 运行")
        self._run_btn.setEnabled(False)
        self._run_btn.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 6px; }"
            "QPushButton:hover { background-color: #2980b9; }"
            "QPushButton:disabled { background-color: #555; }"
        )
        self._run_btn.clicked.connect(self._on_run)
        layout.addWidget(self._run_btn)

        # 状态指示器
        self._state_label = QLabel("状态: 未连接")
        self._state_label.setAlignment(Qt.AlignCenter)
        self._state_label.setStyleSheet(
            "QLabel { background-color: #333; color: #aaa; "
            "border-radius: 4px; padding: 6px; font-weight: bold; }")
        layout.addWidget(self._state_label)

        layout.addStretch()
        return g

    def _build_log_panel(self) -> QGroupBox:
        g = QGroupBox("日志")
        layout = QVBoxLayout(g)

        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.document().setMaximumBlockCount(500)
        self._log_text.setStyleSheet(
            "QTextEdit { background-color: #1e1e1e; color: #ccc; "
            "font-family: Consolas, monospace; font-size: 11px; }")
        layout.addWidget(self._log_text)

        return g

    # ============ 配置管理 ============

    def _load_config(self):
        # 仅恢复控制频率, 不恢复串口 (去除记忆功能, 每次启动重新扫描)
        cfg = load_config()
        if cfg is None:
            return
        if cfg.get("loop_hz"):
            self._loop_spin.setValue(cfg["loop_hz"])

    def _save_current_config(self):
        # 仅保存控制频率, 不保存串口 (去除记忆功能)
        save_config(loop_hz=self._loop_spin.value())

    # ============ UI 事件处理 ============

    def _on_auto_select(self):
        """一键选择两个编号最相近的 COM 口, 点击切换主从正/反向"""
        ports = get_port_descriptions()
        # 排除系统串口 COM1
        ports = {k: v for k, v in ports.items() if not k.upper().endswith("COM1")}
        if len(ports) < 2:
            self._log("一键选择: 可用 COM 口不足 2 个", "warn")
            return

        # 提取 COM 编号并排序
        def _port_num(p):
            m = re.search(r'COM(\d+)', p.upper())
            return int(m.group(1)) if m else 9999

        sorted_ports = sorted(ports.keys(), key=_port_num)

        # 找编号差最小的一对
        best_a, best_b = sorted_ports[0], sorted_ports[1]
        best_diff = _port_num(sorted_ports[1]) - _port_num(sorted_ports[0])
        for i in range(1, len(sorted_ports) - 1):
            diff = _port_num(sorted_ports[i + 1]) - _port_num(sorted_ports[i])
            if diff < best_diff:
                best_diff = diff
                best_a, best_b = sorted_ports[i], sorted_ports[i + 1]

        # 正向: 小编号→主臂, 大编号→从臂; 反向: 大编号→主臂, 小编号→从臂
        if self._auto_select_asc:
            m, s = best_a, best_b
        else:
            m, s = best_b, best_a

        def _set_combo(cb, port):
            for j in range(cb.count()):
                if cb.itemData(j) == port or cb.itemText(j).startswith(port):
                    cb.setCurrentIndex(j)
                    return
            cb.setCurrentText(port)

        _set_combo(self._master_port_cb, m)
        _set_combo(self._slave_port_cb, s)

        direction = "正向" if self._auto_select_asc else "反向"
        self._log(f"一键选择 ({direction}): 主臂={m}, 从臂={s}", "ok")
        self._auto_select_asc = not self._auto_select_asc

    def _refresh_ports(self):
        """定时刷新可用串口列表, 用户打开下拉时不刷新"""
        # 下拉框展开中, 不打断用户操作
        if (self._master_port_cb.view().isVisible() or
            self._slave_port_cb.view().isVisible()):
            return

        ports = list_serial_ports()
        desc = get_port_descriptions()
        if not ports:
            return

        def _update_combo(cb: QComboBox, ports: list[str], desc: dict):
            # 检查端口列表是否变化, 不变就不重建
            current_items = [cb.itemText(i) for i in range(cb.count())]
            new_items = [f"{p} - {desc.get(p, '')}" if desc.get(p) else p for p in ports]
            if current_items == new_items:
                return

            current_data = cb.currentData()  # 用 data 恢复选中项
            cb.blockSignals(True)
            cb.clear()
            for p in ports:
                label = f"{p} - {desc.get(p, '')}" if desc.get(p) else p
                cb.addItem(label, p)
            if current_data:
                idx = cb.findData(current_data)
                if idx >= 0:
                    cb.setCurrentIndex(idx)
            cb.blockSignals(False)

        _update_combo(self._master_port_cb, ports, desc)
        _update_combo(self._slave_port_cb, ports, desc)

    # ============ 控制按钮事件 ============

    def _on_connect(self):
        """连接舵机总线"""
        if self._connected:
            return

        master_port = self._master_port_cb.currentData() or \
                      self._master_port_cb.currentText().split(" - ")[0].strip()

        # 双总线设计: 主臂从臂各用一个控制板(独立串口)
        slave_port = self._slave_port_cb.currentData() or \
                     self._slave_port_cb.currentText().split(" - ")[0].strip()

        baud = int(self._baud_cb.currentText())

        try:
            master_bus = SCServoBus(master_port, baud)
            slave_bus = SCServoBus(slave_port, baud) if slave_port else None

            self._ctrl = DeltaController(
                master_bus=master_bus,
                slave_bus=slave_bus,
                joints=DEFAULT_JOINTS,
                loop_hz=self._loop_spin.value(),
            )

            # 连接回调
            self._ctrl.on_state_change = self._on_ctrl_state
            self._ctrl.on_pose_update = self._on_ctrl_pose

            self._ctrl.connect()
            self._connected = True
            self._update_button_states()

            self._log(f"已连接: 主臂={master_port}" +
                     (f", 从臂={slave_port}" if slave_port else " (单总线模式)"), "ok")
            self._status_bar.showMessage("已连接 - 请使能扭矩")

            # 连接后自动读取舵机限位值
            self._on_read_limits()
            self._apply_limit_btn.setEnabled(True)
            self._export_btn.setEnabled(True)
            self._fill_04095_btn.setEnabled(True)
            # 实时raw限位值按钮需 raw 有变化才可用
            self._update_fill_raw_btn()

            # 重新连接即刷新移动记录
            for bar in self._joint_bars:
                bar.reset_record()

            self._save_current_config()

        except IOError as e:
            QMessageBox.critical(self, "连接失败",
                f"{e}\n\n常见原因:\n"
                "1. USB串口模块松动或掉线 → 重新插拔\n"
                "2. 驱动异常 → 设备管理器中禁用再启用该端口\n"
                "3. 端口被其他程序占用 → 关闭相关软件后重试")
            self._log(f"连接失败: {e}", "error")
        except ConnectionError as e:
            QMessageBox.critical(self, "连接失败",
                f"{e}\n\n串口已打开但舵机未响应，请检查接线和供电。")
            self._log(f"连接失败: {e}", "error")
        except Exception as e:
            QMessageBox.critical(self, "连接失败", str(e))
            self._log(f"连接失败: {e}", "error")

    def _on_disconnect(self):
        """断开连接"""
        if self._ctrl:
            try:
                self._ctrl.disconnect()
            except Exception as e:
                self._log(f"断开时出错: {e}", "warn")
        self._ctrl = None
        self._connected = False
        self._apply_limit_btn.setEnabled(False)
        self._export_btn.setEnabled(False)
        self._fill_04095_btn.setEnabled(False)
        self._fill_raw_btn.setEnabled(False)
        self._update_button_states()
        self._log("已断开连接", "ok")
        self._status_bar.showMessage("已断开 - 请选择串口")

    def _on_launch_venv(self):
        """断开串口, 并在后台全盘搜索 lerobot 环境后启动遥操作 (无视觉同步控制).

        搜索 (含全盘扫描) 放在 _EnvSearchWorker 后台线程执行, 主线程只更新状态,
        因此即使环境不在常见位置、需要全盘扫描, UI 也不会卡顿/假死。
        """
        # 1. 若当前已连接, 先断开释放串口
        if self._connected:
            self._log("启动虚拟环境前断开当前串口连接...", "info")
            self._on_disconnect()

        # 2. 读取主臂/从臂下拉框当前选择的 COM 口
        def _port(cb):
            return (cb.currentData() or
                    cb.currentText().split(" - ")[0].strip())

        master_port = _port(self._master_port_cb)
        slave_port = _port(self._slave_port_cb)
        if not master_port or not slave_port:
            QMessageBox.warning(
                self, "虚拟环境",
                "未检测到主臂或从臂 COM 口, 请先选择串口")
            self._log("虚拟环境: 主臂或从臂 COM 口为空", "error")
            return

        # 3. 后台全盘搜索环境 (不卡 UI)
        self._venv_master_port = master_port
        self._venv_slave_port = slave_port
        self._venv_btn.setEnabled(False)
        self._log("正在搜索 lerobot 虚拟环境 (含全盘扫描)...", "info")
        self._status_bar.showMessage("正在搜索 lerobot 虚拟环境 (全盘扫描)...")
        self._env_worker = _EnvSearchWorker(self._conda_candidate_dirs())
        self._env_worker.progress.connect(self._on_env_search_progress)
        self._env_worker.log.connect(self._log)
        self._env_worker.finished.connect(self._on_env_search_done)
        self._env_worker.start()

    def _on_env_search_progress(self, msg: str):
        self._status_bar.showMessage(msg)

    def _on_env_search_done(self, env_dir: str):
        self._venv_btn.setEnabled(True)
        if not env_dir:
            self._status_bar.showMessage("未找到 lerobot 环境")
            QMessageBox.critical(
                self, "虚拟环境缺失",
                "全盘扫描未找到 lerobot 虚拟环境 (envs\\lerobot 或任意含 "
                "lerobot 包的 python 环境)。\n\n"
                "请先运行环境安装程序:\n\n"
                "    env_setup\\SO-ARM101环境安装程序.exe\n\n"
                "安装完成后重新点击 [虚拟环境] 按钮启动遥操作。")
            self._log("虚拟环境检测未通过: 全盘扫描未找到 lerobot", "error")
            return
        self._log(f"[venv] 找到环境 lerobot: {env_dir}", "info")
        self._log("虚拟环境检测通过 (lerobot 环境就绪)", "ok")
        self._launch_teleop(env_dir, self._venv_master_port, self._venv_slave_port)

    def _launch_teleop(self, env_dir, master_port, slave_port):
        """在指定 lerobot 环境直接运行 python -m lerobot.teleoperate (无视觉同步控制)."""
        # robot = 从臂 (so101_follower), teleop = 主臂 (so101_leader)
        # 直接运行该环境的 python.exe (conda 未注册此环境, conda activate 会报
        # Not a conda environment; 直接调其 python 即可, 与 09:04 成功版本一致)。
        # 解析环境 python: 优先 <env>/Scripts/python.exe (命名环境);
        # 若不在 (如 base 环境, python 在 <conda根>/python.exe) 则退回该路径.
        if os.name == "nt":
            env_py = os.path.join(env_dir, "Scripts", "python.exe")
            if not os.path.exists(env_py):
                alt = os.path.join(env_dir, "python.exe")
                if os.path.exists(alt):
                    env_py = alt
        else:
            env_py = os.path.join(env_dir, "bin", "python")
            if not os.path.exists(env_py):
                alt = os.path.join(env_dir, "python")
                if os.path.exists(alt):
                    env_py = alt
        robot_args = [
            "--robot.type=so101_follower", f"--robot.port={slave_port}",
            "--robot.id=my_awesome_follower_arm",
            "--teleop.type=so101_leader", f"--teleop.port={master_port}",
            "--teleop.id=my_awesome_leader_arm",
        ]
        # 探测 teleoperate 入口模块, 并打印版本/安装路径 (单次快速 import, 不卡顿)
        mod = self._lerobot_teleop_module(env_py) or "lerobot.teleoperate"
        cmd = f'"{env_py}" -m {mod} ' + " ".join(robot_args)

        # 写临时 bat 承载命令 (开新窗口, CREATE_NEW_CONSOLE)
        import tempfile
        script = os.path.join(tempfile.gettempdir(), "SO-Arm101_teleop.bat")
        bat = (
            "@echo off\r\n"
            f"{cmd}\r\n"
        )
        with open(script, "wb") as f:
            f.write(bat.encode("mbcs", errors="replace"))
        args = ["cmd", "/k", script]
        if os.environ.get("SOARM_DEBUG"):
            self._log(f"[调试] 启动脚本:\n{bat}", "info")
        self._status_bar.showMessage("正在启动 lerobot 遥操作...")

        try:
            subprocess.Popen(
                args,
                # cwd 设为 lerobot 环境根目录 (envs\\lerobot): 避免程序 cwd 下若存在
                # 同名 lerobot 副本被 `python -m` 优先导入, 导致误报找不到模块。
                cwd=env_dir,
                creationflags=subprocess.CREATE_NEW_CONSOLE,
            )
        except Exception as e:
            self._log(f"启动虚拟环境失败: {e}", "error")
            QMessageBox.critical(self, "虚拟环境启动失败", str(e))
            return

        self._log("已在新的 cmd 窗口启动 lerobot 遥操作 (直接调用环境 python)", "ok")

    def _venv_python(self) -> str:
        """返回可用的 lerobot venv 的 python.exe (健康校验通过).

        遍历所有候选 conda 目录, 收集含 envs\\lerobot\\Scripts\\python.exe 的候选,
        再做轻量健康预检 (import numpy), 返回**第一个健康**的环境。

        **不限 Python 版本**: 客户自装 / 本项目 env_setup 离线装的均可复用。但若某
        候选环境依赖损坏 (如客户机器上遗留的 Python 3.14 + numpy 2.2.6 不兼容,
        import numpy 直接崩), 自动跳过, 回退到下一个健康候选 (例如 env_setup 装在
        LOCALAPPDATA 的 py311 离线环境)。这同时满足 "客户自装环境能用就复用" 与
        "坏了不要误选坏环境"。

        找不到任何健康环境时, 返回第一个存在但可能损坏的路径 (交由启动预检弹窗给出
        清晰报错); 完全没有任何 lerobot venv 时返回空串。
        """
        found = []
        for base in self._conda_candidate_dirs():
            if not os.path.isdir(base):
                continue
            cand = os.path.join(base, "envs", "lerobot", "Scripts", "python.exe")
            if os.path.exists(cand):
                found.append(cand)
        # 兜底候选: 默认 LOCALAPPDATA\\miniconda3\\envs\\lerobot (与 env_setup default_conda_dir 一致)
        localapp = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~/AppData/Local")
        cand = os.path.normpath(os.path.join(
            localapp, "miniconda3", "envs", "lerobot", "Scripts", "python.exe"))
        if os.path.exists(cand):
            found.append(cand)
        # 去重且保持优先顺序 (~/miniconda3 等用户目录候选在前)
        seen, ordered = set(), []
        for c in found:
            if c not in seen:
                seen.add(c)
                ordered.append(c)
        # 健康预检: 优先返回能正常 import numpy 的环境, 损坏的自动跳过
        for py in ordered:
            if self._venv_healthy(py):
                return py
        # 无健康环境: 返回第一个存在的 (启动预检会弹窗明确报错), 否则空
        return ordered[0] if ordered else ""

    def _venv_healthy(self, py) -> bool:
        """轻量健康预检: 该 venv 能否正常 import numpy (C 扩展可用).

        仅 import numpy 即可快速筛掉 Python 版本过新导致的 numpy C 扩展缺失
        (如 3.14 + numpy 2.2.6), 又快又准; lerobot 包本身的完整性由启动前的
        _check_teleop_import 兜底。
        """
        try:
            r = subprocess.run(
                [py, "-c", "import numpy"],
                capture_output=True, text=True, encoding="utf-8", errors="replace",
                timeout=60,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            return r.returncode == 0
        except Exception:
            return False

    def _check_teleop_import(self, env_py, mod):
        """启动前依赖预检: 仅 import 入口模块 (不执行 __main__), 捕获 numpy /
        torch 等 C 扩展缺失或损坏 (客户的 venv Python 版本过新时常见)。

        返回 (ok, err_text, hint_text):
          ok=True          -> 依赖可用, 可直接启动
          ok=False         -> err_text 为原始报错, hint_text 为给用户的修复命令
        """
        pyver = self._conda_python_version(env_py)  # 形如 "3.14"
        try:
            r = subprocess.run(
                [env_py, "-c", f"import {mod}; print('IMPORT_OK')"],
                capture_output=True, text=True, encoding="utf-8", errors="replace",
                timeout=120,
                cwd=os.path.dirname(os.path.dirname(env_py)),
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except Exception as e:
            return False, f"预检执行异常: {e}", (
                "无法在该虚拟环境执行 python, 请确认文件未损坏或权限正常。")
        if r.returncode == 0:
            return True, "", ""
        err = (r.stderr or r.stdout).strip()
        low = err.lower()
        # 判定 numpy C 扩展缺失 / 不兼容 (客户 venv Python 过新时高发)
        if ("numpy" in low and (
                "_multiarray_umath" in low or "c-extension" in low
                or "no module named 'numpy" in low
                or "numpy from" in low and "source directory" in low)):
            fix = f'"{env_py}" -m pip install --upgrade "numpy>=2.3"'
            hint = (
                f"该虚拟环境的 Python 版本为 {pyver}（过新），已安装的 numpy "
                f"无对应可用的 C 扩展，导致导入失败。\n\n"
                f"请在该 lerobot 虚拟环境中执行以下命令升级 numpy：\n\n"
                f"    {fix}\n\n"
                f"若仍有其它依赖不兼容，建议直接重装该环境的 lerobot：\n"
                f'    "{env_py}" -m pip install --upgrade "lerobot[feetech]"')
            return False, err, hint
        # 其它导入错误 (torch / rerun / 依赖未装全等)
        hint = (
            f"该环境的 lerobot 依赖导入失败（{mod}）。常见原因：依赖未装全 "
            f"或版本不兼容。\n\n"
            f"请在该虚拟环境中重装 lerobot 及其依赖：\n"
            f'    "{env_py}" -m pip install --upgrade "lerobot[feetech]"')
        return False, err, hint

    def _lerobot_teleop_module(self, env_py):
        """探测 venv 里 lerobot 的 teleoperate 入口模块名 (找不到返回空串).

        不同 lerobot 安装版本的入口位置不同, 这里自动探测而非写死:
          - 新版:  lerobot/teleoperate/__main__.py   -> "lerobot.teleoperate"
          - 旧版:  lerobot/scripts/teleoperate.py     -> "lerobot.scripts.teleoperate"
          - 其它:  扫描 lerobot 包内含 teleoperate 且有 __main__/main 的 .py
        同时打印 lerobot 版本/安装路径, 便于安装不完整时排查.
        """
        probe = (
            "import os, sys\n"
            "try:\n"
            "    import lerobot\n"
            "except Exception as e:\n"
            "    print('LEROBOT_IMPORT_FAIL:' + repr(e)); sys.exit(0)\n"
            "base = os.path.dirname(lerobot.__file__)\n"
            "print('LEROBOT_PATH:' + base)\n"
            "print('LEROBOT_VERSION:' + str(getattr(lerobot, '__version__', 'unknown')))\n"
            "cands = []\n"
            "if os.path.isfile(os.path.join(base, 'teleoperate', '__main__.py')):\n"
            "    cands.append('lerobot.teleoperate')\n"
            "if os.path.isfile(os.path.join(base, 'scripts', 'teleoperate.py')):\n"
            "    cands.append('lerobot.scripts.teleoperate')\n"
            "if os.path.isfile(os.path.join(base, 'scripts', 'lerobot_teleoperate.py')):\n"
            "    cands.append('lerobot.scripts.lerobot_teleoperate')\n"
            "for root, dirs, files in os.walk(base):\n"
            "    for fn in files:\n"
            "        if fn.endswith('.py') and 'teleoperate' in fn:\n"
            "            full = os.path.join(root, fn)\n"
            "            try:\n"
            "                txt = open(full, encoding='utf-8', errors='replace').read()\n"
            "            except Exception:\n"
            "                txt = ''\n"
            "            if \"__name__ == '__main__'\" in txt or '__name__ == \"__main__\"' in txt or 'def main(' in txt:\n"
            "                rel = os.path.relpath(full, base).replace(os.sep, '.')\n"
            "                mod = 'lerobot.' + rel[:-3]\n"
            "                if mod not in cands:\n"
            "                    cands.append(mod)\n"
            "print('TELEOP_CANDS:' + '|'.join(cands))\n"
        )
        try:
            r = subprocess.run([env_py, "-c", probe], capture_output=True,
                               text=True, encoding="utf-8", errors="replace", timeout=30)
        except Exception as e:
            self._log(f"探测 lerobot 入口异常: {e}", "error")
            return ""
        for line in (r.stdout or "").splitlines():
            if line.startswith("LEROBOT_PATH:"):
                self._log("lerobot 安装路径: " + line.split(":", 1)[1], "info")
            elif line.startswith("LEROBOT_VERSION:"):
                self._log("lerobot 版本: " + line.split(":", 1)[1], "info")
            elif line.startswith("LEROBOT_IMPORT_FAIL:"):
                self._log("lerobot 导入失败: " + line.split(":", 1)[1], "error")
            elif line.startswith("TELEOP_CANDS:"):
                cands = [c for c in line.split(":", 1)[1].split("|") if c]
                if cands:
                    self._log(f"检测到 teleoperate 入口: {cands[0]}", "info")
                    return cands[0]
        return ""

    def _all_conda_exes(self):
        """返回所有能找到的 conda 可执行文件 (PATH + 候选目录), 去重保序."""
        exes, seen = [], set()
        p = self._conda_exe_via_path()
        if p and p not in seen:
            seen.add(p)
            exes.append(p)
        for base in self._conda_candidate_dirs():
            if not os.path.isdir(base):
                continue
            for rel in ("Scripts/conda.exe", "condabin/conda.bat"):
                c = os.path.normpath(os.path.join(base, rel))
                if os.path.exists(c) and c not in seen:
                    seen.add(c)
                    exes.append(c)
        return exes

    def _conda_env_dir(self, name) -> str:
        """返回名为 name 的 conda/venv 环境完整路径 (如 ...\\envs\\lerobot), 不存在返回空串.

        纯目录探测 (不跑 conda env list, 避免多进程卡顿/崩溃): 遍历候选 conda 根,
        查 <根>/envs/<name> 是否含 Scripts\\python.exe (Windows) 或 bin/python (POSIX)。
        覆盖默认 envs、自定义 envs_dirs、-p 前缀等 (只要目录落在某 conda 根下)。
        """
        py_rel = (os.path.join("Scripts", "python.exe")
                  if os.name == "nt" else os.path.join("bin", "python"))
        for root in self._conda_candidate_dirs():
            if not os.path.isdir(root):
                continue
            env = os.path.join(root, "envs", name)
            if os.path.isdir(env) and os.path.exists(os.path.join(env, py_rel)):
                self._log(f"[venv] 找到环境 {name}: {env}", "info")
                return os.path.normpath(env)
        # base 环境兜底 (手动安装常把 lerobot 装进 base)
        for root in self._conda_candidate_dirs():
            if not os.path.isdir(root):
                continue
            if _lerobot_base_python(root):
                self._log(f"[venv] 找到环境 {name} (base): {root}", "info")
                return os.path.normpath(root)
        self._log(f"[venv] 未在任何候选 conda 根下找到 envs/{name}", "error")
        return ""

    def _conda_env_exists(self, name) -> bool:
        """判断是否存在名为 name 的 conda 虚拟环境 (委托 _conda_env_dir)."""
        return bool(self._conda_env_dir(name))

    def _check_venv_ready(self) -> bool:
        """检测 lerobot conda 环境是否可用 (直接进 conda 环境测试).

        环境安装已独立到 env_setup 程序 (SO-ARM101环境安装程序.exe),
        本程序只负责检测已装好的 conda lerobot 环境并用其启动遥操作,
        不再内置自动安装逻辑. 缺失时引导用户先运行安装程序.
        """
        if not self._find_conda():
            QMessageBox.critical(
                self, "未找到 conda",
                "未检测到 conda (miniconda/anaconda)。\n\n"
                "请先运行环境安装程序:\n\n"
                "    env_setup\\SO-ARM101环境安装程序.exe")
            self._log("虚拟环境检测未通过: 未找到 conda", "error")
            return False
        if not self._conda_env_exists("lerobot"):
            QMessageBox.critical(
                self, "虚拟环境缺失",
                "未检测到名为 lerobot 的 conda 虚拟环境。\n\n"
                "请先运行环境安装程序:\n\n"
                "    env_setup\\SO-ARM101环境安装程序.exe\n\n"
                "安装完成后重新点击 [虚拟环境] 按钮启动遥操作。")
            self._log("虚拟环境检测未通过: 不存在 lerobot 环境", "error")
            return False
        self._log("虚拟环境检测通过 (conda lerobot 环境就绪)", "ok")
        return True

    def _conda_python_version(self, python_exe):
        """返回 conda 根 python 的 'X.Y' 版本串, 失败返回空串."""
        if not python_exe or not os.path.exists(python_exe):
            return ""
        try:
            p = subprocess.run(
                [python_exe, "-c", "import sys; print('%d.%d' % sys.version_info[:2])"],
                capture_output=True, text=True, timeout=20,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            return p.stdout.strip()
        except Exception:
            return ""

    def _conda_candidate_dirs(self):
        """返回所有可能的 conda 根目录候选 (未校验存在性, 已 normpath, 已去重).

        依据官方常见安装约定覆盖:
          Windows 仅当前用户: C:\\Users\\<用户>\\MinicondaX (X=2/3)
          Windows 所有用户:   C:\\ProgramData\\MinicondaX
          Windows winget:     C:\\Users\\<用户>\\Miniconda3
          Windows 隐藏:       %USERPROFILE%\\MinicondaX
          macOS/Linux:        ~/minicondaX, ~/miniconda, /opt/miniconda3
          旧版隐藏:           ~/.minicondaX, ~/.miniforgeX
        含 anaconda / miniforge 各变体及 Python 主版本 2/3 后缀。
        """
        import glob
        prof = os.environ.get("USERPROFILE") or os.path.expanduser("~")
        local = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~/AppData/Local")
        progdata = os.environ.get("ProgramData") or "C:/ProgramData"
        progfiles = os.environ.get("ProgramFiles") or "C:/ProgramFiles"

        candidates = [
            # Windows: 仅当前用户 / winget / 隐藏 (X = 2 或 3)
            f"{prof}/miniconda3", f"{prof}/Miniconda3",
            f"{prof}/miniconda2", f"{prof}/Miniconda2",
            f"{prof}/anaconda3", f"{prof}/Anaconda3",
            f"{prof}/miniforge3", f"{prof}/Miniforge3",
            # Windows: 所有用户 (ProgramData)
            f"{progdata}/miniconda3", f"{progdata}/Miniconda3",
            f"{progdata}/miniconda2", f"{progdata}/Miniconda2",
            f"{progdata}/anaconda3", f"{progdata}/Anaconda3",
            # Windows: AppData(Local) / ProgramFiles
            f"{local}/miniconda3", f"{local}/Miniconda3", f"{local}/anaconda3",
            f"{progfiles}/miniconda3", f"{progfiles}/Miniconda3", f"{progfiles}/anaconda3",
            # macOS / Linux
            "~/miniconda3", "~/miniconda2", "~/miniconda",
            "~/.miniconda3", "~/.miniconda2", "~/.miniconda",
            "/opt/miniconda3", "/opt/miniconda",
            "~/miniforge3", "~/.miniforge3", "~/anaconda3",
            # 常见非系统盘
            "D:/miniconda3", "D:/Miniconda3", "D:/anaconda3",
            "E:/miniconda3", "E:/Miniconda3", "E:/anaconda3",
            "F:/miniconda3", "G:/miniconda3",
        ]
        # 通配: 用户目录 / AppData / ProgramData 下 miniconda*/anaconda*/miniforge* (大小写)
        for pat in ("miniconda*", "Miniconda*", "anaconda*", "Anaconda*",
                    "miniforge*", "Miniforge*"):
            for base in (prof, local, progdata, "~"):
                for d in glob.glob(os.path.expanduser(os.path.join(base, pat))):
                    candidates.append(d)
        # 去重并保持优先顺序
        seen, out = set(), []
        for c in candidates:
            if not c:
                continue
            n = os.path.normpath(c)
            if n not in seen:
                seen.add(n)
                out.append(n)
        return out

    def _detect_existing_conda_dir(self) -> str:
        """探测系统已装的 conda 根目录 (返回已 normpath 路径).

        查找优先级 (对应官方推荐):
          1) `conda info --base` —— conda 自带最快查询, 直接给出 base 根;
          2) 扫描候选安装目录 (官方默认路径表) 找含 python.exe 的 conda 根;
          3) 常见根目录下的有限深度文件系统搜索 (conda 命令不可用时兜底)。
        不限定 Python 版本, 客户已装的兼容 conda 同样识别复用。
        """
        # 1. 最快: conda info --base (文档推荐方法一, 仅 PATH 查询, 不递归)
        conda = self._conda_exe_via_path()
        if conda:
            try:
                r = subprocess.run(
                    [conda, "info", "--base"],
                    capture_output=True, text=True, encoding="utf-8",
                    errors="replace", timeout=30,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                if r.returncode == 0:
                    for line in (r.stdout or "").splitlines():
                        base = line.strip()
                        if base and os.path.isdir(base):
                            return os.path.normpath(base)
            except Exception:
                pass
        # 2. 候选目录扫描 (官方默认路径表)
        for base in self._conda_candidate_dirs():
            if os.path.isdir(base) and os.path.exists(os.path.join(base, "python.exe")):
                return base
        # 3. 有限深度文件系统兜底搜索
        return self._conda_filesystem_scan()

    def _conda_filesystem_scan(self) -> str:
        """兜底: 在常见根目录有限深度内搜索 conda 根 (含 python.exe).

        仅当 `conda info --base` 与候选目录都失败时调用; 不做全盘递归 (避免
        C: 全盘慢扫), 仅扫用户目录 / ProgramData / AppData / 各盘根及
        C:/Users/<用户>/ 下一层, 匹配 miniconda*/anaconda*/miniforge*。
        """
        import glob
        prof = os.environ.get("USERPROFILE") or os.path.expanduser("~")
        local = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~/AppData/Local")
        progdata = os.environ.get("ProgramData") or "C:/ProgramData"
        pats = ("miniconda*", "Miniconda*", "anaconda*", "Anaconda*",
                "miniforge*", "Miniforge*")
        seen = set()

        def _check(d):
            d = os.path.normpath(d)
            if d in seen:
                return None
            seen.add(d)
            if os.path.isdir(d) and os.path.exists(os.path.join(d, "python.exe")):
                return d
            return None

        # 已知用户目录 / ProgramData / AppData (往往就在根的下一层)
        for root in (prof, local, progdata):
            if not os.path.isdir(root):
                continue
            for pat in pats:
                for d in glob.glob(os.path.join(root, pat)):
                    r = _check(d)
                    if r:
                        return r
        # 各盘根 + C:/Users/<用户> 下一层 (覆盖自定义安装位置)
        for root in ("C:/", "D:/", "E:/", "F:/", "G:/", "C:/Users"):
            if not os.path.isdir(root):
                continue
            for pat in pats:
                for d in glob.glob(os.path.join(root, pat)):
                    r = _check(d)
                    if r:
                        return r
            try:
                for sub in os.listdir(root):
                    subd = os.path.join(root, sub)
                    if not os.path.isdir(subd):
                        continue
                    for pat in pats:
                        for d in glob.glob(os.path.join(subd, pat)):
                            r = _check(d)
                            if r:
                                return r
            except Exception:
                pass
        return ""

    def _conda_exe_via_path(self) -> str:
        """仅从 PATH 查找 conda 可执行文件 (conda.exe / conda.bat).

        轻量、无递归依赖, 供 _find_conda / _detect_existing_conda_dir /
        _conda_env_exists 复用, 避免与目录扫描互相递归。
        """
        try:
            p = subprocess.run(
                ["where", "conda"], capture_output=True, text=True, timeout=15,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            if p.returncode == 0:
                exe = bat = ""
                for line in p.stdout.splitlines():
                    line = line.strip()
                    low = line.lower()
                    if low.endswith("conda.exe") and not exe:
                        exe = line
                    elif low.endswith("conda.bat") and not bat:
                        bat = line
                if exe:
                    return exe
                if bat:
                    return bat
        except Exception:
            pass
        return ""

    def _find_conda(self) -> str:
        """探测 conda 可执行文件的绝对路径.

        优先 PATH 中的 conda.exe/conda.bat, 其次扫描候选安装目录的
        Scripts\\conda.exe 与 condabin\\conda.bat. 适配不同用户目录与任意
        安装位置. 不直接调用 _detect_existing_conda_dir, 避免互相递归。
        """
        # 1. PATH
        p = self._conda_exe_via_path()
        if p:
            return p
        # 2. 扫描候选安装目录
        for base in self._conda_candidate_dirs():
            if not os.path.isdir(base):
                continue
            exe = os.path.join(base, "Scripts", "conda.exe")
            if os.path.exists(exe):
                return exe
            bat = os.path.join(base, "condabin", "conda.bat")
            if os.path.exists(bat):
                return bat
        return ""







    def _on_restart_port(self):
        """使用 pnputil 重置所有 COM 端口硬件驱动"""
        self._status_bar.showMessage("正在重置所有 COM 端口 ...")
        QApplication.processEvents()

        ok, msg = restart_all_com_devices()
        if ok:
            self._log(f"COM 端口已重置:\n{msg}", "ok")
        else:
            self._log(f"COM 端口重置失败:\n{msg}", "error")

        self._status_bar.showMessage("就绪 - 请选择串口并连接")
        self._refresh_ports()

    def _on_export_calib(self):
        """导出主臂(leader)和从臂(follower)的校准值 JSON"""
        if not self._ctrl:
            return

        # 0. 导出前暂停控制循环, 避免与串口读取冲突导致 homing 读取失败
        #    记录导出前状态: was_active=是否有控制循环, was_running=是否写从臂
        was_active = self._ctrl.state in (CtrlState.ARMED, CtrlState.RUNNING, CtrlState.PAUSED)
        was_running = self._ctrl.state in (CtrlState.RUNNING, CtrlState.PAUSED)
        if was_active:
            self._ctrl.stop()

        # 1. 读取每个舵机当前 Homing_Offset (寄存器 31, int12 带符号)
        #    lerobot 的 homing_offset 语义即为舵机 Homing_Offset,
        #    每个舵机独立, 值各不相同, 从寄存器 31 直接读取最准确.
        def _read_homing_with_retry(bus, sid: int, retries: int = 5):
            """读取 homing, 失败时重试 (避免瞬时串口冲突)"""
            for _ in range(retries):
                v = bus.read_homing_offset(sid)
                if v is not None:
                    return v
                time.sleep(0.1)
            return None

        master_homing = {}
        slave_homing = {}
        for j in self._ctrl.joints:
            mh = _read_homing_with_retry(self._ctrl.master_bus, j.master_id)
            sh = _read_homing_with_retry(self._ctrl.slave_bus, j.slave_id)
            # 注: 偏移校正面板已移除, 导出直接采用舵机实时 Homing 读数
            master_homing[j.master_id] = mh if mh is not None else 0
            slave_homing[j.slave_id] = sh if sh is not None else 0
            # 诊断日志: 显示每次读取结果 (None 表示读取失败)
            self._log(
                f"导出读取 Homing_Offset: ID{j.master_id} 主={mh} 从={sh}",
                "warn" if mh is None or sh is None else "info")

        # 2. 读取舵机 EEPROM 中实际生效的位置限位 (Min/Max_Position_Limit, 地址9/11)
        #    替代原先的 raw 移动记录 (实时拖动历史极值), 导出=舵机真实限位配置.
        def _read_limits_with_retry(bus, servo_id):
            for _ in range(5):
                try:
                    r = bus.read_position_limits(servo_id)
                    if r is not None:
                        return r
                except Exception:
                    pass
                time.sleep(0.1)
            return None

        master_range = {}
        slave_range = {}
        skip_ids = []
        for i, j in enumerate(self._ctrl.joints):
            mid = j.master_id
            sid = j.slave_id
            mr = _read_limits_with_retry(self._ctrl.master_bus, mid)
            sr = _read_limits_with_retry(self._ctrl.slave_bus, sid)
            if mr is not None:
                master_range[mid] = mr
            else:
                skip_ids.append(f"主{mid}")
            if sr is not None:
                slave_range[sid] = sr
            else:
                skip_ids.append(f"从{sid}")
        if skip_ids:
            self._log(
                "导出校准: 以下舵机读取限位失败, 未写入: " + ", ".join(skip_ids),
                "warn")

        # 3. 生成两个 JSON 数据
        leader_data = export_calibration_file(
            self._ctrl.joints, master_homing, master_range)
        follower_data = export_calibration_file(
            self._ctrl.joints, slave_homing, slave_range)

        # 3.5 同步写入舵机 EEPROM, 使舵机寄存器值与校准文件一致.
        #     lerobot 的 is_calibrated 会读舵机 Min/Max_Position_Limit(9/11)
        #     与 Homing_Offset(31) 跟校准文件比对, 不一致即判定未校准, 触发
        #     交互式校准 (无 stdin 时 EOFError). 因此导出时把 range 和 homing
        #     写回舵机, 确保寄存器值与即将写入的校准文件一致.
        #     仅遍历第2步成功读到限位的舵机, 读取失败的跳过 (其值本就未变).
        self._log("同步写入舵机 EEPROM (range + homing), 使寄存器值与校准文件一致...", "info")
        for j in self._ctrl.joints:
            mid = j.master_id
            sid = j.slave_id
            if mid in master_range:
                mmin = int(master_range[mid][0])
                mmax = int(master_range[mid][1])
                self._ctrl.master_bus.write_position_limits(mid, mmin, mmax)
            if sid in slave_range:
                smin = int(slave_range[sid][0])
                smax = int(slave_range[sid][1])
                self._ctrl.slave_bus.write_position_limits(sid, smin, smax)
            # 写 homing_offset 到 Homing_Offset (地址31, sign-magnitude)
            mh = master_homing.get(mid)
            sh = slave_homing.get(sid)
            if mh is not None:
                self._ctrl.master_bus.write_homing_offset(mid, int(mh))
            if sh is not None:
                self._ctrl.slave_bus.write_homing_offset(sid, int(sh))
        self._log("舵机 EEPROM 同步完成", "ok")

        # 4. 确定保存路径 (自动创建 lerobot 校准目录, 确保校准文件
        #    保存到 lerobot 期望的位置, 避免 "校准文件未找到/不匹配")
        #    兼容性: 同时写入所有候选目录 (so101_*/so_*/leader/follower),
        #    保证任意 lerobot 版本 / 任意电脑都能读到校准文件.
        leader_dirs, follower_dirs = _lerobot_calib_candidates()
        saved_paths = []
        try:
            for d in leader_dirs:
                d.mkdir(parents=True, exist_ok=True)
                p = d / "my_awesome_leader_arm.json"
                save_calib_json(p, leader_data)
                saved_paths.append(str(p))
            for d in follower_dirs:
                d.mkdir(parents=True, exist_ok=True)
                p = d / "my_awesome_follower_arm.json"
                save_calib_json(p, follower_data)
                saved_paths.append(str(p))

            self._log(
                f"校准值已导出到 {len(saved_paths)} 个候选目录", "ok")
            self._status_bar.showMessage(
                "校准值已导出到 lerobot 校准目录 (多版本兼容)")
            QMessageBox.information(
                self, "导出成功",
                "校准值已导出到 lerobot 校准目录 (多版本兼容):\n"
                + "\n".join(saved_paths))
        finally:
            # 导出完成后恢复之前的控制循环 (若之前有控制循环在运行)
            if was_active:
                self._ctrl.restart_loop(was_running)
            self._update_button_states()
            if was_running:
                self._status_bar.showMessage("导出完成 - 已恢复运行")

    def _on_apply_limits(self):
        """一键限位: 将表格中的 min/max 值写入主臂+从臂各舵机 EEPROM.

        注意: EEPROM 写会关断扭矩, 且必须在后台线程执行 (12 舵机 × 多次
        sleep 会阻塞数秒), 否则 GUI 卡死; 同时写前必须暂停 RUNNING 控制循环,
        避免后台 sync_write 与 EEPROM 解锁/锁定指令在总线上冲突导致写入失败.
        """
        if not self._ctrl:
            return
        reply = QMessageBox.question(
            self, "一键限位",
            "将把表格中每个舵机的 min/max 值写入主臂+从臂舵机 EEPROM。\n"
            "写 EEPROM 会关断所有舵机扭矩, 需重新使能。\n\n确定继续吗？",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        # 先把表格里的目标值解析好 (在后台线程启动前, 主线程读 UI 安全)
        targets = []
        for i, j in enumerate(self._ctrl.joints):
            def _parse(idx):
                txt = self._limit_table.item(i, idx).text().strip()
                if txt in ("", "--", "?"):
                    return None
                return int(txt)
            targets.append((j, _parse(1), _parse(2), _parse(3), _parse(4)))

        # 写 EEPROM 期间舵机短暂不响应读指令, 禁用捕获零点/运行等按钮,
        # 避免在此期间点击导致读取超时误报失败.
        self._apply_limit_btn.setEnabled(False)
        self._capture_btn.setEnabled(False)
        self._run_btn.setEnabled(False)
        self._log("一键限位: 正在后台写入 EEPROM...", "info")

        def _worker():
            # 写前暂停控制循环, 避免总线冲突
            was_running = self._ctrl.state == CtrlState.RUNNING
            if was_running:
                self._ctrl.pause()
            time.sleep(0.25)
            failed = []
            ok_count = 0
            try:
                for j, mm_min, mm_max, sm_min, sm_max in targets:
                    try:
                        if mm_min is not None and mm_max is not None:
                            self._ctrl.master_bus.write_position_limits(
                                j.master_id, mm_min, mm_max)
                        if sm_min is not None and sm_max is not None:
                            self._ctrl.slave_bus.write_position_limits(
                                j.slave_id, sm_min, sm_max)
                        ok_count += 1
                    except Exception as e:
                        failed.append(f"{j.name}: {e}")
                        self._log(f"一键限位 [{j.name}] 失败: {e}", "error")
            finally:
                # 写后恢复扭矩并恢复控制循环
                try:
                    self._ctrl.enable_all_torque()
                except Exception:
                    pass
                if was_running:
                    try:
                        self._ctrl.resume()
                    except Exception:
                        pass
            # 回到主线程刷新 UI
            self._apply_limit_btn.setEnabled(True)
            # 重算捕获零点/运行等按钮状态 (EEPROM 写期间曾临时禁用)
            self._update_button_states()
            if failed:
                self._log(
                    f"一键限位部分失败: 成功 {ok_count}/{len(targets)}, "
                    f"失败 {len(failed)}", "error")
                QMessageBox.critical(
                    self, "一键限位失败",
                    "以下关节写入失败:\n" + "\n".join(failed))
            else:
                self._log(
                    f"一键限位完成: 已写入主臂+从臂舵机 EEPROM ({ok_count}/{len(targets)})",
                    "ok")
                self._status_bar.showMessage("一键限位完成 - 请重新使能扭矩")

        threading.Thread(target=_worker, daemon=True).start()

    def _on_enable_torque(self):
        """使能扭矩"""
        if not self._ctrl:
            return
        try:
            self._ctrl.enable_all_torque()
            self._torque_enabled = True
            # 恢复控制循环, 从臂继续跟随
            if self._ctrl.state == CtrlState.PAUSED:
                self._ctrl.resume()
            self._log("所有舵机扭矩已使能", "ok")
            self._status_bar.showMessage("扭矩已使能 - 从臂继续跟随")
        except Exception as e:
            QMessageBox.critical(self, "扭矩使能失败", str(e))

    def _on_disable_torque(self):
        """失能扭矩: 暂停控制循环 + 释放所有舵机扭矩"""
        if not self._ctrl:
            return
        try:
            # 先暂停控制循环, 避免写位置导致舵机重新使能扭矩
            if self._ctrl.state == CtrlState.RUNNING:
                self._ctrl.pause()
            # 等待控制循环真正停止 (当前帧可能仍在写从臂位置)
            time.sleep(0.2)
            self._ctrl.disable_all_torque()
            self._torque_enabled = False
            self._log("所有舵机扭矩已失能, 从臂停止跟随", "warn")
            self._status_bar.showMessage("扭矩已失能 - 舵机可自由转动")
        except Exception as e:
            QMessageBox.critical(self, "扭矩失能失败", str(e))

    def _on_read_limits(self):
        """一键读取: 在后台线程读取舵机 EEPROM 限位值, 填回表格.
        任何状态下可用; 读取前暂停控制循环避免总线冲突, 读取后恢复."""
        if not self._ctrl:
            return
        self._log("一键读取: 正在后台读取舵机 EEPROM 限位值...", "info")

        def _worker():
            was_running = self._ctrl.state == CtrlState.RUNNING
            if was_running:
                self._ctrl.pause()
            time.sleep(0.25)
            master_results = {}
            slave_results = {}
            failed = []
            try:
                for j in self._ctrl.joints:
                    # 主臂总线
                    mr = None
                    for _ in range(5):
                        try:
                            mr = self._ctrl.master_bus.read_position_limits(j.master_id)
                            if mr is not None:
                                break
                        except Exception:
                            pass
                        time.sleep(0.1)
                    if mr is not None:
                        master_results[j.master_id] = (int(mr[0]), int(mr[1]))
                    else:
                        failed.append(f"主{j.master_id}")
                    # 从臂总线
                    sr = None
                    for _ in range(5):
                        try:
                            sr = self._ctrl.slave_bus.read_position_limits(j.slave_id)
                            if sr is not None:
                                break
                        except Exception:
                            pass
                        time.sleep(0.1)
                    if sr is not None:
                        slave_results[j.slave_id] = (int(sr[0]), int(sr[1]))
                    else:
                        failed.append(f"从{j.slave_id}")
            finally:
                if was_running:
                    try:
                        self._ctrl.resume()
                    except Exception:
                        pass
            # 回主线程刷新 UI
            self.read_limits_signal.emit(
                {"master_results": master_results,
                 "slave_results": slave_results, "failed": failed})

        threading.Thread(target=_worker, daemon=True).start()

    @Slot(dict)
    def _on_read_limits_done(self, data: dict):
        master_results = data.get("master_results", {})
        slave_results = data.get("slave_results", {})
        failed = data.get("failed", [])
        for i, j in enumerate(self._ctrl.joints if self._ctrl else []):
            mmin, mmax = master_results.get(j.master_id, (None, None))
            smin, smax = slave_results.get(j.slave_id, (None, None))
            if mmin is not None:
                self._limit_table.item(i, 1).setText(str(mmin))
                self._limit_table.item(i, 2).setText(str(mmax))
            else:
                self._limit_table.item(i, 1).setText("--")
                self._limit_table.item(i, 2).setText("--")
            if smin is not None:
                self._limit_table.item(i, 3).setText(str(smin))
                self._limit_table.item(i, 4).setText(str(smax))
            else:
                self._limit_table.item(i, 3).setText("--")
                self._limit_table.item(i, 4).setText("--")
        if failed:
            self._log(
                "一键读取: 部分舵机失败 - " + ", ".join(failed), "warn")
        else:
            self._log("一键读取完成: 已读入舵机 EEPROM 限位值", "ok")

    def _on_fill_04095(self):
        """(0,4095): 把表格中所有舵机的 min/max 填写为 (0, 4095)"""
        if not self._ctrl:
            return
        try:
            for i in range(NUM_JOINTS):
                self._limit_table.item(i, 1).setText("0")
                self._limit_table.item(i, 2).setText("4095")
                self._limit_table.item(i, 3).setText("0")
                self._limit_table.item(i, 4).setText("4095")
            self._log("已填写 (0,4095) 到限位表格, 点击'一键限位'可写入", "ok")
            self._status_bar.showMessage("已填写 (0,4095) - 点击'一键限位'写入")
        except Exception as e:
            self._log(f"填写失败: {e}", "error")

    def _on_fill_raw_limits(self):
        """实时raw限位值: 按实时 raw 值(min/max)填写限位表格"""
        if not self._ctrl:
            return
        try:
            for i, bar in enumerate(self._joint_bars):
                sid = i + 1
                # 主臂用 _m_max/_m_min, 从臂用 _s_max/_s_min
                # (实时 raw 移动记录, 反映实际拖动/跟随经过的极值)
                self._limit_table.item(i, 1).setText(
                    str(int(bar._m_min)) if bar._m_min is not None else "0")
                self._limit_table.item(i, 2).setText(
                    str(int(bar._m_max)) if bar._m_max is not None else "4095")
                self._limit_table.item(i, 3).setText(
                    str(int(bar._s_min)) if bar._s_min is not None else "0")
                self._limit_table.item(i, 4).setText(
                    str(int(bar._s_max)) if bar._s_max is not None else "4095")
            self._log("已按实时 raw 值(min/max)填写限位表格", "ok")
            self._status_bar.showMessage("已填写实时 raw 限位 - 点击'一键限位'写入")
        except Exception as e:
            self._log(f"填写实时限位失败: {e}", "error")

    def _update_fill_raw_btn(self):
        """根据实时 raw 值是否变化, 启用/禁用'实时raw限位值'按钮"""
        has_data = False
        for bar in self._joint_bars:
            if (bar._m_max is not None or bar._m_min is not None or
                    bar._s_max is not None or bar._s_min is not None):
                has_data = True
                break
        self._fill_raw_btn.setEnabled(self._connected and has_data)

    def _on_edit_center_targets(self):
        """弹出对话框编辑 1~6 号舵机软件中位的目标 raw 值."""
        dialog = QDialog(self)
        dialog.setWindowTitle("设置目标 raw 值")
        dlg_layout = QVBoxLayout(dialog)
        grid = QGridLayout()
        grid.setSpacing(8)
        spins = []
        for i, val in enumerate(self._center_target_values, 1):
            sp = QSpinBox()
            sp.setRange(0, 4095)
            sp.setValue(val)
            sp.setFixedWidth(80)
            sp.setToolTip(f"舵机 {i} 软件中位的目标 raw 值")
            grid.addWidget(QLabel(f"舵机 {i}"), (i - 1) // 3, ((i - 1) % 3) * 2)
            grid.addWidget(sp, (i - 1) // 3, ((i - 1) % 3) * 2 + 1)
            spins.append(sp)
        dlg_layout.addLayout(grid)
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(dialog.accept)
        btns.rejected.connect(dialog.reject)
        dlg_layout.addWidget(btns)
        if dialog.exec() == QDialog.Accepted:
            self._center_target_values = [sp.value() for sp in spins]
            self._log(f"软件中位目标值更新: {self._center_target_values}", "ok")

    def _on_software_center_all(self):
        """一键中位: 读取每个舵机当前位置作基准,
        写入 Homing_Offset 使该物理姿态对应的 raw 值 = 各舵机目标值.
        原理: Present = 物理位置 - Homing_Offset, 故 H_new = H_old + (P - target).
        即 lerobot 标准校准做法, 不依赖舵机 40=128 硬件指令, 仅改 homing 寄存器."""
        if not self._ctrl:
            return
        targets = list(self._center_target_values)
        total = NUM_JOINTS * 2

        def _read_pos(bus, sid):
            for _ in range(5):
                v = bus.read_position(sid)
                if v is not None:
                    return v
                time.sleep(0.1)
            return None

        reply = QMessageBox.question(
            self, "一键中位",
            f"提示: 将读取 1~6 号舵机(主臂+从臂共 {total} 个)当前位置作基准,\n"
            f"写入 Homing_Offset 使各舵机姿态对齐目标 raw 值:\n{targets}\n\n"
            f"把机器调整至初始位置\n\n"
            f"注意: 写 EEPROM 会关断扭矩, 完成后需重新使能。\n\n确定继续吗？",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return
        self._log(f"开始软件中位校准 (目标 raw={targets}) ...", "info")
        # 校准前暂停控制循环, 避免串口读写冲突
        if self._ctrl.state in (CtrlState.ARMED, CtrlState.RUNNING, CtrlState.PAUSED):
            self._ctrl.stop()
        ok_count = 0
        fail_list = []
        for sid in range(1, NUM_JOINTS + 1):
            for arm in ("master", "slave"):
                arm_txt = "主臂" if arm == "master" else "从臂"
                bus = (self._ctrl.master_bus if arm == "master"
                       else self._ctrl.slave_bus)
                target = targets[sid - 1]
                try:
                    P = _read_pos(bus, sid)
                    if P is None:
                        raise IOError("读取当前位置失败")
                    H_old = bus.read_homing_offset(sid)
                    if H_old is None:
                        raise IOError("读取当前 Homing 失败")
                    H_new = H_old + (P - target)
                    bus.write_homing_offset(sid, H_new, unlock_need=True)
                except Exception as e:
                    fail_list.append(f"{arm_txt}{sid}:{e}")
                    self._log(f"{arm_txt}舵机 ID {sid} 软件中位失败: {e}", "error")
                    continue
                ok_count += 1
                self._log(
                    f"{arm_txt}舵机 ID {sid} 校准完成: pos {P} + homing "
                    f"{H_old} → 新 homing {H_new} (目标 {target})", "ok")
        # 校准完成后不自动使能扭矩、不自动进入运行状态, 保持待机
        self._update_button_states()
        msg = f"软件中位校准完成: 成功 {ok_count}/{total}"
        if fail_list:
            msg += "\n\n失败项:\n" + "\n".join(fail_list)
            self._status_bar.showMessage("一键中位: 部分失败")
            QMessageBox.warning(self, "一键中位", msg)
        else:
            self._status_bar.showMessage(
                "一键中位: 全部完成 - 请手动使能扭矩后运行")
            QMessageBox.information(self, "一键中位", msg)

    def _on_capture_zero(self):
        """捕获零点基准"""
        if not self._ctrl:
            return
        self._log("正在读取零点基准...", "info")
        try:
            ok = self._ctrl.capture_zero()
        except Exception as e:
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "捕获零点异常",
                                 f"捕获零点时发生异常:\n{e}")
            return
        if ok:
            self._update_button_states()
            self._status_bar.showMessage("零点已捕获 - 可以开始运行")
            # 显示初始值 (原始值 raw) + 每个舵机 Homing_Offset 校准值
            # 同时缓存 Homing 值, 供导出校准值使用
            self._calib_homing_master = {}
            self._calib_homing_slave = {}
            for i, j in enumerate(self._ctrl.joints):
                md = self._ctrl.master_init[i]
                sd = self._ctrl.slave_init[i]
                # 读取每个舵机的零点偏移校准值 (地址31), 失败不影响主流程
                try:
                    mo = self._ctrl.master_bus.read_homing_offset(j.master_id)
                    so = self._ctrl.slave_bus.read_homing_offset(j.slave_id)
                    self._calib_homing_master[j.master_id] = mo if mo is not None else 0
                    self._calib_homing_slave[j.slave_id] = so if so is not None else 0
                    mo_txt = "?" if mo is None else str(mo)
                    so_txt = "?" if so is None else str(so)
                except Exception:
                    self._calib_homing_master[j.master_id] = 0
                    self._calib_homing_slave[j.slave_id] = 0
                    mo_txt = "?"
                    so_txt = "?"
                self._log(f"  {j.name}: 主={md}  从={sd}  偏差={sd - md}"
                          f"  [校准 主Homing={mo_txt} 从Homing={so_txt}]",
                          "info" if abs(sd - md) < 68 else "warn")

            # 保存零点原始值, 用于计算实时偏移量
            self._zero_m_deg = list(self._ctrl.master_init)
            self._zero_s_deg = list(self._ctrl.slave_init)

            # 捕获零点即开始新一轮移动记录, 清掉之前残留的 min/max 记录,
            # 确保 raw 面板重新从当前实际位置开始记录真实测量值.
            for bar in self._joint_bars:
                bar.reset_record()

            # 更新 raw 面板: 显示当前绝对位置 (原始值 0~4095)
            for i, bar in enumerate(self._joint_bars):
                if i < len(self._ctrl.master_init) and i < len(self._ctrl.slave_init):
                    bar.set_angles(self._ctrl.master_init[i], self._ctrl.slave_init[i])

            # 更新偏移面板: 刚捕获零点, 相对零点的偏移量为 0
            for bar in self._offset_bars:
                bar.set_angles(0.0, 0.0)

            # 启动后台循环 (仅实时读取显示, 不控制从臂)
            try:
                self._ctrl.start()
                self._log("已开始实时读取显示 (raw/偏移) - 点击运行后从臂才跟随", "ok")
                self._status_bar.showMessage("实时显示中 - 点击运行开始跟随")
            except Exception as e:
                self._log(f"启动实时显示失败: {e}", "error")
        else:
            err = getattr(self._ctrl, "last_error", "") or "请检查舵机连接"
            self._log(f"捕获零点失败: {err}", "error")
            QMessageBox.critical(self, "错误", f"读取零点失败: {err}")

    def _on_run(self):
        """开始控制: 让从臂跟随主臂"""
        if not self._ctrl:
            return
        try:
            # 控制循环线程已在捕获零点时启动, 此处仅开启从臂写控制
            if self._ctrl.state == CtrlState.ARMED:
                self._ctrl.enable_control()
            elif self._ctrl.state == CtrlState.PAUSED:
                self._ctrl.resume()
                self._ctrl.enable_control()
            self._update_button_states()
            self._status_bar.showMessage("● 运行中 - 增量映射已激活")
            self._log("增量控制已启动, 从臂开始跟随", "ok")
        except Exception as e:
            QMessageBox.critical(self, "启动失败", str(e))

    def closeEvent(self, event):
        """窗口关闭时断开连接"""
        if self._ctrl:
            try:
                self._ctrl.disconnect()
            except Exception:
                pass
        self._save_current_config()
        event.accept()

    # ============ 按钮状态管理 ============

    def _update_button_states(self):
        """根据当前状态切换按钮启用/禁用"""
        state = self._ctrl.state if self._ctrl else CtrlState.IDLE

        self._connect_btn.setEnabled(not self._connected)
        self._disconnect_btn.setEnabled(self._connected)
        self._restart_port_btn.setEnabled(not self._connected)
        # 使能/失能扭矩: 运行时或暂停时作为补充控制可用
        run_ctrl = state in (CtrlState.RUNNING, CtrlState.PAUSED)
        self._torque_btn.setEnabled(self._connected and run_ctrl)
        self._disable_torque_btn.setEnabled(self._connected and run_ctrl)

        # 捕获零点: 连接后 + 空闲状态即可用 (读取位置不依赖扭矩使能)
        self._capture_btn.setEnabled(
            self._connected and state in (CtrlState.IDLE,))

        self._run_btn.setEnabled(state == CtrlState.ARMED)

        # 一键中位: 连接后即可用 (校准 Homing, 与运行状态无关)
        self._center_btn.setEnabled(self._connected)
        self._center_target_btn.setEnabled(self._connected)

    # ============ 回调转 UI ============

    def _on_ctrl_state(self, state: CtrlState):
        self.state_signal.emit(state.value)

    def _on_ctrl_pose(self, master_deg: list[float], slave_deg: list[float]):
        self.pose_signal.emit(master_deg, slave_deg)

    @Slot(list, list)
    def _on_pose_update(self, master_raw: list[float], slave_raw: list[float]):
        for i, bar in enumerate(self._joint_bars):
            if i < len(master_raw) and i < len(slave_raw):
                # raw 面板: 显示舵机当前绝对位置 (原始值 raw, 0~4095)
                bar.set_angles(master_raw[i], slave_raw[i])

        # 偏移面板: 显示当前值相对零点的偏移量 (实时)
        for i, bar in enumerate(self._offset_bars):
            if i < len(master_raw) and i < len(slave_raw):
                dm = master_raw[i] - self._zero_m_deg[i] if i < len(self._zero_m_deg) else 0.0
                ds = slave_raw[i] - self._zero_s_deg[i] if i < len(self._zero_s_deg) else 0.0
                bar.set_angles(dm, ds)

        # raw 值有变化时启用"实时raw限位值"按钮
        self._update_fill_raw_btn()

    @Slot(int)
    def _on_state_change(self, state_val: int):
        state = CtrlState(state_val)
        state_colors = {
            CtrlState.IDLE:    ("未连接", "#aaa"),
            CtrlState.ARMED:   ("已就绪", "#3498db"),
            CtrlState.RUNNING: ("● 运行中", "#3498db"),
            CtrlState.PAUSED:  ("⏸ 已暂停", "#f39c12"),
            CtrlState.ESTOP:   ("⚠ 紧急停止", "#e74c3c"),
        }
        text, color = state_colors.get(state, (state.name, "#aaa"))
        self._state_label.setText(f"状态: {text}")
        self._state_label.setStyleSheet(
            f"QLabel {{ background-color: #333; color: {color}; "
            f"border-radius: 4px; padding: 6px; font-weight: bold; }}")
        self._update_button_states()

    @Slot(str)
    def _on_status_msg(self, msg: str):
        self._status_bar.showMessage(msg)

    def _log(self, msg: str, level: str = "info"):
        """添加日志行"""
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        colors = {"ok": "#2ecc71", "info": "#aaa", "warn": "#f39c12", "error": "#e74c3c"}
        color = colors.get(level, "#aaa")
        self._log_text.append(
            f"<span style='color:#888'>[{ts}]</span> "
            f"<span style='color:{color}'>{msg}</span>"
        )

    @Slot(int, float)
    def _on_loop_stat(self, count: int, ms: float):
        self._loop_label.setText(f"循环: {count} | 耗时: {ms:.1f} ms")

    # ============ 辅助 ============


# ============================
# 入口
# ============================

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # 暗色主题
    app.setStyleSheet("""
        QMainWindow, QWidget { background-color: #2b2b2b; color: #ddd; }
        QGroupBox {
            border: 1px solid #555; border-radius: 6px;
            margin-top: 10px; padding-top: 15px;
            font-weight: bold; color: #ddd;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px; padding: 0 5px;
        }
        QTableWidget {
            background-color: #333; gridline-color: #555;
            alternate-background-color: #383838;
        }
        QTableWidget::item:selected { background-color: #2c3e50; }
        QHeaderView::section {
            background-color: #444; color: #ddd;
            border: 1px solid #555; padding: 4px;
        }
        QComboBox, QSpinBox, QDoubleSpinBox {
            background-color: #333; color: #ddd;
            border: 1px solid #555; border-radius: 3px;
            padding: 3px 6px;
        }
        QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled {
            background-color: #222; color: #666;
        }
        QComboBox::drop-down { border: none; }
        QComboBox QAbstractItemView {
            background-color: #333; color: #ddd;
            selection-background-color: #2c3e50;
        }
        QPushButton {
            background-color: #444; color: #ddd;
            border: 1px solid #555; border-radius: 4px;
            padding: 5px 12px;
        }
        QPushButton:hover { background-color: #555; }
        QPushButton:disabled {
            background-color: #333; color: #666;
            border-color: #444;
        }
        QCheckBox { color: #ddd; spacing: 5px; }
        QCheckBox:disabled { color: #666; }
        QLabel { color: #ddd; }
        QStatusBar { background-color: #1e1e1e; color: #aaa; }
        QTextEdit {
            background-color: #1e1e1e; color: #ccc;
            border: 1px solid #444; border-radius: 3px;
        }
        QSplitter::handle { background-color: #555; }
        QSplitter::handle:horizontal { width: 3px; }
        QSplitter::handle:vertical { height: 3px; }
    """)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
