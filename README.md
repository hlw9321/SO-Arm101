# SO-ARM101 Control Suite

> English documentation below; 中文说明见文末。

## Download

**[⬇ Download SO-ARM101.exe (Windows x64)](https://github.com/hlw9321/SO-Arm101/releases/latest/download/SO-ARM101.exe)**
— no Python required to run the GUI. Teleoperation still needs a local
`lerobot` environment (see *Quick Start* below).

## Overview

**SO-ARM101 Control Suite** is an open-source Windows toolkit for operating the
low-cost **SO-ARM101** robotic arm (SO101 leader + follower). It provides:

- A PySide6 GUI (`python_app/`) that **auto-detects** your `lerobot` Python
  environment and launches `lerobot.teleoperate` for teleoperation.

## Features

- **Robust environment detection** — finds `lerobot` whether it was
  auto-installed, manually installed, or even installed into the **base** conda
  environment (the most common manual-install pitfall).
- **Bounded full-disk scanner** — runs on a background thread, never blocks the
  UI, and is protected by timeout / max-depth / max-dirs limits so it can never
  hang or crash.
- **COM-port management** — one-click driver reset (`pnputil`) for the SO101
  leader (master) and follower (slave) arms.

## Requirements

- Windows 10 / 11
- Miniconda or Anaconda (recommended) — Python 3.10+
- `lerobot` (tested with 0.3.4) with the `feetech` extras

## Quick Start

1. Set up the `lerobot` environment (see *Manual setup* below).
2. Connect the SO101 **leader** (master) and **follower** (slave) arms via USB.
3. Launch the app: `python_app\run.bat` (or `python python_app\main_app.py`).
4. Click **[虚拟环境]** to auto-detect the environment and start teleoperation.

## Manual setup (without the installer)

```bash
conda create -n lerobot python=3.10
conda activate lerobot
pip install "lerobot[feetech]"
```

The scanner finds this environment automatically. Installing `lerobot` into the
**base** conda environment also works — the detector covers that case too.

## Build from source

- GUI application: `python_app\build_exe.bat`
  (PyInstaller, `python_app\SO-ARM101.spec`)

## Project layout

```
SO-Arm101/
├─ python_app/              # PySide6 GUI + teleoperation launcher
│  ├─ main_app.py           # main window, env detection, teleop launch
│  ├─ delta_controller.py   # arm / servo control
│  ├─ scs_protocol.py       # SCS servo protocol
│  ├─ diag.py               # diagnostics
│  ├─ SO-ARM101.spec        # PyInstaller spec for the app
│  └─ requirements*.txt
```

## Notes

- `build\`, `dist\`, `__pycache__`, `*.pyc` and `*.whl` are build artifacts and
  are excluded via `.gitignore`.
- Hardware / wiring details depend on your specific SO-ARM101 kit; see
  `python_app\delta_arm_config.json` for the arm configuration used by the app.

## License

Apache-2.0 — see [LICENSE](LICENSE).

---

## 中文说明

### 下载

**[⬇ 下载 SO-ARM101.exe（Windows x64）](https://github.com/hlw9321/SO-Arm101/releases/latest/download/SO-ARM101.exe)**
— 运行 GUI 无需安装 Python；遥操作仍需本机已装好 `lerobot` 环境（见"快速开始"）。

### 简介

**SO-ARM101 Control Suite** 是一套用于操控低成本 **SO-ARM101** 机械臂
（SO101 主臂 leader + 从臂 follower）的开源 Windows 工具集，包含：

- 一个 PySide6 图形界面（`python_app/`），可**自动识别**你的 `lerobot` Python
  环境，并启动 `lerobot.teleoperate` 进行遥操作。

### 特性

- **环境识别稳健**：无论自动安装、手动安装，还是把 `lerobot` 装进了 conda
  的 **base** 环境（手动安装最常见的坑），都能识别。
- **有界全盘扫描**：在后台线程执行，不阻塞界面，并有超时 / 最大深度 / 最大目录数
  三重保护，不会卡死或崩溃。
- **串口管理**：一键重置 SO101 主/从臂的驱动（`pnputil`）。

### 环境要求

- Windows 10 / 11
- Miniconda 或 Anaconda（推荐）— Python 3.10+
- `lerobot`（已用 0.3.4 测试）并安装 `feetech` 扩展

### 快速开始

1. 先准备好 `lerobot` 环境（见下方"手动安装"）。
2. 用 USB 连接 SO101 **主臂**（master）与**从臂**（slave）。
3. 启动程序：`python_app\run.bat`（或 `python python_app\main_app.py`）。
4. 点击 **[虚拟环境]** 自动识别环境并启动遥操作。

### 手动安装（不使用安装程序）

```bash
conda create -n lerobot python=3.10
conda activate lerobot
pip install "lerobot[feetech]"
```

扫描器会自动找到该环境。把 `lerobot` 装进 conda 的 **base** 环境同样可行。

### 从源码构建

- 主程序：`python_app\build_exe.bat`（PyInstaller，`python_app\SO-ARM101.spec`）

### 许可证

Apache-2.0 — 详见 [LICENSE](LICENSE)。
