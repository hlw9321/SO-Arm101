# -*- mode: python ; coding: utf-8 -*-


import os
import sys

# PyInstaller 执行 spec 时不提供 __file__, 但注入了 SPECPATH (spec 文件所在目录).
_PROJECT_ROOT = os.path.normpath(os.path.join(SPECPATH, ".."))

# conda python 的 C 扩展（如 _ctypes.pyd）依赖 Library\bin 下的 DLL,
# PyInstaller 默认不会收集，必须在 binaries 里显式加入.
# sys.executable 在 PyInstaller 执行 spec 时指向打包用的 Python (conda 根目录的 python.exe),
# 其所在目录就是 conda 根目录, Library\bin 在其下一级.
_CONDABIN = os.path.normpath(
    os.path.join(os.path.dirname(sys.executable), "Library", "bin")
)
# Python C 扩展依赖的 conda Library\bin DLL, 全部放到 exe 解压根目录,
# 这样 Windows 加载 _ctypes.pyd/_hashlib.pyd/_lzma.pyd/_bz2.pyd 等时能找到.
_EXTRA_BINARIES = []
for _dll in (
    "ffi.dll", "libffi-8.dll", "libffi-7.dll",
    "libcrypto-3-x64.dll", "libssl-3-x64.dll",
    "liblzma.dll", "LIBBZ2.dll",
):
    _fp = os.path.join(_CONDABIN, _dll)
    if os.path.exists(_fp):
        _EXTRA_BINARIES.append((_fp, "."))

# 环境安装已独立到 env_setup 程序 (SO-ARM101环境安装程序.exe),
# 主程序 exe 不再打包 miniconda / wheels / lerobot 等离线安装资源,
# 仅检测并使用已安装好的 lerobot venv 运行遥操作.
a = Analysis(
    ['main_app.py'],
    pathex=[],
    binaries=_EXTRA_BINARIES,
    # 把项目根目录的 lerobot 源码 (0.3.4) 与预烘焙的离线依赖 wheels 目录
    # 一并打包进 exe, 实现完全离线安装. 运行时二者位于 _MEIPASS/lerobot 与
    # _MEIPASS/wheels, 由 main_app._lerobot_src_dir / _lerobot_wheels_dir 解析.
    # 使用绝对路径确保 PyInstaller 能正确收集 (相对路径相对 cwd 易失效).
    datas=[
        (os.path.join(_PROJECT_ROOT, 'python_app', 'delta_arm_config.json'), '.'),
        (os.path.join(_PROJECT_ROOT, 'python_app', 'app_icon.ico'), '.'),
    ],
    # exe 自身运行只需 GUI 与串口依赖. 显式列出以兜底, 避免换机器/重装
    # conda 后 PyInstaller 动态分析漏抓导致 exe 启动报 ModuleNotFoundError.
    hiddenimports=[
        'PySide6',
        'PySide6.QtCore',
        'PySide6.QtGui',
        'PySide6.QtWidgets',
        'serial',
        'serial.serialwin32',
        'serial.tools',
        'serial.tools.list_ports',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # 排除运行遥操作不需要的重型/平台相关依赖, 缩小 exe 体积
        'torch', 'torchvision', 'tensorflow', 'jax',
        'cv2', 'opencv_python', 'transformers', 'diffusers',
        'wandb', 'rerun_sdk', 'gymnasium', 'pygame',
    ],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='SO-ARM101',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['app_icon.ico'],
)
