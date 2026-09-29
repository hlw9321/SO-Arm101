@echo off
chcp 65001 >nul
REM ============================================================
REM  SO-ARM101.exe 打包脚本
REM  用法: 在 conda(base) 激活状态下双击或调用本脚本
REM  例:  conda activate base
REM        python_app\build_exe.bat
REM  脚本会自动确保打包依赖 (PySide6/pyserial/pyinstaller) 已安装,
REM  避免分发时 exe 因缺少依赖而无法启动.
REM ============================================================
setlocal
cd /d "%~dp0"

set "PY=%CONDA_PREFIX%\python.exe"
if not exist "%PY%" set "PY=%~dp0..\..\.conda\python.exe"
if not exist "%PY%" set "PY=python.exe"

echo [build] 确保打包依赖已安装 (PySide6 / pyserial / pyinstaller)...
REM 优先用项目 wheels_build/ 目录离线安装 (无网也可打包); 缺 wheel 时回退在线安装.
"%PY%" -m pip install -r "%~dp0requirements_build.txt" --no-index --find-links "%~dp0..\wheels_build" >nul 2>nul
if errorlevel 1 (
  echo [build] 本地 wheels 不全, 回退在线安装...
  "%PY%" -m pip install -r "%~dp0requirements_build.txt"
  if errorlevel 1 (
    echo [build] ERROR: 安装打包依赖失败
    pause
    exit /b 1
  )
)

REM PyInstaller 重新打包时会删旧 exe, 可能被安全软件(Safe-delete)拦截.
REM 先手动移走旧 exe 以规避.
if exist "dist\SO-ARM101.exe" (
  echo [build] 移走旧 exe 以规避 Safe-delete 拦截...
  move /Y "dist\SO-ARM101.exe" "dist\SO-ARM101_old_%RANDOM%.exe" >nul 2>nul
)

echo [build] 开始 PyInstaller 打包...
"%PY%" -m PyInstaller SO-ARM101.spec --noconfirm
if errorlevel 1 (
  echo [build] ERROR: PyInstaller 打包失败
  pause
  exit /b 1
)

echo [build] 打包完成: dist\SO-ARM101.exe
endlocal
