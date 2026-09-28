@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ========================================
echo   SO-ARM101 增量控制测试仪
echo ========================================
echo.

:: 检查 Python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未找到 Python, 请先安装 Python 3.9+
    pause
    exit /b 1
)

:: 安装依赖
echo [1/2] 检查依赖...
pip install -r requirements.txt -q
if %errorlevel% neq 0 (
    echo [警告] pip 安装失败, 请手动执行: pip install pyserial PySide6
)

:: 启动
echo [2/2] 启动 GUI...
echo.
python main_app.py

pause
