@echo off
echo ========================================
echo Windows 11 安装脚本
echo 请右键以管理员身份运行
echo ========================================
echo.

REM 检查管理员权限
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo 需要管理员权限!
    echo 请右键此文件 -^> 以管理员身份运行
    pause
    exit /b 1
)

echo 管理员权限: OK
echo.

REM 创建目录
if not exist "D:\ISOs" mkdir "D:\ISOs"
if not exist "D:\VMs\Win11" mkdir "D:\VMs\Win11"

REM 检查 ISO
dir "D:\ISOs\*.iso" >nul 2>&1
if %errorLevel% neq 0 (
    echo 未找到 Windows 11 ISO
    echo 请下载: https://www.microsoft.com/software-download/windows11
    echo 保存到: D:\ISOs\
    echo.
    echo 下载完成后重新运行此脚本
    pause
    exit /b 1
)

echo ISO 已找到
echo.

REM 运行 PowerShell 安装脚本
powershell -ExecutionPolicy Bypass -File "D:\oezcon\win11_install_admin.ps1"

echo.
echo 完成!
pause
