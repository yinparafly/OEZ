@echo off
echo ========================================
echo Windows 11 VM Install
echo ========================================
echo.
echo This script will:
echo 1. Create a Hyper-V virtual machine
echo 2. Install Windows 11 automatically
echo 3. Takes about 30-50 minutes
echo.
echo Your current Windows 10 will NOT be affected.
echo.
echo Press any key to start...
pause >nul

powershell -ExecutionPolicy Bypass -Command "Start-Process powershell -ArgumentList '-ExecutionPolicy Bypass -File D:\oezcon\win11_install_admin.ps1' -Verb RunAs"

echo.
echo If UAC prompt appeared, click YES.
echo Then wait for installation to complete.
echo.
echo Use vmconnect.exe localhost Win11 to view progress.
echo.
pause
