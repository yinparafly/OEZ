@echo off
echo ========================================
echo Download Windows 11 ISO
echo ========================================
echo.

if not exist "D:\ISOs" mkdir "D:\ISOs"

echo Checking for existing ISO...
dir "D:\ISOs\*.iso" >nul 2>&1
if %errorLevel% equ 0 (
    echo ISO already exists!
    dir "D:\ISOs\*.iso"
    echo.
    echo Run install_win11.bat next.
    pause
    exit /b 0
)

echo No ISO found.
echo.
echo Please download manually:
echo 1. Open: https://www.microsoft.com/software-download/windows11
echo 2. Select: Windows 11 Disk Image (ISO) for x64 devices
echo 3. Select: Chinese Simplified  
echo 4. Save to: D:\ISOs\
echo.
echo After download, run: install_win11.bat
pause
