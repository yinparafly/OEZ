@echo off
echo ========================================
echo Step 1: Enable Hyper-V Management Tools
echo ========================================
echo.
echo This will enable Hyper-V PowerShell module.
echo You may need to restart after this.
echo.
echo Press any key to start...
pause >nul

powershell -Command "Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-PowerShell -All -NoRestart"

echo.
echo ========================================
echo Step 2: Check if module is available
echo ========================================
echo.

powershell -Command "Import-Module Hyper-V -ErrorAction SilentlyContinue; if (Get-Command Get-VM -ErrorAction SilentlyContinue) { Write-Host 'Hyper-V module OK!' } else { Write-Host 'Module still not available. You may need to restart.' }"

echo.
echo ========================================
echo If module is OK, run install_win11.bat
echo If not, restart computer first
echo ========================================
pause
