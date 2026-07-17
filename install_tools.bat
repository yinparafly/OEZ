@echo off
echo ========================================
echo Installing Hyper-V Management Tools
echo ========================================
echo.
echo Running DISM to enable Hyper-V Management Tools...
echo.

dism /online /enable-feature /featurename:Microsoft-Hyper-V-Tools-All /all /norestart

echo.
echo ========================================
echo Done. Check if module is available now.
echo ========================================
pause
