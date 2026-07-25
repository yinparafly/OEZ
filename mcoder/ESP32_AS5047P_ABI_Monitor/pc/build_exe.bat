@echo off
cd /d "%~dp0"
echo [1/2] pip install pyinstaller + deps...
python -m pip install -q -r requirements.txt pyinstaller
if errorlevel 1 exit /b 1

echo [2/2] build one-file AbiRpmMonitor.exe ...
python -m PyInstaller --noconfirm --clean AbiRpmMonitor.spec
if errorlevel 1 exit /b 1

if exist "dist\AbiRpmMonitor.exe" (
  copy /Y "dist\AbiRpmMonitor.exe" "..\..\AbiRpmMonitor-PC.exe" >nul
  echo.
  echo OK: dist\AbiRpmMonitor.exe
  echo OK: E:\OEZCON\mcoder\AbiRpmMonitor-PC.exe
) else (
  echo BUILD FAILED: exe missing
  exit /b 1
)
pause
