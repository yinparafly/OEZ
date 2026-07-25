@echo off
cd /d "%~dp0"
python -m pip install -r requirements.txt -q
echo Starting ABI Monitor (default BLE; use --usb for serial)...
python abi_monitor.py %*
pause
