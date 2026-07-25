@echo off
cd /d "%~dp0"
python -m pip install -r requirements.txt -q
echo Starting with Bluetooth preferred...
python abi_monitor.py --ble %*
pause
