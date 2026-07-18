@echo off
REM Flash TLE5012B (MT5012) firmware to ESP32-S3
REM Aligned with oez\esp32_can_servo\ESP32_本地烧录方法与注意事项_2026-07-18.md
REM Run from a normal PowerShell/cmd (not Cursor sandbox).

REM Do NOT set ARDUINO_DATA_DIR — it breaks Platform 'esp32:esp32' lookup.
set ARDUINO_DATA_DIR=

set CLI=D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe
set CONFIG=C:\Users\tt\.arduinoIDE\arduino-cli.yaml
set SKETCH=D:\oezcon\mcoder\ESP32_MT5012\firmware\ESP32ReadMT5012
set PORT=COM18
set FQBN=esp32:esp32:esp32s3

echo === Compile (FQBN=%FQBN%) ===
"%CLI%" compile --fqbn %FQBN% "%SKETCH%" --config-file "%CONFIG%"
if errorlevel 1 exit /b 1

echo === Upload to %PORT% ===
"%CLI%" upload --fqbn %FQBN% -p %PORT% "%SKETCH%" --config-file "%CONFIG%"
if errorlevel 1 exit /b 1

echo === Done. Serial monitor 115200 (close other apps first; leave DTR/RTS off) ===
"%CLI%" monitor -p %PORT% -c baudrate=115200 --config-file "%CONFIG%"
