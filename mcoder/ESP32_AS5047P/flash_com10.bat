@echo off
REM Flash ESP32ReadAS5047P to ESP32-S3 on COM10 (user plc machine)
set ARDUINO_DATA_DIR=
set CLI=D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe
set CONFIG=C:\Users\plc\.arduinoIDE\arduino-cli.yaml
set SKETCH=E:\OEZCON\mcoder\ESP32_AS5047P\firmware\ESP32ReadAS5047P
set PORT=COM10
set FQBN=esp32:esp32:esp32s3

echo === Compile (FQBN=%FQBN%) ===
"%CLI%" compile --fqbn %FQBN% "%SKETCH%" --config-file "%CONFIG%"
if errorlevel 1 exit /b 1

echo === Upload to %PORT% ===
"%CLI%" upload --fqbn %FQBN% -p %PORT% "%SKETCH%" --config-file "%CONFIG%"
if errorlevel 1 exit /b 1

echo === Done ===
exit /b 0
