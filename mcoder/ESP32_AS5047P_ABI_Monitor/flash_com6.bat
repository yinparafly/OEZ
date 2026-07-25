@echo off
REM Flash ESP32AbiMonitor to ESP32-S3 on COM6
set CLI=D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe
if not exist "%CLI%" set CLI=D:\Program Files\Arduino CLI\arduino-cli.exe
if not exist "%CLI%" set CLI=D:\program\ArduinoCLI\arduino-cli.exe
set CONFIG=C:\Users\plc\.arduinoIDE\arduino-cli.yaml
set SKETCH=E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\firmware\ESP32AbiMonitor
set PORT=COM6
REM 本机 ESP32-S3 为 Embedded 8MB PSRAM → OPI；若启动后 spiram_free=0 再改 PSRAM=enabled
REM USB-OTG(TinyUSB)=可挂 U盘；串口仍走 CH343。勿用 USBMode=hwcdc（无 MSC）
set FQBN=esp32:esp32:esp32s3:PSRAM=opi,USBMode=default,CDCOnBoot=default

echo === Compile (FQBN=%FQBN%) ===
"%CLI%" compile --fqbn %FQBN% "%SKETCH%" --config-file "%CONFIG%"
if errorlevel 1 exit /b 1

echo === Upload to %PORT% ===
"%CLI%" upload --fqbn %FQBN% -p %PORT% "%SKETCH%" --config-file "%CONFIG%"
if errorlevel 1 exit /b 1

echo === Done ===
exit /b 0
