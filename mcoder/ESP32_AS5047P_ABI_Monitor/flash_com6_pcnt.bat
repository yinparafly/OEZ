@echo off
REM Flash ESP32AbiMonitor_PCNT (A+I rising-edge, no quadrature) to ESP32-S3 on COM6
set CLI=D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe
if not exist "%CLI%" set CLI=D:\Program Files\Arduino CLI\arduino-cli.exe
if not exist "%CLI%" set CLI=D:\program\ArduinoCLI\arduino-cli.exe
set CONFIG=D:\Arduino\.arduinoIDE\arduino-cli.yaml
set SKETCH=D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\firmware\ESP32AbiMonitor_PCNT
set PORT=COM18
set FQBN=esp32:esp32:esp32s3:PSRAM=opi,USBMode=default,CDCOnBoot=default

echo === Compile PCNT A+I (FQBN=%FQBN%) ===
"%CLI%" compile --fqbn %FQBN% "%SKETCH%" --config-file "%CONFIG%"
if errorlevel 1 exit /b 1

echo === Upload to %PORT% ===
"%CLI%" upload --fqbn %FQBN% -p %PORT% "%SKETCH%" --config-file "%CONFIG%"
if errorlevel 1 exit /b 1

echo === Done (PCNT A+I version) ===
exit /b 0
