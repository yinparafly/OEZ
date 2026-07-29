@echo off
REM Flash firmware via ST-Link V2 using OpenOCD
REM Install OpenOCD first:
REM   winget install xpack-dev-tools.openocd-xpack --location "D:\openocd"

set OPENOCD=D:\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe
if not exist "%OPENOCD%" (
    echo OpenOCD not found at %OPENOCD%
    exit /b 1
)

"%OPENOCD%" -f interface/stlink.cfg -c "transport select swd" -f target/stm32f1x.cfg -c "program firmware.hex verify reset exit"
if %ERRORLEVEL% neq 0 (
    echo Flash failed. Check ST-Link connection.
    exit /b 1
)
echo Flash OK
