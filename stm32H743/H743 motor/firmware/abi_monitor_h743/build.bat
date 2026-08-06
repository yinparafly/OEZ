@echo off
rem ============================================================
rem  abi_monitor_h743 一键构建 + 烧录（STM32CubeProgrammer CLI）
rem  用法: build.bat         仅编译
rem        build.bat flash   编译并烧录（ST-Link SWD 四线）
rem ============================================================
setlocal
set PROJ=%~dp0
cd /d "%PROJ%"

set MAKE=mingw32-make
set CUBEPRG="D:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set BIN=%PROJ%build\abi_monitor_h743.bin

%MAKE% all
if errorlevel 1 (
  echo [ERROR] build failed
  exit /b 1
)

if /i "%1"=="flash" (
  echo [INFO] flashing via ST-Link SWD ...
  %CUBEPRG% -c port=SWD mode=HOTPLUG -w %BIN% 0x08000000 -v -rst
  if errorlevel 1 (
    echo [ERROR] flash failed - check ST-Link wiring: P1-1 DIO, P1-2 SWCLK, P1-3 GND, P1-4 5V
    exit /b 1
  )
  echo [OK] flashed and verified
)
exit /b 0
