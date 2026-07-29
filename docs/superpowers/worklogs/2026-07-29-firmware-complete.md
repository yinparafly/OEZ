# Work Log: STM32F103 ABZ Encoder Data Recorder

**Date:** 2026-07-29
**Board:** STM32F103C8T6 (Blue Pill)
**Objective:** Complete firmware + PC tool for AS5047P ABZ encoder data recording

## Tasks Completed

### Task 1-2: Scaffolding + LED Blink
- Created project structure: Makefile, linker.ld, CMSIS + StdPeriph copied
- System clock 72MHz (PLL HSE 8MHz × 9), PC13 LED blink
- Fixed `core_cm3.c` STREX earlyclobber for GCC 13

### Task 3: TIM2 Encoder Driver
- PA0/PA1 encoder mode, 32-bit period via direct ARR write
- Fix: `TIM_Period` capped at 16-bit, use `*(uint32_t*)&TIM2->ARR = 0xFFFFFFFF`

### Task 4: USART1 + TIM3 4kHz + Ring Buffer
- USART1 115200 (PA9/PA10), renamed to `USART1_Init` to avoid collision
- TIM3 at 4kHz (APB1=36MHz, prescaler=0, ARR=8999)
- Ring buffer 3584 entries × uint32_t = 14KB

### Task 5: Recorder State Machine
- State machine: IDLE → MONITOR → TRIGGERED → WRITING
- EXTI0 PB0 Z signal (later moved to PB10)
- IWDG ~1.28s, SysTick ms, RPM calculation every 50ms
- Hysteresis: trigger at RPM>12, clear at RPM<8

### Task 6: SPI1 SD Card + FatFS R0.08a
- SPI1 at PA5-7, CS on PA4
- FatFS R0.08a files from reference project (eziya/STM32_SPI_SDCARD)
- Fixed CS management (move to caller), SDHC/SDSC sector addressing
- `Record_SaveToSD` writes directly from ring buffer (no 13KB stack array)
- `g_fs_mounted` flag, mount once
- Reference projects downloaded: eziya, SDEC, RPM-Counter, NimaMX, datalogger

### Task 7: INDEX.TXT + CSV Naming
- INDEX.TXT read on startup, parse decimal counter
- 4-digit zero-padded CSV filenames: `0001.CSV`–`9999.CSV`
- Increment after each successful save

### Task 8: Startup Delay + PB1 Trigger + Integration
- 3-second startup delay, PB1 debounced trigger input
- GPIO trigger + RPM+Z trigger → same Recorder_Trigger()
- Fixed LED blink timing bug (g_startup_ticks frozen in IDLE state)

### LED Reassignment + PC UI
- PB0=Green LED, PB1=Red LED (based on reference STM32F103-SDCARD)
- Z signal moved to PB10 (EXTI10), start trigger on PB11
- New serial commands: `1`/`2`/`3`/`4` for LED on/off, `g`/`r` for toggle
- PC UI (pc_ui.py): CSV viewer + serial plotter + LED control + Start trigger
- Completion pattern: green LED fast blink 2x + slow blink 1x

### Environment
- OpenOCD 0.12.0 installed to D:\openocd via winget
- Flashing via ST-Link V2 at 950kHz SWD
- Toolchain: arm-none-eabi-gcc 13.2.1 at D:\arm-official

## Build Results

| Date | text | data | bss | dec | Status |
|------|------|------|-----|-----|--------|
| 2026-07-28 (Task 1) | 984 | 0 | 1024 | 2008 | Initial |
| 2026-07-28 (Task 5) | 10516 | 12 | 14976 | 26504 | State machine |
| 2026-07-29 (Final) | 11584 | 12 | 14980 | 26576 | All features |

## Issues & Resolutions
- `TIM_Period` capped at 16-bit in StdPeriph → use direct register write
- `USART_Init` name collision → rename to `USART1_Init`
- IWDG resets during flash → use `program` command instead of manual halt+flash
- LED blink in IDLE/TRIGGERED broken (frozen counter) → add `g_led_tick`
- OpenOCD connection at 100kHz fails → use default 950kHz
