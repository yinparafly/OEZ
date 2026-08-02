# AbiMonitor_TJX — TMS320F28P550 事件驱动测速记录器

Built on TMS320F28P550SJ9 (天机星 board). UTO-based encoder speed measurement + ring-buffered event recording.

## Quick Start

```powershell
# Build (no GUI needed)
Copy-Item -Recurse app/ "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"
cmd /c "D:\ti\ccs2011\ccs\eclipse\ccs-server-cli.bat -workspace D:\temp\opencode\ccs_ws -application com.ti.ccs.apps.buildProject -ccs.projects AbiMonitor_TJX -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport"

# Flash
& "D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe" load --config="targetConfigs\TMS320F28P550SJ9.ccxml" "CPU1_RAM\AbiMonitor_TJX.out"

# Verify
python -c "import serial; s=serial.Serial('COM23',921600,timeout=2); s.write(b'ABI?\r\n'); print(s.readline())"
```

## CLI Commands

| Command | Description |
|---|---|
| `PING` | Echo test |
| `FW?` | Firmware version |
| `SPD?` | RPM, gear, uptime (ms) |
| `ABI?` | Counts, index, ISR debug counters |
| `SNAP?` | Ring/snap buffer status |
| `MONITOR START` | Arm recording |
| `MONITOR STOP` | Force-stop recording |
| `REC MS 2000` | Set max recording duration |
| `SD INIT` | Initialize SD card |
| `SD TEST` | Write/read/verify round-trip |
| `SPI LB` | SPIB internal loopback test |
| `QFLG/QEINT/QCTL/QCMP` | eQEP register peek |

## PC Tools

```bash
# Auto-capture
python pc/abi_tjx_auto.py --auto --rec-ms=2000

# Single command
python pc/abi_tjx_auto.py --cmd=SNAP?

# Test suite
python pc/test_abi_tjx.py
```

## Architecture

```
main loop (1Hz status)
├── cli_task()            serial command handler
├── snap_poll(spd_rpm())  trigger/recording state machine
├── LED (RGB)             blue=armed green=rec purple=done
└── Motor_CloseLoop       PID (spd_rpm feedback)

1kHz ISR (CPUTIMER0)
├── g_us64 clock
├── spd_tick_1khz         EMA-smoothed RPM
└── Encoder_PeriodicUpdate

2kHz ISR (EQEP1 UTO)
├── QPOSCNT → counts accumulator
├── snap_on_event()       ring buffer fill
├── index calibration     PPR EMA
└── direction detection
```

## Pin Map

| Function | Pin | Mux |
|---|---|---|
| EQEP1 A | GPIO50 | EQEP1_A |
| EQEP1 B | GPIO51 | EQEP1_B |
| EQEP1 I | GPIO53 | EQEP1_INDEX |
| SCIA TX | GPIO29 | SCIA_TX |
| SCIA RX | GPIO28 | SCIA_RX |
| SPIB CLK | GPIO14 | SPIB_CLK |
| SPIB PICO | GPIO30 | SPIB_PICO |
| SPIB POCI | GPIO31 | SPIB_POCI |
| SD CS | GPIO6 | GPIO output |
| RGB B | GPIO20 | output (active low) |
| RGB G | GPIO21 | output (active low) |

## Key Discoveries

- **PCM not functional on F28P550SJ9**: Position Compare Match never fires. TI has no PCM examples. Switched to UTO + capture (TI's canonical approach).
- **`#pragma pack` not supported by C2000 compiler**: Use natural alignment (16B struct works without packing).
- **EALLOW required for QPOSCTL writes**: Driverlib `EQEP_enableCompare` re-calls `EQEP_setCompareConfig` which overwrites manual config with invalid SysConfig values.
- **SD SPI needs MISO pull-up**: C2000 internal pull-up insufficient; external 10kΩ required.
