# AS5047P ABI 转速监控（独立工程）

**不修改**既有 `ESP32_AS5047P`。主路径：**短时 RAM 记录 → SD 存档 → USB 读卡或蓝牙从 RAM（空则 SD）取回**。

**交接请先读：** [`docs/项目传递介绍.md`](docs/项目传递介绍.md)（源码位置、架构、版本、坑与联调）。

当前定稿（以串口 `FW?` / 源码为准）：

| 端 | 版本 |
|----|------|
| 固件 | `FW=monitor-v40-counts-bin`（BIN 存 counts；转速 PC/手机算；可回退 v37） |
| Android | `1.9.5-counts-bin` |
| PC | `pc/abi_monitor.py` + 曲线工作室；exe 见 `mcoder/AbiRpmMonitor-PC*.exe` |

## 短时记录流程

| 步骤 | 行为 |
|------|------|
| **MONITOR START** | 武装，开环缓（可静止）；BLE 停实时转速 |
| **\|RPM\| > 门限** | STAGING（短弹射 keep-shot：掉速也尽量触发） |
| **触发** | 回溯 + 再记 **REC MS @ 2 kHz** → INTERNAL RAM |
| **ALIVE** | 自动 **SD SAVE** → `/snap_*.bin` |
| **取数** | 有 BLE：跳过 U 盘，`BLE PULL READY` → `DUMP BIN BLE`（**优先 RAM**）；无 BLE：可 `USB DISK ON` |

## 目录

| 路径 | 说明 |
|------|------|
| `firmware/ESP32AbiMonitor/` | ESP32-S3 固件 |
| `pc/` | PC UI、BLE、曲线工作室、打包 |
| `android/AbiRpmMonitor/` | 手机 App |
| `docs/` | 设计与联调文档 |
| `docs/项目传递介绍.md` | ★ 交接总览 |
| `backups/v37-keep-shot-2026-07-26/` | ★ keep-shot + BLE 可用冻结（回退用） |
| `backups/usb-msc-stable-2026-07-25/` | USB 读卡可用版备份 |

## 接线

| AS5047P | ESP32-S3 |
|---------|----------|
| A / B / I | GPIO15 / 16 / 17 |
| VCC / GND | 3.3V / GND |

SPI SD：`CS=10 SCK=12 MOSI=11 MISO=13`

## 烧录 / PC / 手机

```powershell
E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\flash_com6.bat

cd E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\pc
pip install -r requirements.txt
python abi_monitor.py --ble
# 或 AbiRpmMonitor-PC-new.exe
```

```bat
cd /d E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\android\AbiRpmMonitor
build_apk.bat
adb install -r E:\OEZCON\mcoder\AbiRpmMonitor-debug.apk
```

上电应见：`FW=monitor-v40-counts-bin`，`BLE name=OEZ-ABI`。回退：`docs/版本标签-v37-keep-shot.md`。

## 关键指令

| 指令 | 作用 |
|------|------|
| `MONITOR START` / `STOP` | 武装 / 解除 |
| `DUMP BIN BLE` | 蓝牙拉二进制（RAM，空则 SD） |
| `DUMP BIN` | USB 串口二进制 |
| `SNAP?` | RAM 是否有效 |
| `USB DISK ON` / `OFF` | 只读 U 盘 |
| `TIME <unix_ms>` | 对时 |

更多规则与踩坑：`docs/工作日志与总结-2026-07-25下午-手机BLE.md`。
