# AS5047P ABI 转速监控（独立工程）

**不修改**既有 `ESP32_AS5047P`。当前主路径：**短时 RAM 记录 → SD 存档 → USB 读卡或蓝牙从 RAM 取回**。

固件：`FW=monitor-v26-ble-ram`

## 短时记录流程

| 步骤 | 行为 |
|------|------|
| **MONITOR START** | 武装，开环缓（可静止） |
| **\|RPM\| > 10** | STAGING |
| **I 过 1 圈** | 触发：回溯 ≤400 点 + 再记 **2 s @ 2 kHz** → INTERNAL RAM |
| **ALIVE ~2 s** | 自动 **SD SAVE** → `/snap_*.bin` |
| **取数** | 有 BLE：跳过 U 盘，发 `BLE PULL READY`，PC 自动 `DUMP BIN BLE`（**读 RAM**）；无 BLE：自动 `USB DISK ON` |

`REC NOW` = 强制直采（无门限），同样 RAM→SD。

## 目录

| 路径 | 说明 |
|------|------|
| `firmware/ESP32AbiMonitor/` | ESP32-S3 固件 |
| `pc/abi_monitor.py` | PC UI（USB / BLE）+ 喇叭提醒 |
| `pc/curve_studio.py` | 曲线工作室 |
| `pc/smoke_ble_v26.py` | 引导冒烟（可 `--skip-motor`） |
| `backups/usb-msc-stable-2026-07-25/` | USB 读卡可用版源码备份 |
| `docs/硬件联调清单.md` | 回来联调用 |
| `docs/蓝牙取数-RAM优先.md` | 为何 BLE 读 RAM 不读 SD |
| `docs/Android-APK更新.md` | 手机 APK 编译安装 |
| `android/AbiRpmMonitor/` | Android App（v1.5.0-ble-snap） |

## 接线

| AS5047P | ESP32-S3 |
|---------|----------|
| A / B / I | GPIO15 / 16 / 17 |
| VCC / GND | 3.3V / GND |

SPI SD：`CS=10 SCK=12 MOSI=11 MISO=13`

## 烧录 / PC

```powershell
# 烧录 COM6
E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\flash_com6.bat

cd E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\pc
pip install -r requirements.txt
python abi_monitor.py --ble    # 或 run_ble.bat
python test_snap_parse_offline.py   # 无硬件自检解析
```

上电应见：`FW=monitor-v26-ble-ram`，`BLE name=OEZ-ABI`。

## 关键指令

| 指令 | 作用 |
|------|------|
| `MONITOR START` / `STOP` | 武装 / 解除 |
| `REC NOW` | 强制直采 |
| `SD SAVE` | RAM→SD |
| `DUMP BIN BLE` | 蓝牙从 **RAM** 拉二进制（hex+ACK） |
| `DUMP BIN` | USB 串口二进制 |
| `USB DISK ON` / `OFF` | Native USB 只读 U 盘 |
| `TIME <unix_ms>` | 对时（文件名墙钟） |

## PC 语音（仅 PC，ESP 不播）

正式提醒直接说内容，例如：`已经开始监控，请加油`、`探测到了`、`记录完毕`。  
冒烟脚本里的系统步骤才说：`测试，测试，xxxx`。

## Android APK（v1.5.0-ble-snap）

已与固件 v26 对齐：监控 → SD → 自动 `DUMP BIN BLE`（读 RAM）。说明见 `docs/Android-APK更新.md`。

```bat
cd /d E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\android\AbiRpmMonitor
build_apk.bat
adb install -r E:\OEZCON\mcoder\AbiRpmMonitor-debug.apk
```

手机：扫描 **OEZ-ABI** → 连接 → **开始监控** → 加油 → 自动取回（或点 **拉取RAM**）。  
注意：同一时间板子只能被 **PC 或手机一方** 蓝牙连接。
