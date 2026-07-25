# 蓝牙取数策略：RAM→SD 存档，BLE 读 RAM（空则 SD 回退）

日期：2026-07-25（下午修订）  
固件：**`FW=monitor-v37-keep-shot`**（上午基线曾为 `v26-ble-ram`）

## 备份

可用的 USB 读卡版已冻结在：

`backups/usb-msc-stable-2026-07-25/`

（含当时 `firmware/` + `pc/` + docs。出问题可整目录拷回。）

下午手机 BLE 详志：`docs/工作日志与总结-2026-07-25下午-手机BLE.md`。

## 数据路径（不变）

```text
武装 MONITOR → 环缓 → 触发（含短弹射 keep-shot）→ 正式段写入 INTERNAL RAM
     → ALIVE → 自动 SD SAVE（/snap_*.bin 存档）
```

SD 始终是可靠存档；蓝牙只是另一种取回方式。

## 蓝牙从哪读？

| 来源 | 优点 | 缺点 |
|------|------|------|
| **RAM（优先）** | 与刚写入 SD 的点阵相同；传输时不占 SPI | 掉电丢失；再次 MONITOR START 会清 |
| **SD 回退** | RAM 空时仍能下到上一拍 | 须先整文件载入再 BLE，勿边传边读卡 |

**结论：BLE 优先从 RAM 读（`DUMP BIN BLE` / `src=RAM`）；`SNAP? valid=0` 或 RAM 空则走 SD。**  
需要物理拷贝时仍用 USB MSC / 拔卡（无 BLE 连接时 SAVE 后仍会自动 `USB DISK ON`）。

## 有 BLE 连接时 SAVE 后的行为

- **跳过**自动 `USB DISK ON`（避免 MSC 占卡、打断后续 BLE）
- 发出 `# BLE PULL READY`
- PC / 手机自动 `SNAP?` → `DUMP BIN BLE`（或 SD）

## 协议摘要

```text
# BIN BLE BEGIN src=RAM|SD n=… hz=… bytes=… crc=0x…
B,<seq>,<HEX>
…
# BIN BLE END src=… n=… chunks=… ok=1 crc=0x…
```

每包等主机 `DUMP ACK`。手机 Dump：**Notify 为主**，勿持续 GATT READ。

## PC / 手机用法

```powershell
# PC 源码
cd E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\pc
python abi_monitor.py --ble
# 或单文件
E:\OEZCON\mcoder\AbiRpmMonitor-PC.exe

# 手机
adb install -r E:\OEZCON\mcoder\AbiRpmMonitor-debug.apk
```

扫描 `OEZ-ABI` → 连接 → **开始监控** → 触发后等 PULL READY → 自动拉数 → 曲线 / 工作室。
