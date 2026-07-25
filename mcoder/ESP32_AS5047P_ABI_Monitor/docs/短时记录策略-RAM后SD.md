# 短时记录策略（已采用）· 2026-07-25

## 结论

**2 kHz × 1～2 s ≈ 几十 KB → 内部 RAM 采集；记完后再 SD 存档。**  
不在 ISR / 采样过程中写 SD。

## 流程（当前触发规则）

```
MONITOR START（可静止武装）
  → 环缓临时存（2kHz）
  → |RPM|>10 进入 STAGING，再等 I 过完 1 圈
  → 触发：回溯最近 400 点 + 再记 2s → INTERNAL RAM
  → # SNAP DONE → ALIVE → 自动 SD SAVE → /snap_*.bin
  → USB DISK ON → Native USB 只读 U 盘拷文件（或拔卡）
```

`REC NOW` = 强制直采（无门限，调试用）。  
**不走** 串口大包 DUMP；蓝牙后置。

## 数据量

| 时长 | 点数@2kHz | 约字节（12B/点） |
|------|-----------|------------------|
| 1 s | ~2000 | ~24 KB |
| 2 s | ~4000 | ~48 KB |

默认 `REC MS=2000`。可调 `REC MS 1000`～`3000`。

## 固件

- `FW=monitor-v23-i1rev`
- SPI SD：CS=10 SCK=12 MOSI=11 MISO=13
- 串口：CH343（COM6）；U 盘：板载 Native USB（TinyUSB MSC，只读）
- 命令：`MONITOR START` / `REC NOW` / `SD SAVE` / `USB DISK ON` / `USB DISK OFF`

## 与长采区别

| 场景 | 介质 |
|------|------|
| 短时 1～2 s | INTERNAL RAM → 事后 SD → USB 读盘 |
| CAPTURE 几十秒 | PSRAM（按需分配） |
