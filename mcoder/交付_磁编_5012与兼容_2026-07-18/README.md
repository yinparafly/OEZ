# 交付：磁编 5012 独立版 + 兼容测试版（2026-07-18）

本目录为干净快照，内容与下列工作区工程一致：

| 工程 | 路径 |
|------|------|
| 5012 独立专用版 | `ESP32_MT5012/`（源：`D:\oezcon\mcoder\ESP32_MT5012\`） |
| AS5047P↔5012 兼容测试 | `ESP32_Encoder_Compat/`（源：`D:\oezcon\mcoder\ESP32_Encoder_Compat\`） |
| 上午 AS5047P 稳定版（未改） | `D:\oezcon\mcoder\ESP32_AS5047P\` |

## 芯片结论（5012）

- **Infineon TLE5012B**（模块俗称 5012 / MT5012）
- **3 线 SSC**（SCK / CSQ / DATA 半双工），SPI Mode1
- 读角命令 **0x8021**，15-bit，`deg = raw * 360 / 32768`
- 供电 **3.3V**（芯片 3.0~5.5V）
- 接线：DATA 需将 ESP32 **MOSI∥MISO** 短接（与 AS5047P 四线不同）

## 快速使用

独立 5012：

```bat
D:\oezcon\mcoder\ESP32_MT5012\flash_com18.bat
cd /d D:\oezcon\mcoder\ESP32_MT5012\pc && python encoder_monitor.py
```

兼容测试：

```bat
D:\oezcon\mcoder\ESP32_Encoder_Compat\flash_com18.bat
cd /d D:\oezcon\mcoder\ESP32_Encoder_Compat\pc && python encoder_monitor.py
```

兼容切换：固件宏 `DEFAULT_SENSOR`、串口 `S0`/`S1`、或 PC 传感器下拉。

详情见各子目录 `README.md`。
