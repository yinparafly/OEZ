# AS5047P + ESP32-S3 + PC 监视器（交付备份 2026-07-18）

本目录是 **2026-07-18** 的干净程序备份与说明，用于交接/存档。

**请先阅读详细说明：**

→ [说明_AS5047P_ESP32_PC_2026-07-18.md](./说明_AS5047P_ESP32_PC_2026-07-18.md)

## 快速入口

| 内容 | 路径 |
|------|------|
| 详细说明 | `说明_AS5047P_ESP32_PC_2026-07-18.md` |
| ESP32 固件 | `firmware/ESP32ReadAS5047P/ESP32ReadAS5047P.ino` |
| 烧录脚本 | `flash_com18.bat`（默认 COM18） |
| PC 监视器 | `pc/encoder_monitor.py` |
| PC 依赖 | `pc/requirements.txt` |

## 一句话用法

1. 接线：MOSI11 / MISO13 / SCLK12 / CS10，VCC→**3.3V**，共地。  
2. 烧录：运行 `flash_com18.bat`（或见详细说明中的 CLI 命令）。  
3. PC：`cd pc` → `pip install -r requirements.txt` → `python encoder_monitor.py`。

手册与原理图不在本备份内，见：

- `D:\oezcon\mcoder\手册_AS5047P.pdf`
- `D:\oezcon\mcoder\AS5047P原理图.png`

日常继续开发请用工作工程：`D:\oezcon\mcoder\ESP32_AS5047P\`。
