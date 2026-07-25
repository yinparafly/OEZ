# SPI microSD 接线与自检（ESP32-S3 ABI Monitor）

## 接线

| SD 模块 | ESP32-S3 | 备注 |
|---------|----------|------|
| VCC / 3V3 | 3.3V | 勿接 5V |
| GND | GND | 与编码器共地 |
| CS | GPIO10 | |
| SCK / CLK | GPIO12 | |
| MOSI / DI | GPIO11 | |
| MISO / DO | GPIO13 | |

ABI 占用：A=15 B=16 I=17。勿占用 USB 19/20。

卡格式建议：**FAT32**。

## 固件命令（`FW=monitor-v20-sd`）

上电自动 `SD.begin` + `SD TEST`。

| 命令 | 作用 |
|------|------|
| `SD?` | 卡类型/容量 |
| `SD INIT` | 重新挂载 |
| `SD TEST` | 写读 `/oez_sd_test.txt` |
| `SD LIST` | 列根目录 |
| `SD SAVE` | 把当前 SNAP 写成 `/snap_<n>_<ms>.bin` |

推荐流程：

1. 看开机是否 `# SD OK` / `# SD TEST OK`  
2. 电机转起来 → `REC` → 等 `# SNAP DONE`  
3. `SD SAVE` → 拔卡用读卡器在 PC 上看 `.bin`  
4. （以后）Native USB 挂 MSC，免拔卡  

## 烧录

`flash_com6.bat`（先关占用 COM6 的 UI）
