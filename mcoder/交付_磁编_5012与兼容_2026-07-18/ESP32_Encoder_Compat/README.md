# 磁编码器兼容测试版（AS5047P ↔ TLE5012B）

用于同一套 ESP32 固件/PC 在两种传感器间切换测试。  
**独立专用 5012 请用** `D:\oezcon\mcoder\ESP32_MT5012\`。  
**上午稳定的 AS5047P 专用工程未改动**：`D:\oezcon\mcoder\ESP32_AS5047P\`。

---

## 切换方式

### 1) 编译宏（默认传感器）

`ESP32EncoderCompat.ino` 顶部：

```cpp
#define DEFAULT_SENSOR 1   // 0=AS5047P, 1=TLE5012B
```

### 2) 串口运行时

115200，发一行：

| 命令 | 传感器 |
|------|--------|
| `S0` / `sensor as5047p` | AS5047P |
| `S1` / `sensor tle5012b` / `sensor 5012` | TLE5012B |
| `?` | 查询当前 |

### 3) PC 下拉框

监视器顶部「传感器」下拉 →「切换」（连接后自动按当前选项发一次）。

**重要**：两芯片接线不同，软件切换前请先改好硬件线。

---

## 接线差异

| 信号 | AS5047P | TLE5012B (5012) |
|------|---------|-----------------|
| VCC | 3.3V | 3.3V（芯片 3.0~5.5V） |
| GND | GND | GND |
| CS | GPIO10 | GPIO10 (CSQ) |
| SCLK | GPIO12 | GPIO12 (SCK) |
| 数据 | MOSI=11, MISO=13 **分开** | **DATA 单线**：GPIO11∥GPIO13 短接至 DATA |

引脚默认与上午 AS5047P 工程对齐。

---

## 烧录

```bat
D:\oezcon\mcoder\ESP32_Encoder_Compat\flash_com18.bat
```

或 PowerShell：

```powershell
& "D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32s3 "D:\oezcon\mcoder\ESP32_Encoder_Compat\firmware\ESP32EncoderCompat" --config-file "C:\Users\tt\.arduinoIDE\arduino-cli.yaml"

& "D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" upload --fqbn esp32:esp32:esp32s3 -p COM18 "D:\oezcon\mcoder\ESP32_Encoder_Compat\firmware\ESP32EncoderCompat" --config-file "C:\Users\tt\.arduinoIDE\arduino-cli.yaml"
```

FQBN：`esp32:esp32:esp32s3`。勿设 `ARDUINO_DATA_DIR`。

---

## 开 UI

```powershell
cd D:\oezcon\mcoder\ESP32_Encoder_Compat\pc
pip install -r requirements.txt
python encoder_monitor.py
```

串口协议与专用版相同：`t_ms,raw,deg,rad,rpm,ef,agc,magL,magH`。

---

## 目录

```text
ESP32_Encoder_Compat/
├── README.md
├── flash_com18.bat
├── firmware/ESP32EncoderCompat/
│   ├── ESP32EncoderCompat.ino
│   ├── as5047p_driver.h      ← 独立副本，不改上午工程
│   └── tle5012b_driver.h
└── pc/
    ├── requirements.txt
    └── encoder_monitor.py
```

## 芯片速查（5012）

- 芯片：**Infineon TLE5012B**
- 接口：3 线 SSC，SPI Mode1，半双工 DATA
- 读角：`0x8021` → AVAL 15-bit，`deg = raw * 360 / 32768`
