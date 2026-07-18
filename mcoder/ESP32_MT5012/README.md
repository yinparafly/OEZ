# TLE5012B (5012 / MT5012) + ESP32-S3 独立专用版

依据 **Infineon TLE5012B Datasheet Rev.2.1**（`mcoder/5012/手册_TLE5012B.pdf`）与模块原理图实现。

本目录为**干净独立交付**，不依赖、不修改上午的 `ESP32_AS5047P`。

---

## 芯片 / 接口结论

| 项目 | 结论 |
|------|------|
| 芯片 | **Infineon TLE5012B**（市面模块常标 5012 / MT5012） |
| 主接口 | **3 线 SSC**（SPI 兼容）：`SCK` / `CSQ` / `DATA`（半双工，非 4 线 MOSI+MISO） |
| SPI 模式 | Mode1（CPOL=0, CPHA=1），MSB first；push-pull 最高约 8 Mbit/s |
| 读角 | 命令字 **`0x8021`** → 寄存器 **AVAL (0x02)**，再跟 1 个 Safety Word |
| 角度 | **15-bit**，`deg = (raw & 0x7FFF) * 360 / 32768`（约 0.010986°/LSB） |
| 供电 | 芯片 **VDD 3.0~5.5 V**；对接 ESP32-S3 用 **3.3 V** |
| 上电 | `t_Pon` typ **57 ms** 后才允许正常通信 |
| 次接口 | 模块另有 IIF A/B/Z（本次不用） |

命令字结构（手册 Table 17）：

```text
bit15     RW     1=读
bit14:11  Lock   0000=访问 0x00..0x04
bit10     UPD    0=当前值
bit9:4    ADDR   0x02 = AVAL
bit3:0    ND     数据字个数（读角=1）
→ 0x8021
```

读帧：`COMMAND` →（`twr_delay`）→ `DATA(AVAL)` → `SAFETY`。

---

## 供电与接线

### 模块排针（原理图 P1/P2）

| 模块脚 | 信号 | ESP32-S3 |
|--------|------|----------|
| 1 | GND | GND |
| 2 | VCC | **3.3V** |
| 3 | CSQ | GPIO10 |
| 4 | SCK | GPIO12 |
| 5 | DATA | **GPIO11 与 GPIO13 短接后接 DATA** |
| 6 | NC | — |

说明：TLE5012B 只有一根双向 `DATA`。为与上午 AS5047P 引脚习惯对齐，固件仍用 MOSI=11 / MISO=13；**请在模块侧把 MOSI、MISO 并到 DATA**（模块上已有 100 Ω 串阻）。固件在发完命令后会把 MOSI 切为输入，避免总线争用（与官方/样本 Arduino 写法一致）。

IIF（A/B/Z）本次不接。

---

## 控制频率（固定）

| 宏 | 值 | 含义 |
|----|----|------|
| `CTRL_HZ` | **400** | 串口/控制用滤波角度输出 |
| `OVERSAMPLE` | **4** | 内部 **1600 Hz** 过采样 |
| `SERIAL_BAUD` | **921600** | 400 Hz 行带宽 |

TLE5012B 默认角刷新约 12 kHz，1600 Hz 过采样足够。固件内 4 点短窗平均 + 跳变剔除后按 400 Hz 输出。

## 烧录

| 项 | 值 |
|----|----|
| CLI | `D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe` |
| 配置 | `C:\Users\tt\.arduinoIDE\arduino-cli.yaml` |
| FQBN | `esp32:esp32:esp32s3` |
| Sketch | **仅** `firmware/ESP32ReadMT5012` |
| 串口 | 常用 **COM18**（以设备管理器为准） |

一键（关掉占用串口的程序后）：

```bat
D:\oezcon\mcoder\ESP32_MT5012\flash_com18.bat
```

PowerShell：

```powershell
& "D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32s3 "D:\oezcon\mcoder\ESP32_MT5012\firmware\ESP32ReadMT5012" --config-file "C:\Users\tt\.arduinoIDE\arduino-cli.yaml"

& "D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" upload --fqbn esp32:esp32:esp32s3 -p COM18 "D:\oezcon\mcoder\ESP32_MT5012\firmware\ESP32ReadMT5012" --config-file "C:\Users\tt\.arduinoIDE\arduino-cli.yaml"
```

注意：不要设 `ARDUINO_DATA_DIR`；在本机正常终端执行。

### 串口输出（与现有 UI 兼容，末列 rate_hz）

```text
# sample_hz=1600 ctrl_hz=400 oversample=4 baud=921600
# format: t_ms,raw,deg,rad,rpm,ef,agc,magL,magH,rate_hz
1234,16384,180.000,3.141593,0.00,0,-1,0,0,400
```

- `raw`：15-bit AVAL（由滤波角反推）  
- 输出为 **400 Hz 滤波后**控制用角度  
- `ef`：Safety 系统/接口异常  
- `agc`：固定 `-1`（无 AGC）  
- `magL`：角度无效（Safety bit12=0）  
- `magH`：0  

---

## PC 监视器

```powershell
cd D:\oezcon\mcoder\ESP32_MT5012\pc
pip install -r requirements.txt
python encoder_monitor.py
```

默认优先 COM18 并自动连接；波特率 **921600**。功能与 AS5047P 版相同（钟表相位、记录、RPM 曲线、25 Hz UI 刷新、中文字体、关 DTR/RTS）。

---

## 目录

```text
ESP32_MT5012/
├── README.md
├── flash_com18.bat
├── firmware/ESP32ReadMT5012/ESP32ReadMT5012.ino
└── pc/
    ├── requirements.txt
    └── encoder_monitor.py
```

兼容测试版（AS5047P ↔ TLE5012B）：`D:\oezcon\mcoder\ESP32_Encoder_Compat\`
