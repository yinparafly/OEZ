# AS5047P + ESP32-S3 磁编码器读取

依据 **ams AS5047P Datasheet [v1-01]**（仓库内 `mcoder/手册_AS5047P.pdf`）与模块原理图实现。

用 SPI 读 14-bit 绝对角（电机相位），PC 显示 **0~360° / 0~2π**，支持 **开始/停止记录** 与 **转速 vs 时间**。

---

## 手册要点（已落实到程序）

| 项目 | 手册规定 | 本工程 |
|------|----------|--------|
| SPI 模式 | Mode1：CPOL=0, CPHA=1；MSB first；≤10 MHz | `SPI_MODE1`，2 MHz |
| 帧格式 | 16-bit 命令帧：PARC(偶校验) + R/W + ADDR[13:0] | `buildReadCmd()` |
| 读时序 | 本帧 MISO = **上一命令** 的数据（Figure 15） | 流水线连续发读命令 |
| 角度寄存器 | `ANGLECOM 0x3FFF`：带 DAEC 补偿（高速推荐） | 默认读此寄存器 |
| 帧间隔 | CSn 高电平 ≥350 ns；CLK 周期 ≥100 ns | CS 间 `delayMicroseconds(1)` |
| 上电 | `t_pon` 后才有有效角（typ 10 ms） | `setup` 里 delay ≥20 ms |
| 诊断 | `DIAAGC`：AGC、MAGL/MAGH | 串口附带 agc/mag 标志 |

命令帧结构（手册 Figure 13）：

```text
bit15     PARC   偶校验（对低 15 位）
bit14     R/W    1=读, 0=写
bit13:0   ADDR   寄存器地址
```

读回数据帧（Figure 14）：

```text
bit15     PARD   偶校验
bit14     EF     1=上一命令帧出错
bit13:0   DATA   14-bit 角度等
```

---

## 供电

实物模块丝印为 **3.3~5V**（板级已做好芯片供电扩展），对接 ESP32-S3：

| 模块 | ESP32-S3 |
|------|----------|
| VCC（或标 5V 的电源脚） | **3.3V** |
| GND | GND |

**推荐直接接 3.3V**，SPI 电平一致。不要接 5V 再直连 ESP32（5V 时 MISO 可能超 IO 耐压）。

---

## SPI 接线

### 模块 P1（2.54 排针）

| P1 | 信号 | ESP32-S3 |
|----|------|----------|
| 1 | MOSI | GPIO11 |
| 2 | MISO | GPIO13 |
| 3 | SCLK | GPIO12 |
| 4 | CS   | GPIO10 |
| 5 | GND  | GND |
| 6 | VCC  | **3.3V**（板丝印 3.3~5V，对接 ESP32 用 3.3V） |

### 模块 P3（GH1.25-6P，顺序与 P1 相反）

| P3 | 信号 | ESP32-S3 |
|----|------|----------|
| 1 | VCC  | 电源 |
| 2 | GND  | GND |
| 3 | CS   | GPIO10 |
| 4 | SCLK | GPIO12 |
| 5 | MISO | GPIO13 |
| 6 | MOSI | GPIO11 |

ABI / UVW 本次不用。引脚可在 `.ino` 顶部 `PIN_*` 修改。

---

## ESP32 程序

路径：`firmware/ESP32ReadAS5047P/ESP32ReadAS5047P.ino`

### 控制频率（固定）

| 宏 | 值 | 含义 |
|----|----|------|
| `CTRL_HZ` | **400** | 串口/控制用滤波角度输出 |
| `OVERSAMPLE` | **4** | 内部过采样倍数 → **1600 Hz** 读角 |
| `SAMPLE_US` | 625 | `micros()` 调度周期 |
| `SERIAL_BAUD` | **921600** | 400 Hz 行输出带宽需要 |

为何 1600→400：控制环 400 Hz 每周期至少 1 点相位即可；4 点短窗滑动平均 + unwrap 跳变剔除，在几乎不增延迟（2.5 ms）下压噪声。AS5047P 内部刷新 MHz 级，1600 Hz 完全够。

### 本地烧录（推荐 arduino-cli）

本机方法与 `oez/esp32_can_servo/ESP32_本地烧录方法与注意事项_2026-07-18.md` 一致：

| 项 | 值 |
|----|----|
| CLI | `D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe` |
| 配置 | `C:\Users\tt\.arduinoIDE\arduino-cli.yaml` |
| FQBN | `esp32:esp32:esp32s3`（不要用 `yd_esp32s3_n8r2`） |
| Sketch | **仅** `firmware/ESP32ReadAS5047P`（不要编父目录） |
| 串口 | 设备管理器确认（当前常用 **COM18**） |
| 监视 | **921600**；关闭 DTR/RTS 复位 |

一键（先确认 COM，关掉占用串口的监视器）：

```bat
D:\oezcon\mcoder\ESP32_AS5047P\flash_com18.bat
```

或手工 PowerShell：

```powershell
& "D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32s3 "D:\oezcon\mcoder\ESP32_AS5047P\firmware\ESP32ReadAS5047P" --config-file "C:\Users\tt\.arduinoIDE\arduino-cli.yaml"

& "D:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" upload --fqbn esp32:esp32:esp32s3 -p COM18 "D:\oezcon\mcoder\ESP32_AS5047P\firmware\ESP32ReadAS5047P" --config-file "C:\Users\tt\.arduinoIDE\arduino-cli.yaml"
```

注意：不要设置 `ARDUINO_DATA_DIR`；在**本机正常** PowerShell/cmd 执行，勿依赖 Cursor 沙箱终端。

### 串口输出

1. Arduino IDE：开发板 **ESP32S3 Dev Module**（FQBN 同上）  
2. 上传后串口 **921600**，格式：

```text
# sample_hz=1600 ctrl_hz=400 oversample=4 ...
# format: t_ms,raw,deg,rad,rpm,ef,agc,magL,magH,rate_hz
1234,8192,180.000,3.141593,0.00,0,128,0,0,400
```

- 输出为 **400 Hz 滤波后**控制用角度；`rate_hz` 恒为 400  
- `raw`：14-bit（由滤波角反推），`deg = raw * 360 / 16384`  
- `ef`：SPI 错误标志；`magL/magH`：磁场过弱/过强（气隙/磁铁）

---

## PC 程序

```powershell
cd D:\oezcon\mcoder\ESP32_AS5047P\pc
pip install -r requirements.txt
python encoder_monitor.py
```

默认优先 **COM18**，启动时若存在则自动连接（可用程序内「连接/断开」）。串口 **921600**，已关闭 DTR/RTS。UI 约 25 Hz 刷新表盘/曲线，数据按 400 Hz 全收；标题显示采样/控制频率。记录上限约 12 万点（~5 min）。

| 功能 | 说明 |
|------|------|
| 实时角度 | 度 / 弧度 / RPM |
| 相位钟表 | 0° 在上方，顺时针（电机相位习惯） |
| 诊断 | AGC、MAGL/MAGH、SPI EF |
| 开始/停止记录 | 绘制该段 RPM vs t |
| 导出 CSV | `t_s,deg,rad,rpm,raw` |
| 中文字体 | 自动加载微软雅黑等，避免表盘汉字方框 |

**2026-07-18 交付快照**（说明 + 干净程序备份）：  
`D:\oezcon\mcoder\交付_AS5047P_ESP32_PC_2026-07-18\`

---

## 目录

```text
ESP32_AS5047P/
├── README.md
├── flash_com18.bat
├── firmware/ESP32ReadAS5047P/ESP32ReadAS5047P.ino
└── pc/
    ├── requirements.txt
    └── encoder_monitor.py
```

说明书：`mcoder/手册_AS5047P.pdf`（及副本 `AS5047P_datasheet.pdf`）。

---

## 常见问题

1. **角度卡在 ~359.9 / raw=16383** — 全 1：查 CS/时钟/MOSI/MISO、供电模式、偶校验。  
2. **偶发 EF=1** — 降 SPI 时钟、缩短杜邦线、确认 Mode1。  
3. **MAGL=1** — 磁铁太远/太弱（手册正常工作约 35~70 mT）。  
4. **MAGH=1** — 气隙过小、磁场过强。  
5. **VCC 接 5V 再直连 ESP32** — 不推荐；用 3.3V。  
6. **磁场** — 手册正常工作约 35~70 mT；气隙过大 → MAGL，过近 → MAGH。
