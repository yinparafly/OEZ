# AS5047P + ESP32-S3 + 航模电调转速控制

依据 **ams AS5047P** 读电机相位，经串口上报；PC 监视角度/转速，并通过圆盘设定目标转速，驱动电调 PWM。

**控制策略**：台阶辨识油门–转速前馈图 + PI 闭环；`noload` / `flap` 两套 profile。空载学完后再接扑翼需重学。

---

## 接线

### 编码器 AS5047P（3.3V）

| 信号 | ESP32-S3 |
|------|----------|
| MOSI | GPIO11 |
| MISO | GPIO13 |
| SCLK | GPIO12 |
| CS   | GPIO10 |
| VCC  | **3.3V** |
| GND  | GND |

### 电调 PWM

| 信号 | ESP32-S3 |
|------|----------|
| 油门信号 | **GPIO9** |
| 信号地 | GND（与电调共地） |

- PWM：**400 Hz**，脉宽 **1000~2000 μs**（航模标准；**最低油门 1000 μs**）
- 上电固件**立刻**输出 1000 μs
- **不要**把电调 BEC 的 5V 接到 ESP32

---

## 烧录

| 项 | 值 |
|----|----|
| FQBN | `esp32:esp32:esp32s3` |
| Sketch | `firmware/ESP32ReadAS5047P` |
| 串口 | 常用 COM18（以设备管理器为准） |
| 波特率 | **921600**；关闭 DTR/RTS |

```bat
E:\OEZCON\mcoder\ESP32_AS5047P\flash_com18.bat
```

---

## 串口协议

遥测（约 100 Hz）：

```text
t_ms,raw,deg,rad,rpm,ef,agc,magL,magH,pulse_us,target_rpm,mode,kp,ki,kd,profile,run
```

| mode | 含义 | run | 含义 |
|------|------|-----|------|
| 0 SAFE | 强制最低油门 | 0 IDLE | 未运行 |
| 1 OPEN | 开环前馈 | 1 RUN | 运行中 |
| 2 CLOSED | 前馈+PI | 2 ESTOP | 急停 |
| 3 LEARN | 学习中 | | |

指令（行末 `\n`）：

| 指令 | 说明 |
|------|------|
| `START` / `STOP` / `ESTOP` | 启动 / 正常停 / 急停（立刻 1000 μs） |
| `RPM <0..2000>` | 设定目标（需再 `START` 才跑；运行中可热更新） |
| `SOFT ON\|OFF` / `SOFT RATE <rpm/s>` | 缓启动 |
| `MODE OPEN\|CLOSED\|SAFE` | 控制模式 |
| `PROFILE noload\|flap` | 前馈图槽位 |
| `MEASURE START` / `ABORT` | 进入/退出测量（手动脉宽观察转速） |
| `PWM <1000..2000>` | 测量模式下设脉宽 |
| `MEASURE HOLD` | 记录当前 (pulse, rpm) 点 |
| `MEASURE AUTO` / `LEARN START` | 自动台阶测量 |
| `MEASURE SAVE` / `CLEAR` | 保存前馈图并建议 PID / 清空 |
| `ESCCAL HIGH\|LOW\|DONE` | 电调油门行程校准（配对） |
| `PID <kp> <ki> <kd>` / `PID SAVE` | 热改 / 存 NVS |
| `ADAPT ON\|OFF` | 有界 Kp 自适应 |
| `PING` | 心跳（PC 约 0.5 s 一次，防主机超时急停） |

---

## PC 程序

```powershell
cd E:\OEZCON\mcoder\ESP32_AS5047P\pc
pip install -r requirements.txt
python encoder_monitor.py
```

| 功能 | 说明 |
|------|------|
| 转速圆盘 | 按住旋转；**4 圈 = 2000 RPM** |
| 目标输入框 | 0~2000，与圆盘同步 |
| 启动 / 停止 / 急停 | 急停红色；Esc=急停，空格=停止 |
| 缓启动 | 勾选 + 斜率 RPM/s |
| 学习 | 选 profile 后「开始学习」；换扑翼后用 `flap` 重学 |
| 开环/闭环 | 学习完成后默认进 CLOSED |

---

## 推荐操作顺序

### A. 电调与控制器“配对”（油门行程校准）

航模电调并不做电机型号配对，而是**学习本控制器的最低/最高油门脉宽**（Hobbywing / BLHeli / ArduPilot 通用）：

1. PC 点 **高油门**（输出 2000 μs）
2. **再给电调动力电池上电**，听确认“最大油门”的提示音
3. 点 **低油门**（1000 μs），听长鸣确认最小油门
4. 点 **完成**

以后 ESP 的 1000~2000 μs 才与电调行程对齐。

### B. 测量模式（先于闭环）

1. 选 profile：`noload`（空载）或 `flap`（扑翼）
2. **开始自动学习** → 确认后脉宽自动台阶上升（约 +50μs / 2s），UI 滑条会跟着动
3. 结束后会写 NVS 并建议 PID；也可点 **保存+建议PID**
4. 需要微调时用 **仅手动** + 滑条/±50 / HOLD
5. 闭环用建议的 Kp/Ki（或手动改后点应用）

### C. 闭环控制

1. 圆盘/输入设目标 RPM → 缓启动 → **启动**
2. 按建议值微调 PID 后「下发 / 保存」
3. 接上扑翼后换 `flap` **重新测量**，勿直接用空载图

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

交付快照（编码器监视，不含本电调扩展）：`mcoder/交付_AS5047P_ESP32_PC_2026-07-18/`
