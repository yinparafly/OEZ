# 工作同步清单（ESP32 + PC）

> 供另一台计算机上的 Cursor 阅读，继续今日电机转速控制相关工作。

**仓库：** https://github.com/yinparafly/OEZ  
**分支：** `feat/motor-pitch-rpm-apk`  
**工程根目录：** `mcoder/ESP32_AS5047P/`

```powershell
git clone -b feat/motor-pitch-rpm-apk https://github.com/yinparafly/OEZ.git
cd OEZ\mcoder\ESP32_AS5047P
```

若仓库已存在：

```powershell
git fetch origin
git checkout feat/motor-pitch-rpm-apk
git pull
```

---

## 一、ESP32 端

| 名称 | 路径 | 说明 |
|------|------|------|
| **ESP32ReadAS5047P**（主固件） | `firmware/ESP32ReadAS5047P/ESP32ReadAS5047P.ino` | ESP32-S3：读 AS5047P 角度/转速；GPIO9 输出航模电调 PWM（1000–2000 μs）；开环/闭环/学习/测量；目标最高 **6000 RPM**；控制环 **`CTRL_HZ = 250`**（避免 100 Hz 时 ~3000 RPM Nyquist 折叠） |
| **flash_com18.bat** | `flash_com18.bat` | 本机烧录脚本（串口名按实际改，常用 COM10/COM18） |
| **README** | `README.md` | 接线、烧录、串口协议说明 |

**硬件要点：**

- 板子：ESP32-S3
- 编码器：AS5047P（SPI：CS=10, SCLK=12, MISO=13, MOSI=11）
- 电调 PWM：GPIO9
- 扑翼霍尔：GPIO4=下扑0°、GPIO5=上举~180°；YL-57/A3144 **VCC=5V**；厂商 51 样例为 **触发=低**；DO 须分压到 3.3V（见 `扑翼霍尔接线_2026-07-21.md`）
- 串口：**921600**，关 DTR/RTS
- 烧录 FQBN：`esp32:esp32:esp32s3`

**今日关键修复（已在该 `.ino`）：**

- `CTRL_HZ` 100 → **250**
- STOP / LEARN 不被 UI 抢油门
- 学习可升到约 98% 转速上限或 2000 μs
- 开环映射不再误抬 crawl 最低油门

---

## 二、PC 端

| 名称 | 路径 | 说明 |
|------|------|------|
| **encoder_monitor.py**（主界面） | `pc/encoder_monitor.py` | Python GUI：串口遥测、目标转速、学习/测量、电调校准、相位/停机等；学习时写 CSV；期望固件 **250 Hz**（`EXPECTED_CTRL_HZ = 250`） |
| **analyze_learn_log.py** | `pc/analyze_learn_log.py` | 分析 `learn_*.csv`：油门是否单调、转速是否 Nyquist 折叠 |
| **auto_learn_test.py** | `pc/auto_learn_test.py` | 无界面自动学习测试（可 `--dry` 只查 ctrl_hz） |
| **requirements.txt** | `pc/requirements.txt` | 依赖：`pyserial`、`matplotlib`、`numpy` |

**PC 启动：**

```powershell
cd OEZ\mcoder\ESP32_AS5047P\pc
pip install -r requirements.txt
python encoder_monitor.py
```

（串口按设备管理器改，常见 COM10）

---

## 三、相关但非 ESP32/PC 主程序（可选）

| 名称 | 路径 | 说明 |
|------|------|------|
| **MotorPitchRpm** | `android/MotorPitchRpm/` | Android 听音 FFT 测 RPM；云端已编 APK |
| Actions | `.github/workflows/motor-pitch-rpm.yml` | 云端编 APK |

---

## 四、给另一台 Cursor 的开场白（可粘贴）

```text
同步分支：feat/motor-pitch-rpm-apk
工程：mcoder/ESP32_AS5047P

ESP32 主程序：firmware/ESP32ReadAS5047P/ESP32ReadAS5047P.ino
  - AS5047P + 电调 PWM，CTRL_HZ=250，学习/STOP/开环已修

PC 主程序：pc/encoder_monitor.py
  - 串口 921600 监控与控制；配套 analyze_learn_log.py、auto_learn_test.py

说明见 README.md 与本文件「工作同步清单_另一台Cursor.md」。
请从该工程继续电机转速控制相关工作。
```

---

## 五、两台电脑日常同步

| 动作 | 命令 |
|------|------|
| 开工前拉最新 | `git pull` |
| 改完提交 | `git add …` → `git commit -m "…"` |
| 推到云端 | `git push` |
| 另一台更新 | `git pull` |

始终在同一分支：`feat/motor-pitch-rpm-apk`。

**已在 GitHub：** ESP32 固件、PC 监控与分析脚本、Android 源码与 Actions。  
**本机未入库（默认拉不到）：** 部分 md 建议文档、APK、本机 Gradle 镜像改动等。
