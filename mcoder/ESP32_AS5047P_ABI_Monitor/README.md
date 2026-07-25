# AS5047P ABI 转速监控（独立工程）

**不修改**既有 `ESP32_AS5047P` 电机控制工程。本目录为新增：ABI 正交测速 + 板内记录 + BLE + PC/Android 显示回放。

## 语义：开始 / 停止监控

| 步骤 | 行为 |
|------|------|
| **MONITOR START** | 武装探测（不立刻写主记录） |
| **\|RPM\| > 20** | 进入 **临时数据池**（STAGING） |
| **净转角 ≥ 1 周** | 判定真实转动 → **追溯提交**整池到主记录 → 再继续记 **2 s** |
| **噪声** | 掉速/反向抖动/**1 秒内未满 1 周** → **丢弃临时池**，不进主记录 |
| **MONITOR STOP** | 解除武装；若在临时池则丢弃 |

- **板内主记录**：只含已确认段；多段同一缓冲，用 `seg` 区分。
- **导出**：`LOG DUMP` 按段输出 `S BEGIN` / `D,...` / `S END`；PC「拉取并按段保存」→ `pc/logs/<时间>/abi_seg001_….csv` 每段一个文件。
- 回放：选择段号后「回放该段」。
- 清空：`LOG CLEAR`（主记录 + 临时池）。

## 目录

| 路径 | 说明 |
|------|------|
| `firmware/ESP32AbiMonitor/` | ESP32-S3 固件（PCNT + esp_timer 2kHz + BLE `OEZ-ABI`） |
| `pc/abi_monitor.py` | PC UI（USB / BLE） |
| `android/AbiRpmMonitor/` | Android App |

## 接线（ESP32-S3）

| AS5047P | ESP32-S3 |
|---------|----------|
| A | GPIO15 |
| B | GPIO16 |
| I（可选） | GPIO17 |
| VCC | 3.3V |
| GND | GND |

默认按芯片 **4000 steps/rev**（decimal 最高档）。若你改过 ABIRES，请同步改固件 `ABI_STEPS_PER_REV`。

> 本固件只用 ABI，不占用原工程的 SPI/ESC 引脚约定；可与电机控制板分时烧录，或另板运行。

## 烧录

- 板型：`esp32:esp32:esp32s3:PSRAM=opi`（本机 8MB Embedded PSRAM；`flash_com6.bat` 已写死）
- Sketch：`firmware/ESP32AbiMonitor`
- 串口波特率：**921600**
- 成功时应看到 `spiram_free=` 很大，且 `log_cap=600000 psram=1`（约 **10 min @1kHz** / **5 min @2kHz**）

上电应看到：

```text
# ESP32 AS5047P ABI Monitor
# BLE name=OEZ-ABI ...
# ready. MONITOR START to record |rpm|>20
```

## 协议（USB / BLE Nordic UART）

### 实时遥测（默认约 20 Hz）

```text
L,t_ms,rpm,dir,mon,log_n,log_drop,meas_hz,counts
```

- `dir`：`+1` / `0` / `-1`
- `mon`：`1`=监控中（可记录），`0`=不记录

### 指令

| 指令 | 作用 |
|------|------|
| `MONITOR START` | 开始监控（允许记录） |
| `MONITOR STOP` | 停止监控（不再记录） |
| `MONITOR?` | 查询 |
| `LOG CLEAR` | 清空板内环 |
| `LOG?` | 条数/容量 |
| `LOG DUMP [n]` | 导出记录；行 `D,idx,t_ms,rpm,dir`，结束 `D END n` |
| `ABI?` | 引脚/采样率/当前转速 |
| `BLE RATE <5..50>` | BLE/USB 遥测频率 |
| `PING` | `# PONG` |

## PC（USB / 蓝牙）

```powershell
cd E:\OEZCON\mcoder\ESP32_AS5047P_ABI_Monitor\pc
pip install -r requirements.txt
python abi_monitor.py
# 或双击 run_monitor.bat / run_ble.bat（默认选中蓝牙）
```

界面顶部 **连接方式**：

| 方式 | 操作 |
|------|------|
| **USB串口** | 选 COM → 连接 |
| **蓝牙BLE** | 点「扫描蓝牙」→ 选 `OEZ-ABI` → 连接 |

- 依赖：`bleak`（已在 `requirements.txt`）
- 连上后自动发 `TIME` 对时；状态栏显示收包数
- 注意：同一时间板子只能被 **手机或 PC 一方** 蓝牙连接
## Android

```bat
cd android\AbiRpmMonitor
gradlew.bat assembleDebug
```

APK：`app\build\outputs\apk\debug\app-debug.apk`  
手机：扫描 → 选 **OEZ-ABI** → 连接 → **开始监控** / **停止监控** → **拉取回放**。

## 板内容量

编译打开 **OPI PSRAM** 后优先分配约 **600000** 点（≈3.6MB）：

| 写入率 | 满环时长 |
|--------|----------|
| 1 kHz | ≈ **600 s（10 min）** |
| 2 kHz（当前采样/记录） | ≈ **300 s（5 min）** |

无 PSRAM 时回退 12k/4k。环满覆盖最旧，`log_drop` 累加。清空：`LOG CLEAR` / UI「清空记录」。
