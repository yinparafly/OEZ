# 天机星 TMS320F28P550 事件测速监控器 — 设计（阶段1）

日期：2026-08-01
硬件：立创·天机星 TMS320F28P550SJ9 开发板
源码参考：`D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\`（ESP32-S3 版，协议与业务逻辑来源）
信号源：AS5047P 磁编码器 ABI 输出（十进制默认 **1000 PPR**，可低速自检校准）

## 1. 目的与背景

ESP32 版用 **2kHz 固定采样 + counts 差分** 测速并记录。弹射过程转速剧烈变化，
AB 脉冲在时间轴上**非均匀分布**：低速时稀疏、高速时密集。固定采样会：
- 高速段每个采样周期内夹带大量边沿（丢失边沿细节）
- 低速段每个采样周期可能 0~1 个边沿（量化台阶，见 ESP32 毛刺文档）

本设计改为 **事件触发测速 + 事件点记录**：每个 AB 边沿（或 A 边沿）是一个
速度点，记录其时刻与计数，速度点时间戳**非均匀**，但完整还原真实速度轨迹。

**阶段范围（本次实现）：**
1. 事件触发测速（EQEP 硬件捕获）
2. 记录：武装 → 触发 → 回溯 400 事件点 + 再记 2400 事件点（SNAP_CAP=3500 有余量）
3. 无记录时 10Hz 串口遥测；记录时串口静默，完成后串口上传或写 SD
4. SD 卡：外接 SPI SD 模块 + FatFS，记录完自动存 `/snap_*.bin`
5. 1kHz 速度估计流 + 无新事件外推（为阶段2 闭环预留，本阶段实现算法）
6. 分档测速：约 6000 RPM 为界，以下用 ABI 4X、以上只用 A+I（1X），
   自动切换+迟滞（实现细节见 §3：>6500 切 1X，<5500 切回 4X）

**阶段2（将来，只留接口）：**
- 400~600Hz 固定频率速度输出，供外部做速度闭环控制
- 蓝牙串口模块

## 2. 核心架构：事件驱动测速

```
AB 边沿 ──► EQEP1 硬件计数(4X/1X 可切) + 捕获锁存
              │  QCPRD 自动锁存相邻边沿间隔，QCTMR 锁存边沿时刻（无需 CPU 每边沿处理）
              │
              ├─ 记录模式：QEP 捕获中断 → 极小 ISR 写 1 点 (t_us, counts, index_n) → 环缓冲
              │              （点时刻=边沿时刻，非均匀 ✓）
              └─ 实时模式：1kHz 定时器读最新捕获 → 有新事件用实测，无则外推
                              → 10Hz 串口遥测
                              └──(阶段2) 400~600Hz 固定频率闭环输出
```

关键点：
- **QCPRD/QCTMR 硬件锁存**：每个边沿硬件自动记录间隔与时刻，CPU 不需要每个边沿
  都中断；事件率再高（4X@6000RPM≈400kHz）也只由 1kHz 轮询消费。
- **记录 ISR 极小**（~0.3µs）：读 QPOSCNT/QCPRD + 写 16B 点，4X 满速事件率
  约占 CPU 13%，可承受。
- 速度 = 一圈步数 ÷ 边沿间隔（由 QCPRD 直接得出，无需差分采样）。

## 3. 测速与分档

| 转速范围 | 模式 | 一圈步数 | 最大事件率 | 记录 steps= |
|---|---|---|---|---|
| < 5500 RPM | ABI **4X** | 4000 | ~367 kHz | 4000 |
| ≥ 6500 RPM | 只 A + I（**1X**） | 1000 | ~108 kHz | 1000 |
| 5500~6500 | 保持当前档（迟滞防抖） | — | — | — |

- 自动切换带迟滞：>6500 切 1X，<5500 切回 4X，中间区间保持，防频繁切换。
- steps 随档位变化 → `BIN END` 行上报 `steps=`，PC 解析覆盖（见 §5）。
- EQEP 1X 模式：位置计数器在 A 的边沿计数；I 用于每圈复位/校准与圈数累计。
- 切换时机在 1kHz 主循环（非 ISR）执行，切换瞬间计数基准按档位换算保持连续。

## 4. 记录流程（武装 → 触发 → 回溯+续记）

```
空闲/遥测 ──MONITOR START──► 武装：环缓冲(800 点)持续记事件点
  │                              │ |RPM|>10 且 I 过 1 圈
  │                              ▼
  │                          STAGING/确认
  │                              │ 回溯环缓冲末 400 点 拷入主缓冲
  │                              ▼
  │                          RECORD：再记 2400 事件点
  │                              │ 满 → SNAP DONE → ALIVE(2×1s)
  │                              ▼
  │                     自动写 SD /snap_*.bin → # SNAP DUMP READY
  │                              │
  │                     PC 自动 DUMP BIN（串口拉 RAM，SRC=RAM）
  └────────────────────────────────┘
```

- 触发条件：|RPM|>10 且 I 过 1 圈（沿用 ESP32 语义）。
- 记录期间**串口静默**（不灌遥测，保证记录/回传通道干净）；完成后再恢复 10Hz。
- 事件点数：SNAP_CAP=3500（回溯 400 + 触发后 3100，比要求 2400 多）。
- 记录时如果发生档位切换，主缓冲内 counts 以同一档位连续换算，保证差分正确。

## 5. 串口协议（PC 兼容 + steps 修复）

波特率 **921600**（abi_monitor.py 默认）。

指令（与 ESP32 v40 一致）：`MONITOR START/STOP`、`REC MS <ms>`、`SNAP?`、
`DUMP BIN`、`FW?`、`TIME <unix_ms>`、`PING`、`HELP`、`ABI?`。

遥测（非记录时，10Hz）：`L,<t_ms>,<rpm_x10>,<dir>,...` 行（与 ESP32 格式一致）。

标记行：`# MONITOR armed`、`# SNAP DONE n=`、`# ALIVE n=`、`# SNAP DUMP READY n=`、
`# BIN END n= hz= steps= bytes= crc=`。

DUMP BIN 帧（照抄 ESP32 v2）：
```
AA×10 + 55          (11B preamble)
0xAB1C0002           (4B magic LE)
n                    (2B 点数 LE)
hz                   (2B 采样率 LE，填 0——事件模式无固定采样率；PC 端 hz 仅显示用
                      （`@ {hz}Hz`），RPM 计算全用 t_us 差分，不受影响)
点阵                 (n × 16B: t_us u32, counts i64, index_n u32)
CRC32                (4B LE, zlib CRC32 of payload)
```
`BIN END` 行带 `steps=<4000|1000>`。

**PC 端修改（约 10 行，abi_monitor.py）**：
- 解析 `BIN END` 行 `steps=` 参数，覆盖 `ABI_STEPS_PER_REV`（RPM 计算用）。
- 此修改同时修掉 ESP32 版 4X 计数 + PC steps=1000 的 4 倍转速 bug 隐患。

## 6. 内存布局（启用 L1/L2 SARAM）

F28P55x 默认链接布局约 66KB 用户 RAM；SysConfig 将 L1/L2 SARAM（32KB）配为
数据 RAM → 约 98KB 可用。

| 段 | 大小 | 放置 |
|---|---|---|
| SNAP 主缓冲 3500×16B | 56.0 KB | RAMGS0-3 + RAMLS8/9 或 L1/L2（链接器显式段） |
| RING 环缓冲 800×16B | 12.8 KB | 同上第二段 |
| 代码 + 常量 + bss + 栈 | ~20 KB | RAMLS0-7 + RAMM0/M1 |
| FatFS 工作区 + 扇区缓冲 | ~3 KB | bss |

点结构（v2，与 ESP32 完全一致）：
```c
typedef struct {          // 16B packed
  uint32_t t_us;          // 距本段起点的相对时间戳（事件时刻）
  int64_t  counts;        // 档位统一换算后的累计 counts（4X 或 1X 基准）
  uint32_t index_n;       // I 过零累计圈数
} SnapPoint;
```

## 7. SD 卡（本次实现）

- 外接 SPI SD 模块 → **SPIB**（SysConfig 选 3 引脚 + 1 GPIO CS，避开 GPIO0-3 板载 W25Q32/SPIA）。
- FatFS（chaN 纯 C，diskio 适配层写 SPIB 驱动）。
- 记录完成后自动写 `/snap_YYYYMMDD_HHMMSS_n.bin` + `.txt` 元数据（TIME 对时后；
  未对时用 `b<millis>` 后备，与 ESP32 一致）。
- PC 新增指令 `SD DUMP <file>`：串口读 SD 文件内容回传（SPI 读卡，不经 U 盘）。
- 写 SD 在记录完成后的主循环执行（非 ISR），期间停采样/停遥测，防 SPI 争用。

## 8. 外推算法（阶段1 实现，供阶段2 闭环）

1kHz 定时器输出速度估计时：
- 最近 1ms 内有新事件 → 输出实测速度（QCPRD 换算）。
- 无新事件 → 外推：
  - 默认：前 2 点线性外推（斜率=最近两点速度差/时间差）。
  - 可选：前 3~5 点最小二乘线性拟合外推（抗噪声，切换参数）。
- 外推速度钳制：不小于 0（单向转轴不反推），不超最大物理转速。
- 停止判定：事件间隔 > 阈值（如 200ms）→ 输出 0。

## 9. 引脚规划（以官方原理图核对）

| 信号 | 引脚 | 备注 |
|---|---|---|
| AS5047P A | GPIO50 (EQEP1_A) | 3.3V 直连 |
| AS5047P B | GPIO51 (EQEP1_B) | 3.3V 直连 |
| AS5047P I | GPIO53 (EQEP1_INDEX) | 3.3V 直连 |
| SCIA TX/RX | GPIO29/28 | 板载 USB-UART → PC @ 921600 |
| RGB LED | GPIO20/21 | 状态指示（武装=蓝，记录=绿，就绪=紫） |
| SD CS | 1×GPIO（SysConfig 选） | 未占用 |
| SD SCK/MOSI/MISO | SPIB×3（SysConfig 选） | 避开 GPIO0-3/50/51/53/28/29 |

## 10. 环境与工程

- CCS **20.1.1** + C2000WARE **5.04.00.00** + XDS110 驱动，全部装到 `D:\ti\`（安装包在 `D:\oezcon\TMS320\05-【TMS320F28P550】开发工具\`）。
- 工程：复制 N20 例程工程模板（EQEP1+SCIA+RGB 已配好）→ 新工程 `AbiMonitor_TJX`，
  位置 `D:\oezcon\TMS320\AbiMonitor_TJX\`。
- 开发方式：SysConfig（c2000.syscfg）+ driverlib C 代码。
- 下载：CCS Debug（RAM 调试）/ 烧写 FLASH 后串口验证。

## 11. 固件模块划分

| 文件 | 职责 |
|---|---|
| `empty_driverlib_main.c` | 主循环：命令解析、遥测、状态机轮询 |
| `eqep_abi.c/h` | EQEP1 初始化（4X/1X 切换、捕获中断、I 计数、64 位 counts 扩展） |
| `snap_bin.c/h` | 事件点记录：环缓冲/主缓冲/回溯/ALIVE/DUMP BIN/CRC32（移植 ESP32） |
| `speed_est.c/h` | 1kHz 速度估计 + 外推（线性/拟合）+ 分档切换判定 |
| `cli.c/h` | 串口指令解析与应答（协议同 ESP32） |
| `sd_fatfs.c/h` | FatFS + SPIB diskio + 存档文件名/元数据 |
| `lckfb_tjx_init.*` | 复用官方（delay/lc_printf） |

## 12. 测试计划

1. **低速自检（校准 steps）**：手转 N 圈，串口 `ABI?` 查 counts → 验证 1000×N（1X）
   或 4000×N（4X）；steps 常量按实测修正（AS5047P 手册 1000 PPR 十进制）。
2. **实时遥测**：手转/电机，PC 看 10Hz `L,` 行 RPM 与实际相符。
3. **事件记录**：MONITOR START → 弹射 → SNAP DONE → ALIVE → PC 自动 DUMP BIN →
   曲线工作室出图（非均匀时间轴，验证高速细节保留）。
4. **分档切换**：用高转速源验证 >6500 切 1X、<5500 切回 4X，steps= 随之变化，
   PC RPM 数值正确（无 4 倍跳变）。
5. **SD 存档**：插入 SD 卡 → 记录完成自动存 `/snap_*.bin` → `SD DUMP` 拉回验证 CRC。
6. **外推**：低速缓转 → 1kHz 估计流无新事件时段输出平滑外推值，停止后归 0。
7. **烧录 FLASH** 版回归全部上述项。

## 13. 风险与对策

| 风险 | 对策 |
|---|---|
| 4X 高速事件率 400kHz ISR 压力 | ISR 极小化；QCPRD 硬件锁存不依赖逐事件处理；可降 2X |
| 分档切换造成 counts 基准跳变 | 切换时按档位比例换算基准，BIN 内统一档位 |
| PC steps 硬编码 | 本次已安排 PC 解析 BIN END steps=（约 10 行） |
| FatFS 移植工作量 | 用现成 chaN FatFS R0.15 纯 C，仅 diskio 适配层自写 |
| SD 与记录争用 | 写 SD 在主循环、停采样期间执行（同 ESP32 策略） |
| L1/L2 配成 RAM 后 Flash 执行变慢 | 代码留 RAM；必要时只把大缓冲放 L1/L2 |
