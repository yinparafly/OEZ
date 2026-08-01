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
2. 记录：武装 → 触发 → 回溯 400 事件点 + 再记 2400 事件点（SNAP_CAP=3400 有余量）
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
- **QCPRD/QCTMR 硬件锁存**：每个边沿硬件自动记录间隔与时刻。实时遥测模式
  1kHz 轮询只读最新锁存值（**不依赖中断**）；**记录模式才开捕获中断**，两种
  模式由同一状态机切换，保证干净。
- **记录 ISR 极小**：C28x 中断进出开销小（无流水线冲刷），ISR 只做读
  QPOSCNT/QCPRD + 写 16B 点 + 指针自增（无浮点、无 printf、无长临界区），
  估计 0.3–0.5µs @200MHz；4X 满速 400kHz 事件率约占用 CPU 12–20%，实测
  验证（§12.7）。预留 2X 降级开关兜底（事件率减半）。
- **为什么不用高频轮询替代记录中断**（dp 建议）：QCPRD/QCTMR 只锁存*最近
  一个*边沿，轮询率必须高于事件率才不丢点；400kHz 事件率下 100kHz 轮询会丢
  3/4 事件且时间戳退化为轮询时刻。故记录模式必须用中断，实时遥测才用轮询。
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

**分档切换伪代码（1kHz 主循环，非 ISR）：**
```
if rpm_est > 6500 and gear == GEAR_4X:
    gear = GEAR_1X                    # 停 4X 计数，锁存当前 QPOSCNT
    counts_ref = qep_counts_4x / 4    # 按比例换算基准（向零取整）
    idx_cal_pending = 1               # 等下一个 I 脉冲做绝对重校准
elif rpm_est < 5500 and gear == GEAR_1X:
    gear = GEAR_4X
    counts_ref = qep_counts_1x * 4
    idx_cal_pending = 1
```
- **I 重校准**（dp 建议，消除切换换算舍入误差/位置漂移）：`idx_cal_pending`
  置位后，下一个 I 上升沿把位置基准强制置为已知值（QPOSCNT 清零或补偿偏移），
  之后所有 counts 点以该基准重新对齐；主缓冲内单次记录保持同一档位连续换算。

## 4. 记录流程（武装 → 触发 → 回溯+续记）

```
空闲/遥测 ──MONITOR START──► 武装：环缓冲(1200 点)持续记事件点
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

- 触发条件：|RPM|>10 且 I 过 1 圈（沿用 ESP32 语义）；I 边沿配置需在自检步骤
  验证低速可靠性。
- 记录期间**串口静默**（不灌遥测，保证记录/回传通道干净）；完成后再恢复 10Hz。
- 事件点数：SNAP_CAP=3400（回溯 400 + 触发后 3000，比要求 2400 多 600 余量）；
  RING_CAP=1200（4X 满速下覆盖约 4.8ms 回溯余量，G 建议）；`SNAP CAP <n>`
  可配置上限（默认 3400）。
- **t_us 全局零基准**（dp 建议）：所有点时间戳以"触发确认时刻"为 0 起算、
  单调递增、跨文件可比；环缓冲内点存相对时刻，回溯拷入主缓冲时整体平移对齐。
- 记录时如果发生档位切换，主缓冲内 counts 以同一档位连续换算，保证差分正确。

## 5. 串口协议（PC 兼容 + steps 修复）

波特率 **921600**（abi_monitor.py 默认）。

指令（与 ESP32 v40 一致）：`MONITOR START/STOP`、`REC MS <ms>`、`SNAP?`、
`DUMP BIN`、`FW?`、`TIME <unix_ms>`、`PING`、`HELP`、`ABI?`。

遥测（非记录时，10Hz）：`L,<t_ms>,<rpm_x10>,<dir>,...` 行（与 ESP32 格式一致）。

标记行：`# MONITOR armed`、`# SNAP DONE n=`、`# ALIVE n=`、`# SNAP DUMP READY n=`、
`# BIN END n= hz= steps= mode= bytes= crc=`（`mode=event` 标明事件模式，PC 据此
处理 hz=0 显示与时间轴）。

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

**PC 端修改（约 15 行，abi_monitor.py）**：
- 解析 `BIN END` 行 `steps=` 和 `mode=`：steps 覆盖 `ABI_STEPS_PER_REV`
  （RPM 计算用）；mode=event 时 hz 仅作展示（显示"事件模式"，不参与绘图，
  时间轴一律用 t_us 差分）。
- 此修改同时修掉 ESP32 版 4X 计数 + PC steps=1000 的 4 倍转速 bug 隐患。

## 6. 内存布局（启用 L1/L2 SARAM）

F28P55x 默认链接布局约 66KB 用户 RAM；SysConfig 将 L1/L2 SARAM（32KB）配为
数据 RAM → 约 98KB 可用。

| 段 | 大小 | 放置 |
|---|---|---|
| SNAP 主缓冲 3400×16B | 54.4 KB | RAMGS0-3 + RAMLS8/9 或 L1/L2（链接器显式段） |
| RING 环缓冲 1200×16B | 19.2 KB | 同上第二段 |
| 代码 + 常量 + bss + 栈 | ~20 KB | RAMLS0-7 + RAMM0/M1 |
| FatFS 工作区 + 扇区缓冲 | ~3 KB | bss |

点结构（v2，与 ESP32 完全一致）：
```c
typedef struct {          // 16B packed（#pragma pack / __attribute__((packed))，
  uint32_t t_us;          //  TI C2000 编译器下验证 sizeof==16 无 padding）
  int64_t  counts;        // 档位统一换算后的累计 counts（4X 或 1X 基准）
  uint32_t index_n;       // I 过零累计圈数
} SnapPoint;
```
- 链接器用显式段放置两大缓冲，编译后核对 `.map` 文件确认落在 L1/L2 或 GS RAM。

## 7. SD 卡（本次实现）

- 外接 SPI SD 模块 → **SPIB**（SysConfig 选 3 引脚 + 1 GPIO CS，避开 GPIO0-3 板载 W25Q32/SPIA）。
- FatFS（chaN 纯 C，diskio 适配层写 SPIB 驱动）。
- 记录完成后自动写 `/snap_YYYYMMDD_HHMMSS_n.bin` + `.txt` 元数据（TIME 对时后；
  未对时用 `b<millis>` 后备，与 ESP32 一致）。
- PC 新增指令 `SD DUMP <file>`：串口读 SD 文件内容回传（SPI 读卡，不经 U 盘）。
- 写 SD 在记录完成后的主循环执行（非 ISR），期间停采样/停遥测，防 SPI 争用；
  写卡失败自动重试（≤3 次）并上报 `# SD ERR <code>`；写卡期间 RGB 闪烁指示。
- SPIB 引脚在工程初始化阶段即锁定（SysConfig 核对全部占用），填入 §9 表格
  备查，避免与板载外设冲突（G 建议）。

## 8. 外推算法（阶段1 实现，供阶段2 闭环）

1kHz 定时器输出速度估计时：
- 最近 1ms 内有新事件 → 输出实测速度（QCPRD 换算）。
- 无新事件 → 线性外推：前 2 点斜率（最近两点速度差/时间差）。
- **停止判定**：最近事件间隔 > 100ms（可配置）→ 直接输出 0，不强行外推
  （防低速抖动误报，dp 建议）。外推速度钳制：不小于 0（单向转轴不反推），
  不超最大物理转速（常量可配置）。
- 阶段 2 再引入 3~5 点最小二乘线性拟合外推（抗噪声），本阶段不实现。

## 9. 引脚规划（以官方原理图核对）

| 信号 | 引脚 | 备注 |
|---|---|---|
| AS5047P A | GPIO50 (EQEP1_A) | 3.3V 直连 |
| AS5047P B | GPIO51 (EQEP1_B) | 3.3V 直连 |
| AS5047P I | GPIO53 (EQEP1_INDEX) | 3.3V 直连 |
| SCIA TX/RX | GPIO29/28 | 板载 USB-UART → PC @ 921600 |
| RGB LED | GPIO20/21 | 状态指示（武装=蓝，记录=绿，写SD=绿闪，就绪=紫） |
| SD CS | 1×GPIO（工程初始化锁定） | 未占用 |
| SD SCK/MOSI/MISO | SPIB×3（工程初始化锁定） | 避开 GPIO0-3/50/51/53/28/29 |

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
| `cli.c/h` | 串口指令解析与应答（协议同 ESP32）+ 接收超时/帧错误处理 |
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
7. **ISR 开销实测**：记录模式下 CPU 定时器测 ISR 执行周期数，验证 4X 满速 CPU
   占用在预算内；超出则启用 2X 降级开关复测。
8. **极限场景**（G 建议）：长时间空闲后突然弹射（验证环缓冲无污染）；档位切换
   发生在记录过程中（验证 counts 连续性 + I 重校准）；写卡失败重试路径。
9. **烧录 FLASH** 版回归全部上述项。

## 13. 风险与对策

| 风险 | 对策 |
|---|---|
| 4X 高速事件率 400kHz ISR 压力 | ISR 极致精简（无浮点/printf）；实测执行时间（§12.7）；2X 降级开关兜底 |
| 分档切换造成 counts 基准跳变 | 切换伪代码 + I 上升沿绝对重校准（§3）；BIN 内统一档位 |
| 环缓冲回溯余量不足（G 建议） | RING_CAP=1200（4X 满速约 4.8ms）；`SNAP CAP` 可配置 |
| 大缓冲内存布局（G 建议） | 链接器显式段 + `.map` 核对；packed 验证 sizeof==16 |
| PC steps 硬编码 / hz=0 兼容（dp 建议） | PC 解析 BIN END steps=+mode=（约 15 行）；hz=0 仅显示事件模式 |
| FatFS 移植工作量 | 用现成 chaN FatFS R0.15 纯 C，仅 diskio 适配层自写 |
| SD 与记录争用 / 写卡失败 | 主循环停采样写卡 + 重试≤3 + `# SD ERR` 上报 |
| 高速 AB 信号完整性（dp 建议） | 信号线靠近源处预留 100pF 焊盘（硬件滤波）；接线用短跳线 |
| L1/L2 配成 RAM 后 Flash 执行变慢 | 代码留 RAM；必要时只把大缓冲放 L1/L2 |
