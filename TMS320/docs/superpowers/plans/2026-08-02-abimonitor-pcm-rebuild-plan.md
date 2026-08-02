# AbiMonitor_TJX 实施计划 v2：PCM 事件内核重构（链路验证优先）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在原 13 Task 计划（`2026-08-01-abimonitor-eventspeed-plan.md`，Task 1/2 已完成）基础上重构：
① 先验证全链路（SD + PC UI + 串口交互，记录内容先用简单测试数据）；② PCM 事件内核为**唯一测速源**
（1kHz 采样作废）；③ 弹射记录（回溯 400 + 续记 2400）；④ 原计划剩余任务重基；⑤ 待办（QPOSCMP 动态步长
/ CLB 对比试验 / 闭环 CSV 日志）。

**Design doc:** `TMS320/docs/superpowers/specs/2026-08-02-abimonitor-pcm-rebuild-design.md`

**Tech Stack:** CCS 20.1.1（Theia）+ C2000Ware 5.04.00.00 + TI C2000 编译器 22.6.1.LTS（`--abi=eabi`）
+ SysConfig 1.23 + driverlib + FatFS R0.15 + Python3（pyserial/matplotlib）。

**构建/烧录事实（2026-08-02 实测确定）：**
- 无头构建（Theia 版 CCS 20 正确方法）：`D:\ti\ccs2011\ccs\eclipse\ccs-server-cli.bat -workspace D:\temp\opencode\ccs_ws -application com.ti.ccs.apps.buildProject -ccs.projects AbiMonitor_TJX -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport`（autoImport 首次必需；改完源码须 Copy-Item 同步到 `D:\temp\opencode\ccs_ws\AbiMonitor_TJX\`）。
- 烧录：`& "D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe" load --config="D:\temp\opencode\ccs_ws\AbiMonitor_TJX\targetConfigs\TMS320F28P550SJ9.ccxml" <out>`；成功标志 `Running... Success`（2026-08-02 已用此流程烧录 build_out22 成功）。
- 串口：COM23，921600 8N1。

## Global Constraints

- **ESP32 工程只读**：`D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\` 固件、abi_monitor.py、curve_studio.py 一律不修改；参考只用于移植。
- 工程 `D:\oezcon\TMS320\AbiMonitor_TJX\`（源）↔ `D:\temp\opencode\ccs_ws\AbiMonitor_TJX\`（构建副本）；PC 工具 `D:\oezcon\TMS320\pc\`。
- 设备：TMS320F28P550SJ9，128PDT，150MHz；EQEP1（A=GPIO50 B=GPIO51 I=GPIO53）；SCIA（TX=GPIO29 RX=GPIO28，921600）；RGB_B=GPIO20 / RGB_G=GPIO21（低电平点亮）；SPIA GPIO0-3 板载 W25Q32 禁用；SD 用 SPIB（CLK=GPIO14, PICO=GPIO30, POCI=GPIO31, CS=GPIO6）。
- 记录规格：SNAP_CAP=3400×16B、RING_CAP=1200×16B、BACKTRACK_N=400、steps=4000；点格式 `<IqI`；帧 `AA×10+55 + magic 0xAB1C0002 + n u16 + hz u16(=0) + 点阵 + crc32`；CRC 与 zlib 一致。
- 触发：|rpm|>10 进 STAGING，确认 OR{I+1圈, ≥200ms, ring≥120, (rpm跌落且ring≥40)}；rpm跌=|rpm|<5。
- 时间戳：CPUTIMER0 自由运行 + 1kHz ISR 维护 g_us64/g_us_tick_cnt + 亚 µs 插值（÷150）。
- 内存：两大缓冲显式段 `SNAP_RAM`；兜底 SNAP_CAP=2800、RING_CAP=800（.map 核对不足即启用）。
- 每次构建：CPU1_RAM（RAM 调试）；最终 CPU1_FLASH 烧录回归。
- 每个固件任务验证 = 构建通过 + 烧录后串口断言；PC 任务验证 = `python test_abi_tjx.py` 全绿。

---

## 阶段一：链路验证（用户要求第一步，记录内容先用测试数据）

### Task 1: 全链路验证——SD 冒烟 + PC 工具骨架 + 串口交互（测试数据）

**Files:**
- Modify: `D:\oezcon\TMS320\pc\abi_tjx.py`、`D:\oezcon\TMS320\pc\test_abi_tjx.py`（骨架：串口 + 帧重组 + 存盘）
- Modify: `AbiMonitor_TJX\c2000.syscfg`（SCIA 921600、SPIB+CS 引脚、CPUTIMER0 1kHz、EQEP1 正交）
- Create: `AbiMonitor_TJX\app\sd_fatfs.h/.c`（第一阶段：diskio + 挂载 + `SD INIT/SD?/SD TEST`）
- Create: `AbiMonitor_TJX\app\cli.h/.c`（第一阶段：最小命令表 `PING/FW?/SNAP?/SD?/SD INIT/SD TEST/HELP`）

**Interfaces:**
- Consumes: Task 2 基线工程（已就绪）。
- Produces: 可验证的完整链路：PC 连接 → 发命令 → 板端应答 → SD 写读往返一致；为 Task 2-5 提供 cli_printf/cli_put_raw 基础设施。

- [ ] **Step 1: PC 解析库 + 测试（原 Task 4 原样执行）**
  - 写 `test_abi_tjx.py` 失败测试 → 实现 `abi_tjx.py`（SNAP_PREAMBLE、snap_crc32、parse_bin_end、parse_snap_bindump、rpm_from_counts_series、parse_bin_frame）→ ALL PASS（代码骨架见原计划 Task 4）。
- [ ] **Step 2: SysConfig 板级配置（原 Task 3）**
  - SCIA 921600（保留 RX 中断）；EQEP1（A=GPIO50 B=GPIO51 I=GPIO53，正交，QPOSMAX=0xFFFFFFFF，关 unit timer）；CPUTIMER0 1kHz + registerInterrupts；SPIB（GPIO14/30/31）+ CS GPIO6（普通输出）；删 EPWM4/EPWM8 实例；RGB GPIO20/21。
- [ ] **Step 3: FatFS R0.15 下载 + ffconf 配置 + diskio（SPIB 轮询）**
  - `https://elm-chan.org/fsw/ff/arc/ff15.zip` → `app\fatfs\`（ff.c/ff.h/ffconf.h/ffsystem.c/diskio.c/diskio.h；不需要 ffunicode.c）。
  - ffconf：FF_USE_LFN=1、FF_FS_TINY=0、FF_MIN/MAX_SS=512、FF_FS_READONLY=0、FF_VOLUMES=1。
  - diskio：SPIB 400kHz 初始化 / 25MHz 读写；CS 由 GPIO 控制；`disk_initialize/read/write/ioctl(CTRL_SYNC, GET_SECTOR_COUNT/SIZE)`。
- [ ] **Step 4: 最小 cli.c + sd_fatfs.c（第一阶段版）**
  - `cli_init/cli_poll/cli_printf/cli_put_raw/cli_put_raw_byte`（SCI 轮询直写，关中断防穿插）。
  - 命令：`PING→# PONG`、`FW?→# FW=abi_tjx-v2-pcm-eventsnap`、`SNAP?→# SNAP src=RAM valid=0 n=0 ring=0`、`SD?→# SD ready=0/1`、`SD INIT→# SD OK type=... size_MB=...`、`SD TEST→写读 /oez_sd_test.txt 往返一致`、`HELP`。
- [ ] **Step 5: 构建 + 烧录 + 串口验证（用测试数据）**
  - 无头构建（ccs-server-cli.bat 命令见文档头）；DSLite 烧录；PowerShell 串口脚本或 `pyserial` 交互：
    - `PING`/`FW?`/`SNAP?` 应答正确；无卡 `SD?→ready=0`；插卡 `SD INIT→# SD OK`、`SD TEST→往返一致`。
  - **链路验证判定（用户要求）**：PC `abi_tjx.py --port COM23` 能连接、发命令收应答、`SD DUMP` 拉回文件内容与板端一致 → 链路 OK，进入阶段二。
- [ ] **Step 6: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/c2000.syscfg TMS320/AbiMonitor_TJX/app/cli.h TMS320/AbiMonitor_TJX/app/cli.c TMS320/AbiMonitor_TJX/app/sd_fatfs.h TMS320/AbiMonitor_TJX/app/sd_fatfs.c TMS320/AbiMonitor_TJX/app/fatfs TMS320/pc/abi_tjx.py TMS320/pc/test_abi_tjx.py
  git commit -m "feat: chain verify (SD SPIB+FatFS smoke, minimal CLI, PC tool skeleton) + parse lib"
  ```

---

## 阶段二：PCM 事件内核 + 记录 + 集成

### Task 2: eqep_abi.c——PCM 逐计数事件内核（唯一测速源）

**Files:**
- Create: `AbiMonitor_TJX\app\eqep_abi.h`、`AbiMonitor_TJX\app\eqep_abi.c`
- Modify: `empty_driverlib_main.c`（临时：main 调 abi_init，1Hz 打印 counts/index/period/missed）

**Interfaces:**
- Consumes: Task 1 生成的 `Module_EQEP_*` 宏、CPUTIMER0 宏。
- Produces（Task 3/5 依赖）：
  - `void abi_init(void)`
  - `__interrupt void INT_Module_EQEP_ISR(void)`（PCM/QDC/IEL 三源分发）
  - `int64_t abi_counts(void)`、`uint32_t abi_index_n(void)`、`uint64_t abi_now_us(void)`
  - `int abi_gear(void)`、`void abi_set_gear(int gear)`（步长 1/4，待办 A 扩展动态）
  - `uint32_t abi_last_period_us(void)`、`uint32_t abi_missed_events(void)`
  - `__interrupt void INT_TIMER0_ISR(void)`（1kHz：g_us64/g_us_tick_cnt 维护 + snap_set_ms；rpm 估算在 Task 3）

- [ ] **Step 1: 写 eqep_abi.h/.c**（代码骨架见原计划 Task 6 Step 1/2；PCSHDW/PCLOAD 裸写 QPOSCTL；`EQEP_enableCompare` + `EQEP_setCompareConfig` 武装 ±step；QDC 重武装；IEL 校准挂起标志）
- [ ] **Step 2: 构建 + 烧录 + 串口冒烟**
  - 手转/电驱动多圈：`ABI?` 打印 counts（正转增/反转减）、index_n 每圈 +1、period 静止保持、missed_events=0。
- [ ] **Step 3: ISR 开销实测（强制检查点）**
  - CPUTIMER0 差分测 INT_Module_EQEP_ISR 周期数；>0.6µs（≈90 周期）→ 立即启用 GEAR_1X（N=4）降档，结果记 spec §4.4。
- [ ] **Step 4: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/eqep_abi.h TMS320/AbiMonitor_TJX/app/eqep_abi.c
  git commit -m "feat: PCM per-count event kernel (sole speed source), 64-bit us clock, missed counter"
  ```

### Task 3: speed_est.c——PCM 重基的 1kHz 速度流（删 1kHz 采样路径）

**Files:**
- Create: `AbiMonitor_TJX\app\speed_est.h`、`AbiMonitor_TJX\app\speed_est.c`
- Modify: `eqep_abi.c`（1kHz ISR 里调 spd_tick_1khz，由 INT_TIMER0_ISR 收口）

**Interfaces:**
- Consumes: `abi_last_period_us/abi_gear/abi_set_gear/abi_now_us/abi_counts`（Task 2）、`snap_set_ms`（Task 5，先留空接）
- Produces: `spd_init/spd_tick_1khz/spd_rpm/spd_gear/spd_abs_ms`

- [ ] **Step 1: 写 speed_est.h/.c**（骨架见原计划 Task 7 Step 1/2；**事件新近性用 abi_event_count() 计数法**——eqep_abi 增 `uint32_t abi_event_count(void)`，ISR 里 ++，speed_est 比较计数变化判定新事件；删原 1kHz 采样 Encoder_Count 路径；外推 + 100ms 停止判定 + 档位迟滞 6500/5500）
- [ ] **Step 2: 构建 + 烧录 + 冒烟**
  - 低速手转/电驱动 RPM 合理；停转 100ms 归 0；>6500 切 GEAR_1X（电驱动多速验证）。
- [ ] **Step 3: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/speed_est.h TMS320/AbiMonitor_TJX/app/speed_est.c TMS320/AbiMonitor_TJX/app/eqep_abi.c
  git commit -m "feat: 1kHz speed flow re-based on PCM events, gear hysteresis, extrapolation"
  ```

### Task 4: snap_bin.c 记录引擎（原 Task 5 原样）

**Files:**
- Create: `AbiMonitor_TJX\app\snap_bin.h/.c`（骨架见原计划 Task 5 Step 1/2；`#ifndef HAVE_CLI` 临时替身现在指向正式 cli，无需替身）
- Modify: `snap_bin.h` 增 `snap_force_done(void)`、`snap_set_ms(uint32_t)`（1kHz ISR 注入）

**Interfaces:** 原计划 Task 5 Interfaces 原样（snap_on_event/snap_trigger_from_ring/snap_poll_done/snap_dump_binary/snap_dump_hex/snap_print_status/snap_crc32/snap_data/snap_data_bytes）

- [ ] **Step 1: 写 snap_bin.h/.c**（移植 ESP32 snap_bin.cpp 逻辑；环缓冲/回溯平移/ALIVE 2s/DUMP BIN 帧 + BIN END 行/CRC32）
- [ ] **Step 2: 构建 + 冒烟**：`SNAP STATUS` → `# SNAP n=0 ring=0 ...`；`HEX DUMP`/`DUMP BIN` 空缓冲提示。
- [ ] **Step 3: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/snap_bin.h TMS320/AbiMonitor_TJX/app/snap_bin.c
  git commit -m "feat: snap_bin event recording engine (ring/backtrack/alive/dump/crc32)"
  ```

### Task 5: cli.c 完整命令表 + sd_fatfs.c 完整存档（原 Task 8/9 合并）

**Files:**
- Modify: `AbiMonitor_TJX\app\cli.h/.c`（补全命令：MONITOR START/STOP、REC MS、SNAP STATUS、DUMP BIN、HEX DUMP、TIME、ABI?、SD DUMP；L, 遥测 10Hz）
- Modify: `AbiMonitor_TJX\app\sd_fatfs.c`（完整版：sd_save_snap 存档 + .txt 元数据 + sd_dump_file + sd_poll archive_edge 驱动）

**Interfaces:** 原计划 Task 8/9 Interfaces 原样。

- [ ] **Step 1: cli.c 补全命令表 + 遥测 L, 行（20 字段对齐 ESP32）**
- [ ] **Step 2: sd_fatfs.c 完整版**（存档文件名 snap_YYYYMMDD_HHMMSS_n.bin / b<ms>；写卡期间停采样停遥测；重试 ≤3；`# SD ERR`/`# SD SAVE OK`）
- [ ] **Step 3: 构建 + 烧录 + 冒烟**：`PING`/`TIME 1780123456789`/`ABI?`/`SD INIT`/`SD TEST`/`DUMP BIN`（空）/`SNAP STATUS` 全应答正确。
- [ ] **Step 4: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/cli.c TMS320/AbiMonitor_TJX/app/cli.h TMS320/AbiMonitor_TJX/app/sd_fatfs.c
  git commit -m "feat: full CLI + telemetry L-line + SD archive"
  ```

### Task 6: main.c 状态机集成（武装→STAGING→RECORD→ALIVE→SD→READY + 闭环测试保留）

**Files:**
- Modify: `AbiMonitor_TJX\empty_driverlib_main.c`（集成实现；保留闭环测试序列，反馈重基到 spd_rpm；`g_seq_done` 跑完即停逻辑保留）

**Interfaces:** 原计划 Task 10 原样（monitor_state_machine 触发逻辑照搬 ESP32 语义；LED 武装蓝/记录绿/写SD绿闪/就绪紫；`snap_force_done` 作为 REC MS 超时上限）。

- [ ] **Step 1: 状态机 + 主循环集成**（骨架见原计划 Task 10 Step 1/2；`snap_force_done` 由 main 检查 REC MS 超时调用）
- [ ] **Step 2: 闭环测试序列重基**：pi_target 序列（1500/2000 RPM 等）反馈改 `spd_rpm()`；PWM 输出仍 Motor_Set_PWM；科目完成后 g_seq_done=1 永久停机。
- [ ] **Step 3: 遥测调度 + LED**
- [ ] **Step 4: 构建 CPU1_RAM + CPU1_FLASH 双配置**
- [ ] **Step 5: 上板冒烟（无 SD 卡也应工作）**
  - boot banner → MONITOR START → armed → 电驱动 → CONFIRM → RECORD → SNAP DONE → ALIVE 2 拍 → DUMP READY → PC DUMP BIN 拉帧成功；闭环科目跑完电机停止（验证 g_seq_done）。
- [ ] **Step 6: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/empty_driverlib_main.c TMS320/AbiMonitor_TJX/app/snap_bin.h TMS320/AbiMonitor_TJX/app/snap_bin.c
  git commit -m "feat: main state machine (arm/staging/record/alive) + closed-loop re-based on PCM + stop-after-complete"
  ```

### Task 7: PC 工具完整版（原 Task 11）

**Files:**
- Modify: `D:\oezcon\TMS320\pc\abi_tjx.py`（串口层 SerialLink + 帧重组 + 自动流程 + tkinter GUI + matplotlib 出图）
- Modify: `D:\oezcon\TMS320\pc\test_abi_tjx.py`（补帧流解析 + BIN END steps 覆盖）

**Interfaces:** 原计划 Task 11 原样（自动流程 TIME+START→遥测→SNAP DONE→ALIVE→DUMP READY→DUMP BIN→校验→存盘→出图；超时 5s 兜底 SNAP STATUS）。

- [ ] **Step 1: 补测试（帧流组装 + steps 覆盖）→ ALL PASS**
- [ ] **Step 2: SerialLink（pyserial 921600，reader 线程，帧重组）**
- [ ] **Step 3: 自动流程 + 存盘 .bin/.csv**
- [ ] **Step 4: tkinter GUI + matplotlib 出图（rpm 非均匀时间轴 + counts 子图）**
- [ ] **Step 5: 与固件联调（板在手）**：完整弹射记录 → 自动 BIN OK → 存盘 → 出图。
- [ ] **Step 6: Commit**
  ```bash
  git add TMS320/pc/abi_tjx.py TMS320/pc/test_abi_tjx.py
  git commit -m "feat: abi_tjx PC tool full (serial/auto-pull/save/plot)"
  ```

### Task 8: 硬件联调 + 回归（原 Task 12，重基到 PCM）

**Files:**
- Modify: 按发现修正 `AbiMonitor_TJX\app\*` 与 `TMS320\pc\*`
- Create: `TMS320\docs\联调记录\2026-08-02-abimonitor-bringup.md`

- [ ] **Step 1: 低速自检（校准 steps）**：`ABI?` 电驱动多圈 → canonical counts=4000×N
- [ ] **Step 2: 实时遥测**：10Hz `L,` rpm 与实际相符
- [ ] **Step 3: 事件记录**：MONITOR START → 弹射 → SNAP DONE → ALIVE → DUMP READY → PC 自动拉帧 → 出图
- [ ] **Step 4: 分档切换**：>6500 切 1X、<5500 回 4X；counts 无阶跃
- [ ] **Step 5: ISR 开销实测（满载复核）**：4X 满速 CPU <30% 预算
- [ ] **Step 6: SD 存档**：插卡 → 记录完自动存 → SD DUMP 拉回 → PC 校验 CRC
- [ ] **Step 7: 外推**：低速缓转 → 无事件时段线性外推；停止 100ms 归 0
- [ ] **Step 8: 极限场景**：空闲后突弹射（环缓冲无污染）；记录中档位切换（counts 连续）；拔卡写失败重试
- [ ] **Step 9: 闭环测试回归**：1500/2000 RPM 科目全程正常、跑完停机；闭环过程用待办 C 日志验证
- [ ] **Step 10: 波特率误差测量**：1KB 环回测偏差；>±2% 改 460800
- [ ] **Step 11: CPU1_FLASH 回归**：烧 FLASH 重跑 Step 2-9
- [ ] **Step 12: Commit**
  ```bash
  git add TMS320/docs/联调记录/2026-08-02-abimonitor-bringup.md TMS320/AbiMonitor_TJX/app TMS320/pc
  git commit -m "test: bring-up pass — PCM event kernel (see bringup log)"
  ```

### Task 9: 收尾（原 Task 13）

- [ ] **Step 1: 回填 spec**（实测值：内存段地址、SPIB 引脚确认、ISR 周期数、事件率、波特率偏差）
- [ ] **Step 2: README**（构建/烧录/Python 工具用法；无头构建命令记录）
- [ ] **Step 3: 阶段2 backlog 确认**（完整 PC GUI / 400-600Hz 闭环输出 / 蓝牙 / 待办 A/B/C）
- [ ] **Step 4: Commit**

---

## 阶段三：待办（用户指定新内容，原始计划完成后执行）

### Task 10: 待办 A——QPOSCMP 动态步长（连续平滑调整）

**Files:**
- Modify: `AbiMonitor_TJX\app\eqep_abi.c`、`AbiMonitor_TJX\app\speed_est.c`

**Goal:** 取代两档硬切换：按最近速度估计**连续调整 PCM 步长 N**（低速 N=1 高精度，
高速 N=8/16 降 ISR 率），使中断频率稳定在安全值（如 <250kHz），精度与量程兼得。
迟滞防抖；canonical counts 恒精确（QPOSCNT 硬件累加，N 只影响 ISR 触发密度）。

- [ ] **Step 1: 步长自适应表**（如 rpm→N 映射：≤6000→1, ≤12000→4, ≤20000→8；迟滞区间）
- [ ] **Step 2: 改造 ISR 武装逻辑**（N 变化在 1kHz 主循环写，ISR 用影子值；漏事件计数核对）
- [ ] **Step 3: 构建 + 冒烟**：多速度电驱动验证中断率稳定、missed_events=0、counts 连续
- [ ] **Step 4: Commit**

### Task 11: 待办 B——CLB 方案对比试验（三指标对比）

**Files:**
- Create: `AbiMonitor_TJX\app\clb_pcm.c/h`（试验代码，独立模块，可编译开关隔离）
- Modify: `c2000.syscfg`（CLB1 配置；CLBINPUTXBAR/CLBOUTPUTXBAR 映射）

**Hardware facts（已核实）：** CLB1_BASE=0x00003000；driverlib `clb.h` 全 API
（CLB_configCounterLoadMatch/CLB_clearFIFOs/CLB_writeFIFOs/CLB_getInterruptTag/CLB_clearInterruptTag）；
CLB_LOCAL_IN_MUX_INPUT1..16（CLBINPUTXBAR 16 输入）；CLB_GLOBAL_IN_MUX_*；CLA_TRIGGER_CLB1INT=127；
DMA_TRIGGER_CLB1INT=127；4 深硬件 FIFO；CLB1INT 可中断 CPU。

**Goal:** 用 CLB 计数器匹配 + FIFO 免 CPU 时间戳方案与 PCM 对比**三指标**：
① 测速精度（时间戳分辨率/事件率误差）② 中断稳定性（满载中断抖动/CPU 占用）
③ 最大量程（可支持的峰值事件率）。试验数据与结论归档到联调文档，验证 PCM 选型或改进方向。

- [ ] **Step 1: CLB1 逻辑设计**（计数器匹配产生事件 → FIFO 存时间戳/计数 → CLB1INT 批量消费）
- [ ] **Step 2: 实现 + 构建**（与 PCM 编译开关切换）
- [ ] **Step 3: 试验**：同一电机场景分别用 PCM / CLB 记录，三指标对比
- [ ] **Step 4: 结论归档 + Commit**

### Task 12: 待办 C——闭环测试日志 CSV（500ms/条）

**Files:**
- Modify: `AbiMonitor_TJX\empty_driverlib_main.c`、`AbiMonitor_TJX\app\sd_fatfs.c`

**Goal:** 闭环科目（1500/2000 RPM）过程以 500ms 间隔记一行 CSV 到 SD：
`t_ms, rpm_target, rpm_act, pwm_out, pi_int, gear, index_n`，供闭环调参分析。

- [ ] **Step 1: 闭环日志缓冲 + SD 追加写**
- [ ] **Step 2: 构建 + 烧录 + 冒烟**：跑完闭环科目 → SD 卡得到 CSV → PC 校验
- [ ] **Step 3: Commit**

---

## Self-Review（写作时已核对）

1. **用户决策全覆盖**：链路验证优先（Task 1）→ PCM 唯一测速源（Task 2/3，删 1kHz 采样）
   → 弹射记录（Task 4-6）→ 原计划重基（Task 1-9 覆盖原 Task 3-13）→ 待办 A/B/C（Task 10-12）。
2. **无冲突**：Task 6 保留 g_seq_done（已实现并烧录）；闭环反馈改 spd_rpm（PCM 源）。
3. **构建/烧录命令**：以 2026-08-02 实测成功的 ccs-server-cli.bat + DSLite 流程为准。
4. **协议不变**：SNAP v2 帧、点格式、触发语义与 ESP32/原计划完全一致；PC 端仅重基数据源。
