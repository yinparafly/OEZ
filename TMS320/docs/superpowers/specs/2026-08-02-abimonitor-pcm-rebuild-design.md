# AbiMonitor_TJX 重构设计：PCM 事件内核为唯一测速源（v2）

> 日期：2026-08-02（晚）
> 状态：已与用户确认（brainstorming 结论），按本文档执行
> 关联：原设计 `2026-08-01-abimonitor-eventspeed-design.md`（阶段1，历史快照保留）；
>       新实施计划 `2026-08-02-abimonitor-pcm-rebuild-plan.md`

---

## 1. 决策记录（2026-08-02 与用户确认）

1. **1kHz 采样测速作废**：当前固件 `empty_driverlib_main.c` 用 1kHz 定时采样
   Encoder_Count 差分测速（原始方式）。**不再使用**。测速与记录**唯一来源 =
   PCM 逐边沿事件内核**（`eqep_abi.c`），保证采集精确。
2. **闭环测速重基**：电机闭环测试（1500/2000 RPM 序列）仍保留，但其速度反馈
   改由 PCM 事件内核提供（不是 1kHz 采样）。
3. **电机跑完即停**：测试序列全部科目完成后电机永久停机（`g_seq_done` 已实现，
   不再无限循环）。固件已构建并烧录（build_out22 / dslite_load22 成功）。
4. **执行顺序**（用户确认）：
   ```
   ① 链路验证（SD + PC UI + 串口交互，记录内容先用简单测试数据）
   ② 弹射记录（PCM 事件内核）
   ③ 原计划剩余 11 个 Task（以本设计重基）
   ④ 待办（写入计划 Task 12-14）：
      - QPOSCMP 动态步长（连续平滑调整，低速 N=1、高速 N=8/16）
      - CLB 方案对比试验（CLB1 计数器匹配 + FIFO，与 PCM 三指标对比）
      - 闭环测试日志 CSV（500ms/条）
   ```
5. **文档策略**：新 spec（本文）+ 新 plan（`2026-08-02-abimonitor-pcm-rebuild-plan.md`）；
   原 spec/plan 保留为历史快照不动。

## 2. 目标（不变，来自原 spec §1）

ABI 编码器（1000PPR，4X=4000 计数/圈）**事件驱动非均匀采样**测速记录：
每个计数变化记录 16B 事件点 `(t_us u32, counts i64, index_n u32)`，还原弹射
速度轨迹：低速不量化、高速不丢总计数。设计上限 20000rpm（4X 时 1.333M
边沿/s，0.75µs/边沿）。

## 3. 四层架构

```
┌─────────────────────────────────────────────────────────────┐
│ L1 事件内核  app/eqep_abi.c   （唯一测速源，唯一 counts 源）    │
│    QPOSCMP 逐计数中断(PCM) + QDC 方向 + IEL Index             │
│    CPUTIMER0 自由运行 → 64 位 µs 时钟（1kHz ISR 维护+插值）    │
│    输出：canonical counts / index_n / 事件时刻 / 漏事件计数    │
├─────────────────────────────────────────────────────────────┤
│ L2 记录引擎  app/snap_bin.c   （移植 ESP32 snap_bin.cpp）      │
│    环缓冲(RING_CAP=1200) 武装期持续记 → 触发回溯400 → 主缓冲    │
│    SNAP_CAP=3400 满 → SNAP DONE → ALIVE 2s → DUMP READY      │
├─────────────────────────────────────────────────────────────┤
│ L3 存储与交互  app/sd_fatfs.c + app/cli.c                     │
│    SD: SPIB + FatFS R0.15，自动存 /snap_*.bin + .txt 元数据    │
│    CLI: 串口命令（MONITOR/REC/SNAP?/DUMP BIN/HEX/SD/ABI?）    │
├─────────────────────────────────────────────────────────────┤
│ L4 PC 工具  D:\oezcon\TMS320\pc\abi_tjx.py                   │
│    串口 921600 → 自动拉帧 → CRC 校验 → 存 .bin/.csv → 出图      │
│    闭环测试日志解析（待办 C 起）                                │
└─────────────────────────────────────────────────────────────┘
```

- L1→L2 接口：`snap_on_event(t_us, counts, index_n)`（事件 ISR 内调用，极简）。
- L1→闭环接口：`abi_now_us()` + 事件差分 → 1kHz 速度估计流 `speed_est.c`
  （rpm 供闭环反馈与触发判定；ISR 内禁浮点/除法）。
- 2kHz 采样路径彻底删除（`Encoder_Count` 定时差分不再使用）。

## 4. L1 事件内核（方案 A，来自原设计 §3 与三朋友评审）

### 4.1 原理

```
QPOSCMP = QPOSCNT ± N   （N=1 逐计数；N=4 降速档；动态步长见待办 A）
PCSHDW=1 + PCLOAD=1      （影子使能；匹配瞬间硬件自动装载影子值，ISR 无竞争）
方向反转 → QDC 中断重新武装（防 QPOSCNT 背离 QPOSCMP 失配）
Index    → IEL 中断（每圈校准 + 圈计数）
三个源同一 INT_EQEP1，ISR 内读 QFLG 分发
```

### 4.2 ISR 流程

```
读 QFLG → 分发：
PCM:  先读 QPOSCNT + 时间戳（缩小窗口）
      delta = raw − raw_prev（±2³¹ 回绕修正）→ canonical counts += delta
      漏事件计数：|delta| > step 时 g_missed_events++
      t_us = µs 时钟差分 → 周期（整数 µs，rpm 移 1kHz）
      snap_on_event(now, counts, index_n)
      重新武装 QPOSCMP = raw + dir × step
QDC:  按新方向重新武装（raw_prev = raw）
IEL:  index_n++；若校准挂起 → canonical = index_n × 4000
清 QFLG（写完 CMP 后）→ 清 ACK
```

### 4.3 时间戳

- CPUTIMER0 自由运行（countdown，周期 = SYSCLK/1000 = 150000 tick @150MHz）。
- 1kHz ISR 维护：`g_us64`（每 tick +1000）+ `g_us_tick_cnt`（countdown 快照）。
- 亚 µs 插值：`t_us = g_us64 + ((g_us_tick_cnt − cnt + period) % period)/150`。
- u64 撕裂读：hi/lo/hi retry。
- PIE 组 1（TIMER0）天然优先组 5（EQEP1），g_us64 不漂移；代价 EQEP ISR
  偶尔被 1kHz 抢占（已含预算）。

### 4.4 ISR 开销预算（150MHz）

| 项 | 值 |
|---|---|
| 20000rpm 4X | 1.333M 边沿/s，0.75µs/边沿 ≈ 112 周期 |
| ISR 预算 | 0.6µs = 90 周期（禁浮点/除法） |
| N=1 满速 | ≈80% CPU —— **不可持续。⚠ 默认必须用 N≥4 进入 12000rpm 区**（否则系统 80% CPU 被 ISR 吞掉，不可用） |
| N=4 满速 | 333k 边沿/s → 3.0µs/边沿 → ≈22% CPU ✓ |
| 强制检查点 | Task 6 Step 4 实测 >0.6µs 立即降档 |

## 5. L2 记录引擎（移植 ESP32 v40 语义，协议不变）

- 常量：`SNAP_CAP=3400`、`RING_CAP=1200`、`BACKTRACK_N=400`、`steps=4000`。
- 点格式 16B packed LE：`<IqI`（t_us u32, counts i64, index_n u32），编译期
  `sizeof==16` 断言。
- 触发语义（照搬 ESP32）：武装期 `|rpm|>10` 进 STAGING；确认 OR：
  `{I+1圈, ≥200ms, ring≥120, (rpm跌落且ring≥40)}`；`rpm跌` = |rpm|<5。
- 回溯：`snap_trigger_from_ring(BACKTRACK_N)` 拷环缓冲末 400 点入主缓冲，
  t_us 整体平移到回溯起点为 0。
- 完成后：`# SNAP DONE` → ALIVE 2×1s → `# SNAP DUMP READY` → 自动 SD 存档。
- 记录期间串口静默（cli_muted），PC 自动 `DUMP BIN` 拉帧。
- CRC32：poly 0xEDB88320, init 0xFFFFFFFF, final ~（与 zlib 一致）。

## 6. L3 存储与交互

- SD：**SPIB（最终锁定：CLK=GPIO14, PICO=GPIO30, POCI=GPIO31, CS=GPIO6）**。
  - 已核实（SysConfig F28P55x.json 引脚复用数据 + 2026-08-02 评审确认）：128PDT 封装全部可用；
    GPIO14/30/31 硬件 SPIB Mux 支持，CS=GPIO6 软件片选（低有效，diskio 层 GPIO_writePin 控制，不占硬件 PTE）；
    与 EQEP1（50/51/53）、SCIA（28/29）、RGB（20/21）、板载 W25Q32 SPIA（0-3）均无冲突；
    GPIO6 原模板 EPWM4_A 已随 EPWM 模块删除而空闲。
  - 备选（**仅当实测 SD 读写不稳定时**切换）：CLK=GPIO4, PICO=GPIO7, POCI=GPIO6, CS=GPIO15（物理布局更紧凑）。
- FatFS R0.15（FF_USE_LFN=1, FF_FS_TINY=0, FF_MIN/MAX_SS=512）。写卡在主循环（非 ISR），
  期间停采样/停遥测，重试 ≤3 次，`# SD ERR <code>` 上报；写卡 RGB 绿闪。
- 存档名：`snap_YYYYMMDD_HHMMSS_n.bin`（TIME 对时后）或 `b<ms>`（未对时）；
  伴 `.txt` 元数据（stamp/iso_local/unix_ms/synced/n/bytes/crc/hz/steps/mode/
  board_ms/file）。
- CLI 命令（ESP32 v40 子集）：`MONITOR START|STOP`、`REC MS <ms>`、`SNAP?`、
  `SNAP STATUS`、`DUMP BIN`、`HEX DUMP`、`FW?`、`TIME <unix_ms>`、`PING`、
  `HELP`、`ABI?`、`SD?`、`SD INIT`、`SD TEST`、`SD DUMP <file>`。
- 遥测 `L,` 行（非记录时 10Hz；对齐 ESP32 20 字段）。

## 7. L4 PC 工具

- `D:\oezcon\TMS320\pc\abi_tjx.py`：pyserial 921600 8N1 + tkinter 精简 GUI +
  matplotlib 出图（rpm vs t_ms 非均匀时间轴 + counts vs t_ms）。
- 自动流程：TIME+START → 遥测显示 → 识别 SNAP DONE/ALIVE/DUMP READY →
  自动 DUMP BIN → CRC 校验 → 存 `.bin`/`.csv` → 出图；每步超时 5s 主动
  `SNAP STATUS` 查询兜底。
- `test_abi_tjx.py`：CRC/BIN END 正则/v2 解析/steps 覆盖/帧流组装 纯脚本断言。
- ESP32 工程（firmware/abi_monitor.py/curve_studio.py）一律只读不修改。

## 8. 闭环测试（保留，重基到 PCM）

- 测试序列：1500/2000 RPM 科目（斜坡 + 闭环，各 20s，间隔停 5s），
  **全部科目完成后永久停机**（`g_seq_done`，不再循环）。
- 速度反馈 = PCM 事件内核输出（speed_est 1kHz 流），非 1kHz 采样。
- ⚠ **实时性表述（评审确认）**：闭环 PI 控制器**仍以 1kHz 周期运行**
  （`Motor_CloseLoop` 每 500ms 控制一拍在 1kHz 时基上调度），但其 **rpm 输入
  由最近 PCM 事件差分 + 外推得到**，而非固定采样差分——闭环本身不是事件驱动，
  只是测速源换成了事件内核。
- 待办 C：闭环过程 CSV 日志（500ms/条：t_ms, rpm_target, rpm_act, pwm_out,
  pi_int 等），排原始计划后实施。

## 9. 内存与链接（最终结论）

- **F28P550SJ9 可用 SARAM ≈133KB**（M0/M1 + LS0-9 + GS0-3 + 可选 L1/L2）。
- SNAP 主缓冲 3400×16B = 54.4KB + RING 环缓冲 1200×16B = 19.2KB = **73.6KB，有余量**。
- 显式段 `SNAP_RAM > RAMGS0-3 | RAMLS8/9`（L1/L2 作为兜底，SysConfig 如可配为数据 RAM 则优先）；
  构建后 `.map` 强制检查 SNAP_RAM 存在且两缓冲落在 SRAM（**硬性规则：不足立即启用兜底 SNAP_CAP=2800/RING_CAP=800**）。
- 代码/常量/bss/栈 ~20KB 放 RAMLS0-7 + RAMM0/M1；FatFS 工作区 + 扇区缓冲 ~3KB。

## 10. 待办（原计划 13 Task 完成后）

| # | 内容 | 目标 |
|---|---|---|
| A | **QPOSCMP 动态步长**：按最近速度估计连续调整 N（低速 N=1 高精度、高速 N=8/16 降 ISR 率），迟滞防抖。**切换瞬间原则（提前定死，实现时无歧义）**：① 切换 N 时用当前 QPOSCNT 重新武装 CMP（不沿用旧目标，防失配）；② canonical counts 继续用硬件差值累加，切换本身不产生计数修正，保证连续；③ 切换窗口内漏事件计数（|delta|>step）单独观察记录，联调时核对切换瞬间无误报。 | 中断频率稳定在安全值，精度/范围兼得 |
| B | **CLB 方案对比试验**：CLB1（@0x3000，CLBINPUTXBAR 16 入、CLBOUTPUTXBAR、CLB1INT、4 深硬件 FIFO）计数器匹配+FIFO 免 CPU 时间戳，与 PCM 对比三指标：测速精度 / 中断稳定性 / 最大量程 | 验证 PCM 为最优解或改进方向 |
| C | **闭环测试日志 CSV（500ms/条）**：闭环过程记录到 SD | 供闭环调参分析 |

## 10. 原计划任务重基说明（delta 视角）

原计划 Task 1/2 已完成（环境 + 基线工程，commit 20834f5）。剩余任务重基：

| 原 Task | 变化 |
|---|---|
| 3 SysConfig | 基本不变；EQEP1 中断 = PCM/QDC/IEL；CPUTIMER0 1kHz；SPIB+CS |
| 4 PC 解析库 | 不变（协议未动），提前到新计划 Task 1 Step 1 |
| 5 snap_bin.c | 不变（接口原样） |
| 6 eqep_abi.c | 核心重基：**唯一测速源**；闭环反馈接入；ISR 预算实测为强制检查点 |
| 7 speed_est.c | 重基：rpm 全部来自 PCM 事件（删 1kHz 采样路径）；分档判定保留 |
| 8 cli.c | 不变 |
| 9 sd_fatfs.c | 链路验证优先：新计划 Task 1 先做 SD 冒烟（diskio+挂载），完整存档回 Task 6 |
| 10 main 状态机 | 集成 + 闭环测试序列（重基到 PCM 反馈，跑完即停） |
| 11 PC 工具 | 链路验证优先：Task 1 做骨架（连接/拉帧/存盘/出图），完整 GUI 回 Task 8 |
| 12 联调 | 以 PCM 事件为唯一数据源重跑 |
| 13 收尾 | 不变 |

## 11. 风险与对策（原 spec §13 增量）

| 风险 | 对策 |
|---|---|
| PCM 事件内核与闭环并存 → ISR 与 1kHz 竞争 | 预算已含；PIE 优先级天然保证时钟不漂移；实测检查点 |
| 删 2kHz 采样后闭环带宽下降 | 闭环科目固定 1500/2000 RPM 慢变，PCM 事件率充足（≥26.6kHz@1000rpm），无影响 |
| QPOSCMP 回绕/反向失配 | QPOSMAX=0xFFFFFFFF 天然模 2³² 安全；QDC 重武装；上板 Checklist 最高优先级 |
| 待办 A 动态步长改档瞬间丢事件 | 迟滞 + canonical 连续（QPOSCNT 差值恒精确）；漏事件计数观察 |
| 待办 B CLB 复杂度 | 仅为对比试验，不阻塞主线；试验结论归档 |

## 12. 上板验证 Checklist（★=最高优先级，沿用原方案）

- [ ] ★ **QPOSMAX→0 回绕瞬间的匹配行为**（CMP 回绕安全，失配则中断停摆）
- [ ] ★ **方向反转瞬间**（QDC 重武装窗口，换向不丢事件）
- [ ] **正交 4X 确认**：电驱动多圈多速，QPOSCNT 每圈恒 ≈4000
- [ ] **PCM N=1 逐计数**：低速每计数都进 ISR（漏事件计数验证）
- [ ] 100kHz 模拟脉冲测 ISR 实际开销（>0.6µs 立即降档）
- [ ] **时间戳插值公式**：已知频率脉冲校准（countdown 方向 + 跨 1kHz 边界取模）；**联调记录强制输出插值误差数据**（已知频率源下相邻事件 t_us 差分的均值/最大值/偏差），校准未通过前不得视为时间戳已验收
- [ ] eQEP 输入 qualifier 同步模式确认
- [ ] 921600 波特率实测偏差（LSPCLK=37.5MHz，BRR=4 → 937500，+1.72% < ±2%）
- [ ] PCM 匹配延迟 td(PCS-OUT)QEP=7 周期（记录为系统误差）
