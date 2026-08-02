# AbiMonitor_TJX 工作日志（TMS320F28P550 事件驱动测速记录器）

> 目的：按日期沉淀本项目的**初始计划、过程发现的问题、决策与依据**，作为以后 C2000/TMS320 项目的经验基础。
> 项目：`D:\oezcon\TMS320\AbiMonitor_TJX\`（固件）+ `D:\oezcon\TMS320\pc\`（PC 工具）。
> 相关文档：计划 `docs/superpowers/plans/2026-08-01-abimonitor-eventspeed-plan.md`（原计划，历史快照）；
> 设计 `docs/superpowers/specs/2026-08-01-abimonitor-eventspeed-design.md`（原设计，历史快照）；
> 重构 spec `docs/superpowers/specs/2026-08-02-abimonitor-pcm-rebuild-design.md`（PCM 唯一测速源，当前）；
> 重构 plan `docs/superpowers/plans/2026-08-02-abimonitor-pcm-rebuild-plan.md`（当前执行计划）；
> 问题与发现 `docs/2026-08-02_eQEP捕获中断缺失_问题与发现.md`；
> 新方案 `docs/2026-08-02_新方案_事件驱动测速记录器_PCM方案.md`。

---

## 项目概述

在天机星 TMS320F28P550 开发板上实现**事件触发（非均匀采样）测速记录器**：
EQEP 捕获每个 ABI 编码器边沿（1000PPR → 4X=4000 计数/圈），记录 16B 事件点 (t_us, counts, index_n)，
回溯 400 点 + 续记 2400 点，存档到 SD 卡，配套自建精简 PC 工具 `abi_tjx.py`。
ESP32 工程（`D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\`）只读，仅作移植参考。

- 设备：TMS320F28P550SJ9（**150MHz** C28x + CLA + NPU + TMU/FPU，1088KB FLASH + 133KB RAM）
- 外设分配：EQEP1（A=GPIO50 B=GPIO51 I=GPIO53）、SCIA 921600（TX=GPIO29 RX=GPIO28）、
  RGB_B=GPIO20 / RGB_G=GPIO21（低电平点亮）、SD 用 SPIB（避开 0-3/6/14/20/21/28/29/50/51/53）
- 工具链：CCS 20.1.1 + C2000Ware 5.04.00.00 + 编译器 22.6.1.LTS + SysConfig
- 记录规格：SNAP_CAP=3400 点、RING_CAP=1200 点、回溯 400 点；BIN 帧 `AA×10+55+magic 0xAB1C0002+...`
- 触发语义：|rpm|>10 进 STAGING，确认 OR{I+1圈, ≥200ms, ring≥120, (rpm跌落且ring≥40)}

---

## 时间线

### 2026-08-01：初始计划
- 编写实施计划（13 个 Task）+ 设计文档。核心架构假设：**eQEP 捕获单元（QCAP）"捕获周期就绪"中断**
  触发逐边沿 ISR（原计划写 `EQEP_INT_CAPTURE_PERIOD` 等枚举名，未经验证）。
- 计划要点（最初的）：
  - 每个 ABI 边沿进 ISR → 读 QCTMR 扩展 64 位时间戳（qctmr64 方案）→ 写事件点
  - 硬件 1X/4X 分辨率切换降 ISR 速率（`EQEP_CONFIG_4X_RESOLUTION`）
  - QCPRD 周期测速
  - 设计上限 20000rpm（4X 时 1.33M 边沿/s），ISR 预算 0.6µs
- Task 1/2 完成：环境安装、复制模板建工程 + 基线构建（commit 20834f5）。

### 2026-08-02：静态分析 → 重大发现（未上板即排除架构级坑）
- 核对 C2000Ware 5.04 头文件 + TI 官方例程，发现 **5 个问题**（详见下文"过程发现"）：
  1. eQEP 捕获单元**无中断**（原架构核心假设不成立）
  2. 原计划枚举名全部不存在
  3. QCPRD/QCTMR 是 16 位，低速溢出
  4. QCTMR 每捕获清零，不能作绝对时钟（qctmr64 方案作废）
  5. EPG 模块也无边沿测量中断
- 新增问题 6：**正交模式天然 4X**，无法硬件切 1X/4X（v2 修正，推翻"运行时切分辨率"设计）。
- 决策：方案 A（PCM 位置比较逐计数中断），详见下节。
- 归档立创 wiki 接线/入门文档到 `docs/lckfb_wiki/`。
- **主频修正：150MHz 非 200MHz**（朋友 G 指出，device.h L299 公式 + 数据手册确认）。
  连锁影响：ISR 预算 90 周期/0.6µs、QCPRD 溢出 436.9µs≈34.3rpm、921600 波特率误差 +1.72%（可直接用）。
- 三位朋友（G/k/dp）答复确认全部发现（存档 `docs/superpowers/specs/`）：
  - 方案 A 合理（k ★★★★ 首选）；PCLOAD=1 影子自动装载无竞争（dp）；漏事件计数是优雅降级（k）
  - DMA 是唯一"隐藏路径"但对本项目无用（dp）；CLB+XBAR+DMA / eCAP / GPIO XINT 均不改变选型
  - k 的加固建议已吸收进 Task 6：ISR 先读计数+时间戳、漏事件计数器、写完 CMP 再清标志、中高速自动加大 N
- **第三轮评审（G/k/dp 评审 PCM 方案，2026-08-02 晚）**，关键修正与确认：
  - **硬伤修正 1：事件率算错**。20000rpm 4X = 1.333M 边沿/s = 0.75µs/边沿 ≈112.5 周期（非 400kHz/2.5µs）。
    N=1 满速 80% CPU 占用不可持续 → 满速必须 GEAR_1X（N=4，22% CPU）。
  - **硬伤修正 2：ISR 内禁浮点/除法**。rpm 计算移出 ISR → 1kHz speed_est 用整数 period 差分换算；
    `abi_last_rpm` 从 eqep_abi 接口删除。
  - 确认：eQEP 输入同步模式最小周期 13.33ns（20000rpm 时 A/B 仅 333kHz，无压力，但 qualifier 必须同步模式）；
    LSPCLK 默认 /4=37.5MHz（BRR=4→937500 +1.72% ✓，勿改 LSPCLK 分频）；
    **PIE 组 1（TIMER0）天然优先于组 5（EQEP1）**，g_us64 不漂移（hw_ints.h L80/100）；
    PCM 匹配延迟 td(PCS-OUT)QEP=7 SYSCLK（固定，不影响差分测速）；
    QPOSMAX=0xFFFFFFFF 时 CMP=raw+dir×step 32 位截断天然回绕安全；
    漏事件合并点由 PC 端相邻点 counts 差分识别（16B 协议不变）；
    Index 校准只在低速稳定时执行；
    CLA 卸载 QPOSCNT 读/差分留作后期优化。
  - **验证方式更新**：4X 确认/PCM 逐计数/遥测等全部改为**电驱动多圈多速度**验证（100/500/2000/6000rpm 分段），手转仅作补充。
- 状态：Task 3（SysConfig 板级配置）进行中。

### 2026-08-02（晚）：方案确认 → 重构决策 + 电机跑完即停 + 无头构建/烧录打通
- **用户确认执行方向（brainstorming 澄清）**：
  - 1kHz 采样测速**作废**，**PCM 逐边沿事件内核 = 唯一测速源**（闭环反馈 + 记录都基于它，保证采集精确）。
  - 闭环测试（1500/2000 RPM 序列）保留，速度反馈重基到 PCM；科目跑完**永久停机**不再循环。
  - 执行顺序：① 链路验证（SD + PC UI + 串口交互，先用测试数据）② 弹射记录（PCM）③ 原计划剩余 11 Task 重基 ④ 待办：QPOSCMP 动态步长 + CLB 对比试验 + 闭环 CSV 日志（500ms/条）。
  - 文档策略：新写 spec/plan（`2026-08-02-abimonitor-pcm-rebuild-*`），原文档保留为历史快照。
- **电机跑完即停已实现并烧录**：`empty_driverlib_main.c` 新增 `g_seq_done`，状态机最后科目完成后永久停机（不再无限循环）；build_out22 构建 0 错误，dslite_load22 `Running... Success`。
- **无头构建方法打通（Theia 版 CCS 20）**：`ccs-server-cli.bat -workspace <ws> -application com.ti.ccs.apps.buildProject -ccs.projects AbiMonitor_TJX -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport`。
  - 踩坑记录：`ccs-serverc.exe -data ...` 只输出 "Initializing CCS... Done" 不构建（EXIT=0 假成功）；Theia `ccstudio.exe --no-splash --data ...` 无头崩溃（EPIPE/CSSUpdaterElectronApplicationContribution）；`-ccs.projects` 找不到项目时须加 `-ccs.autoImport`。
- **烧录命令（2026-08-02 实测）**：`DSLite.exe load --config="<proj>\targetConfigs\TMS320F28P550SJ9.ccxml" <out>` → `Running... Success`。

---

## 过程发现的问题（全部记录，供以后项目借鉴）

### 问题 1（核心）：F28P55x eQEP 捕获单元没有中断
- QCAPCTL 仅 UPPS/CCPS/CEN 三字段，**无中断使能位**（hw_eqep.h L123-130）。
- QFLG 13 个标志（INT/PCE/PHE/QDC/WTO/PCU/PCO/PCR/PCM/SEL/IEL/UTO/QMAE）**无捕获位**（L164-179）。
- TI 官方 ex4/ex5 均用 **UTO（单位时间超时）中断 + 轮询 QPOSCNT**，捕获单元只做硬件锁存。
- QCPRD/QCTMR 更新**不产生任何中断**；DMA 触发是唯一隐藏路径，但只能搬运数据、不能记时间戳。
- **教训**：写计划前先核对器件 TRM/头文件的中断源列表，别假设"外设应该有边沿中断"。

### 问题 2：原计划枚举名全部不存在（编译即错）
- `EQEP_INT_CAPTURE_PERIOD`/`EQEP_INT_INDEX_EVENT`/`EQEP_CAPTURE_*`/`EQEP_intEnable`/`EQEP_CONFIG_4X_RESOLUTION` 均不存在。
- 正确名：`EQEP_INT_INDEX_EVNT_LATCH`（eqep.h L144）、`EQEP_enableInterrupt(base,intFlags)`（L648）、
  `EQEP_enableCapture(base)` 单参数（L980）、`EQEP_setCompareConfig(base,config,compareValue,cycles)`（L1875）。
- **教训**：以头文件为准，先 grep 验证 API 名再写计划。

### 问题 3：QCPRD/QCTMR 是 16 位，低速直接溢出
- @150MHz 最大 65535 tick ≈ 436.9µs → 低于 2289 边沿/s（≈34.3rpm @4000 计数/圈）溢出，周期测速失效。
- CCPS 分频可降速但损失高速分辨率，且需按速度动态切换。

### 问题 4：QCTMR 每捕获边沿清零，不能作绝对时间戳
- 原 qctmr64 方案（回绕计数扩展 64 位时钟）作废 → 改用 **CPUTIMER0 自由运行 + 1kHz ISR 维护 64 位 µs 时钟**。

### 问题 5：EPG（增强 PWM 生成器）模块无边沿测量中断
- 只有 SIGGEN0_DONE/FILL（信号发生器输出侧），不能替代捕获中断。

### 问题 6：正交模式天然 4X，无法硬件切分辨率（v2 修正）
- 1X/2X 分辨率枚举映射 QDECCTL.XCR，**只对 CLOCK_DIR（QSRC=01）模式生效**；正交模式恒 4X。
- "运行时切 1X/4X 降 ISR 速率"做不到 → 改用 **PCM 步长 N**（软件等效降速）。

### 问题 7：主频认知修正（2026-08-02，朋友 G 指出）
- F28P55x = **150MHz**（非 200MHz）。证据：device.h L299 公式（20MHz×30/(2×1×2)=150MHz）+ 数据手册。
- 连锁修正：ISR 预算 90 周期、QCPRD 溢出 34.3rpm、921600 波特率误差 +1.72%（BRR=4→937500）。

---

## 决策记录

### D1（2026-08-02）：方案 A —— PCM 位置比较逐计数中断（已选用）
- 原理：QPOSCMP=QPOSCNT±N，PCSHDW=1+PCLOAD=1（匹配时影子自动装载），每个计数变化触发一次 PCM 中断。
- 中断源：PCM（位置比较匹配）+ QDC（方向改变，反向重新武装）+ IEL（Index 锁存），同一 INT_EQEP1 内 QFLG 分发。
- 时间戳：CPUTIMER0 自由运行 @150MHz + 1kHz ISR 维护 g_us64（每 tick +1000）+ countdown 快照插值（亚 µs 精度）。
- 周期测速：ISR 内 µs 差分；**不启用 QCAP**（16 位溢出且无中断，无收益）。
- 分档：GEAR_4X→N=1 逐计数、GEAR_1X→N=4（高速降 ISR 速率 /4）；canonical 4000 计数/圈恒精确。
- 优雅降级：ISR 追不上时 delta>1 用 QPOSCNT 差值追赶（计数精确、中间边沿时间丢失）+ 漏事件计数 g_missed_events。
- 加固（k 建议已吸收）：ISR 先读 QPOSCNT+时间戳再分发；写完 QPOSCMP 再清标志；中高速自动加大 N。
- 验证计划：Task 6 Step 4 强制检查点（100kHz 模拟脉冲测 ISR 开销，>0.6µs 立即降档）。

### D2（2026-08-02）：方案 B 备选（留作将来对比）
- UTO 周期采样（TI ex4 模式）：1kHz 采样最稳最简，但逐边沿事件时间全丢。
- 对比时机：Task 12 联调后把 eqep_abi.c 换成方案 B 对比。

### D3（2026-08-02）：921600 波特率直接用
- LSPCLK=37.5MHz（SYSCLK/4）时 BRR=4 → 实际 937500，误差 +1.72% <±2%，无需降 460800。
- 实测方法（Task 12 Step 11）：PC 发 1KB 定长包板端环回测实际偏差。

### D4（2026-08-02）：其他事件驱动路线评估（均不改变 D1）
- GPIO XINT：真边沿中断但无 4X 解码、噪声敏感（★★）
- CLB+XBAR+DMA：片上小 FPGA，每边沿时间戳+计数 DMA 搬运免 CPU，但开发成本高（★，后期优化）
- eCAP：有边沿中断但 1-2 路无法同时处理 A+B 正交，不适合

### D5（2026-08-02 晚）：PCM 事件内核 = 唯一测速源（用户确认，1kHz 采样作废）
- 原固件 1kHz 定时采样 Encoder_Count 差分测速（"原始方式"）**不再使用**；测速与记录唯一来源 = PCM 逐边沿事件内核（eqep_abi.c）。
- 闭环反馈重基：Motor_CloseLoop 的速度输入改 `spd_rpm()`（PCM 事件流），不读 1kHz 采样。
- 验证顺序：链路验证（SD+PC UI+交互，测试数据）→ 弹射记录（PCM）→ 原计划剩余 Task → 待办（QPOSCMP 动态步长/CLB 对比/闭环 CSV 日志）。
- 新文档：spec `2026-08-02-abimonitor-pcm-rebuild-design.md`、plan `2026-08-02-abimonitor-pcm-rebuild-plan.md`。

### D6（2026-08-02 晚）：电机测试跑完即停（不再无限循环）
- 原因：用户发现电机一直转——原状态机是无限循环测试序列。
- 实现：`g_seq_done` 标志；最后科目完成后永久 `Motor_Set_PWM(1,0)`；已构建（build_out22）并烧录（dslite_load22 Success）。

### D7（2026-08-02 晚）：Theia 版 CCS 20 无头构建正确方法
- `ccs-server-cli.bat`（非 ccs-serverc.exe / 非 theia ccstudio.exe）；`-workspace` 必须指向工程所在目录；首次 `-ccs.autoImport`；构建产物在 `CPU1_RAM\` 下。详见复用要点 14。

---

## 复用要点（以后 C2000/TMS320 项目直接可用）

1. **外设能力先查头文件再设计**：中断源列表在 QFLG/QEINT 等寄存器定义里；API 名以 `driverlib/xx.h` 枚举为准（grep 验证）。
2. **eQEP 三件套现实**：捕获单元纯锁存无中断；正交恒 4X；QCPRD 16 位。逐计数中断要用 PCM+QDC 组合（方案 A 是通用套路）。
3. **主频先确认**：F28P55x=150MHz（C2000 各型号不同，F2837x 是 200MHz）；系统时钟定义在 `device/device.h` 的 DEVICE_SYSCLK_FREQ 宏，公式可查。
4. **SCI 波特率手算**：BAUD = LSPCLK/((BRR+1)×8)，LSPCLK=SYSCLK/4；921600 @150MHz 用 BRR=4（+1.72%）。
5. **64 位 µs 时钟通用方案**：CPUTIMER 自由运行（countdown）+ 1kHz ISR 维护 u64 + countdown 快照插值 → 亚 µs 精度、撕裂读用 hi/lo/hi。
6. **QPOSCMP 影子装载**：PCSHDW=1+PCLOAD=1 时匹配瞬间硬件自动装载影子值，ISR 只需更新影子、写完再清标志。
7. **静态分析先于上板**：本次未上板就排除了架构级坑，省了大量调试时间；头文件+官方例程是最高效的证据源。
8. **主频修正连锁检查清单**：ISR 周期预算、定时器溢出阈值、波特率误差——主频改一处，三处都要重算。
9. **事件率/ISR 预算要先算对**：rpm×counts/rev/60 才是 4X 事件率（20000rpm@4000 = 1.333M/s，不是 400kHz）；
   满速 CPU 占用 = ISR 开销 ÷ 事件间隔，超 ~50% 就必须降档（本项目 N=4）。
10. **ISR 内禁浮点/除法**：换算（rpm）放 1kHz 定时任务，ISR 只做整数差分+累计。
11. **PIE 优先级查 hw_ints.h**：`INT_TIMER0=0x00260107U`（组1.7）、`INT_EQEP1=0x00400501U`（组5.1）——
    组号小优先，1kHz 时钟 ISR 天然优先于外设 ISR，g_us64 不漂移。
12. **F28P55x LSPCLK 默认 /4**（sysctl.h L557）：37.5MHz；改分频会动波特率误差，改前必算
    （/1 时 921600 → BRR=20 → 892857 −3.1% 超差）。
13. **eQEP 输入 qualifier 必须同步模式**（数据手册 6.16.5.1.1，最小周期 2×tc）；PCM 匹配延迟 7 SYSCLK（固定）。
14. **Theia 版 CCS 20 无头构建/烧录（2026-08-02 实测打通）**：
    - 构建：`D:\ti\ccs2011\ccs\eclipse\ccs-server-cli.bat -workspace <工程父目录> -application com.ti.ccs.apps.buildProject -ccs.projects <名> -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport`（autoImport 首次必需）。
    - 烧录：`D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe load --config="<proj>\targetConfigs\TMS320F28P550SJ9.ccxml" <out>`；成功标志 `Running... Success`。
    - 陷阱：`ccs-serverc.exe` 只初始化不构建（假成功）；Theia ccstudio.exe 无头崩溃；改源码后必须把文件同步到构建副本（Copy-Item）再构建。
