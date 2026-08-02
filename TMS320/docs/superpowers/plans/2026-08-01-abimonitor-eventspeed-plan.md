# AbiMonitor_TJX 事件测速监控器 实施计划（阶段1）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在天机星 TMS320F28P550 开发板上实现事件触发（非均匀采样）测速记录器：EQEP 硬件捕获每个 ABI 边沿，回溯 400 点 + 续记 2400 点，串口/事件点存档到 SD，配套自建精简 PC 工具；ESP32 工程一律不动。

**Architecture:** 事件驱动测速。EQEP1 位置比较（PCM）逐计数中断 ISR（⚡ 捕获单元无中断，方案 A 见决策记录）把 16B 事件点 (t_us, counts, index_n) 写入环缓冲/主缓冲；1kHz CPU 定时器做速度估计（实测+外推）、分档判定（PCM 步长 1/4 + 迟滞）、10Hz 遥测；记录完成后 ALIVE 2s 再写 SD。PC 端全新自建 `abi_tjx.py`（精简 tkinter + 解析库 + 测试），协议与 ESP32 v40 对齐。

**Tech Stack:** CCS 20.1.1（Theia）+ C2000Ware 5.04.00.00 + TI C2000 编译器 22.6.1.LTS（`--abi=eabi`）+ SysConfig 1.23 + driverlib（C28x 150/200MHz）+ FatFS R0.15 + Python3（pyserial/matplotlib，Windows 10/11）。设计文档：`TMS320/docs/superpowers/specs/2026-08-01-abimonitor-eventspeed-design.md`。

## Global Constraints

- **ESP32 工程只读**：`D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\` 下固件、abi_monitor.py、curve_studio.py 等一律**不修改**；参考只用于移植。
- 工具链安装目标 `D:\ti\`（CCS + C2000Ware_5_04_00_00 + ti_emupack XDS110 驱动）；安装包在 `D:\oezcon\TMS320\05-【TMS320F28P550】开发工具\`。
- 工程位置 `D:\oezcon\TMS320\AbiMonitor_TJX\`（复制自 N20 模板 `TJX-TMS320F28P550-ProjectTemplate\`）；PC 工具 `D:\oezcon\TMS320\pc\`。
- 设备：TMS320F28P550SJ9，128PDT，EQEP1（A=GPIO50 B=GPIO51 I=GPIO53），SCIA（TX=GPIO29 RX=GPIO28，**921600** 8N1），RGB_B=GPIO20 / RGB_G=GPIO21（低电平点亮），SPIA GPIO0-3 板载 W25Q32 **禁用**，SD 用 **SPIB**（具体引脚任务 2 锁定，避开 0-3/6/14/20/21/28/29/50/51/53）。
- 记录规格：SNAP_CAP=3400 点×16B、RING_CAP=1200 点×16B、回溯 BACKTRACK_N=400；canonical counts（4000 步/圈恒定）→ `BIN END steps=4000`；触发语义照搬 ESP32：|rpm|>10 进 STAGING，确认条件 OR{I+1圈, ≥200ms, ring≥120, (rpm跌落且ring≥40)}。
- 点格式 16B packed LE：`t_us u32, counts i64, index_n u32`；BIN 帧：`AA×10+55 + magic 0xAB1C0002 + n u16 + hz u16(=0) + 点阵 + crc32(payload) u32`；CRC32 位与 ESP32 一致（poly 0xEDB88320, init 0xFFFFFFFF, final ~）。
- 内存：两大缓冲显式段放置；首选 SysConfig 把 L1/L2 SARAM 配为数据 RAM（任务 2 验证，TRM 内存映射为准）；**兜底**（L1/L2 不可用）：SNAP_CAP=2800、RING_CAP=800，且 FatFS 工作区压缩。
- 每次构建：`CPU1_RAM` 配置（RAM 调试）；最终 `CPU1_FLASH` 烧录回归。
- 每个固件任务验证 = 构建通过 + 烧录后串口断言（`SNAP?`/`ABI?`/`SNAP STATUS`/`HEX DUMP`/`FW?`）；PC 任务验证 = `python test_abi_tjx.py` 全绿。
- FatFS R0.15（elm-chan，`https://elm-chan.org/fsw/ff/arc/ff15.zip`），FF_USE_LFN=1、FF_LFN_UNICODE=0、FF_USE_STRFUNC=0；无网络时改用 FF_USE_LFN=0（文件名缩为 8.3 短名）。
- 波特率：921600。**主频已确定为 150MHz**（device.h L299 公式 20MHz×30/(2×1×2)，用户确认 F28P55x 为 150MHz）；LSPCLK=37.5MHz 时 BRR=4 → 实际 937500，误差 **+1.72% < ±2%**，921600 可直接用，无需降 460800（2026-08-02 修正，原按 200MHz 估算 -3.1% 作废）。**实测方法（评审 G/dp）：PC 发 1KB 定长包，板端环回，测收发时间差计算实际波特率偏差（Task 12 Step 11）。**

---

### 决策记录（2026-08-02）：eQEP 捕获单元无中断 → 方案 A（PCM 逐计数中断）

**问题**（详见 `TMS320/docs/2026-08-02_eQEP捕获中断缺失_问题与发现.md`）：
F28P55x eQEP 捕获单元**没有中断**：QCAPCTL 仅 UPPS/CCPS/CEN 三字段（hw_eqep.h L123-130）、QFLG 仅 13 个标志无捕获位（L164-179）、TI 官方 eqep_ex4 用 UNIT_TIME_OUT 中断（标题 "via unit timeout interrupt"）。原计划枚举 `EQEP_INT_CAPTURE_PERIOD`/`EQEP_INT_INDEX_EVENT`/`EQEP_CAPTURE_*` 均不存在；Index 正确名 **`EQEP_INT_INDEX_EVNT_LATCH`**（eqep.h L144）。QCPRD/QCTMR 是 **16 位**（SYSCLK=150MHz 时 65535 tick ≈ 436.9µs，约 34.3rpm 以下失效）；QCTMR 每捕获事件清零，**不能作绝对时钟**（原 qctmr64 方案作废）。EPG 只有 SIGGEN0_DONE/FILL 中断，无边沿测量中断。**正交模式下计数天然 4X**（1X/2X 分辨率枚举是 QDECCTL.XCR，仅 CLOCK_DIR 模式生效），无法硬件切分辨率降 ISR 速率。

**方案 A（已选用）——位置比较逐计数中断**：
- 初始化：`EQEP_enableCompare()`（QPOSCTL.PCE=1）；QPOSCTL 裸写 **PCSHDW=1、PCLOAD=1**（影子使能、匹配装载）；`EQEP_setCompareConfig(base, EQEP_COMPARE_NO_SYNC_OUT, QPOSCNT±1, 0)` 武装；`EQEP_enableInterrupt(base, EQEP_INT_POS_COMP_MATCH | EQEP_INT_DIR_CHANGE | EQEP_INT_INDEX_EVNT_LATCH)`。
- ISR（`INT_Module_EQEP_ISR`，同一条 INT_EQEP1 分发）：PCM → 读 QPOSCNT 差分（±2³¹ 回绕修正）累计 canonical counts、读 µs 时钟得 t_us、µs 差分得周期、`snap_on_event`、按 `EQEP_getDirection()`（返回 int16_t ±1）重新武装 QPOSCMP=QPOSCNT±1；QDC → 按新方向重新武装（防反向背离失配）；IEL → index_n++ 与绝对校准。
- 时间戳：**CPUTIMER0 自由运行 @SYSCLK + 1kHz ISR 维护 64 位回绕计数**（不用 QCTMR）。
- 周期测速：ISR 内 µs 差分，**不启用 QCAP**（16 位溢出且无中断，无收益）。
- 降速分档（GEAR）：正交模式无硬件 1X/4X 可切 → **PCM 步长 N**（GEAR_4X→N=1 逐计数、GEAR_1X→N=4），ISR 速率 /N；QPOSCNT 差值恒精确，canonical 不丢。
- ISR 追不上时：delta>1 追赶，计数精确、中间边沿时间丢失（优雅降级）。

**方案 B（备选，留作将来对比）**：UNIT_TIME_OUT 周期采样（TI ex4 模式），UTO 中断轮询 QPOSCNT/QCPRD，1kHz 采样实现最简最稳，但每 tick 只记 1 点、逐边沿事件时间全丢。
**对比时机**：Task 12 联调后，可选把 `app/eqep_abi.c` 换成方案 B 实现，对比事件时间分辨率与 CPU 开销。

**三朋友答复确认（2026-08-02，G/k/dp 三份答复已存档 `TMS320/docs/superpowers/specs/`）**：
- 三人一致确认：QCAP 无中断（11 个中断源 PCE/PHE/QDC/WTO/PCU/PCO/PCR/PCM/SEL/IEL/UTO，无捕获位）、正交模式恒 4X、XCR 只对 CLOCK_DIR 生效、QCPRD 16 位、QCTMR 清零；方案 A 合理（k 评 ★★★★ 首选，dp 称"极具创造性变通"，TI E2E 有人讨论过类似做法）。
- **主频修正：F28P55x = 150MHz**（非 200MHz；device.h L299 公式与用户确认一致）。全计划已改为 150MHz 口径：ISR 预算 90 周期/0.6µs、QCPRD 溢出 436.9µs≈34.3rpm、921600 波特率误差 +1.72% 可用。
- PCLOAD=1 匹配自动装载影子值：dp 确认 ISR 只需更新影子值、匹配时硬件自动装载，无竞争（前提 ISR 足够快）；k 强调**写完 QPOSCMP 再清标志**。
- 已吸收的加固项（k）→ Task 6 Step 2 代码：① ISR 开头先读 QPOSCNT + 时间戳再分发标志（缩小窗口）；② 加**漏事件计数器**（本次 delta > step 时累加，联调观察优雅降级）；③ 中高速自动加大 N（按最近速度估计）；④ 回绕/QPOSMAX 行为上板实测（Task 6 Step 3 冒烟 + Task 11/12 联调）。
- G 建议：Task 6 不要直接写整套业务，先做极简 PCM 验证程序（100kHz 模拟脉冲测 CPU 占用）。→ 已落实：Task 6 Step 4 是强制检查点（ISR 开销实测，>0.6µs 立即降档），不再新增任务。
- 其他路线记录（dp/G）：GPIO XINT 边沿中断（无 4X 解码，噪声敏感 ★★）、CLB+XBAR+DMA（片上小 FPGA，开发成本高 ★ 后期优化）、eCAP（F28P55x 有 eCAP 模块，但 1-2 路无法同时处理 A+B 正交，不适合）——均不改变当前方案 A 选型。

---

### Task 1: 环境安装（CCS 20.1.1 + C2000Ware 5.04 + XDS110 驱动）

**Files:**
- Install: `D:\ti\ccs20xx\`（CCS 20.1.1）、`D:\ti\c2000ware\C2000Ware_5_04_00_00\`、XDS110 驱动（ti_emupack）
- 来源：`D:\oezcon\TMS320\05-【TMS320F28P550】开发工具\`（CCS_20.1.1.00008_win.zip、C2000Ware_5_04_00_00_setup.exe、ti_emupack…）

**Interfaces:**
- Produces: `C2000WARE_ROOT` 环境变量/CCS 首选项指向 D:\ti 安装（驱动 driverlib.lib 与 device 支持路径）；CCS 可启动、XDS110 可枚举。

- [ ] **Step 1: 解压 CCS 并安装**
  - 7-Zip 解压 `CCS_20.1.1.00008_win.zip` 到 `D:\temp\opencode\ccs_setup\`，运行安装器（GUI，接受默认组件 + 勾选 C2000 器件支持；安装目录改为 `D:\ti\ccs20xx`）。
  - 预期：`D:\ti\ccs20xx\ccs\eclipse\ccstudio.exe` 存在。
- [ ] **Step 2: 安装 C2000Ware**
  - 静默或 GUI 安装 `C2000Ware_5_04_00_00_setup.exe` 到 `D:\ti\c2000ware\`。
  - 预期：`D:\ti\c2000ware\C2000Ware_5_04_00_00\driverlib\f28p55x\` 存在。
- [ ] **Step 3: 安装 XDS110 驱动（ti_emupack）**
  - 运行 emupack 安装器；预期设备管理器出现 `TI XDS110 Debug Probe`（未插板则安装后待验证）。
- [ ] **Step 4: 验证安装**
  - 启动 ccstudio.exe 一次建默认工作区；CCS 首选项 → Products/C2000Ware 定位 `D:\ti\c2000ware\C2000Ware_5_04_00_00`。
  - 预期：CCS 能列出 TMS320F28P550SJ9 器件并显示 C2000Ware 已识别（Products 视图）。
- [ ] **Step 5: Commit**
  ```bash
  # 环境安装在仓库外，无可提交文件；在 docs 记录一条环境说明（写入 Task 12 的完成文档）
  ```
  （此步仅记录，不产生提交。）

---

### Task 2: 复制模板建 AbiMonitor_TJX 工程 + 基线构建

**Files:**
- Copy: `D:\oezcon\TMS320\立创·天机星TMS320F28P550开发板【模块移植代码】\控制类\N20直流减速电机-带霍尔编码器\TJX-TMS320F28P550-ProjectTemplate\` → `D:\oezcon\TMS320\AbiMonitor_TJX\`
- Delete: `AbiMonitor_TJX\CPU1_RAM\`、`AbiMonitor_TJX\CPU1_FLASH\`（陈旧产物，含他人机器绝对路径）
- Modify: `AbiMonitor_TJX\.project`（工程名）、`AbiMonitor_TJX\.ccsproject`（可保留）

**Interfaces:**
- Consumes: Task 1 安装（C2000Ware 路径）。
- Produces: 可构建的 RAM 配置基线（含 SysConfig 预生成产物 `CPU1_RAM\syscfg\board.c/h`）；后续所有任务在此工程内加文件。

- [ ] **Step 1: 复制工程并清理**
  ```powershell
  Copy-Item -Recurse "D:\oezcon\TMS320\立创·天机星TMS320F28P550开发板【模块移植代码】\控制类\N20直流减速电机-带霍尔编码器\TJX-TMS320F28P550-ProjectTemplate" "D:\oezcon\TMS320\AbiMonitor_TJX"
  Remove-Item -Recurse "D:\oezcon\TMS320\AbiMonitor_TJX\CPU1_RAM","D:\oezcon\TMS320\AbiMonitor_TJX\CPU1_FLASH"
  ```
- [ ] **Step 2: 改名工程**
  - 编辑 `AbiMonitor_TJX\.project`：`<name>TJX-TMS320F28P550-ProjectTemplate</name>` → `<name>AbiMonitor_TJX</name>`（唯一匹配）。
- [ ] **Step 3: 导入并配置 C2000Ware 路径**
  - CCS：File → Import → CCS Projects，选 `D:\oezcon\TMS320\AbiMonitor_TJX`，构建配置只留 `CPU1_RAM`、`CPU1_FLASH`（删 LAUNCHXL 两个）。
  - Window → Preferences → Products：指向 `D:\ti\c2000ware\C2000Ware_5_04_00_00`。
- [ ] **Step 4: 基线构建 CPU1_RAM**
  - 预期：编译 0 错误 0 警告；`AbiMonitor_TJX\CPU1_RAM\AbiMonitor_TJX.out` + `.map` 生成；map 中 RAMLS4/6/7、RAMGS0-3、RAMLS8/9 空闲（基线内存清单记录备用）。
  - 若警告非零：先记下，任务 4 起逐一消除（不得静默忽略）。
- [ ] **Step 5: 烧录验证（可选，板已在手时）**
  - 连接 XDS110 → Debug 启动 CPU1_RAM 配置 → 串口助手 115200 看到模板 banner 与 Encoder_Count 滚动。
  - 预期：banner 与计数输出正常（这同时验证 EQEP1 A/B 接线 GPIO50/51 有效——板载编码器例程）。
- [ ] **Step 6: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/.project TMS320/AbiMonitor_TJX/.cproject TMS320/AbiMonitor_TJX/.ccsproject TMS320/AbiMonitor_TJX/c2000.syscfg TMS320/AbiMonitor_TJX/empty_driverlib_main.c TMS320/AbiMonitor_TJX/*.cmd TMS320/AbiMonitor_TJX/device TMS320/AbiMonitor_TJX/lckfb_tjx_init TMS320/AbiMonitor_TJX/module_driver TMS320/AbiMonitor_TJX/targetConfigs
  git commit -m "feat: AbiMonitor_TJX project from N20 template (baseline RAM build)"
  ```

---

### Task 3: SysConfig 板级配置（921600 / EQEP1 捕获+Index / 1kHz 定时器 / SPIB / L1-L2 RAM）

**Files:**
- Modify: `AbiMonitor_TJX\c2000.syscfg`（SysConfig GUI 编辑，产物 board.c/h 生成于 `CPU1_RAM\syscfg\`）
- Modify: `AbiMonitor_TJX\28p55x_generic_ram_lnk.cmd` 与 `28p55x_generic_flash_lnk.cmd`（L1/L2 区域与显式段）
- 参考：`D:\oezcon\TMS320\03-【TMS320F28P550】开源软件\05_timer_example\c2000.syscfg`（cputimer.js 用法）、`08_spi_example\c2000.syscfg`（spi.js 用法）

**Interfaces:**
- Consumes: Task 2 基线工程。
- Produces（后续任务依赖的生成宏，build 后出现在 `syscfg\board.h`）：
  - `Debug_Serial_BASE`(=SCIA_BASE)、`Debug_Serial_BAUDRATE`(921600)
  - `Module_EQEP_BASE`(=EQEP1_BASE)、`INT_Module_EQEP_ISR` 注册、`INT_Module_EQEP_INTERRUPT_ACK_GROUP`
  - `myCPUTIMER0`（或自命名）定时器宏 `Module_TIMER0_BASE`(=CPUTIMER0_BASE)、`INT_TIMER0_ISR`
  - `SPIB` 宏（`SD_SPI_BASE`、`SD_CS` GPIO 宏）
  - `RGB_B`/`RGB_G`（GPIO20/21，active-low）
  - 内存：L1/L2 段可用（链接 cmd 追加 `SNAP_RAM` 段）

- [ ] **Step 1: 打开 c2000.syscfg，删除 EPWM 模块**
  - 删除 `Module_PWM_M`（EPWM4/GPIO6）、`Module_PWM_P`（EPWM8/GPIO14）两个实例（本工程不用电机 PWM）。
- [ ] **Step 2: SCIA 波特率 921600**
  - `Debug_Serial` 实例 Baud Rate 改为 **921600**；保留 RX 中断、非 FIFO。
  - 记录项：若任务 6 实测收发异常，按 Global Constraints 降 460800。
- [ ] **Step 3: EQEP1 配置（PCM 逐计数中断 + Index，见决策记录）**
  - `Module_EQEP` 实例：A=GPIO50、B=GPIO51，**新增 Index=GPIO53**（EQEP1_INDEX）。
  - 解码器：`EQEP_setDecoderConfig` 用 `EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE`（⚠ 本器件无 4X/1X 分辨率枚举，XCR 仅 CLOCK_DIR 模式；正交模式天然 4X=4000 计数/圈）。
  - 位置计数器模式：`EQEP_POSITION_RESET_MAX_POS`、max=0xFFFFFFFF；**关** unit timer（模板的 50ms 锁存路径弃用）。
  - **位置比较（PCM）**：`EQEP_enableCompare()`（PCE=1）；QPOSCTL 裸写 **PCSHDW=1、PCLOAD=1**（driverlib 无封装）；`EQEP_setCompareConfig(base, EQEP_COMPARE_NO_SYNC_OUT, QPOSCNT±1, 0)` 武装首值。
  - **中断源（同一 INT_EQEP1）**：`EQEP_INT_POS_COMP_MATCH`（逐计数）+ `EQEP_INT_DIR_CHANGE`（反向重新武装）+ `EQEP_INT_INDEX_EVNT_LATCH`（Index 事件，⚠ 不是 INDEX_EVENT）。
  - **不启用 QCAP 捕获单元**（无中断源 + 16 位 QCPRD 低速溢出；周期测速改由 ISR 内 µs 差分，见 Task 6）。
  - 勾选 `registerInterrupts`（生成 `INT_Module_EQEP_ISR` 骨架，Task 6 实现）。
  - ⚠ 枚举以本工程 `device\driverlib\eqep.h` / `inc\hw_eqep.h` 实际为准：`EQEP_INT_INDEX_EVNT_LATCH`（L144）、`EQEP_enableInterrupt(base,intFlags)`（L648）、`EQEP_getDirection()` 返回 int16_t ±1（L597）、`EQEP_setCompareConfig(base,config,compareValue,cycles)`（L1875，内部写 QPOSCMP + QPOSCTL）、`EQEP_getCapturePeriod/Timer` 返回 uint16_t（L1030/L1056）；不存在 `EQEP_INT_CAPTURE_PERIOD`/`EQEP_INT_INDEX_EVENT`/`EQEP_CAPTURE_BOTH_RISING_FALLING`/`EQEP_CONFIG_4X_RESOLUTION`。
- [ ] **Step 4: 新增 CPU Timer（1kHz）**
  - 仿 `05_timer_example\c2000.syscfg`：cputimer.js 实例 `Module_TIMER0`（CPUTIMER0），period=SYSCLK/1000，startTimer、enableInterrupt、registerInterrupts。
- [ ] **Step 5: 新增 SPIB + CS GPIO（SD 卡）**
  - spi.js 实例 `SD_SPI`：**SPIB**，controller、8-bit、无 FIFO 无中断；引脚（已按 F28P55x.json 复用数据核实）：CLK=**GPIO14**（mode9）、PICO=**GPIO30**（mode3）、POCI=**GPIO31**（mode3）、CS=**GPIO6**（普通 GPIO 输出）；四脚在 U21/U22 排针同一片区（28/29 为 SCIA 保留不动）。注意：GPIO32 虽为 SPIB_CLK mode3 但板上排针未引出，不可用；GPIO6/14 原被模板 EPWM4/EPWM8 占用，Task 3 Step 2 删除 EPWM 实例后即空闲。
  - ⚠ 逐一核对 SysConfig 冲突提示；选中后把最终引脚号记录到 spec §9 表格（编辑 `TMS320/docs/superpowers/specs/2026-08-01-abimonitor-eventspeed-design.md` §9 三行）。
- [ ] **Step 6: L1/L2 SRAM 内存验证与链接脚本**
  - SysConfig → Memory 配置把 L1/L2 SRAM 分配为 CPU 数据 RAM（以 TRM 与 SysConfig 实际选项为准）。
  - 打开 `D:\oezcon\TMS320\06-官方资料` 下 F28P55x TRM 内存映射节确认 L1/L2 地址与长度，把 region 追加进两个 `.cmd` 的 MEMORY（如 `L1SRAM origin=0x... length=0x4000`、`L2SRAM ...`；地址以 TRM 实际为准）。
  - 在两个 `.cmd` 的 SECTIONS 增加：
    ```
    SNAP_RAM  > L1SRAM | L2SRAM
    ```
    （若两段都进不了 SNAP_RAM，先放 L1SRAM；构建后核对 .map。）
  - **兜底**：若 SysConfig/TRM 确认无 L1/L2 可用 → 按 Global Constraints：SNAP_CAP=2800、RING_CAP=800，并后续任务同步改常量（Task 4 Step 1）。
- [ ] **Step 7: 构建 CPU1_RAM 验证**
  - 预期：SysConfig 重新生成 board.c/h；编译 0 错误；.map 中 `SNAP_RAM` 段存在且 L1/L2 区域被占用。
  - **硬性规则（评审 G）**：若 .map 显示 SNAP_RAM 不足 → 立即启用兜底（2800/800，见 Global Constraints），不得带病进入联调。
- [ ] **Step 8: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/c2000.syscfg TMS320/AbiMonitor_TJX/28p55x_generic_ram_lnk.cmd TMS320/AbiMonitor_TJX/28p55x_generic_flash_lnk.cmd TMS320/docs/superpowers/specs/2026-08-01-abimonitor-eventspeed-design.md
  git commit -m "feat: syscfg pins/baud/capture/timer/SPIB + L1-L2 SNAP_RAM section"
  ```

---

### Task 4: PC 解析库先行（abi_tjx 核心，TDD 纯 Python）

**Files:**
- Create: `D:\oezcon\TMS320\pc\abi_tjx.py`（第 1 版：解析库；GUI 在 Task 10）
- Create: `D:\oezcon\TMS320\pc\test_abi_tjx.py`（纯脚本 assert，仿 `ESP32_AS5047P_ABI_Monitor\pc\test_snap_parse_offline.py` 风格）

**Interfaces:**
- Consumes: 无（纯 Python；解析格式来自 spec §5 与 ESP32 BIN 帧定义）。
- Produces（Task 10 与测试依赖的精确签名）：
  - `SNAP_PREAMBLE: bytes`（`b"\xAA"*10+b"\x55"`）
  - `SNAP_MAGIC_V2 = 0xAB1C0002`；`SNAP_POINT_SIZE_V2 = 16`
  - `def snap_crc32(data: bytes) -> int`（逐位 CRC-32，poly 0xEDB88320，init 0xFFFFFFFF，final XOR ~，与 zlib.crc32 结果一致）
  - `def parse_bin_end(line: str) -> dict | None`（正则解析 `# BIN END n= hz= steps= mode= bytes= crc=`，字段顺序可变，返回 dict 或 None）
  - `def parse_snap_bindump(raw: bytes, steps: int = 4000) -> tuple[int, int, list[tuple]]`（`(n, hz, rows)`；rows 为 `(t_ms, rpm, dir, seg=1, index_n, t_rel=0, unix=0, counts)` 8 元组；`hz=0` 合法）
  - `def rpm_from_counts_series(t_us: list[int], counts: list[int], *, steps: int = 4000, vel_win: int = 16) -> list[float]`
  - `def parse_bin_frame(buf: bytes) -> tuple[int, int, list[tuple]] | None`（帧组装器输出→解析，容错 preamble 前缀）

- [ ] **Step 1: 写失败测试 `test_abi_tjx.py`**

```python
import struct, sys, zlib
sys.path.insert(0, r"D:\oezcon\TMS320\pc")
from abi_tjx import (SNAP_MAGIC_V2, SNAP_POINT_SIZE_V2, snap_crc32,
                     parse_bin_end, parse_snap_bindump, parse_bin_frame,
                     rpm_from_counts_series)

def _make_v2(t_us, counts, idx):
    return struct.pack("<IqI", t_us, counts, idx)

def main() -> int:
    assert snap_crc32(b"") == 0
    assert snap_crc32(b"123456789") == 0xCBF43926
    assert snap_crc32(b"abc") == zlib.crc32(b"abc")

    m = parse_bin_end("# BIN END n=10 hz=0 steps=4000 mode=event bytes=172 crc=0x12345678")
    assert m and m["n"] == 10 and m["steps"] == 4000 and m["mode"] == "event"

    n = 64
    payload = b"".join(_make_v2(i * 500, i, 0) for i in range(n))
    crc = snap_crc32(payload)
    blob = struct.pack("<IHH", SNAP_MAGIC_V2, n, 0) + payload + struct.pack("<I", crc)
    gn, ghz, rows = parse_snap_bindump(blob)
    assert gn == n and ghz == 0 and len(rows) == n
    assert abs(rows[-1][1] - 30.0) < 0.5, rows[-1][1]   # 1/4000*1e6/500*60=30
    assert rows[-1][7] == n - 1                          # counts 透传

    rpms = rpm_from_counts_series([i*500 for i in range(n)], list(range(n)), steps=4000)
    assert abs(rpms[-1] - 30.0) < 0.5

    frame = b"\xAA"*10 + b"\x55" + blob
    r2 = parse_bin_frame(frame)
    assert r2 and r2[0] == n

    try:
        blob_bad = blob[:-1]
        parse_snap_bindump(blob_bad)
        return 1
    except ValueError:
        pass
    print("test_abi_tjx: ALL PASS")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: 运行确认失败**
  Run: `python test_abi_tjx.py`（在 `D:\oezcon\TMS320\pc\`）
  Expected: ImportError（abi_tjx.py 不存在）
- [ ] **Step 3: 实现 `abi_tjx.py` 解析库**

```python
import re, struct
SNAP_PREAMBLE = b"\xAA" * 10 + b"\x55"
SNAP_MAGIC_V2 = 0xAB1C0002
SNAP_POINT_SIZE_V2 = 16
SNAP_POINT_FMT = "<IqI"          # t_us u32, counts i64, index_n u32 (LE)

def snap_crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1))
    return ~crc & 0xFFFFFFFF

_BIN_END_RE = re.compile(
    r"# BIN END n=(\d+) hz=(\d+) steps=(-?\d+) mode=(\w+) bytes=(\d+) crc=0x([0-9A-Fa-f]+)")

def parse_bin_end(line: str) -> dict | None:
    m = _BIN_END_RE.search(line)
    if not m:
        return None
    return {"n": int(m.group(1)), "hz": int(m.group(2)), "steps": int(m.group(3)),
            "mode": m.group(4), "bytes": int(m.group(5)), "crc": int(m.group(6), 16)}

def rpm_from_counts_series(t_us, counts, *, steps=4000, vel_win=16):
    out = [0.0] * len(counts)
    w = max(1, int(vel_win))
    for i in range(len(counts)):
        j = i - w if i >= w else 0
        dc = counts[i] - counts[j]
        dt = t_us[i] - t_us[j]
        if dt <= 0:
            out[i] = out[i - 1] if i else 0.0
            continue
        out[i] = (dc / steps) * (1_000_000.0 / dt) * 60.0
    return out

def parse_snap_bindump(raw: bytes, steps: int = 4000):
    if len(raw) < 12:
        raise ValueError(f"bin too short: {len(raw)}")
    magic, n, hz = struct.unpack_from("<IHH", raw, 0)
    if magic != SNAP_MAGIC_V2:
        raise ValueError(f"bad magic 0x{magic:08X}")
    need = 8 + n * SNAP_POINT_SIZE_V2 + 4
    if len(raw) < need:
        raise ValueError(f"bin len {len(raw)} < need {need}")
    payload = raw[8:8 + n * SNAP_POINT_SIZE_V2]
    (crc,) = struct.unpack_from("<I", raw, 8 + n * SNAP_POINT_SIZE_V2)
    got = snap_crc32(payload)
    if got != crc:
        raise ValueError(f"CRC mismatch got=0x{got:08X} expect=0x{crc:08X}")
    t_list, c_list, idx_list = [], [], []
    for i in range(n):
        t_us, counts, index_n = struct.unpack_from(SNAP_POINT_FMT, payload, i * SNAP_POINT_SIZE_V2)
        t_list.append(int(t_us)); c_list.append(int(counts)); idx_list.append(int(index_n))
    rpms = rpm_from_counts_series(t_list, c_list, steps=steps)
    rows = []
    for i in range(n):
        rpm = rpms[i]
        direc = 1 if rpm > 0.5 else (-1 if rpm < -0.5 else 0)
        rows.append((t_list[i] / 1000.0, rpm, direc, 1, idx_list[i], 0, 0, c_list[i]))
    return n, hz, rows

def parse_bin_frame(buf: bytes):
    if buf.startswith(SNAP_PREAMBLE):
        buf = buf[len(SNAP_PREAMBLE):]
    if len(buf) < 12:
        return None
    return parse_snap_bindump(buf)
```

- [ ] **Step 4: 运行确认通过**
  Run: `python test_abi_tjx.py`
  Expected: `test_abi_tjx: ALL PASS`
- [ ] **Step 5: Commit**
  ```bash
  git add TMS320/pc/abi_tjx.py TMS320/pc/test_abi_tjx.py
  git commit -m "feat: abi_tjx parse lib (crc32/bin-end/v2/steps) + tests"
  ```

---

### Task 5: snap_bin.c 移植（记录引擎：环缓冲/回溯/ALIVE/DUMP/CRC）

**Files:**
- Create: `AbiMonitor_TJX\app\snap_bin.h`、`AbiMonitor_TJX\app\snap_bin.c`
- Modify: `AbiMonitor_TJX\empty_driverlib_main.c`（Task 9 集成，本任务只加 include 与最小调用验证）
- 移植蓝本：`D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\firmware\ESP32AbiMonitor\snap_bin.cpp`（逻辑照搬，动态分配→静态数组，printf→cli_printf）

**Interfaces:**
- Consumes: `cli_printf`（Task 7 提供；本任务先以临时 `snap_print_status` 直写 SCI 验证，Task 9 前不依赖完整 cli）
- Produces（Task 6/7/9 与 ISR 依赖的精确签名）：
  - `void snap_init(void)`
  - `void snap_set_steps(int32_t steps)`（恒 4000，留接口）
  - `void snap_arm_begin(void)` / `void snap_arm_end(void)` / `int snap_is_armed(void)`
  - `void snap_on_event(uint64_t t_us, int64_t counts, uint32_t index_n)`（捕获 ISR 调用；armed 且非记录时写环，记录时写主缓冲）
  - `int snap_trigger_from_ring(uint16_t backtrack_n)`（回溯拷主缓冲，置记录态，返回 0/1）
  - `int snap_is_recording(void)`；`uint32_t snap_count(void)`；`uint16_t snap_ring_count(void)`
  - `int snap_poll_done(void)`（主循环轮询：done→打印 `# SNAP DONE`→ALIVE 2 拍→`# SNAP DUMP READY`→置 archive_edge；返回 1 表示 ready 翻转）
  - `int snap_take_archive_edge(void)`（消费 archive_edge）
  - `int snap_dump_ready(void)`
  - `void snap_dump_binary(void)`（帧 + `# BIN END n= hz= steps= mode=event bytes= crc=`）
  - `void snap_dump_hex(void)`（`# HEX BEGIN/END`）
  - `void snap_print_status(void)`（`# SNAP n= ring= hz= ready= rec= arm= alive= bytes= magic=v2`）
  - `uint32_t snap_crc32(const uint8_t*, uint32_t)`
  - `const SnapPoint* snap_data(void)`；`uint32_t snap_data_bytes(void)`
  - `SnapPoint` 结构（16B packed）：见 Global Constraints

- [ ] **Step 1: 写 `snap_bin.h`**

```c
#ifndef APP_SNAP_BIN_H
#define APP_SNAP_BIN_H
#include <stdint.h>
#include <stddef.h>

#define SNAP_CAP       3400u
#define RING_CAP       1200u
#define BACKTRACK_N    400u
#define SNAP_MAGIC_V2  0xAB1C0002u
#define SNAP_HZ_EVENT  0u

#pragma pack(push, 1)
typedef struct {
    uint32_t t_us;
    int64_t  counts;
    uint32_t index_n;
} SnapPoint;
#pragma pack(pop)
typedef char snap_pt_size_ok[(sizeof(SnapPoint) == 16) ? 1 : -1];

void snap_init(void);
void snap_set_steps(int32_t steps);
int32_t snap_steps(void);
void snap_arm_begin(void);
void snap_arm_end(void);
int  snap_is_armed(void);
void snap_on_event(uint64_t t_us, int64_t counts, uint32_t index_n);
int  snap_trigger_from_ring(uint16_t backtrack_n);
int  snap_is_recording(void);
uint32_t snap_count(void);
uint16_t snap_ring_count(void);
int  snap_poll_done(void);
int  snap_take_archive_edge(void);
int  snap_dump_ready(void);
void snap_dump_binary(void);
void snap_dump_hex(void);
void snap_print_status(void);
uint32_t snap_crc32(const uint8_t *data, uint32_t len);
const SnapPoint *snap_data(void);
uint32_t snap_data_bytes(void);
#endif
```

- [ ] **Step 2: 写 `snap_bin.c`（移植 ESP32 逻辑，静态缓冲）**

```c
#include "snap_bin.h"
#include <string.h>
/* 临时输出：Task 7 前用 SCI 直写（复用 lc_printf 亦可，见 Step 3 说明） */

static SnapPoint  g_snap[SNAP_CAP];
static uint32_t   g_snap_n;
static SnapPoint  g_ring[RING_CAP];
static volatile uint16_t g_ring_w, g_ring_n;
static volatile int       g_armed_ring, g_snap_rec, g_snap_done_edge;
static volatile int       g_alive_running, g_dump_ready, g_archive_edge;
static uint8_t    g_alive_i;
static uint32_t   g_alive_next_ms;
static uint64_t   g_rec_base_us;      /* 记录起点绝对时间（回溯起点） */
static uint32_t   g_ms;               /* 1ms tick（Task 6 提供，先由外部注入） */
static int32_t    g_steps = 4000;

void snap_set_ms(uint32_t ms) { g_ms = ms; }   /* Task 6 每 1kHz tick 注入 */

void snap_set_steps(int32_t steps) { g_steps = steps; }
int32_t snap_steps(void) { return g_steps; }

void snap_init(void) { g_snap_n = 0; g_ring_w = 0; g_ring_n = 0;
    g_armed_ring = 0; g_snap_rec = 0; g_snap_done_edge = 0;
    g_alive_running = 0; g_dump_ready = 0; g_archive_edge = 0; }

static void snap_print(const char *s) { cli_printf("%s", s); } /* Task 7 后指向 cli */

void snap_arm_begin(void) { g_armed_ring = 1; g_ring_w = 0; g_ring_n = 0; }
void snap_arm_end(void)   { g_armed_ring = 0; }
int  snap_is_armed(void)  { return g_armed_ring; }

static void store_ring(uint64_t t_us, int64_t counts, uint32_t index_n) {
    SnapPoint p;
    p.t_us = (uint32_t)t_us; p.counts = counts; p.index_n = index_n;
    g_ring[g_ring_w] = p;
    g_ring_w = (uint16_t)((g_ring_w + 1) % RING_CAP);
    if (g_ring_n < RING_CAP) g_ring_n++;
}

void snap_on_event(uint64_t t_us, int64_t counts, uint32_t index_n) {
    if (!g_armed_ring && !g_snap_rec) return;
    if (g_snap_rec) {
        if (g_snap_n < SNAP_CAP) {
            SnapPoint p;
            p.t_us = (uint32_t)(t_us - g_rec_base_us);
            p.counts = counts; p.index_n = index_n;
            g_snap[g_snap_n++] = p;
            if (g_snap_n >= SNAP_CAP) { g_snap_rec = 0; g_snap_done_edge = 1; }
        }
        return;
    }
    store_ring(t_us, counts, index_n);
}

int snap_trigger_from_ring(uint16_t backtrack_n) {
    uint16_t n_bt = (uint16_t)((g_ring_n < backtrack_n) ? g_ring_n : backtrack_n);
    if (n_bt == 0) return 0;
    g_snap_n = 0;
    uint16_t start = (uint16_t)((g_ring_w + RING_CAP - n_bt) % RING_CAP);
    uint64_t t0 = g_ring[start].t_us;
    for (uint16_t i = 0; i < n_bt; i++) {
        SnapPoint p = g_ring[(start + i) % RING_CAP];
        p.t_us = (uint32_t)(p.t_us - t0);
        g_snap[g_snap_n++] = p;
    }
    g_rec_base_us = t0;
    g_snap_rec = 1;
    g_snap_done_edge = 0;
    return 1;
}

int  snap_is_recording(void) { return g_snap_rec; }
uint32_t snap_count(void)    { return g_snap_n; }
uint16_t snap_ring_count(void){ return g_ring_n; }

int snap_poll_done(void) {
    int edge = 0;
    if (g_snap_done_edge) {
        g_snap_done_edge = 0;
        cli_printf("# SNAP DONE n=%lu — RAM full; ALIVE then SD archive\n",
                   (unsigned long)g_snap_n);
        g_dump_ready = 0; g_alive_running = 1; g_alive_i = 0;
        g_alive_next_ms = g_ms + 1000u;
    }
    if (g_alive_running) {
        if ((int32_t)(g_ms - g_alive_next_ms) >= 0) {
            g_alive_i++;
            cli_printf("# ALIVE %u n=%lu\n", (unsigned)g_alive_i,
                       (unsigned long)g_snap_n);
            if (g_alive_i >= 2) {
                g_alive_running = 0;
                g_dump_ready = 1;
                g_archive_edge = 1;
                cli_printf("# SNAP DUMP READY n=%lu — auto SD SAVE; optional DUMP BIN\n",
                           (unsigned long)g_snap_n);
                edge = 1;
            } else {
                g_alive_next_ms = g_ms + 1000u;
            }
        }
    }
    return edge;
}

int snap_take_archive_edge(void) { if (!g_archive_edge) return 0; g_archive_edge = 0; return 1; }
int snap_dump_ready(void) { return g_dump_ready && !g_snap_rec && !g_alive_running; }

uint32_t snap_crc32(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return ~crc;
}

const SnapPoint *snap_data(void)      { return g_snap; }
uint32_t snap_data_bytes(void)        { return g_snap_n * sizeof(SnapPoint); }

void snap_dump_binary(void) {
    if (g_snap_n == 0) { cli_printf("# SNAP empty (no buffer)\n"); return; }
    uint32_t payload = g_snap_n * sizeof(SnapPoint);
    uint32_t crc = snap_crc32((const uint8_t *)g_snap, payload);
    uint16_t n16 = (uint16_t)g_snap_n, hz16 = (uint16_t)SNAP_HZ_EVENT;
    uint32_t magic = SNAP_MAGIC_V2;
    for (int i = 0; i < 10; i++) cli_put_raw_byte(0xAA);
    cli_put_raw_byte(0x55);
    cli_put_raw(&magic, 4);
    cli_put_raw(&n16, 2);
    cli_put_raw(&hz16, 2);
    cli_put_raw((const uint8_t *)g_snap, payload);
    cli_put_raw(&crc, 4);
    cli_printf("\n# BIN END n=%u hz=%u steps=%ld mode=event bytes=%u crc=0x%08lX\n",
               (unsigned)n16, (unsigned)hz16, (long)g_steps,
               (unsigned)(8 + payload + 4), (unsigned long)crc);
}

void snap_dump_hex(void) {
    cli_printf("# HEX BEGIN n=%lu (t_us,counts,index_n) magic=v2\n", (unsigned long)g_snap_n);
    for (uint32_t i = 0; i < g_snap_n; i++)
        cli_printf("# %lu %lu %lld %lu\n", (unsigned long)i,
                   (unsigned long)g_snap[i].t_us, (long long)g_snap[i].counts,
                   (unsigned long)g_snap[i].index_n);
    cli_printf("# HEX END\n");
}

void snap_print_status(void) {
    cli_printf("# SNAP n=%lu ring=%u hz=%lu ready=%d rec=%d arm=%d alive=%d bytes=%lu magic=v2\n",
               (unsigned long)g_snap_n, (unsigned)g_ring_n, (unsigned long)SNAP_HZ_EVENT,
               g_dump_ready, g_snap_rec, g_armed_ring, g_alive_running,
               (unsigned long)(g_snap_n * sizeof(SnapPoint)));
}
```

  - 说明：`cli_printf` / `cli_put_raw` / `cli_put_raw_byte` 由 Task 7 提供；本任务用下列**临时替身**放在文件底部（`#ifndef HAVE_CLI` 由 Task 7 移除）：
    ```c
    #ifndef HAVE_CLI
    #include <stdio.h>
    #include "tjx_init.h"
    #include "driverlib.h"
    static void cli_printf(const char *f, ...) { char b[128]; va_list ap; va_start(ap,f); vsnprintf(b,128,f,ap); va_end(ap); lc_printf(b); }
    static void cli_put_raw(const void *p, uint32_t n) { const uint8_t *b=p; for (uint32_t i=0;i<n;i++) lc_printf("%c", b[i]); }
    static void cli_put_raw_byte(uint8_t c) { lc_printf("%c", c); }
    #endif
    ```
    ⚠ `snap_poll_done`/`snap_dump_binary` 里用到 va_list → 文件加 `#include <stdarg.h>`。
- [ ] **Step 3: 构建验证**
  - `empty_driverlib_main.c` 里 `#include "app/snap_bin.h"`，`main()` 中调 `snap_init(); snap_set_steps(4000);`。
  - 预期：CPU1_RAM 构建 0 错误；.map 中 `SNAP_RAM` 段容纳 g_snap/g_ring（约 73.6KB 数据）且 L1/L2 已使用；`snap_pt_size_ok` 无编译错误（sizeof==16 已验证）。
- [ ] **Step 4: 上板冒烟（串口断言）**
  - 烧录 → 串口发 `SNAP STATUS`（本任务临时在 main 循环里 if 收到字符直接回显 `snap_print_status()`，Task 7 换正式 cli）→ 预期输出 `# SNAP n=0 ring=0 ...`。
- [ ] **Step 5: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/snap_bin.h TMS320/AbiMonitor_TJX/app/snap_bin.c
  git commit -m "feat: snap_bin event recording engine (ring/backtrace/alive/dump/crc32)"
  ```

---

### Task 6: eqep_abi.c（硬件层：捕获 ISR / canonical counts / Index / 档位切换）

**Files:**
- Create: `AbiMonitor_TJX\app\eqep_abi.h`、`AbiMonitor_TJX\app\eqep_abi.c`
- Modify: 无（ISR 注册由 Task 3 SysConfig `registerInterrupts` 生成的 board.c 完成）

**Interfaces:**
- Consumes: Task 3 生成的 `Module_EQEP_*` 宏；`snap_on_event`（Task 5）。
- Produces（Task 7/9 依赖的精确签名）：
  - `void abi_init(void)`（解码器正交 4X、PCM/QDC/IEL 中断；置 gear=4X）
  - `__interrupt void INT_Module_EQEP_ISR(void)`（PCM 逐计数 / QDC 反向 / IEL Index 三源分发）
  - `int64_t abi_counts(void)`（canonical 4000 步/圈基准累计）
  - `uint32_t abi_index_n(void)`
  - `uint64_t abi_now_us(void)`（CPUTIMER0 扩展的 64 位 µs，1kHz ISR 维护）
  - `int  abi_gear(void)`（1=GEAR_4X 逐计数 2=GEAR_1X 每 4 计数）
  - `void abi_set_gear(int gear)`（改 PCM 步长 g_step；置 idx_cal_pending）
  - `uint32_t abi_last_period_us(void)`（最近事件间隔 µs，0=无事件；rpm 由 Task 7 1kHz 换算，ISR 内禁浮点）
  - `uint32_t abi_missed_events(void)`（漏事件计数，联调观察优雅降级；ISR 内 delta>step 时 ++）

- [ ] **Step 1: 写 `eqep_abi.h`**
  ```c
  #ifndef APP_EQEP_ABI_H
  #define APP_EQEP_ABI_H
  #include <stdint.h>
  #define GEAR_4X 1
  #define GEAR_1X 2
  void abi_init(void);
  __interrupt void INT_Module_EQEP_ISR(void);   /* SysConfig 注册；PCM/QDC/IEL 分发 */
  int64_t abi_counts(void);
  uint32_t abi_index_n(void);
  uint64_t abi_now_us(void);
  int  abi_gear(void);
  void abi_set_gear(int gear);
  uint32_t abi_last_period_us(void);

  uint32_t abi_missed_events(void);
  #endif
  ```
- [ ] **Step 2: 写 `eqep_abi.c`（方案 A：PCM 逐计数 ISR，见决策记录）**
  - 核对（实现前必读）：`device\driverlib\eqep.h` 与 `device\driverlib\inc\hw_eqep.h` 中**精确枚举名**（`EQEP_INT_POS_COMP_MATCH`、`EQEP_INT_DIR_CHANGE`、`EQEP_INT_INDEX_EVNT_LATCH`、`EQEP_INT_*` 无 CAPTURE）、`EQEP_enableInterrupt`（L648）、`EQEP_getDirection`（L597，返回 int16_t ±1）、`EQEP_setCompareConfig`（L1875，**内部写 QPOSCMP 且只写 active；PCSHDW/PCLOAD 无封装需裸写 QPOSCTL**）。
  ```c
  #include "eqep_abi.h"
  #include "snap_bin.h"
  #include "driverlib.h"
  #include "board.h"
  #include <limits.h>

  #define SYSCLK_MHZ (DEVICE_SYSCLK_FREQ / 1000000u)   /* 150（F28P55x 主频） */

  static volatile int      g_gear = GEAR_4X;
  static volatile int      g_step = 1;           /* PCM 步长：GEAR_4X→1、GEAR_1X→4 */
  static volatile int64_t  g_counts_canon;       /* 恒 4000 步/圈基准 */
  static volatile uint32_t g_index_n;
  static volatile int      g_idx_cal_pending;
  static volatile uint32_t g_last_period_us;
  static volatile uint64_t g_last_evt_us;        /* 上一事件绝对 µs */
  static volatile uint32_t g_missed_events;      /* 漏事件计数：delta>step 时累加（k 建议） */
  static int32_t raw_prev;

  /* 64 位 µs 时钟：CPUTIMER0 周期=SYSCLK/1000 向下计数，1kHz ISR（Task 7）维护
     g_us64（每 tick +1000）与 g_us_tick_cnt（countdown 快照）；
     亚 µs 精度：t_us = g_us64 + ((g_us_tick_cnt − CPUTimer_getTimerCount + period) % period)/SYSCLK_MHZ
     （u64 撕裂读用 hi/lo/hi retry） */
  extern volatile uint64_t g_us64;
  extern volatile uint32_t g_us_tick_cnt;
  #define TIMER0_PERIOD (DEVICE_SYSCLK_FREQ / 1000u)

  uint64_t abi_now_us(void) {
      uint32_t cnt = CPUTimer_getTimerCount(CPUTIMER0_BASE);
      uint32_t el  = (g_us_tick_cnt + TIMER0_PERIOD - cnt) % TIMER0_PERIOD;
      return g_us64 + (uint64_t)(el / SYSCLK_MHZ);
  }
  void abi_init(void) {
      g_gear = GEAR_4X; g_step = 1; g_counts_canon = 0; g_index_n = 0;
      g_idx_cal_pending = 0; g_last_period_us = 0; g_last_evt_us = 0; raw_prev = 0;
      g_missed_events = 0;
      EQEP_setDecoderConfig(Module_EQEP_BASE,
          EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE);
          /* ⚠ 本器件无 4X/1X 分辨率枚举（XCR 仅 CLOCK_DIR 模式）；正交天然 4X */
      EQEP_setPositionCounterConfig(Module_EQEP_BASE,
          EQEP_POSITION_RESET_MAX_POS, 0xFFFFFFFFu);
      abi_set_gear(GEAR_4X);
      EQEP_enableCompare(Module_EQEP_BASE);                /* QPOSCTL.PCE=1 */
      HWREGH(Module_EQEP_BASE + EQEP_O_QPOSCTL) |=
          (EQEP_QPOSCTL_PCSHDW | EQEP_QPOSCTL_PCLOAD);     /* 影子使能+匹配装载 */
      EQEP_setCompareConfig(Module_EQEP_BASE, EQEP_COMPARE_NO_SYNC_OUT,
          (uint32_t)((int32_t)EQEP_getPosition(Module_EQEP_BASE)
                     + EQEP_getDirection(Module_EQEP_BASE) * g_step), 0u);
      EQEP_enableInterrupt(Module_EQEP_BASE,
          EQEP_INT_POS_COMP_MATCH | EQEP_INT_DIR_CHANGE | EQEP_INT_INDEX_EVNT_LATCH);
      EQEP_clearInterruptStatus(Module_EQEP_BASE,
          EQEP_INT_POS_COMP_MATCH | EQEP_INT_DIR_CHANGE | EQEP_INT_INDEX_EVNT_LATCH);
  }

  void abi_set_gear(int gear) {
      g_step = (gear == GEAR_1X) ? 4 : 1;        /* ISR 速率 /step，canonical 仍精确 */
      g_gear = gear;
      g_idx_cal_pending = 1;
  }

  __interrupt void INT_Module_EQEP_ISR(void) {
      uint32_t st = EQEP_getInterruptStatus(Module_EQEP_BASE);
      if (st & EQEP_INT_POS_COMP_MATCH) {
          int32_t raw = (int32_t)EQEP_getPosition(Module_EQEP_BASE);
          uint64_t now = abi_now_us();           /* k：先读计数+时间戳，再分发，缩小窗口 */
          int32_t delta = raw - raw_prev;
          if (delta >  (int32_t)0x3FFFFFFF) delta -= (int32_t)0x80000000u;
          if (delta < -(int32_t)0x3FFFFFFF) delta += (int32_t)0x80000000u;
          raw_prev = raw;
          if (delta > g_step || delta < -g_step) g_missed_events++;  /* 漏事件计数（k 建议） */
          g_counts_canon += delta;
          if (g_last_evt_us) {
              g_last_period_us = (uint32_t)(now - g_last_evt_us);
              /* 只做整数 µs 差分（ISR 禁浮点/除法，90 周期预算）；rpm 由 1kHz speed_est 算（Task 7） */
          }
          g_last_evt_us = now;
          snap_on_event(now, g_counts_canon, g_index_n);
          EQEP_setCompareConfig(Module_EQEP_BASE, EQEP_COMPARE_NO_SYNC_OUT,
              (uint32_t)(raw + EQEP_getDirection(Module_EQEP_BASE) * g_step), 0u);
      }
      if (st & EQEP_INT_DIR_CHANGE) {          /* 反向：按新方向重新武装 */
          int32_t raw = (int32_t)EQEP_getPosition(Module_EQEP_BASE);
          raw_prev = raw;
          EQEP_setCompareConfig(Module_EQEP_BASE, EQEP_COMPARE_NO_SYNC_OUT,
              (uint32_t)(raw + EQEP_getDirection(Module_EQEP_BASE) * g_step), 0u);
      }
      if (st & EQEP_INT_INDEX_EVNT_LATCH) {
          g_index_n++;
          if (g_idx_cal_pending) { g_idx_cal_pending = 0; g_counts_canon = (int64_t)g_index_n * 4000; }
      }
      EQEP_clearInterruptStatus(Module_EQEP_BASE,
          EQEP_INT_POS_COMP_MATCH | EQEP_INT_DIR_CHANGE | EQEP_INT_INDEX_EVNT_LATCH);
      Interrupt_clearACKGroup(INT_Module_EQEP_INTERRUPT_ACK_GROUP);
  }

  int64_t abi_counts(void)    { return g_counts_canon; }
  uint32_t abi_index_n(void)  { return g_index_n; }
  int  abi_gear(void)         { return g_gear; }
  uint32_t abi_last_period_us(void) { return g_last_period_us; }
  uint32_t abi_missed_events(void)  { return g_missed_events; }   /* 联调观察优雅降级 */
  ```
  - ⚠ `HWREGH` 需要 `#include "inc/hw_eqep.h"`（driverlib.h 已带）；PCSHDW/PCLOAD 无 driverlib 封装（eqep.c L65-76 只写 QPOSCMP active 值 + QPOSCTL 的 PCSHDW/PCLOAD 由 config 传入 `EQEP_setCompareConfig` 内 regValue 写入——实现时确认 `EQEP_setCompareConfig` 的 config 参数是否可带 `EQEP_QPOSCTL_PCSHDW|EQEP_QPOSCTL_PCLOAD`，可则免裸写）。
- [ ] **Step 3: 构建 + 上板冒烟**
  - ISR 已按方案 A 写为 `__interrupt void INT_Module_EQEP_ISR(void)`（Step 2 代码，PCM/QDC/IEL 三源在同一个 INT_EQEP1 内由 QFLG 分发）；注册由 SysConfig `registerInterrupts` 生成的 `board.c` 完成，main 无需再注册。
  - 手转或电驱动（多圈多速度更佳），串口打印 `abi_counts()/abi_index_n()/abi_last_period_us()` 断言：正反转 counts 增减、I 每圈 +1、静止时周期保持最后值。
- [ ] **Step 4: ISR 开销实测（强制检查点——评审 G：Task 6 完成后立即做，不拖到联调）**
  - 方法：CPUTIMER0 读 `CPUTimer_getTimerCount` 差值（或 GPIO 翻转示波器法）测 `INT_Module_EQEP_ISR` 周期数；结果记入 spec §12.7 与联调文档。
  - 判定：>0.6µs（≈90 周期 @150MHz）→ **立即启用降档**（GEAR_1X：PCM 步长 N=4，ISR 速率 /4、canonical 仍精确，eqep_abi/speed_est 约 6 行），不等到联调。
- [ ] **Step 5: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/eqep_abi.h TMS320/AbiMonitor_TJX/app/eqep_abi.c
  git commit -m "feat: eqep PCM per-count ISR (POS_COMP_MATCH/QDC/IEL), canonical counts, gear step"
  ```

---

### Task 7: speed_est.c（1kHz 速度估计 + 外推 + 分档判定）

**Files:**
- Create: `AbiMonitor_TJX\app\speed_est.h`、`AbiMonitor_TJX\app\speed_est.c`
- Modify: `snap_bin.c`（Task 9 接 `snap_set_ms`；本任务先由 main 调用）

**Interfaces:**
- Consumes: `abi_last_period_us/abi_gear/abi_set_gear`（Task 6）、`snap_set_ms`（Task 5）、`abi_now_us`。
- Produces（Task 9 依赖）：
  - `void spd_init(void)`
  - `void spd_tick_1khz(void)`（CPUTIMER0 ISR 调：测速/外推/分档切换；内部调 `snap_set_ms(g_ms++)`）
  - `float spd_rpm(void)`（当前估计 RPM，供遥测与触发判定）
  - `int  spd_gear(void)`
  - `uint32_t spd_abs_ms(void)`（毫秒时钟，供 ALIVE/超时）

- [ ] **Step 1: 写 `speed_est.h`**
  ```c
  #ifndef APP_SPEED_EST_H
  #define APP_SPEED_EST_H
  #include <stdint.h>
  void spd_init(void);
  void spd_tick_1khz(void);
  float spd_rpm(void);
  int   spd_gear(void);
  uint32_t spd_abs_ms(void);
  #endif
  ```
- [ ] **Step 2: 写 `speed_est.c`**
  ```c
  #include "speed_est.h"
  #include "eqep_abi.h"
  #include "snap_bin.h"
  #include <math.h>

  #define STOP_US       100000u   /* 100ms 无事件 → 0 */
  #define GEAR_HI_RPM   6500.0f
  #define GEAR_LO_RPM   5500.0f
  #define MAX_RPM       20000.0f

  static volatile float  g_rpm;
  static volatile uint32_t g_ms;
  static volatile int    g_gear = GEAR_4X;
  static uint64_t last_t_us;
  static float    last_rpm1, last_rpm2;   /* 最近两次实测 */
  static uint64_t last_t1_us, last_t2_us;
  static int      have_1, have_2;

  /* 64 位 µs 时钟（eqep_abi extern g_us64）：
     CPUTIMER0 周期=SYSCLK/1000 向下计数；1kHz ISR 维护：
     g_us64 每 tick +1000；g_us_tick_cnt=ISR 开头读到的 countdown 快照（≈period−ISR 延迟）。
     abi_now_us() = g_us64 + ((g_us_tick_cnt − CPUTimer_getTimerCount + period) % period)/SYSCLK_MHZ */
  static volatile uint64_t g_us64;
  static volatile uint32_t g_us_tick_cnt;

  void spd_init(void) { g_rpm = 0; g_ms = 0; have_1 = have_2 = 0; last_t_us = 0;
      g_us64 = 0; g_us_tick_cnt = 0; }

  uint32_t spd_abs_ms(void) { return g_ms; }

  void spd_tick_1khz(void) {
      g_ms++;
      snap_set_ms(g_ms);
      g_us64 += 1000u;                                   /* 1kHz 基准累加 */
      g_us_tick_cnt = CPUTimer_getTimerCount(CPUTIMER0_BASE);   /* 亚 ms 插值快照 */
      uint64_t now = abi_now_us();
      uint32_t per = abi_last_period_us();
      if (per > 0) {                       /* 最近 1ms 有（或刚有过）新事件 */
          /* rpm 在此计算（1kHz，非 ISR）：GEAR_1X 时 per 覆盖 step=4 个计数 */
          float rpm = 60.0f * 1000000.0f /
              ((float)per * (float)(g_gear == GEAR_4X ? 4000 : 1000));
          /* 事件新近性：用上次事件时刻估算；若 per 为陈旧值则走外推 */
          if (now - last_t_us > 3000u) {   /* 3ms 无事件 → 外推 */
              goto extrap;
          }
          last_t2_us = last_t1_us; last_rpm2 = last_rpm1;
          last_t1_us = now;        last_rpm1 = rpm;
          have_2 = have_1; have_1 = 1;
          g_rpm = rpm;
          last_t_us = now;
      } else {
      extrap:
          if (!have_1 || (now - last_t1_us > STOP_US)) { g_rpm = 0; goto gear; }
          if (have_2 && (now - last_t2_us) > 0 && last_t1_us > last_t2_us) {
              float slope = (last_rpm1 - last_rpm2) /
                            (float)(last_t1_us - last_t2_us);
              float est = last_rpm1 + slope * (float)(now - last_t1_us);
              g_rpm = (est < 0) ? 0 : ((est > MAX_RPM) ? MAX_RPM : est);
          } else {
              g_rpm = last_rpm1;           /* 单点保持 */
          }
      }
  gear:
      if      (g_rpm > GEAR_HI_RPM && g_gear == GEAR_4X) { g_gear = GEAR_1X; abi_set_gear(GEAR_1X); }
      else if (g_rpm < GEAR_LO_RPM && g_gear == GEAR_1X) { g_gear = GEAR_4X; abi_set_gear(GEAR_4X); }
  }

  float spd_rpm(void) { return g_rpm; }
  int   spd_gear(void){ return g_gear; }
  ```
  - ⚠ 外推的"新事件检测"依赖 `abi_last_period_us()` 新近性，语义上简化为"上一次 ISR 距今"；若实测抖动明显，改为在 eqep_abi 暴露 `abi_event_count()`（捕获 ISR 里 ++），`speed_est` 比较计数变化，实现时二选一（优先计数法，改动 3 行）。
- [ ] **Step 3: 构建 + 上板冒烟**
  - main 临时：CPUTIMER0 ISR 调 `spd_tick_1khz()`；串口 1Hz 打印 `spd_rpm()`。
  - 断言：低速（手转或电驱动）RPM 数值合理（≈实际）；停转 100ms 后归 0；高速档（>6500）切换时 `spd_gear()` 变 1X（优先用电机驱动多速度验证；无电机时暂以手摇方式验证切换阈值，任务 11 再用电机复核）。
- [ ] **Step 4: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/speed_est.h TMS320/AbiMonitor_TJX/app/speed_est.c
  git commit -m "feat: 1kHz speed estimate + extrapolation + gear switch"
  ```

---

### Task 8: cli.c（命令表 / 遥测 / 静默 / 原始字节输出）

**Files:**
- Create: `AbiMonitor_TJX\app\cli.h`、`AbiMonitor_TJX\app\cli.c`
- Modify: `snap_bin.c`（移除 Task 5 临时替身 `#ifndef HAVE_CLI` 段）

**Interfaces:**
- Consumes: snap_bin API（Task 5）、eqep_abi API（Task 6）、speed_est API（Task 7）、SCIA 宏（Task 3）。
- Produces（Task 9/10 依赖）：
  - `void cli_init(void)`
  - `void cli_poll(void)`（主循环调：收字符组行，命中指令分发；`#` 标记行输出）
  - `void cli_printf(const char *fmt, ...)`（`# ` 前缀自动加在首行标记、遥测行不加；本任务实现为：透传格式串，调用方自行含 `# `）
  - `void cli_put_raw(const void *p, uint32_t n)`（DUMP BIN 二进制体，直写 SCIA，不掺换行）
  - `void cli_put_raw_byte(uint8_t c)`
  - `void cli_set_muted(int m)` / `int cli_muted(void)`（记录期间静默遥测）
  - `void cli_telem_tick(void)`（1Hz/10Hz 遥测调度与 `L,` 行生成）

- [ ] **Step 1: 写 `cli.h`**
  ```c
  #ifndef APP_CLI_H
  #define APP_CLI_H
  #include <stdint.h>
  void cli_init(void);
  void cli_poll(void);
  void cli_printf(const char *fmt, ...);
  void cli_put_raw(const void *p, uint32_t n);
  void cli_put_raw_byte(uint8_t c);
  void cli_set_muted(int m);
  int  cli_muted(void);
  void cli_telem_tick(void);
  #endif
  ```
- [ ] **Step 2: 写 `cli.c`（命令表与遥测）**
  - 指令表（与 ESP32 v40 一致的子集）：`MONITOR START|ON|1`、`MONITOR STOP|OFF|0`、`REC MS <ms>`（500–3000 取整 100）、`SNAP?`、`SNAP STATUS`、`DUMP BIN`、`HEX DUMP`、`FW?`、`TIME <unix_ms>`、`PING`、`HELP`、`ABI?`。
  - 应答文本照 spec §5/Global Constraints（`# MONITOR armed: ...`、`# SNAP DONE n=...` 等；`→`/`—` 字符按 ESP32 原文保留）。
  - 遥测 `L,` 行 20 字段（对齐 ESP32）：`L,%lu,%s%d.%d,%d,%d,%u,%lu,%lu,%lld,%u,%u,%u,%d.%d,%lu,%u,%s%d.%d,%d.%d,%lu,%ld,%lu,%llu` 即 `L,t_ms,rpm,dir,armed,log_n,drop,hz,counts,seg,phase,pool_n,revs,remain,segs,revs_total,revs_abs,index_n,index_signed,t_rel,unix_ms`（Hz 字段填实测估算：任务 6 事件率或 0）。
  - 解析：读字符 → 缓冲 128B 行 → 回车触发 `handle_cmd`；未知 → `# ERR unknown: %s`。
  - `cli_put_raw`：关全局中断写 SCIA（用 `SCI_writeCharArray` 轮询循环），防遥测穿插。
- [ ] **Step 3: 移除 snap_bin.c 临时替身**
  - 删除 `#ifndef HAVE_CLI ... #endif` 段；`snap_bin.c` 顶部 `#include "app/cli.h"`。
- [ ] **Step 4: 构建 + 上板冒烟**
  - 断言：`PING`→`# PONG`；`FW?`→`# FW=abi_tjx-v1-eventsnap ...`；`HELP` 输出命令表；`TIME 1780123456789`→`# TIME ok`；`SNAP?`→`# SNAP src=RAM valid=0 n=0 ...`。
- [ ] **Step 5: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/cli.h TMS320/AbiMonitor_TJX/app/cli.c TMS320/AbiMonitor_TJX/app/snap_bin.c
  git commit -m "feat: CLI command table, telemetry L-line, raw byte output"
  ```

---

### Task 9: sd_fatfs.c（FatFS + SPIB diskio + 存档 + SD DUMP）

**Files:**
- Create: `AbiMonitor_TJX\app\sd_fatfs.h`、`AbiMonitor_TJX\app\sd_fatfs.c`、`AbiMonitor_TJX\app\fatfs\ff.h/ff.c/ffconf.h/ffsystem.c/diskio.h/diskio.c/ffunicode.c(如需要)`
- Download: FatFS R0.15：`https://elm-chan.org/fsw/ff/arc/ff15.zip`（解压 ff15/source/ 下文件入 `app\fatfs\`）

**Interfaces:**
- Consumes: Task 3 SPIB 宏（`SD_SPI_BASE`、CS GPIO）、`snap_data/snap_data_bytes/snap_steps/snap_count`（Task 5）、TIME 状态（Task 8 cli 存 unix_ms）。
- Produces（Task 10 与主循环依赖）：
  - `int sd_init(void)`（挂载；返回 0=ok）
  - `int sd_save_snap(void)`（`/snap_YYYYMMDD_HHMMSS_n.bin` 或 `b<millis>` + `.txt` 元数据；返回 0=ok，`# SD ERR <code>`）
  - `int sd_dump_file(const char *name)`（`SD DUMP <file>` 回传）
  - `void sd_poll(void)`（主循环：检测 archive_edge → 停采样写卡 → 恢复）

- [ ] **Step 1: 下载并放入 FatFS 源文件**
  - 解压 `ff15.zip` → `AbiMonitor_TJX\app\fatfs\`（ff.c/ff.h/ffconf.h/ffsystem.c/diskio.c/diskio.h）；**不需要** ffunicode.c（FF_LFN_UNICODE=0）。
- [ ] **Step 2: 配置 ffconf.h**
  ```c
  #define FF_FS_READONLY 0
  #define FF_FS_MINIMIZE 0
  #define FF_USE_STRFUNC 0
  #define FF_USE_MKFS    0
  #define FF_USE_FASTSEEK 0
  #define FF_USE_LFN     1
  #define FF_LFN_UNICODE 0
  #define FF_VOLUMES     1
  #define FF_MIN_SS      512
  #define FF_MAX_SS      512
  #define FF_FS_TINY     0
  #define FF_FS_EXFAT    0
  #define FF_FS_LOCK     0
  #define FF_USE_CHMOD   0
  ```
- [ ] **Step 3: 写 diskio 适配层（SPIB 轮询读写扇区）**
  - `disk_initialize/disk_status/disk_read/disk_write/disk_ioctl`（CTRL_SYNC 用等 SPI 空闲；GET_SECTOR_COUNT/SIZE 用 CSD 解析或 `disk_ioctl(GET_SECTOR_COUNT)`）。
  - 实现要点：SPIB `SPI_writeDataNonBlocking`/`SPI_readDataBlockingNonFIFO`（08_spi_example 模式）；8-bit 数据；CS 由 GPIO 控制；SPI 时钟 400kHz（初始化）→ 25MHz（读写）。
- [ ] **Step 4: 写 `sd_fatfs.c`**
  - `sd_init`：`f_mount`；`SD?` 应答 `# SD ready=1 type=... size_MB=... free_MB=...`。
  - `sd_save_snap`：文件名 `snap_%s_%u.bin`（`%s`=TIME 同步后 `YYYYMMDD_HHMMSS`，否则 `b%lu`(ms)）；`.txt` 元数据（stamp/iso_local/unix_ms/synced/n/bytes/crc/hz/steps/mode/board_ms/file，`hz=0`、`steps=4000`、`mode=event`）。
  - `SD DUMP <file>`：`f_open` → 分块（如 512B）`cli_put_raw` → `# SD DUMP END file= n= crc=`。
  - `sd_poll`：`snap_take_archive_edge()` 为真 → 停采样（`spd` 1kHz 照跑但 snap 静默，SCIA 打 `# SD SAVE ...` 前先 mute 遥测）→ `sd_save_snap` → 重试 ≤3 → `# SD ERR <code>` 或 `# SD SAVE OK <path> n= bytes= crc= time= synced=`。
- [ ] **Step 5: 构建 + 上板冒烟**
  - 断言：插卡 → `SD INIT` → `# SD OK type=...`；`SD TEST` 写读 `/oez_sd_test.txt` 往返一致；无卡时 `SD?` → `# SD ready=0 ...`。
- [ ] **Step 6: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/app/sd_fatfs.h TMS320/AbiMonitor_TJX/app/sd_fatfs.c TMS320/AbiMonitor_TJX/app/fatfs
  git commit -m "feat: FatFS on SPIB + snap archive + SD DUMP"
  ```

---

### Task 10: main.c 状态机集成（武装→STAGING→RECORD→ALIVE→SD→READY）

**Files:**
- Modify: `AbiMonitor_TJX\empty_driverlib_main.c`（全部替换为集成实现）
- 蓝本：`ESP32AbiMonitor.ino:637-676` 触发逻辑（STAGING 确认 OR 条件）、`loop()` 遥测调度

**Interfaces:**
- Consumes: 全部上文 API（snap_bin/eqep_abi/speed_est/cli/sd_fatfs）。
- Produces: 完整固件（Task 11 联调对象）。

- [ ] **Step 1: 状态机与主循环**
  ```c
  enum { PH_IDLE = 0, PH_STAGING = 1, PH_RECORD = 2 };
  static volatile int g_armed, g_phase;
  static uint32_t g_stage_start_ms, g_stage_index0, g_rec_duration_ms = 1000;

  void main(void) {
      Device_init(); Device_initGPIO(); Interrupt_initModule();
      Interrupt_initVectorTable(); Board_init(); C2000Ware_libraries_init();
      EINT; ERTM;
      cli_init(); snap_init(); snap_set_steps(4000); spd_init();
      abi_init();
      lc_printf("# AbiMonitor_TJX FW=abi_tjx-v1-eventsnap\n");
      lc_printf("# SNAP RAM buffer %ux16B ~%luKB OK\n",
                (unsigned)SNAP_CAP, (unsigned long)(SNAP_CAP*16/1024));
      while (1) {
          cli_poll();
          monitor_state_machine();      /* Step 2 */
          snap_poll_done();
          sd_poll();                    /* Task 9 */
          if (snap_take_archive_edge()) { /* 经 sd_poll 处理，此处兜底清零 */ }
          telem_schedule();             /* Step 3 */
      }
  }
  ```
- [ ] **Step 2: 触发逻辑（照搬 ESP32 语义）**
  ```c
  static void monitor_state_machine(void) {
      float rpm = spd_rpm();
      int arpm = (rpm > 0) ? (int)rpm : (int)(-rpm);
      uint32_t now_ms = spd_abs_ms();
      if (!g_armed) return;
      if (g_phase == PH_IDLE) {
          if (arpm > 10) {
              cli_set_muted(1);
              g_phase = PH_STAGING;
              g_stage_start_ms = now_ms;
              g_stage_index0 = abi_index_n();
              cli_printf("# STAGING |rpm|>10 waiting I+1rev ring=%u\n",
                         (unsigned)snap_ring_count());
          }
      } else if (g_phase == PH_STAGING) {
          int rpm_drop   = (arpm < 5);
          int short_hold = (now_ms - g_stage_start_ms) >= 200u;
          int i_plus     = ((int32_t)(abi_index_n() - g_stage_index0) >= 1);
          int ring_ok    = (snap_ring_count() >= 120);
          int drop_salvage = rpm_drop && (snap_ring_count() >= 40);
          if (i_plus || short_hold || ring_ok || drop_salvage) {
              if (snap_trigger_from_ring(BACKTRACK_N)) {
                  g_phase = PH_RECORD;
                  cli_printf("# CONFIRM |rpm|>10 & I+1rev seg=1 snap_n=%lu → +REC %lums (backtrack≤%u)\n",
                             (unsigned long)snap_count(), (unsigned long)g_rec_duration_ms,
                             (unsigned)BACKTRACK_N);
              }
          }
      } else if (g_phase == PH_RECORD) {
          if (!snap_is_recording()) {   /* snap_poll_done 已打印 SNAP DONE/ALIVE */
              g_phase = PH_IDLE;
              g_armed = 0;
              snap_arm_end();
              cli_set_muted(0);
          }
      }
  }
  ```
  - 武装：`MONITOR START` → `snap_arm_begin(); g_armed=1;` + 打印 `# MONITOR armed: ...`；`MONITOR STOP` → 清理。`REC MS <ms>` → `g_rec_duration_ms`。
  - 记录时长按**事件数**由 SNAP_CAP 截断（`snap_on_event` 满则 done）；`REC MS` 作为超时上限：在 `snap_poll_done` 前由 main 检查（实现：记录模式下若 `spd_abs_ms()-g_rec_start_ms > g_rec_duration_ms` → 置 `g_snap_rec=0; g_snap_done_edge=1`，需给 snap_bin 加 `snap_force_done(void)`——**本任务在 snap_bin.h/.c 增加 `void snap_force_done(void);`**，实现 3 行：置位 done 边沿）。
- [ ] **Step 3: 遥测调度**
  - `telem_schedule()`：非武装且非记录且非静默 → 每 100ms 一行 `L,`（`cli_telem_tick`）；记录/STAGING 中 1Hz 或静默（`cli_muted` 控制）。
  - `L,` 行字段按 Task 8 Step 2；`counts` 用 `abi_counts()`，`index_n` 用 `abi_index_n()`，`rpm` 用 `spd_rpm()`。
- [ ] **Step 4: LED 状态**
  - 武装=蓝（GPIO20=0，GPIO21=1）；记录=绿（GPIO20=1，GPIO21=0）；写 SD=绿闪 200ms 周期；就绪=紫（20=0，21=0 即双开）。放主循环节拍内切换。
- [ ] **Step 5: 构建 CPU1_RAM + CPU1_FLASH 双配置**
  - 预期：两配置均 0 错误；FLASH 配置 map 中 SNAP_RAM 在 RAM 运行段。
- [ ] **Step 6: 上板冒烟（无 SD 卡时也应工作）**
  - 断言：boot banner → `MONITOR START` → armed 行；转动（电驱动多速或手转）→ CONFIRM → RECORD → `# SNAP DONE n≈几百` → ALIVE 2 拍 → `# SNAP DUMP READY` → PC `DUMP BIN` 拉帧成功。
- [ ] **Step 7: Commit**
  ```bash
  git add TMS320/AbiMonitor_TJX/empty_driverlib_main.c TMS320/AbiMonitor_TJX/app/snap_bin.h TMS320/AbiMonitor_TJX/app/snap_bin.c
  git commit -m "feat: main state machine (arm/staging/record/alive), LED, telemetry"
  ```

---

### Task 11: abi_tjx.py 完整 PC 工具（串口 + 自动拉帧 + 存盘 + 出图）

**Files:**
- Modify: `D:\oezcon\TMS320\pc\abi_tjx.py`（Task 4 库基础上加串口/GUI）
- Modify: `D:\oezcon\TMS320\pc\test_abi_tjx.py`（补帧组装器与 BIN END 流解析测试）

**Interfaces:**
- Consumes: Task 4 解析库；固件协议（Task 8/10）。
- Produces: `python abi_tjx.py [--port COMx]` 可运行 GUI；`--selftest` 跑解析测试。

- [ ] **Step 1: 补测试（帧流解析 + BIN END 驱动 steps）**
  - 新增断言：`assembly_stream_test()`——模拟固件字节流（preamble+frame+`\n# BIN END ...\n`），`find_frame(buf)` 提取 raw → `parse_snap_bindump(raw, steps=4000)` 通过；BIN END steps=1000 时 rpm 数值 ×4（`rpm_from_counts_series(..., steps=1000)` 验证）。
  - Run: `python test_abi_tjx.py` → ALL PASS。
- [ ] **Step 2: 串口层 `SerialLink`**
  - `pyserial`：921600 8N1；reader 线程积累字节；行事件回调 + 二进制帧重组（preamble 搜索 → magic/n → 按 n 收齐 payload+crc → 抛 `("bindump", raw)` 事件）。
  - `send(cmd)` 写 `cmd+"\n"`；`arm_bindump(timeout_s=15)` 标记等待帧。
- [ ] **Step 3: 自动流程（响应 `# SNAP DONE` → ALIVE → `# SNAP DUMP READY` → 发 `DUMP BIN` → 收到帧 → 校验 → 存盘 → 提示出图）**
  - 存盘名：`snap_<YYYYmmdd_HHMMSS>_<n>.bin`（PC 本地时间）；同时写 `<name>.csv`（t_ms,rpm,dir,index_n,counts）。
  - `steps` 取 BIN END 行的 `steps=`（默认 4000）；`mode=event` 时 hz=0 显示"事件模式"。
  - **超时机制（dp 建议）**：等待 SNAP DONE/ALIVE/DUMP READY 各超时 5s → 主动发 `SNAP STATUS` 查询 → 仍无响应报错退出，避免假死。
- [ ] **Step 4: tkinter GUI（精简）**
  - 布局：端口下拉 + 连接/断开；按钮：`TIME+START`（发 TIME 再 MONITOR START）、`STOP`、`手动拉取 BIN`、`打开曲线`；状态行（armed/RECORD/READY）；遥测日志框（只显示 `L,` 解析出的 rpm/dir/armed + 标记行）。
  - 出图：matplotlib 子图 1：rpm vs t_ms（非均匀 t_us 时间轴，散点+线）；子图 2：counts vs t_ms；窗口标题含 steps/mode/点数。
  - 依赖缺失：`pip install pyserial matplotlib`（Step 0 前提检查）。
- [ ] **Step 5: 与固件联调（板在手）**
  - 启动 GUI → 连接 → TIME+START → 转动（电驱动多速优先）→ 自动出现 BIN OK → 存盘 → 打开曲线验证非均匀时间轴与高速细节。
- [ ] **Step 6: Commit**
  ```bash
  git add TMS320/pc/abi_tjx.py TMS320/pc/test_abi_tjx.py
  git commit -m "feat: abi_tjx PC tool (serial/auto-pull/save/plot) + frame tests"
  ```

---

### Task 12: 硬件联调 + 回归（spec §12 测试计划 1-12）

**Files:**
- Modify: 按发现修正 `AbiMonitor_TJX\app\*` 与 `TMS320\pc\*`
- Create: `TMS320\docs\联调记录\2026-08-01-abimonitor-bringup.md`（联调日志：每项断言结果、波形截图、遗留问题）

**Interfaces:**
- Consumes: Task 10/11 产物。

- [ ] **Step 1: 低速自检（校准 steps）**
  - `ABI?`：电驱动多圈多速度（或手转 N 圈）→ canonical counts=4000×N（4X 档）；验证 AS5047P 1000 PPR 假设（不符则调整 steps 常量并记录）。
- [ ] **Step 2: 实时遥测**：10Hz `L,` 行 rpm 与实际相符、方向正确。
- [ ] **Step 3: 事件记录**：MONITOR START → 弹射 → SNAP DONE → ALIVE → DUMP READY → PC 自动拉帧 → 出图（非均匀时间轴、无 2kHz 台阶）。
- [ ] **Step 4: 分档切换**：高速源（或手摇极限）验证 >6500 切 1X、<5500 回 4X；canonical counts 无阶跃；I 对齐生效。
- [ ] **Step 5: ISR 开销实测（满载复核）**：Task 6 门已测周期数；此处 4X 满速实际运行确认 CPU 占用 <30% 预算；超则按 Task 6 结论启用降档（GEAR_1X：PCM 步长 N=4，ISR 速率 /4，canonical 仍精确）。
- [ ] **Step 6: SD 存档**：插入卡 → 记录完自动存 → `SD DUMP` 拉回 → PC 校验 CRC。
- [ ] **Step 7: 外推**：低速缓转 → 无事件时段输出线性外推；停止 100ms 归 0。
- [ ] **Step 8: 极限场景**（G 建议）：长时间空闲后突然弹射（环缓冲无污染）；记录中途档位切换（counts 连续）；写卡失败重试路径（拔卡模拟）。
- [ ] **Step 9: 融合过渡交叉验证**（用户要求，spec §12.9）：弹射/扑翼场景各 ≥3 次，PC 双曲线（QCPRD 间隔 vs counts 差分）叠加 + `# GEAR` 切换点标注，无毛刺无跳变 → 结案记录；有毛刺则定位修正重测。
- [ ] **Step 10: 档位收益评估**（评审 G：提前到联调而非 backlog）：实测确认 G2/G3 实际收益（设备是否真会到档）；无收益则记录结论并收敛档位表。
- [ ] **Step 11: 波特率误差测量**（评审 G/dp）：PC 发 1KB 定长包，板端环回，测收发时间差计算实际波特率与标称 921600 偏差；>±2% → 固件与 PC 同步改 460800 并重测。
- [ ] **Step 12: CPU1_FLASH 回归**：烧 FLASH 全量重跑 Step 2-11。
- [ ] **Step 13: Commit**
  ```bash
  git add TMS320/docs/联调记录/2026-08-01-abimonitor-bringup.md TMS320/AbiMonitor_TJX/app TMS320/pc
  git commit -m "test: bring-up pass — event speed monitor (see bringup log)"
  ```

---

### Task 13: 收尾（文档 + 阶段2 backlog 落实）

**Files:**
- Modify: `TMS320\docs\superpowers\specs\2026-08-01-abimonitor-eventspeed-design.md`（把实测值回填：L1/L2 地址、SPIB 引脚、SYSCLK、ISR 周期数、实际事件率）
- Create: `TMS320\项目文档.md` 已存在（2026-08-01 建档，审查与接手入口）——本任务只做**回填与修订**（实测值、联调结果、变更记录）

**Interfaces:**
- Consumes: 全部任务结果。

- [ ] **Step 1: 回填 spec**：§6 内存表（实际段地址）、§9 引脚表（SPIB/CS 最终值）、§12 实测结果、§13 风险实际处置。
- [ ] **Step 2: 写 README**：目录结构、CPU1_RAM/CPU1_FLASH 构建入口、XDS110 烧录步骤、`python abi_tjx.py --port COMx` 用法、`python test_abi_tjx.py` 测试说明。
- [ ] **Step 3: 确认阶段2 backlog 已记录**（spec §14：完整 PC GUI、400-600Hz 闭环、蓝牙、MCI 扩展）。
- [ ] **Step 4: Commit**
  ```bash
  git add TMS320/docs TMS320/README.md
  git commit -m "docs: spec finalization + README + phase-2 backlog"
  ```

---

## Self-Review（写作时已核对）

1. **Spec 覆盖**：§2 事件架构→Task 6；§3 分档 canonical→Task 6/7；§4 记录流程→Task 5/10；§5 协议→Task 8/11；§6 内存→Task 3/5；§7 SD→Task 9；§8 外推→Task 7；§9 引脚→Task 3；§10 环境→Task 1/2；§11 模块→Task 4-10；§12 测试→Task 12；§13 风险→各任务兜底；§14 backlog→Task 13。PC 端"自建不碰 ESP32"→Task 4/11（全部新文件）。
2. **占位扫描**：无 TBD/TODO；任务 6 枚举名"以头文件为准"为明确执行指令而非占位（设备头文件名无法在计划时虚构）。
3. **类型一致**：`snap_on_event(uint64_t,int64_t,uint32_t)`、`abi_counts()→int64_t`、`spd_rpm()→float`、`parse_snap_bindump(raw,steps=4000)→(n,hz,rows)` 在全部任务中签名一致；Task 10 新增 `snap_force_done` 已标注加入位置；`cli_printf/cli_put_raw/cli_put_raw_byte` 三函数在 Task 5 临时替身、Task 8 正式实现间签名一致。
4. **注意点**：Task 6 的 ISR 名称修正（INT_Module_EQEP_ISR 合并分发）已内嵌在步骤中；波特率/内存/LFN 均有 Global Constraints 兜底。
