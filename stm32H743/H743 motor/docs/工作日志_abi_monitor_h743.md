# abi_monitor_h743 工作日志（STM32H743 ABI 编码器事件流监控/记录器）

> 目的：按日期沉淀本项目的初始计划、过程发现、决策与依据，作为 STM32H7 项目经验基础，并作为后续接收人员的交接文档。
> 项目：`D:\oezcon\stm32H743\H743 motor\firmware\abi_monitor_h743\`（固件）
> 计划：`docs\superpowers\plans\2026-08-06-stm32h743-abimotor-plan.md`（主计划，全部 Task 已勾选）
> 接线：`docs\接线文档.md`
> 参考源：`2.参考例程\1.基础例程\1.LED闪烁`（Keil 模板）、`SDMMC-SD卡移植FatFSC`（SD 例程）
> 本文档最后重写：2026-08-07（统一为 UTF-8，补全交接信息）

---

## 项目概述

在 FK743M4-XIH6-V1.1（STM32H743XIH6）开发板上复刻 DSP 的 UTO 方案：AS5047P ABI 编码器
（1000PPR → 4X=4000 计数/圈）事件流监控/记录器。BIN v2 16B 帧与原有 PC 工具字节级兼容
（magic 0xAB1C0002）。自动触发记录 |RPM|>10 的转速变化（回溯 400 点 + 记录 500ms），完成后
**先存芯片内部 Flash（掉电不丢），再备份 FAT32 SD 卡（时间命名、不覆盖）**。

- 设备：STM32H743XIH6（480MHz CM7，2MB Flash，512KB AXI SRAM）
- 外设分配：
  - 编码器 A=PA5(TIM2_CH1)、B=PA1(TIM2_CH2)、Index=PA4(EXTI4)
  - USART1=PA9/PA10 @ 921600 8N1；LED=PC13（低电平点亮）
  - PWM=PA7(TIM3_CH2) 500Hz，占空比 0..1000‰
  - SDMMC1=PC8..12 + PD2（4bit + CMD）
- 时钟链：HSE25 → SYSCLK480 → HCLK240 → APB1/2=120 → TIM5=240MHz（PSC=3 → 60MHz 刻度）
- 记录规格：ring 1024 点、SNAP_CAP 31000 点、回溯 400 点、0.5s 窗口（计划 0.8s，实施调为 0.5s，见"差异"）
- 抽稀档位：3/4/5 档可配，默认 3 档 div 1/2/4 @ 4000/8000rpm

---

## 工具链 / 文件位置（后续接收人员必读）

本机**无 Keil UV4**，编译/烧录全部走 GNU 工具链：

| 项 | 位置 | 说明 |
|----|------|------|
| GCC 编译器 | `D:\arm-official\arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi\bin\` | 13.2.1 完整版（含 cc1.exe） |
| （不可用） | `D:\arm-gcc\xpack-arm-none-eabi-gcc-13.2.1-1.1` | xpack 版缺 cc1.exe，弃用 |
| 构建驱动 | `mingw32-make`（`d:\TDM-GCC-64\bin\`） | 用工程根 `Makefile` |
| 烧录 A（主用） | OpenOCD xpack `D:\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe` | Makefile `flash` 目标：stlink + reset run |
| 烧录 B（备选） | `STM32_Programmer_CLI.exe`（STM32Cube v2.15） | SWD HOTPLUG |

### 关键命令（固件根目录 = `firmware/abi_monitor_h743`）
```bat
mingw32-make -j8          :: 编译（-Og，0 Error）
mingw32-make flash        :: OpenOCD 烧录 + 校验 + 软复位
```

### 工程新增目录（GNU 工具链，Keil 工程之外）
- `gcc\startup_stm32h743xx.s`：GNU as 启动文件（向量表 166 项与 Keil 一致）
- `gcc\stm32h743xx_flash.ld`：链接脚本（FLASH 2MB@0x08000000，RAM 512K@0x24000000）
- `Makefile`：源清单与 uvprojx 一致（App + Core + Drivers/User + HAL + FatFS）
- `FatFs/`：FatFS R0.13（ff.c/h、ffconf.h LFN=0、diskio.c 对齐中转）

### 源码布局
```
firmware/abi_monitor_h743/
  Core/Src/main.c           主循环：自动保存 DONE 上升沿触
  App/
    abi.c/h                 TIM2 4X + EXTI4 Index + TIM5 UTO 事件流 + 测速 + EMA 校准
    snap_bin.c/h            ring 缓存 + 触发状态机 + BIN v2 打包 + crc32
    sd_card.c/h             SDMMC1 + FatFS 自动备份
    flash_save.c/h          Bank2 扇区0..3 记录保存 + DUMP PC 帧
    app_config.c/h          档位表 + steps_per_rev + pol 持久化（CRC16 + Flash）
    app_cli.c/h             CLI 命令
    app_pwm.c/h             PA7/TIM3_PWM
Drivers/User/Src/           usart.c（USART1 IRQ + Usart_Write）led.c
pc_tool/                    PC 侧验证/自学习脚本（见下）
pc/abi_monitor.py          PC UI（拷贝自 mcoder，未改）
docs/                       plans/ 计划、工作日志、接线文档
```

### PC 工具与脚本
| 脚本 | 用途 |
|------|------|
| `pc/abi_monitor.py` | PC UI（曲线/tuf/还原 DUMP）——启动方式见第 7 节，**"打不开"现象见最后一节** |
| `pc_tool/learn_ramp.py` | 油门空载自学习（8 档爬升，PWM 自动回零） |
| `pc_tool/snap_verify.py` | ARM→PWM 自动触发→SNAP=3 验证 |
| `pc_tool/sd_verify.py` | SD 自动 SAVE+LS 验证 |
| `pc_tool/flash_verify.py` | Flash 保存自动化验证（时序不稳，仅参考） |
| `pc_tool/verify_cal.py` | CAL 命令 + 极性 + EMA 校准验证 |
| `pc_tool/verify_task7.py` | 全流程实测（ARM→驱动→保存→复位持久） |

---

## 时间线（工作过程回顾）

### 2026-08-06 上午：Task 1 工程骨架 + Task 1A PWM（编译/烧录/实机全部打通）
- 拷贝 `1.LED闪烁` 例程 → `firmware\abi_monitor_h743`
- 自建 `Drivers\User\Src\usart.c/h`（USART1 921600，IRQ 收 FIFO，回调 `Cli_OnChar`）
- 自建 `App\app_config.c/h`（3/4/5 档抽稀表 + Bank2 扇区 7=0x081E0000 Flash 持久化，CRC16 查表，H7 32B FLASHWORD）
- 自建 `App\app_cli.c/h`（HELP/ID/CFG SHOW/SAVE/RESET/GEAR + PWM）
- 自建 `App\app_pwm.c/h`（PA7/TIM3_CH2，500Hz：PSC=2→120MHz/…，见代码）
- **编译坑**：① `usart.c` 笔误 `&gp`→`&gpio`（GCC 报错才发现）；② CRC16 表手工数据错乱→脚本生成 256 项标准表；③ `stm32h7xx_hal_conf.h` 需开 UART/TIM 模块；④ xpack GCC 缺 cc1 → 换 arm-official 完整版
- **烧录成功**：14.68KB → 0x08000000，verify 通过；上电读 `0x081E0000` = `A5A5C0DE 04020103`（默认配置写入成功）

### 2026-08-06 晚：Task 1A/2 测速修复 + 油门自学习
- **测速 bug 修复（3 个）**：
  1. TIM2 CNT 上电垃圾值 → `Abi_Init` 末尾 `__HAL_TIM_SET_COUNTER(0)` 清零
  2. EXTI4 配了 RISING_FALLING → idx 每转 +2（Δcnt/Δidx≈2000 而非 4000）→ 改只上升沿
  3. A/B 相序致 TIM2 反向计数（正转 cnt 递减）→ UTO ISR 与 `Abi_GetCnt` 统一取反 `0u - CNT` → 显示正
- 修复后交叉验证：PWM 900 → rpm≈6000，Δcnt/Δidx≈4000；rpm 钳制 ±60000 防污染
- **learn_ramp.py 空载自学习结果**：
  ```
  125‰→1113rpm  250‰→3196  375‰→4571  500‰→5166
  625‰→5472     750‰→5555  875‰→5800  1000‰→6303   启动≈125‰
  ```
  0~500‰ 近线性 ~10rpm/‰，500~1000‰ 饱和（空载）
- 输出：`pc_tool/calib/learn_20260806_224602.csv` + `learn_notes.md`（载荷/电机/电调变化须重学）

### 2026-08-06 晚：Task 3 事件记录管线（自动触发）
- **触发验证升级**（用户指示手动拧电机→PWM 自动驱动）；`snap_verify.py`：PWM0→ARM→PWM<n>→轮询 SNAP 至 state=3
- **snap_bin.c/h**：ring 1024 点环形缓存（head/tail 掩码）；SNAP_CAP **31000**（512KB AXI RAM 内 ring 16KB + bss 余量，计划 32768 超界改小）；状态机 0=IDLE/1=ARM/2=REC/3=DONE；ARM 后 |rpm|>10+Index 一整圈 → 回溯 400 点 + 记 0.5s
- 验证：PWM 700→count=12042；PWM 500→count=9849（自动触发通过）

### 2026-08-07 早：Task 4 SD 存储（SDMMC1 + FatFS 自动备份）
- 移植 FatFS **R0.13** 到 `firmware/FatFS`（LFN=0，VOLUMES=1，MAX_SS=512，FS_TINY=1）
- 新建 `diskio.c` glue：HAL_SD 轮询写、ClockDiv=10→24MHz、get_fattime()=0（无 RTC）
- 自建 `App/sd_card.c/h`：Sd_Init/Mount/SaveSnap/Ls/Raw/Stat
- 文件名规则（用户要求）：`S%07lu.BIN` + `FA_CREATE_NEW`，FR_EXIST → +1（60s 启动内唯一）
- 写入帧=BIN v2 + crc32；snap_bin.c 加 `Snap_Points()`/`Snap_Crc32()`；SNAP_HZ=1000
- **板级 3 个 RFI**：
  1. **死循环复位**：Snap_IsReady/Sd_SaveSnap 死循环+回调 → 返回参数 + 一次性标志（自动仅一次）
  2. **DCache 与 SDMMC 内部 DMA 冲突**：RAW 读出全 0 → **main.c 关闭 D-Cache（只开 ICache）**（根因是 D-cache 与 SDMMC 内部 DMA 缓存一致性，在注释中说明勿重开）　※
  3. **FatFS 缓冲 4B 对齐** → SDMMC DMA 需 32B → diskio.c `uint32_t[128] aligned(8)` 中转
- 实测：16GB FAT32（SDHC type=1 31116288 块），`S000018.BIN` 等 sz=101495 = 11+8+6342*16+4，自动保存 ret=0；重启数据仍在

### 2026-08-07 午：Task 5 Flash 芯片保存 + DUMP + README
- 用户要求"芯片保存之后，再在卡里备份一份"：snap 先存内部 Flash（掉电不丢），再 SD
- 新建 `App/flash_save.c/h`：Bank2 扇区 0..3（0x08100000，512KB，一次记录）；32B 头 {magic=SNAP_MAGIC_V2, n, hz, crc32(zlib)} + n×16B；H7 32B FLASHWORD 编程（静态 aligned(32)）；DUMP 从 Flash 直读按 PC 帧发
- usart.c/h 新增 `Usart_Write`（原始块发送，DUMP 用）
- CLI：FLASH / FSAVE / DUMP
- **自动保存时序 bug 已修复**：旧 `g_snap_autosaved` 置 1 后永不复位、只第一次保存 → 改 **DONE 上升沿 `g_done_prev`**，每次触发都 Flash 先 SD 后
- 上板验证：n=6273；`FLASH` stored==current==count；DUMP 100436B magic=AB1C0002 n=6273 hz=1000 crc_ok=True；Reset 后 data=1 stored=6273（掉电保留）
- 排查经验：调试用 `openocd -c halt` 检查 PC 未 resume 会"无响应"→用 `-c reset run`；CLI 在 USART IRQ 中执行，Flash 擦写忙碌时回显被吞（重试即可，非故障）

### 2026-08-07 下午：Task 6 实测校准（步数 EMA + 极性）
- 新增 `cfg_steps_per_rev`（默认 4000）+ `cfg_pol`（A/B 极性）进 AppConfig Flash 持久化
- Index→Index EMA 平滑（`(ema*7+d)/8`）；测速公式改用 `Abi_GetStepsPerRev()`
- CLI：`CAL STEPS / SET <n> / RESET`、`CFG POL 0|1`；`Config_Persist` 统一保护不互相覆盖
- 实测：PWM900 ~1.5s → idx 146 圈，EMA=4000（AS5074P 4X 标称）；CAL SET 4250 后 `reset run` 仍 4250（Flash 持久化生效）

### 2026-08-07 夜：Task 7 综合验证（全流程实测）
- 全流程：ARM → PWM 900 1.5s → 自动 done=6850 点 → Flash+SD 自动保存（S000511.BIN）→ DUMP 109668B（magic=AB1C0002 n=6850 hz=1000，ret 0）→ 复位后 FLASH stored=6850 仍在
- 修正计划书 T1/T2/T3 历史子步骤勾选遗漏
- 提交：Task6 58b42df；Task7 fb5bceb

### 2026-08-07 夜：PC UI 协议适配（UI "捕捉不到转动" 修复）
- **根因**：`pc/abi_monitor.py` 是旧字幕工程拷贝，命令/推送协议与本固件完全不匹配——
  UI 发 `MONITOR START/STOP`、`SNAP?`、`ABI?`、`TIME`、`LOG DUMP USB`、`REC NOW`；固件只认 `ARM`/`DISARM`/`SNAP`/`DUMP`；
  UI 实时转速只认 `L,` 短帧（`L,rpm.0,dir,armed,phase,remain,log_n`），固件推的是 `rpm = N` 文本行 → 点①监控后板子根本没进 armed，转速框永远是 "—"
- **修复（固件 app_cli.c 加 PC 兼容层，UI 零改动）**：
  - `MONITOR START` → Snap_Arm + 开启 10Hz `L,` 帧推送；`MONITOR STOP` → Snap_Disarm
  - `SNAP?`/`ABI?` → cli_snap / cli_id；`TIME` → 空应答；`REC NOW` → Snap_Arm；`REVS` → 空应答
  - `LOG DUMP [USB]` → snap 点按 `D,t_ms,rpm.0,dir,1,idx` 文本行回传 + `D END n`（UI 自动拉取路径）
  - snap DONE → 广播 `# RECORD done | n=… seg=1 RECORD done` → UI `_schedule_read_once` 自动拉取
- **细节坑**：L 帧 `parts[1]` 必须带小数（`0.0`），否则 UI 按长帧解析错位；`put(".0,")` 后不能再补 `put(",")`（会变空字段导致 UI ValueError 丢帧）
- 实测（COM21）：`ABI?` 返回版本 ✓；`MONITOR START` 后 10Hz `L,0.0,0,1,0,0,0` 帧正常 ✓；帧格式与 UI 短帧解析逐字段对齐 ✓
- **待实机验证**：电机通电转轴时 UI 实时转速跳动 + 触发记录自动拉取（本轮电机未通电，cnt=0）
- 提交：`docs(h743): PC UI 协议适配`

### 2026-08-07 深夜：拷贝曲线工作室等 PC 模块（曲线工作室"能开了"）
- **根因**：`pc/abi_monitor.py` 是从旧字幕工程拷来的 UI，依赖 4 个外部模块但只拷了主文件 → `snap_edit`/`rpm_chart`/`curve_studio`/`abi_sim` 缺失，import 被 try/except 吞成 `None`，主流程能跑但「打开曲线工作室」报 `无法加载`
- **修复**：从 `D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\pc\` 拷贝 `snap_edit.py`（剪辑/距离/滤波 7 函数）、`rpm_chart.py`（RpmChart 绘图）、`curve_studio.py`（open_curve_studio 独立窗口）、`abi_sim.py`（SimLink 仿真，可选）到本工程 `pc\`
- **验证**：`py_compile` 4 模块全 OK；真实 import 均成功（snap_edit 7 函数、RpmChart、open_curve_studio、SimLink）；签名与 abi_monitor.py 调用逐一核对匹配；curve_studio 对 abi_monitor 的引用均有 try/except fallback
- 提交：`feat(pc): 补曲线工作室/编辑/绘图模块（拷自旧工程）`

---

## 遇到的问题 & 如何解决（完整清单）

| # | 问题 | 根因 | 解决 |
|---|------|------|------|
| 1 | xpack GCC 缺 cc1.exe，编译失败 | xpack 版本缺编译核心 | 换 `D:\arm-official` 完整版；Makefile GCC_PREFIX 已改 |
| 2 | CRC16 表手工数据错乱（excess elements） | 表靠手抄 | 脚本生成标准 256 项表替换 |
| 3 | USART 不工作 | 例程无 USART、hal_conf 未开 UART | 自建 usart.c（IRQ+FIFO）；开启 `HAL_UART_MODULE_ENABLED` |
| 4 | 编译报错 `&gp` | 笔误 | 改 `&gpio` |
| 5 | TIM2 CNT 上电垃圾值 | 上电 CNT 未知 | `__HAL_TIM_SET_COUNTER(0)` |
| 6 | idx 每转 +2 | EXTI RISING_FALLING | 改只上升沿 |
| 7 | rpm 显示负数/垃圾 | A/B 反序反向计数 | `0u - TIM2->CNT` 统一取反（signi完成） |
| 8 | rpm 尖峰 | 单帧异常 | ±60000 钳制 |
| 9 | SNAP_CAP 32768 放不下 | RAM 预算 BSS 溢出 | 改 31000 |
| 10 | 自动保存只触发一次 | `g_snap_autosaved` 不复位 | 改 DONE 上升沿 `g_done_prev` |
| 11 | SD 读写坏 | DCache 与 SDMMC DMA 冲突 | **main.c 关 D-Cache（只开 ICache）**；缓冲 32B对齐（diskio.c 中转） |
| 12 | REM 无 RTC | 无 RTC | `get_fattime()` 固定 0，文件时间序 |
| 13 | DUMP/FLASH 串口无响应 | `openocd -c halt` 后 stuck | `-c reset run` 恢复 |
| 14 | Flash 擦写时 CLI 回显被吞 | CLI 在 USART IRQ 执行+擦写阻塞 | 属正常，加长等待重试 |
| 15 | 配置结构变大（36B）超 H7 32B FLASHWORD | AppCfg 加了 steps+pol | Config_Save 两次 32B 对齐写（64B 缓冲） |
| 16 | PC UI"打不开" | 单实例端口锁被残留进程占用 | 已修复：锁冲突弹窗提示 + `启动PC_UI.bat` 一键 --usb（commit 41de68） |
| 17 | PC UI 点①监控"捕捉不到转动" | UI 命令/推送协议与本固件不匹配（旧工程拷贝） | 固件加 PC 兼容层：MONITOR START/STOP、SNAP?、ABI?、TIME、LOG DUMP、L, 帧 10Hz、# RECORD done（见当日日志） |

---

## 与计划的实施差异（接收人员关注）

| 计划 | 实施 | 原因 |
|------|------|------|
| Keil MDK5 + 例程可视化 | **无 Keil，GNU gcc + Makefile + OpenOCD/STM32CubeProg CLI** | 本机无 Keil UV4 |
| 记录窗口 0.8s | **0.5s**（readme/文档同步 0.5s） | 与 Snap 点数/RAM/@M成熟档位匹配；PCM 档位数名以实施为准 |
| ADC？ | — | 无 |
| SNAP_CAP 32768 | **31000** | BSS 余量考虑 |
| 自动保存顺序 | 计划先 SD；**实施先 Flash 后 SD** | 用户指示"芯片保存之后在卡里备份" |
| 文件名 | 计划旧命名 | **`S%07lu.BIN` + FA_CREATE_NEW（不覆盖）**，用户要求 |
| 接线 A 相 PA0 | **PA5（TIM2_CH1）** | PA0 板上未引出（丝印 A0C=PA0_C 仅 ADC） |
| 档数 3 | 默认 3（可 4/5） | 同计划 |
| app 文件名 | `App/cli.c` → **`App/app_cli.c`** | 其他模块衔接 |
| interrupt 回调 | `App_UartRxCb` → **`Cli_OnChar`** | 自建 usart.c 设计 |
| FLASH_BASE | 0x081F0000 → **0x081E0000**（扇区 7） | H7 扇区对齐 |

### 遗留/待办（接收人员）
- [ ] **PC UI（pc/abi_monitor.py）"打不开"**：用户已要求稍后处理。排查结论：脚本实际可正常进入 mainloop（本会话运行并从超时验证），Tk/依赖齐全；**最可疑是单实例端口锁（端口 47329 已被某实例占用时第二个进程静默 `sys.exit(0)`）**。处理入口：核对本机是否残留旧 python 进程（`netstat -ano | findstr 47329` / 任务管理器杀残留）+ 幂等锁改为可强制覆盖。另：UI 有 `ui_crash.log`（`pc/` 下）。
- [ ] 转速闭环/PWM 驱动段（非本期）——需电机动力电源接入后实测 | 本工作日志已含接线提醒
- [ ] 有屏蔽说明（burst）更新（PC UI 曲线）。DUMP ± 对应帧
- [ ] SD 卡驱动不恢复上，仅本启动期的 S<秒> 唯一

---

## 重要运行备注（调试/验证时）
- 串口 COM21 @ 921600（CP210x USB-TTL）
- 板上电机动力电源只在测试瞬间接入（PWM 900 ~1.5s 即停）
- `RPM ON` 每秒回显 转速+counts+idx+div，是验证测速的最快方式
- 凡是改动内存访问/DMA，**勿重开 D-Cache**（见 issue 11）
- Git 提交规范与本仓库一致：`feat(h743): Task...` / `docs(h743): ...`

## 提交历史
```
fb5bceb docs(h743): Task7 综合验证全流程实测通过（ARM→驱动→自动保存→DUMP→复位持久）
58b42df feat(h743): Task6 Index一圈步数EMA校准(可存Flash) + A/B方向极性可配(POL)
73d210f feat(h743): Task5 Flash芯片保存+DUMP PC帧(掉电不丢) + README + 自动保存改DONE上升沿
d81255c feat(h743): Task4 SDMMC1+FatFS 自动备份(时间名不覆盖,BIN v2+crc32) + 自动验证脚本 + 板级RFI修复(DCache/对齐/DMA)
9ccfca5 feat(h743): Task3 事件记录管线 ring回溯+0.8s自动触发 BIN v2 + 自动验证脚本
5cb88eb feat(h743): Task2 测速修复(TIM2清零/EXTI4上升沿/方向补偿) + 油门特性自学习脚本
```