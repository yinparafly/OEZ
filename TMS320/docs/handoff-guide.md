# AbiMonitor_TJX —— 接手工工程备忘录

> 日期：2026-08-02（最后更新）  
> 目的：让新接手的工程师一次性了解项目目标、已完成的工作、当前阻塞点、文件位置、构建流程和下一步操作。  
> 阅读顺序：**§1 概览 → §2 当前状态（最重要）→ §3 快速接手指南 → §4 文件清单 → 附录（按需查阅）**。

---

## 1. 项目概览

### 1.1 做什么

在天机星 TMS320F28P550SJ9 开发板上实现**事件驱动（非均匀采样）测速记录器**：

- ABI 编码器（1000PPR，4X 解码 = 4000 计数/圈）的**每个计数变化**触发 DSP 中断，记录 16B 事件点 `(t_us u32, counts i64, index_n u32)`。
- **回溯 400 点 + 续记 2400 点**组成一次弹射记录（总容量 3400 点），帧格式 `AA×10 + 55 + magic 0xAB1C0002 + … + CRC32`。
- 记录内容存档到 SD 卡（SPIB, FatFS R0.15），PC 端 `abi_tjx.py`（pyserial）自动拉帧、校验、出图。

### 1.2 核心决策（已确认，不可轻易推翻）

1. **PCM 逐边沿事件内核 = 唯一测速源**——原固件的 1kHz 定时采样 `Encoder_Count` 差分测速**已作废**。所有速度反馈（记录触发判定、闭环 PI 控制 RPM 输入）均来自 PCM 事件流。
2. **电机闭环测试跑完即停**——测试序列 1500/2000 RPM 各 20s，科目完成后永久停机（`g_seq_done=1`），不再无限循环。
3. **SPIB 引脚最终锁定**：CLK=GPIO14, PICO=GPIO30, POCI=GPIO31, CS=GPIO6（128PDT 封装全部可用，与 EQEP1/SCIA/RGB 无冲突）。
4. **时间戳方案**：CPUTIMER0 自由运行（countdown 150000 tick @150MHz → 1ms）+ 1kHz ISR 维护 64 位 µs 时钟 + countdown 快照插值（亚 µs 精度 `/150`）。
5. **执行顺序**：①链路验证（SD + PC + 串口）→ ②PCM 弹射记录 → ③原计划剩余 Task → ④待办（动态步长 / CLB 对比 / 闭环 CSV）。

### 1.3 设备与环境

| 项 | 值 |
|---|---|
| 芯片 | TMS320F28P550SJ9（128PDT, 150MHz, 1088KB Flash, 133KB SARAM） |
| 开发板 | 天机星（立创 TJX），板载 W25Q32（SPIA, GPIO0-3, **不使用**） |
| 编码器 | ABI 1000PPR → 4X=4000 计数/圈 |
| 电机驱动 | EPWM1 GPIO0/1（PWM 输出 + Hall 编码器反馈） |
| 串口 | SCIA GPIO29(TX)/GPIO28(RX), 921600 8N1, COM23 |
| RGB LED | GPIO20(B, 低亮) / GPIO21(G, 低亮) |
| SD 卡 | SPIB GPIO14(CLK)/30(PICO)/31(POCI), CS=GPIO6（软件控制） |
| EQEP1 | A=GPIO50, B=GPIO51, I=GPIO53（正交模式, QPOSMAX=0xFFFFFFFF） |

---

## 2. 当前状态（最重要——先读这里）

### 2.1 已完成（done ✓）

| # | 事项 | 验证状态 |
|---|---|---|
| 1 | 基线工程（Task 1/2）：从立创模板复制、构建通过 → 0 错误 0 警告 | 已构建 + 烧录 |
| 2 | g_seq_done 电机跑完即停 + FeedForward 清理 | 已构建 + 烧录 (build_out22) |
| 3 | 设计文档 3 份：重构 spec、重构 plan（12 Task）、工作日志（D1-D7 决策） | `docs/superpowers/` |
| 4 | PC 解析库 `pc/abi_tjx.py`（SNAP_PREAMBLE, snap_crc32, parse_bin_end, parse_snap_bindump, parse_bin_frame, rpm_from_counts_series）| `test_abi_tjx.py` **ALL PASS** |
| 5 | SysConfig `c2000.syscfg`：SCIA 921600, EQEP1 正交, CPUTIMER0 1kHz, SPIB + SD_CS GPIO6, RGB GPIO20/21, EPWM1 | board.h 宏已验证 |
| 6 | FatFS R0.15 已解压到 `app/fatfs/`，ffconf.h 已改 `FF_USE_LFN=1` | — |
| 7 | diskio.c 写完成：SD 卡 SPI 模式 0 底层驱动（SPIB 400kHz 初始化 / 25MHz 读写, CMD0/CMD8/ACMD41/CMD58/CMD9 CSD 解析, SPI_setConfig 切换波特率） | 编译通过（2 个 shift 警告已修复为 uint32_t 转换） |
| 8 | sd_fatfs.c/h：挂载 + 测试写读往返（创建文件 → 写 64B → 读回 → memcmp 校验 → 删除） | 编译通过 |
| 9 | cli.c/h：PING/FW?/SD?/SD INIT/SD TEST/HELP 最小命令集 | 编译通过 |
| 10 | main.c 已集成 cli_init() + cli_task()（call site 在 while(1) 的 LED 闪烁后） | 编译通过 |
| 11 | 无头构建 + 烧录流程已验证打通（ccs-server-cli.bat + DSLite.exe） | — |

### 2.2 当前阻塞——链接失败（RAM 不够）

**原因**：FatFS `FF_USE_LFN=1` 启用了 `ffunicode.c`（Unicode 转换大表），导致：

- `.const` 段膨胀到 **0x8086**（≈33KB）——当前链接脚本 `RAMLS5|RAMLS6` 只有 4KB
- `.text` 段膨胀到 **0x482c**（≈18KB）——当前链接脚本 `RAMLS0-5` = 6×2KB = 12KB 不够

**具体错误**（`28p55x_generic_ram_lnk.cmd` 第 58 行）：
```text
error #10099-D: program will not fit into available memory
section ".const" size 0x8086 page 0 → placement fails for RAMLS5|RAMLS6
section ".text" size 0x482c page 0 → placement fails for RAMLS0-5
```

**同时有 unresolved symbols**（`disk_status` 在 cli.c 的 `SD?` 命令中引用——此符号已在 diskio.c 中定义，链接问题关联 `.text` 内存不足导致 `diskio.obj` 未链接进去）。

### 2.3 修复方向（优先级排序）

1. **方案 A（推荐，最小改动）**：**关掉 `FF_USE_LFN`**（ffconf.h 改回 `0`）——Unicode 转换表不再编译进 `.const`，`.const` 立即缩小到几百字节，链接通过。负面影响：文件名不支持中文长文件名，本项目不需要。
2. **方案 B（用更多 RAM）**：修改 `28p55x_generic_ram_lnk.cmd` 第 48/58 行，把 `.text` 和 `.const` 放到更大的 RAM 段：
   - `.text` 改成 `>> RAMLS0 | RAMLS1 | RAMLS2 | RAMLS3 | RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMGS0 | RAMGS1`  
   - `.const` 改成 `>> RAMGS0 | RAMGS1 | RAMGS2 | RAMGS3 | RAMLS8 | RAMLS9`
   - RAM 总量 133KB，代码+常量≈51KB，仍有余量。**注意**：RAMGS0-3 和 LS8/9 要预留 73.6KB 给后续事件缓冲 `SNAP_RAM`。
3. **方案 C（Flash 构建）**：用 `CPU1_FLASH` 配置编译，`.const` 放 Flash（不在乎体积），但调试阶段灵活性降低。

### 2.4 Task 1 剩余待做

| Step | 内容 | 状态 |
|---|---|---|
| Step 1 | PC 解析库 | ✅ done |
| Step 2 | SysConfig | ✅ done |
| Step 3 | FatFS + diskio | ✅ 源码写完，**链接阻塞** |
| Step 4 | cli + sd_fatfs | ✅ 源码写完，**链接阻塞** |
| Step 5 | 构建 + 烧录 + 串口断言 | ❌ blocked by 链接 |
| Step 6 | Commit | ❌ blocked |

---

## 3. 快速接手指南

### 3.1 第一件事：修链接 → 构建 → 烧录 → 串口验证

```powershell
# 1. 修 ffconf.h（最简单路径：关 LFN）
notepad D:\oezcon\TMS320\AbiMonitor_TJX\app\fatfs\ffconf.h
# 改 FF_USE_LFN 从 1 改回 0

# 2. 同步到构建副本
Copy-Item -Recurse -Force "D:\oezcon\TMS320\AbiMonitor_TJX\app" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"

# 3. 无头构建
cmd /c "D:\ti\ccs2011\ccs\eclipse\ccs-server-cli.bat -workspace D:\temp\opencode\ccs_ws -application com.ti.ccs.apps.buildProject -ccs.projects AbiMonitor_TJX -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport"

# 4. 构建成功 → 烧录
& "D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe" load --config="D:\temp\opencode\ccs_ws\AbiMonitor_TJX\targetConfigs\TMS320F28P550SJ9.ccxml" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\CPU1_RAM\AbiMonitor_TJX.out"
# 成功标志: Running... Success

# 5. 串口验证（pyserial 或任何终端 COM23, 921600 8N1）
# 发: PING  → 应答: # PONG
# 发: FW?   → 应答: # FW 0.1.0-PCMREBUILD
# 发: SD?   → 应答: # SD NOINIT (无卡) 或 # SD READY (有卡)
# 发: SD INIT → 应答: # SD OK 或 # SD FAIL ...
# 发: SD TEST → 应答: # SDTEST OK (写读往返一致)
```

### 3.2 第二件事：继续实施计划

实施计划详见 `docs/superpowers/plans/2026-08-02-abimonitor-pcm-rebuild-plan.md`，Task 2-12 尚未开始。

当前在 **Task 1（链路验证）** 的 Step 5，修好链接阻塞后应完成 Step 5（串口断言全部命令 → 链路 OK），然后 Commit。

### 3.3 重要注意事项

- **源码不能直接在 `D:\temp\opencode\ccs_ws\` 下编辑**——那是构建副本。源码在 `D:\oezcon\TMS320\AbiMonitor_TJX\`，改完必须 Copy-Item 同步再构建。
- **C2000 的 char 是 16 位**——`uint8_t` 实际 16 位，移位运算小心（`<< 8` 或 `<< 16` 要转 `uint32_t`）。
- **ISR 内禁浮点/除法**——rpm 换算放 1kHz 定时任务，ISR 只做整数差分。
- **LSPCLK = SYSCLK/4 = 37.5MHz**，勿改（会影响 SCI 波特率）。
- **921600 波特率**：BRR=4 → 实际 937500，误差 +1.72%，可接受。
- **正交模式天然 4X**——不能硬件切 1X/4X，只能用 PCM 步长 N 软降速。

### 3.4 构建命令速记

| 操作 | 命令 |
|---|---|
| 构建 (RAM) | `cmd /c "D:\ti\ccs2011\ccs\eclipse\ccs-server-cli.bat -workspace D:\temp\opencode\ccs_ws -application com.ti.ccs.apps.buildProject -ccs.projects AbiMonitor_TJX -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport"` |
| 构建 (FLASH) | 同上，`CPU1_RAM` 改 `CPU1_FLASH` |
| 烧录 | `& "D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe" load --config="<proj>\targetConfigs\TMS320F28P550SJ9.ccxml" <out>` |
| 同步源码 | `Copy-Item -Recurse -Force "D:\oezcon\TMS320\AbiMonitor_TJX\app" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"`（改其他目录同理） |
| PC 测试 | `python D:\oezcon\TMS320\pc\test_abi_tjx.py`（ALL PASS 即通过） |

---

## 4. 完整文件清单

### 4.1 固件源码（`D:\oezcon\TMS320\AbiMonitor_TJX\`）

| 路径 | 用途 | 状态 |
|---|---|---|
| `empty_driverlib_main.c` | main() + 闭环测试状态机 + CLI 集成 | ✅ 已写 CLI hook |
| `c2000.syscfg` | SysConfig（SCIA/EQEP/SPIB/CPUTIMER0/RGB） | ✅ |
| `app/cli.h` | CLI 头文件（cli_init/cli_task 声明, cli_put_raw/cli_printf 导出） | ✅ |
| `app/cli.c` | 最小 CLI 实现（PING/FW?/SD?/SD INIT/SD TEST/HELP） | ✅ 编译通过 |
| `app/sd_fatfs.h` | SD FatFS 封装头文件 | ✅ |
| `app/sd_fatfs.c` | SD 挂载 + 测试写读往返 | ✅ 编译通过 |
| `app/fatfs/ff.h` | FatFS R0.15 主头文件（原版） | ✅ |
| `app/fatfs/ff.c` | FatFS R0.15 核心逻辑（原版） | ✅ |
| `app/fatfs/ffconf.h` | FatFS 配置（FF_USE_LFN=1 **←当前阻塞根源**） | ⚠ 需改 |
| `app/fatfs/diskio.h` | FatFS 磁盘接口头文件（原版） | ✅ |
| `app/fatfs/diskio.c` | **自写** SPIB SD 卡底层驱动（400kHz init / 25MHz rw, CMD0/CMD8/ACMD41/CMD9 CSD, disk_read/write/ioctl） | ✅ 编译通过 |
| `app/fatfs/ffunicode.c` | Unicode 转换大表（**当 FF_USE_LFN=1 时编译入 .const 33KB**） | ⚠ 导致内存溢出 |
| `app/fatfs/ffsystem.c` | FatFS 系统接口（时间戳等，原版） | ✅ |
| `app/fatfs/00history.txt` | FatFS 版本历史 | — |
| `app/fatfs/00readme.txt` | FatFS 说明 | — |
| `28p55x_generic_ram_lnk.cmd` | RAM 构建链接脚本 | ⚠ 需扩展 .text/.const 段 |
| `28p55x_generic_flash_lnk.cmd` | Flash 构建链接脚本 | — |
| `device/` | TI 自动生成（device.c/h, f28p55x_codestartbranch.asm） | 不修改 |
| `lckfb_tjx_init/` | 立创板级初始化库（tjx_init.c/h, lc_printf 等） | 不修改 |
| `module_driver/` | 电机 Hall 编码器驱动（bsp_motor_hallencoder.c/h） | 不修改 |
| `targetConfigs/` | JTAG 目标配置（TMS320F28P550SJ9.ccxml） | 不修改 |
| `CPU1_RAM/` | **构建产物目录**（.out, .map, .obj → 在副本 `D:\temp\opencode\ccs_ws\` 下） | 构建自动生成 |

### 4.2 ESP32 参考工程（只读，不修改）

| 路径 | 用途 |
|---|---|
| `D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\` | ESP32 原版固件、abi_monitor.py、curve_studio.py、snap_bin.cpp（协议/业务逻辑参考） |

### 4.3 PC 工具（`D:\oezcon\TMS320\pc\`）

| 路径 | 用途 | 状态 |
|---|---|---|
| `abi_tjx.py` | 解析库（SNAP_PREAMBLE, snap_crc32, parse_bin_end, parse_snap_bindump, parse_bin_frame, rpm_from_counts_series） | ✅ ALL PASS |
| `test_abi_tjx.py` | 解析库测试（CRC/解析/CRC损坏否定/帧流） | ✅ ALL PASS |

### 4.4 文档（`D:\oezcon\TMS320\docs\`）

| 路径 | 用途 |
|---|---|
| `superpowers/specs/2026-08-02-abimonitor-pcm-rebuild-design.md` | **当前设计文档**（PCM 唯一测速源，含评审全部采纳点） |
| `superpowers/plans/2026-08-02-abimonitor-pcm-rebuild-plan.md` | **当前实施计划（12 Task）** |
| `superpowers/specs/2026-08-01-abimonitor-eventspeed-design.md` | 原设计（历史快照，仅供参考） |
| `superpowers/plans/2026-08-01-abimonitor-eventspeed-plan.md` | 原计划（历史快照） |
| `工作日志_AbiMonitor_TJX.md` | **工作日志**（D1-D7 决策记录, 复用要点 1-14, 构建/烧录踩坑） |
| `2026-08-02_eQEP捕获中断缺失_问题与发现.md` | 静态分析发现的问题 1-7 |
| `2026-08-02_新方案_事件驱动测速记录器_PCM方案.md` | 方案 A 选择分析 |
| `lckfb_wiki/` | 立创 wiki 文档存档（接线图/入门文档） |

### 4.5 构建副本（`D:\temp\opencode\ccs_ws\AbiMonitor_TJX\`）

此目录是 CCS 无头构建的工作区。**不要直接编辑**这里的文件。每次在 `D:\oezcon\TMS320\AbiMonitor_TJX\` 修改后，Copy-Item 同步到此目录再构建。

### 4.6 工具链路径

| 工具 | 路径 | 版本 |
|---|---|---|
| CCS (Theia) | `D:\ti\ccs2011\` | 20.1.1 |
| C2000Ware | `C:\ti\c2000ware\C2000Ware_5_04_00_00\` | 5.04.00.00 |
| 编译器 | `D:\ti\ccs2011\ccs\tools\compiler\ti-cgt-c2000_22.6.1.LTS\` | 22.6.1.LTS |
| SysConfig | CCS 内嵌 | 1.23 |
| driverlib | `C:\ti\c2000ware\C2000Ware_5_04_00_00\driverlib\f28p55x\driverlib\` | — |
| DSLite (烧录) | `D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe` | — |

---

## 附录

### A. 构建/烧录踩坑记录

- `ccs-serverc.exe -data ...` 只输出 "Initializing CCS... Done" 就退出，**不构建**（EXIT=0 假成功）→ 不可用。
- Theia `ccstudio.exe --no-splash --data ...` 无头崩溃（EPIPE/CSSUpdaterElectronApplicationContribution）→ 不可用。
- **正确方法**：`ccs-server-cli.bat`（Eclipse 架构的 CLI），需要 `-ccs.projects` 指定项目名；首次须加 `-ccs.autoImport`。
- 改源码后**必须有 Copy-Item** 同步到构建副本——CCS 构建读取的是工作区副本文件。

### B. C2000 编程提醒

| 要点 | 说明 |
|---|---|
| char = 16 位 | uint8_t 也是 16 位，8 位只是逻辑范围；移位 >=8 位需先转 uint32_t |
| int = 32 位 | eabi 模式下的 int |
| 内存段分散 | 不同 RAM 段地址不连续（LS: 0x8000-0xBFFF, GS: 0xC000-0x13FFF, LS8/9: 0x14000-0x17FFF），大数组不能跨段**连续**存放（`>>` 操作符可分散分配） |
| ISR 在 .text | 假设函数调用寻址范围可跨任何 RAM/Flash |

### C. 关键 SysConfig 宏（board.h 自动生成）

```c
#define SD_SPI_BASE          SPIB_BASE
#define SD_CS_GPIO_PIN_CONFIG GPIO_6_GPIO6
#define Debug_Serial_BASE    SCIA_BASE
#define Debug_Serial_BAUDRATE 921600
#define Module_EQEP_BASE     EQEP1_BASE
#define RGB_B_GPIO_PIN_CONFIG GPIO_20_GPIO20
#define RGB_G_GPIO_PIN_CONFIG GPIO_21_GPIO21
#define myCPUTIMER0_BASE     CPUTIMER0_BASE
#define INT_Module_EQEP_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP5
```

### D. SPI API 速记（driverlib）

```c
// 配置波特率/模式（必须在 SPISWRESET 状态调用）
SPI_setConfig(SD_SPI_BASE, lspclkHz, SPI_PROT_POL0PHA0, SPI_MODE_CONTROLLER, bitRate_kHz, 8U);

// 字节收发（16 位寄存器高位传 8 位数据，低位读回）
SPI_writeDataNonBlocking(SD_SPI_BASE, (uint16_t)dat << 8U);
uint8_t rx = (uint8_t)(SPI_readDataBlockingNonFIFO(SD_SPI_BASE) & 0xFFU);

// CS 控制（GPIO6, 低有效）
GPIO_writePin(6U, 0U);  // CS=0 选通
GPIO_writePin(6U, 1U);  // CS=1 释放
```

### E. SCI API 速记（driverlib）

```c
// 阻塞发送（等待 FIFO 有空位）
SCI_writeCharBlockingFIFO(Debug_Serial_BASE, (uint16_t)c);

// 非阻塞接收
if (SCI_isDataAvailableNonFIFO(Debug_Serial_BASE)) {
    uint16_t rx = SCI_readCharNonBlocking(Debug_Serial_BASE) & 0xFFU;
}
```

### F. 设计文档关键参数

| 参数 | 值 |
|---|---|
| SNAP_CAP | 3400 点 |
| RING_CAP | 1200 点 |
| BACKTRACK_N | 400 点 |
| steps (每圈计数) | 4000 |
| 点格式 | `struct { uint32_t t_us; int64_t counts; uint32_t index_n; }` = 16B packed LE |
| BIN 帧头 | `AA AA AA AA AA AA AA AA AA AA 55` + `00 02 1C AB` (magic 0xAB1C0002 LE) + n(u16) + hz(u16=0) + 点阵 + crc32 |
| CRC32 | poly 0xEDB88320, init 0xFFFFFFFF, final ~, 与 zlib 一致 |
| 触发 | 武装期 `|rpm|>10` 进 STAGING; 确认 OR{I+1圈, ≥200ms, ring≥120, (rpm跌且ring≥40)} |
| ISR 预算 | 0.6µs ≈ 90 周期 @150MHz |
| GEAR_4X (N=1) | 逐计数, 20000rpm 时 1.333M 边沿/s, **80% CPU, 不可持续** |
| GEAR_1X (N=4) | 每 4 计数, 20000rpm 时 333k 边沿/s, **22% CPU**
