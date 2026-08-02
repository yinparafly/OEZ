# AbiMonitor_TJX 操作指南：构建、烧录、串口通讯与验证

> 覆盖无头构建、DSLite 烧录、串口交互验证、PC 测试套件的完整操作流程。  
> 所有路径和命令均已在 2026-08-02 实测打通。

---

## 1. 环境一览

| 项 | 值 |
|---|---|
| 开发板 | 天机星 TMS320F28P550SJ9 (128PDT) |
| 调试器 | 板载 XDS110 (JTAG) |
| 串口 | COM23, 921600 8N1, SCIA (GPIO29 TX / GPIO28 RX) |
| 源码目录 | `D:\oezcon\TMS320\AbiMonitor_TJX\` |
| 构建副本 | `D:\temp\opencode\ccs_ws\AbiMonitor_TJX\` |
| CCS 安装 | `D:\ti\ccs2011\` |
| PC 工具 | `D:\oezcon\TMS320\pc\` |

**核心原则**：源码在 `D:\oezcon\TMS320\AbiMonitor_TJX\` 下编辑，编辑完成后 **必须同步到构建副本再构建**。构建副本仅用于 CCS 无头构建，不直接编辑。

---

## 2. 源码→构建→烧录→验证 完整工作流

### 2.1 工作流总览

```
编辑源码 ──→ Copy-Item 同步到构建副本 ──→ ccs-server-cli.bat 无头构建
                                              │
                                              ├── 构建失败 → 看错误 → 修源码 → 回到开始
                                              │
                                              └── 构建成功 (0 errors)
                                                   │
                                                   └──→ DSLite 烧录 .out
                                                         │
                                                         └──→ 串口验证 (PING/FW?/SD? 等)
                                                               │
                                                               └──→ PC 测试套件 (test_abi_tjx.py)
```

### 2.2 Step 1：编辑源码

在源码目录下编辑文件。常用文件位置：

```
D:\oezcon\TMS320\AbiMonitor_TJX\
├── empty_driverlib_main.c    ← 主程序
├── c2000.syscfg              ← 外设配置（修改后需重新生成 board.h）
├── app\cli.c / cli.h         ← CLI 命令表
├── app\sd_fatfs.c / sd_fatfs.h ← SD 卡操作
├── app\fatfs\diskio.c        ← SD SPI 底层驱动
├── app\fatfs\ffconf.h        ← FatFS 配置
└── 28p55x_generic_ram_lnk.cmd ← RAM 链接脚本
```

### 2.3 Step 2：同步到构建副本

每次改完源码，必须把改动的文件拷贝到构建工作区副本：

```powershell
# 同步整个 app 目录（最常用）
Copy-Item -Recurse -Force "D:\oezcon\TMS320\AbiMonitor_TJX\app" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"

# 同步 main.c（单独改 main 时）
Copy-Item -Force "D:\oezcon\TMS320\AbiMonitor_TJX\empty_driverlib_main.c" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"

# 同步链接脚本（如果改了）
Copy-Item -Force "D:\oezcon\TMS320\AbiMonitor_TJX\28p55x_generic_ram_lnk.cmd" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"

# 同步 SysConfig（如果改了）
Copy-Item -Force "D:\oezcon\TMS320\AbiMonitor_TJX\c2000.syscfg" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"
```

### 2.4 Step 3：无头构建

在 PowerShell 中执行（**必须用 `cmd /c` 包裹**，否则参数被 PowerShell 吞掉）：

```powershell
cmd /c "D:\ti\ccs2011\ccs\eclipse\ccs-server-cli.bat -workspace D:\temp\opencode\ccs_ws -application com.ti.ccs.apps.buildProject -ccs.projects AbiMonitor_TJX -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport"
```

**参数说明**：

| 参数 | 含义 | 备注 |
|---|---|---|
| `-workspace` | 工程所在的 workspace 父目录 | 是 `ccs_ws` 不是 `AbiMonitor_TJX` |
| `-application` | 固定值 | `com.ti.ccs.apps.buildProject` |
| `-ccs.projects` | 工程名 | `AbiMonitor_TJX` |
| `-ccs.configuration` | 构建配置 | `CPU1_RAM` (RAM 调试) 或 `CPU1_FLASH` (Flash 烧录) |
| `-ccs.buildType` | 构建类型 | `full` (全量) 或 `incremental` (增量) |
| `-ccs.listErrors` | 列出错误 | 可选，构建完显示错误列表 |
| `-ccs.autoImport` | 自动导入工程 | 首次构建必须加，后续可省略 |

**构建输出产物**（在构建副本下）：`CPU1_RAM\AbiMonitor_TJX.out`

**解读构建输出**：

```text
# 成功
**** Build Finished ****
Errors for project 'AbiMonitor_TJX' (0):
CCS headless build complete! 0 out of 1 projects have errors.

# 失败（error 相关信息会打在中间）
gmake: *** [app/cli.obj] Error 1              ← 某文件编译失败
error #10099-D: program will not fit ...        ← 链接失败（内存不够）
error #10234-D: unresolved symbols remain       ← 链接失败（符号找不到）
```

**快速过滤错误**：

```powershell
cmd /c "..." 2>&1 | Select-String "error|Error|undefined|unresolved|cannot|fatal"
```

### 2.5 Step 4：烧录

构建成功（0 errors）后，用 DSLite 烧录 `.out` 到开发板：

```powershell
& "D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe" load --config="D:\temp\opencode\ccs_ws\AbiMonitor_TJX\targetConfigs\TMS320F28P550SJ9.ccxml" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\CPU1_RAM\AbiMonitor_TJX.out"
```

**成功标志**：

```text
... Running... Success
```

**注意事项**：
- 开发板 USB 线必须连接（XDS110 调试器）。
- 烧录完成后程序 **自动开始运行**（无手动复位）。
- `.ccxml` 文件由 SysConfig 自动生成，包含了 JTAG 连接配置和芯片型号（TMS320F28P550SJ9）。

---

## 3. 串口通讯与固件验证

### 3.1 方式 A：pyserial 交互脚本（推荐）

```powershell
python
```

```python
import serial
ser = serial.Serial('COM23', 921600, timeout=1)
ser.write(b'PING\r\n')
print(ser.readline())   # b'# PONG\r\n'
ser.write(b'FW?\r\n')
print(ser.readline())   # b'# FW 0.1.0-PCMREBUILD\r\n'
ser.write(b'SD?\r\n')
print(ser.readline())   # b'# SD NOINIT\r\n' 或 b'# SD READY\r\n'
ser.write(b'SD INIT\r\n')
print(ser.readline())   # b'# SD OK\r\n'
ser.write(b'SD TEST\r\n')
print(ser.readline())   # b'# SDTEST OK\r\n'
ser.write(b'HELP\r\n')
print(ser.readline())   # b'# CMDS: PING FW? SD? SD INIT SD TEST HELP\r\n'
ser.close()
```

### 3.2 方式 B：任意串口终端

使用 PuTTY、MobaXterm、Tera Term 或任何串口终端工具：

| 设置 | 值 |
|---|---|
| Port | COM23 |
| Baud | 921600 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| 换行 | CR 或 CR+LF |

连接后直接输入命令（以回车结束），固件回应用 `# ` 前缀。

### 3.3 第一阶段验证命令表（Task 1 全部断言）

| 命令 | 期望应答 | 说明 |
|---|---|---|
| `PING` | `# PONG` | 基础连通性 |
| `FW?` | `# FW 0.1.0-PCMREBUILD` | 固件版本 |
| `SD?` | `# SD NOINIT` 或 `# SD READY` | SD 卡状态（无卡=NOINIT, 有卡初始化后=READY） |
| `SD INIT` | `# SD OK` 或 `# SD FAIL <原因>` | 初始化 SD 卡并挂载 FatFS |
| `SD TEST` | `# SDTEST OK` 或 `# SDTEST FAIL <原因>` | 写 64B 文件 → 读回 → memcmp 校验 → 删文件 |
| `HELP` | `# CMDS: PING FW? SD? SD INIT SD TEST HELP` | 命令列表 |
| `(任意未知)` | `# UNKNOWN` | 未知命令不会导致崩溃 |

**验收标准（链路验证判定）**：

1. `PING` / `FW?` / `HELP` 应答正确（无需 SD 卡）
2. 不插卡：`SD?` → `NOINIT`（diskio 初始化失败 → STA_NOINIT）
3. 插卡：`SD INIT` → `# SD OK`（挂载成功）
4. 插卡：`SD TEST` → `# SDTEST OK`（写读往返一致）
5. PC 端 `abi_tjx.py --port COM23` 能连接并发命令收应答 → **链路 OK，Task 1 完结**

### 3.4 串口注意事项

- 固件主循环里有 `lc_printf` 定时打印（编码器状态/EPWM 诊断），**会在 CLI 交互时混入干扰输出**。正常现象——后续 Task 6 集成状态机后会关闭无关打印。
- CLI 使用 SCI FIFO 模式（SysConfig 默认开启），`SCI_writeCharBlockingFIFO` 自动等待 FIFO 有空位。
- RX 中断（`INT_Debug_Serial_RX_ISR`）已在 SysConfig 注册，当前仅清标志（不做接收——接收由 `cli_task()` 轮询 `SCI_isDataAvailableNonFIFO` 完成）。
- 发送命令以 `\r` 或 `\n` 结束均可（CR 和 LF 均视作命令终结符）。

---

## 4. PC 端解析库测试（独立于板端）

### 4.1 一键测试

```powershell
python D:\oezcon\TMS320\pc\test_abi_tjx.py
```

### 4.2 期望输出

```
test_constants PASS
test_snap_preamble PASS
test_snap_magic_v2 PASS
test_crc32_known_vector PASS
test_crc32_empty PASS
test_crc32_arbitrary PASS
test_parse_bin_end PASS
test_parse_snap_bindump_basic PASS
test_parse_snap_bindump_crc_fail PASS
test_parse_bin_frame PASS
test_rpm_from_counts_series PASS
ALL PASS
```

### 4.3 测试覆盖内容

| 测试项 | 验证点 |
|---|---|
| constants | SNAP_PREAMBLE, SNAP_MAGIC_V2 常量值正确 |
| crc32 | poly 0xEDB88320 标准测试向量、空数据、任意数据（与 zlib Python 结果一致） |
| parse_bin_end | `# BIN END steps=3400 crc=0xABCD1234` 正则解析 |
| parse_snap_bindump | 完整 BIN 帧解析（10×AA + 55 + magic + n + hz + 点阵 + crc32），CRC 损坏检测 |
| parse_bin_frame | 帧级重组（多个 payload 拼接） |
| rpm_from_counts_series | 非均匀采样 counts → RPM 差分计算 |

### 4.4 实现文件

- `test_abi_tjx.py`：纯脚本断言，无依赖（只用标准库 struct + zlib + re），可离线运行。
- `abi_tjx.py`：解析库，接口见源码注释。

---

## 5. 故障排查速查

### 5.1 构建失败

| 现象 | 原因 | 解决 |
|---|---|---|
| `fatal error #1965: cannot open source file "ff.h"` | include 路径不包含 `app/fatfs/` | 改用 `#include "fatfs/ff.h"` 或在 .cproject 加 include path |
| `error #20: identifier "DSTATUS" is undefined` | 未 include diskio.h | 添加 `#include "fatfs/diskio.h"` |
| `warning #64-D: shift count is too large` | C2000 的 char=16bit，对 uint8_t 移位 ≥8 位超宽 | 先转 `(uint32_t)csd[n]` 再移位 |
| `error #10099-D: program will not fit` | .const/.text 段超出链接脚本的内存区域 | 扩大链接脚本的段映射或关掉 FF_USE_LFN |
| `unresolved symbols remain` | 通常伴随内存不足错误——源 .obj 未链接进去 | 先修内存溢出 |
| `ccs-server-cli.bat` 只输出帮助 | 参数被 PowerShell 吞掉 | 用 `cmd /c "... "` 包裹 |
| `ccs-server-cli.bat` 找不到工程 | 缺少 `-ccs.autoImport` | 首次加此参数 |

### 5.2 烧录失败

| 现象 | 原因 | 解决 |
|---|---|---|
| DSLite 找不到设备 | 开发板 USB 未连 | 连接 USB 线，检查设备管理器 XDS110 |
| DSLite 报连接错误 | JTAG 配置不对 | 确认 .ccxml 文件路径正确 |
| 烧录成功但板不运行 | 程序可能卡在初始化 | 串口看是否有 boot banner（当前固件会打印立创欢迎信息） |
| `ccs-serverc.exe` 假成功 | 该工具只初始化不构建 | 换用 `ccs-server-cli.bat` |

### 5.3 串口无响应

| 现象 | 原因 | 解决 |
|---|---|---|
| 终端无任何输出 | COM 口不对 | 设备管理器查看 XDS110 Class Application/User UART 的 COM 号 |
| 输出乱码 | 波特率不对 | 确认 921600 8N1（不是 115200） |
| 发命令无应答 | RX/TX 反了 | GPIO28=RX, GPIO29=TX |
| 只能发不能收 | 程序飞了或卡在 ISR | 重新烧录 |
| 混入编码器打印 | 主循环 lc_printf 干扰 | 正常现象——后续 Task 会关掉 |

---

## 6. 常用命令速查卡

```powershell
# ===== 构建 =====
cmd /c "D:\ti\ccs2011\ccs\eclipse\ccs-server-cli.bat -workspace D:\temp\opencode\ccs_ws -application com.ti.ccs.apps.buildProject -ccs.projects AbiMonitor_TJX -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport"

# ===== 烧录 =====
& "D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe" load --config="D:\temp\opencode\ccs_ws\AbiMonitor_TJX\targetConfigs\TMS320F28P550SJ9.ccxml" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\CPU1_RAM\AbiMonitor_TJX.out"

# ===== 同步源码 =====
Copy-Item -Recurse -Force "D:\oezcon\TMS320\AbiMonitor_TJX\app" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"
Copy-Item -Force "D:\oezcon\TMS320\AbiMonitor_TJX\empty_driverlib_main.c" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"

# ===== PC 测试 =====
python D:\oezcon\TMS320\pc\test_abi_tjx.py

# ===== 串口快速验证（Python） =====
python -c "import serial; s=serial.Serial('COM23',921600,timeout=1); s.write(b'PING\r\n'); print(s.readline().decode().strip()); s.close()"

# ===== 过滤构建错误 =====
cmd /c "..." 2>&1 | Select-String "error|Error|undefined|fatal"
```

---

## 7. 一切一图流

```
┌──────────────────────────────────────────────────────────────────────┐
│  源码目录                        构建副本                             │
│  D:\oezcon\TMS320\    Copy-Item  D:\temp\opencode\ccs_ws\             │
│  AbiMonitor_TJX\  ───────────→  AbiMonitor_TJX\                       │
│  ├── app/                        ├── app/            (同步的副本)       │
│  │   ├── cli.c/h                 │   ├── cli.c/h                       │
│  │   ├── sd_fatfs.c/h            │   ├── sd_fatfs.c/h                  │
│  │   └── fatfs/                  │   └── fatfs/                        │
│  ├── empty_driverlib_main.c      │   ├── empty_driverlib_main.c         │
│  ├── c2000.syscfg                │   ├── c2000.syscfg                  │
│  └── ...                         │   └── CPU1_RAM/    ← 构建产物在这里  │
│                                  │       └── AbiMonitor_TJX.out        │
│  只编辑这个                       └─────────────────────────────────── │
│                                            │                           │
│                      ccs-server-cli.bat 构建 │                           │
│                                            ↓                           │
│                                     CPU1_RAM\AbiMonitor_TJX.out        │
│                                            │                           │
│                               DSLite 烧录  │                           │
│                                            ↓                           │
│                                  ┌──────────────────┐                  │
│                                  │  TMS320F28P550SJ9 │                  │
│                                  │  天机星开发板      │                  │
│                                  └────┬───┬─────────┘                  │
│                                XDS110 │   │ SCIA (COM23)               │
│                                (烧录) │   │ (通讯)                     │
│                                       │   │                            │
│                                       │   ├─── 发命令 ──→ PING/FW?/SD? │
│                                       │   ├─── 收应答 ──→ # PONG/...   │
│                                       │   │                            │
│  PC 验证：                             │   │                            │
│  ├── test_abi_tjx.py (独立)            │   │                            │
│  └── pyserial 交互 (COM23) ────────────┘   │                            │
└──────────────────────────────────────────────────────────────────────┘
```
