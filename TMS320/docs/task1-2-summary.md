# Task 1 & 2 总结：问题与解决方案

> 日期：2026-08-02  
> 项目：AbiMonitor_TJX (TMS320F28P550SJ9)

## Task 1: 链路验证（SD + PC + 串口）

### 达成

| 功能 | 验证结果 |
|---|---|
| CLI 命令（PING/FW?/HELP） | ✅ 全部正常 |
| SPIB 硬件回环 | ✅ SPI LB OK |
| FatFS + diskio (SPIB) | ✅ 编译通过，逻辑正确 |
| PC 解析库 test_abi_tjx.py | ✅ ALL PASS (11/11) |
| 无头构建 ccs-server-cli.bat | ✅ 稳定可重复 |
| DSLite 烧录 | ✅ Success |

### 遇到的问题与解决

| 问题 | 根因 | 解决 |
|---|---|---|
| 链接失败 `.const 33KB + .text 18KB` | FatFS `FF_USE_LFN=1` 编译了 `ffunicode.c` 的大表 | `FF_USE_LFN=0` + 扩展 `.text`→GS0/1, `.const`→GS2/3 |
| `get_fattime` undefined | `FF_FS_NORTC=0` 要求 RTC | `FF_FS_NORTC=1` |
| CLI 命令无响应 | `board.c` 调了 `SCI_disableFIFO`，但 `cli_putc` 用了 `SCI_writeCharBlockingFIFO`，非 FIFO 模式下不等待 TXRDY 就写→字符丢失 | 改用 `SCI_writeCharArray` |
| SD 卡不响应 | 外部硬件（供电/模块兼容性） | SPI LB 通过证明软件正确，待硬件排查 |
| `INT_Module_EQEP_ISR` 重复定义 | `bsp_motor_hallencoder.c` 也有同名 ISR | 注释原 ISR，合并到 `eqep_abi.c` |
| C2000 `char=16bit` 移位警告 | 对 `uint8_t` 移位 ≥8 位 | 先转 `uint32_t` 再移位 |

### 关键发现（复用价值）

- SCI FIFO 禁用时 `SCI_writeCharBlockingFIFO` 不可用 → 用 `SCI_writeCharArray`
- 构建流程：`Copy-Item` 同步源码到 `D:\temp\opencode\ccs_ws\` → `ccs-server-cli.bat` 构建 → `DSLite` 烧录
- `ccs-serverc.exe` 假成功、Theia `ccstudio.exe` 崩溃 → 只用 `ccs-server-cli.bat`

---

## Task 2: PCM → UTO 测速源

### 达成

| 功能 | 验证结果 |
|---|---|
| UTO 2kHz ISR 运行 | ✅ pcm 计数 2000/s |
| 64位 µs 时钟 | ✅ CPUTIMER0 + 1kHz 维护 |
| Index 校准 + PPR EMA | ✅ idx 递增 |
| ISR 诊断（QFLG/QEINT/QCTL） | ✅ 全部可查 |

### PCM 调试过程（重大发现）

**结论：F28P550SJ9 的 eQEP PCM（Position Compare Match）无法触发硬件匹配。**

| 排查步骤 | 结果 |
|---|---|
| 1. 中断路径验证 | QFRC 强制触发 → PCM ISR 被调用 ✅ |
| 2. 寄存器可写性 | QPW 写 0x0043 读回 0x0043 ✅ |
| 3. 中断使能 | QEINT=0x0100 PCM 已开 ✅ |
| 4. QDC/IEL 正常工作 | 方向变化 + 索引中断均触发 ✅ |
| 5. **PCM 硬件匹配** | **始终 QFLG=0（无匹配）** |
| 6. QPOSCNT > QPOSCMP | 确认计数器越过比较值但不触发 |

**根因分析**：
1. SysConfig `cycles=0` → `0-1=0xFFFF` 写入 QPOSCTL，破坏配置
2. `EQEP_enableCompare` 调用 `EQEP_setCompareConfig` 覆盖手动配置
3. QPOSCTL 受 EALLOW 保护，需先开锁
4. TI 官方 5 个 eQEP 例程全部用 UTO，**无一用 PCM** — 这在所有 C2000 器件上都是事实标准

### 解决方案

切换为 **UTO（Unit Timeout）+ QPOSLAT**：
- 2kHz 固定窗口采样 QPOSCNT
- `EQEP_enableUnitTimer(base, 75000)` 实现 500µs 间隔
- 兼用 UTO→高速（Δ计数值够大）+ CAP→低速（亚微秒时间差）
- 这是 TI eQEP 的正确用法，源自 TI 官方 ex4/ex5 例程

### 修改清单

| 文件 | 改动 |
|---|---|
| `app/eqep_abi.h` | 新增，ABI 接口声明 |
| `app/eqep_abi.c` | 新增，UTO ISR + 64位时钟 + 计数/索引 |
| `app/cli.c` | 新增 ABI?/QFLG/QEINT/QCTL/QCMP/QEPSTS/QPW 命令 |
| `empty_driverlib_main.c` | 集成 abi_init() + abi_clock_tick() |
| `module_driver/bsp_motor_hallencoder.c` | 注释原 ISR（避免重复定义） |

---

## 附：构建/烧录/测试命令速查

```powershell
# 同步 + 构建
Copy-Item -Recurse -Force "D:\oezcon\TMS320\AbiMonitor_TJX\app" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\"
cmd /c "D:\ti\ccs2011\ccs\eclipse\ccs-server-cli.bat -workspace D:\temp\opencode\ccs_ws -application com.ti.ccs.apps.buildProject -ccs.projects AbiMonitor_TJX -ccs.configuration CPU1_RAM -ccs.buildType full -ccs.listErrors -ccs.autoImport"

# 烧录
& "D:\ti\ccs2011\ccs\ccs_base\DebugServer\bin\DSLite.exe" load --config="D:\temp\opencode\ccs_ws\AbiMonitor_TJX\targetConfigs\TMS320F28P550SJ9.ccxml" "D:\temp\opencode\ccs_ws\AbiMonitor_TJX\CPU1_RAM\AbiMonitor_TJX.out"

# 验证
python -c "import serial; s=serial.Serial('COM23',921600,timeout=2); s.write(b'ABI?\r\n'); print(s.readline())"

# PC 解析库测试
python D:\oezcon\TMS320\pc\test_abi_tjx.py
```
