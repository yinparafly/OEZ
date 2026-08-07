# ABI 电机监控记录器 — STM32H743

基于 **FK743M4-XIH6-V1.1（STM32H743XIH6）** 的 AS5047P ABI 编码器事件流监控/记录器。
自动触发记录 |RPM|>10 的转速变化事件（回溯 400 点 + 记录 500ms），完成后**先保存到芯片内部 Flash（掉电不丢），再备份到 FAT32 SD 卡（时间命名、不覆盖）**；PC 端工具可从 DUMP 或 SD 文件还原曲线。

- 固件目录：`firmware/abi_monitor_h743/`
- PC 工具：`pc/abi_monitor.py`（拷贝自 mcoder 工程，未修改）

---

## 1. 接线

| 信号 | MCU 引脚 | 说明 |
|------|----------|------|
| A 相 | PA5（TIM2_CH1） | 编码器 A，4X 计数 |
| B 相 | PA1（TIM2_CH2） | 编码器 B，4X 计数 |
| Index | PA4（EXTI4） | 每圈 Index 脉冲（触发/校准） |
| PWM | PA7（TIM3_CH2） | 电调油门 500Hz，占空比 0..1000‰ |
| TX | PA9（USART1_TX） | 串口 921600 8N1 |
| RX | PA10（USART1_RX） | |
| SD | PC8..PC12、PD2（SDMMC1） | 4bit 数据 + CMD |
| LED | PC13（低电平点亮） | 用户灯 |

> 地面场地供电：模块 3.3V，编码器/电调信号共地。电机动力电源接入前请勿长时间转电机。

## 2. 构建 / 烧录

```bash
cd "firmware/abi_monitor_h743"
mingw32-make -j8      # 编译（arm-none-eabi-gcc）
mingw32-make flash     # OpenOCD ST-Link 烧录 + 校验 + 软复位
```

- 串口：**COM 口 @ 921600**，`py -m pip install pyserial`
- 需要 `D:\arm-official` 工具链 + OpenOCD xpack（Makefile 内已配置）。

## 3. CLI 命令

| 命令 | 说明 |
|------|------|
| `HELP` | 帮助 |
| `ID` | 版本 & 波特率 |
| `CFG SHOW / SAVE / RESET / GEAR n d1..dn b1..b(n-1)` | 档位抽稀表（默认 3 档 div 1/2/4 @ 4000/8000rpm，可存 Flash） |
| `CFG POL 0\|1` | A/B 方向极性（0=计数反向取反（默认），1=正向直接计入） |
| `CAL STEPS` | 显示 Index→Index 实测一圈步数（EMA 平滑） |
| `CAL SET <n>` | 手动写一圈步数并保存 Flash（测速公式实时用） |
| `CAL RESET` | 恢复默认 4000 |
| `PWM 0..1000` | 测试电机占空比（‰，0 停机；Pm_Tick 斜坡靠近目标） |
| `RPM [ON/OFF]` | 实时转速回显 |
| `CNT / IDX` | 编码器计数 / Index 圈数 |
| `ARM / DISARM` | 预置自动触发（转速过阈 + 一整圈 → 回溯 400 点 + 记 500ms） |
| `SNAP` | snap 状态 state/count/ready |
| `FLASH` | 芯片保存状态（data/stored/current/saved） |
| `FSAVE` | 手动把当前 snap 写入芯片 Flash |
| `DUMP` | 芯片内记录按 PC 帧发串口（abi_monitor.py 可曲线还原） |
| `SD INIT / SAVE / LS / STAT` | SD 卡挂载 / 备份 / 列表 / 卡信息 |
| `SD RAW [w] sec` | 裸扇区诊断 |

1. 插好 SD 卡（FAT32），`SD INIT` 挂载；
2. `ARM` → 转电机（或 `PWM nnn`）→ 自动触发 DONE → **自动 Flash 保存 + SD 备份**（每次触发一条流程）；
3. `FLASH` 确认 saved=1、`SD LS` 看 `S<秒>.BIN`；
4. 掉电重启后 `FLASH` 仍显示 stored=n（芯片保存生效）；要取数时发 `DUMP`，PC 端收帧画曲线。

## 4. BIN v2 帧格式（PC 工具可解析）

SD 文件内容与 DUMP 串口帧字节完全一致：

```
[11 B] preamble      0xAA×10 + 0x55
[8 B ] 头部           magic(LE u32)=0xAB1C0002 | n(LE u16 点数) | hz(LE u16 占位=1000)
[n×16] 点阵           每点16B: t_us(u32) | counts(i64, 低32=c_lo,高32=0) | index(u32)
[4 B ] crc32(zlib, LE)  对点阵字节计算
```

`PC 端解析示例`（`pc/abi_monitor.py` 内已含）：`struct.unpack("<IHH", f[11:19])` → 点数；`struct.unpack_from("<IqI", pts, 16*i)` → 每点。

## 5. 0.5s 窗口截断说明（必读）

- 默认档位（1/2/4 @ 4000/8000 rpm）下记录点率 ≤ 26.7kHz：div=1@4000 约 21.3k 点/0.5s、div=2@8000 约 21.3k 点/0.5s、div=4@12000 约 12k 点/0.5s，**均 < 32768 上限，0.5s 窗口恒不截断**。
- **若手动把高速档 div 降到 1**（如 `CFG GEAR 3 1 4000 1 8000 1`），记录点率可超上限，窗口会被截短：
  事件点超 32768 即停，实际时长 = 32768 / 点率（例如 div=1@12000rpm ≈ 64k 点/s → 截到约 0.51s）。

## 6. 芯片 Flash 保存区域

- Bank2 **扇区 0..3（0x08100000，共 512KB）**，与配置扇区（0x081E0000）互不干扰。
- 每次 FSAVE 擦 4 扇区再整帧写入（含 magic/n/hz/crc32 校验头），**覆盖写最新一次触发记录**。
- Flash 编程按 H7 32B FLASHWORD、源缓冲对齐；DUMP 从 Flash 直读，无需 RAM 缓冲。

## 7. PC 工具使用

```bash
cd pc
python abi_monitor.py --usb      # 指定串口方式（默认优先蓝牙）
# 或仅需还原帧：python <(先 DUMP) → 保存为 .bin → 交给 curve_studio/abi_monitor 曲线
```

> 每次触发后板子的 DUMP 内容与 SD 文件字节一致，可交叉比对其一致性。

---

### 已知约束 / 提示

- 无 RTC，文件名取启动秒序（`get_fattime` 固定 0），仅同一启动内的 S<秒> 唯一。
- FatFS 使用 **R0.13**（例程版本），`FF_USE_LFN=0`（8.3 短名）。
- **SDMMC1 内部 DMA 直访内存，故禁用 DCache（只开 ICache）**——读写缓冲不一致的根因。改动涉及内存/DMA时勿重开 D-Cache 而不隔离缓冲。
- FATFS 缓冲需 32B对齐（diskio.c 已加中转），改动 buffer 布局勿破坏对齐。
- 配置区与记录区都在 Bank2：`CFG GEAR`/`FSAVE` 解锁擦写期间会短暂阻塞 Flash 读公共代码（几 ms~），属正常。