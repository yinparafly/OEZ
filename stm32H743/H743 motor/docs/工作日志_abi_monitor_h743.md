# abi_monitor_h743 工作日志（STM32H743 ABI 编码器事件流监控/记录器）

> 目的：按日期沉淀本项目的初始计划、过程发现、决策与依据，作为 STM32H7 项目经验基础。
> 项目：`D:\oezcon\stm32H743\H743 motor\firmware\abi_monitor_h743\`（固件）
> 计划：`docs\superpowers\plans\2026-08-06-stm32h743-abimotor-plan.md`（主计划）
> 接线：`docs\接线文档.md`
> 参考源：`2.参考例程\1.基础例程\1.LED闪烁`（Keil 模板）、`SDMMC-SD卡移植FatFSC`（SD 例程）

---

## 项目概述

在 FK743M4-XIH6-V1.1（STM32H743XIH6）开发板上复刻 DSP 的 UTO 方案：AS5047P ABI 编码器
（1000PPR → 4X=4000 计数/圈）事件流监控/记录器。BIN v2 16B 帧与原有 PC 工具字节级兼容
（magic 0xAB1C0002）。

- 设备：STM32H743XIH6（480MHz CM7，2MB Flash，512KB AXI SRAM）
- 外设分配：编码器 A=PA5(TIM2_CH1)、B=PA1(TIM2_CH2)、Index=PA4(EXTI4)；
  USART1=PA9/PA10 921600 8N1；LED=PC13（低电平点亮）；PWM=PA7(TIM3_CH2) 500Hz；
  SDMMC1=PC8~12+PD2（Task 5）
- 时钟链：HSE25 → SYSCLK480 → HCLK240 → APB1/2=120 → TIM5=240MHz（PSC=3 → 60MHz 刻度）
- 记录规格：SNAP_CAP 3400 点、RING_CAP 1200 点、回溯 400 点（计划值，Task 2/3 实施）
- 抽稀档位：3/4/5 档可配，默认 3 档 div 1/2/4 @ 4000/8000rpm，0.8s 窗口恒 ≤32k 点

---

## 工具链情况（2026-08-06 最终确定）

本机**无 Keil UV4**（全盘搜索失败），编译/烧录改用：

| 项 | 位置 | 说明 |
|----|------|------|
| GCC 编译器 | `D:\arm-official\arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi\bin\` | 13.2.1 **完整版（含 cc1.exe）** |
| （不可用） | `D:\arm-gcc\xpack-arm-none-eabi-gcc-13.2.1-1.1` | xpack 版缺 cc1.exe，CreateProcess 失败，弃用 |
| 构建驱动 | `mingw32-make`（`d:\TDM-GCC-64\bin\`） | 用 `Makefile`，-j8 并行编译 |
| 烧录 | `D:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe` | **v2.15.0**，SWD HOTPLUG 模式 |
| 备选烧录 | `D:\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe` | ST-Link V2 探测验证可用（SWD DPIDR 0x6ba02477），未实际烧录 |

### 关键命令
```bat
:: 编译（工程根目录）
mingw32-make -j8 all

:: 一键编译+烧录（build.bat）
build.bat flash
```

### GNU 工具链新增文件（Keil 工程之外）
- `gcc\startup_stm32h743xx.s`：GNU as 语法启动文件，向量表 166 项与 Keil 版逐项一致（自己数过）
- `gcc\stm32h743xx_flash.ld`：链接脚本（FLASH 2MB @0x08000000，RAM 512K @0x24000000，stack 0x400 heap 0x200 与 Keil 一致）
- `Makefile`：源文件清单与 uvprojx 完全一致（25 个 HAL + Core + App + Drivers/User）
- `build.bat`：编译 + STM32_Programmer_CLI 烧录（`-w bin 0x08000000 -v -rst`）

### 烧录接线（ST-Link V2，4 线）
P1-1 DIO(SWDIO/PA13) → SWDIO；P1-2 SWCLK → SWCLK；P1-3 GND；P1-4 5V（不接 RST、不接 3.3V）

### STM32CubeProgrammer CLI 注意
- 连续执行可能报 `ST-LINK error (DEV_USB_COMM_ERR)`，等 2~3 秒重试即可（测试过 3 次）
- 连接参数：`-c port=SWD mode=HOTPLUG`，识别 Device ID 0x450 = STM32H7xx，Flash 2MB

---

## 时间线

### 2026-08-06（晚）：Task 2 测速修复 + 油门特性自学习
- **测速 bug 修复（3 个，rpm 从溢出垃圾值 → 正确读数）**：
  1. TIM2 CNT 上电为垃圾值（曾读到 4294756902≈2³²-21万）→ `Abi_Init` 后 `__HAL_TIM_SET_COUNTER(&htim2, 0)` 清零起点
  2. EXTI4 Index 配了 `RISING_FALLING` → idx 每转 +2（Δcnt/Δidx≈2000 而非 4000）→ 改只上升沿
  3. 编码器 A/B 相序导致 TIM2 反向计数（正转 cnt 递减）→ UTO ISR 与 `Abi_GetCnt` 统一取反 `0u - CNT`，rpm 显示正数
- 修复后交叉验证：PWM 900 → rpm≈6000，idx 增速一致（Δcnt/Δidx≈4000，每转 4000 counts 确认）
- **rpm 合理性钳制**：±60000 rpm 超限截断（防单帧异常污染）
- **新增 `pc_tool/learn_ramp.py` 油门特性自学习**（8 档连续爬升，每档 2s 斜坡+1s 采样×3，超 10000rpm 停机，finally 必 PWM 0）
  - 首跑结果（空载）：
    ```
    125‰→1113rpm  250‰→3196  375‰→4571  500‰→5166
    625‰→5472     750‰→5555  875‰→5800  1000‰→6303
    启动油门 ≈ 125‰
    ```
  - 曲线形状：0~500‰ 近似线性（~10rpm/‰），500~1000‰ 趋于饱和（电调/电机特性）
- 输出：`pc_tool/calib/learn_20260806_224602.csv` + `learn_notes.md`（空载声明，载荷/电机/电调改变须重新学习）

### 2026-08-06（晚）：Task 3 事件记录管线（自动触发）
- **触发验证升级**（用户指示）：手动拧电机 → PWM 自动驱动。`pc_tool/snap_verify.py`：`PWM 0` → `ARM` → `PWM <n>` → 轮询 `SNAP` 至 state=3 → 验证点数
- **snap_bin.c/h**（新建 Task 3）：
  - ring 1024 点环形缓存（head/tail 单调 + 掩码），SNAP_CAP **31000**（512KB AXI SRAM 内 ring 16KB + bss 余量，计划 32768 超界改小）
  - 触发状态机 0=IDLE/1=ARM/2=REC/3=DONE；ARM 后 |rpm|>10 置 armed_moving，Index 转整圈 → 回溯 400 点 + 记 0.8s
  - `Snap_BuildBin` 打包 v2（magic 0xAB1C0002 + 16B 点）
  - CLI 新增 `ARM`/`DISARM`/`SNAP`
- **自动触发验证通过**：
  ```text
  PWM 700 → state 2→3, count=12042（回溯 1088 + 0.8s 窗口）验证通过
  PWM 500 → state 3, count=9849（点率随转速下降）         验证通过
  ```

---

### 2026-08-06：Task 1 骨架 + Task 1A PWM（编译/烧录/实机验证全部打通）
- 拷贝 `1.LED闪烁` 例程 → `firmware\abi_monitor_h743`（完整 Keil 工程 + 源码）
- 自建 `Drivers\User\Src\usart.c/h`（USART1 921600，中断收 FIFO，IRQ 直接在 usart.c，回调 `Cli_OnChar`）
- 自建 `App\app_config.c/h`（3/4/5 档抽稀表 + Bank2 扇区 7 = 0x081E0000 Flash 持久化，
  CRC16 查表校验，H7 32B FLASHWORD 编程）
- 自建 `App\app_cli.c/h`（HELP/ID/CFG SHOW/SAVE/RESET/GEAR + PWM）
- 自建 `App\app_pwm.c/h`（PA7/TIM3_CH2，500Hz：PSC=7 → 30MHz 计数率，ARR=59999，占空比 0..1000‰）
- 手动 Keil 工程 uvprojx 同步：Application/App 组 + usart + hal_uart + hal_tim + include 路径

### 编译过程发现并修复
1. `usart.c` 笔误 `&gp` 应为 `&gpio`（GCC 报错才发现，Keil 没跑过）
2. `app_config.c` CRC16 表原手工数据错乱（excess elements）→ 用脚本重新生成标准 256 项表替换
3. `stm32h7xx_hal_conf.h` 需开 `HAL_UART_MODULE_ENABLED` 和 `HAL_TIM_MODULE_ENABLED`
4. xpack GCC 缺 cc1.exe → 换 arm-official 完整版（`Makefile` 里 GCC_PREFIX 已改）

### 实机验证（2026-08-06 下午）
- **烧录成功**：`abi_monitor_h743.bin` 14.68KB → 0x08000000，verify + software reset 通过
- **固件运行验证**：上电后读 Flash 配置区 `0x081E0000` = `A5A5C0DE 04020103` ——
  Config_Load 检测无有效配置并写入默认档位成功（magic A5A5C0DE + gear_n=3, div 1/2/4），
  证明 CPU/Flash/CRC/系统时钟全部正常
- **板上 LED**：白灯=电源指示常亮（正常）；蓝灯=PC13 用户 LED 被 LED_Init() 点亮（低电平点亮，正常），
  同时证明固件已启动
- 待验证：串口 921600 交互（HELP/ID/CFG/PWM 命令）、PWM 波形输出（示波器/电机）

---

## 待办
- [ ] 串口验证 CLI（A9/A10 USB-TTL 转接线）
- [ ] Task 2：编码器 ABI 捕获（TIM2 边沿中断 + TIM5 64 位时间戳）
- [ ] Task 3：事件存储/回溯/抽稀档位切换
- [ ] Task 4：BIN v2 帧 + PC 工具联调
- [ ] Task 5：SD 卡存储（SDMMC1）
- [ ] 电机动力电源接入后才能做的：PWM 驱动电机 + 转速闭环验证（用户需在接线后提醒）

---

### 2026-08-07��Task 4 SD ���洢��SDMMC1 + FatFS �Զ����ݣ��ϰ���֤ͨ����
- ������������ FatFS��**R0.13**��ff.c/ff.h/ffconf.h/integer.h/diskio.h���� `firmware/abi_monitor_h743/FatFS`��ffconf��`FF_USE_LFN=0`��`FF_VOLUMES=1`��`FF_MAX_SS=512`��`FF_FS_TINY=1`
- ��д `FatFS/diskio.c` glue���������� sd_diskio/ff_gen_drv����HAL_SD ��ѯ��д��ClockDiv=10 �� 24MHz��`get_fattime()` ���� 0���� RTC��
- �Խ� `App/sd_card.c/h`��Sd_Init���ݵȣ�/Sd_Mount/Sd_SaveSnap/Sd_Ls/Sd_Raw/Sd_Stat
- **�ļ��������ǣ��û�Ҫ��**��`S%07lu.BIN` �뼶ʱ��� + `FA_CREATE_NEW`��FR_EXIST ��+1 ���� ��60���������Ǿ��ļ�
- д��֡ = BIN v2 ֡ + **crc32**��snap_bin.c ���� `Snap_Points()`/`Snap_Crc32()` ȫ 256 ������PC ���� `abi_monitor.py` ��ԭ��������SNAP_HZ=1000
- CLI��SD INIT/SAVE/LS/RAW [w]/STAT��main.c �������� + �Զ����ݣ��� `SD_CardMounted() && !g_snap_autosaved` һ���ԣ�
- Makefile �� hal_sd/hal_sd_ex/ll_sdmmc/FatFs Դ��������hal_conf �� `HAL_SD_MODULE_ENABLED`
### �弶�Ų�������ӣ������޸���
1. **��ѭ������**��Snap_IsReady��Sd_SaveSnap��δ���ؿ��������� HAL ��ʱ �� ���سɹ� + һ���Ա�־���Զ�����
2. **DCache �� SDMMC �ڲ� DMA ��һ��**��RAW ����ȫ 0���� **main.c ���� DCache**���� ICache��ע��ԭ��
3. **FatFS ����� 4B ����** �� SDMMC �ڲ� DMA ������FR_DISK_ERR���� diskio.c �� `uint32_t[128] aligned(8)` ��ת + memcpy ·��
### ʵ����֤��2026-08-07��
- �� 16GB FAT32��SDHC��type=1, blocks=31116288�������� COM21 @ 921600
- `pc_tool/sd_verify.py` ȫ�Զ���PWM600 ~1s ��ͣ����֤ state=3 �� SD SAVE �� LS��
- **�����save ret=0��S000018.BIN / S000020.BIN sz=101495 = 11+8+6342��16+4 ��ȷһ��**�����α��治���� ?��RAW д���� match ?
- ���� 4 ��ɣ�������Flash �־û����� + README���ƻ� Task 5/Step2��

---

## ����
- [ ] ������֤ CLI��A9/A10 USB-TTL ת���ߣ�
- [x] Task 2�������� ABI ����TIM2 �����ж� + TIM5 64 λʱ�����
- [x] Task 3���¼��洢/����/��ϡ��λ�л�
- [x] Task 4��SD ���洢��SDMMC1 + FatFS �Զ����ݣ�
- [ ] Task 5��BIN v2 ֡ + PC �������� / Flash �־û�����
- [ ] ���������Դ�����������ģ�PWM ������� + ת�ٱջ���֤���û����ڽ��ߺ����ѣ�

---

### 2026-08-07��Task 5 Flash оƬ���� + DUMP + README���ϰ���֤ͨ����
- �û�Ҫ��"оƬ����֮���ڿ��ﱸ��һ��"��snap �ȴ��ڲ� Flash�����粻�������ٱ��� SD
- �½� `App/flash_save.c/h`��Bank2 ���� 0..3��0x08100000 �� 512KB�������һ�μ�¼��
  32B ͷ {magic=SNAP_MAGIC_V2, n, hz=1000, crc32(zlib)} + n x SnapPt��H7 32B FLASHWORD ���
  ����̬ aligned(32) ���壩��FLASH_Dump �� Flash ֱ���ֿ鰴 PC ֡��preamble+<IHH+payload+crc32������ȫ�ٷ�
- usart.c/h ���� `Usart_Write`��ԭʼ�ֽڿ鷢�ͣ�DUMP �ã�
- CLI��FLASH��״̬��/ FSAVE���ֶ����棩/ DUMP��оƬ��¼ PC ֡��
- main.c �Զ������ **DONE ������**��g_done_prev����ÿ�δ������ �� Flash_SaveSnap() �ȡ�SD ���ݺ�
  ���޸��� g_snap_autosaved �� 1 ��������λ��ֻ��һ�α���� bug��
- ���� `pc/abi_monitor.py`��δ�ģ��� `H743 motor/pc/`
### ʵ����֤��2026-08-07��
- �Զ� DONE �� `Flash save ok: n=6273` + `SD save ok: Sxxxxx.BIN`��`FLASH` stored==current==snap count
- **DUMP ������֡��100436B��magic=AB1C0002 n=6273 hz=1000 crc_ok=True**
- **OpenOCD ��λ�� `FLASH` �� data=1 stored=6273**��оƬ������籣����?
- �Ų��¼��������"��������/��������Ӧ"����ʵΪ����ʱ OpenOCD `-c halt` ��� PC ��δ resume��
  оƬ��ͣס��`reset run` ���ָ���CLI ������ USART IRQ ��ִ�У�Flash ��дæµ�ڼ� FLASH ���Կ��ܱ�
  �ϲ�/�̵������Լ��ɣ��ǹ�������
- README��H743 motor/README.md���ɣ����߱�/����/CLI ȫ��/BIN v2 ֡��ʽ/0.5s ���ڽض�˵��/Flash ����/PC �����÷�

---

## ����
- [x] ������֤ CLI��A9/A10 USB-TTL ת���ߣ�
- [x] Task 2�������� ABI ����TIM2 �����ж� + TIM5 64 λʱ�����
- [x] Task 3���¼��洢/����/��ϡ��λ�л�
- [x] Task 4��SD ���洢��SDMMC1 + FatFS �Զ����ݣ�
- [x] Task 5��Flash оƬ���� + DUMP PC ֡ + README
- [ ] Task 6��ʵ��У׼��һȦ���� EMA/���ԣ�
- [ ] ���������Դ�����������ģ�PWM ������� + ת�ٱջ���֤���û����ڽ��ߺ����ѣ�
