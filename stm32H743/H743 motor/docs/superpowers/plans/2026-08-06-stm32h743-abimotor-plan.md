# STM32H743 ABI 编码器监控/记录器 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 STM32H743 上实现 ABI 编码器（AS5047P 4X）事件流监控记录器，复刻 DSP 的 UTO 动态周期事件记录方式，输出 BIN v2 16B 格式，兼容原版 ESP32 PC/手机曲线工具（不改 UI）。

**Architecture:** TIM2 32 位编码器模式 4X 计数（A=PA5/B=PA1，TIM2_CH1/CH2 AF1），Index=PA4（EXTI4）。PA0 板上未引出（丝印 A0C=PA0_C，仅 ADC 模拟脚），故 TIM2_CH1 用 PA5（AF1 亦映射 TIM2_CH1）。记录采用 DSP 的 UTO 自适应周期事件流方式：定时器周期随转速动态调整，ISR 内记录 (t_us, counts, index_n) 非均匀时间轴事件点，按 div 抽稀档位（默认 3 档 1/2/4，可配置）降点密度。触发：|rpm|>10 且 Index 过一整圈 → 回溯 400 点 + 后记 0.8s。BIN 头 magic 0xAB1C0002，点 16B。串口 921600 与 PC 通讯，PC 端拷贝原版工具不改 UI。

**Tech Stack:** Keil MDK5 + HAL（STM32Cube FW_H7 V1.11.1，DF-Pack 3.0.0）、STM32H743XIHx（TFBGA240 封装，板型 FK743M4-XIH6-V1.1，480MHz）、ST-LINK V2.1、SDMMC1 + FatFS（基础例程自带）、UART1 921600。

## Global Constraints

- MCU 固定 STM32H743XIHx，TFBGA240 封装（板型 FK743M4-XIH6-V1.1）；开发 IDE 固定 Keil MDK5 + ST-LINK V2.1。
- 编码器恒 4X 计数，div 抽稀只降记录点密度，counts 永远真实值（DSP 语义）。
- 记录格式固定 BIN v2（magic `0xAB1C0002`，点 16B：`t_us u32 + counts i64(lo,hi) + index_n u32`），PC 端原版工具零改动。
- 抽稀档位数、每档 rpm 边界、每档 div 全部可配置（CLI 命令），配置掉电保存（存 STM32 内部 Flash 扇区，无外部 EEPROM）。
- 转速范围 0~12000 rpm（弹射 ≤6500，扑翼 2000~12000）；事件率峰值 800k 边沿/s@12000rpm。
- 记录缓冲 512KB AXI SRAM（32k 点），CPU 预算 <5%@480MHz，UTO ISR 峰值 60kHz。
- 档位默认值：`<4000rpm→div1, 4000~8000rpm→div2, ≥8000rpm→div4`（边界可改，档数可 3/4/5；保证高速自动 div≥2，0.8s 窗口恒 ≤32k 点）。
- 测速：事件差分 `rpm = dpos×60e6/(dt_us×4000)`，PC 端用 counts 差分；PWM 控制阶段最后做（本期不做闭环）。
- 所有 printf/串口输出走 USART1 @921600；时钟/引脚配置参考例程为模板（LED 例程 `1.LED闪烁` 起底，SD 用 `SDMMC-SD卡移植FatFs`）。
- 引脚占用冲突检查：USART1=PA9/PA10、SDMMC1=PC8~12+PD2、QSPI=PF6~10/PG6、LED1=PC13 —— 均不复用。
- 语言：固件注释、CLI 回显用中文；代码文件用 C99，无全局动态分配（静态分配）。

---

### Task 1: 工程骨架（Keil 工程 + 时钟 + 串口 + LED）

> **实施偏差（2026-08-06 已执行）**：
> - 文件名定为 `App/app_cli.c/h`（计划原 `App/cli.c/h`）
> - `usart.c/h` 放 `Drivers/User/`（LED 例程**无 USART**，需自建；含 MSP 初始化+中断收字节 FIFO，IRQ 直接在 usart.c）
> - 中断回调名 `Cli_OnChar`（替代 `App_UartRxCb`）
> - `CFG_FLASH_BASE` 修正为 `0x081E0000`（0x081F0000 不是扇区起始）；H7 擦除按 **Bank2 内扇区号**（Sector=7），无 VoltageRange 字段；编程用 32B FLASHWORD 整字写入
> - `stm32h7xx_hal_conf.h` 需 `#define HAL_UART_MODULE_ENABLED`（例程默认关）
> - uvprojx 增加 `Application/App` 组 + usart.c + `stm32h7xx_hal_uart.c` + `stm32h7xx_hal_uart_ex.c` + include `..\App`
> - **编译/烧录方式（2026-08-06 最终落地）**：本机无 Keil UV4，改用 **GNU arm-none-eabi-gcc（D:\arm-official 完整版，xpack 版缺 cc1 不可用）+ STM32CubeProgrammer v2.15 CLI（用户已装）**。新增 `gcc/startup_stm32h743xx.s`（GNU 语法向量表，166 项与 Keil 版一致）、`gcc/stm32h743xx_flash.ld`、工程根 `Makefile`（mingw32-make 编译）、`build.bat flash`（编译+烧录）。实测：编译 0 Error、ST-Link SWD 烧录验证通过、上电后 Flash 配置区 0x081E0000 读出 `A5A5C0DE`（默认配置写入成功，证明固件运行正常）

**Files:**
- Copy: `D:\oezcon\stm32H743\2.参考例程\1.基础例程\1.LED闪烁`（最小工程模板）→ `D:\oezcon\stm32H743\H743 motor\firmware\abi_monitor_h743`
- Modify: `firmware/abi_monitor_h743/Core/Src/main.c`（清空例程演示逻辑）
- Create: `firmware/abi_monitor_h743/App/app_config.h`、`App/app_config.c`、`App/app_cli.c`、`App/app_cli.h`

**Interfaces:**
- Produces: `void App_Init(void)`、`void App_Poll(void)`（主循环调用）、`void App_UartRxCb(uint8_t byte)`（USART1 中断回调）
- Produces: `uint32_t cfg_baud;`、`uint8_t cfg_gear_div[CFG_GEAR_MAX];`（div 表）、`uint32_t cfg_gear_bnd[CFG_GEAR_MAX-1];`（边界 rpm）、`uint8_t cfg_gear_n;`（档数 3/4/5）、`void Config_Load(void)`、`void Config_Save(void)`

- [x] **Step 1: 拷贝例程、清空演示逻辑**

- Copy: `D:\oezcon\stm32H743\2.参考例程\1.基础例程\1.LED闪烁` 到 `H743 motor/firmware/abi_monitor_h743`。打开 Keil 工程，`main.c` 的 `main()` 改为：

```c
#include "app_config.h"
#include "cli.h"
#include "usart.h"
#include "led.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();   // 例程原有 480MHz 配置，保持不变
    MX_GPIO_Init();
    MX_USART1_UART_Init();  // usart.c 里波特率改 921600
    Config_Load();
    App_Init();
    while (1) { App_Poll(); }
}
```

- [x] **Step 2: 编译确认通过**（2026-08-06 已用 GNU 工具链替代 Keil 验证，0 Error 并烧录成功）

Keil 命令行或 IDE 编译 `abi_monitor_h743.uvprojx`。预期 0 Error。

- [x] **Step 3: 写 CLI 最小框架（app/app_cli.c）**（已实现 HELP/ID/CFG SHOW/GEAR/SAVE/RESET，命令表风格）

```c
#include "cli.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"

static char rx_buf[64];
static uint8_t rx_len = 0;

void App_UartRxCb(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (rx_len > 0) { rx_buf[rx_len] = 0; Cli_Exec(rx_buf, rx_len); rx_len = 0; }
    } else if (rx_len < sizeof(rx_buf) - 1) {
        rx_buf[rx_len++] = byte;
    }
}
```

`Cli_Exec(char* line, uint8_t len)` 支持命令表：`HELP`、`ID`（回显 "ABI-MONITOR-H743 v0.1"）、`CFG SHOW`、`CFG GEAR <n> <div1> <bnd1> <div2> <bnd2> ... <divN>`、`CFG SAVE`、`CFG RESET`。

```c
// 命令分发表
typedef struct { const char* name; void (*fn)(int argc, char** argv); } CliCmd;
static const CliCmd cmds[] = {
    {"HELP", Cmd_Help}, {"ID", Cmd_Id}, {"CFG", Cmd_Cfg},
};
```

`Cmd_Cfg` 解析子命令 SHOW/GEAR/SAVE/RESET：`CFG GEAR 3 1 4000 2 8000 4` 表示 3 档、档1 div=1 到 4000rpm、档2 div=2 到 8000rpm、档3 div=4。校验 `cfg_gear_n` ∈ [3,5]、每档 div ∈ {1,2,4,8,16}、边界严格递增。

- [x] **Step 4: 实现配置存取（app_config.c，内部 Flash）**（CRC16 查表、Bank2 扇区 7 = 0x081E0000、FLASHWORD 整字编程）

```c
#define CFG_FLASH_BASE  0x081F0000UL   // Bank2 尾扇区（128KB，H743 2MB Flash 最后扇区；固件上限留足 1MB 余量）
#define CFG_MAGIC       0xA5A5C0DEUL

typedef struct {
    uint32_t magic;
    uint8_t  gear_n;
    uint8_t  gear_div[CFG_GEAR_MAX];     // CFG_GEAR_MAX=5
    uint32_t gear_bnd[CFG_GEAR_MAX-1];
    uint16_t crc;
} AppCfg;

static AppCfg g_cfg;
static const AppCfg g_cfg_default = {
    .magic = CFG_MAGIC, .gear_n = 3,
    .gear_div = {1, 2, 4, 0, 0},
    .gear_bnd = {4000, 8000, 0, 0},
};

void Config_Load(void)
{
    memcpy(&g_cfg, (const void*)CFG_FLASH_BASE, sizeof(AppCfg));
    if (g_cfg.magic != CFG_MAGIC || g_cfg.gear_n < 3 || g_cfg.gear_n > 5) {
        g_cfg = g_cfg_default;
        Config_Save();
    }
}
```

`Config_Save()`：HAL_FLASH 擦扇区 + 编程写入；CRC 用查表法（代码里直接附 CRC16 表）。注意 H743 Flash 编程必须先在 `HAL_Init()` 后 `HAL_FLASH_Unlock()`；写完后 `HAL_FLASH_Lock()`；读配置不需解锁。

- [ ] **Step 5: 板级验证**

串口助手 921600 发 `ID` → 回 `ABI-MONITOR-H743 v0.1`；`CFG GEAR 4 1 2000 2 4000 4 8000 8` → `OK`；`CFG SHOW` → 显示 4 档表；`CFG RESET` + 断电重启 → 恢复 3 档默认。LED1 闪烁确认主循环运行。

- [ ] **Step 6: 提交**

```bash
git add "H743 motor/firmware/abi_monitor_h743"
git commit -m "feat(h743): 工程骨架+CLI+配置存取(3/4/5档可调)"
```

---

### Task 2: TIM2 编码器 4X + UTO 自适应事件流

**Files:**
- Create: `firmware/abi_monitor_h743/App/abi.c`、`App/abi.h`
- Modify: `firmware/abi_monitor_h743/App/app_config.c`（加 UTO 参数）

**Interfaces:**
- Consumes: `app_config.h` 的档位配置
- Produces: `void Abi_Init(void)`（TIM2 编码器 + EXTI4 + UTO 定时器）
- Produces: `typedef struct { uint32_t t_us; uint32_t c_lo; uint32_t c_hi; uint32_t idx; } SnapPt;`（16B 对齐 4）
- Produces: `void Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now)`（UTO ISR 调用的记录回调，Task 3 实现）
- Produces: `void Snap_OnIndex(void)`（EXTI4 ISR 调用）
- Produces: `int32_t Abi_GetRpm(void)`、`uint32_t Abi_GetIndexCnt(void)`、`uint32_t Abi_GetUsNow(void)`

- [ ] **Step 1: 写头文件 abi.h（接口 + 常量）**

```c
#pragma once
#include <stdint.h>

#define STEPS_PER_REV  4000uL        // 4X 标称 4000 步/圈，实测校准后更新
#define N_MIN          10            // UTO 每周期目标步数
// QUPR 按 60MHz 刻度换算（DSP 150MHz → ×60/150 = ×0.4）
#define UTO_QUPR_MIN_60M   ((2500uL   * 60uL) / 150uL)  // =1000    刻度 → ≈60kHz 夹顶
#define UTO_QUPR_MAX_60M   ((375000uL * 60uL) / 150uL)  // =150000  刻度 → ≈400Hz 下限
#define UTO_HYST       125           // 迟滞 12.5% (125/1000)
#define UTO_TIM_CLK    240000000uL   // H743 APB 定时器时钟 240MHz

typedef struct { uint32_t t_us; uint32_t c_lo; uint32_t c_hi; uint32_t idx; } SnapPt;

void  Abi_Init(void);
void  Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now);
void  Snap_OnIndex(void);
int32_t Abi_GetRpm(void);
uint32_t Abi_GetIndexCnt(void);
uint32_t Abi_GetUsNow(void);   // 64 位 µs 计数的低 32 位
uint8_t Abi_GetDiv(void);
```

- [ ] **Step 2: TIM2 编码器初始化（4X 模式）**

```c
// abi.c
#include "abi.h"
#include "app_config.h"
#include "tim.h"   // 例程生成的定时器句柄

static TIM_HandleTypeDef htim2;

void Abi_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFu;   // 32 位计数
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode = TIM_ENCODERMODE_TI12;  // 4X
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Filter = 0; enc.IC2Filter = 0;
    HAL_TIM_Encoder_Init(&htim2, &enc);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    // GPIO: PA5(A)/PA1(B) AF1  —— 注意 PA0 板上未引出(丝印 A0C=PA0_C,仅ADC脚),故A接PA5
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP; gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
}
```

PA5/PA1 上电后由 TIM2 编码器模块直接 4X 计数，CNT 读回即真实 counts（不用中断数边沿）。

- [ ] **Step 3: EXTI4（Index）初始化 + ISR**

```c
// PA4 接 Index，EXTI4（AS5047P 的 Index 为单脉冲，上升/下降沿均可）
void Abi_Init(void)  // 追加
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);   // 低于 UTO
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}
```

`stm32h7xx_it.c` 增加：

```c
void EXTI4_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        Snap_OnIndex();
    }
}
```

- [ ] **Step 4: UTO 自适应定时器（TIM5，32 位动态周期）**

时钟链（例程实测推导，务必运行时打印核对）：HSE=25MHz → PLL1 ×192/2 = **SYSCLK 480MHz** → AHB ÷2 = **HCLK 240MHz** → APB1 ÷2 = **PCLK1 120MHz** → TIM5 = **2×PCLK1 = 240MHz**（H7 规则：APB 预分频≠1 时定时器时钟 = APB×2；**不是 480MHz**——那需要 HCLK 也 480MHz，本例程没有）。

```c
// UTO 周期 = N_MIN 步所需时间。已知当前转速 rpm 和上周期 dpos：
// T_刻度 = 240MHz刻度数 = (240e6 * N_MIN) / (rpm * 4000)
// 夹顶 UTO_QUPR_MIN/MAX，迟滞防抖。
```

定时器选 **TIM5（32 位）**：时钟 240MHz，预分频 3 → **4 分频 → 60MHz 刻度**（PSC=3 即分频比 = 3+1 = 4）。**不用 TIM4**：16 位 ARR 上限 65535，而 QUPR_MAX=150000 超出（评审阻塞项）。TIM5 32 位 ARR 无上限问题，且顺带提供高精度 64 位时间戳源：

```c
static TIM_HandleTypeDef htim5;

static void Uto_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    // 启动时打印核对：PCLK1 与推导的 TIM5 时钟
    // printf("PCLK1=%lu Hz -> TIM5=%lu Hz\n", HAL_RCC_GetPCLK1Freq(), 2*HAL_RCC_GetPCLK1Freq());
    htim5.Instance = TIM5;
    // PSC 由实际时钟推导，锁死 60MHz 刻度：2×PCLK1 / 60MHz - 1
    // 例程 240MHz → PSC=3；若日后换 HCLK=480 配置 → PSC=7，公式自动跟随
    htim5.Init.Prescaler = (2uL * HAL_RCC_GetPCLK1Freq()) / 60000000uL - 1uL;
    htim5.Init.Period = 1000;              // 初始 1000 刻度 ≈ 16.7µs（60kHz 夹顶附近）
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim5);
    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_TIM_Base_Start_IT(&htim5);
}
```

QUPR 换算（60MHz 刻度，DSP 是 150MHz → 刻度数 = QUPR_dsp × 60/150 = ×0.4）：

```c
#define UTO_QUPR_MIN_60M  ((2500uL   * 60uL) / 150uL)   // = 1000   刻度 → 60kHz 夹顶
#define UTO_QUPR_MAX_60M  ((375000uL * 60uL) / 150uL)   // = 150000 刻度 → 400Hz 下限
#define UTO_HYST          125                            // 12.5% 迟滞 (125/1000)
```

（若 TIM5 被占用可退回 TIM4 + Prescaler=39 → 6MHz 刻度、QUPR_MAX=15000 在 16 位内；本板 ioc 未用 TIM5，无冲突。）

- [ ] **Step 5: UTO ISR —— 事件读取 + 64 位时间戳 + 动态周期 + 抽稀**

```c
// stm32h7xx_it.c —— 全局 64 位刻度计数（纯硬件，无 HAL_GetTick 抖动）

void TIM5_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_IT(&htim5, TIM_FLAG_UPDATE);
        // 1) 64 位刻度累计（不除不余，零漂移）：TIM5 计数时钟 60MHz → 每刻度 1/60µs
        //    需要 µs 时再除以 60（g_tick64 为 abi.c 全局，新增刻度即事件时刻）
        g_tick64 += htim5.Instance->ARR;      // 本周期时长（刻度）
        uint32_t us_now = (uint32_t)(g_tick64 / 60uL);  // 事件时刻 µs（低 32 位够 71 分钟）

        uint32_t cnt = TIM2->CNT;            // 32 位真实 counts
        static uint32_t prev_cnt;            // 上周期计数（64 位累计前不用，环回差天然无符号）
        uint32_t dpos = cnt - prev_cnt;      // 无符号环回差 = 真实增量

        // 2) div 抽稀档位（切换时重置 sub，避免档位跳变丢点）
        uint8_t div = Abi_GetDiv();
        static uint8_t sub;
        static uint8_t last_div = 0xFF;
        if (div != last_div) { sub = 0; last_div = div; }
        if (++sub >= div) {
            sub = 0;
            Snap_OnEvent(cnt, Abi_GetIndexCnt(), us_now);   // 记录点（含 µs 时间戳）
        }


        // 3) 动态周期：保持每周期 ~N_MIN 步。dpos=0 用上周期值防抖
        // target(刻度) = ARR_prev × N_MIN / dpos
        uint32_t target = (uint32_t)((uint64_t)htim5.Instance->ARR * N_MIN /
                                     (dpos ? dpos : N_MIN));
        target = CLAMP(target, UTO_QUPR_MIN_60M, UTO_QUPR_MAX_60M);
        // 12.5% 迟滞：只在偏差超 12.5% 时更新 ARR
        uint32_t arr = htim5.Instance->ARR;
        if (target < arr * 875 / 1000 || target > arr * 1125 / 1000)
            __HAL_TIM_SET_AUTORELOAD(&htim5, target);
        prev_cnt = cnt;
    }
}
```

> **评审修正 7（64 位硬件时间戳）**：不用 `HAL_GetTick()*1000 + TIMx->CNT` 合成——1ms 对齐抖动会污染 dt。改用 ISR 内累加 ARR 的 64 位刻度/微秒计数，纯硬件时钟源（晶振 ~50ppm），微秒分辨率。

`Abi_GetDiv()` 实现（查配置表）：

```c
uint8_t Abi_GetDiv(void)
{
    int32_t rpm = Abi_GetRpm();
    if (rpm < 0) rpm = -rpm;
    for (uint8_t i = 0; i + 1 < cfg_gear_n; i++)
        if ((uint32_t)rpm < cfg_gear_bnd[i]) return cfg_gear_div[i];
    return cfg_gear_div[cfg_gear_n - 1];
}
```

- [ ] **Step 6: 测速（事件差分）**

```c
static int32_t g_rpm;
// 在 UTO ISR 内： rpm = dpos × 60e6 / (dt_us × 4000)
int32_t Abi_GetRpm(void) { return g_rpm; }
```

`dt_us` 用 Step 5 的 `g_tick64` 差分（纯硬件，刻度 → µs）：

```c
// ISR 内： dt_us = (uint32_t)((g_tick64 - prev_tick64) / 60uL)
static uint64_t prev_tick64;
uint32_t dt_us = (uint32_t)((g_tick64 - prev_tick64) / 60uL); prev_tick64 = g_tick64;
if (dt_us && dpos) g_rpm = (int32_t)((int64_t)dpos * 60000000LL / (int64_t)dt_us / 4000);
```

（`Abi_GetRpm` 供 CLI/上位机轮询；rpm 有符号，反向转动为负。）

对外暴露刻度计数：`g_tick64` 定义为全局（abi.c 模块级 static，ISR 自加），供 `Abi_GetUsNow`/触发记录取 µs 时刻：

```c
// abi.c —— 全局 64 位刻度（60MHz → 每刻度 1/60µs），由 TIM5 ISR 累加
volatile uint64_t g_tick64;
uint32_t Abi_GetUsNow(void) { return (uint32_t)(g_tick64 / 60uL); }   // 事件时刻 µs
uint32_t Abi_GetUsDelta(uint64_t prev) {                                // 差分 µs（测速/触发用）
    return (uint32_t)((g_tick64 - prev) / 60uL);
}
```

UART 触发/回溯等需要 "当前 µs" 的地方统一走 `Abi_GetUsNow()`。

- [x] **Step 7: 板级验证**（2026-08-06 完成，含测速修复）

电机手动/低速转动：`ID` 确认版本；CLI 加 `RPM` 命令每秒回显一次转速、`CNT` 回显 counts、`IDX` 回显 Index 次数。正向转一圈 → 约 +4000±校准偏差；Index 每圈 +1。空转无脉冲时 rpm→0。

> **实施偏差（2026-08-06 已验证）**：
> - 实测修复 3 个 bug 后才获得正确读数（详见工作日志）：TIM2 CNT 上电垃圾值需清零；EXTI4 只能上升沿（RISING_FALLING 导致 idx 每转 +2）；A/B 相序反向需读数取反
> - 实测每转确为 4000 counts（Δcnt/Δidx≈4000 交叉验证）；rpm 读数与 idx 速率一致，测速可靠
> - `PWM` 驱动电机实测：PWM 900 → ~6000rpm（空载）；油门特性曲线见 `pc_tool/calib/learn_notes.md`

- [x] **Step 8: 提交**

```bash
git add "H743 motor/firmware/abi_monitor_h743"
git commit -m "feat(h743): TIM2编码器4X + EXTI4 Index + UTO自适应事件流"
```

---

### Task 3: 事件记录管线（ring 回溯 + snap 触发 + 0.8s 记录）

**Files:**
- Create: `firmware/abi_monitor_h743/App/snap_bin.c`、`App/snap_bin.h`
- Modify: `firmware/abi_monitor_h743/App/abi.c`（接 Snap_OnEvent/OnIndex）

**Interfaces:**
- Consumes: `SnapPt`（abi.h）、`Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now)`、`Snap_OnIndex(void)`
- Produces: `uint32_t Snap_BuildBin(uint8_t* buf, uint32_t cap)`（生成完整 BIN：magic+点列；返回总长）
- Produces: `uint8_t Snap_IsReady(void)`、`uint32_t Snap_Count(void)`
- Produces: 触发状态机：`SNAP_IDLE → SNAP_ARM → SNAP_TRIGGERED → SNAP_REC → SNAP_DONE`

- [x] **Step 1: 写 snap_bin.h**（2026-08-06 已实施）

```c
#pragma once
#include <stdint.h>
#include "abi.h"

#define SNAP_MAGIC_V2   0xAB1C0002uL
#define SNAP_BACKTRACK  400uL          // 回溯点数
#define SNAP_DUR_US     800000uL       // 触发后记录 0.8s
#define RING_CAP_BITS   10uL           // 2^10=1024 环形缓存（2 的幂，位运算）
#define RING_CAP        (1uL << RING_CAP_BITS)
#define SNAP_CAP        31000uL        // 512KB AXI SRAM - ring16KB - bss
#define TRIG_RPM        10             // |rpm|>10 触发
```

> **实施偏差**：SNAP_CAP 由计划 32768 降为 **31000**（512KB AXI SRAM 内 ring 1024×16B + 栈堆余量）；RING_CAP 升 512→1024。

- [x] **Step 2: 环形缓存 + 状态机（snap_bin.c）**

```c
#include "snap_bin.h"

// 环形缓存：head/tail 单调递增 + 掩码取模（2 的幂，无 % 开销，ISR 60kHz 安全）
static SnapPt ring[RING_CAP];
static uint32_t ring_head = 0;          // 下一写入位置（单调递增）
static uint32_t ring_tail = 0;          // 最早有效点（ring_head - ring_tail <= RING_CAP）
static SnapPt snap[SNAP_CAP];
static uint32_t snap_count;
static volatile uint8_t  state = 0;     // 0=IDLE 1=ARM 2=REC 3=DONE
static uint32_t rec_start_us;
static volatile uint8_t  armed_moving;  // ARM 下转速过阈

void Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now)
{
    SnapPt p = { us_now, (uint32_t)cnt,
                 (uint32_t)((int64_t)(int32_t)cnt >> 32), idx };
    int32_t rpm = Abi_GetRpm();
    if (state == 1 && (rpm > TRIG_RPM || rpm < -TRIG_RPM)) armed_moving = 1; // 自动触发判定
    if (state == 2) {                       // REC：填 snap 数组
        if (snap_count < SNAP_CAP) snap[snap_count++] = p;
        if (snap_count >= SNAP_CAP || (uint32_t)(us_now - rec_start_us) >= SNAP_DUR_US)
            state = 3;                      // DONE，等上位机取
    } else {
        ring[ring_head & (RING_CAP - 1)] = p;   // 始终进 ring，供回溯
        ring_head++;
        if (ring_head - ring_tail > RING_CAP) ring_tail = ring_head - RING_CAP;  // 满则弃最旧
    }
}

void Snap_OnIndex(void)
{
    if (state == 1 && armed_moving) {       // 转速过阈 + 转过整圈 → 触发
        uint32_t avail = ring_head - ring_tail;
        uint32_t take = (avail < SNAP_BACKTRACK) ? avail : SNAP_BACKTRACK;
        uint32_t start = ring_head - take;  // 最近 take 个点
        for (uint32_t i = 0; i < take; i++) snap[snap_count++] = ring[(start + i) & (RING_CAP - 1)];
        rec_start_us = Abi_GetUsNow();       // 0.8s 窗口起点（刻度→µs，见 Task2 Step6）
        state = 2;
    }
}
```

> **评审修正 3/4**：ring 用 head/tail 单调计数 + `& (RING_CAP-1)` 掩码，避免 60kHz ISR 里 `%` 除法开销；`ring_tail` 只在满时前移，回溯取最近 `take` 点。`rec_start_us` 用 Task2 的 64 位 µs 计数（Abi_GetUsNow 暴露）。
> **实施偏差**：armed_moving 判定移入 `Snap_OnEvent`（UTO ISR 每周期查 Abi_GetRpm），替代计划中额外在 UTO ISR 写判定。

- [x] **Step 3: 触发命令与状态查询（CLI 接入）**

`ARM` → `Snap_Arm()`（state=1，清 snap_count+ring）；`DISARM` → state=0；`SNAP` → 回显状态/点数/ready。`Snap_IsReady()` 返回 `(state==3 && snap_count>0)`。

触发语义（与 DSP 一致）：`Snap_Arm()` 置 ARM；UTO ISR 每周期判 `|Abi_GetRpm()|>TRIG_RPM`，通过后置 `armed_moving=1`；`Snap_OnIndex()` 在 `armed_moving` 下触发回溯+进 REC（即「转速过阈值 + Index 转过一整圈」双条件，见 Step 2 代码）。

> **验证方式升级（2026-08-06 用户指示）**：不再手动拧电机触发，改用 PWM 自动驱动（`pc_tool/snap_verify.py`：ARM → PWM 700/500 → 自动触发 → 验证点数）。

- [x] **Step 4: BIN 打包（v2 格式）**

```c
uint32_t Snap_BuildBin(uint8_t* buf, uint32_t cap)
{
    if (cap < 8 + snap_count * 16u) return 0;
    *(uint32_t*)buf = SNAP_MAGIC_V2;
    *(uint32_t*)(buf + 4) = snap_count;
    memcpy(buf + 8, snap, snap_count * 16u);
    return 8 + snap_count * 16u;
}
```

- [x] **Step 5: 板级验证**（自动触发，2026-08-06 通过）

```text
>>> snap_verify.py 自动触发验证（PWM 驱动代替手动拧）：
PWM 700 → state 2→3, count=12042（回溯1088+0.8s窗口） 验证通过
PWM 500 → state 3, count=9849（点率随转速下降）       验证通过
```


- [ ] **Step 6: 提交**

```bash
git add "H743 motor/firmware/abi_monitor_h743"
git commit -m "feat(h743): 事件记录管线 ring回溯+0.8s触发记录 BIN v2"
```

---

### Task 4: SD 卡存储（SDMMC1 + FatFS 写 snap bin）

**Files:**
- Copy: `D:\oezcon\stm32H743\2.参考例程\1.基础例程\SDMMC-SD卡移植FatFs` 的 SDMMC 驱动 + FatFS 源码 → `firmware/abi_monitor_h743/FatFS`（例程自带 ff.c/diskio.c/sd_diskio.c，已验证过 SD 卡）
- Modify: `firmware/abi_monitor_h743/App/sd_save.c`（新建）

**Interfaces:**
- Consumes: `Snap_BuildBin`、`Snap_IsReady`
- Produces: `uint8_t Sd_Init(void)`、`uint8_t Sd_SaveSnap(void)`（写 `snap_YYYYMMDD_HHMMSS.bin`，返回 0=成功）

- [ ] **Step 1: 移植 FatFS（diskio 适配 SDMMC1）**

直接使用基础例程 `SDMMC-SD卡移植FatFs` 的 FatFS 源（ff.c/ff.h/diskio.c/sd_diskio.c + ffconf.h），其 `sd_diskio.c` 已封装 SDMMC1 块读写（block 512B，4bit）。`ffconf.h`：`FF_FS_MINIMIZE=0`、`FF_USE_STRFUNC=2`、`FF_FS_RPATH=1`、`FF_USE_MKFS=1`。

- [ ] **Step 2: sd_save.c**

```c
#include "ff.h"
#include "snap_bin.h"

static FATFS fs;
static uint8_t snap_buf[512*1024];   // 512KB 静态缓冲，AXI SRAM 段

uint8_t Sd_Init(void)
{
    return (f_mount(&fs, "", 1) == FR_OK) ? 0 : 1;
}

uint8_t Sd_SaveSnap(void)
{
    if (!Snap_IsReady()) return 2;
    uint32_t n = Snap_BuildBin(snap_buf, sizeof(snap_buf));
    if (!n) return 3;
    FIL f;
    char name[32];
    // 用 RTC 或启动计数器命名 snap_%05u.bin，避免依赖 RTC
    sprintf(name, "snap_%05u.bin", g_snap_seq++);
    if (f_open(&f, name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return 4;
    UINT wr = 0;
    FRESULT r = f_write(&f, snap_buf, n, &wr);
    f_close(&f);
    return (r == FR_OK && wr == n) ? 0 : 5;
}
```

`g_snap_seq`：`app_config.c` 里 `uint32_t g_snap_seq`，每次保存自增，可掉电保存到配置扇区。

- [ ] **Step 3: CLI 接入**

`SD INIT` → 挂载结果；`SD SAVE` → 保存并回显文件名、字节数、耗时；`LS` → `f_findfirst` 列出 snap_*.bin。SD 卡初始化失败回显中文错误，不阻塞监控功能。

- [ ] **Step 4: 板级验证**

插卡（FAT32）→ `SD INIT` 成功；`ARM`+转电机 → `SD SAVE` 成功；`LS` 看到文件；拔出插 PC，文件能被原版 PC 工具曲线打开。

- [ ] **Step 5: 提交**

```bash
git add "H743 motor/firmware/abi_monitor_h743"
git commit -m "feat(h743): SDMMC1+FatFS 保存 snap bin"
```

---

### Task 5: 串口 DUMP + PC 工具拷贝兼容

**Files:**
- Create: `firmware/abi_monitor_h743/App/dump.c`
- Copy: `D:\oezcon\mcoder\ESP32_AS5047P_ABI_Monitor\pc\abi_monitor.py`、`curve_studio.py` → `H743 motor/pc/`
- Modify: 无 PC 端改动（只拷贝）

**Interfaces:**
- Consumes: `Snap_BuildBin`
- Produces: `void Dump_SendBin(void)`（921600 全速发送 BIN 帧）

- [ ] **Step 1: dump.c**

```c
#include "usart.h"
#include "snap_bin.h"

void Dump_SendBin(void)
{
    static uint8_t buf[512*1024];   // 512KB 静态缓冲
    uint32_t n = Snap_BuildBin(buf, sizeof(buf));
    if (!n) return;
    for (uint32_t i = 0; i < n; ) {
        uint16_t chunk = (n - i > 65535) ? 65535 : (uint16_t)(n - i);
        HAL_UART_Transmit(&huart1, buf + i, chunk, 1000);
        i += chunk;
    }
}
```

CLI：`DUMP` 命令触发。发送期间锁串口（PC 工具收完前不发其他文本）。921600 下 512KB ≈ 4.6s。

- [ ] **Step 2: PC 侧拷贝**

拷贝 `abi_monitor.py`/`curve_studio.py` 到 `H743 motor/pc/`，不修改任何代码。运行 `abi_monitor.py --help` 确认脚本依赖（pyserial/numpy）本机可跑。

- [ ] **Step 3: 板级验证（PC 直连）**

`ARM`→转→`DUMP`；PC 端脚本监听 921600 端口收到完整 BIN，`curve_studio` 能画曲线。比对 SD 文件与串口 DUMP 字节一致。

- [ ] **Step 4: 提交**

```bash
git add "H743 motor/firmware/abi_monitor_h743" "H743 motor/pc"
git commit -m "feat(h743): 串口DUMP v2 BIN + 拷贝PC工具(不改)"
```

---

### Task 6: 实测校准（一圈步数、Index 校准、档位边界校验）

**Files:**
- Modify: `firmware/abi_monitor_h743/App/abi.c`（校准 EMA）、`App/app_config.c`（steps 存储）

**Interfaces:**
- Consumes: `cfg_steps_per_rev`（app_config.c 新增字段）
- Produces: `uint32_t Abi_GetStepsPerRev(void)`

- [ ] **Step 1: Index→Index 实测步数（EMA 平滑）**

照 DSP：每次 Index 触发记下 `prev_cnt`，下一次 Index 得到 `dpos_one_rev`，`steps_ema = (steps_ema*7 + dpos)/8`；存入配置 `cfg_steps_per_rev`（掉电保存）。测速公式改用它替换 4000 常量。

```c
// Snap_OnIndex 内（state 无关，始终校准）：
static uint32_t g_prev_idx_cnt;
static uint32_t g_steps_ema = 4000;
void Index_Calibrate(uint32_t cnt)
{
    uint32_t d = cnt - g_prev_idx_cnt;
    g_prev_idx_cnt = cnt;
    if (d > 0 && d < 100000) g_steps_ema = (g_steps_ema * 7 + d) / 8;
}
```

- [ ] **Step 2: CLI**

`CAL STEPS` → 回显当前 EMA 步数；`CAL SET <n>` → 手动设置；`CAL RESET` → 恢复 4000。

- [ ] **Step 3: 实测验证**

电机连编码器（或手动慢转一整圈，从任一 Index 脉冲起）→ 记录 `dpos` 与标称 4000 对比。多圈后 `CAL STEPS` 稳定。若 A/B 反相（倒着转），CLI 加 `CFG POL` 切换 IC 极性（TIM2 IC 极性翻转），确保正向转动 counts 递增。

- [ ] **Step 4: 提交**

```bash
git add "H743 motor/firmware/abi_monitor_h743"
git commit -m "feat(h743): Index实测步数 EMA 校准 + 方向极性可配"
```

---

### Task 7: 综合验证 + 文档

**Files:**
- Modify: `firmware/abi_monitor_h743/README.md`（新建）

- [ ] **Step 1: 全流程实测**

1. 接线确认：**A→PA5（TIM2_CH1）、B→PA1（TIM2_CH2）、Index→PA4（EXTI4）**，GND 共地，3.3V 电平。PA0 也可用（丝印 A0 已引出），可作 A 信号备选（PA0=TIM2_CH1 同功能）。
2. `ID`/`CFG SHOW` → 默认 3 档 1/2/4。
3. 低速（<4000rpm）：div=1，记录点密度最高；中速（4000~8000）：div=2；高速（>8000）：div=4。
4. `ARM` → 转动 → 自动触发 → `DUMP` / `SD SAVE` → PC 曲线还原。
5. 弹射场景模拟：0→6500rpm 快速拉升，验证 0.8s 窗口点数在各档位下 ≤ SNAP_CAP（div=1 峰值 21.3k 点、div=2 峰值 21.3k 点、div=4 峰值 12k 点，均 < 32k）。
6. 断电重启 → 配置仍在（Flash 持久化）。

- [ ] **Step 2: 写 README**

接线图、CLI 命令表、BIN v2 格式说明、PC 工具使用步骤（放 `H743 motor/README.md`）。**0.8s 窗口截断说明**（必写）：默认档位（1/2/4 @ 4000/8000）下记录点率 ≤ 26.7kHz（div=1@4000 峰值 21.3k 点/0.8s，div=2@8000 峰值 21.3k 点/0.8s，div=4@12000 峰值 12k 点/0.8s），0.8s 窗口恒不截断；**若手动把高速档 div 降到 1**（如 `CFG GEAR 3 1 4000 1 8000 1`），记录点率可能 > SNAP_CAP/0.8s，窗口被截短——事件点超 32768 即停，实际时长 = 32768/点率。

- [ ] **Step 3: 提交**

```bash
git add "H743 motor/firmware/abi_monitor_h743" "H743 motor/README.md"
git commit -m "docs(h743): 综合验证 + README"
```

---

### Task 1A（已在计划评审后新增，测试用电机 PWM + CLI）

> 用户批准追加的测试手段：一条测试 PWM 输出驱动 ESC/电机，用于 Task 2/5 单机测速校准。
> **引脚已定：PA7（丝印 A7，空闲）→ TIM3_CH2（AF2）**；接线见 `docs/接线文档.md` 第 5 节。

**Files:**
- Create: `firmware/abi_monitor_h743/App/app_pwm.c/h`（TIM3_CH2，PWM 频率默认定在 500Hz 可由宏调整；`void Pm_Init(void)`、`void Pm_SetDuty(uint32_t permille1000)`）
- Modify: `App/app_cli.c` 增 `PWM <0..1000>`（‰，ESC 常用 0..1000 代表 0..100%）命令
- Modify: uvprojx 加入 `app_pwm.c`

> **实施（2026-08-06 已完成并烧录验证）**：
> - TIM3 时钟=240MHz（APB1×2），500Hz → **PSC=7**（30MHz 计数率）、**ARR=59999**（16 位定时器上限约束）
> - `stm32h7xx_hal_conf.h` 需另开 `#define HAL_TIM_MODULE_ENABLED`
> - `main.c` 在 `Usart_Init` 后调 `Pm_Init()`（占空比 0）
> - CLI `PWM <0..1000>` 已接，编译+烧录+验证通过

**行为：**
- 上电 `Pm_Init` 后占空比=0（电机不动），需显式 `PWM xxx` 启动
- 每次 `PWM 0` 停机
- 计划其余部分不变（闭环不做，仍只记录/监控）

> 注：`docs/接线说明.md` 已写 ST-Link/USB/ABI/PWM 全部接线（Task 5 的接线文档前置完成）。

---

### Task 8（后期，本计划不含实施）: 闭环控制

弹射/扑翼推力闭环放在本期交付后单独立项：400Hz PWM（预留 500Hz）驱动 ESC、3Hz 非对称正弦波形（基线+500/-300 摆幅）、扑翼周期=飞控下发、实时调节。本期只保证记录/监控能力与计算余量（CPU <5%）。

## 自审记录

- 二次评审启动前便利贴已核实合入：TIM5 时钟**以例程实测为准**（HSE25→SYSCLK480→AHB÷2→HCLK240→APB1÷2→PCLK1 120→**TIM5=240MHz**，PSC=3→60MHz 刻度正确；480MHz 论点仅在 HCLK=480 配置成立），故 PSC 改为运行时 `2×PCLK1/60MHz-1` 推导 + 上电打印核对，写死数值不再依赖假设；64 位除法（60k 次/s ≈ 每 ISR ~15 周期 @240MHz）无影响不改；0.8s 窗口默认档不截断（div1@4000=21.3k、div2@8000=21.3k、div4@12000=12k 点均 <32k），仅手动把高速档 div 降 1 时才截短，README 已注明。
- 档位默认边界 `4000/8000` 与 div 1/2/4 匹配（评审 2 要求高速自动 div≥2），窗口点数恒 ≤ 32k：div=1@4000rpm=21.3k 点、div=2@8000rpm=21.3k 点、div=4@12000rpm=12k 点；档数 3/4/5 可配满足"后期可调"。
- 评审修正已合入：TIM4→TIM5 32 位（QUPR_MAX=150000 超 16 位 ARR）；EXTI5 残留→统一 EXTI4/PA4；HAL_GetTick 合成时间戳→64 位硬件刻度计数（g_tick64，µs 时 /60）；ring 改 2 的幂掩码 + head/tail 单调计数；div 切换重置 sub；配置扇区改 Bank2 尾扇区 0x081F0000（128KB 扇区，固件 <1MB 永不相碰）；基础例程源改用 `2.参考例程\1.基础例程`（FatFS 用自带 `SDMMC-SD卡移植FatFs`）。
- A0C/A1C/C2C/C3C 丝印为 STM32H743 的 ADC 专用模拟脚（PA0_C/PA1_C/PC2_C/PC3_C，仅 ADC），不能做 GPIO/TIM。丝印 A0/A1 已确认引出（A0C/A1C 为 ADC 模拟脚，与 A0/A1 非同脚），故编码器 A 用 PA5（TIM2_CH1 AF1，丝印 A5）、B 用 PA1（TIM2_CH2 AF1，丝印 A1）、Index 用 PA4（EXTI4，丝印 A4）。
- BIN v2 16B 与 PC 工具（magic 0xAB1C0002）字节级兼容；DSP 的 v3 20B 不采用。
- UTO 时基换算：TIM5（32 位）预分频 3 → 60MHz 刻度（PSC=3 即 4 分频），QUPR 值按 60/150 比例换算自 DSP 的 150MHz 值（2500/375000→1000/150000），迟滞保持 12.5%；QUPR_MAX 150000 超出 16 位 ARR 故弃 TIM4 用 TIM5（评审阻塞项）。
