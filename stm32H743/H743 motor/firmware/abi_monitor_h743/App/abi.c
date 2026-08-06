/**
  ******************************************************************************
  * @file    abi.c
  * @brief   AS5047P ABI 编码器捕获（Task 2）
  *          - TIM2 编码器 4X（PA5=A, PA1=B, AF1），32 位 CNT
  *          - EXTI4（PA4）= Index，中断计数
  *          - TIM5 UTO 自适应定时器（32 位，60MHz 刻度）：每周期 ~N_MIN 步，
  *            动态 ARR（QUPR 夹顶 + 12.5% 迟滞），ISR 内累计 64 位硬件时间戳
  *          - 抽稀档位：按 rpm 查 app_config 档位表，切换档位重置 sub
  *          - 测速：事件差分 dpos×60e6/(dt_us×4000) rpm
  * 时钟链：HSE25 → SYSCLK480 → HCLK240 → PCLK1 120 → TIM5 = 2×PCLK1 = 240MHz
  *          PSC = 2×PCLK1/60MHz - 1 = 3（运行时推导，锁死 60MHz 刻度）
  ******************************************************************************
  */
#include "abi.h"
#include "app_config.h"
#include "stm32h7xx_hal.h"

/*------------------------------------------ 内部状态 -------------------------*/

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim5;

/* 64 位刻度计数（60MHz → 每刻度 1/60µs），TIM5 ISR 内累加，纯硬件零漂移 */
static volatile uint64_t g_tick64;

static volatile int32_t  g_rpm;      /* 事件差分测速（有符号） */
static volatile uint32_t g_idx_cnt;  /* Index 圈数 */

/*------------------------------------------ TIM2 编码器 4X --------------------*/

static void Tim2_Encoder_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_1;      /* PA5=A, PA1=B */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFFu;   /* 32 位计数 */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = { 0 };
    enc.EncoderMode = TIM_ENCODERMODE_TI12;       /* 4X */
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;  /* TI1 → IC1 */
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;  /* TI2 → IC2 */
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Filter   = 0;
    enc.IC2Filter   = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) {
        while (1) { }
    }
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);            /* 清零计数起点（上电 CNT 是垃圾值） */
}

/*------------------------------------------ EXTI4 Index ----------------------*/

static void Exti4_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin  = GPIO_PIN_4;                        /* PA4 = Index */
    gpio.Mode = GPIO_MODE_IT_RISING;               /* 只上升沿：每转 +1（RISING_FALLING 会每转 +2） */
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);        /* 低于 UTO(TIM5) */
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI4_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        g_idx_cnt++;
        Snap_OnIndex();
    }
}

/*------------------------------------------ TIM5 UTO -------------------------*/

static void Tim5_Uto_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance               = TIM5;
    /* PSC 由实际时钟推导，锁死 60MHz 刻度：2×PCLK1 / 60MHz - 1 */
    htim5.Init.Prescaler         = (uint32_t)((2uL * HAL_RCC_GetPCLK1Freq()) / 60000000uL) - 1uL;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = UTO_QUPR_MIN_60M;   /* 初始 60kHz 夹顶 */
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        while (1) { }
    }

    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    HAL_TIM_Base_Start_IT(&htim5);
}

void TIM5_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) == RESET) return;
    __HAL_TIM_CLEAR_IT(&htim5, TIM_FLAG_UPDATE);

    /* 1) 64 位刻度累计（零漂移）：本周期 ARR 刻度 → 事件时刻 */
    g_tick64 += htim5.Instance->ARR;
    uint32_t us_now = (uint32_t)(g_tick64 / 60uL);

    /* 2) 事件读取：A/B 相序使 TIM2 反向计数，取反后正转 = 递增（与 idx 同向） */
    uint32_t cnt = 0u - TIM2->CNT;
    static uint32_t prev_cnt;
    int32_t dpos = (int32_t)(cnt - prev_cnt);
    prev_cnt = cnt;

    /* 3) 测速：事件差分 rpm = dpos×60e6 / (dt_us×4000) */
    static uint64_t prev_tick64;
    uint32_t dt_us = (uint32_t)((g_tick64 - prev_tick64) / 60uL);
    prev_tick64 = g_tick64;
    if (dt_us && dpos) {
        int64_t rpm = (int64_t)dpos * 60000000LL / (int64_t)dt_us / (int64_t)STEPS_PER_REV;
        if (rpm > 60000)  rpm = 60000;    /* 合理性钳制：>6 万 rpm 视为异常 */
        if (rpm < -60000) rpm = -60000;
        g_rpm = (int32_t)rpm;
    }

    /* 4) 抽稀档位（切换时重置 sub，避免档位跳变丢点） */
    uint8_t div = Abi_GetDiv();
    static uint8_t sub;
    static uint8_t last_div = 0xFF;
    if (div != last_div) { sub = 0; last_div = div; }
    if (++sub >= div) {
        sub = 0;
        Snap_OnEvent(cnt, g_idx_cnt, us_now);   /* Task 3 记录点 */
    }

    /* 5) 动态周期：保持每周期 ~N_MIN 步，夹顶 + 迟滞防抖 */
    uint32_t target = (uint32_t)((uint64_t)htim5.Instance->ARR * N_MIN /
                                 (dpos ? (uint32_t)(dpos < 0 ? -dpos : dpos) : N_MIN));
    if (target < UTO_QUPR_MIN_60M) target = UTO_QUPR_MIN_60M;
    if (target > UTO_QUPR_MAX_60M) target = UTO_QUPR_MAX_60M;
    uint32_t arr = htim5.Instance->ARR;
    if (target < arr * (1000 - UTO_HYST) / 1000 || target > arr * (1000 + UTO_HYST) / 1000) {
        __HAL_TIM_SET_AUTORELOAD(&htim5, target);
    }
    prev_cnt = cnt;
}

/*------------------------------------------ 对外接口 -------------------------*/

void Abi_Init(void)
{
    Tim2_Encoder_Init();
    Exti4_Init();
    Tim5_Uto_Init();
}

int32_t Abi_GetRpm(void)      { return (int32_t)g_rpm; }
uint32_t Abi_GetIndexCnt(void){ return g_idx_cnt; }
uint32_t Abi_GetUsNow(void)   { return (uint32_t)(g_tick64 / 60uL); }
uint32_t Abi_GetCnt(void)     { return 0u - TIM2->CNT; }   /* 与 UTO ISR 同向补偿 */

/* 查档位表：|rpm| < bnd[i] → div[i]，否则末档 */
uint8_t Abi_GetDiv(void)
{
    int32_t rpm = Abi_GetRpm();
    if (rpm < 0) rpm = -rpm;
    for (uint8_t i = 0; i + 1 < cfg_gear_n; i++) {
        if ((uint32_t)rpm < cfg_gear_bnd[i]) return cfg_gear_div[i];
    }
    return cfg_gear_div[cfg_gear_n - 1];
}

/* Task 2 空实现（weak）：Task 3 snap_bin.c 强定义覆盖 */
#if defined(__GNUC__)
__attribute__((weak))
#endif
void Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now)
{
    (void)cnt; (void)idx; (void)us_now;
}

#if defined(__GNUC__)
__attribute__((weak))
#endif
void Snap_OnIndex(void)
{
}
