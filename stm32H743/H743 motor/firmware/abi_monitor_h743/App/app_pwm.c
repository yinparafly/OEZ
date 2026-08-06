/**
  ******************************************************************************
  * @file    app_pwm.c
  * @brief   TIM3_CH2 (PA7, AF2) PWM 油门输出（RC 电调信号）
  * @note    50Hz 周期 20ms（RC 电调标准频率），脉宽 ~1ms(停) ~ 2ms(全速)
  *          TIM3 时钟 = APB1 x2 = 240MHz；16 位 ARR 上限 65535：
  *          计数率 = 240M/(PSC+1) ≤ 3.28MHz 才放得下 50Hz
  *          PSC=73 → 3.243MHz 计数率，ARR+1 = 64865 → ARR = 64864
  *          脉宽:   1.02ms = 3308 tick，2.0ms = 6486 tick，线性映射
  * @note    油门渐变（安全）：Pm_SetDuty 只记目标值，Pm_Tick() 每 5ms
  *          把当前脉宽向目标逼近一步（≤32 tick ≈ 0.01ms，爬升率 ≈ 2ms/s），
  *          绝不允许瞬间跳变（电调要求平滑油门）。
  *          PWM 0 = 最低油门 1.02ms（>1ms 死区，电调静默），绝不"无信号"。
  *          上电输出最低油门（电机不动，电调不叫）。
  ******************************************************************************
  */
#include "app_pwm.h"
#include "stm32h7xx_hal.h"

#define PWM_PSC         73u              /* 240MHz/74 = 3.243MHz 计数率 */
#define PWM_ARR         64864u           /* ARR+1 = 3.243MHz/50Hz = 64865 */
#define PWM_PULSE_MIN   3308u            /* 1.02ms = 有效最小油门（>1ms 死区） */
#define PWM_PULSE_MAX   6486u            /* 2.0ms = 全速 */

#define RAMP_STEP_TICK  32u              /* 每步 ≤ 32 tick ≈ 0.01ms */
#define RAMP_PERIOD_MS  5u               /* 每 5ms 走一步（≈2ms/s 柔和爬升） */

static TIM_HandleTypeDef htim3;
static volatile uint32_t pwm_cur_tick = PWM_PULSE_MIN;   /* 当前实际脉宽 */
static volatile uint32_t pwm_tgt_tick = PWM_PULSE_MIN;   /* 目标脉宽 */
static uint32_t ramp_last_ms;

void Pm_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin       = GPIO_PIN_7;                 /* PA7 = TIM3_CH2 */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = PWM_PSC;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = PWM_ARR;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        while (1) { }
    }

    TIM_OC_InitTypeDef oc = { 0 };
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = PWM_PULSE_MIN;              /* 上电最低油门 1.02ms */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2) != HAL_OK) {
        while (1) { }
    }

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    ramp_last_ms = HAL_GetTick();
}

/* 0..1000‰ → 1.02ms..2.0ms（停..全速）线性映射（仅记录目标，渐变逼近） */
void Pm_SetDuty(uint32_t permille1000)
{
    if (permille1000 > 1000) permille1000 = 1000;
    pwm_tgt_tick = PWM_PULSE_MIN +
                   (uint32_t)((uint64_t)permille1000 * (PWM_PULSE_MAX - PWM_PULSE_MIN) / 1000);
}

/* 主循环周期调用：每 5ms 向目标逼近一步（≤0.01ms），爬升率 ≈ 2ms/s */
void Pm_Tick(void)
{
    uint32_t now = HAL_GetTick();
    if (now - ramp_last_ms < RAMP_PERIOD_MS) return;
    ramp_last_ms = now;

    uint32_t cur = pwm_cur_tick;
    uint32_t tgt = pwm_tgt_tick;
    if (cur == tgt) return;

    if (cur < tgt) {
        cur += RAMP_STEP_TICK;
        if (cur > tgt) cur = tgt;
    } else {
        cur -= RAMP_STEP_TICK;
        if (cur < tgt) cur = tgt;
    }
    pwm_cur_tick = cur;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cur);
}

uint32_t Pm_GetDuty(void)
{
    return (uint32_t)((uint64_t)(pwm_cur_tick - PWM_PULSE_MIN) * 1000 /
                      (PWM_PULSE_MAX - PWM_PULSE_MIN));
}