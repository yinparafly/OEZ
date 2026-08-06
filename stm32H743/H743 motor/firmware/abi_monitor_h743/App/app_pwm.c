/**
  ******************************************************************************
  * @file    app_pwm.c
  * @brief   TIM3_CH2 (PA7, AF2) PWM 输出
  * @note    TIM3 时钟 = APB1 x2 = 240MHz（16 位定时器，ARR 上限 65535）
  *          500Hz: PSC=7（8 分频 -> 30MHz），ARR+1 = 30MHz/500 = 60000
  * @note    占空比 0..1000（‰）。上电默认 0（电机不动），需显式命令启动。
  ******************************************************************************
  */
#include "app_pwm.h"
#include "stm32h7xx_hal.h"

#define PWM_FREQ_HZ     500u             /* 输出频率 */
#define PWM_PSC         7u               /* 240MHz/(7+1) = 30MHz 计数率 */
#define PWM_ARR         60000u           /* ARR+1 = 30MHz/500Hz */

static TIM_HandleTypeDef htim3;

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
    htim3.Init.Period            = PWM_ARR - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        while (1) { }
    }

    TIM_OC_InitTypeDef oc = { 0 };
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;                          /* 初始占空比 0 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2) != HAL_OK) {
        while (1) { }
    }

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Pm_SetDuty(uint32_t permille1000)
{
    if (permille1000 > 1000) permille1000 = 1000;
    uint32_t pulse = (uint32_t)((uint64_t)permille1000 * (htim3.Init.Period + 1) / 1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}