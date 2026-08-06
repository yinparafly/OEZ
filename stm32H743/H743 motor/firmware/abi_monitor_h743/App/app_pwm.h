/**
  ******************************************************************************
  * @file    app_pwm.h
  * @brief   TIM3_CH2 (PA7) PWM 输出，用于测试用电机/ESC 驱动
  * @note    频率默认 500Hz；占空比 0..1000（‰）
  ******************************************************************************
  */
#ifndef __APP_PWM_H
#define __APP_PWM_H

#include <stdint.h>

void Pm_Init(void);
void Pm_SetDuty(uint32_t permille1000);

#endif /* __APP_PWM_H */
