/**
  ******************************************************************************
  * @file    app_pwm.h
  * @brief   TIM3_CH2 (PA7, AF2) PWM 油门输出（RC 电调信号，DSP 兼容�?  * @note    - 400Hz 周期 2.5ms；脉�?1.02ms(�? ~ 2.0ms(全�?
  *          - PWM 0..1000（‰）�?1.02ms..2.0ms 线性映�?  *          - PWM 0 = 最低油�?1.02ms（保守贴近电调要求，绝不无信号）
  *          - 油门渐变：Pm_SetDuty 设目标，Pm_Tick �?5ms 渐进逼近
  *            （从当前脉宽逐级爬向目标，禁止瞬间跳变——电调安全）
  *          - 主循环必须周期调�?Pm_Tick()
  ******************************************************************************
  */
#ifndef __APP_PWM_H
#define __APP_PWM_H

#include <stdint.h>

void Pm_Init(void);
void Pm_SetDuty(uint32_t permille1000);   /* 设目标油�?0..1000（渐变逼近�?*/
void Pm_Tick(void);                       /* 周期调用：渐变逼近目标 */

uint32_t Pm_GetDuty(void);                /* 当前实际输出�?�?*/

#endif /* __APP_PWM_H */