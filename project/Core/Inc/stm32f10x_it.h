#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#include "stm32f10x.h"
#include <stdbool.h>

extern volatile bool g_z_signal_occurred;
extern volatile uint32_t g_sys_tick;

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void USART1_IRQHandler(void);

#endif