#include "stm32f10x_it.h"
#include "encoder.h"
#include "ring_buffer.h"
#include "usart_cmd.h"

volatile uint32_t g_last_count = 0;
volatile uint32_t g_last_time = 0;
volatile float g_current_rpm = 0.0f;
volatile bool g_z_signal_occurred = false;

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1); }
void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void) { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) {}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        Buffer_Write(Encoder_GetCount());
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
        Cmd_ProcessChar(USART_ReceiveData(USART1));
    }
}

void EXTI0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0)) {
        EXTI_ClearITPendingBit(EXTI_Line0);
        g_z_signal_occurred = true;
    }
}
