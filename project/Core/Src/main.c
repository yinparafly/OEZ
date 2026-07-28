#include "main.h"
#include "encoder.h"
#include "ring_buffer.h"
#include "usart_cmd.h"
#include "recorder.h"

static void LED_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    LED_OFF();
}

static void TIM3_Sampling_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 8999;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

static void EXTI_Z_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void IWDG_Init(void) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(200);
    IWDG_Enable();
}

volatile uint32_t g_tick_ms = 0;
void SysTick_Handler(void) { g_tick_ms++; }

int main(void) {
    LED_GPIO_Init();
    Encoder_Init();
    Buffer_Init();
    TIM3_Sampling_Init();
    USART1_Init(115200);
    EXTI_Z_Init();
    Recorder_Init();
    IWDG_Init();
    SysTick_Config(SystemCoreClock / 1000);

    while (1) {
        uint32_t now = g_tick_ms;
        IWDG_ReloadCounter();
        Recorder_MainLoop(now);

        if (Recorder_GetState() == RECORDER_IDLE && Cmd_GetStartFlag()) {
            Cmd_ClearStartFlag();
            Recorder_StartMonitor();
        }

        if (Recorder_IsTriggered()) {
            Record_SaveToSD();
        }

        /* LED: IDLE slow blink, MONITOR on, TRIGGERED fast blink, WRITING off */
        static uint32_t last_led = 0;
        uint32_t period;
        switch (Recorder_GetState()) {
            case RECORDER_IDLE:      period = 500; break;
            case RECORDER_MONITOR:   period = 0;   break;
            case RECORDER_TRIGGERED: period = 50;  break;
            case RECORDER_WRITING:   period = 0xFFFFFFFF; break;
            default:                 period = 500; break;
        }
        if (period == 0) { LED_ON(); }
        else if (period == 0xFFFFFFFF) { LED_OFF(); }
        else if ((now - last_led) >= period) { LED_TOGGLE(); last_led = now; }
    }
}
