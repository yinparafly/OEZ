#include "main.h"
#include "encoder.h"
#include "ring_buffer.h"
#include "usart_cmd.h"
#include "recorder.h"
#include "stm32f10x_it.h"

static void LED_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_InitStructure.GPIO_Pin = STATUS_LED_PIN;
    GPIO_Init(STATUS_LED_PORT, &GPIO_InitStructure);
    STATUS_LED_OFF();

    GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN;
    GPIO_Init(GREEN_LED_PORT, &GPIO_InitStructure);
    GREEN_LED_OFF();

    GPIO_InitStructure.GPIO_Pin = RED_LED_PIN;
    GPIO_Init(RED_LED_PORT, &GPIO_InitStructure);
    RED_LED_OFF();
}

static void START_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = START_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(START_PORT, &GPIO_InitStructure);
}

static void Z_EXTI_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin = Z_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Z_PORT, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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

static void IWDG_Init(void) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(200);
    IWDG_Enable();
}

void Delay(uint32_t ms) {
    uint32_t target = g_sys_tick + ms;
    while (g_sys_tick < target);
}

static void LED_Test(void) {
    GREEN_LED_ON();
    RED_LED_ON();
    Delay(5000);
    for (int i = 0; i < 25; i++) {
        Delay(200);
        GREEN_LED_TOGGLE();
        RED_LED_TOGGLE();
    }
    GREEN_LED_OFF();
    RED_LED_OFF();
}

int main(void) {
    LED_GPIO_Init();
    SysTick_Config(SystemCoreClock / 1000);
    LED_Test();
    START_GPIO_Init();
    Z_EXTI_Init();
    Encoder_Init();
    Buffer_Init();
    TIM3_Sampling_Init();
    USART1_Init(115200);
    Recorder_Init();
    IWDG_Init();

    while (1) {
        IWDG_ReloadCounter();
        if (Recorder_GetState() == RECORDER_WRITING) {
            Record_SaveToSD();
        }
    }
}
