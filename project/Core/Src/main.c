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

static void blink_green(int count) {
    for (int i = 0; i < count; i++) {
        GREEN_LED_ON();
        Delay(200);
        GREEN_LED_OFF();
        Delay(200);
    }
}

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint32_t clk;
    const char* name;
} PinEntry;

#define PINA(n)  {GPIOA, GPIO_Pin_##n,  RCC_APB2Periph_GPIOA, "PA" #n}
#define PINB(n)  {GPIOB, GPIO_Pin_##n,  RCC_APB2Periph_GPIOB, "PB" #n}
#define PINC(n)  {GPIOC, GPIO_Pin_##n,  RCC_APB2Periph_GPIOC, "PC" #n}

static void test_candidate_pin(GPIO_TypeDef* port, uint16_t pin, uint32_t clk) {
    RCC_APB2PeriphClockCmd(clk, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(port, &gpio);

    uint16_t on_times[] = {500, 400, 300, 200, 100};
    for (int j = 0; j < 5; j++) {
        GPIO_SetBits(port, pin);
        Delay(on_times[j]);
        GPIO_ResetBits(port, pin);
        Delay(100);
    }

    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(port, &gpio);
    RCC_APB2PeriphClockCmd(clk, DISABLE);
}

static void LED_Scan(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    PinEntry pins[] = {
        PINA(2), PINA(3), PINA(8), PINA(11), PINA(12), PINA(15),
        PINB(1), PINB(2), PINB(5), PINB(6), PINB(7), PINB(8), PINB(9),
        PINB(12), PINB(13), PINB(14), PINB(15),
        PINC(0), PINC(1), PINC(2), PINC(3), PINC(4),
        PINC(5), PINC(6), PINC(7), PINC(8), PINC(9),
        PINC(10), PINC(11), PINC(12), PINC(14), PINC(15),
    };
    int count = sizeof(pins) / sizeof(pins[0]);

    GREEN_LED_ON();
    Delay(2000);
    GREEN_LED_OFF();
    Delay(2000);

    for (int round = 0; round < 3; round++) {
        for (int idx = 0; idx < count; idx++) {
            int tens = idx / 10;
            int ones = idx % 10;

            if (tens > 0) blink_green(tens);
            Delay(600);

            if (ones > 0 || tens == 0) blink_green(ones);
            Delay(1000);

            test_candidate_pin(pins[idx].port, pins[idx].pin, pins[idx].clk);

            Delay(1500);
        }

        GREEN_LED_ON();
        Delay(2000);
        GREEN_LED_OFF();
        Delay(1500);
    }

    RED_LED_OFF();
    GREEN_LED_OFF();
    STATUS_LED_OFF();
}

int main(void) {
    LED_GPIO_Init();
    SysTick_Config(SystemCoreClock / 1000);

    GREEN_LED_ON();
    RED_LED_ON();
    Delay(1000);
    GREEN_LED_OFF();
    RED_LED_OFF();
    Delay(500);

    LED_Scan();

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
        uint8_t tport; uint16_t tpin;
        if (Cmd_GetPinTest(&tport, &tpin)) {
            GPIO_TypeDef* port;
            uint32_t clk;
            if (tport == 0) { port = GPIOA; clk = RCC_APB2Periph_GPIOA; }
            else if (tport == 1) { port = GPIOB; clk = RCC_APB2Periph_GPIOB; }
            else { port = GPIOC; clk = RCC_APB2Periph_GPIOC; }
            test_candidate_pin(port, tpin, clk);
            LED_GPIO_Init();
        }
    }
}