#include "main.h"
#include "encoder.h"

static void LED_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    LED_OFF();
}

void Delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) { __NOP(); }
}

int main(void) {
    LED_GPIO_Init();
    Encoder_Init();
    while (1) {
        LED_TOGGLE();
        Delay(500);
    }
}
