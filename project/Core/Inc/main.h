#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

#define LED_PIN       GPIO_Pin_13
#define LED_PORT      GPIOC
#define LED_ON()      GPIO_SetBits(LED_PORT, LED_PIN)
#define LED_OFF()     GPIO_ResetBits(LED_PORT, LED_PIN)
#define LED_TOGGLE()  (LED_PORT->ODR ^= LED_PIN)

void Delay(uint32_t ms);

#endif
