#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

#define STATUS_LED_PIN       GPIO_Pin_13
#define STATUS_LED_PORT      GPIOC
#define STATUS_LED_ON()      GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN)
#define STATUS_LED_OFF()     GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN)
#define STATUS_LED_TOGGLE()  (STATUS_LED_PORT->ODR ^= STATUS_LED_PIN)

#define GREEN_LED_PIN       GPIO_Pin_0
#define GREEN_LED_PORT      GPIOB
#define GREEN_LED_ON()      GPIO_ResetBits(GREEN_LED_PORT, GREEN_LED_PIN)
#define GREEN_LED_OFF()     GPIO_SetBits(GREEN_LED_PORT, GREEN_LED_PIN)
#define GREEN_LED_TOGGLE()  (GREEN_LED_PORT->ODR ^= GREEN_LED_PIN)

#define RED_LED_PIN         GPIO_Pin_1
#define RED_LED_PORT        GPIOB
#define RED_LED_ON()        GPIO_ResetBits(RED_LED_PORT, RED_LED_PIN)
#define RED_LED_OFF()       GPIO_SetBits(RED_LED_PORT, RED_LED_PIN)
#define RED_LED_TOGGLE()    (RED_LED_PORT->ODR ^= RED_LED_PIN)

#define START_PIN           GPIO_Pin_11
#define START_PORT          GPIOB

#define Z_PIN               GPIO_Pin_10
#define Z_PORT              GPIOB

void Delay(uint32_t ms);

#endif
