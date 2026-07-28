#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void Encoder_Init(void);
uint32_t Encoder_GetCount(void);
int Encoder_GetDirection(void);

#endif