#ifndef __USART_CMD_H
#define __USART_CMD_H

#include "stm32f10x.h"
#include <stdbool.h>

#define CMD_START_CHAR  'G'

void USART1_Init(uint32_t baud);
bool Cmd_GetStartFlag(void);
void Cmd_ClearStartFlag(void);
void Cmd_ProcessChar(uint8_t c);

#endif
