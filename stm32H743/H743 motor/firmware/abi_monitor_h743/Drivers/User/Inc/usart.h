/**
  ******************************************************************************
  * @file    usart.h
  * @brief   USART1 串口驱动（中断收 + 轮询发）
  ******************************************************************************
  */
#ifndef __USART_H
#define __USART_H

#include <stdint.h>

void      Usart_Init(void);
uint8_t   Usart_GetChar(char *c);       /* 从接收 FIFO 取一个字节，空返回 0 */
void      Usart_Print(const char *s);
void      Usart_Printc(char c);
void      Usart_Write(const uint8_t *buf, uint32_t len);  /* 原始字节块发送（DUMP 用） */

#endif /* __USART_H */