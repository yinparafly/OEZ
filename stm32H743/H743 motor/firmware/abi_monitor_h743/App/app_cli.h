/**
  ******************************************************************************
  * @file    app_cli.h
  * @brief   CLI 交互层（HELP/ID/CFG GEAR/SHOW/SAVE/RESET）
  ******************************************************************************
  */
#ifndef __APP_CLI_H
#define __APP_CLI_H

#include <stdint.h>

void Cli_Init(void);
void Cli_OnChar(char c);    /* usart 中断回调逐字节喂入 */
void Cli_Poll(void);        /* 主循环轮询（预留） */
void Cli_PrintU32(uint32_t v);

#endif /* __APP_CLI_H */