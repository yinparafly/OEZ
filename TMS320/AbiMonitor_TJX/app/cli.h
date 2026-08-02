#ifndef CLI_H
#define CLI_H

#include <stdint.h>

void cli_init(void);
void cli_task(void);

// ISR 调用：将接收到的字符推入 CLI 缓冲区（在中断上下文中安全）
void cli_push_rx(char c);

#endif
