#ifndef CLI_H
#define CLI_H

#include <stdint.h>

void cli_init(void);
void cli_task(void);

// ISR 调用：将接收到的字符推入 CLI 缓冲区（在中断上下文中安全）
void cli_push_rx(char c);

/* 输出：文本行（\n 自动转 \r\n）与原始字节（二进制帧用） */
void cli_put_raw(const char *s);
void cli_printf(const char *fmt, ...);
void cli_put_raw_bytes(const unsigned char *buf, uint32_t len);

#endif
