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

/* 非零时禁止文本输出（DUMP BIN 二进制帧期间置位） */
extern volatile int g_dump_active;

/* 电机活动标志：MOTOR <dir> <spd> 置 1；MOTOR 0 0 / 看门狗停机 清 0。
   main 看门狗用（PWM 活动但 0.5s 无编码器事件 → 停机） */
extern volatile uint32_t g_motor_active;

/* 重启闭环测试序列（重置所有状态并从头跑 1500→2000 RPM） */
void seq_restart(void);

#endif
