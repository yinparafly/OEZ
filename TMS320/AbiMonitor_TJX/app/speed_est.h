#ifndef SPEED_EST_H
#define SPEED_EST_H

#include <stdint.h>

void     spd_init(void);
void     spd_tick_1khz(void);

/* 返回最新事件 rpm（int32 有符号，spec §4.3a-C1）：
   正 = 正转、负 = 反转、0 = 停转/超时清零。
   只读缓存，值来自 abi_get_latest_rpm()（UTO 事件差分 + 外推衰减
   已在 eqep_abi 实现，本文件只读，勿复制衰减逻辑）。 */
int32_t  spd_rpm(void);
int      spd_gear(void);
uint32_t spd_abs_ms(void);

#endif
