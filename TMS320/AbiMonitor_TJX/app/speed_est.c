#include "speed_est.h"
#include "eqep_abi.h"
#include "board.h"

/* 测速源 = UTO 事件流（spec v3.1 §4.3a-D2）：
   spd_rpm() 直接读 abi_get_latest_rpm()（事件差分 + 外推衰减归属 eqep_abi）。
   原 1kHz 差分路径（abi_counts 1ms 差分 + EMA）已删除（Task 2）。
   spd_tick_1khz() 保留 1ms 绝对时钟维护（abs_ms，SPD?/snap 触发用）。 */

static uint32_t abs_ms = 0;

void spd_init(void)
{
    abs_ms = 0;
}

void spd_tick_1khz(void)
{
    abs_ms++;
}

int32_t  spd_rpm(void)    { return abi_get_latest_rpm(); }
int      spd_gear(void)   { return abi_gear(); }
uint32_t spd_abs_ms(void) { return abs_ms; }
