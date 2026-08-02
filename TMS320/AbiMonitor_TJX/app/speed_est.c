#include "speed_est.h"
#include "eqep_abi.h"
#include "board.h"

/* UTO runs at 2kHz fixed — gear switching not needed for UTO.
   Gear functions retained for future PCM re-introduction (Task 10). */

static int64_t  last_cnt    = 0;
static uint32_t rpm_ema     = 0;
static uint32_t stop_ticks  = 0;
static uint32_t abs_ms      = 0;

void spd_init(void)
{
    last_cnt   = abi_counts();
    rpm_ema    = 0;
    stop_ticks = 0;
    abs_ms     = 0;
}

void spd_tick_1khz(void)
{
    int64_t now   = abi_counts();
    int64_t delta = now - last_cnt;
    last_cnt      = now;

    abs_ms++;

    if (delta != 0)
    {
        int64_t ad = (delta > 0) ? delta : -delta;
        /* RPM = ad * 1000 * 60 / PPR.  Use default PPR=4000 (4X * 1000 line).
           With 1kHz tick: ad counts/ms → RPM = ad * 60,000 / 4000 = ad * 15. */
        uint32_t rpm = (uint32_t)((uint64_t)ad * 15u);
        /* EMA smoothing */
        rpm_ema = (rpm_ema * 7u + rpm) / 8u;
        stop_ticks = 0;
    }
    else
    {
        stop_ticks++;
        if (stop_ticks >= 100)   /* 100 ms without events → RPM = 0 */
            rpm_ema = 0;
    }

    /* Gear switch — retained but UTO doesn't benefit from it.
       Thresholds from design spec §4.4: >12000 switch to 1X, <5500 switch back.
       Currently a no-op since UTO is fixed-rate. */
}

uint32_t spd_rpm(void)   { return rpm_ema; }
int      spd_gear(void)  { return abi_gear(); }
uint32_t spd_abs_ms(void){ return abs_ms; }
