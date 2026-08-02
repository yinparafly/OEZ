#ifndef SPEED_EST_H
#define SPEED_EST_H

#include <stdint.h>

void     spd_init(void);
void     spd_tick_1khz(void);
uint32_t spd_rpm(void);
int      spd_gear(void);
uint32_t spd_abs_ms(void);

#endif
