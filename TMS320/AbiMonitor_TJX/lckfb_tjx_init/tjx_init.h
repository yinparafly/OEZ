#ifndef __TJX_INIT_H__
#define __TJX_INIT_H__

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

#ifndef u64
#define u64 uint64_t
#endif

void delay_us(int us);
void delay_1us(int us);
void delay_ms(int ms);
void delay_1ms(int ms);
void delay_s(int s);

int lc_printf(char* format, ...);

#ifdef __cplusplus
}
#endif

#endif // __TJX_INIT_H__


