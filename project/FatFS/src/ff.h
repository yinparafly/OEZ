#ifndef __FF_H
#define __FF_H

#include "stm32f10x.h"

typedef int FRESULT;

#define FR_OK        0
#define FR_ERROR     1

#define FA_READ            0x01
#define FA_WRITE           0x02
#define FA_CREATE_ALWAYS   0x08

typedef struct { uint32_t dummy; } FATFS;
typedef struct { uint32_t dummy; } FIL;

FRESULT f_mount(uint8_t vol, FATFS* fs);
FRESULT f_open(FIL* fp, const char* path, uint8_t mode);
FRESULT f_close(FIL* fp);
int f_printf(FIL* fp, const char* fmt, ...);

#endif
