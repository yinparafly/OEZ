#include "ff.h"

FRESULT f_mount(uint8_t vol, FATFS* fs) {
    (void)vol; (void)fs;
    return FR_OK;
}

FRESULT f_open(FIL* fp, const char* path, uint8_t mode) {
    (void)fp; (void)path; (void)mode;
    return FR_OK;
}

FRESULT f_close(FIL* fp) {
    (void)fp;
    return FR_OK;
}

int f_printf(FIL* fp, const char* fmt, ...) {
    (void)fp; (void)fmt;
    return 0;
}
