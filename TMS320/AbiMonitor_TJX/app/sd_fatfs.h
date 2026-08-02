#ifndef SD_FATFS_H
#define SD_FATFS_H

#include "fatfs/ff.h"

int sd_fatfs_init(void);
int sd_fatfs_test(void);
const char *sd_fatfs_last_err(void);

#endif
