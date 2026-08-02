#include "sd_fatfs.h"
#include "fatfs/diskio.h"
#include "board.h"
#include "driverlib.h"
#include <string.h>
#include <stdio.h>

static FATFS fs;
static char last_err[64];

#define FATFS_TEST_FNAME "T1TEST.BIN"

const char *sd_fatfs_last_err(void)
{
    return last_err;
}

static void set_err(const char *s)
{
    strncpy(last_err, s, sizeof(last_err) - 1);
    last_err[sizeof(last_err) - 1] = '\0';
}

int sd_fatfs_init(void)
{
    FRESULT fr;
    DSTATUS st;
    last_err[0] = '\0';

    st = disk_initialize(0);
    if (st & STA_NOINIT)
    {
        snprintf(last_err, sizeof(last_err), "disk_init fail st=%u", (unsigned)st);
        return -1;
    }
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK)
    {
        snprintf(last_err, sizeof(last_err), "mount fail fr=%d", (int)fr);
        return -1;
    }
    return 0;
}

int sd_fatfs_test(void)
{
    FIL f;
    FRESULT fr;
    UINT bw, br;
    uint8_t wbuf[64];
    uint8_t rbuf[64];
    unsigned int i;

    for (i = 0; i < sizeof(wbuf); i++)
        wbuf[i] = (uint8_t)i;

    fr = f_open(&f, FATFS_TEST_FNAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
        set_err("open w fail");
        return -1;
    }
    fr = f_write(&f, wbuf, sizeof(wbuf), &bw);
    if (fr != FR_OK || bw != sizeof(wbuf))
    {
        set_err("write fail");
        f_close(&f);
        return -1;
    }
    fr = f_sync(&f);
    if (fr != FR_OK)
    {
        set_err("sync fail");
        f_close(&f);
        return -1;
    }
    fr = f_close(&f);
    if (fr != FR_OK)
    {
        set_err("close fail");
        return -1;
    }

    fr = f_open(&f, FATFS_TEST_FNAME, FA_READ);
    if (fr != FR_OK)
    {
        set_err("open r fail");
        return -1;
    }
    fr = f_read(&f, rbuf, sizeof(rbuf), &br);
    f_close(&f);
    if (fr != FR_OK || br != sizeof(rbuf))
    {
        set_err("read fail");
        return -1;
    }
    if (memcmp(wbuf, rbuf, sizeof(wbuf)) != 0)
    {
        set_err("mismatch");
        return -1;
    }
    f_unlink(FATFS_TEST_FNAME);
    set_err("ok");
    return 0;
}
