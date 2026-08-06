/*-----------------------------------------------------------------------*/
/* Low level disk I/O glue for FatFs - direct HAL_SD (polling)           */
/* Task 4: 直接对接 sd_card.c 的 SDMMC1（无 BSP 层，可靠性优先）          */
/* SDMMC 内部 DMA 要求数据 4B 对齐：FatFS 缓冲可能不对齐，中转保证       */
/*-----------------------------------------------------------------------*/
#include "diskio.h"
#include "sd_card.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* 中转缓冲：SDMMC 内部 DMA 需 4B 对齐（FatFS win/目录缓冲不能保证） */
static uint32_t s_sd_buf[512 / 4] __attribute__((aligned(8)));   /* 512B */

static DRESULT sd_disk_read_core(uint32_t sector, uint32_t cnt)
{
    return (HAL_SD_ReadBlocks(&hsd1, (uint8_t *)s_sd_buf, sector, cnt, HAL_MAX_DELAY) == HAL_OK)
           ? RES_OK : RES_ERROR;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    if (SD_Init() != 0) return STA_NOINIT;
    return 0;
}

/* FAT 时间戳（无 RTC 板）：固定回传合理值，满足 f_open/f_sync 链接 */
DWORD get_fattime(void)
{
    return 0;   /* 1980-01-01 00:00（文件名已含秒序，PC 不依赖此） */
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0) return RES_PARERR;
    if (count == 0) return RES_PARERR;

    if (((uint32_t)buff & 3u) == 0)
        return (HAL_SD_ReadBlocks(&hsd1, buff, sector, count, HAL_MAX_DELAY) == HAL_OK)
               ? RES_OK : RES_ERROR;

    /* 未对齐：经中转缓冲 */
    for (UINT i = 0; i < count; i++) {
        if (sd_disk_read_core(sector + i, 1) != RES_OK) return RES_ERROR;
        memcpy(buff + i * 512, s_sd_buf, 512);
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0) return RES_PARERR;
    if (count == 0) return RES_PARERR;

    if (((uint32_t)buff & 3u) == 0)
        return (HAL_SD_WriteBlocks(&hsd1, (BYTE *)buff, sector, count, HAL_MAX_DELAY) == HAL_OK)
               ? RES_OK : RES_ERROR;

    /* 未对齐：经中转缓冲 */
    for (UINT i = 0; i < count; i++) {
        memcpy(s_sd_buf, buff + i * 512, 512);
        if (HAL_SD_WriteBlocks(&hsd1, (uint8_t *)s_sd_buf, sector + i, 1, HAL_MAX_DELAY) != HAL_OK)
            return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    HAL_SD_CardInfoTypeDef info;

    if (pdrv != 0) return RES_PARERR;
    switch (cmd)
    {
        case CTRL_SYNC:
            if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) return RES_ERROR;
            return RES_OK;

        case GET_SECTOR_COUNT:
            if (HAL_SD_GetCardInfo(&hsd1, &info) != HAL_OK) return RES_ERROR;
            *(DWORD *)buff = info.BlockNbr;
            return RES_OK;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            return RES_OK;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 512;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}