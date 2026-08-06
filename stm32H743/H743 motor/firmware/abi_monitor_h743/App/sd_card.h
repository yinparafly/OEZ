#ifndef __SD_CARD_H
#define __SD_CARD_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

extern SD_HandleTypeDef hsd1;   /* SDMMC1 句柄（diskio.c 使用） */

/* SDMMC1 精简驱动（polling，可靠优先；ClockDiv=10 -> 24MHz，标准限内） */
uint8_t SD_Init(void);          /* 0=OK 1=失败 */
uint8_t SD_CardMounted(void);
uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypeDef *info);   /* 0=OK */

/* FatFS 挂载 */
uint8_t Sd_Mount(void);         /* 0=OK 1=无卡/未就绪 2=无文件系统 */

/* 自动备份：Snap DONE 后调，BIN v2 帧写卡，时间名不覆盖 */
uint8_t Sd_SaveSnap(void);      /* 0=OK 1=卡未就绪 2=无数据 3=写失败 */

/* 调试/CLI */
void    Sd_Ls(void);            /* 列出根目录文件 */
void    Sd_Raw(uint32_t sector, uint8_t write);/* 裸扇区读写（诊断 HAL 层） */
uint8_t Sd_Stat(void);          /* 卡信息打印，0=OK */

#endif /* __SD_CARD_H */
