/**
  ******************************************************************************
  * @file    flash_save.h
  * @brief   Task 5 Flash 芯片保存接口（snap 持久化到内部 Flash + PC 帧 DUMP）
  ******************************************************************************
  */
#ifndef __FLASH_SAVE_H
#define __FLASH_SAVE_H

#include <stdint.h>

uint8_t   Flash_SaveSnap(void);      /* snap 写入 Flash：0=ok 2=无数据 3=擦除失败 4=编程失败 */
uint8_t   Flash_IsData(void);        /* Flash 内是否有有效记录（magic 匹配） */
uint32_t  Flash_StoredCount(void);   /* Flash 内记录点数（非法时 0） */
uint8_t   Flash_Dump(void);          /* Flash 记录按 PC 帧发串口（0=ok 2=无数据 6=发送失败） */

uint8_t   Flash_IsSaved(void);       /* 本次 snap 是否已写 Flash */
uint32_t  Flash_Count(void);         /* 当前内存 snap 点数 */

#endif /* __FLASH_SAVE_H */