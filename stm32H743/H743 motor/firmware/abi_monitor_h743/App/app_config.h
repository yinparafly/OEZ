/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   ABI 监控器应用配置（档位抽稀 / 串口 / 版本）
  * @note    配置行为 3/4/5 档，每档有 div 与 rpm 边界；掉电保存在内部 Flash。
  ******************************************************************************
  */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include <stdint.h>

/*------------------------------------------ 版本 ----------------------------------*/

#define APP_VERSION_MAJOR   0
#define APP_VERSION_MINOR   1
#define APP_VERSION_STR     "ABI-MONITOR-H743 v0.1"

/*------------------------------------------ 档位 ----------------------------------*/

#define CFG_GEAR_MAX        5               /* 档数上限（3/4/5 可配）               */
#define CFG_GEAR_MIN        3               /* 档数下限                             */

/*------------------------------------------ 串口 ----------------------------------*/

#define CFG_UART_BAUD       921600UL        /* 固定 921600（PC 端原版工具用）      */

/*------------------------------------------ Flash 存储 -----------------------------*/

#define CFG_FLASH_BASE      0x081E0000UL    /* Bank2 末扇区（扇区15=Bank2第8个=0x081E0000）*/
#define CFG_MAGIC           0xA5A5C0DEUL

/*------------------------------------------ 宏 ------------------------------------*/

#define CLAMP(x, lo, hi)   (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))

/*------------------------------------------ 接口 ------------------------------------*/

void    Config_Load(void);
void    Config_Save(void);
void    Config_Reset(void);
uint8_t Config_GetGearN(void);
uint8_t Config_GetGearDiv(uint8_t i);
uint32_t Config_GetGearBnd(uint8_t i);
uint8_t Config_CheckGear(uint8_t n, const uint8_t *div, const uint32_t *bnd);
void    Config_SetGear(uint8_t n, const uint8_t *div, const uint32_t *bnd);

#endif /* __APP_CONFIG_H */