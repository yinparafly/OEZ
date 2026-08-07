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

/* 默认一圈步数（4X 标称 4000）/ 极性（0=反向计数需取反，1=正向计入） */
#define CFG_STEPS_PER_REV_DEFAULT  4000UL
#define CFG_POL_DEFAULT            0

/*------------------------------------------ 动态滤波（Task 8） -----------------------*/

#define FLT_ICF_N        5            /* IC 输入滤波档位数（对应 5 段转速） */
#define FLT_EMA_MAX      500u         /* EMA 系数 ‰ 上限（0.500） */
#define FLT_EMA_DEF      150u         /* EMA α=0.150（ESP32 同款） */
#define FLT_DPOS_MAX     20u          /* dpos 超限倍数上限（×N_MIN） */
#define FLT_DPOS_DEF     6u           /* |dpos|>6*N_MIN 视为毛刺 */
#define FLT_ZERO_MAX     60u          /* 静止判定周期数上限 */
#define FLT_ZERO_DEF     10u          /* 连续 10 周期 dpos==0 强制归零 */
#define FLT_RATE_MAX     100000u      /* 转速变化率限幅 rpm/s 上限（0=自动实测学习） */
#define FLT_RATE_DEF     0u           /* 0=自动实测爬坡斜率学习（Task 8b） */

/* 默认 IC 滤波动态表（按 |rpm| 查表，IC1F/IC2F 值 0..15）：
 * 低速边沿宽信号毛刺多→强滤波；高速边沿窄→弱滤波防丢步（dynanmic filter.txt 推荐） */
#define FLT_ICF_DEF      { 0x0F, 0x0A, 0x06, 0x04, 0x01 }
#define FLT_BND_DEF      { 500, 1500, 4000, 8000 }
#define FLT_ICF_MIN_DEF  1                    /* 范围下限（弱滤波端） */
#define FLT_ICF_MAX_DEF  15                   /* 范围上限（强滤波端） */

/*------------------------------------------ 测速滤波接口 -----------------------------*/

extern uint8_t  cfg_flt_ema0;                /* EMA 系数 ‰（10..500，默认 150=0.150） */
extern uint8_t  cfg_flt_dpos;                /* dpos 超限倍数 ×N_MIN（2..20，默认 6） */
extern uint8_t  cfg_flt_zero;                /* 静止判定周期数（5..60，默认 10） */
extern uint16_t cfg_flt_rate;                /* 转速变化率限幅 rpm/s（0=关，默认 4000） */
extern uint8_t  cfg_flt_icf[FLT_ICF_N];      /* IC 输入滤波动态表值（0..15, 全自动默认） */
extern uint16_t cfg_flt_bnd[FLT_ICF_N - 1];  /* 表边界 rpm（默认 500/1500/4000/8000） */
extern uint8_t  cfg_flt_icf_min;             /* 人工范围下限（钳制动态表值） */
extern uint8_t  cfg_flt_icf_max;             /* 人工范围上限 */

uint8_t  Config_GetFltEma(void);
uint8_t  Config_GetFltDpos(void);
uint8_t  Config_GetFltZero(void);
uint16_t Config_GetFltRate(void);
uint8_t  Config_GetFltIcf(uint8_t i);
uint16_t Config_GetFltBnd(uint8_t i);
uint8_t  Config_GetFltIcfMin(void);
uint8_t  Config_GetFltIcfMax(void);
/* 设置：返回 0=OK 1=范围非法（全部会校验并落 Flash 持久化） */
uint8_t  Config_SetFltEma(uint8_t v);
uint8_t  Config_SetFltDpos(uint8_t v);
uint8_t  Config_SetFltZero(uint8_t v);
uint8_t  Config_SetFltRate(uint16_t v);
uint8_t  Config_SetFltIcfTbl(uint8_t i, uint8_t v);   /* 改表项（含 min/max 钳位约束） */
uint8_t  Config_SetFltBnd(uint8_t i, uint16_t v);     /* 改边界（严格递增校验） */
uint8_t  Config_SetFltRange(uint8_t lo, uint8_t hi);  /* 人工范围（lo<=hi, 且含动态表值） */
void     Config_FltReset(void);                       /* 恢复全部动态滤波默认 */

/*------------------------------------------ 宏 ------------------------------------*/

#define CLAMP(x, lo, hi)   (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))

/*------------------------------------------ 接口 ------------------------------------*/

extern uint32_t cfg_baud;
extern uint8_t  cfg_gear_n;                          /* 当前档数 3/4/5 */
extern uint8_t  cfg_gear_div[CFG_GEAR_MAX];          /* div 表 */
extern uint32_t cfg_gear_bnd[CFG_GEAR_MAX - 1];      /* rpm 边界 */
extern uint32_t cfg_steps_per_rev;                   /* 实测一圈步数（Index→Index EMA） */
extern uint8_t  cfg_pol;                             /* A/B 方向极性（0=反向取反，1=正向） */

void    Config_Load(void);
void    Config_Save(void);
void    Config_Reset(void);
uint8_t Config_GetGearN(void);
uint8_t Config_GetGearDiv(uint8_t i);
uint32_t Config_GetGearBnd(uint8_t i);
uint8_t Config_CheckGear(uint8_t n, const uint8_t *div, const uint32_t *bnd);
void    Config_SetGear(uint8_t n, const uint8_t *div, const uint32_t *bnd);
void    Config_SetStepsPerRev(uint32_t steps);
void    Config_SetPol(uint8_t pol);

#endif /* __APP_CONFIG_H */