/**
  ******************************************************************************
  * @file    abi.h
  * @brief   AS5047P ABI 编码器捕获：TIM2 4X 计数 + EXTI4 Index + TIM5 UTO 事件流
  ******************************************************************************
  */
#ifndef __ABI_H
#define __ABI_H

#include <stdint.h>

#define STEPS_PER_REV  4000uL        /* 4X 标称 4000 步/圈，实测校准后更新 */
#define N_MIN          10            /* UTO 每周期目标步数 */

/* QUPR 按 60MHz 刻度换算（DSP 150MHz → ×60/150 = ×0.4） */
#define UTO_QUPR_MIN_60M   ((2500uL   * 60uL) / 150uL)  /* =1000   刻度 → ≈60kHz 夹顶 */
#define UTO_QUPR_MAX_60M   ((375000uL * 60uL) / 150uL)  /* =150000 刻度 → ≈400Hz 下限 */
#define UTO_HYST           125        /* 迟滞 12.5% (125/1000) */

/* 事件点 16B，与 BIN v2 帧字节兼容（PC 工具直接解析） */
typedef struct {
    uint32_t t_us;   /* 事件时刻 µs（g_tick64/60 低 32 位） */
    uint32_t c_lo;   /* counts 低 32 位 */
    uint32_t c_hi;   /* 保留（BIN v2 用 0） */
    uint32_t idx;    /* Index 圈数 */
} SnapPt;

void  Abi_Init(void);
void  Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now, uint32_t sub_us);  /* 记录回调 + sub-µs 余数 */
void  Snap_OnIndex(void);                                         /* EXTI4 ISR 调用 */
int32_t  Abi_GetRpm(void);
uint32_t Abi_GetIndexCnt(void);
uint32_t Abi_GetUsNow(void);   /* 64 位 µs 计数的低 32 位 */
uint8_t  Abi_GetDiv(void);
uint32_t Abi_GetCnt(void);     /* 当前 counts（调试用） */
uint32_t Abi_GetStepsPerRev(void); /* 测速用一圈步数（cfg_steps_per_rev，默认 4000） */
uint32_t Abi_GetCalib(void);       /* 当前 Index→Index EMA 步数（0=尚无首圈差分） */
void Abi_SetCalibSteps(uint32_t steps); /* CAL SET：写入 EMA 基准并持久化 */

/* Task 8 动态输入滤波（TIM2 IC1F/IC2F，写 CCMR1 正确位：IC1F bit7:4, IC2F bit15:12） */
void Abi_SetInputFilter(uint8_t val);     /* 0..15，直接改写寄存器（不停 TIM2） */
uint8_t Abi_GetInputFilter(void);         /* 当前生效 IC 滤波值 */

#endif /* __ABI_H */
