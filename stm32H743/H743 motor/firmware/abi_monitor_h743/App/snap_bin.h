#ifndef __SNAP_BIN_H
#define __SNAP_BIN_H

#include <stdint.h>
#include "abi.h"

/* BIN v2 帧头 + 16B 事件点，与 PC 工具字节级兼容 */
#define SNAP_MAGIC_V2   0xAB1C0002uL
#define SNAP_BACKTRACK  400uL          /* 回溯点数 */
#define SNAP_DUR_US     800000uL       /* 触发后记录 0.8s */

/* 环形缓存 2^10（避免数组超 RAM）：回溯 400 点 + 余量 */
#define RING_CAP_BITS   10uL
#define RING_CAP        (1uL << RING_CAP_BITS)

/* 事件点缓冲：512KB AXI SRAM 上限，ring 16KB + bss 余量 → 31000 点 */
#define SNAP_CAP        31000uL

#define TRIG_RPM        10             /* |rpm|>10 触发（自动：PWM 拉转即触发） */

void  Snap_Init(void);
void  Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now);
void  Snap_OnIndex(void);
void  Snap_Arm(void);
void  Snap_Disarm(void);
uint8_t  Snap_IsReady(void);
uint32_t Snap_Count(void);
uint32_t Snap_BuildBin(uint8_t *buf, uint32_t cap);
uint32_t Snap_State(void);

#endif /* __SNAP_BIN_H */