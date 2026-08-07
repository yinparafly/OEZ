#ifndef __SNAP_BIN_H
#define __SNAP_BIN_H

#include <stdint.h>
#include "abi.h"

/* BIN v2 帧头 + 16B 事件点，与 PC 工具字节级兼容 */
#define SNAP_MAGIC_V2   0xAB1C0002uL
#define SNAP_BACKTRACK  100uL          /* 触发回溯点数（实测触发前点基本为 0，100 足够） */
#define SNAP_DUR_US     500000uL       /* 触发后记录 0.5s（用户定：减小窗口+SD实时写可行） */

/* 环形缓存 2^10（避免数组超 RAM）：回溯 100 点 + 余量 */
#define RING_CAP_BITS   10uL
#define RING_CAP        (1uL << RING_CAP_BITS)

/* 事件点缓冲：512KB AXI SRAM 上限，ring 16KB + bss 余量 → 31000 点 */
#define SNAP_CAP        31000uL

#define TRIG_RPM        10             /* |rpm|>10 触发（自动：PWM 拉转即触发） */

/* BIN v2 帧 hz 元数据（PC 工具以 t_us 为准换算，此处兼容占位） */
#define SNAP_HZ         1000u

void  Snap_Init(void);
void  Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now, uint32_t sub_us);
void  Snap_OnIndex(void);
void  Snap_Arm(void);
void  Snap_Disarm(void);
uint8_t  Snap_IsReady(void);
uint32_t Snap_Count(void);
uint32_t Snap_BuildBin(uint8_t *buf, uint32_t cap);
uint32_t Snap_State(void);
const SnapPt *Snap_Points(void);   /* 当前 snap 点阵（Task 4 SD 备份用） */
uint32_t Snap_Crc32(void);         /* 点阵 zlib crc32（PC 工具对齐） */

#endif /* __SNAP_BIN_H */