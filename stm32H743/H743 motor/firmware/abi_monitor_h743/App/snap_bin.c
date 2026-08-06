/**
  ******************************************************************************
  * @file    snap_bin.c
  * @brief   Task 3 事件记录管线：环形缓存回溯 + 触发状态机 + 0.8s 记录 BIN v2
  *
  * 自动触发（替代手动拧电机验证）：
  *   ARM 后，UTO ISR 每周期查 |rpm|>TRIG_RPM 置 armed_moving；
  *   下个 Index（转过一整圈）回溯 400 点 + 记 0.8s → DONE。
  ******************************************************************************
  */
#include <string.h>
#include "snap_bin.h"

/* 环形缓存：head/tail 单调递增 + 掩码取模（2 的幂，ISR 60kHz 无除法） */
static SnapPt ring[RING_CAP];
static uint32_t ring_head;   /* 下一写入位置（单调） */
static uint32_t ring_tail;   /* 最早有效点 */

static SnapPt snap[SNAP_CAP];
static uint32_t snap_count;

static volatile uint8_t state;          /* 0=IDLE 1=ARM 2=REC 3=DONE */
static volatile uint8_t armed_moving;   /* ARM 下转速已过阈值 */
static uint32_t rec_start_us;

void Snap_Init(void)
{
    ring_head = 0; ring_tail = 0; snap_count = 0;
    state = 0; armed_moving = 0;
}

void Snap_Arm(void)
{
    snap_count = 0; state = 1; armed_moving = 0;
    ring_head = 0; ring_tail = 0;
}

void Snap_Disarm(void)
{
    state = 0; armed_moving = 0;
}

void Snap_OnEvent(uint32_t cnt, uint32_t idx, uint32_t us_now)
{
    SnapPt p;
    p.t_us = us_now;
    p.c_lo = cnt;
    p.c_hi = 0;
    p.idx  = idx;

    /* 自动触发判定：ARM 且 |rpm| > 阈值 → 等下一 Index */
    int32_t rpm = Abi_GetRpm();
    if (state == 1) {
        if (rpm > TRIG_RPM || rpm < -TRIG_RPM) armed_moving = 1;
    }

    if (state == 2) {                       /* REC：填 snap */
        if (snap_count < SNAP_CAP) snap[snap_count++] = p;
        if (snap_count >= SNAP_CAP || (uint32_t)(us_now - rec_start_us) >= SNAP_DUR_US)
            state = 3;                      /* DONE */
    } else {                                /* IDLE/ARM：始终进 ring 供回溯 */
        ring[ring_head & (RING_CAP - 1)] = p;
        ring_head++;
        if (ring_head - ring_tail > RING_CAP) ring_tail = ring_head - RING_CAP;
    }
}

void Snap_OnIndex(void)
{
    if (state == 1 && armed_moving) {       /* 转速过阈值 + 转过整圈 → 触发 */
        uint32_t avail = ring_head - ring_tail;
        uint32_t take  = (avail < SNAP_BACKTRACK) ? avail : SNAP_BACKTRACK;
        uint32_t start = ring_head - take;
        for (uint32_t i = 0; i < take; i++)
            snap[snap_count++] = ring[(start + i) & (RING_CAP - 1)];
        rec_start_us = Abi_GetUsNow();
        state = 2;
    }
}

uint8_t Snap_IsReady(void)   { return (state == 3 && snap_count > 0) ? 1 : 0; }
uint32_t Snap_Count(void)    { return snap_count; }
uint32_t Snap_State(void)    { return (uint32_t)state; }

uint32_t Snap_BuildBin(uint8_t *buf, uint32_t cap)
{
    uint32_t need = 8 + snap_count * 16uL;
    if (cap < need || snap_count == 0) return 0;
    uint32_t magic = SNAP_MAGIC_V2, n = snap_count;
    memcpy(buf, &magic, 4);
    memcpy(buf + 4, &n, 4);
    memcpy(buf + 8, snap, snap_count * 16uL);
    return need;
}