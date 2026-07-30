/************************************************
 * 手拧电机长采：30~100s @1kHz → PSRAM，供回放/自动测试
 ************************************************/
#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

bool capAlloc();
void capFree();
bool capStart(uint32_t duration_ms);
void capStop();
bool capIsRunning();
uint32_t capRemainMs();
size_t capCount();
size_t capCapacity();
uint32_t capDurationMs();
bool capTakeDoneEdge();
uint32_t capIsrHits();
uint32_t capRingDrop();
bool capIsDumping();

/** 在 1kHz 定时回调里调用：只写 DRAM 环 */
void capPushIsr(uint32_t t_ms, float rpm, int64_t counts, uint32_t index_n);
/** 主循环调用：把环刷进 PSRAM */
void capPoll();
/** 主循环调用：非阻塞分块 DUMP（需 CAPTURE NEXT 推进） */
void capDumpPoll();

/**
 * 启动串口导出（非阻塞）。stride=10 → 约 200Hz，默认稳妥；
 * stride=1 为全量（慢，必须 ACK）。
 * C,t_ms,rpm,dcounts,dindex
 */
void capDumpUsb(uint16_t stride = 10);
/** PC 收到 # CAP CHUNK … need=NEXT 后必须发 CAPTURE NEXT */
void capDumpNext();
