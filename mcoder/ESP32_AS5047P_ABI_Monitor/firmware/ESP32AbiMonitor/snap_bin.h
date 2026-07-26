/************************************************
 * 短时记录仪（采用方案）：
 *  - MONITOR 武装：环缓临时存
 *  - |RPM|>10 且 I 过 1 圈 → 回溯 400 点 + 再记 2s → INTERNAL RAM
 *  - SNAP DONE → ALIVE → 自动 SD SAVE
 *  - REC NOW：仍可强制直采（无门限）
 ************************************************/
#pragma once

#include <stddef.h>
#include <stdint.h>

bool snapAlloc();
void snapFree();

void snapSetStepsPerRev(int32_t steps);
int32_t snapStepsPerRev();
uint32_t snapFileMagic();
size_t snapPointSize();

/** 强制直采（无门限） */
bool snapRecStart(uint32_t duration_ms, uint32_t sample_hz);

/** 武装：开环缓；触发前持续写入 */
bool snapArmBegin();
void snapArmEnd();
bool snapIsArmed();

/**
 * 条件满足后：把环缓末尾 backtrack_n 点拷入主缓冲，再继续记 duration_ms
 * 返回 false=无数据/已在记
 */
bool snapTriggerFromRing(uint16_t backtrack_n, uint32_t duration_ms);

bool snapIsRecording();
uint16_t snapCount();
uint32_t snapHz();
uint16_t snapRingCount();

/** DUMP BIN / SD SAVE 是否已过 ALIVE */
bool snapDumpReady();

/** 武装环缓 或 正式记录 时由 2kHz 调用 */
bool snapOnSampleCounts(int64_t counts, uint32_t index_n, uint32_t now_us);

void snapPollDone();
bool snapTakeArchiveEdge();

bool snapDumpBinary();
bool snapDumpHex();
void snapPrintStatus();
/** 填充一行 # DIAG …（含 snap/ring），便于 BLE */
void snapDiagLine(char* out, size_t out_sz);

const uint8_t* snapDataBytes();
size_t snapDataLen();

uint32_t snapCrc32(const uint8_t* data, size_t len);
